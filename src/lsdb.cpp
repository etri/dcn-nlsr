/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2014-2021,  The University of Memphis,
 *                           Regents of the University of California,
 *                           Arizona Board of Regents.
 *
 * This file is part of NLSR (Named-data Link State Routing).
 * See AUTHORS.md for complete list of NLSR authors and contributors.
 *
 * NLSR is free software: you can redistribute it and/or modify it under the terms
 * of the GNU General Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later version.
 *
 * NLSR is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * NLSR, e.g., in COPYING.md file.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "lsdb.hpp"

#include "logger.hpp"
#include "nlsr.hpp"
#include "tlv-nlsr.hpp"
#include "utility/name-helper.hpp"
#include <ndn-cxx/mgmt/nfd/status-dataset.hpp>
#include <ndn-cxx/mgmt/nfd/command-options.hpp>
#include <ndn-cxx/mgmt/nfd/rib-entry.hpp>
#include <ndn-cxx/mgmt/nfd/controller.hpp>
#include <ndn-cxx/encoding/nfd-constants.hpp>
#include <ndn-cxx/mgmt/control-response.hpp>

namespace nlsr {

INIT_LOGGER(Lsdb);

const ndn::time::steady_clock::TimePoint Lsdb::DEFAULT_LSA_RETRIEVAL_DEADLINE =
  ndn::time::steady_clock::TimePoint::min();

Lsdb::Lsdb(ndn::Face& face, ndn::KeyChain& keyChain, ConfParameter& confParam,
           NamePrefixTable& namePrefixTable, RoutingTable& routingTable, ndn::nfd::Controller &controller)
  : m_face(face)
  , m_scheduler(face.getIoService())
  , m_confParam(confParam)
  , m_namePrefixTable(namePrefixTable)
  , m_routingTable(routingTable)
  , m_sync(m_face,
           [this] (const ndn::Name& routerName, const Lsa::Type& lsaType,
                   const uint64_t& sequenceNumber, const uint64_t area) {
             return isLsaNew(routerName, lsaType, sequenceNumber, area);
           }, m_confParam)
  , m_lsaRefreshTime(ndn::time::seconds(m_confParam.getLsaRefreshTime()))
  , m_adjLsaBuildInterval(m_confParam.getAdjLsaBuildInterval())
  , m_thisRouterPrefix(m_confParam.getRouterPrefix())
  , m_sequencingManager(m_confParam.getStateFileDir(), m_confParam.getHyperbolicState())
  , m_onNewLsaConnection(m_sync.onNewLsa->connect(
      [this] (const ndn::Name& updateName, uint64_t sequenceNumber,
              const ndn::Name& originRouter, uint64_t area) {
        ndn::Name lsaInterest{updateName};
        lsaInterest.appendNumber(sequenceNumber);
        expressInterest(area, lsaInterest, 0);
      }))
  , m_segmentPublisher(m_face, keyChain)
  //, m_isBuildAdjLsaSheduled(false)
  //, m_adjBuildCount(0)
  , m_controller(controller)
  ,m_keyChain(keyChain)
{
	for(auto &adj : m_confParam.getAdjacencyList()){
		m_sequencingManager.initiateSeqNoFromFile(adj.getAreaId());
	}
}

void
Lsdb::updateForRedistributeLsa(bool appNamePrefix, uint64_t& areaId, uint64_t&origin, NamePrefixList &npl)
{
    if(appNamePrefix){
        ExternalNameLsa exNameLsa(m_thisRouterPrefix, m_sequencingManager.getExternalNameLsaSeq(areaId) + 1,
                getLsaExpirationTimePoint(), npl, areaId);
        m_sequencingManager.increaseExternalNameLsaSeq(areaId);
        m_sequencingManager.writeSeqNoToFile(areaId);

        m_sync.publishRoutingUpdate(areaId, Lsa::Type::EXTERNALNAME, m_sequencingManager.getExternalNameLsaSeq(areaId));

        installLsa(std::make_shared<ExternalNameLsa>(exNameLsa));

        for( auto area : m_confParam.getAreaIdList() ){
            if( areaId !=  area){
                m_sync.publishRoutingUpdate(area, Lsa::Type::EXTERNALNAME,  
                        m_sequencingManager.getExternalNameLsaSeq(area) + 1 );

                m_sequencingManager.increaseExternalNameLsaSeq(area);
                m_sequencingManager.writeSeqNoToFile(area);
            }
        }
    }else{
        NetExternalNameLsa netExNameLsa(m_thisRouterPrefix, m_sequencingManager.getNetExternalNameLsaSeq(areaId) + 1,
                getLsaExpirationTimePoint(), npl, areaId);
        m_sequencingManager.increaseNetExternalNameLsaSeq(areaId);
        m_sequencingManager.writeSeqNoToFile(areaId);

        m_sync.publishRoutingUpdate(areaId, Lsa::Type::NET_EXTERNALNAME, 
            m_sequencingManager.getNetExternalNameLsaSeq(areaId));

        for( auto area : m_confParam.getAreaIdList() ){
            if( areaId !=  area){
                m_sync.publishRoutingUpdate(area, Lsa::Type::NET_EXTERNALNAME,  
                        m_sequencingManager.getNetExternalNameLsaSeq(area) + 1 );

                m_sequencingManager.increaseNetExternalNameLsaSeq(area);
                m_sequencingManager.writeSeqNoToFile(area);
            }
        }

        installLsa(std::make_shared<NetExternalNameLsa>(netExNameLsa));
    }
}

void
Lsdb::redistributeLsa(uint64_t areaId, uint64_t origin)
{

    if( m_confParam.getAreaIdList().find(areaId) == m_confParam.getAreaIdList().end() ){
        std::cout << "Warning... There is No AreaId(" << areaId << ")" << std::endl;
        return;
    }

    ndn::nfd::CommandOptions options;
    options.setTimeout(3000_ms);

    /*
R1(ARB)에서 ip route 10.0.0.0 255.255.0.0 172.11.0.5
R1# redistribute static metric 1 subnets

R3#show ip route 
Codes: C - connected, S - static, R - RIP, M - mobile, B - BGP
       D - EIGRP, EX - EIGRP external, O - OSPF, IA - OSPF inter area 
              N1 - OSPF NSSA external type 1, N2 - OSPF NSSA external type 2
                     E1 - OSPF external type 1, E2 - OSPF external type 2
                            i - IS-IS, su - IS-IS summary, L1 - IS-IS level-1, L2 - IS-IS level-2
                                   ia - IS-IS inter area, * - candidate default, U - per-user static route
                                          o - ODR, P - periodic downloaded static route

                                          Gateway of last resort is not set

                                          O IA 172.10.0.0/16 [110/30] via 172.13.0.2, 00:11:17, Ethernet1/0
                                          O IA 172.11.0.0/16 [110/30] via 172.13.0.2, 00:11:17, Ethernet1/0
                                          O    172.12.0.0/16 [110/20] via 172.13.0.2, 00:11:27, Ethernet1/0
                                          C    172.13.0.0/16 is directly connected, Ethernet1/0
                                          O IA 172.14.0.0/16 [110/40] via 172.13.0.2, 00:11:17, Ethernet1/0
                                          O IA 172.15.0.0/16 [110/40] via 172.13.0.2, 00:11:17, Ethernet1/0
                                               10.0.0.0/16 is subnetted, 1 subnets
                                          O E2    10.0.0.0 [110/1] via 172.13.0.2, 00:00:00, Ethernet1/0
                                           R3#

    */

    m_controller.fetch<ndn::nfd::RibDataset>(
            [this, areaId, origin] (const std::vector<ndn::nfd::RibEntry>& dataset) {
                NamePrefixList appNpl;
                NamePrefixList netNpl;
            for (const ndn::nfd::RibEntry& entry : dataset) {
            for (const ndn::nfd::Route& route : entry.getRoutes()) {
            //if( route.getOrigin() == ndn::nfd::ROUTE_ORIGIN_STATIC ){
            if( route.getOrigin() == origin ){
            if( (route.getFlags() & ndn::nfd::ROUTE_FLAG_NET_NAME) )
                netNpl.insert( entry.getName() );
            else
                appNpl.insert( entry.getName() );
            }

            }   
            }   
            uint64_t area = areaId;
            uint64_t ori = origin;
            if(appNpl.size()>0)
            updateForRedistributeLsa(true, area, ori, appNpl);
            if(netNpl.size()>0)
                updateForRedistributeLsa(false,area, ori, netNpl);
            },  
            [](uint32_t code, const std::string& reason){
            }
            ,
            options
   );

}

void
Lsdb::buildAndInstallOwnNameLsa(uint64_t areaId)
{
    NameLsa nameLsa(m_thisRouterPrefix, m_sequencingManager.getNameLsaSeq(areaId) + 1,
            getLsaExpirationTimePoint(), m_confParam.getNamePrefixList(areaId), areaId);

    m_sequencingManager.increaseNameLsaSeq(areaId);
    m_sequencingManager.writeSeqNoToFile(areaId);
    m_sync.publishRoutingUpdate(areaId, Lsa::Type::NAME, m_sequencingManager.getNameLsaSeq(areaId));

    installLsa(std::make_shared<NameLsa>(nameLsa));

    if( m_confParam.isABR() ){
        for( auto area : m_confParam.getAreaIdList() ){
            if( areaId !=  area){
                //m_sync.addUserNode(area, Lsa::Type::INTERNAME);
                m_sync.publishRoutingUpdate(area, Lsa::Type::INTERNAME,  
                        m_sequencingManager.getInterNameLsaSeq(area) + 1 );
                m_sequencingManager.increaseInterNameLsaSeq(area);
                //std::cout << "AddUserNode(IA-NAME): ---> " << area << " With SeqNo: " <<
                    //m_sequencingManager.getInterNameLsaSeq(area) << std::endl;
                m_sequencingManager.writeSeqNoToFile(area);
            }
        }
    }
}

void
Lsdb::buildAndInstallOwnCoordinateLsa()
{
    uint64_t area = 9000;
    CoordinateLsa corLsa(m_thisRouterPrefix, m_sequencingManager.getCorLsaSeq(area) + 1,
            getLsaExpirationTimePoint(), m_confParam.getCorR(),
            m_confParam.getCorTheta(), area);
    m_sequencingManager.increaseCorLsaSeq(area);
    m_sequencingManager.writeSeqNoToFile(area);

    // Sync coordinate LSAs if using HR or HR dry run.
    if (m_confParam.getHyperbolicState() != HYPERBOLIC_STATE_OFF) {
        m_sync.publishRoutingUpdate(area, Lsa::Type::COORDINATE, m_sequencingManager.getCorLsaSeq(area));
    }

    installLsa(std::make_shared<CoordinateLsa>(corLsa));
}

void
Lsdb::scheduleAdjLsaBuild(uint64_t area)
{
  m_adjBuildCount[area]++;

  if (m_confParam.getHyperbolicState() == HYPERBOLIC_STATE_ON) {
    // Don't build adjacency LSAs in hyperbolic routing
    NLSR_LOG_DEBUG("Adjacency LSA not built. Currently in hyperbolic routing state.");
    return;
  }

  if (m_isBuildAdjLsaSheduled[area]) {
    NLSR_LOG_DEBUG("Rescheduling Adjacency LSA build in " << m_adjLsaBuildInterval);
  }
  else {
    NLSR_LOG_DEBUG("Scheduling Adjacency LSA build in " << m_adjLsaBuildInterval << " on Area:" << area);
    m_isBuildAdjLsaSheduled[area] = true;
  }
  m_scheduledAdjLsaBuild[area] = m_scheduler.schedule(m_adjLsaBuildInterval, [area, this] { buildAdjLsa(area); });
}

template<typename T>
void
Lsdb::writeLog() const
{
  if ((T::type() == Lsa::Type::COORDINATE &&
      m_confParam.getHyperbolicState() == HYPERBOLIC_STATE_OFF) ||
      (T::type() == Lsa::Type::ADJACENCY &&
      m_confParam.getHyperbolicState() == HYPERBOLIC_STATE_ON)) {
    return;
  }

  NLSR_LOG_DEBUG("---------------" << T::type() << " LSDB-------------------");
  auto lsaRange = m_lsdb.get<byType>().equal_range(T::type());
  for (auto lsaIt = lsaRange.first; lsaIt != lsaRange.second; ++lsaIt) {
    auto lsaPtr = std::static_pointer_cast<T>(*lsaIt);
    NLSR_LOG_DEBUG(lsaPtr->toString());
  }
}

void
Lsdb::writeLog() const
{
    writeLog<CoordinateLsa>();
    writeLog<NameLsa>();
    writeLog<InterNameLsa>();
    writeLog<ExternalNameLsa>();
    writeLog<NetExternalNameLsa>();
    writeLog<AdjLsa>();
}

    void
Lsdb::processMonitoringInterest(const ndn::Name& name, const ndn::Interest& interest)
{

    std::cout << "Interest received for LSA: " << interest.getName().toUri() << std::endl;

    auto lsaRange = getLsdbIterator<NameLsa>();
    for (auto lsaIt = lsaRange.first; lsaIt != lsaRange.second; ++lsaIt) {
        NameLsa lsa((*lsaIt)->wireEncode());
        Origin& origin = m_origins.emplace(lsa.getOriginRouter().toUri(), Origin()).first->second;
        ptree nameLsa ;//= origin.nameLsa;
        nameLsa.put("Origin Router", lsa.getOriginRouter().toUri());
        nameLsa.put("Sequence Number", lsa.getSeqNo());
        auto duration = lsa.getExpirationTimePoint() - ndn::time::system_clock::now();
        nameLsa.put("Expires in", ndn::time::duration_cast<ndn::time::milliseconds>(duration));
        nameLsa.put("Area ID", lsa.getAreaId());

        ptree names;
        for(const auto&name: lsa.getNpl().getNames()){
            ptree nameLsa;
            nameLsa.put("name", name.toUri());
            names.push_back(ptree::value_type("", nameLsa));
        }   
        nameLsa.add_child("Names", names);
        origin.nameLsa.add_child("", nameLsa);
    }

    auto lsaRange2 = getLsdbIterator<InterNameLsa>();
    for (auto lsaIt = lsaRange2.first; lsaIt != lsaRange2.second; ++lsaIt) {
        InterNameLsa lsa((*lsaIt)->wireEncode());
        Origin& origin = m_origins.emplace(lsa.getOriginRouter().toUri(), Origin()).first->second;
        ptree nameLsa ;//= origin.nameLsa;
        nameLsa.put("Origin Router", lsa.getOriginRouter().toUri());
        nameLsa.put("Sequence Number", lsa.getSeqNo());
        auto duration = lsa.getExpirationTimePoint() - ndn::time::system_clock::now();
        nameLsa.put("Expires in", ndn::time::duration_cast<ndn::time::milliseconds>(duration));
        nameLsa.put("Area ID", lsa.getAreaId());

        ptree names;
        for(const auto&name: lsa.getNpl().getNames()){
            ptree nameLsa;
            nameLsa.put("name", name.toUri());
            names.push_back(ptree::value_type("", nameLsa));
        }   
        nameLsa.add_child("Names", names);
        origin.nameLsa.add_child("", nameLsa);
    }

    auto lsaRange3 = getLsdbIterator<ExternalNameLsa>();
    for (auto lsaIt = lsaRange3.first; lsaIt != lsaRange3.second; ++lsaIt) {
        ExternalNameLsa lsa((*lsaIt)->wireEncode());
        Origin& origin = m_origins.emplace(lsa.getOriginRouter().toUri(), Origin()).first->second;
        ptree nameLsa ;//= origin.nameLsa;
        nameLsa.put("Origin Router", lsa.getOriginRouter().toUri());
        nameLsa.put("Sequence Number", lsa.getSeqNo());
        auto duration = lsa.getExpirationTimePoint() - ndn::time::system_clock::now();
        nameLsa.put("Expires in", ndn::time::duration_cast<ndn::time::milliseconds>(duration));
        nameLsa.put("Area ID", lsa.getAreaId());

        ptree names;
        for(const auto&name: lsa.getNpl().getNames()){
            ptree nameLsa;
            nameLsa.put("name", name.toUri());
            names.push_back(ptree::value_type("", nameLsa));
        }   
        nameLsa.add_child("Names", names);
        origin.nameLsa.add_child("", nameLsa);
    }

    auto lsaRange4 = getLsdbIterator<NetExternalNameLsa>();
    for (auto lsaIt = lsaRange4.first; lsaIt != lsaRange4.second; ++lsaIt) {
        NetExternalNameLsa lsa((*lsaIt)->wireEncode());
        Origin& origin = m_origins.emplace(lsa.getOriginRouter().toUri(), Origin()).first->second;
        ptree nameLsa ;//= origin.nameLsa;
        nameLsa.put("Origin Router", lsa.getOriginRouter().toUri());
        nameLsa.put("Sequence Number", lsa.getSeqNo());
        auto duration = lsa.getExpirationTimePoint() - ndn::time::system_clock::now();
        nameLsa.put("Expires in", ndn::time::duration_cast<ndn::time::milliseconds>(duration));
        nameLsa.put("Area ID", lsa.getAreaId());

        ptree names;
        for(const auto&name: lsa.getNpl().getNames()){
            ptree nameLsa;
            nameLsa.put("name", name.toUri());
            names.push_back(ptree::value_type("", nameLsa));
        }   
        nameLsa.add_child("Names", names);
        origin.nameLsa.add_child("", nameLsa);
    }

    auto lsaRange1 = getLsdbIterator<AdjLsa>();
    for (auto lsaIt = lsaRange1.first; lsaIt != lsaRange1.second; ++lsaIt) {
        AdjLsa lsa((*lsaIt)->wireEncode());
        Origin& origin = m_origins.emplace(lsa.getOriginRouter().toUri(),
                Origin()).first->second;
        ptree adjLsa ;
        //origin.nameLsa;
        adjLsa.put("Origin Router", lsa.getOriginRouter().toUri());
        adjLsa.put("Sequence Number", lsa.getSeqNo());
        auto duration = lsa.getExpirationTimePoint() - ndn::time::system_clock::now();
        adjLsa.put("Expires in", ndn::time::duration_cast<ndn::time::milliseconds>(duration));
        adjLsa.put("Area ID", lsa.getAreaId());

        //NLSR_LOG_INFO("Adj LSA000: " << lsa.getOriginRouter()); 
        ptree adjs;
        for(const auto&adj: lsa.getAdl().getAdjList()){
            //NLSR_LOG_INFO("Adj LSA: " << adj);
            ptree adjLsa;
            adjLsa.put("name", adj.getName().toUri());
            adjLsa.put("uri", adj.getFaceUri());
            adjLsa.put("cost", adj.getLinkCost());
            adjs.push_back(ptree::value_type("", adjLsa));
        }   
        adjLsa.add_child("Adjacents", adjs);
        origin.adjLsa.add_child("", adjLsa);
    }

    ptree originrouters;
    NLSR_LOG_INFO("OriginRouters: " << m_origins.size());
    ptree& ch = originrouters.add_child("OriginRouters", ptree{});
    ptree origins;
    for(const auto &item:m_origins){
        ptree router;
        router.put("OriginRouter", item.first);
        router.push_back(std::make_pair("ADJACENCY LSA", item.second.adjLsa));
        router.push_back(std::make_pair("NAME LSA", item.second.nameLsa));
        router.push_back(std::make_pair("INTER-NAME LSA", item.second.inter_nameLsa));
        router.push_back(std::make_pair("EXTERNL-NAME LSA", item.second.external_nameLsa));
        router.push_back(std::make_pair("NET-EXTERNL-NAME LSA", item.second.net_external_nameLsa));
        ch.add_child("", router);
    }

    boost::property_tree::write_json("/tmp/lsa.json", originrouters);
    m_origins.clear();

    std::ostringstream buf;
    write_json (buf, originrouters, false);
    std::string json = buf.str();
    ndn::Data data(interest.getName());
    data.setFreshnessPeriod(1_ms);
    data.setContent( reinterpret_cast<const uint8_t*>(json.data()), json.length());
    m_keyChain.sign(data, ndn::signingWithSha256());
    m_face.put(data);
}

void
Lsdb::processInterest(const ndn::Name& name, const ndn::Interest& interest)
{
    ndn::Name interestName(interest.getName());
    NLSR_LOG_DEBUG("Interest received for LSA: " << interestName);
    std::cout << "Interest received for LSA: " << interestName << std::endl;

    if (interestName[-2].isVersion()) {
        // Interest for particular segment
        if (m_segmentPublisher.replyFromStore(interestName)) {
            NLSR_LOG_TRACE("Reply from SegmentPublisher storage");
            return;
        }

        // Remove version and segment
        // Remove version , segment and param
        interestName = interestName.getSubName(0, interestName.size() - 2);
        NLSR_LOG_TRACE("Interest w/o segment and version: " << interestName);
    }

    // increment RCV_LSA_INTEREST
    lsaIncrementSignal(Statistics::PacketType::RCV_LSA_INTEREST);

    std::string chkString("LSA");
    int32_t lsaPosition = util::getNameComponentPosition(interestName, chkString);

    std::string areaIdComponent;
    std::istringstream(interestName.get(lsaPosition-1).toUri()) >> areaIdComponent;
    int solicitedArea=-1;
    sscanf(areaIdComponent.c_str(), "area-%d", &solicitedArea);


    // Forms the name of the router that the Interest packet came from.
    ndn::Name originRouter = m_confParam.getNetwork();
    originRouter.append(interestName.getSubName(lsaPosition + 1,
                interestName.size() - lsaPosition - 3)); // added <areaId>/LSA

    // if the interest is for this router's LSA
    if (originRouter == m_thisRouterPrefix && lsaPosition >= 0) {
        uint64_t seqNo = interestName[-1].toNumber();   // params-sha256이 존재 함.
        NLSR_LOG_DEBUG("LSA sequence number from interest: " << seqNo);

        std::string lsaType = interestName[-2].toUri();
        Lsa::Type interestedLsType;
        std::istringstream(lsaType) >> interestedLsType;
        if (interestedLsType == Lsa::Type::BASE) {
            NLSR_LOG_WARN("Received unrecognized LSA type: " << lsaType);
            return;
        }

        incrementInterestRcvdStats(interestedLsType);
        if (processInterestForLsa(interest, originRouter, interestedLsType, seqNo, solicitedArea)) {
            lsaIncrementSignal(Statistics::PacketType::SENT_LSA_DATA);
        }

    }else if (auto lsaSegment = m_lsaStorage.find(interest)) {
        // else the interest is for other router's LSA, serve signed data from LsaSegmentStorage
        NLSR_LOG_TRACE("Found data in lsa storage. Sending the data for " << interest.getName());
        std::cout << "Found data in lsa storage. Sending the data for " << interest.getName() << std::endl;
        m_face.put(*lsaSegment);
    }
}

bool
Lsdb::processInterestForLsa(const ndn::Interest& interest, const ndn::Name& originRouter,
        Lsa::Type lsaType, uint64_t seqNo, uint64_t myArea)
{
    NLSR_LOG_DEBUG(interest << " received for " << lsaType << " on Area " << myArea);

    if( lsaType == Lsa::Type::EXTERNALNAME ){
        NamePrefixList allNpl;

        for (auto lsaIt = m_lsdb.begin(); lsaIt != m_lsdb.end(); ++lsaIt) {
            auto lsaPtr = std::static_pointer_cast<Lsa>(*lsaIt);

            if( lsaPtr->getType() == Lsa::Type::BASE 
                or lsaPtr->getType() == Lsa::Type::ADJACENCY 
                or lsaPtr->getType() == Lsa::Type::COORDINATE 
                )
                continue;

            if(lsaPtr->getType() == Lsa::Type::EXTERNALNAME ){
                auto externalNameLsa = std::static_pointer_cast<ExternalNameLsa>(*lsaIt);
                for(auto name: externalNameLsa->getNpl().getNames())
                    allNpl.insert(name);
            }
        }

        ExternalNameLsa nameLsa( m_thisRouterPrefix, seqNo,
            getLsaExpirationTimePoint(), allNpl, myArea);

        m_segmentPublisher.publish(interest, m_thisRouterPrefix, interest.getName(),
            nameLsa.wireEncode(), m_lsaRefreshTime, m_confParam.getSigningInfo());

        incrementDataSentStats(lsaType);
        return true;
    }

    if( lsaType == Lsa::Type::NET_EXTERNALNAME ){
        NamePrefixList allNpl;

        for (auto lsaIt = m_lsdb.begin(); lsaIt != m_lsdb.end(); ++lsaIt) {
            auto lsaPtr = std::static_pointer_cast<Lsa>(*lsaIt);

            if( lsaPtr->getType() == Lsa::Type::BASE 
                or lsaPtr->getType() == Lsa::Type::ADJACENCY 
                or lsaPtr->getType() == Lsa::Type::COORDINATE 
                )
                continue;
            if(lsaPtr->getType() == Lsa::Type::NET_EXTERNALNAME ){
                auto interNameLsa = std::static_pointer_cast<NetExternalNameLsa>(*lsaIt);
                for(auto name: interNameLsa->getNpl().getNames())
                    allNpl.insert(name);
            }
        }

        NetExternalNameLsa nameLsa( m_thisRouterPrefix, seqNo,
            getLsaExpirationTimePoint(), allNpl, myArea);

        m_segmentPublisher.publish(interest, m_thisRouterPrefix, interest.getName(),
            nameLsa.wireEncode(), m_lsaRefreshTime, m_confParam.getSigningInfo());

        incrementDataSentStats(lsaType);
        return true;
    }

    if( lsaType == Lsa::Type::INTERNAME ){
        // https://etrioss.kr/hii/dcn/dcn-nlsr/-/issues/23
        NamePrefixList allNpl;

        for (auto lsaIt = m_lsdb.begin(); lsaIt != m_lsdb.end(); ++lsaIt) {
            auto lsaPtr = std::static_pointer_cast<Lsa>(*lsaIt);
            if( myArea == lsaPtr->getAreaId())
                continue;

            if( lsaPtr->getType() == Lsa::Type::BASE 
                or lsaPtr->getType() == Lsa::Type::ADJACENCY 
                or lsaPtr->getType() == Lsa::Type::COORDINATE 
                )
                continue;
            if( lsaPtr->getType() == Lsa::Type::NAME ){
                auto nameLsa = std::static_pointer_cast<NameLsa>(*lsaIt);
                for(auto name: nameLsa->getNpl().getNames())
                    allNpl.insert(name);
            }
            if(lsaPtr->getType() == Lsa::Type::INTERNAME ){
                auto interNameLsa = std::static_pointer_cast<InterNameLsa>(*lsaIt);
                for(auto name: interNameLsa->getNpl().getNames())
                    allNpl.insert(name);
            }
        }

        NameLsa nameLsa( m_thisRouterPrefix, seqNo,
            getLsaExpirationTimePoint(), allNpl, myArea);

        m_segmentPublisher.publish(interest, m_thisRouterPrefix, interest.getName(),
            nameLsa.wireEncode(), m_lsaRefreshTime, m_confParam.getSigningInfo());

        incrementDataSentStats(lsaType);
        return true;
    }

    if (auto lsaPtr = findLsa(originRouter, lsaType, myArea)) {
        NLSR_LOG_TRACE("Verifying SeqNo for " << lsaType << " is same as requested.");
        if (lsaPtr->getSeqNo() == seqNo) {
            m_segmentPublisher.publish(interest, interest.getName(), interest.getName(),
                lsaPtr->wireEncode(), m_lsaRefreshTime, m_confParam.getSigningInfo());
            incrementDataSentStats(lsaType);
            return true;
        }
    }
    else {
        NLSR_LOG_TRACE(interest << "  was not found in our LSDB");
    }
    return false;
}

void
Lsdb::installLsa(std::shared_ptr<Lsa> lsa)
{
    NLSR_LOG_DEBUG("InstallLsa called");
    auto timeToExpire = m_lsaRefreshTime;

    uint64_t myAreaId = lsa->getAreaId();

    auto chkLsa = findLsa(lsa->getOriginRouter(), lsa->getType(), myAreaId);

    if (chkLsa == nullptr) {
        NLSR_LOG_DEBUG("Adding " << lsa->getType() << " LSA");
        ndn::time::seconds timeToExpire = m_lsaRefreshTime;

        m_lsdb.emplace(lsa);

        // Add any new name prefixes to the NPT if from another router
        if (lsa->getOriginRouter() != m_thisRouterPrefix) {
            // Pass the origin router as both the name to register and where it came from.
            m_namePrefixTable.addEntry(lsa->getOriginRouter(), lsa->getOriginRouter(), lsa->getType(), myAreaId);

            if (lsa->getType() == Lsa::Type::NET_EXTERNALNAME) {
                auto nlsa = std::static_pointer_cast<NetExternalNameLsa>(lsa);
                for (const auto& name : nlsa->getNpl().getNames()) {
                    if (name != m_thisRouterPrefix) {
                        m_namePrefixTable.addEntry(name, nlsa->getOriginRouter(), nlsa->getType(), myAreaId);
                    }
                }
            }
            if (lsa->getType() == Lsa::Type::EXTERNALNAME) {
                auto nlsa = std::static_pointer_cast<ExternalNameLsa>(lsa);
                for (const auto& name : nlsa->getNpl().getNames()) {
                    if (name != m_thisRouterPrefix) {
                        m_namePrefixTable.addEntry(name, nlsa->getOriginRouter(), nlsa->getType(), myAreaId);
                    }
                }
            }
            //>modori on 20210615
            if (lsa->getType() == Lsa::Type::INTERNAME) {
                auto nlsa = std::static_pointer_cast<InterNameLsa>(lsa);
                for (const auto& name : nlsa->getNpl().getNames()) {
                    if (name != m_thisRouterPrefix) {
                        m_namePrefixTable.addEntry(name, nlsa->getOriginRouter(), nlsa->getType(), myAreaId);
                    }
                }
            }

            if (lsa->getType() == Lsa::Type::NAME) {
                auto nlsa = std::static_pointer_cast<NameLsa>(lsa);
                for (const auto& name : nlsa->getNpl().getNames()) {
                    if (name != m_thisRouterPrefix) {
                        m_namePrefixTable.addEntry(name, nlsa->getOriginRouter(), nlsa->getType(), myAreaId);
                    }
                }
            }

            auto duration = lsa->getExpirationTimePoint() - ndn::time::system_clock::now();
            if (duration > ndn::time::seconds(0)) {
                timeToExpire = ndn::time::duration_cast<ndn::time::seconds>(duration);
            }
        }

        if ((lsa->getType() == Lsa::Type::ADJACENCY && m_confParam.getHyperbolicState() != HYPERBOLIC_STATE_ON)||
                (lsa->getType() == Lsa::Type::COORDINATE && m_confParam.getHyperbolicState() != HYPERBOLIC_STATE_OFF)) {
            m_routingTable.scheduleRoutingTableCalculation(myAreaId);
        }

        lsa->setExpiringEventId(scheduleLsaExpiration(lsa, timeToExpire));

    }
    // Else this is a known name LSA, so we are updating it.
    else if (chkLsa->getSeqNo() < lsa->getSeqNo()) {
        NLSR_LOG_DEBUG("Updating " << lsa->getType() << " LSA:");
        std::cout << "Updating " << lsa->getType() << " LSA:" << std::endl;
        NLSR_LOG_DEBUG(chkLsa->toString());
        chkLsa->setSeqNo(lsa->getSeqNo());
        chkLsa->setExpirationTimePoint(lsa->getExpirationTimePoint());

        if (lsa->getType() == Lsa::Type::NAME ) {
            auto chkNameLsa = std::static_pointer_cast<NameLsa>(chkLsa);
            auto nlsa = std::static_pointer_cast<NameLsa>(lsa);
            chkNameLsa->getNpl().sort();
            nlsa->getNpl().sort();
            if (!chkNameLsa->isEqualContent(*nlsa)) {
                // Obtain the set difference of the current and the incoming
                // name prefix sets, and add those.
                std::list<ndn::Name> newNames = nlsa->getNpl().getNames();
                std::list<ndn::Name> oldNames = chkNameLsa->getNpl().getNames();
                std::list<ndn::Name> namesToAdd;
                std::set_difference(newNames.begin(), newNames.end(), oldNames.begin(), oldNames.end(),
                        std::inserter(namesToAdd, namesToAdd.begin()));
                for (const auto& name : namesToAdd) {

                    chkNameLsa->addName(name);

                    if (nlsa->getOriginRouter() != m_thisRouterPrefix && name != m_thisRouterPrefix) {
                        m_namePrefixTable.addEntry(name, nlsa->getOriginRouter(), chkNameLsa->getType(),
                        chkNameLsa->getAreaId());
                    }
                }

                chkNameLsa->getNpl().sort();

                // Also remove any names that are no longer being advertised.
                std::list<ndn::Name> namesToRemove;
                std::set_difference(oldNames.begin(), oldNames.end(), newNames.begin(), newNames.end(),
                        std::inserter(namesToRemove, namesToRemove.begin()));
                for (const auto& name : namesToRemove) {
                    NLSR_LOG_DEBUG("Removing name" << name << " from Name LSA no longer advertised.");
                    chkNameLsa->removeName(name);
                    if (nlsa->getOriginRouter() != m_thisRouterPrefix && name != m_thisRouterPrefix) {
                        m_namePrefixTable.removeEntry(name, nlsa->getOriginRouter(), chkNameLsa->getAreaId());
                    }
                }
            }
        }
        else if ( lsa->getType() == Lsa::Type::NET_EXTERNALNAME ) {
            auto chkNameLsa = std::static_pointer_cast<NetExternalNameLsa>(chkLsa);
            auto nlsa = std::static_pointer_cast<NetExternalNameLsa>(lsa);
            chkNameLsa->getNpl().sort();
            nlsa->getNpl().sort();
            if (!chkNameLsa->isEqualContent(*nlsa)) {
                // Obtain the set difference of the current and the incoming
                // name prefix sets, and add those.
                std::list<ndn::Name> newNames = nlsa->getNpl().getNames();
                std::list<ndn::Name> oldNames = chkNameLsa->getNpl().getNames();
                std::list<ndn::Name> namesToAdd;
                std::set_difference(newNames.begin(), newNames.end(), oldNames.begin(), oldNames.end(),
                        std::inserter(namesToAdd, namesToAdd.begin()));
                for (const auto& name : namesToAdd) {
                    chkNameLsa->addName(name);
                }

                chkNameLsa->getNpl().sort();

                // Also remove any names that are no longer being advertised.
                std::list<ndn::Name> namesToRemove;
                std::set_difference(oldNames.begin(), oldNames.end(), newNames.begin(), newNames.end(),
                        std::inserter(namesToRemove, namesToRemove.begin()));
                for (const auto& name : namesToRemove) {
                    NLSR_LOG_DEBUG("Removing name" << name << " from Name LSA no longer advertised.");
                    chkNameLsa->removeName(name);
                }
            }
        }
        else if ( lsa->getType() == Lsa::Type::EXTERNALNAME ) {
            auto chkNameLsa = std::static_pointer_cast<ExternalNameLsa>(chkLsa);
            auto nlsa = std::static_pointer_cast<ExternalNameLsa>(lsa);
            chkNameLsa->getNpl().sort();
            nlsa->getNpl().sort();
            if (!chkNameLsa->isEqualContent(*nlsa)) {
                // Obtain the set difference of the current and the incoming
                // name prefix sets, and add those.
                std::list<ndn::Name> newNames = nlsa->getNpl().getNames();
                std::list<ndn::Name> oldNames = chkNameLsa->getNpl().getNames();
                std::list<ndn::Name> namesToAdd;
                std::set_difference(newNames.begin(), newNames.end(), oldNames.begin(), oldNames.end(),
                        std::inserter(namesToAdd, namesToAdd.begin()));
                for (const auto& name : namesToAdd) {
                    chkNameLsa->addName(name);
                }

                chkNameLsa->getNpl().sort();

                // Also remove any names that are no longer being advertised.
                std::list<ndn::Name> namesToRemove;
                std::set_difference(oldNames.begin(), oldNames.end(), newNames.begin(), newNames.end(),
                        std::inserter(namesToRemove, namesToRemove.begin()));
                for (const auto& name : namesToRemove) {
                    NLSR_LOG_DEBUG("Removing name" << name << " from Name LSA no longer advertised.");
                    chkNameLsa->removeName(name);
                }
            }
        }
        else if ( lsa->getType() == Lsa::Type::INTERNAME ) {
            auto chkNameLsa = std::static_pointer_cast<InterNameLsa>(chkLsa);
            auto nlsa = std::static_pointer_cast<InterNameLsa>(lsa);
            chkNameLsa->getNpl().sort();
            nlsa->getNpl().sort();
            if (!chkNameLsa->isEqualContent(*nlsa)) {
                // Obtain the set difference of the current and the incoming
                // name prefix sets, and add those.
                std::list<ndn::Name> newNames = nlsa->getNpl().getNames();
                std::list<ndn::Name> oldNames = chkNameLsa->getNpl().getNames();
                std::list<ndn::Name> namesToAdd;
                std::set_difference(newNames.begin(), newNames.end(), oldNames.begin(), oldNames.end(),
                        std::inserter(namesToAdd, namesToAdd.begin()));
                for (const auto& name : namesToAdd) {

                    chkNameLsa->addName(name);

                    if (nlsa->getOriginRouter() != m_thisRouterPrefix && name != m_thisRouterPrefix) {
                        m_namePrefixTable.addEntry(name, nlsa->getOriginRouter(), chkNameLsa->getType(),
                        chkNameLsa->getAreaId());
                    }
                }

                chkNameLsa->getNpl().sort();

                // Also remove any names that are no longer being advertised.
                std::list<ndn::Name> namesToRemove;
                std::set_difference(oldNames.begin(), oldNames.end(), newNames.begin(), newNames.end(),
                        std::inserter(namesToRemove, namesToRemove.begin()));
                for (const auto& name : namesToRemove) {
                    NLSR_LOG_DEBUG("Removing name" << name << " from Name LSA no longer advertised.");
                    chkNameLsa->removeName(name);
                    if (nlsa->getOriginRouter() != m_thisRouterPrefix && name != m_thisRouterPrefix) {
                        m_namePrefixTable.removeEntry(name, nlsa->getOriginRouter(), chkNameLsa->getAreaId());
                    }
                }
            }
        }
        else if (lsa->getType() == Lsa::Type::ADJACENCY) {
            auto chkAdjLsa = std::static_pointer_cast<AdjLsa>(chkLsa);
            auto alsa = std::static_pointer_cast<AdjLsa>(lsa);
            //std::cout << alsa->toString() << std::endl;
            if (!chkAdjLsa->isEqualContent(*alsa)) {
                chkAdjLsa->resetAdl();
                for (const auto& adjacent : alsa->getAdl()) {
                    chkAdjLsa->addAdjacent(adjacent);
                }
                m_routingTable.scheduleRoutingTableCalculation(myAreaId);
            }
        }
        else {
            auto chkCorLsa = std::static_pointer_cast<CoordinateLsa>(chkLsa);
            auto clsa = std::static_pointer_cast<CoordinateLsa>(lsa);
            if (!chkCorLsa->isEqualContent(*clsa)) {
                chkCorLsa->setCorRadius(clsa->getCorRadius());
                chkCorLsa->setCorTheta(clsa->getCorTheta());
                if (m_confParam.getHyperbolicState() != HYPERBOLIC_STATE_OFF) {
                    m_routingTable.scheduleRoutingTableCalculation(myAreaId);
                }
            }
        }

        if (chkLsa->getOriginRouter() != m_thisRouterPrefix) {
            auto duration = lsa->getExpirationTimePoint() - ndn::time::system_clock::now();
            if (duration > ndn::time::seconds(0)) {
                timeToExpire = ndn::time::duration_cast<ndn::time::seconds>(duration);
            }
        }
        chkLsa->getExpiringEventId().cancel();
        chkLsa->setExpiringEventId(scheduleLsaExpiration(chkLsa, timeToExpire));
        NLSR_LOG_DEBUG("Updated " << lsa->getType() << " LSA:");
        NLSR_LOG_DEBUG(chkLsa->toString());

    }
}

bool
Lsdb::removeLsa(const ndn::Name& router, Lsa::Type lsaType, uint64_t area)
{
  auto lsaIt = m_lsdb.get<byName>().find(std::make_tuple(router, lsaType, area));

  if (lsaIt != m_lsdb.end()) {
    auto lsaPtr = *lsaIt;
    NLSR_LOG_DEBUG("Removing " << lsaType << " LSA:");
    NLSR_LOG_DEBUG(lsaPtr->toString());
    // If the requested name LSA is not ours, we also need to remove
    // its entries from the NPT.
    if (lsaPtr->getOriginRouter() != m_thisRouterPrefix) {
      m_namePrefixTable.removeEntry(lsaPtr->getOriginRouter(), lsaPtr->getOriginRouter(), area);

      if (lsaType == Lsa::Type::NAME) {
        auto nlsaPtr = std::static_pointer_cast<NameLsa>(lsaPtr);
        for (const auto& name : nlsaPtr->getNpl().getNames()) {
          if (name != m_thisRouterPrefix) {
            m_namePrefixTable.removeEntry(name, nlsaPtr->getOriginRouter(), area);
          }
        }
      }
    }
    m_lsdb.erase(lsaIt);
    return true;
  }
  return false;
}

void
Lsdb::buildAdjLsa(uint64_t area)
{
  NLSR_LOG_TRACE("buildAdjLsa called");

  m_isBuildAdjLsaSheduled[area] = false;

  if (m_confParam.getAdjacencyList().isAdjLsaBuildable(m_confParam.getInterestRetryNumber())) {

    int adjBuildCount = m_adjBuildCount[area];
    // Only do the adjLsa build if there's one scheduled
    if (adjBuildCount > 0) {
      // It only makes sense to do the adjLsa build if we have neighbors
      if (m_confParam.getAdjacencyList().getNumOfActiveNeighbor() > 0) {
        NLSR_LOG_DEBUG("Building and installing own Adj LSA");
        buildAndInstallOwnAdjLsa(area);
      }
      // We have no active neighbors, meaning no one can route through
      // us.  So delete our entry in the LSDB. This prevents this
      // router from refreshing the LSA, eventually causing other
      // routers to delete it, too.
      else {
        NLSR_LOG_DEBUG("Removing own Adj LSA; no ACTIVE neighbors");

        removeLsa(m_thisRouterPrefix, Lsa::Type::ADJACENCY, area);
        // Recompute routing table after removal
        m_routingTable.scheduleRoutingTableCalculation(area);
      }
      // In the case that during building the adj LSA, the FIB has to
      // wait on an Interest response, the number of scheduled adj LSA
      // builds could change, so we shouldn't just set it to 0.
      m_adjBuildCount[area] = m_adjBuildCount[area] - adjBuildCount;
    }
  }
  // We are still waiting to know the adjacency status of some
  // neighbor, so schedule a build for later (when all that has
  // hopefully finished)
  else {
    m_isBuildAdjLsaSheduled[area] = true;
    auto schedulingTime = ndn::time::seconds(m_confParam.getInterestRetryNumber() *
                                             m_confParam.getInterestResendTime());
    m_scheduledAdjLsaBuild[area] = m_scheduler.schedule(schedulingTime, [area, this] { buildAdjLsa(area); });
  }
}

void
Lsdb::buildAndInstallOwnAdjLsa(uint64_t area)
{
  NLSR_LOG_DEBUG("BuildAndInstallOwnAdjLsa called for area " << area);

  AdjLsa adjLsa(m_thisRouterPrefix, m_sequencingManager.getAdjLsaSeq(area) + 1,
                getLsaExpirationTimePoint(),
                m_confParam.getAdjacencyList().getNumOfActiveNeighbor(),
                m_confParam.getAdjacencyList(), area);
  m_sequencingManager.increaseAdjLsaSeq(area);
  m_sequencingManager.writeSeqNoToFile(area);

  //Sync adjacency LSAs if link-state or dry-run HR is enabled.
  if (m_confParam.getHyperbolicState() != HYPERBOLIC_STATE_ON) {
    m_sync.publishRoutingUpdate(area, Lsa::Type::ADJACENCY, m_sequencingManager.getAdjLsaSeq(area));
  }

  installLsa(std::make_shared<AdjLsa>(adjLsa));
}

void
Lsdb::expireOrRefreshLsa(std::shared_ptr<Lsa> lsa)
{
  NLSR_LOG_DEBUG("ExpireOrRefreshLsa called for " << lsa->getType());
  NLSR_LOG_DEBUG("OriginRouter: " << lsa->getOriginRouter() << " Seq No: " << lsa->getSeqNo());

  auto lsaIt = m_lsdb.get<byName>().find(std::make_tuple(lsa->getOriginRouter(), lsa->getType(), lsa->getAreaId()));

  // If this name LSA exists in the LSDB
  if (lsaIt != m_lsdb.end()) {
    auto lsaPtr = *lsaIt;
    NLSR_LOG_DEBUG(lsaPtr->toString());
    NLSR_LOG_DEBUG("LSA Exists with seq no: " << lsaPtr->getSeqNo());
    // If its seq no is the one we are expecting.
    if (lsaPtr->getSeqNo() == lsa->getSeqNo()) {
      if (lsaPtr->getOriginRouter() == m_thisRouterPrefix) {
        NLSR_LOG_DEBUG("Own " << lsaPtr->getType() << " LSA, so refreshing it.");
        NLSR_LOG_DEBUG("Current LSA:");
        NLSR_LOG_DEBUG(lsaPtr->toString());
        lsaPtr->setSeqNo(lsaPtr->getSeqNo() + 1);
        m_sequencingManager.setLsaSeq(lsaPtr->getAreaId(), lsaPtr->getSeqNo(), lsaPtr->getType());
        lsaPtr->setExpirationTimePoint(getLsaExpirationTimePoint());
        NLSR_LOG_DEBUG("Updated LSA:");
        NLSR_LOG_DEBUG(lsaPtr->toString());
        // schedule refreshing event again
        lsaPtr->setExpiringEventId(scheduleLsaExpiration(lsaPtr, m_lsaRefreshTime));
        m_sequencingManager.writeSeqNoToFile( lsaPtr->getAreaId() );
        m_sync.publishRoutingUpdate(lsaPtr->getAreaId(), lsaPtr->getType(), m_sequencingManager.getLsaSeq(lsaPtr->getAreaId(), lsaPtr->getType()));
      }
      // Since we cannot refresh other router's LSAs, our only choice is to expire.
      else {
        NLSR_LOG_DEBUG("Other's " << lsaPtr->getType() << " LSA, so removing from LSDB");
        removeLsa(lsaPtr->getOriginRouter(), lsaPtr->getType(), lsaPtr->getAreaId());
      }
    }
  }
}

void
Lsdb::expressInterest(uint64_t areaId, const ndn::Name& interestName, uint32_t timeoutCount,
                      ndn::time::steady_clock::TimePoint deadline)
{
  // increment SENT_LSA_INTEREST
  lsaIncrementSignal(Statistics::PacketType::SENT_LSA_INTEREST);

  if (deadline == DEFAULT_LSA_RETRIEVAL_DEADLINE) {
    deadline = ndn::time::steady_clock::now() + ndn::time::seconds(static_cast<int>(LSA_REFRESH_TIME_MAX));
  }
  // The first component of the interest is the name.
  ndn::Name lsaName = interestName.getSubName(0, interestName.size()-1);
  // The seq no is the last
  uint64_t seqNo = interestName[-1].toNumber();

  // If the LSA is not found in the list currently.
  if (m_highestSeqNo.find(lsaName.append(std::to_string(areaId)) ) == m_highestSeqNo.end()) {
    m_highestSeqNo[lsaName.append(std::to_string(areaId))] = seqNo;
  }
  // If the new seq no is higher, that means the LSA is valid
  else if (seqNo > m_highestSeqNo[lsaName.append(std::to_string(areaId))]) {
    m_highestSeqNo[lsaName.append(std::to_string(areaId))] = seqNo;
  }
  // Otherwise, its an old/invalid LSA
  else if (seqNo < m_highestSeqNo[lsaName.append(std::to_string(areaId)) ]) {
    return;
  }

  ndn::Interest interest(interestName);
  ndn::util::SegmentFetcher::Options options;
  options.interestLifetime = m_confParam.getLsaInterestLifetime();

  //interest.setApplicationParameters((uint8_t *)&areaId, sizeof(uint64_t));

  NLSR_LOG_DEBUG("Fetching Data for LSA: " << interestName << " Seq number: " << seqNo << " on Area: " << areaId);

  // modori on 20210615
      Lsa::Type interestedLsType;
      std::istringstream(interestName[-2].toUri()) >> interestedLsType;

  auto fetcher = ndn::util::SegmentFetcher::start(m_face, interest,
      m_confParam.getValidator(), options);

  auto it = m_fetchers.insert(fetcher).first;

  fetcher->afterSegmentValidated.connect([this] (const ndn::Data& data) {
    // Nlsr class subscribes to this to fetch certificates
    afterSegmentValidatedSignal(data);

    // If we don't do this IMS throws: std::bad_weak_ptr: bad_weak_ptr
    auto lsaSegment = std::make_shared<const ndn::Data>(data);
    m_lsaStorage.insert(*lsaSegment);
    const ndn::Name& segmentName = lsaSegment->getName();
    // Schedule deletion of the segment
    m_scheduler.schedule(ndn::time::seconds(LSA_REFRESH_TIME_DEFAULT),
                         [this, segmentName] { m_lsaStorage.erase(segmentName); });
  });

  fetcher->onComplete.connect([=] (const ndn::ConstBufferPtr& bufferPtr) {
    m_lsaStorage.erase(ndn::Name(lsaName).appendNumber(seqNo - 1));
    afterFetchLsa(bufferPtr, interestName, areaId);
    m_fetchers.erase(it);
  });

  fetcher->onError.connect([=] (uint32_t errorCode, const std::string& msg) {
    onFetchLsaError(areaId, errorCode, msg, interestName, timeoutCount, deadline, lsaName, seqNo);
    m_fetchers.erase(it);
  });

  Lsa::Type lsaType;
  std::istringstream(interestName[-2].toUri()) >> lsaType;
  incrementInterestSentStats(lsaType);
}

void
Lsdb::onFetchLsaError(uint64_t area, uint32_t errorCode, const std::string& msg, const ndn::Name& interestName,
                      uint32_t retransmitNo, const ndn::time::steady_clock::TimePoint& deadline,
                      ndn::Name lsaName, uint64_t seqNo)
{
  NLSR_LOG_DEBUG("Failed to fetch LSA: " << lsaName << ", Error code: " << errorCode
                 << ", Message: " << msg);

  if (ndn::time::steady_clock::now() < deadline) {
    auto it = m_highestSeqNo.find(lsaName.append(std::to_string(area)));
    if (it != m_highestSeqNo.end() && it->second == seqNo) {
      // If the SegmentFetcher failed due to an Interest timeout, it is safe to re-express
      // immediately since at the least the LSA Interest lifetime has elapsed.
      // Otherwise, it is necessary to delay the Interest re-expression to prevent
      // the potential for constant Interest flooding.
      ndn::time::seconds delay = m_confParam.getLsaInterestLifetime();

      if (errorCode == ndn::util::SegmentFetcher::ErrorCode::INTEREST_TIMEOUT) {
        delay = ndn::time::seconds(0);
      }
      m_scheduler.schedule(delay, std::bind(&Lsdb::expressInterest, this,
                                            area, interestName, retransmitNo + 1, deadline));
    }
  }
}

void
Lsdb::afterFetchLsa(const ndn::ConstBufferPtr& bufferPtr, const ndn::Name& interestName, uint64_t fromAreaId)
{
	NLSR_LOG_DEBUG("Received data for LSA interest: " << interestName);
	std::cout << "Received data for LSA interest: " << interestName << ", fromAreaId: " << fromAreaId << std::endl;
	lsaIncrementSignal(Statistics::PacketType::RCV_LSA_DATA);

	ndn::Name lsaName = interestName.getSubName(0, interestName.size()-1);
	uint64_t seqNo = interestName[-1].toNumber();

	if (m_highestSeqNo.find(lsaName.append(std::to_string(fromAreaId))) == m_highestSeqNo.end()) {
		m_highestSeqNo[lsaName.append(std::to_string(fromAreaId))] = seqNo;
	}
	else if (seqNo > m_highestSeqNo[lsaName.append(std::to_string(fromAreaId))]) {
		m_highestSeqNo[lsaName.append(std::to_string(fromAreaId))] = seqNo;
		NLSR_LOG_TRACE("SeqNo for LSA(name): " << interestName << "  updated");
	}
	else if (seqNo < m_highestSeqNo[lsaName.append(std::to_string(fromAreaId))]) {
		return;
	}

	std::string chkString("LSA");
	int32_t lsaPosition = util::getNameComponentPosition(interestName, chkString);

	if (lsaPosition >= 0) {
		// Extracts the prefix of the originating router from the data.
		ndn::Name originRouter = m_confParam.getNetwork();
		originRouter.append(interestName.getSubName(lsaPosition + 1,
            interestName.size() - lsaPosition - 3));

		try {
			Lsa::Type interestedLsType;
			std::istringstream(interestName[-2].toUri()) >> interestedLsType;

			if (interestedLsType == Lsa::Type::BASE) {
				NLSR_LOG_WARN("Received unrecognized LSA Type: " << interestName[-2].toUri());
				return;
			}

			ndn::Block block(bufferPtr);

            uint64_t myAreaId = std::numeric_limits<uint64_t>::max();
            if( !m_confParam.isABR() ){
                myAreaId = *m_confParam.getAreaIdList().begin();
            }else{
                for( auto & adj : m_confParam.getAdjacencyList() ){
                    if( !originRouter.compare( adj.getName() ) )
                        myAreaId = adj.getAreaId();
                }
            }

			if (interestedLsType == Lsa::Type::NAME) {
				lsaIncrementSignal(Statistics::PacketType::RCV_NAME_LSA_DATA);

				if( isLsaNew(originRouter, interestedLsType, seqNo, fromAreaId) ) {
                    // modori on 20210616
                    installLsa(std::make_shared<NameLsa>(block));

                    if( m_confParam.isABR() ){

                        for( auto & toAreaId : m_confParam.getAreaIdList() ){
                            if( toAreaId == myAreaId )
                                continue;
                            m_sequencingManager.increaseInterNameLsaSeq(toAreaId);
                            m_sequencingManager.writeSeqNoToFile(toAreaId);
                            m_sync.publishRoutingUpdate(toAreaId, Lsa::Type::INTERNAME,
                                    m_sequencingManager.getInterNameLsaSeq(toAreaId));
                        }

                    }

                }else{
                    std::cout << "!!!! isLsaNew: " << interestName << ", seqNo: " << seqNo << std::endl;
                }
			} else if (interestedLsType == Lsa::Type::INTERNAME) {
                if( block.type() == ndn::tlv::nlsr::NameLsa ){
                    NameLsa nameLsa(block);
                    if(nameLsa.getNpl().getNames().size()==0){
                        std::cout << "Warning... interNameLsa's Npl is empty" << std::endl;
                        return;
                    }
				    if (isLsaNew(originRouter, interestedLsType, seqNo, fromAreaId)) {
                        InterNameLsa interNameLsa( nameLsa.getOriginRouter(), 
                           seqNo, getLsaExpirationTimePoint(), nameLsa.getNpl(), myAreaId);

                        //std::cout << interNameLsa << std::endl;
				    	installLsa(std::make_shared<InterNameLsa>(interNameLsa));
                    if( m_confParam.isABR() ){
                        for( auto & toAreaId : m_confParam.getAreaIdList() ){
                            if( toAreaId == fromAreaId )
                                continue;
                            m_sequencingManager.increaseInterNameLsaSeq(toAreaId);
                            m_sequencingManager.writeSeqNoToFile(toAreaId);
                            m_sync.publishRoutingUpdate(toAreaId, Lsa::Type::INTERNAME,
                                    m_sequencingManager.getInterNameLsaSeq(toAreaId));
                        }
                    }
				    }
                }
			} else if (interestedLsType == Lsa::Type::NET_EXTERNALNAME) {
                if( block.type() == ndn::tlv::nlsr::NetExternalNameLsa ){
                    NetExternalNameLsa nameLsa(block);
                    if(nameLsa.getNpl().getNames().size()==0){
                        std::cout << "Warning... NetExternalNameLsa's Npl is empty" << std::endl;
                        return;
                    }
				    if (isLsaNew(originRouter, interestedLsType, seqNo, fromAreaId)) {
                        //std::cout << nameLsa << std::endl;
				    	installLsa(std::make_shared<NetExternalNameLsa>(block));
                    if( m_confParam.isABR() ){
                        for( auto & toAreaId : m_confParam.getAreaIdList() ){
                            if( toAreaId == fromAreaId )
                                continue;
                            m_sequencingManager.increaseNetExternalNameLsaSeq(toAreaId);
                            m_sequencingManager.writeSeqNoToFile(toAreaId);
                            m_sync.publishRoutingUpdate(toAreaId, Lsa::Type::NET_EXTERNALNAME,
                                    m_sequencingManager.getNetExternalNameLsaSeq(toAreaId));
                        }
                    }
				    }
                }
			} else if (interestedLsType == Lsa::Type::EXTERNALNAME) {
                if( block.type() == ndn::tlv::nlsr::ExternalNameLsa ){
                    ExternalNameLsa nameLsa(block);
                    if(nameLsa.getNpl().getNames().size()==0){
                        std::cout << "Warning... ExternalNameLsa's Npl is empty" << std::endl;
                        return;
                    }
				    if (isLsaNew(originRouter, interestedLsType, seqNo, fromAreaId)) {
                        //std::cout << nameLsa << std::endl;
				    	installLsa(std::make_shared<ExternalNameLsa>(block));
                    if( m_confParam.isABR() ){
                        for( auto & toAreaId : m_confParam.getAreaIdList() ){
                            if( toAreaId == fromAreaId )
                                continue;
                            m_sequencingManager.increaseExternalNameLsaSeq(toAreaId);
                            m_sequencingManager.writeSeqNoToFile(toAreaId);
                            m_sync.publishRoutingUpdate(toAreaId, Lsa::Type::EXTERNALNAME,
                                    m_sequencingManager.getExternalNameLsaSeq(toAreaId));
                        }
                    }
				    }
                }
			} else if (interestedLsType == Lsa::Type::ADJACENCY) {
				lsaIncrementSignal(Statistics::PacketType::RCV_ADJ_LSA_DATA);
				if (isLsaNew(originRouter, interestedLsType, seqNo, fromAreaId)) {
					installLsa(std::make_shared<AdjLsa>(block));
				}
			}
			else if (interestedLsType == Lsa::Type::COORDINATE) {
				lsaIncrementSignal(Statistics::PacketType::RCV_COORD_LSA_DATA);
				if (isLsaNew(originRouter, interestedLsType, seqNo, fromAreaId)) {
					installLsa(std::make_shared<CoordinateLsa>(block));
				}
			}
		}
		catch (const std::exception& e) {
			NLSR_LOG_TRACE("LSA data decoding error :( " << e.what());
			return;
		}
	}
}

} // namespace nlsr
