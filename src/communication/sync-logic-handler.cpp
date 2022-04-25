/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/**
 * Copyright (c) 2014-2020,  The University of Memphis,
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

#include "sync-logic-handler.hpp"
#include "common.hpp"
#include "conf-parameter.hpp"
#include "lsa/lsa.hpp"
#include "logger.hpp"
#include "utility/name-helper.hpp"

namespace nlsr {

INIT_LOGGER(SyncLogicHandler);

SyncLogicHandler::SyncLogicHandler(ndn::Face& face, const IsLsaNew& isLsaNew,
                                   const ConfParameter& conf)
  : onNewLsa(std::make_unique<OnNewLsa>())
  , m_syncFace(face)
  , m_isLsaNew(isLsaNew)
  , m_confParam( const_cast<ConfParameter&>(conf))
  , m_nameLsaUserPrefix(ndn::Name(m_confParam.getSyncUserPrefix()).append(boost::lexical_cast<std::string>(Lsa::Type::NAME)))
  , m_syncLogic(m_syncFace, m_confParam.getSyncProtocol(), m_confParam.getSyncPrefix(),
                m_nameLsaUserPrefix, m_confParam.getSyncInterestLifetime(),
                std::bind(&SyncLogicHandler::processUpdate, this, _1, _2), conf)
{
}

    void
SyncLogicHandler::processUpdate(const ndn::Name& updateName1, uint64_t highSeq)
{
    NLSR_LOG_DEBUG("Update Name: " << updateName1 << " Seq no: " << highSeq);
    std::cout << "Update Name: " << updateName1 << " Seq no: " << highSeq << std::endl;
    //modori
    Lsa::Type lsaType;
    std::istringstream(updateName1.get(updateName1.size()-1).toUri()) >> lsaType;


    ndn::Name updateName = updateName1.getSubName(0, updateName1.size()-1);
    updateName.append(boost::lexical_cast<std::string>(lsaType));

    int32_t nlsrPosition = util::getNameComponentPosition(updateName, NLSR_COMPONENT);
    int32_t lsaPosition = util::getNameComponentPosition(updateName, LSA_COMPONENT);

    if (nlsrPosition < 0 || lsaPosition < 0) {
        NLSR_LOG_WARN("Received malformed sync update");
        return;
    }

    std::string areaId;
    std::istringstream(updateName1.get(lsaPosition-1).toUri()) >> areaId;
    int solicitedArea=-1;
    sscanf(areaId.c_str(), "area-%d", &solicitedArea);

    //std::cout << "Info: Area: " << solicitedArea << ", " << areaId << std::endl;

    ndn::Name networkName = updateName.getSubName(1, nlsrPosition-1);
    ndn::Name routerName = updateName.getSubName(lsaPosition + 1).getPrefix(-1);

    //std::cout << "routerName: " << routerName << std::endl;

    ndn::Name originRouter = networkName;
    originRouter.append(routerName);

#if 0
    if(lsaType == Lsa::Type::INTERNAME){
        if( m_confParam.isABR() ){ // ABR
            //ABR이 관리하고 있는 Area인 경우에는 Drop
            if(m_confParam.getAreaIdList().find(solicitedArea) != m_confParam.getAreaIdList().end() ){
                std::cout << "ABR and solicitedArea(" << solicitedArea << ") exist." << std::endl;
                std::cout << "So, We don't have to request INTERNAME LSA(Drop)." << std::endl;
                return;
            }else{
                //ABR이 관리하는 Area가 아니기 때문에 Inter LSA를 가져와야 하나, 만일 ABR의 이웃이 아니면 Drop
                if(!m_confParam.getAdjacencyList().isNeighbor(originRouter)){
                    std::cout << "ABR and not solicitedArea but haven't OriginRouter-> Drop" << std::endl;
                    return;
                }
            }
        }
    }
#endif
    processUpdateFromSync(originRouter, updateName, highSeq, solicitedArea);

}

void
SyncLogicHandler::processUpdateFromSync(const ndn::Name& originRouter,
                                        const ndn::Name& updateName, uint64_t seqNo, uint64_t area)
{
  NLSR_LOG_DEBUG("Origin Router of update: " << originRouter);
  //std::cout << "Origin Router of update: " << updateName << ", Area: " << area << std::endl;

  // A router should not try to fetch its own LSA
  if (originRouter != m_confParam.getRouterPrefix()) {

    Lsa::Type lsaType;
    std::istringstream(updateName.get(updateName.size()-1).toUri()) >> lsaType;

    NLSR_LOG_DEBUG("Received sync update with higher " << lsaType <<
                   " sequence number than entry in LSDB");

    if (m_isLsaNew(originRouter, lsaType, seqNo, area)) {
      if (lsaType == Lsa::Type::ADJACENCY && seqNo != 0 &&
          m_confParam.getHyperbolicState() == HYPERBOLIC_STATE_ON) {
        NLSR_LOG_ERROR("Got an update for adjacency LSA when hyperbolic routing " <<
                       "is enabled. Not going to fetch.");
        return;
      }

      if (lsaType == Lsa::Type::COORDINATE && seqNo != 0 &&
          m_confParam.getHyperbolicState() == HYPERBOLIC_STATE_OFF) {
        NLSR_LOG_ERROR("Got an update for coordinate LSA when link-state " <<
                       "is enabled. Not going to fetch.");
        return;
      }
      (*onNewLsa)(updateName, seqNo, originRouter, area);
    }
  }
}

void
SyncLogicHandler::publishRoutingUpdate(uint64_t toArea, const Lsa::Type& type, const uint64_t& seqNo)
{
	ndn::Name userPrefix("localhop");
	userPrefix.append(m_confParam.getNetwork());
	userPrefix.append("nlsr");
	userPrefix.append("area-"+std::to_string(toArea));
	userPrefix.append("LSA");
	userPrefix.append(m_confParam.getSiteName());
	userPrefix.append(m_confParam.getRouterName());
	//userPrefix.append("area"+std::to_string(toArea));

	//userPrefix.append(boost::lexical_cast<std::string>(type));
    if( type==Lsa::Type::INTERNAME )
	    userPrefix.append("IA-NAME");
    else if( type==Lsa::Type::NAME )
	    userPrefix.append(boost::lexical_cast<std::string>(type));
    else if( type==Lsa::Type::EXTERNALNAME )
	    userPrefix.append("Ext-NAME");
    else if( type==Lsa::Type::NET_EXTERNALNAME )
	    userPrefix.append("Ext-Net-NAME");
    else if( type==Lsa::Type::ADJACENCY )
	    userPrefix.append(boost::lexical_cast<std::string>(type));
    else {
        std::cout << "0-addUserNode: " << type << std::endl;
	    userPrefix.append("BASE");
    }

	m_syncLogic.publishUpdate(toArea, userPrefix, seqNo);
}

void
SyncLogicHandler::addUserNode(uint64_t toArea, const Lsa::Type& type)
{
	ndn::Name userPrefix("localhop");
	userPrefix.append(m_confParam.getNetwork());
	userPrefix.append("nlsr");
	userPrefix.append("area-"+std::to_string(toArea));
	userPrefix.append("LSA");
	userPrefix.append(m_confParam.getSiteName());
	userPrefix.append(m_confParam.getRouterName());
	//userPrefix.append("area"+std::to_string(fromArea));
	//userPrefix.append(boost::lexical_cast<std::string>(type));
    if( type==Lsa::Type::INTERNAME )
	    userPrefix.append("IA-NAME");
    else if( type==Lsa::Type::NAME )
	    userPrefix.append(boost::lexical_cast<std::string>(type));
    else if( type==Lsa::Type::EXTERNALNAME )
	    userPrefix.append("Ext-NAME");
    else if( type==Lsa::Type::NET_EXTERNALNAME )
	    userPrefix.append("Ext-Net-NAME");
    else if( type==Lsa::Type::ADJACENCY )
	    userPrefix.append(boost::lexical_cast<std::string>(type));
    else {
        std::cout << "1-addUserNode: " << type << std::endl;
	    userPrefix.append("BASE");
    }

	m_syncLogic.addUserNode(toArea, userPrefix);

}
} // namespace nlsr
