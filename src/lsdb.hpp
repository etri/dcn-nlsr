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

#ifndef NLSR_LSDB_HPP
#define NLSR_LSDB_HPP

#include "conf-parameter.hpp"
#include "lsa/lsa.hpp"
#include "lsa/name-lsa.hpp"
#include "lsa/inter-lsa.hpp"
#include "lsa/external-lsa.hpp"
#include "lsa/net-external-lsa.hpp"
#include "lsa/coordinate-lsa.hpp"
#include "lsa/adj-lsa.hpp"
#include "sequencing-manager.hpp"
#include "test-access-control.hpp"
#include "communication/sync-logic-handler.hpp"
#include "statistics.hpp"
#include "route/name-prefix-table.hpp"

#include <ndn-cxx/security/key-chain.hpp>
#include <ndn-cxx/util/signal.hpp>
#include <ndn-cxx/util/time.hpp>
#include <ndn-cxx/encoding/nfd-constants.hpp>
#include <ndn-cxx/util/segment-fetcher.hpp>
#include <ndn-cxx/ims/in-memory-storage-persistent.hpp>

#include <boost/multi_index_container.hpp>
#include <boost/multi_index/hashed_index.hpp>
#include <boost/multi_index/composite_key.hpp>

#include <PSync/segment-publisher.hpp>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include <iostream>

namespace nlsr {

using boost::property_tree::ptree;
using boost::property_tree::read_json;
using boost::property_tree::write_json;

namespace bmi = boost::multi_index;

static constexpr ndn::time::seconds GRACE_PERIOD = 10_s;
static constexpr char BACKBONE_AREA = 0;

class Lsdb
{
public:
  Lsdb(ndn::Face& face, ndn::KeyChain& keyChain, ConfParameter& confParam,
       NamePrefixTable& namePrefixTable, RoutingTable& routingTable, ndn::nfd::Controller &);

  ~Lsdb()
  {
    for (const auto& sp : m_fetchers) {
      sp->stop();
    }
  }


void updateForRedistributeLsa(bool,uint64_t& areaId, uint64_t&, NamePrefixList &redistributeNpl);
  /*! \brief Returns whether the LSDB contains some LSA.
   */
  bool
  doesLsaExist(const ndn::Name& router, Lsa::Type lsaType, uint64_t area)
  {
    return m_lsdb.get<byName>().find(std::make_tuple(router, lsaType, area)) != m_lsdb.end();
  }

  void
  redistributeLsa(uint64_t, uint64_t);

  /*! \brief Builds a name LSA for this router and then installs it
      into the LSDB.
  */
  void
  buildAndInstallOwnNameLsa(uint64_t);

  /*! \brief Builds a cor. LSA for this router and installs it into the LSDB. */
  void
  buildAndInstallOwnCoordinateLsa();

  /*! \brief Schedules a build of this router's LSA. */
  void
  scheduleAdjLsaBuild(uint64_t);

  template<typename T>
  void
  writeLog() const;

  void
  writeLog() const;

  /* \brief Process interest which can be either:
   * 1) Discovery interest from segment fetcher:
   *    /localhop/<network>/nlsr/LSA/<site>/<router>/<lsaType>/<seqNo>
   * 2) Interest containing segment number:
   *    /localhop/<network>/nlsr/LSA/<site>/<router>/<lsaType>/<seqNo>/<version>/<segmentNo>
  */
  void
  processInterest(const ndn::Name& name, const ndn::Interest& interest);

  void
  processMonitoringInterest(const ndn::Name& name, const ndn::Interest& interest);
  bool
  getIsBuildAdjLsaSheduled(uint64_t areaId) const
  {
      auto it = m_isBuildAdjLsaSheduled.find(areaId);
      if(it!=m_isBuildAdjLsaSheduled.end())
          return it->second;
      else
          return false;
  }

  SyncLogicHandler&
  getSync()
  {
    return m_sync;
  }

  template<typename T>
  std::shared_ptr<T>
  findLsa(const ndn::Name& router, uint64_t area) const
  {
    return std::static_pointer_cast<T>(findLsa(router, T::type(), area));
  }

  struct area_hash {
    int
    operator()(const uint64_t& id) const {
      return std::hash<uint64_t>{}(id);
    }
  };
  struct name_hash {
    int
    operator()(const ndn::Name& name) const {
      return std::hash<ndn::Name>{}(name);
    }
  };

  struct enum_class_hash {
    template<typename T>
    int
    operator()(T t) const {
      return static_cast<int>(t);
    }
  };

  struct byName{};
  struct byType{};

  using LsaContainer = boost::multi_index_container<
    std::shared_ptr<Lsa>,
    bmi::indexed_by<
      bmi::hashed_unique<
        bmi::tag<byName>,
        bmi::composite_key<
          Lsa,
          bmi::const_mem_fun<Lsa, ndn::Name, &Lsa::getOriginRouterCopy>,
          bmi::const_mem_fun<Lsa, Lsa::Type, &Lsa::getType>,
          bmi::const_mem_fun<Lsa, uint64_t, &Lsa::getAreaId>
        >,
        bmi::composite_key_hash<name_hash, enum_class_hash, area_hash>
      >,
      bmi::hashed_non_unique<
        bmi::tag<byType>,
        bmi::const_mem_fun<Lsa, Lsa::Type, &Lsa::getType>,
        enum_class_hash
      >
    >
  >;

  template<typename T>
  std::pair<LsaContainer::index<Lsdb::byType>::type::iterator,
            LsaContainer::index<Lsdb::byType>::type::iterator>
  getLsdbIterator() const
  {
    return m_lsdb.get<byType>().equal_range(T::type());
  }

PUBLIC_WITH_TESTS_ELSE_PRIVATE:
  std::shared_ptr<Lsa>
  findLsa(const ndn::Name& router, Lsa::Type lsaType, uint64_t area) const
  {
    auto it = m_lsdb.get<byName>().find(std::make_tuple(router, lsaType, area));
    return it != m_lsdb.end() ? *it : nullptr;
  }

  void
  incrementDataSentStats(Lsa::Type lsaType) {
    if (lsaType == Lsa::Type::NAME) {
      lsaIncrementSignal(Statistics::PacketType::SENT_NAME_LSA_DATA);
    }
    else if (lsaType == Lsa::Type::ADJACENCY) {
      lsaIncrementSignal(Statistics::PacketType::SENT_ADJ_LSA_DATA);
    }
    else if (lsaType == Lsa::Type::COORDINATE) {
      lsaIncrementSignal(Statistics::PacketType::SENT_COORD_LSA_DATA);
    }
    else if (lsaType == Lsa::Type::INTERNAME) {
      lsaIncrementSignal(Statistics::PacketType::SENT_INTERNAME_LSA_DATA);
    }
    else if (lsaType == Lsa::Type::EXTERNALNAME) {
      lsaIncrementSignal(Statistics::PacketType::SENT_EXTERNALNAME_LSA_DATA);
    }
  }

  void
  incrementInterestRcvdStats(Lsa::Type lsaType) {
    if (lsaType == Lsa::Type::NAME) {
      lsaIncrementSignal(Statistics::PacketType::RCV_NAME_LSA_INTEREST);
    }
    else if (lsaType == Lsa::Type::ADJACENCY) {
      lsaIncrementSignal(Statistics::PacketType::RCV_ADJ_LSA_INTEREST);
    }
    else if (lsaType == Lsa::Type::INTERNAME) {
      lsaIncrementSignal(Statistics::PacketType::RCV_INTERNAME_LSA_INTEREST);
    }
    else if (lsaType == Lsa::Type::EXTERNALNAME) {
      lsaIncrementSignal(Statistics::PacketType::RCV_EXTERNALNAME_LSA_INTEREST);
    }
    else if (lsaType == Lsa::Type::COORDINATE) {
      lsaIncrementSignal(Statistics::PacketType::RCV_COORD_LSA_INTEREST);
    }
  }

  void
  incrementInterestSentStats(Lsa::Type lsaType) {
    if (lsaType == Lsa::Type::NAME) {
      lsaIncrementSignal(Statistics::PacketType::SENT_NAME_LSA_INTEREST);
    }
    else if (lsaType == Lsa::Type::ADJACENCY) {
      lsaIncrementSignal(Statistics::PacketType::SENT_ADJ_LSA_INTEREST);
    }
    else if (lsaType == Lsa::Type::COORDINATE) {
      lsaIncrementSignal(Statistics::PacketType::SENT_COORD_LSA_INTEREST);
    }
    else if (lsaType == Lsa::Type::INTERNAME) {
      lsaIncrementSignal(Statistics::PacketType::SENT_INTERNAME_LSA_INTEREST);
    }
    else if (lsaType == Lsa::Type::EXTERNALNAME) {
      lsaIncrementSignal(Statistics::PacketType::SENT_EXTERNALNAME_LSA_INTEREST);
    }
  }

  /*! Returns whether a seq. no. from a certain router signals a new LSA.
    \param originRouter The name of the originating router.
    \param lsaType The type of the LSA.
    \param seqNo The sequence number to check.
  */
  bool
  isLsaNew(const ndn::Name& originRouter, const Lsa::Type& lsaType, uint64_t lsSeqNo, uint64_t area)
  {
    // Is the name in the LSDB and the supplied seq no is the highest so far
    auto lsaPtr = findLsa(originRouter, lsaType, area);
    return lsaPtr ? lsaPtr->getSeqNo() < lsSeqNo : true;
  }

  void
  installLsa(std::shared_ptr<Lsa> lsa);

  /*! \brief Remove a name LSA from the LSDB.
    \param router The name of the router that published the LSA to remove.
    \param lsaType The type of the LSA.

    This function will remove a name LSA from the LSDB by finding an
    LSA whose name matches key. This removal also causes the NPT to
    remove those name prefixes if no more LSAs advertise them.
   */
  bool
  removeLsa(const ndn::Name& router, Lsa::Type lsaType, uint64_t);

  /*! \brief Attempts to construct an adj. LSA.

    This function will attempt to construct an adjacency LSA. An LSA
    can only be built when the status of all of the router's neighbors
    is known. That is, when we are not currently trying to contact any
    neighbor.
   */
  void
  buildAdjLsa(uint64_t);

  /*! \brief Wrapper event to build and install an adj. LSA for this router. */
  void
  buildAndInstallOwnAdjLsa(uint64_t);

  /*! \brief Schedules a refresh/expire event in the scheduler.
    \param lsa The LSA.
    \param expTime How many seconds to wait before triggering the event.
   */
  ndn::scheduler::EventId
  scheduleLsaExpiration(std::shared_ptr<Lsa> lsa, ndn::time::seconds expTime)
  {
    return m_scheduler.schedule(expTime + GRACE_PERIOD, [this, lsa] { expireOrRefreshLsa(lsa); });
  }

  /*! \brief Either allow to expire, or refresh a name LSA.
    \param lsa The LSA.
  */
  void
  expireOrRefreshLsa(std::shared_ptr<Lsa> lsa);

  bool
  processInterestForLsa(const ndn::Interest& interest, const ndn::Name& originRouter,
                        Lsa::Type lsaType, uint64_t seqNo, uint64_t);

  void
  expressInterest(uint64_t,const ndn::Name& interestName, uint32_t timeoutCount,
                  ndn::time::steady_clock::TimePoint deadline = DEFAULT_LSA_RETRIEVAL_DEADLINE);

  /*!
     \brief Error callback when SegmentFetcher fails to return an LSA

     In all error cases, a reattempt to fetch the LSA will be made.

     Segment validation can fail either because the packet does not have a
     valid signature (fatal) or because some of the certificates in the trust chain
     could not be fetched (non-fatal).

     Currently, the library does not provide clear indication (besides a plain-text message
     in the error callback) of the reason for the failure nor the segment that failed
     to be validated, thus we will continue to try to fetch the LSA until the deadline
     is reached.
   */
  void
  onFetchLsaError(uint64_t,uint32_t errorCode, const std::string& msg,
                  const ndn::Name& interestName, uint32_t retransmitNo,
                  const ndn::time::steady_clock::TimePoint& deadline,
                  ndn::Name lsaName, uint64_t seqNo);

  /*!
     \brief Success callback when SegmentFetcher returns a valid LSA

     \param interestName The base Interest used to fetch the LSA in the format:
            /<network>/NLSR/LSA/<site>/%C1.Router/<router>/<lsa-type>/<seqNo>
   */
  void
  afterFetchLsa(const ndn::ConstBufferPtr& bufferPtr, const ndn::Name& interestName, uint64_t);

  void
  emitSegmentValidatedSignal(const ndn::Data& data)
  {
    afterSegmentValidatedSignal(data);
  }

  ndn::time::system_clock::TimePoint
  getLsaExpirationTimePoint() const
  {
    return ndn::time::system_clock::now() + ndn::time::seconds(m_confParam.getRouterDeadInterval());
  }

public:
  ndn::util::signal::Signal<Lsdb, Statistics::PacketType> lsaIncrementSignal;
  ndn::util::signal::Signal<Lsdb, const ndn::Data&> afterSegmentValidatedSignal;

PUBLIC_WITH_TESTS_ELSE_PRIVATE:
  ndn::Face& m_face;
  ndn::Scheduler m_scheduler;

  ConfParameter& m_confParam;
  NamePrefixTable& m_namePrefixTable;
  RoutingTable& m_routingTable;

  SyncLogicHandler m_sync;

  LsaContainer m_lsdb;

  ndn::time::seconds m_lsaRefreshTime;
  ndn::time::seconds m_adjLsaBuildInterval;
  const ndn::Name& m_thisRouterPrefix;

  // Maps the name of an LSA to its highest known sequence number from sync;
  // Used to stop NLSR from trying to fetch outdated LSAs
  std::map<ndn::Name, uint64_t> m_highestSeqNo;

  SequencingManager m_sequencingManager;

  ndn::util::signal::ScopedConnection m_onNewLsaConnection;

  std::set<std::shared_ptr<ndn::util::SegmentFetcher>> m_fetchers;
  psync::SegmentPublisher m_segmentPublisher;

  std::map<uint64_t, bool> m_isBuildAdjLsaSheduled;
  std::map<uint64_t, int64_t> m_adjBuildCount;
  std::map<uint64_t, ndn::scheduler::ScopedEventId> m_scheduledAdjLsaBuild;

  ndn::InMemoryStoragePersistent m_lsaStorage;

  ndn::nfd::Controller& m_controller;

  const ndn::Name::Component NAME_COMPONENT = ndn::Name::Component("lsdb");
  static const ndn::time::steady_clock::TimePoint DEFAULT_LSA_RETRIEVAL_DEADLINE;

private:
  struct Origin
  {   
      ptree adjLsa;
      ptree nameLsa;
      ptree inter_nameLsa;
      ptree external_nameLsa;
      ptree net_external_nameLsa;
  };  
  std::map<ndn::Name, Origin> m_origins;

  ndn::KeyChain& m_keyChain;
};

} // namespace nlsr

#endif // NLSR_LSDB_HPP
