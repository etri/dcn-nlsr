/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/**
 * Copyright (c) 2014-2018,  The University of Memphis,
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
 **/

#include "sync-protocol-adapter.hpp"
#include "logger.hpp"
#include "lsa/lsa.hpp"

INIT_LOGGER(SyncProtocolAdapter);

namespace nlsr {

const auto FIXED_SESSION = ndn::name::Component::fromNumber(0);

SyncProtocolAdapter::SyncProtocolAdapter(ndn::Face& face,
                                         int32_t syncProtocol,
                                         const ndn::Name& syncPrefix,
                                         const ndn::Name& userPrefix, // nameLSA
                                         ndn::time::milliseconds syncInterestLifetime,
                                         const SyncUpdateCallback& syncUpdateCallback, const ConfParameter&conf)
 : m_syncProtocol(syncProtocol)
 , m_syncUpdateCallback(syncUpdateCallback)
, m_confParam(const_cast<ConfParameter&>(conf))
{
  if (m_syncProtocol == SYNC_PROTOCOL_CHRONOSYNC) {
    NDN_LOG_DEBUG("Using ChronoSync");
    m_chronoSyncLogic = std::make_shared<chronosync::Logic>(face,
                          syncPrefix,
                          userPrefix,
                          std::bind(&SyncProtocolAdapter::onChronoSyncUpdate, this, _1),
                          chronosync::Logic::DEFAULT_NAME,
                          chronosync::Logic::DEFAULT_VALIDATOR,
                          chronosync::Logic::DEFAULT_RESET_TIMER,
                          chronosync::Logic::DEFAULT_CANCEL_RESET_TIMER,
                          chronosync::Logic::DEFAULT_RESET_INTEREST_LIFETIME,
                          syncInterestLifetime,
                          chronosync::Logic::DEFAULT_SYNC_REPLY_FRESHNESS,
                          chronosync::Logic::DEFAULT_RECOVERY_INTEREST_LIFETIME,
                          FIXED_SESSION);
  }
  else {
    NDN_LOG_DEBUG("Using PSync");
#if 1

	for( auto areaId : m_confParam.getAreaIdList()){
		ndn::Name sPrefix;
		ndn::Name uPrefix;

        uPrefix.append("localhop");
        uPrefix.append(m_confParam.getNetwork());
        uPrefix.append("nlsr");
        uPrefix.append("area-"+std::to_string(areaId));
        uPrefix.append("LSA");
        uPrefix.append(m_confParam.getSiteName());
        uPrefix.append(m_confParam.getRouterName());
        //uPrefix.append("area"+std::to_string(areaId));

        uPrefix.append(boost::lexical_cast<std::string>(Lsa::Type::NAME));

        sPrefix.append("localhop");	
        sPrefix.append(m_confParam.getNetwork());	
        sPrefix.append("nlsr");
        sPrefix.append("area-"+std::to_string(areaId));
        sPrefix.append("sync");
        sPrefix.appendVersion(m_confParam.getVersion());

        std::cout << "uPrefix: " << uPrefix << ", sPrefix: " << sPrefix << std::endl;
        m_psyncLogic2[areaId] = std::make_shared<psync::FullProducer>(80,
                face, sPrefix, uPrefix,
                std::bind(&SyncProtocolAdapter::onPSyncUpdate, this, _1),
                syncInterestLifetime);

        uPrefix = "localhop";
        uPrefix.append(m_confParam.getNetwork());
        uPrefix.append("nlsr");
        uPrefix.append("area-"+std::to_string(areaId));
        uPrefix.append("LSA");
        uPrefix.append(m_confParam.getSiteName());
        uPrefix.append(m_confParam.getRouterName());

        uPrefix.append(boost::lexical_cast<std::string>(Lsa::Type::ADJACENCY));
        std::cout << "uPrefix: " << uPrefix << std::endl;
        m_psyncLogic2[areaId]->addUserNode(uPrefix);

        uPrefix = "localhop";
        uPrefix.append(m_confParam.getNetwork());
        uPrefix.append("nlsr");
        uPrefix.append("area-"+std::to_string(areaId));
        uPrefix.append("LSA");
        uPrefix.append(m_confParam.getSiteName());
        uPrefix.append(m_confParam.getRouterName());
        uPrefix.append("IA-NAME");
        std::cout << "uPrefix: " << uPrefix << std::endl;
        m_psyncLogic2[areaId]->addUserNode(uPrefix);

        uPrefix = "localhop";
        uPrefix.append(m_confParam.getNetwork());
        uPrefix.append("nlsr");
        uPrefix.append("area-"+std::to_string(areaId));
        uPrefix.append("LSA");
        uPrefix.append(m_confParam.getSiteName());
        uPrefix.append(m_confParam.getRouterName());
        //uPrefix.append(boost::lexical_cast<std::string>(Lsa::Type::EXTERNALNAME));
        uPrefix.append("Ext-NAME");
        std::cout << "uPrefix: " << uPrefix << std::endl;
        m_psyncLogic2[areaId]->addUserNode(uPrefix);

        uPrefix = "localhop";
        uPrefix.append(m_confParam.getNetwork());
        uPrefix.append("nlsr");
        uPrefix.append("area-"+std::to_string(areaId));
        uPrefix.append("LSA");
        uPrefix.append(m_confParam.getSiteName());
        uPrefix.append(m_confParam.getRouterName());
        //uPrefix.append(boost::lexical_cast<std::string>(Lsa::Type::NET_EXTERNALNAME));
        uPrefix.append("Ext-Net-NAME");
        std::cout << "uPrefix: " << uPrefix << std::endl;
        m_psyncLogic2[areaId]->addUserNode(uPrefix);
    }

#else
    m_psyncLogic = std::make_shared<psync::FullProducer>(80,
                     face,
                     syncPrefix,
                     userPrefix,
                     std::bind(&SyncProtocolAdapter::onPSyncUpdate, this, _1),
                     syncInterestLifetime);
#endif
  }
}

void
SyncProtocolAdapter::addUserNode(uint64_t toArea, const ndn::Name& userPrefix)
{
  if (m_syncProtocol == SYNC_PROTOCOL_CHRONOSYNC) {
    m_chronoSyncLogic->addUserNode(userPrefix, chronosync::Logic::DEFAULT_NAME, FIXED_SESSION);
  }
  else {
    //m_psyncLogic->addUserNode(userPrefix);
    std::cout << "SyncProtocolAdapter: " << userPrefix << " is added into Area(" << toArea << ")" << std::endl;
    m_psyncLogic2[toArea]->addUserNode(userPrefix);
  }
}

void
SyncProtocolAdapter::publishUpdate(uint64_t area, const ndn::Name& userPrefix, uint64_t seq)
{
//	std::cout << "SyncProtocolAdapter::publishUpdate: " << userPrefix << std::endl;
  if (m_syncProtocol == SYNC_PROTOCOL_CHRONOSYNC) {
    m_chronoSyncLogic->updateSeqNo(seq, userPrefix);
  }
  else {
    //m_psyncLogic->publishName(userPrefix, seq);
    m_psyncLogic2[area]->publishName(userPrefix, seq);
  }
}

void
SyncProtocolAdapter::onChronoSyncUpdate(const std::vector<chronosync::MissingDataInfo>& updates)
{
  NLSR_LOG_TRACE("Received ChronoSync update event");

  for (const auto& update : updates) {
    // Remove FIXED_SESSION
    m_syncUpdateCallback(update.session.getPrefix(-1), update.high);
  }
}

void
SyncProtocolAdapter::onPSyncUpdate(const std::vector<psync::MissingDataInfo>& updates)
{
  NLSR_LOG_TRACE("Received PSync update event");

  for (const auto& update : updates) {
    m_syncUpdateCallback(update.prefix, update.highSeq);
  }
}

} // namespace nlsr
