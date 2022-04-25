/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2014-2021,  The University of Memphis,
 *                           Regents of the University of California
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

#include "routing-table.hpp"
#include "nlsr.hpp"
#include "map.hpp"
#include "conf-parameter.hpp"
#include "routing-table-calculator.hpp"
#include "routing-table-entry.hpp"
#include "name-prefix-table.hpp"
#include "logger.hpp"
#include "tlv-nlsr.hpp"

#include <list>
#include <string>

namespace nlsr {

INIT_LOGGER(route.RoutingTable);

RoutingTable::RoutingTable(ndn::Scheduler& scheduler, Fib& fib, Lsdb& lsdb,
                           NamePrefixTable& namePrefixTable, ConfParameter& confParam)
  : afterRoutingChange{std::make_unique<AfterRoutingChange>()}
  , m_scheduler(scheduler)
  , m_fib(fib)
  , m_lsdb(lsdb)
  , m_namePrefixTable(namePrefixTable)
  , m_routingCalcInterval{confParam.getRoutingCalcInterval()}
  , m_isRoutingTableCalculating(false)
  , m_isRouteCalculationScheduled(false)
  , m_confParam(confParam)
{
    for(auto &adj : m_confParam.getAdjacencyList()){
        m_rTable[adj.getAreaId()] = std::make_shared<std::list<RoutingTableEntry>>();
    }
}

void
RoutingTable::calculate(uint64_t area)
{
    //std::cout << " RoutingTable::calculate on Area: " << area << std::endl;
  m_lsdb.writeLog();
  m_namePrefixTable.writeLog();
  if (m_isRoutingTableCalculating == false) {
    // setting routing table calculation
    m_isRoutingTableCalculating = true;

    bool isHrEnabled = m_confParam.getHyperbolicState() != HYPERBOLIC_STATE_OFF;

    if ((!isHrEnabled &&
         m_lsdb.doesLsaExist(m_confParam.getRouterPrefix(), Lsa::Type::ADJACENCY, area))
        ||
        (isHrEnabled &&
         m_lsdb.doesLsaExist(m_confParam.getRouterPrefix(), Lsa::Type::COORDINATE, area))) {
      if (m_lsdb.getIsBuildAdjLsaSheduled(area) != 1) {
        NLSR_LOG_TRACE("Clearing old routing table");
        clearRoutingTable(area);
        // for dry run options
        clearDryRoutingTable();

        NLSR_LOG_DEBUG("Calculating routing table");

        // calculate Link State routing
        if ((m_confParam.getHyperbolicState() == HYPERBOLIC_STATE_OFF) ||
            (m_confParam.getHyperbolicState() == HYPERBOLIC_STATE_DRY_RUN)) {
          calculateLsRoutingTable(area);
        }
        // calculate hyperbolic routing
        if (m_confParam.getHyperbolicState() == HYPERBOLIC_STATE_ON) {
          calculateHypRoutingTable(false);
        }
        // calculate dry hyperbolic routing
        if (m_confParam.getHyperbolicState() == HYPERBOLIC_STATE_DRY_RUN) {
          calculateHypRoutingTable(true);
        }
        // Inform the NPT that updates have been made
        NLSR_LOG_DEBUG("Calling Update NPT With new Route");

        //modori on 20210615
        (*afterRoutingChange)(*m_rTable[area], area);
        //(*afterRoutingChange)(m_rTable, area);

        NLSR_LOG_DEBUG(*this);
        m_namePrefixTable.writeLog();
        m_fib.writeLog();
      }
      else {
        NLSR_LOG_DEBUG("Adjacency building is scheduled, so routing table can not be calculated :(");
      }
    }
    else {
      NLSR_LOG_DEBUG("No Adj LSA of router itself, so Routing table can not be calculated :(");
      clearRoutingTable(area);
      clearDryRoutingTable(); // for dry run options
      // need to update NPT here
      NLSR_LOG_DEBUG("Calling Update NPT With new Route");
        //modori on 20210615
        (*afterRoutingChange)(*m_rTable[area], area);
      //(*afterRoutingChange)(m_rTable, area);
      NLSR_LOG_DEBUG(*this);
      m_namePrefixTable.writeLog();
      m_fib.writeLog();
      // debugging purpose end
    }
    m_isRouteCalculationScheduled = false; // clear scheduled flag
    m_isRoutingTableCalculating = false; // unsetting routing table calculation
  }
  else {
    scheduleRoutingTableCalculation(area);
  }
}

void
RoutingTable::calculateLsRoutingTable(uint64_t area)
{
  NLSR_LOG_TRACE("CalculateLsRoutingTable Called on Area: " << area);
  //std::cout << "CalculateLsRoutingTable Called on Area: " << area << std::endl;

  Map map;
  auto lsaRange = m_lsdb.getLsdbIterator<AdjLsa>();
  map.createFromAdjLsdb(lsaRange.first, lsaRange.second, area);
  map.writeLog();

  size_t nRouters = map.getMapSize();

  LinkStateRoutingTableCalculator calculator(nRouters);

  calculator.calculatePath(map, *this, m_confParam, m_lsdb, area);
}

void
RoutingTable::calculateHypRoutingTable(bool isDryRun)
{
  Map map;
  auto lsaRange = m_lsdb.getLsdbIterator<CoordinateLsa>();
  map.createFromCoordinateLsdb(lsaRange.first, lsaRange.second);
  map.writeLog();

  size_t nRouters = map.getMapSize();

  HyperbolicRoutingCalculator calculator(nRouters, isDryRun, m_confParam.getRouterPrefix());

  calculator.calculatePath(map, *this, m_lsdb, m_confParam.getAdjacencyList());
}

void
RoutingTable::scheduleRoutingTableCalculation(uint64_t area)
{
  if (!m_isRouteCalculationScheduled) { // multi 20210609
    NLSR_LOG_DEBUG("Scheduling routing table calculation in " << m_routingCalcInterval);
    m_scheduler.schedule(m_routingCalcInterval, [area, this] { calculate(area); });
    m_isRouteCalculationScheduled = true; // multi 
  }
}

static bool
routingTableEntryCompare(RoutingTableEntry& rte, ndn::Name& destRouter)
{
  return rte.getDestination() == destRouter;
}

void
RoutingTable::addNextHop(const ndn::Name& destRouter, NextHop& nh, uint64_t area)
{
  NLSR_LOG_DEBUG("Adding " << nh << " for destination: " << destRouter << " on Area: " << area);
  std::cout << "Adding " << nh << " for destination: " << destRouter << " on Area: " << area << std::endl;

//modori on 20210615
  //RoutingTableEntry* rteChk = findRoutingTableEntry(destRouter);
  RoutingTableEntry* rteChk = findRoutingTableEntry(destRouter, area);
  if (rteChk == nullptr) {
    RoutingTableEntry rte(destRouter);
    rte.getNexthopList().addNextHop(nh);
    //modori on 20210615
    auto rt = m_rTable[area];
    rt->push_back(rte);
    //m_rTable.push_back(rte);
  }
  else {
    rteChk->getNexthopList().addNextHop(nh);
  }
}

RoutingTableEntry*
RoutingTable::findRoutingTableEntry(const ndn::Name& destRouter, uint64_t area)
{
    //modori on 20210615
    //std::cout << "findRoutingTableEntry : " << destRouter << " on Area: " << area << std::endl;
    auto rtIt = m_rTable.find(area);
    if(rtIt == m_rTable.end())
        return nullptr;

    auto rt = rtIt->second;

  //auto it = std::find_if(m_rTable.begin(), m_rTable.end(),
  auto it = std::find_if(rt->begin(), rt->end(),
                         std::bind(&routingTableEntryCompare, _1, destRouter));
  //if (it != m_rTable.end()) {
  if (it != rt->end()) {
    return &(*it);
  }
  return nullptr;
}

void
RoutingTable::addNextHopToDryTable(const ndn::Name& destRouter, NextHop& nh)
{
  NLSR_LOG_DEBUG("Adding " << nh << " to dry table for destination: " << destRouter);

  auto it = std::find_if(m_dryTable.begin(), m_dryTable.end(),
                         std::bind(&routingTableEntryCompare, _1, destRouter));
  if (it == m_dryTable.end()) {
    RoutingTableEntry rte(destRouter);
    rte.getNexthopList().addNextHop(nh);
    m_dryTable.push_back(rte);
  }
  else {
    it->getNexthopList().addNextHop(nh);
  }
}

void
RoutingTable::clearRoutingTable(uint64_t area)
{
    #if 0
  if (m_rTable.size() > 0) {
    m_rTable.clear();
  }
#else
  auto rt = m_rTable[area];
  if (rt->size() > 0) {
    rt->clear();
  }
#endif
}

void
RoutingTable::clearDryRoutingTable()
{
  if (m_dryTable.size() > 0) {
    m_dryTable.clear();
  }
}

template<ndn::encoding::Tag TAG>
size_t
RoutingTableStatus::wireEncode(ndn::EncodingImpl<TAG>& block) const
{
  size_t totalLength = 0;

  for (auto it = m_dryTable.rbegin(); it != m_dryTable.rend(); ++it) {
    totalLength += it->wireEncode(block);
  }

#if 0
  for (auto it = m_rTable.rbegin(); it != m_rTable.rend(); ++it) {
    totalLength += it->wireEncode(block);
  }
#endif

  totalLength += block.prependVarNumber(totalLength);
  totalLength += block.prependVarNumber(ndn::tlv::nlsr::RoutingTable);

  return totalLength;
}

NDN_CXX_DEFINE_WIRE_ENCODE_INSTANTIATIONS(RoutingTableStatus);

const ndn::Block&
RoutingTableStatus::wireEncode() const
{
  if (m_wire.hasWire()) {
    return m_wire;
  }

  ndn::EncodingEstimator estimator;
  size_t estimatedSize = wireEncode(estimator);

  ndn::EncodingBuffer buffer(estimatedSize, 0);
  wireEncode(buffer);

  m_wire = buffer.block();

  return m_wire;
}

void
RoutingTableStatus::wireDecode(const ndn::Block& wire)
{
  //m_rTable.clear();

  m_wire = wire;

  if (m_wire.type() != ndn::tlv::nlsr::RoutingTable) {
    NDN_THROW(Error("RoutingTable", m_wire.type()));
  }

  m_wire.parse();
  auto val = m_wire.elements_begin();

  std::set<ndn::Name> destinations;
  for (; val != m_wire.elements_end() && val->type() == ndn::tlv::nlsr::RoutingTableEntry; ++val) {
    auto entry = RoutingTableEntry(*val);

    if (destinations.emplace(entry.getDestination()).second) {
      //m_rTable.push_back(entry);
    }
    else {
      // If destination already exists then this is the start of dry HR table
      m_dryTable.push_back(entry);
    }
  }

  if (val != m_wire.elements_end()) {
    NDN_THROW(Error("Unrecognized TLV of type " + ndn::to_string(val->type()) + " in RoutingTable"));
  }
}

std::ostream&
operator<<(std::ostream& os, const RoutingTableStatus& rts)
{
  os << "Routing Table:\n";
#if 0
  for (const auto& rte : rts.getRoutingTableEntry()) {
    os << rte;
  }
#endif

  if (!rts.getDryRoutingTableEntry().empty()) {
    os << "Dry-Run Hyperbolic Routing Table:\n";
    for (const auto& rte : rts.getDryRoutingTableEntry()) {
      os << rte;
    }
  }
  return os;
}

} // namespace nlsr
