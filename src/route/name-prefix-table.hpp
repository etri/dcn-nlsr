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
 **/

#ifndef NLSR_NAME_PREFIX_TABLE_HPP
#define NLSR_NAME_PREFIX_TABLE_HPP

#include "name-prefix-table-entry.hpp"
#include "routing-table-pool-entry.hpp"
#include "signals.hpp"
#include "test-access-control.hpp"
#include "route/fib.hpp"
#include "lsa/lsa.hpp"

#include <list>
#include <map>
#include <unordered_map>

namespace nlsr {

class NamePrefixTable
{
public:
  using RoutingTableEntryPool =
    std::unordered_map<ndn::Name, std::shared_ptr<RoutingTablePoolEntry>>;
  using NptEntryList = std::list<std::shared_ptr<NamePrefixTableEntry>>;
  using const_iterator = NptEntryList::const_iterator;

  NamePrefixTable(Fib& fib, RoutingTable& routingTable,
                  std::unique_ptr<AfterRoutingChange>& afterRoutingChangeSignal);

  ~NamePrefixTable();

  /*! \brief Adds a destination to the specified name prefix.
    \param name The name prefix
    \param destRouter The destination router prefix

    This method adds a router to a name prefix table entry. If the
    name prefix table entry does not exist, it is created. The method
    will first look through its local pool of cached entries to find
    the routing information for destRouter. If it is not found there,
    it will construct one and fill it with information from an
    appropriate RoutingTableEntry in the routing table. If there isn't
    a match, it will instantiate it with no next hops. The FIB will be
    notified of the change to the NPT entry, too.
   */
  void
  addEntry(const ndn::Name& name, const ndn::Name& destRouter, Lsa::Type, uint64_t);

  /*! \brief Removes a destination from a name prefix table entry.
    \param name The name prefix
    \param destRouter The destination.

    This method removes a destination from an entry. It will not fail
    if an invalid name/destination pair are passed. After removal, if
    the RoutingTablePoolEntry has a use count of 0, it is deleted from
    the table. Additionally, if the name prefix has no routing table
    entries associated with it, it is deleted from the NPT. In any
    case, the FIB is informed of the changes.
   */
  void
  removeEntry(const ndn::Name& name, const ndn::Name& destRouter, uint64_t );

  /*! \brief Updates all routing information in the NPT.

    Takes in a list of entries that are assumed to be exhaustive, and
    updates each pool entry with the next hop information contained in
    the corresponding entry in entries. If no entry is found, it is
    assumed that the destination for that pool entry is inaccessible,
    and its next hop information is deleted.
   */
  void
  updateWithNewRoute(const std::list<RoutingTableEntry>& entries, uint64_t&);

  /*! \brief Adds a pool entry to the pool.
    \param rtpe The entry.

    \return A shared_ptr to the entry, now in the pool.

    Adds a RoutingTablePoolEntry to the NPT's local pool. Shared
    pointers are used because it eliminates complicated hacks to deal
    with lifetime issues, and to simplify memory management.
   */
  std::shared_ptr<RoutingTablePoolEntry>
  addRtpeToPool(RoutingTablePoolEntry& rtpe, uint64_t);

  /*! \brief Removes a pool entry from the pool.
    \param rtpePtr The shared_ptr to the entry.

    Removes a pool entry from the pool. Comparing these shared_ptrs
    should not be a problem, because the same pointer is moved around,
    all sourced from this central location. A more robust solution is
    certainly possible, though.
  */
  void
  deleteRtpeFromPool(std::shared_ptr<RoutingTablePoolEntry> rtpePtr, uint64_t);

  void
  writeLog();

  const_iterator
  begin() const;

  const_iterator
  end() const;

PUBLIC_WITH_TESTS_ELSE_PRIVATE:
//  RoutingTableEntryPool m_rtpool;
    std::map<uint64_t, std::shared_ptr<RoutingTableEntryPool>> m_rtpool2;
  NptEntryList m_table;

private:
  Fib& m_fib;
  RoutingTable& m_routingTable;
  // added by ETRI(modori) on 20210604
  std::map<uint64_t, RoutingTable*> m_routingTables;
  ndn::util::signal::Connection m_afterRoutingChangeConnection;
};

inline NamePrefixTable::const_iterator
NamePrefixTable::begin() const
{
  return m_table.begin();
}

inline NamePrefixTable::const_iterator
NamePrefixTable::end() const
{
  return m_table.end();
}

std::ostream&
operator<<(std::ostream& os, const NamePrefixTable& table);

} // namespace nlsr

#endif // NLSR_NAME_PREFIX_TABLE_HPP
