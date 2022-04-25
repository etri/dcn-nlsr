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

#ifndef NLSR_LSA_LSA_HPP
#define NLSR_LSA_LSA_HPP

#include "name-prefix-list.hpp"
#include "adjacent.hpp"
#include "adjacency-list.hpp"
#include "test-access-control.hpp"

#include <ndn-cxx/util/scheduler.hpp>

namespace nlsr {

/*!
   \brief Data abstraction for Lsa
   Lsa := LSA-TYPE TLV-LENGTH
            Name
            SequenceNumber
            ExpirationTimePoint
 */
class Lsa
{
public:
  class Error : public ndn::tlv::Error
  {
  public:
    using ndn::tlv::Error::Error;
  };

  enum class Type {
    ADJACENCY,
    COORDINATE,
    NET_EXTERNALNAME, // E(Net External Name, static from nfdc)
    EXTERNALNAME, // E(External Name, static from nfdc)
    INTERNAME, // O IA(Inter Area)
    NAME, // O(Intra Area)
    BASE
  };

protected:
  Lsa(const ndn::Name& originRouter, uint64_t seqNo,
      ndn::time::system_clock::TimePoint expirationTimePoint, uint64_t);

  Lsa() = default;

public:
  virtual
  ~Lsa() = default;

  virtual Type
  getType() const = 0;

  void
  setAreaId(uint64_t id)
  {
    m_areaId = id;
    m_wire.reset();
  }

  uint64_t
  getAreaId() const
  {
    return m_areaId;
 }
  void
  setSeqNo(uint64_t seqNo)
  {
    m_seqNo = seqNo;
    m_wire.reset();
  }

  uint64_t
  getSeqNo() const
  {
    return m_seqNo;
  }

  const ndn::Name&
  getOriginRouter() const
  {
    return m_originRouter;
  }

  void setOriginRouter(ndn::Name ori)
  {
      m_originRouter=ori;
    m_wire.reset();
  }

  ndn::Name
  getOriginRouterCopy() const
  {
    return m_originRouter;
  }

  const ndn::time::system_clock::TimePoint&
  getExpirationTimePoint() const
  {
    return m_expirationTimePoint;
  }

  void
  setExpirationTimePoint(const ndn::time::system_clock::TimePoint& lt)
  {
    m_expirationTimePoint = lt;
    m_wire.reset();
  }

  void
  setExpiringEventId(ndn::scheduler::EventId eid)
  {
    m_expiringEventId = std::move(eid);
  }

  ndn::scheduler::EventId
  getExpiringEventId() const
  {
    return m_expiringEventId;
  }

  /*! Get data common to all LSA types.
   */
  virtual std::string
  toString() const;

  virtual const ndn::Block&
  wireEncode() const = 0;

protected:
  template<ndn::encoding::Tag TAG>
  size_t
  wireEncode(ndn::EncodingImpl<TAG>& block) const;

  void
  wireDecode(const ndn::Block& wire);

PUBLIC_WITH_TESTS_ELSE_PROTECTED:
  ndn::Name m_originRouter;
  uint64_t m_seqNo = 0;
  ndn::time::system_clock::TimePoint m_expirationTimePoint;
  ndn::scheduler::EventId m_expiringEventId;
//ETRI 
 uint64_t m_areaId;

  mutable ndn::Block m_wire;
};

NDN_CXX_DECLARE_WIRE_ENCODE_INSTANTIATIONS(Lsa);

std::ostream&
operator<<(std::ostream& os, const Lsa::Type& type);

std::istream&
operator>>(std::istream& is, Lsa::Type& type);

} // namespace nlsr

#endif // NLSR_LSA_LSA_HPP
