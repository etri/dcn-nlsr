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

#include "net-external-lsa.hpp"
#include "tlv-nlsr.hpp"

#include <iostream>
namespace nlsr {

NetExternalNameLsa::NetExternalNameLsa(const ndn::Name& originRouter, uint64_t seqNo,
                 const ndn::time::system_clock::TimePoint& timepoint,
                 const NamePrefixList& npl,uint64_t area)
  : Lsa(originRouter, seqNo, timepoint, area)
{
//    std::cout << __func__ << " Npls: " << npl.getNames().size() << std::endl;
  for (const auto& name : npl.getNames()) {
	addName(name);
  }
}

NetExternalNameLsa::NetExternalNameLsa(const ndn::Block& block)
{
  wireDecode(block);
}

template<ndn::encoding::Tag TAG>
size_t
NetExternalNameLsa::wireEncode(ndn::EncodingImpl<TAG>& block) const
{
  size_t totalLength = 0;

  auto names = m_npl.getNames();
  for (auto it = names.rbegin();  it != names.rend(); ++it) {
    totalLength += it->wireEncode(block);
  }

  totalLength += Lsa::wireEncode(block);

  totalLength += block.prependVarNumber(totalLength);
  totalLength += block.prependVarNumber(ndn::tlv::nlsr::NetExternalNameLsa);

  return totalLength;
}

NDN_CXX_DEFINE_WIRE_ENCODE_INSTANTIATIONS(NetExternalNameLsa);

const ndn::Block&
NetExternalNameLsa::wireEncode() const
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
NetExternalNameLsa::wireDecode(const ndn::Block& wire)
{
  m_wire = wire;

  if (m_wire.type() != ndn::tlv::nlsr::NetExternalNameLsa) {
    NDN_THROW(Error("NetExternalNameLsa", m_wire.type()));
  }

  m_wire.parse();

  auto val = m_wire.elements_begin();

  if (val != m_wire.elements_end() && val->type() == ndn::tlv::nlsr::Lsa) {
    Lsa::wireDecode(*val);
    ++val;
  }
  else {
    NDN_THROW(Error("Missing required Lsa field"));
  }

  NamePrefixList npl;
  for (; val != m_wire.elements_end(); ++val) {
    if (val->type() == ndn::tlv::Name) {
      npl.insert(ndn::Name(*val));
    }
    else {
      NDN_THROW(Error("Name", val->type()));
    }
  }
  m_npl = npl;
}

bool
NetExternalNameLsa::isEqualContent(const NetExternalNameLsa& other) const
{
  return m_npl == other.getNpl();
}

std::string
NetExternalNameLsa::toString() const
{
  std::ostringstream os;
  os << Lsa::toString();
  os << "      Names:\n";
  int i = 0;
  for (const auto& name : m_npl.getNames()) {
    os << "        Name " << i++ << ": " << name << "\n";
  }

  return os.str();
}

std::ostream&
operator<<(std::ostream& os, const NetExternalNameLsa& lsa)
{
  return os << lsa.toString();
}

} // namespace nlsr
