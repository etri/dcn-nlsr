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

#ifndef NLSR_SEQUENCING_MANAGER_HPP
#define NLSR_SEQUENCING_MANAGER_HPP

#include "conf-parameter.hpp"
#include "lsa/lsa.hpp"
#include "test-access-control.hpp"

#include <ndn-cxx/face.hpp>

#include <list>
#include <string>

namespace nlsr {

class SequencingManager
{
public:
  SequencingManager(const std::string& filePath, int hypState);

  void
  setLsaSeq(uint64_t area, uint64_t seqNo, Lsa::Type lsaType)
  {
	SeqInfo &seq = m_seqInfo[area];

    switch (lsaType) {
      case Lsa::Type::ADJACENCY:
        seq.m_adjLsaSeq = seqNo;
        break;
      case Lsa::Type::COORDINATE:
        seq.m_corLsaSeq = seqNo;
        break;
      case Lsa::Type::NAME:
        seq.m_nameLsaSeq = seqNo;
        break;
      case Lsa::Type::INTERNAME:
        seq.m_interNameLsaSeq = seqNo;
        break;
      case Lsa::Type::EXTERNALNAME:
        seq.m_externalNameLsaSeq = seqNo;
        break;
      case Lsa::Type::NET_EXTERNALNAME:
        seq.m_netExternalNameLsaSeq = seqNo;
        break;
      default:
        return;
    }
  }

  uint64_t
  getLsaSeq(uint64_t area, Lsa::Type lsaType)
  {
	SeqInfo &seq = m_seqInfo[area];
    switch (lsaType) {
      case Lsa::Type::ADJACENCY:
        return seq.m_adjLsaSeq;
      case Lsa::Type::COORDINATE:
        return seq.m_corLsaSeq;
      case Lsa::Type::NAME:
        return seq.m_nameLsaSeq;
      case Lsa::Type::INTERNAME:
        return seq.m_interNameLsaSeq;
      case Lsa::Type::NET_EXTERNALNAME:
        return seq.m_netExternalNameLsaSeq;
      case Lsa::Type::EXTERNALNAME:
        return seq.m_externalNameLsaSeq;
      default:
        return 0;
    }
  }

    uint64_t
     getInterNameLsaSeq(uint64_t area)
     {
	   SeqInfo &seq = m_seqInfo[area];
       return seq.m_interNameLsaSeq;
     }

     void
     setInterNameLsaSeq(uint64_t area, uint64_t num)
     {
       m_seqInfo[area].m_interNameLsaSeq = num;
     }
       uint64_t
     getNetExternalNameLsaSeq(uint64_t area)
     {
       return m_seqInfo[area].m_netExternalNameLsaSeq;
     }

     void
     setNetExternalNameLsaSeq(uint64_t area, uint64_t nlsn)
     {
       m_seqInfo[area].m_netExternalNameLsaSeq = nlsn;
     }

       uint64_t
     getExternalNameLsaSeq(uint64_t area)
     {
       return m_seqInfo[area].m_externalNameLsaSeq;
     }

     void
     setExternalNameLsaSeq(uint64_t area, uint64_t nlsn)
     {
       m_seqInfo[area].m_externalNameLsaSeq = nlsn;
     }

  uint64_t
  getNameLsaSeq(uint64_t area) const
  {
	   const auto &it = m_seqInfo.find(area);
	   
    return it->second.m_nameLsaSeq;
  }

  void
  setNameLsaSeq(uint64_t area, uint64_t nlsn)
  {
    m_seqInfo[area].m_nameLsaSeq = nlsn;
  }

  uint64_t
  getAdjLsaSeq(uint64_t area) const
  {
	   const auto &it = m_seqInfo.find(area);
    return it->second.m_adjLsaSeq;
  }

  void
  setAdjLsaSeq(uint64_t area, uint64_t alsn)
  {
    m_seqInfo[area].m_adjLsaSeq = alsn;
  }

  uint64_t
  getCorLsaSeq(uint64_t area) const
  {
	   const auto &it = m_seqInfo.find(area);
    return it->second.m_corLsaSeq;
  }

  void
  setCorLsaSeq(uint64_t area, uint64_t clsn)
  {
    m_seqInfo[area].m_corLsaSeq = clsn;
  }

  void
  increaseInterNameLsaSeq(uint64_t area)
  {
    m_seqInfo[area].m_interNameLsaSeq++;
  }
  void
  increaseExternalNameLsaSeq(uint64_t area)
  {
    m_seqInfo[area].m_externalNameLsaSeq++;
  }
  void
  increaseNetExternalNameLsaSeq(uint64_t area)
  {
    m_seqInfo[area].m_netExternalNameLsaSeq++;
  }

  void
  increaseNameLsaSeq(uint64_t area)
  {
    m_seqInfo[area].m_nameLsaSeq++;
  }

  void
  increaseAdjLsaSeq(uint64_t area)
  {
    m_seqInfo[area].m_adjLsaSeq++;
  }

  void
  increaseCorLsaSeq(uint64_t area)
  {
    m_seqInfo[area].m_corLsaSeq++;
  }

  void
  writeSeqNoToFile(uint64_t) const;

  void
  initiateSeqNoFromFile(uint64_t);
PUBLIC_WITH_TESTS_ELSE_PRIVATE:

private:
  /*! \brief Set the sequence file directory

    If the string is empty, home directory is set as sequence file directory

  \param filePath The directory where sequence file will be stored
 */
  void
  setSeqFileDirectory(const std::string& filePath);

  void
  writeLog(uint64_t) const;

private:
  std::string m_seqFileNameWithPath;
  struct SeqInfo{
	  uint64_t m_nameLsaSeq;
	  uint64_t m_interNameLsaSeq;
	  uint64_t m_externalNameLsaSeq;
	  uint64_t m_netExternalNameLsaSeq;
	  uint64_t m_adjLsaSeq;
	  uint64_t m_corLsaSeq;
	  std::string m_seqFileNameWithPath;
  };
	std::map<uint64_t, SeqInfo> m_seqInfo;

PUBLIC_WITH_TESTS_ELSE_PRIVATE:
  int m_hyperbolicState;
};

} // namespace nlsr
#endif // NLSR_SEQUENCING_MANAGER_HPP
