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

#include "sequencing-manager.hpp"
#include "logger.hpp"

#include <iostream>
#include <string>
#include <fstream>
#include <pwd.h>
#include <cstdlib>
#include <unistd.h>

namespace nlsr {

INIT_LOGGER(SequencingManager);

SequencingManager::SequencingManager(const std::string& filePath, int hypState)
  : m_hyperbolicState(hypState)
{
  setSeqFileDirectory(filePath);
  //initiateSeqNoFromFile();
}

void
SequencingManager::writeSeqNoToFile(uint64_t areaId) const
{
  writeLog(areaId);

	auto it = m_seqInfo.find(areaId);
	auto seq =it->second;

  std::ofstream outputFile(seq.m_seqFileNameWithPath.c_str());
  std::ostringstream os;
  os << "NameLsaSeq " << std::to_string(seq.m_nameLsaSeq) << "\n"
     << "AdjLsaSeq "  << std::to_string(seq.m_adjLsaSeq)  << "\n"
     << "CorLsaSeq "  << std::to_string(seq.m_corLsaSeq) << "\n"
     << "InterNameLsaSeq "  << std::to_string(seq.m_interNameLsaSeq)  << "\n"
     << "ExternalNameLsaSeq "  << std::to_string(seq.m_externalNameLsaSeq) << "\n"
     << "NetExternalNameLsaSeq "  << std::to_string(seq.m_netExternalNameLsaSeq) ;
  outputFile << os.str();
  outputFile.close();

}

void
SequencingManager::initiateSeqNoFromFile(uint64_t areaId)
{
	//std::cout << "initiateSeqNoFromFile's Area: " << areaId << std::endl;
	SeqInfo &seq= m_seqInfo.emplace(areaId, SeqInfo()).first->second;
	 
	std::string fileName = "/nlsrSeqNo" + std::to_string(areaId) + ".txt";
  seq.m_seqFileNameWithPath = m_seqFileNameWithPath + fileName;

  NLSR_LOG_DEBUG("Seq File Name: " << seq.m_seqFileNameWithPath);
  std::ifstream inputFile(seq.m_seqFileNameWithPath.c_str());

  std::string seqType;
  // Good checks that file is not (bad or eof or fail)
  if (inputFile.good()) {
    inputFile >> seqType >> seq.m_nameLsaSeq;
    inputFile >> seqType >> seq.m_adjLsaSeq;
    inputFile >> seqType >> seq.m_corLsaSeq;
    inputFile >> seqType >> seq.m_interNameLsaSeq;
    inputFile >> seqType >> seq.m_externalNameLsaSeq;
    inputFile >> seqType >> seq.m_netExternalNameLsaSeq;

    inputFile.close();

    // Increment by 10 in case last run of NLSR was not able to write to file
    // before crashing
    seq.m_nameLsaSeq += 10;

    // added by ETRI(modori) on 20210603
    seq.m_interNameLsaSeq += 10;
    seq.m_externalNameLsaSeq += 10;
    seq.m_netExternalNameLsaSeq += 10;

    // Increment the adjacency LSA seq. no. if link-state or dry HR is enabled
    if (m_hyperbolicState != HYPERBOLIC_STATE_ON) {
      if (seq.m_corLsaSeq != 0) {
        NLSR_LOG_WARN("This router was previously configured for hyperbolic " <<
                      "routing without clearing the seq. no. file.");
        seq.m_corLsaSeq = 0;
      }
      seq.m_adjLsaSeq += 10;
    }

    // Similarly, increment the coordinate LSA seq. no only if link-state is disabled.
    if (m_hyperbolicState != HYPERBOLIC_STATE_OFF) {
      if (seq.m_adjLsaSeq != 0) {
        NLSR_LOG_WARN("This router was previously configured for link-state " <<
                      "routing without clearing the seq. no. file.");
        seq.m_adjLsaSeq = 0;
      }
      seq.m_corLsaSeq += 10;
    }
  }

  writeLog(areaId);
}

void
SequencingManager::setSeqFileDirectory(const std::string& filePath)
{
  m_seqFileNameWithPath = filePath;

  if (m_seqFileNameWithPath.empty()) {
    std::string homeDirPath(getpwuid(getuid())->pw_dir);
    if (homeDirPath.empty()) {
      homeDirPath = getenv("HOME");
    }
    m_seqFileNameWithPath = homeDirPath;
  }
}

void
SequencingManager::writeLog(uint64_t areaId) const
{
	auto it = m_seqInfo.find(areaId);
	auto seq =it->second;
  if (m_hyperbolicState == HYPERBOLIC_STATE_OFF ||
      m_hyperbolicState == HYPERBOLIC_STATE_DRY_RUN) {
    NLSR_LOG_DEBUG("Adj LSA seq no: " << seq.m_adjLsaSeq);
  }
  if (m_hyperbolicState == HYPERBOLIC_STATE_ON ||
      m_hyperbolicState == HYPERBOLIC_STATE_DRY_RUN) {
    NLSR_LOG_DEBUG("Cor LSA Seq no: " << seq.m_corLsaSeq);
  }
  NLSR_LOG_DEBUG("Name LSA Seq no: " << seq.m_nameLsaSeq);
  NLSR_LOG_DEBUG("Inter Name LSA Seq no: " << seq.m_interNameLsaSeq);
  NLSR_LOG_DEBUG("External Name LSA Seq no: " << seq.m_externalNameLsaSeq);
}

} // namespace nlsr
