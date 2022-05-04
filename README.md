# DCN-NLSR: Named Data Link State Routing Protocol for DCN(Data Centric Networking)

# Overview

DCN-NLSR is an intra-domain routing protocol supporting multi-area for Named Data Networking(NDN).  
DCN-NLSR based on NLSR is an application level protocol simular to many IP routing protocol,  
but DCN-NLSR uses NDN's Interest/Data packets to dissemminate rotuing updates inside a single Area.  
NLSR is the first intra-domin routing protocol in most NDN researches, but doesn't support multi-Area concept.  
We extend NLSR to have Multi-Area which keeps the NLSR's design patterns.

DCN-NLSR is designed to accomplish three key tasks:
- discover adjacent neighbors in specific area
- disseminate and synchronize topology, name prefix in specific area
- calculate a routing table and populate NFD's FIB in specific area

DCN-NLSR uses the following modules to support Multi-Area(LSA 비교 추가)
- Hello Protocol - determines the status of neighboring routers using periodic Hello Interests and notifies
other modules when neighbors' statuses change.
- PartialSync - provides network-wide synchronization of DCN-NLSR LSDBs.
- Sync Logic Hnadler - handles sync update notications from NSync by retrieving updated LSAs.
- LSAs - represent routing information published by the router.
- LSDB - stores the LSA information distributed by other routers in the network.
- Routing table - calculates and maintains a list of next hops for each router in the network.
- Name Prefix table - stores all advertised name prexes and their next hops.
- FIB - maintains a shadow FIB which represents the intended state of NFD's FIB
- Prefix Update Processor - listens for dynamic prex announcements to advertise or withdraw name
prefixes.
새로운 섹션 추가
새로 추가된 LSA에 대한 이름과 naming/기능설명 추가
Below figure shows the list of Tables that were modified (yellow) to support Multi-Area.  
<img src="/images/ma-2.GIF" width="50%" height="%40">

The key design goals of DCN-NLSR is to provide the following principles:  
- internet scalibility via multi-area
- compatibility with NFD and existing NLSR, and  
- maintaining NLSR's routing architecture to inherit its advantages of midularity and extensibility.  

# Build
Execute the following commands to build PSync:  
cd PSync  
./waf configure  
./waf  
sudo ./waf install

Execute the following commands to build DCN-NLSR:  
./waf configure  
./waf  
sudo ./waf install  
# Configuration
After installing NLSR from source, you need to create a configuration file for NLSR. Please take a look at [nlsr.conf](/nlsr.conf) for a sample configuration.  
# Sample Topology Configuration
Please refer to [topology](/conf)
# Running  
Run dcn-nlsr with the follwing command:  
nlsr  

# Version (DCN-NLSR 0.7.1)
- Based on NLSR 0.7.1 & ndn-cxx 0.7.1
 - Added Features :
      * Anycast Name Prefix
      * External NetName LSA for Static Redistribution
      * Inter-Area Name LSA
      * BGP Redistribution

 - Not Supported Yet :
      * Stub Area
      * Mac OS Platform

# Supported platforms
DCN-NLSRD has been tested on the following platforms:

- Ubuntu 18.04 (amd64, armhf)

Mac OS shall be supported in the future releases.

## Credits  
DCN-NLSR is designed and developed by:   

- Sung Hyuk Byun (shbyun@etri.re.kr)
- Jong Seok Lee (viper@etri.re.kr) 


This work is one of research results of the project "Hyper-connected Intelligent Infrastructure Technology Development" conducted by ETRI, Korea. The  project leaders are:  

- Namseok Ko (nsko@etri.re.kr)
- Sun Me Kim (kimsunme@etri.re.kr) 
