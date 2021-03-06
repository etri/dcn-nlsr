# MA-NLSR: MultiArea-Named Data Link State Routing Protocol for DCN(Data Centric Networking)

# Overview

MA-NLSR is an intra-domain routing protocol supporting multi-area for Named Data Networking(NDN).  
MA-NLSR based on NLSR is an application level protocol simular to many IP routing protocol,  
but MA-NLSR uses NDN's Interest/Data packets to dissemminate rotuing updates inside a single Area.  
NLSR is the first intra-domin routing protocol in most NDN researches, but doesn't support multi-Area concept.  
We extend NLSR to have Multi-Area which keeps the NLSR's design patterns.

MA-NLSR is designed to accomplish three key tasks:
- discover adjacent neighbors in specific area
- disseminate and synchronize topology, name prefix in specific area
- calculate a routing table and populate NFD's FIB in specific area

# MA-NLSR Architecture  
MA-NLSR modifies the following modules to support "multiple areas" like OSPF.  
The LSA format is show below.  
Each LSA has the name \/\<network\>\/NLSR\/\<Area-ID\>\/LSA\/\<site\>\/\<router\>\/\<lsa-type\>\/\<version\>,  
where \<lsa-type\> can be NAME, ADJACENCY, IA-NAME, Ext-NAME, Ext-Net-NAME.  

**IA-NAME, Ext-NAME, Ext-Net-NAME** are a newly added LSAs to support multi-area.  
* NAME Lsa: to carry Name Prefixes within the area.  
* ADJACENCY Lsa: to carry Adjacency Prefiexes within the area.  
* IA-NAME Lsa: to carry Name Prefiexes to which another Area belongs.  
* Ext-NAME Lsa: to advertise the Name prefixes originated static configuration by an operator.  
* Ext-Net-NAME Lsa: to redistribute the Name prefixes from EGP like BGP.  

The **\<Area-ID\>** component identifies the Area to which the router belongs.

<!--
## Hello Protocol - 수정된 부분 설명 추가.  
## PartialSync - .
## Sync Logic Hnadler -.
## LSAs -.
## LSDB - 
## Routing table 
## Name Prefix table 
## FIB 
## Prefix Update Processor   
-->

Below figure shows the list of Tables that were modified (yellow) to support Multi-Area.  
<img src="/images/ma-2.GIF" width="50%" height="%40">

The key design goals of MA-NLSR is to provide the following principles:  
- internet scalibility via multi-area
- compatibility with NFD and existing NLSR, and  
- maintaining NLSR's routing architecture to inherit its advantages of midularity and extensibility.  

# Build
Execute the following commands to build PSync:  
$ cd PSync  
$ ./waf configure  
$ ./waf  
$ sudo ./waf install

Execute the following commands to build MA-NLSR:  
$ ./waf configure  
$ ./waf  
$ sudo ./waf install  
# Configuration
After installing MA-NLSR from source, you need to create a configuration file for MA-NLSR. Please take a look at [nlsr.conf](/nlsr.conf) for a sample configuration.  
# Testing MA-NLSR
Please refer to [Configuration](/conf)
# Running  
Run dcn-nlsr with the follwing command:  
$ nlsr  

# Version (MA-NLSR 0.7.1)
- Based on NLSR 0.7.1 & ndn-cxx 0.7.1
 - Added Features :
      * Anycast Name Prefix
      * External Name LSA for Static by operator
      * Inter-Area Name LSA among Areas
      * External NetName LSA for BGP Redistribution

 - Not Supported Yet :
      * Stub Area
      * Mac OS Platform

# Supported platforms
MA-NLSRD has been tested on the following platforms:

- Ubuntu 18.04 (amd64, armhf)

Mac OS shall be supported in the future releases.

## Credits  
MA-NLSR is designed and developed by:   

- Sung Hyuk Byun (shbyun@etri.re.kr)
- Jong Seok Lee (viper@etri.re.kr) 


This work is one of research results of the project "Hyper-connected Intelligent Infrastructure Technology Development" conducted by ETRI, Korea. The  project leaders are:  

- Namseok Ko (nsko@etri.re.kr)
- Sun Me Kim (kimsunme@etri.re.kr) 
