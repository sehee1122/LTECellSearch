#!/bin/bash

source oaienv
source ./cmake_targets/tools/init_nas_nos1 eNB
cd cmake_targets
sudo -E ./lte_noS1_build_oai/build/lte-softmodem-nos1 -O $OPENAIR_TARGETS/PROJECTS/GENERIC-LTE-EPC/CONF/enb.band7.tm1.usrpb210.conf 2>&1 | tee ENB.log
