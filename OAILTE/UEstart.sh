#!/bin/bash

source oaienv
source ./targets/bin/init_nas_nos1 UE
cd cmake_targets
sudo -E ./lte_noS1_build_oai/build/lte-softmodem-nos1  -U -C2660000000 -r50 --ue-scan-carrier --ue-txgain 90 --ue-rxgain 115 >&1 | tee UE.log
