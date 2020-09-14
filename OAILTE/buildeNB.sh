#!/bin/bash

source oaienv
cd cmake_targets
sudo ./build_oai -w USRP --eNB --noS1
