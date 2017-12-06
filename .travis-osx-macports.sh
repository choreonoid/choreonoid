#!/bin/bash
set -ev

export COLUMNS=80
curl -LO https://raw.githubusercontent.com/GiovanniBussi/macports-ci/master/macports-ci
chmod +x ./macports-ci
./macports-ci install
PATH="/opt/local/bin:$PATH"
./misc/script/install-requisites-macports.sh
sudo port select --set python python36
cmake .
make
sudo make install
