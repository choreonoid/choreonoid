#!/bin/sh

export COLUMNS=80
curl -LO https://raw.githubusercontent.com/GiovanniBussi/macports-ci/master/macports-ci
chmod +x ./macports-ci
./macports-ci install
PATH="/opt/local/bin:$PATH"
./misc/script/install-requisities-macports.sh
cmake .
make
sudo make install
