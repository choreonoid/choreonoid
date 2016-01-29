#!/bin/bash

set -x

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu precise main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -

sudo add-apt-repository -y ppa:hrg/daily
sudo apt-get update -qq
sudo apt-get install -qq -y libcnoid-dev libsdformat-dev

cmake -DBUILD_SDFLOADER_PLUGIN=ON .
make


