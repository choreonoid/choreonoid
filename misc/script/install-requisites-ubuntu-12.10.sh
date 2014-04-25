#!/bin/sh

sudo apt-get install \
build-essential \
cmake-curses-gui \
libboost-all-dev \
libqt4-dev \
libqt4-opengl-dev \
qt4-dev-tools \
qt4-qtconfig \
qt4-doc-html \
libglew1.6-dev \
libyaml-dev \
gettext \
zlib1g-dev \
libjpeg62-dev \
libpng12-dev \
libode-dev \
libomniorb4-dev \
libcos4-dev \
omniidl \
omniorb-nameserver \
libgstreamer0.10-dev \
libgstreamer-plugins-base0.10-dev \
libpulse-dev \
libsndfile1-dev \
uuid-dev # for compiling OpenRTM-aist

# The eigen pakcage of Ubuntu 12.10 has a bug.
# You should install the latest version manually
# libeigen3-dev \
# libeigen3-doc \
