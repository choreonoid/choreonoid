#!/bin/sh

sudo --preserve-env=DEBIAN_FRONTEND,TZ \
apt-get -y install \
build-essential \
cmake-curses-gui \
libeigen3-dev \
uuid-dev \
libxfixes-dev \
libyaml-dev \
libfmt-dev \
gettext \
zlib1g-dev \
libzip-dev \
libjpeg-dev \
libpng-dev \
libfreetype-dev \
qt6-base-dev \
qt6-svg-dev \
qt6-translations-l10n \
libglu1-mesa-dev \
libegl-dev \
python3-dev \
python3-numpy \
libassimp-dev \
libode-dev \
libfcl-dev \
libpulse-dev \
libsndfile1-dev \
libgstreamer1.0-dev \
libgstreamer-plugins-base1.0-dev
