#!/bin/sh

sudo --preserve-env=DEBIAN_FRONTEND,TZ \
apt-get -y install \
build-essential \
cmake-curses-gui \
libboost-dev \
libboost-system-dev \
libboost-program-options-dev \
libboost-iostreams-dev \
libboost-filesystem-dev \
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
qt5-default \
libqt5x11extras5-dev \
libqt5svg5-dev \
qttranslations5-l10n \
python3-dev \
python3-numpy \
libassimp-dev \
libode-dev \
libfcl-dev \
libpulse-dev \
libsndfile1-dev \
libgstreamer1.0-dev \
libgstreamer-plugins-base1.0-dev
