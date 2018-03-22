FROM ubuntu:trusty

RUN apt-get update && \
    apt-get install -y sudo software-properties-common && \
    add-apt-repository http://openrtm.org/pub/Linux/ubuntu/ && \
    apt-get update && \
    apt-get install -y --force-yes \
        gcc g++ \
        make uuid-dev libboost-filesystem-dev \
        libomniorb4-dev omniidl \
        openrtm-aist openrtm-aist-doc \
        openrtm-aist-dev openrtm-aist-example \
        python-yaml && \
    apt-get clean

ADD . /choreonoid

RUN cd /choreonoid && \
    echo "y" | ./misc/script/install-requisites-ubuntu-14.04.sh && \
    cmake . \
        -DCMAKE_BUILD_TYPE=Release       \
        -DINSTALL_SDK:BOOL=TRUE          \
        -DINSTALL_DEPENDENCIES:BOOL=TRUE \
        -DCNOID_ENABLE_GETTEXT:BOOL=TRUE \
        -DUSE_EXTERNAL_EIGEN:BOOL=TRUE   \
        -DUSE_EXTERNAL_YAML:BOOL=TRUE    \
        -DENABLE_CORBA:BOOL=TRUE         \
        -DBUILD_CORBA_PLUGIN:BOOL=TRUE   \
        -DBUILD_OPENRTM_PLUGIN:BOOL=TRUE \
        -DBUILD_GROBOT_PLUGIN:BOOL=TRUE  \
        -DBUILD_ASSIMP_PLUGIN:BOOL=FALSE \
        -DBUILD_PYTHON_PLUGIN:BOOL=TRUE && \
    make && \
    make install
