FROM ubuntu:artful

ENV CLANG_VERSION 4.0

# we have to install llvm-dev package as well due to -flto option
# see: https://bugs.launchpad.net/ubuntu/+source/llvm-toolchain-snapshot/+bug/1254970
RUN apt-get update && \
    apt-get install -y sudo clang-${CLANG_VERSION} llvm-${CLANG_VERSION}-dev && \
    apt-get clean

ENV CC /usr/bin/clang-${CLANG_VERSION}
ENV CXX /usr/bin/clang++-${CLANG_VERSION}

ADD . /choreonoid

RUN cd /choreonoid && \
    echo "y" | ./misc/script/install-requisites-ubuntu-17.10.sh && \
    cmake . && \
    make && \
    make install
