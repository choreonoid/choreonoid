FROM ubuntu:artful

RUN apt-get update && \
    apt-get install -y sudo && \
    apt-get clean

ADD . /choreonoid

RUN cd /choreonoid && \
    echo "y" | ./misc/script/install-requisites-ubuntu-17.10.sh && \
    cmake . && \
    make && \
    make install
