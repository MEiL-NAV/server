FROM ubuntu:22.04

ADD . /code

RUN apt-get update && \
    apt-get install -y gcc g++ make cmake  \
    build-essential curl autoconf automake \
    libtool pkg-config libsodium-dev git wget \
    libx11-dev software-properties-common && \
    apt-get update

#C++
RUN apt-get install -y libeigen3-dev
RUN apt-get install -y libzmq3-dev
RUN apt-get install -y libgtest-dev
RUN apt-get install -y libyaml-cpp-dev
RUN apt-get install -y libboost-all-dev
RUN apt-get install -y libmodbus-dev


RUN wget https://github.com/zeromq/cppzmq/archive/refs/tags/v4.10.0.tar.gz && \
    tar -xvf v4.10.0.tar.gz && cd cppzmq-4.10.0 && mkdir build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release .. && make install && cd ../..

WORKDIR /code

RUN ./build.sh

WORKDIR /code/build

ENTRYPOINT ["/code/build/server"]