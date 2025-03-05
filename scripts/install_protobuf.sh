#!/bin/bash
set -e

if [ -z "$1" ]; then
    echo "Usage: $0 <protobuf_version>"
    exit 1
fi

PROTOBUF_VERSION="$1"

SRC_DIR=$(pwd)

mkdir protobuf && cd protobuf

wget https://github.com/protocolbuffers/protobuf/archive/refs/tags/v${PROTOBUF_VERSION}.tar.gz -O protobuf.tar.gz
tar -xzf protobuf.tar.gz --one-top-level=protobuf --strip-components=1

cd protobuf

mkdir build && cd build 
cmake -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/usr/local \
      -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
      -Dprotobuf_BUILD_TESTS=OFF \
      -Dprotobuf_BUILD_EXAMPLES=OFF \
      ..
make -j$(nproc) && make install && ldconfig

rm -rf $SRC_DIR/protobuf

echo "Protobuf ${PROTOBUF_VERSION} has been successfully built and installed."