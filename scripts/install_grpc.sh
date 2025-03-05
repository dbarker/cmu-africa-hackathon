#!/bin/bash
set -e

if [ -z "$1" ]; then
    echo "Usage: $0 <grpc_version>"
    exit 1
fi

GRPC_VERSION="$1"

SRC_DIR=$(pwd)

echo "Installing gRPC version ${GRPC_VERSION}"

git clone --depth 1 --branch ${GRPC_VERSION} https://github.com/grpc/grpc.git grpc

cd grpc

git submodule update --init --recursive --depth 1

mkdir -p cmake/build && cd cmake/build

cmake -DgRPC_INSTALL=ON \
      -DgRPC_BUILD_TESTS=OFF \
      -DCMAKE_BUILD_TYPE=Release \
      -DgRPC_PROTOBUF_PROVIDER=package \
      -DCMAKE_INSTALL_PREFIX=/usr/local ../.. 

make -j$(nproc) && make install && ldconfig

rm -rf $SRC_DIR/grpc