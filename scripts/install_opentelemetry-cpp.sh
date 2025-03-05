#!/bin/bash
set -e
if [ -z "$1" ]; then
    echo "Usage: $0 <otel-cpp-branch>"
    exit 1
fi

OPENTELEMETRY_CPP_BRANCH="$1"

SRC_DIR=$(pwd)

git clone --depth 1 --branch ${OPENTELEMETRY_CPP_BRANCH} https://github.com/open-telemetry/opentelemetry-cpp.git opentelemetry-cpp

cd opentelemetry-cpp

git submodule update --init --recursive --depth 1 

mkdir build && cd build 

cmake   -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
        -DBUILD_SHARED_LIBS=ON \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DWITH_ABI_VERSION_2=ON \
        -DWITH_ABI_VERSION_1=OFF \
        -DWITH_OTLP_GRPC=ON \
        -DWITH_OTLP_RETRY_PREVIEW=ON \
        -DWITH_OTLP_GRPC_SSL_MTLS_PREVIEW=ON \
        -DWITH_OTLP_FILE=ON \
        -DWITH_ASYNC_EXPORT_PREVIEW=ON \
        -DWITH_METRICS_EXEMPLAR_PREVIEW=ON \
        -DWITH_THREAD_INSTRUMENTATION_PREVIEW=ON \
        -DWITH_EXAMPLES=OFF \
        -DWITH_BENCHMARK=OFF \
        -DBUILD_TESTING=OFF \
        -DOPENTELEMETRY_INSTALL=ON .. 

make -j$(nproc) && make install && ldconfig

rm -rf $SRC_DIR/opentelemetry-cpp

echo "opentelemetry-cpp version ${OPENTELEMETRY_CPP_BRANCH} has been successfully built and installed."