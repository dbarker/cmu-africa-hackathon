#!/bin/bash
set -e  

if [ -z "$3" ]; then
    echo "Usage: $0 OPENTELEMETRY_PYTHON_VERSION WHEELHOUSE_DIR BUILD_FROM_SOURCE"
    exit 1
fi

OPENTELEMETRY_PYTHON_VERSION=$1
WHEELHOUSE_DIR=$2
MODE=$3

SRC_DIR=$(pwd)

PIP_VERSION=$(pip --version | awk '{print $2}' | cut -d. -f1,2)

if awk -v ver="$PIP_VERSION" 'BEGIN { if (ver >= 23.0) exit 0; else exit 1 }'; then
    PIP_FLAG="--break-system-packages --no-cache-dir --ignore-installed"
else
    PIP_FLAG=""
fi

echo "Detected pip version: $PIP_VERSION"
echo "Using pip flag: '$PIP_FLAG'"

if [ "$MODE" == "BUILD_AND_INSTALL" ]; then

    echo "Installing OpenTelemetry Python version: $1. Outputting wheels to $2"
    git clone --depth 1 --branch ${OPENTELEMETRY_PYTHON_VERSION} https://github.com/open-telemetry/opentelemetry-python.git opentelemetry-python
    cd opentelemetry-python
    pip install --no-cache-dir build wheel $PIP_FLAG
    MODULES=(
        "opentelemetry-api"
        "opentelemetry-sdk"
        "opentelemetry-semantic-conventions"
        "opentelemetry-proto"
        "exporter/opentelemetry-exporter-otlp-proto-common"
        "exporter/opentelemetry-exporter-otlp-proto-grpc"
    )
    for MODULE in "${MODULES[@]}"; do
        echo "Building wheel for ${MODULE}"
        (cd ${SRC_DIR}/opentelemetry-python/${MODULE} && python3 -m build --wheel --outdir ${WHEELHOUSE_DIR})
    done
    cd ${SRC_DIR}
    rm -rf $SRC_DIR/opentelemetry-python
fi

pip install --no-cache-dir ${WHEELHOUSE_DIR}/*.whl $PIP_FLAG

echo "OpenTelemetry Python installation complete."