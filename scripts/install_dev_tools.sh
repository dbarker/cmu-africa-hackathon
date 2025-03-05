#!/bin/bash
set -e
apt-get update
apt-get install -y nano gitk cmake-gui libtool pkg-config gdb gdbserver wget libgmock-dev python3 python3-pip python3-venv clang-format
