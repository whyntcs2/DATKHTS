#!/usr/bin/env bash
set -eu

export PATH=/mingw64/bin:/usr/bin:$PATH
cd "$(dirname "$0")/c_code"
make clean
make
