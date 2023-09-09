#!/bin/bash
if [[ -z $1 ]]; then
mkdir -p build
cd build
cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=1 ..
make -j12
elif [[ "$1" == "install" ]]; then
mkdir -p build
cd build
cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=1 ..
make install -j12
elif [[ "$1" == "debug" ]]; then
mkdir -p debug
cd debug
cmake -DCMAKE_BUILD_TYPE=Debug ..
make
elif [[ "$1" == "project" ]]; then
mkdir -p build
cd build
cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=1 ..
elif [[ "$1" == "test" ]]; then
ctest --output-on-failure --test-dir build --build-run-dir .. --build-exe-dir ..
fi
