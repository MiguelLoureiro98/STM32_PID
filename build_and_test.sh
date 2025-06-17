#!/bin/bash

build_type=${1:-"Debug"}

if ([ $build_type != "Debug" ] && [ $build_type != "Release" ])
then

    echo "Error: $build_type unrecognised. Please select either Debug or Release as the build type."
    exit 1

fi

echo "Build type: $build_type"

cmake -DCMAKE_BUILD_TYPE=$build_type -S . -B build/
cd build
make
cd tests
./test_lib --verbose