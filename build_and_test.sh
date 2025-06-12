#!/bin/bash

cmake -S . -B build/
cd build
make
./test_lib --verbose