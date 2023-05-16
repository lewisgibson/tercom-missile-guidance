#!/usr/bin/env bash
cmake \
    -B build \
    -Wdev #\
#-DBUILD_EXAMPLES=ON

cmake \
    --build build \
    --parallel $(nproc)
