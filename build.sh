#!/usr/bin/env bash
cmake -B build -Wdev
cmake --build build --parallel $(nproc)
