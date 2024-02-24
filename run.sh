#!/bin/bash
pushd ./tests
cmake -B ./build && cmake --build ./build && ./build/Debug/tests.exe
popd