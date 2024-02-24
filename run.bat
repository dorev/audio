@echo off
pushd .
cd ./tests
cmake -B ./build && cmake --build ./build && .\build\Debug\tests.exe
popd