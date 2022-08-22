#!/bin/bash
echo

mkdir build && cd build
cmake .. && make 
./path_planning