#!/bin/bash
echo "Starte Code"

mkdir build
cd build/
cmake ..
make
./driver_cpp_node

echo "Fertig!"