#!/bin/bash

# Bricht das Skript ab, wenn ein Befehl fehlschlägt (z.B. wenn make Fehler wirft)
set -e

rm -rf build

echo "Starte Code"

# -p verhindert den Fehler, wenn 'build' schon da ist
mkdir -p build
cd build

cmake ..
make

# Führe das Programm nur aus, wenn es erfolgreich gebaut wurde
./driver_cpp_node

echo "Fertig!"