#!/bin/sh
rm -r shaders_generated
mkdir shaders_generated
mv src/shaders_generated/* shaders_generated/
mv src/include/Solver_ubo.h include/
cd shaders_generated
bash build.sh
cd ../
make -j 10