#!/bin/sh
cd ../../
./kslicer "apps/20_updated_cfd_solver/src/test_class.cpp" -mainClass Solver -stdlibfolder TINYSTL -pattern ipv -Iapps/LiteMath IncludeToShaders -shaderCC GLSL -DKERNEL_SLICER -v -ITINYSTL ignorecc
cd apps/20_updated_cfd_solver
rm -r shaders_generated
mkdir shaders_generated
mv src/shaders_generated/* shaders_generated/
mv src/include/Solver_ubo.h include/
cd shaders_generated
bash build.sh
cd ../
rm CMAkeCache.txt
cmake .
make clean
make -j 10