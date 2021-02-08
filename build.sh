#! /bin/bash

build="tmp"
workPath="/nfs"
#installword='install/strip'
installword='install'

mkdir ${build}
cd ${build}
cmake .. -DCMAKE_INSTALL_PREFIX=${workPath} -DCMAKE_TOOLCHAIN_FILE=../compiler.himi200.cmake
cpunum=$(cat /proc/cpuinfo | grep processor | wc -l)
make -j${cpunum}
make $installword


