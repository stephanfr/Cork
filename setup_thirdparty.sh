#!/bin/bash

sudo apt-get install m4

mkdir thirdparty
cd thirdparty

mkdir tbb
cd tbb
wget https://github.com/oneapi-src/oneTBB/releases/download/2017_U8/tbb2017_20170807oss_lin.tgz
tar zxvf tbb2017_20170807oss_lin.tgz

cd ..

mkdir mpir
cd mpir
wget https://mpir.org/mpir-2.7.2.tar.bz2
tar -xf mpir-2.7.2.tar.bz2
cd mpir-2.7.2
sed -i 's/GLOBAL_FUNC\[:space:\]/GLOBAL_FUNC\[\[:space:\]\]/' configure
./configure
make
make install

cd ../..

mkdir boost
cd boost
wget https://boostorg.jfrog.io/artifactory/main/release/1.65.1/source/boost_1_65_1.tar.bz2
tar --bzip2 -xf boost_1_65_1.tar.bz2
cd boost_1_65_1
./bootstrap.sh
./b2 install
