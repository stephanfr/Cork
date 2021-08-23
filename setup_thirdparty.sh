#!/bin/bash

sudo apt-get install -y m4
sudo apt-get install -y yasm

mkdir thirdparty
cd thirdparty

mkdir tbb
cd tbb
wget --timestamping https://github.com/oneapi-src/oneTBB/releases/download/v2021.3.0/oneapi-tbb-2021.3.0-lin.tgz
tar zxvf oneapi-tbb-2021.3.0-lin.tgz
sudo cp ./oneapi-tbb-2021.3.0/lib/intel64/gcc4.8/* /usr/local/lib/.

cd ..

mkdir mpir
cd mpir
wget --timestamping https://mpir.org/mpir-3.0.0.tar.bz2
tar -xf mpir-3.0.0.tar.bz2
cd mpir-3.0.0
sed -i 's/GLOBAL_FUNC\[:space:\]/GLOBAL_FUNC\[\[:space:\]\]/' configure
./configure
make
make check
sudo make install

cd ../..

mkdir boost
cd boost
wget --timestamping https://boostorg.jfrog.io/artifactory/main/release/1.77.0/source/boost_1_77_0.tar.bz2
tar --bzip2 -xf boost_1_77_0.tar.bz2
cd boost_1_77_0
./bootstrap.sh
sudo ./b2 install
