# RPMPLv2

Tested for Ubuntu 20.04:


```
git clone repo_url
git submodule update --init --recursive
sudo apt install libeigen3-dev libkdl-parser-dev libgflags-dev libgoogle-glog-dev liborocos-kdl-dev libyaml-cpp-dev liburdf-dev

mkdir build
cd build
cmake ..
make
```
