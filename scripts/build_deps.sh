#!/bin/sh
set -e -x

# get,build,install c-sdk from edgexfoundry
if [ -d deps ]
then
  mkdir -p deps
  cd deps
  wget https://github.com/edgexfoundry/device-sdk-c/archive/0.7.1.tar.gz
  tar -xzf 0.7.1.tar.gz
  cd device-sdk-c-0.7.1
  ./scripts/build.sh
  cp -rf include/* /usr/include/
  cp build/release/c/libcsdk.so /usr/lib/
fi
