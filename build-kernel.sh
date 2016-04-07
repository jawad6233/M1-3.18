#!/bin/bash
#Stop script if something is broken
set -e

export ARCH=arm64
export CROSS_COMPILE="ccache /mnt/1TB/android/M1/kernel/stock/toolchain/aarch64-linux-android-4.9/bin/aarch64-linux-android-"
export HW_PROJECT="M1_K109_DT600"

#Echo actual vars
echo "We are actually building for $HW_PROJECT with $CROSS_COMPILE"
make runbo_m1_defconfig
#make clean
#export MAKEFLAGS=-j9
make -j9
#./build.sh