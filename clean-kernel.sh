#!/bin/bash
#Stop script if something is broken
set -e

export ARCH=arm64
export CROSS_COMPILE="ccache /mnt/1TB/android/M1/kernel/stock/toolchain/aarch64-linux-android-4.9/bin/aarch64-linux-android-"
export HW_PROJECT="M1_K109_DT600"

#Echo actual vars
echo "We are actually clean for $HW_PROJECT with $CROSS_COMPILE"
make len6735_66_m0_M1_K109_DT600_defconfig
make clean

