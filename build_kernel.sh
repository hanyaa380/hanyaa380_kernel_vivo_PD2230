#!/bin/bash
PATH=`pwd`/aarch64-linux-android-4.9/bin:$PATH
rm -rf out
mkdir -p out
make -j32 O=out ARCH=arm64 CROSS_COMPILE=aarch64-linux-android- k6833v1_64_k419_defconfig
make -j32 O=out ARCH=arm64 CROSS_COMPILE=aarch64-linux-android-
