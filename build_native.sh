#!/usr/bin/env bash
set -e

cd "$( dirname "${BASH_SOURCE[0]}" )"

TOPDIR=${PWD}

echo "${TOPDIR}"

KERNELRELEASE=$(uname -r)

echo "${KERNELRELEASE}"

# module name should not have underscore
PKGNAME="mira-driver"
# release version
PKGVER="0.1.5"
# arm64 for 64bit or armhf for 32bit
ARCH="arm64"

PKGDIR=$TOPDIR/$PKGNAME"_"$PKGVER"-1_"$ARCH

echo "$PKGDIR"

mkdir -p $PKGDIR/DEBIAN

echo "Package: $PKGNAME
Version: $PKGVER
Architecture: $ARCH
Maintainer: Zhenyu Ye <zhenyu.ye@ams-osram.com>
Description: Mira driver for RPI.
 It contains mira220, mira220color, mira050, mira050color." > $PKGDIR/DEBIAN/control

MODULEDIR=$PKGDIR/usr/lib/modules/$KERNELRELEASE/kernel/drivers/media/i2c
mkdir -p $MODULEDIR

(cd mira220/src && make)
(cd mira220/src && make INSTALL_MOD_PATH=$PKGDIR install)

(cd mira050/src && make)
(cd mira050/src && make INSTALL_MOD_PATH=$PKGDIR install)


dpkg-deb --build --root-owner-group $PKGDIR

sudo dpkg -i $PKGDIR.deb
