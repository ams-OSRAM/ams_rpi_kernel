#!/usr/bin/env bash
set -e

cd "$( dirname "${BASH_SOURCE[0]}" )"

TOPDIR=${PWD}

echo "Building from DIR: ${TOPDIR}"

KERNELRELEASE=$(uname -r)

echo "Building for kernel version: ${KERNELRELEASE}"

if [ ! -d "/usr/src/linux-headers-${KERNELRELEASE}" ]
then
	echo "Kernel header /usr/src/linux-headers-${KERNELRELEASE} not found."
	echo "Try of obtain it via apt."
	sudo apt-get install raspberrypi-kernel-headers
	if [ ! -d "/usr/src/linux-headers-${KERNELRELEASE}" ]
	then
		echo "Failed to install kernel header. Exiting."
		exit 1
	fi
fi

echo "#############################"
echo "# Creating deb pkg folder"
echo "#############################"

# module name should not have underscore
PKGNAME="mira-driver"
# release version
PKGVER="0.1.5"
# arm64 for 64bit or armhf for 32bit
ARCH="arm64"

PKGDIR=$TOPDIR/$PKGNAME"_"$PKGVER"-1_"$ARCH

echo "Creating deb package at $PKGDIR"

mkdir -p $PKGDIR/DEBIAN

echo "Package: $PKGNAME
Version: $PKGVER
Architecture: $ARCH
Maintainer: Zhenyu Ye <zhenyu.ye@ams-osram.com>
Description: Mira driver for RPI.
 It contains mira220, mira220color, mira050, mira050color." > $PKGDIR/DEBIAN/control

echo "#############################"
echo "# Add device tree to deb"
echo "#############################"

mkdir -p $PKGDIR/boot/overlays

for dtname in "mira220" "mira220color" "mira050" "mira050color"; do
	dtdir=$(echo $dtname | sed 's/color//')
	echo "Building device tree source $dtdir/src/$dtname.dts"
	(cd $dtdir/src && cpp -nostdinc -I include -I arch -I/usr/src/linux-headers-${KERNELRELEASE}/include/ -undef -x assembler-with-cpp  $dtname-overlay.dts $dtname-overlay.dts.preprocessed)
	(cd $dtdir/src && dtc -I dts -O dtb $dtname-overlay.dts.preprocessed -o $dtname.dtbo)
	(cd $dtdir/src && rm $dtname-overlay.dts.preprocessed)
	echo "Copying device tree blob overlay $dtdir/src/$dtname.dtbo to $PKGDIR/boot/overlays/"
	sudo cp $dtdir/src/$dtname.dtbo $PKGDIR/boot/overlays/
done

echo "#############################"
echo "# Add driver module to deb"
echo "#############################"

MODULEDIR=$PKGDIR/usr/lib/modules/$KERNELRELEASE/kernel/drivers/media/i2c
mkdir -p $MODULEDIR

(cd mira220/src && make)
(cd mira220/src && make INSTALL_MOD_PATH=$PKGDIR install)
(cd mira220/src && make clean)

(cd mira050/src && make)
(cd mira050/src && make INSTALL_MOD_PATH=$PKGDIR install)
(cd mira050/src && make clean)

echo "#############################"
echo "# Build and install deb pkg"
echo "#############################"

echo "Building $PKGDIR.deb"
dpkg-deb --build --root-owner-group $PKGDIR

echo "Backing up mira dtbo in /boot/overlays"
for file in `ls /boot/overlays/mira*.dtbo`; do
	echo "Moving ${file} to ${file}bak"
	sudo mv ${file} ${file}bak
done

echo "Installing $PKGDIR.deb"
sudo dpkg -i $PKGDIR.deb

echo "Installed to /usr/lib/modules/$KERNELRELEASE/kernel/drivers/media/i2c"

echo "Post installation: rebuild dependency modules to make modules loaded by kernel"
sudo depmod

