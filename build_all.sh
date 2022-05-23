#!/usr/bin/env bash
set -e

cd "$( dirname "${BASH_SOURCE[0]}" )"

echo "${PWD}"

# checkout kernel source
COMMIT=6dafd553901ea01d8871010488121e21d131fea4
# Check if the unzipped directory of the Linux source exist; Otherwise download it.
if [[ ! -d $PWD/linux-${COMMIT} ]]
then
	echo "Downloading Raspberry Pi Linux repo commit ${COMMIT}"
	wget https://github.com/raspberrypi/linux/archive/${COMMIT}.zip
	unzip ${COMMIT}.zip
fi
ln -sf $PWD/linux-${COMMIT} linux

# apply patches and sources
echo "Applying patches to Linux source"
(cd $PWD/mira220/patch && ./apply_patch.sh)
echo "Copying source files to Linux source"
(cd $PWD/mira220/src && ./apply_src.sh)
echo "Applying patches to Linux source"
(cd $PWD/mira050/patch && ./apply_patch.sh)
echo "Copying source files to Linux source"
(cd $PWD/mira050/src && ./apply_src.sh)


# prepare compilation script
echo "Preparing a build.sh script inside Linux source dir"
ln -sf $PWD/common/build.sh ./linux/build.sh

# config, build, and install kernel
echo "Inside Linux source dir, configuring the build"
(cd $PWD/linux && ./build.sh restoreconfig)
echo "Inside Linux source dir, compiling kernel and driver modules"
(cd $PWD/linux && ./build.sh build)
echo "Copying artifacts to RPI_KERNEL_INSTALL_DIR defined in linux/build.sh"
(cd $PWD/linux && ./build.sh install)
