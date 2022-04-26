
# checkout kernel source
COMMIT=6dafd553901ea01d8871010488121e21d131fea4
wget https://github.com/raspberrypi/linux/archive/${COMMIT}.zip
echo "Downloading Raspberry Pi Linux repo commit ${COMMIT}"
unzip ${COMMIT}.zip
ln -s $PWD/linux-${COMMIT} linux

# apply patches and sources
echo "Applying patches to Linux source"
(cd mira220/patch && ./apply_patch.sh)
echo "Copying source files to Linux source"
(cd mira220/src && ./apply_src.sh)

# prepare compilation script
echo "Preparing a build.sh script inside Linxu source dir"
ln -s $PWD/common/build.sh ./linux/build.sh

# config, build, and install kernel
echo "Inside Linux source dir, configuring the build"
(cd ./linux && ./build.sh restoreconfig)
echo "Inside Linux source dir, compiling kernel and driver modules"
(cd ./linux && ./build.sh build)
echo "Copying artifacts to RPI_KERNEL_INSTALL_DIR defined in linux/build.sh"
(cd ./linux && ./build.sh install)

