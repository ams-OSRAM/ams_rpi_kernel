
# checkout kernel source
TAG=rpi-5.15.y
echo "Cloning Raspberry Pi Linux source tag ${TAG}"
git clone --depth 1 --branch $TAG https://github.com/raspberrypi/linux.git

# apply patches and sources
echo "Applying patches to Linux source"
(cd mira220/patch-$TAG && ./apply_patch.sh)
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

