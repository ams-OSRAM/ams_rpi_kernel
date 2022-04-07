
# checkout kernel source
TAG=rpi-5.15.y
git clone --depth 1 --branch $TAG https://github.com/raspberrypi/linux.git

# apply patches and sources
(cd mira220/patch-$TAG && ./apply_patch.sh)
(cd mira220/src && ./apply_src.sh)

# prepare compilation script
ln -s $PWD/common/build.sh ./linux/build.sh

# config, build, and install kernel
(cd ./linux && ./build.sh restoreconfig)
(cd ./linux && ./build.sh build)
(cd ./linux && ./build.sh install)

