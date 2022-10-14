#!/usr/bin/env bash

cd "$( dirname "${BASH_SOURCE[0]}" )"

set -e

NAME=$0
COMMAND=$1

export BUILD_DIR=${PWD}/build
export RPI_KERNEL_INSTALL_DIR=${RPI_KERNEL_INSTALL_DIR:-"${BUILD_DIR}/kernel_install"}
export RPI_KERNEL_BUILD_DIR=${RPI_KERNEL_BUILD_DIR:-"${BUILD_DIR}/kernel"}
export AMSRPIKERNEL_BUILD_DIR=${AMSRPIKERNEL_BUILD_DIR:-"${BUILD_DIR}/ams_rpi_kernel"}

docker run --rm \
	-v "$PWD:$PWD" \
	--workdir "$PWD" \
	--env CONTAINER_UID=$(id -u) \
	--env CONTAINER_GID=$(id -g) \
	--env CONTAINER_USERNAME=$(id -un) \
	--env RPI_KERNEL_BUILD_DIR=${RPI_KERNEL_BUILD_DIR} \
	--env RPI_KERNEL_INSTALL_DIR=${RPI_KERNEL_INSTALL_DIR} \
	ams_rpi_docker_x86:latest ./build_all.sh
