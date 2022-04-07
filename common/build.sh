#!/usr/bin/env bash

set -e

cd "$( dirname "${BASH_SOURCE[0]}" )"

# Since Raspberry OS is 32-bit by default (64-bit versions are not oficially released yet) we build
# the kernel for a 32-bit ARM architecture.
export ARCH=arm
export CROSS_COMPILE=arm-linux-gnueabihf-
export KERNEL=kernel7l

CONFIG=bcm2711_defconfig
NAME=$0
COMMAND=$1
RPI_KERNEL_INSTALL_DIR=/media/pi
INSTALL_DIR=${RPI_KERNEL_INSTALL_DIR:-/srv/nfs/rpi4/}
BUILD_DIR=${RPI_KERNEL_BUILD_DIR:-build}

# ASCII Escape sequences
RED_COLOR=31
YELLOW_COLOR=33
BLUE_COLOR=34
CYAN_COLOR=36

set_attribute() {
    echo -ne "\u001b[${1}m"
}

reset_attributes() {
    echo -ne "\u001b[0m"
}

log_warning() {
    set_attribute ${YELLOW_COLOR}
    echo "$@"
    reset_attributes
}

log_error() {
    set_attribute ${RED_COLOR}
    echo "$@"
    reset_attributes
}

usage() {
    set_attribute ${BLUE_COLOR}
    cat <<EOF
Usage: ${NAME} COMMAND
Supported commands are:
    clean:            Cleans the kernel output.
    restoreconfig:    Creates the config from the ${CONFIG} target.
    menuconfig:       Configure the target interactively using ncurses.
    build:            Builds kernel image, modules and device tree binaries.
    modules:          Builds the kernel modules only.
    dtbs:             Builds the device tree binaries only.
    kernel:           Builds the compressed kernel image only.
    install:          Builds and installs kernel and modules in the rootdir specified by
                      RPI_KERNEL_INSTALL_DIR.
    install_modules:  Builds and installs modules in the rootdir specified by RPI_KERNEL_INSTALL_DIR.
    install_dtbs:     Builds and installs dtbs in the rootdir specified by RPI_KERNEL_INSTALL_DIR.
    install_kernel:   Builds and installs kernel in the rootdir specified by RPI_KERNEL_INSTALL_DIR.
    compdb:           Generates a compilation database that can be used with clangd or IntelliSense
                      in VSCode.
EOF
    reset_attributes
    exit 1
}

install_cross_compiler() {
    echo "Would you like to install it now? (you may be asked for your password)"
    select response in "Yes" "No"; do
        case ${response} in
            Yes) sudo apt install -y gcc-arm-linux-gnueabihf; break;;
            No) exit 0;;
            *) echo "Please select one of the available options";;
        esac
    done
}

check_dependencies() {
    if ! ${CROSS_COMPILE}gcc --version > /dev/null 2>&1; then
        log_warning "Could not find ${CROSS_COMPILE}gcc, which is required to build the Raspberry Pi kernel."
        install_cross_compiler
    fi
}

run_command() {
    set_attribute ${CYAN_COLOR}
    echo -n "Running command: "
    reset_attributes
    echo "\"$@\""
    if ! "$@"; then
        log_error "Error running command"
        exit 1
    fi
}
run_command_as_su_no_fail() {
    set_attribute ${CYAN_COLOR}
    echo -n "Running command as super user: "
    reset_attributes
    echo "\"$@\""
    if ! "sudo" "$@"; then
        log_error "Error running command"
    fi
}

run_command_as_su() {
    set_attribute ${CYAN_COLOR}
    echo -n "Running command as super user: "
    reset_attributes
    echo "\"$@\""
    if ! "sudo" "$@"; then
        log_error "Error running command"
        exit 1
    fi
}

run_make() {
    run_command make -j"$(nproc)" "$@" INSTALL_MOD_PATH="${INSTALL_DIR}" "O=${BUILD_DIR}"
}

run_install_modules() {
    run_command_as_su make -j"$(nproc)" modules_install INSTALL_MOD_PATH="${INSTALL_DIR}" "O=${BUILD_DIR}"
}

run_install_dtbs() {
    run_command_as_su mkdir -p "${INSTALL_DIR}/boot/overlays"
    run_command_as_su cp "${BUILD_DIR}"/arch/arm/boot/dts/*.dtb "${INSTALL_DIR}/boot/"
    run_command_as_su cp "${BUILD_DIR}"/arch/arm/boot/dts/overlays/*.dtb* "${INSTALL_DIR}/boot/overlays"
}

run_install_kernel() {
    run_command_as_su mkdir -p "${INSTALL_DIR}/boot"
    run_command_as_su_no_fail cp "${INSTALL_DIR}/boot/$KERNEL.img" "${INSTALL_DIR}/boot/${KERNEL}-backup.img"
    run_command_as_su cp "${BUILD_DIR}/arch/arm/boot/zImage" "${INSTALL_DIR}/boot/${KERNEL}.img"
}

run_install() {
    run_install_modules
    run_install_kernel
    run_install_dtbs
}

run_build() {
    if [ ! -f ${BUILD_DIR}/.config ];
    then
        log_warning "No configuration found, using default configuration."
        run_make ${CONFIG}
    fi
    run_make "$@"
}

gen_compile_commands() {
    KERNEL_SOURCE_DIR=${PWD}
    run_command pushd ${BUILD_DIR}
    run_command ${KERNEL_SOURCE_DIR}/scripts/clang-tools/gen_compile_commands.py
    run_command popd
    run_command ln -s -f ${BUILD_DIR}/compile_commands.json compile_commands.json
}

check_dependencies

case ${COMMAND} in
    clean)
        run_make clean
        run_command rm -rf ${BUILD_DIR};;
    restoreconfig) run_make ${CONFIG};;
    menuconfig) run_make menuconfig;;
    build) run_build zImage dtbs modules;;
    modules) run_build modules;;
    dtbs) run_build dtbs;;
    kernel) run_make zImage;;
    install) run_install;;
    install_modules) run_install_modules;;
    install_dtbs) run_install_dtbs;;
    install_kernel) run_install_kernel;;
    compdb) gen_compile_commands;;
    --) run_make "${@:2}";;
    *) usage;;
esac
