# SPDX-License-Identifier: GPL-2.0

KDIR ?= /lib/modules/`uname -r`/build

default:
	$(MAKE) -C $(KDIR) M=$$PWD

KERNELRELEASE ?= $(shell uname -r)
KERNEL_SRC ?= /lib/modules/$(KERNELRELEASE)/build
MODSRC := $(shell pwd)/

INSTALL_MOD_PATH ?= /usr
INSTALL_MOD_DIR ?= /kernel/drivers/media/i2c/

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(MODSRC) modules

install:
	echo "Installing to ${INSTALL_MOD_PATH}/lib/modules/$(KERNELRELEASE)/${INSTALL_MOD_DIR}"
	$(MAKE) INSTALL_MOD_PATH=${INSTALL_MOD_PATH}  INSTALL_MOD_DIR=${INSTALL_MOD_DIR} -C $(KERNEL_SRC) M=$(MODSRC) CONFIG_MODULE_COMPRESS_XZ=y modules_install

post_intall:
	depmod -A

clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(MODSRC) clean

