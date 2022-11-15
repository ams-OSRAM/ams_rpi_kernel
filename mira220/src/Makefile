# SPDX-License-Identifier: GPL-2.0

KERNELRELEASE ?= $(shell uname -r)

KDIR ?= /lib/modules/$(KERNELRELEASE)/build
INCDIR ?= /usr/src/linux-headers-$(KERNELRELEASE)/include

KERNEL_SRC ?= /lib/modules/$(KERNELRELEASE)/build
MODSRC := $(shell pwd)/

INSTALL_MOD_PATH ?= /usr
INSTALL_MOD_DIR ?= /kernel/drivers/media/i2c/

default:
	$(MAKE) -C $(KDIR) M=$$PWD CPATH=$(INCDIR)

install:
	$(MAKE) INSTALL_MOD_PATH=${INSTALL_MOD_PATH}  INSTALL_MOD_DIR=${INSTALL_MOD_DIR} -C $(KERNEL_SRC) M=$(MODSRC) CONFIG_MODULE_COMPRESS_XZ=y modules_install

post_intall:
	depmod -A

clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(MODSRC) clean

