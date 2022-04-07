LINUX_PATH=../../linux
PATCH_PATH=.
patch $LINUX_PATH/arch/arm64/configs/bcm2711_defconfig < $PATCH_PATH/arm64_bcm2711_defconfig.patch
patch $LINUX_PATH/arch/arm/configs/bcm2711_defconfig < $PATCH_PATH/arm_bcm2711_defconfig.patch
patch $LINUX_PATH/drivers/media/i2c/Kconfig < $PATCH_PATH/i2c_Kconfig.patch
patch $LINUX_PATH/drivers/media/i2c/Makefile < $PATCH_PATH/i2c_Makefile.patch
patch $LINUX_PATH/arch/arm/boot/dts/overlays/Makefile < $PATCH_PATH/overlay_Makefile.patch
