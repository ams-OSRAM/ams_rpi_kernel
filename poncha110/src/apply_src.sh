LINUX_PATH=../../linux
PATCH_PATH=.
cp $PATCH_PATH/poncha110_mono_color-overlay.dtsi $LINUX_PATH/arch/arm/boot/dts/overlays/
cp $PATCH_PATH/poncha110-overlay.dts $LINUX_PATH/arch/arm/boot/dts/overlays/
cp $PATCH_PATH/poncha110color-overlay.dts $LINUX_PATH/arch/arm/boot/dts/overlays/
cp $PATCH_PATH/poncha110.inl $LINUX_PATH/drivers/media/i2c/
cp $PATCH_PATH/poncha110.c $LINUX_PATH/drivers/media/i2c/
cp $PATCH_PATH/poncha110color.c $LINUX_PATH/drivers/media/i2c/
