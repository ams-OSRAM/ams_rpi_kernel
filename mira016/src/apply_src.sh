LINUX_PATH=../../linux
PATCH_PATH=.
cp $PATCH_PATH/mira016_mono_color-overlay.dtsi $LINUX_PATH/arch/arm/boot/dts/overlays/
cp $PATCH_PATH/mira016-overlay.dts $LINUX_PATH/arch/arm/boot/dts/overlays/
cp $PATCH_PATH/mira016.inl $LINUX_PATH/drivers/media/i2c/
cp $PATCH_PATH/mira016_registers.inl $LINUX_PATH/drivers/media/i2c/
cp $PATCH_PATH/mira016.c $LINUX_PATH/drivers/media/i2c/
