LINUX_PATH=../../linux
PATCH_PATH=.
cp $PATCH_PATH/mira130_mono_color-overlay.dtsi $LINUX_PATH/arch/arm/boot/dts/overlays/
cp $PATCH_PATH/mira130-overlay.dts $LINUX_PATH/arch/arm/boot/dts/overlays/
cp $PATCH_PATH/mira130.inl $LINUX_PATH/drivers/media/i2c/
cp $PATCH_PATH/mira130.c $LINUX_PATH/drivers/media/i2c/
