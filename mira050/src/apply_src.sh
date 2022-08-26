LINUX_PATH=../../linux
PATCH_PATH=.
cp $PATCH_PATH/mira050_mono_color-overlay.dtsi $LINUX_PATH/arch/arm/boot/dts/overlays/
cp $PATCH_PATH/mira050-overlay.dts $LINUX_PATH/arch/arm/boot/dts/overlays/
cp $PATCH_PATH/mira050color-overlay.dts $LINUX_PATH/arch/arm/boot/dts/overlays/
cp $PATCH_PATH/mira050.inl $LINUX_PATH/drivers/media/i2c/
cp $PATCH_PATH/mira050.c $LINUX_PATH/drivers/media/i2c/
cp $PATCH_PATH/mira050color.c $LINUX_PATH/drivers/media/i2c/
cp $PATCH_PATH/mira050pmic.c $LINUX_PATH/drivers/media/i2c/
