LINUX_PATH=../../linux
PATCH_PATH=.

insert_file_A_into_file_B_before_pattern_C_if_pattern_D_does_not_exist () {
	A="$1"
	B="$2"
	C="$3"
	D="$4"
	grep -qF "$D" "$B"
	if [ $? -ne 0 ]; then
		echo "File $B does not contain pattern $D."
		echo "Insert contents of file $A before pattern $C."
		sed -i '/'"${C}"'/e cat '"${A}"'' $B
	else
		echo "File $B already contains pattern $D."
		echo "Skip inserting."
	fi
}


# Patch defconfig of RPI 3&4 CPUs for 32&64bit OS

# 64bit OS RPI 3
INSERT_FILE=$PATCH_PATH/defconfig.txt
TARGET_FILE=$LINUX_PATH/arch/arm64/configs/bcmrpi3_defconfig
INSERT_BEFORE="CONFIG_VIDEO_IMX219=m"
INSERT_IF_NOT_EXIST="CONFIG_VIDEO_MIRA220=m"
insert_file_A_into_file_B_before_pattern_C_if_pattern_D_does_not_exist "$INSERT_FILE" "$TARGET_FILE" "$INSERT_BEFORE" "$INSERT_IF_NOT_EXIST" 

# 64bit OS RPI 4
TARGET_FILE=$LINUX_PATH/arch/arm64/configs/bcm2711_defconfig
insert_file_A_into_file_B_before_pattern_C_if_pattern_D_does_not_exist "$INSERT_FILE" "$TARGET_FILE" "$INSERT_BEFORE" "$INSERT_IF_NOT_EXIST" 

# 32bit OS RPI 3
TARGET_FILE=$LINUX_PATH/arch/arm/configs/bcm2709_defconfig
insert_file_A_into_file_B_before_pattern_C_if_pattern_D_does_not_exist "$INSERT_FILE" "$TARGET_FILE" "$INSERT_BEFORE" "$INSERT_IF_NOT_EXIST" 

# 32bit OS RPI 4
TARGET_FILE=$LINUX_PATH/arch/arm/configs/bcm2711_defconfig
insert_file_A_into_file_B_before_pattern_C_if_pattern_D_does_not_exist "$INSERT_FILE" "$TARGET_FILE" "$INSERT_BEFORE" "$INSERT_IF_NOT_EXIST" 

# 32bit OS RPI 3
TARGET_FILE=$LINUX_PATH/arch/arm/configs/bcmrpi_defconfig
insert_file_A_into_file_B_before_pattern_C_if_pattern_D_does_not_exist "$INSERT_FILE" "$TARGET_FILE" "$INSERT_BEFORE" "$INSERT_IF_NOT_EXIST" 

# Patch i2c Kconfig

INSERT_FILE=$PATCH_PATH/i2c_Kconfig.txt
TARGET_FILE=$LINUX_PATH/drivers/media/i2c/Kconfig
INSERT_BEFORE="config VIDEO_IMX208"
INSERT_IF_NOT_EXIST="config VIDEO_MIRA220"
insert_file_A_into_file_B_before_pattern_C_if_pattern_D_does_not_exist "$INSERT_FILE" "$TARGET_FILE" "$INSERT_BEFORE" "$INSERT_IF_NOT_EXIST" 

# Patch i2c Makefile

INSERT_FILE=$PATCH_PATH/i2c_Makefile.txt
TARGET_FILE=$LINUX_PATH/drivers/media/i2c/Makefile
INSERT_BEFORE="imx208.o"
INSERT_IF_NOT_EXIST="mira220.o"
insert_file_A_into_file_B_before_pattern_C_if_pattern_D_does_not_exist "$INSERT_FILE" "$TARGET_FILE" "$INSERT_BEFORE" "$INSERT_IF_NOT_EXIST" 

# Patch overlay Makefile

INSERT_FILE=$PATCH_PATH/overlay_Makefile.txt
TARGET_FILE=$LINUX_PATH/arch/arm/boot/dts/overlays/Makefile
INSERT_BEFORE="imx219.dtbo"
INSERT_IF_NOT_EXIST="mira220.dtbo"
insert_file_A_into_file_B_before_pattern_C_if_pattern_D_does_not_exist "$INSERT_FILE" "$TARGET_FILE" "$INSERT_BEFORE" "$INSERT_IF_NOT_EXIST" 


