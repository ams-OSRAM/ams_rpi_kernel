# Overview
The purpose of this document is to describe how to (re-)install Quadric Dev Kit driver (`thor`) after the Mira220 driver is installed. It mainly involves installing a specific version of Linux kernel header, such that the Quadric Dev Kit driver can be compiled.

# Assumption: OS versions
- OS image used on Quadric Dev Kit: [link](https://downloads.raspberrypi.org/raspios_arm64/images/raspios_arm64-2022-04-07/)
- OS source tag used by Mira220 driver: [link](https://github.com/raspberrypi/linux/releases/tag/1.20220331)

# Step 1: Download Linux kernel source to /usr/src
```
cd /usr/src
sudo wget https://github.com/raspberrypi/linux/archive/refs/tags/1.20220331.tar.gz
sudo tar -zxf ./1.20220331.tar.gz
sudo mv linux-1.20220331 linux-headers-5.15.32-v8
```

# Step 2: Link kernel modules, installed by Mira220 driver script, to Linux kernel source
```
cd /lib/modules/5.15.32-v8
sudo rm build
sudo ln -s /usr/src/linux-headers-5.15.32-v8 build
sudo rm source
sudo ln -s /usr/src/linux-headers-5.15.32-v8 source
```

# Install Linux kernel headers
```
cd /usr/src
cd linux-headers-5.15.32-v8
KERNEL=kernel8
sudo make bcm2711_defconfig
sudo make prepare
sudo make modules_prepare
```

# (Re-install) the actual Quadric Dev Kit driver
```
sudo dpkg -i thor-dkms_0.3.0-1_all.deb
```
