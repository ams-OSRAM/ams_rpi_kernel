# Prerequisites:
- Raspberry Pi 4 (RPI4). It is tested on RPI4, but should also support Raspberry Pi Compute Module 4 or Raspberry Pi 3.
- Raspberry Pi OS Bullseye 64bit (recommended) or 32bit. See compilation notes below to set the OS version.
- Mira220 sensor board V3.0 or Mira050 sensor board V1.0.
- Host machine running Ubuntu 18.04 or later.
- Host has the tool chain needed to compile a standard Raspberry Pi Linux Kernel. Refer to [Raspberry Pi doc](https://www.raspberrypi.com/documentation/computers/linux_kernel.html) on the required tools. Preferably, make sure the host has the required tool by compiling a standard Raspberry Pi kernel as stated in the Raspberry Pi doc.

# Compilation and installation:
- Before compilation, configure whether 64bit OS or 32bit OS is built. Open the file `common/build.sh` and edit the variable `OSBIT` to be `64` or `32`. This variable controls a few build options. For example, it controls the `KERNEL` option that points to the actual kernel file used in the RPI. By default, 32-bit RPI 4 uses `kernel7l.img`, 64-bit RPI 4 uses `kernel8.img`.
- Before compilation, configure the RPI hardware type. Open the file `common/build.sh` and edit the variable `RPIHW` to be `RPI4B` or `RPI3B`. This variable controls what kind of build config is used.
- Open the file `common/build.sh` and point `RPI_KERNEL_INSTALL_DIR=` to the actual location that the Raspberry Pi SD card is mounted on the Ubuntu host. Alternative to mounting the SD card by physically plugging it on to the Ubuntu host, it is possible to mount it via ssh by the command `sudo sshfs -o sftp_server="/usr/bin/sudo /usr/lib/openssh/sftp-server" pi@IP_OF_PI:/ /media/pi` where `IP_OF_PI` is the IP address of pi and `/media/pi` is an example mount point on the host.
- Run `build_all.sh`. The script performs the following: (1) Download a specific version of Linux kernel source; (2) Apply Mira220 and Mira050 related source code and patches; (3) Prepare compilation script; (4) Performs actual compilation and installation. To re-run the `build_all.sh` for a specific step, comment out the parts that are not relevant.
- Post-installation, log on to the Raspberry Pi, add a new line to `/boot/config.txt`. Depending on whether Mira220 or Mira050 is connected, this new line will be either `dtoverlay=mira220` or `dtoverlay=mira050` (but not both!). This line tells the RPI to load the corresponding driver at boot time.

# Tests:
- Test whether installation script successfully copies files. After reboot, issue the command `uname -a` and the printed out timestamp of the kernel should match the time that you compile it.
- Test whether Mira220 or Mira050 driver is loaded. As mentioned above, the line `dtoverlay=mira220` or `dtoverlay=mira220` should be added to `/boot/config.txt` on RPI. After reboot, the `dmesg` command should print out logs with keywords `MIRA220` and `MIRA220PMIC` for Mira220 driver; `MIRA050` and `MIRA050PMIC` for Mira050 driver;. For a 64 bit kernel - it could be that you need to add `kernel=kernel8.img` to `config.txt`
- Test whether the power management IC driver module (MIRA220PMIC/MIRA050PMIC) is working. The green LED on the sensor board should be turned on.
- To further test the actual driver module (MIRA220/MIRA050), please refer to a separate repo `ams_rpi_software` and follow instructions from there.

# Post-installation:
- If the RPI has other custom driver modules, such as the Quadric Dev Kit driver (`thor`), these driver needs to be re-installed. That is because the Mira220/Mira050 driver installation script over-writes the kernel image. The reinstallation of `thor` driver requires a specific version of Linux kernel header. The process of getting the Linux kernel header and re-stalling `thor` driver is described in [doc/reinstall_thor_driver.md](doc/reinstall_thor_driver.md).
