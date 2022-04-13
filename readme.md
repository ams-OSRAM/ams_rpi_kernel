Prerequisites:
- Raspberry Pi 4 (RPI4). It is tested on RPI4, but should also support Raspberry Pi Compute Module 4 or Raspberry Pi 3.
- Raspberry Pi OS Bullseye 32bit.
- Mira220 sensor board V3.0
- Host machine running Ubuntu 18.04 or later.

Compilation and installation:
- Open the file `common/build.sh` and point `RPI_KERNEL_INSTALL_DIR=` to the actual location that the Raspberry Pi SD card is mounted on the Ubuntu host. Alternative to mounting the SD card by physically plugging it on to the Ubuntu host, it is possible to mount it via ssh by the command `sudo sshfs -o sftp_server="/usr/bin/sudo /usr/lib/openssh/sftp-server" pi@IP_OF_PI:/ /media/pi` where `IP_OF_PI` is the IP address of pi and `/media/pi` is an example mount point on the host.
- Run `build_all.sh`. The script performs the following: (1) clone a Linux kernel source; (2) Apply Mira220 related source code and patches; (3) Prepare compilation script; (4) Performs actual compilation and installation. To re-run the `build_all.sh` for a specific step, comment out the parts that are not relevant.
- On the Raspberry Pi, add `dtoverlay=mira220` to `/boot/config.txt` to tell the RPI to load the driver at boot time.

Tests:
- Test whether installation script successfully copies files. After reboot, issue the command `uname -a` and the printed out timestamp of the kernel should match the time that you compile it.
- Test whether Mira220 driver is loaded. As mentioned above, the line `dtoverlay=mira220` should be added to `/boot/config.txt` on RPI. After reboot, the `dmesg` command should print out logs with keywords `MIRA220` and `MIRA220PMIC`.
- Test whether the power management IC driver module (MIRA220PMIC) is working. The green LED on the Mira220 sensor board should be turned on.
- To further test the actual driver module (MIRA220), please refer to a separate repo `ams_rpi_software` and follow instructions from there.
