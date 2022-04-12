Prerequiresites:
- Raspberry Pi 4 (RPI4). It is tested on RPI4, but shoudl also support Raspberry Pi Compute Module 4 or Raspberry Pi 3.
- Raspberry Pi OS Bullseye 32bit.
- Mira220 sensor board V3.0

Compilation and installation:
- Open the file `common/build.sh` and point `RPI_KERNEL_INSTALL_DIR=` to the actual location that the Raspberry Pi SD card is mounted on the Ubuntu host. Alternative to mounting the SD card by physically plugging it on to the Ubuntu host, it is possible to mount it via ssh by the command `sudo sshfs -o sftp_server="/usr/bin/sudo /usr/lib/openssh/sftp-server" pi@IP_OF_PI:/ /media/pi` where `IP_OF_PI` is the IP address of pi and `/media/pi` is an example mount point on the host.
- Run `build_all.sh`
- Add `dtoverlay=mira220` to the `/boot/config.txt`

Tests:
- After reboot, issue the command `uname -a` and the printed out timestamp of the kernel should match the time that you compile it.
- The green LED on the Mira220 sensor board is turned on.
- To further test the driver, we will need userspace software in a separate repo, and follow instructions from there.
