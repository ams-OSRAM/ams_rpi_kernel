# Overview
The purpose of this document is to describe how to (re-)install Quadric Dev Kit driver (`thor`) after the Mira220 driver is installed. It mainly involves installing a specific version of Linux kernel header, such that the Quadric Dev Kit driver can be compiled.

# Assumption: OS versions
- OS image used on Quadric Dev Kit: [link](https://downloads.raspberrypi.org/raspios_arm64/images/raspios_arm64-2022-09-07/)
- OS source tag used by Mira220 driver: [link](https://github.com/raspberrypi/linux/releases/tag/1.20220830)

Issue the following command to make sure the kernel version is `5.15.61-v8+`.
```
uname -a
```

# Step 1: Purge any installed dkms packages related to Quadric driver
```
sudo dpkg --purge thor-dkms thor-tool
```

# Step 2: Install kernel header 
```
sudo apt-get install raspberrypi-kernel-headers
```

# Step 3: (Re-install) the actual Quadric Dev Kit driver
```
sudo dpkg -i thor-dkms_0.3.0-1_all.deb
```

# Step 4: (Re-install) Quadric Dev Kit driver tools
```
sudo dpkg -i thor-tool_0.3.1-1_all.deb
```

# Step 5: Test installation
Check whether the driver can detect Quadric Dev Kit hardware by the command below.
```
thor-tool probe
```
Run a list of test by the command below.
```
thor-tool test
```

