# Version v0.1.5

New features:
- Support mira220color and mira050color.
- Mira220 driver supports two modes: (1) 640x480 120fps; (2) 1600x1400 30fps. BPP:8/10/12.

Bug fixes:
- Fix Mira220 bring up problem by increasing reset time.

# Version v0.1.4

New features:
- Add Mira050 support. Reduce Mira220 power consumption by keeping reset signal high most of the time.

# Version v0.1.3

Bug fixes:
- Increase reset timing to avoid instable reset.

# Version v0.1.2

New features:
- Use simplified flags `OSBIT` and `RPIHW` in `common/build.sh` to configure 32&64bit OS, and HW version of RPI.

Bug fixes:
- The RPI Linux kernel tag rpi-5.15.y is unstable. Use commit id instead. Tested on 32&64bit OS.

# Version v0.1.1

New features:
- Add 64-bit OS support (tested by overwriting an 32bit OS).

# Version v0.1.0

New features:
- Basic mode: 1600x1400 with 8 bits per pixel.

Known issues:
- Each time a new image or video is captured, the mira220pmic module requires reloading.

