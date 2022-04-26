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

