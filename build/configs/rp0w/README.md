# Rasberry Pi Zero W

The Rasberry Pi Zero W is a Single Chip Computer for Wi-Fi™. The Rasberry Pi Zero W has a Wi-Fi subsystem, and application subsystem.

## Information

The Rasberry Pi Zero W is a Single Chip Computer for Wi-Fi™. It is based on an Broadcom BCM2835® chip with ARM1176® core.

## Environment Set-up

## How to program a binary

After building Tizen RT, you can make bootable SD for raspberry pi 0 w

So, you should make bootable SD at first, after that, you can make new image by copying the build image 

### Make bootable SD master image 
You should make bootable SD following to raspberry pi web site
https://github.com/raspberrypi/firmware/tree/master/boot

After making it (maybe Raspbian), there are many files in the master SD,
remove all files except bootcode.bin(50KB), start.elf(2.8MB) and kernel.img  

You need only these three files for boot TizenRT.

After making master SD, 
copy created output binary (/build/output/bin/tinyara.bin) to kernel.img in the SD.


## Configuration Sets

#### tash

#### demo

#### fstest

#### usbtest

#### nettest

