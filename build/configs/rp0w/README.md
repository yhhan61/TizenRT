# Rasberry Pi Zero W

The Rasberry Pi Zero W is a Single Chip Computer for Wi-Fi™. The Rasberry Pi Zero W has a Wi-Fi subsystem, and application subsystem.

## Information

The Rasberry Pi Zero W is a Single Chip Computer for Wi-Fi™. It is based on an Broadcom BCM2835® chip with ARM1176® core.

## Environment Set-up
### On Chip Debugger installation

OpenOCD is used to program and debug.

OpenOCD v0.10.0 is recommended and can be installed like below,
but pre-built OpenOCD binaray on tools/openocd/linux64(or 32) can be used without installing.
```bash
sudo apt-get build-dep openocd
git clone --depth 1 -b v0.10.0 https://git.code.sf.net/p/openocd/code openocd-code
cd openocd-code
./bootstrap
./configure
make
sudo make install
```

## How to program a binary

There are two methods, using OpenOCD or script.

After building Tizen RT, follow below steps at $TIZENRT_BASEDIR/os folder.

TIZENRT_BASEDIR was set at [[Getting the sources]](../../../README.md#getting-the-sources) tab of Quick Start.

### Using download script

```bash
make download ALL
```
This makes complete set of binaries programmed.

### Using OpenOCD

This is used to program a partial binary.

Export 'OPENOCD_SCRIPTS' to environment variable.

```bash
export OPENOCD_SCRIPTS=$TIZENRT_BASEDIR/build/configs/rp0w/tools/openocd
```

At first, programming the complete set of binaries are needed.
After buiding a Tizen RT, execute as follows at os folder.
```bash
openocd -f rp0w.cfg -c ' \
    flash_write bl1    ../build/configs/rp0w/bin/bl1.bin;      \
    flash_write bl2    ../build/configs/rp0w/bin/bl2.bin;      \
    flash_write sssfw  ../build/configs/rp0w/bin/sssfw.bin;    \
    flash_write wlanfw ../build/configs/rp0w/bin/wlanfw.bin;   \
    flash_write os     ../build/output/bin/tinyara_head.bin;       \
    exit'
```

Once the complete binaries are successfully programmed, each partition can be updated seperately with new one.
```bash
openocd -f rp0w.cfg -c ' \
    flash_write os ../build/output/bin/tinyara_head.bin; exit'
```

## Configuration Sets

will be updated

#### nettest

#### tash

