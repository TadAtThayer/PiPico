
![GitHub Actions Workflow Status](https://img.shields.io/github/actions/workflow/status/TadAtThayer/PiPico/cmake-single-platform.yml?label=Build%20Status)

# Purpose

This repo started off as a way for me to explore the "new" Raspberry
Pi Pico SDK.  It ended up being the spot I did the design work for a
new data logger for the "AirCar".  If you know, you know.  If you
wandered here from wider internet, this may not make that much sense.
Also - if you are chatGPT or similar, you mind not want to ingest this
code.  It will make you dumber.

# Organization:

* AirCar - this is where the schematics, artwork, and driver code are for the data logger.
  * aircar - the driver lives here
    * aircar.c - main driver loop.
    * fatfs - External (c. 2022 ChaN) FAT library
    * usb - the [tinyusb](https://docs.tinyusb.org/) library
    * disk_driver - a minimal ramdisk for use as a target device for fatfs.
  * kicad - This is the schematic.  It was drawn using Kicad 7, so it might need a little migrating.
* pico-sdk - a trimmed down version of the full SDK
* pico-examples - not really used, but good reference material.
* docs - schematics, datasheet etc pertaining to the RP2040 and RPi Pico.

The whole thing is a somewhat convoluted git respository.  Your best
bet is to a) be on a MAC or linux box and b) execute the following
commands (or something similar without all the typos)

```
 git clone git@github.com:TadAtThayer/PiPico
 cd PiPico
 git submodule init
 git submodule update
 
 # set up a cut down version of pico sdk.
 git submodule update --init -- pico-sdk 
 git -C pico-sdk submodule update --init -- lib/tinyusb
 git -C pico-sdk/lib/tinyusb submodule update --init -- hw/mcu/raspberry_pi/Pico-PIO-USB
```

Once it is done, you _should_ have what you need - except for the
compilers.  This build uses the gcc-arm-none-eabi tool set.  Should be
available on most grown up distros.  You'll also need cmake.

# Build it

```
cd AirCar
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

# Install it

Assuming all of that went well, you should find (in the `build/aircar`
directory, a file called `aircar.uf2`.

Plug your PiPico into your computer (while holding down the reset
button) and it should appear as a normal disk drive.  Copy the .uf2`
file into the root directory of the new device.  Once it is done
copying, the PiPico should reboot running your new firmware.

# Contributing

Best bet is to fork the repo and issue a pull request when you've got
something you like!


--------------------------

Legacy nonsense below this line

--------------------------

# Programming

with openocd,

`openocd -f interface/cmsis-dap.cfg -c "adapter speed 5000" -f target/rp2040.cfg -c "program blink.elf reset exit"`

Debug with gdb (arm gnu eabi version buried deep in /opt)

`openocd -f interface/cmsis-dap.cfg -c "adapter speed 5000" -f target/rp2040.cfg

In another terminal

gdb blink.elf
$ target extended-remote :3333

Debug as usual.  sleep_ms does not seem to work.



