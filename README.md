# csse4011-argus-violet
## CSSE4011 Final Project
Argus-Violet is a smart camera tracking system utilising machine learning, built around an ESP32-CAM mounted on a two-axis tilt and swivel servo, PC interface, and a NVIDIA Jetson Xavier board. For more information, see the Wiki tab.

To use this code first run
```
west init -m git@github.com:DomRicho/csse4011-argus-violet --mr main csse4011_project
cd csse4011_project
west update
```
to build run
```
cd csse4011-argus-violet
west build -b esp32s3_devkitc app -d app/build
```
if experiencing a 
```
zephyr_install/zephyr-sdk-0.17.0/xtensa-espressif_esp32_zephyr-elf/bin/xtensa-espressif_esp32_zephyr-elf-gcc
not found - Please check your toolchain installation
```
error, run 
```
cd zephyr_install/zephyr-sdk-0.17.0 
./setup.sh

```

```
west flash -d app/build
```

