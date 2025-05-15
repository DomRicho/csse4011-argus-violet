# csse4011-argus-violet
## CSSE4011 Final Project

To use this code first run
```
west init -m https://github.com/DomRicho/csse4011-argus-violet --mr main csse4011_project
cd csse4011_project
west update
```
to build run
```
cd csse4011-argus-violet
west build -b esp32s3_devkitc app -d app/build
```

```
west flash -d app/build
```

