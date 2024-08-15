# Linux Drivers

This repository contains source code for some Linux character device drivers that I developed as a part of the Linux drivers course at the University of Wroclaw (a.y. 2021-2022):
* [driver_calc](https://github.com/panantoni01/Linux_Driver_Virtual/tree/main/driver_calc) - driver of a simple arithmetic peripheral, that is capable of performing +,-,*,/ operations
* [driver_litex_gpio](https://github.com/panantoni01/Linux_Driver_Virtual/tree/main/driver_litex_gpio) - driver of a LiteX's GPIO device that counts the number interrupts raised from the device
* [driver_si7021](https://github.com/panantoni01/Linux_Driver_Virtual/tree/main/driver_si7021) - driver of Silabs' SI7021 humidity and temperature sensor, connected via I2C
* [driver_iio_si7020](https://github.com/panantoni01/Linux_Driver_Virtual/tree/main) - IIO driver of Silabs' SI702x sensors. This driver was not written from stratch, only the on-chip heater support was added ([link](https://github.com/torvalds/linux/commit/2aac3f9aec74b28ea73ad96efbcc0c56e5ff814f))
* [driver_si7210](https://github.com/panantoni01/Linux_Driver_Virtual/tree/main/driver_si7210) - IIO driver of Silabs' SI7210 magnetic sensor 

All the drivers can be built and run on a Vexriscv processor that is emulated in [Renode](https://github.com/renode/renode) framework.

## Prerequisites

### Renode
This project uses a customized version of Renode, which is pinned as a submodule. All the prerequisites needed for building Renode from sources are described in the [Renode's documentation](https://renode.readthedocs.io/en/latest/advanced/building_from_sources.html) In order to build it one can use the command:
```
make build-renode
```

Downloading the latest renode tarball is also possible, however this is less recommended since it would result in a limited functionality of some devices that I used in this repository.
```
wget https://dl.antmicro.com/projects/renode/builds/renode-latest.linux-portable.tar.gz
mkdir renode_portable
tar xf  renode-latest.linux-portable.tar.gz -C renode_portable --strip-components=1
```

## Environment setup
* `make build-buildroot` - build `rootfs.cpio` initrd image. A 32-bit RISC-V toolchain is also built which can be later used to build the linux kernel, opensbi and the driver itself if there is no RISC-V toolchain available on the system. In order to use the buildroot toolchain, the PATH variable needs to be modified:
```
export PATH=${REPO_PATH}/build/buildroot/host/bin:${PATH}
```

* `make build-linux` - build a Linux kernel for RISC-V architecture.

* `make build-opensbi` - build OpenSBI bootloader.

## Build and run

### Build a selected driver
In any of the `driver_*` directories, the following commands can be used to build all the files required to emulate a specific driver and test it in Renode:
* `make dtb` - build the device tree blob, that is used by kernel to read the hardware configuration
* `make modules` - build the `*.ko` file, that is the kernel module (later loaded with *insmod*)
* `make test` - build the test application

One can also simply type `make` to build all the required targets.

Once all the files are built, one should also create a `virtio` image, which contains all the built files. This approach makes it easy to access the kernel module and test application from the emulated Linux system:
* `make build-virtio` - build the virtio image

### Run Renode simulation
First, start Renode:
```
./build/renode/renode
```
Load the selected Renode script:
```
(monitor) include @driver_*/scripts/litex.resc
```
And finally start the emulation:
```
(machine-0) start
```
