# Installation and Setup

## Prerequisites
In order to build the examples, you need the NXP MCUXpresso SDK for the
Kinetis KW41Z and some additional packages.

### Get the NXP MCUXpresso SDK
NXP provides the MCUXpresso SDK at
[mcuxpresso.nxp.com](https://mcuxpresso.nxp.com). Click **SDK Builder** and
choose **New Configuration** in the dropdown menu. Select the **MKW41Z512VHT4**
processor. Select **Select Additional Configuration Settings** and choose Linux
with all toolchains. Finally click **Go To SDK Builder**. Once your SDK is
available for download, download and unpack it to a desired directory.

### Install a Toolchain
In a Debian/Ubuntu Linux based environment you need to install the following
list of packages:
```sh
apt install cmake gcc-arm-none-eabi gdb-arm-none-eabi
```
If you need tools to debug a BLE application you may want to additionally
install *bluez*:
```sh
apt install bluez
```
To generate the documentation you need *Doxygen*:
```sh
apt install doxygen
```

### Install pyOCD
*pyOCD* is used for flashing the examples onto the phyWAVE module. Get the
source code from
```sh
git clone https://github.com/PHYTEC-Messtechnik-GmbH/pyOCD/tree/pr-target-kw41z
```
and follow the instruction of the included `README.rst` to install pyOCD. It is
recommended to install pyOCD locally for your user from source with:
```sh
python setup.py install --user
```
> **Note:** If flashing fails with a USB timeout error or similar and you are
> using a virtual machine as your host setting a lower frequency may be
> necessary. Set the `--frequency` value to e.g. `100000` in
> `cmake/pb-daplink.cmake`:
> ```
> ...
> add_custom_target(flash
>   COMMAND pyocd-flashtool --target ${PYOCD_TARGET} --chip_erase --frequency 100000
> ...
> ```

## Clone the phyWAVE BSP
Clone the phyWAVE BSP to your preferred directory:
```sh
git clone https://github.com/PHYTEC-Messtechnik-GmbH/phywave-mcux-bsp
```
This repository includes the BSP and examples for the phyNODE-Athena development
board described in this manual.

### Environment Setup
Before configuring the build files of the examples the following environment
variables have to be set (e.g. in your `~/.bashrc`):
```sh
# do not change this
export GCC_TOOLCHAIN_DIR=/usr/

# original MCUXpresso SDK from NXP
export MCU_SDK_KW41Z_DIR=/home/user/foo_dir/mcux-sdk/

# general board support for these examples
export PHYWAVE_SDK_DIR=/home/user/foo_dir/phywave-mcux-bsp/
```

## Building and flashing an example
Connect the debugger via USB to the host computer. To build and flash any
example enter its base directory and execute `cmake` and `make`:
```sh
# enter the base directory of an example
cd examples/phynode_ble/

# generate the Makefile
cmake .

# build the example (does not flash)
make

# build and flash
make flash
```

## Documentation
Each example project contains a `README.md` with a brief introduction about
their usage and functionality.

Note that the [MCUXpresso SDK](https://mcuxpresso.nxp.com) from NXP downloaded
previously contains documentation for the underlying framework and RTOS.
