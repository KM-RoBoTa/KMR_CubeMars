# Library for CubeMars AK motors in MIT mode, in C++

TODO:
- candump -xtd can0
- continue from stopMotors, that needs to be checked
- clear all flags before sending anything, just in case

1 motor (2 physically connected);
Write enable motors elapsed: 9 us
Read enable motors elapsed: 959 us
Total enable motors elapsed: 1066 us

2 motors:
Write enable motors elapsed: 8 us
Read enable motors elapsed: 609 us
Write enable motors elapsed: 10 us
Read enable motors elapsed: 353 us
Total enable motors elapsed: 1100 us

3 motors:
Write enable motors elapsed: 9 us
Read enable motors elapsed: 355 us
Write enable motors elapsed: 8 us
Read enable motors elapsed: 358 us
Write enable motors elapsed: 8 us
Read enable motors elapsed: 339 us
Total enable motors elapsed: 1234 us

- Position example with 3 motors: Set: 1005 us, get: 1007 us
- Position example with 2 motors: Set: 698 us, get: 692 us
- Position example with 1 motor: Set: 348 us, get: 341 us

This library allows the easy control of CubeMars AK motors in MIT mode. <br /> 
It is written in C++ for the Linux OS, but has only been tested with Archlinux ARM. <br /> 
It uses SI units, the only exception being temperature expressed in °C instead of Kelvins.

In addition to a full documentation, this library also provides several examples to illustrate its use.<br /> 
To get the most complete documentation, it is recommended to regenerate the doxygen version locally.

**Requirement**: depending on your hardware, a CAN controller might be necessary. For example, if using a Raspberry Pi, you can use a [CAN module containing a MCP2515 CAN controller with an SPI interface](https://joy-it.net/en/products/SBC-CAN01).

## Links

- Repository: https://github.com/KM-RoBoTa/KMR_CubeMars
- How to [setup](docs/markdown_sources/setup_git.md)
- How to [use](docs/markdown_sources/use_git.md)

## Quick commands 
Get IP:
```bash
sudo arp-scan --localnet
```

Set up can:
```bash
sudo ip link set can0 up type can bitrate 1000000
```

## About

KM-RoBoTa SA's KMR_CubeMars library to control CubeMars AK motors in MIT mode.

### Authors
Library written by Katarina Lichardova: katarina.lichardova@km-robota.com

### Copyright
Copyright 2021-2025, Kamilo Melo. \n
This code is under MIT licence: https://opensource.org/licenses/MIT


## Supported models

- AK10-9
- AK60-6
- AK70-10
- AK80-6
- AK80-8
- AK80-9
- AK80-80
- AK80-64
     
Note: the documentation explains how to add a model into the library