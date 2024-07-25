
# ADS8634 user space driver 

This project provides a user-space application for interacting with the ADS8634 analog-to-digital converter (ADC) via SPI. The application allows users to read from and write to registers of the ADS8634, configure settings, and switch between different pages.

## Features

- **Read from a register**
    ./spi -r 0x60

- **Write to a register**
    ./spi -w 0x60 0x10

- **Switch between pages**
    ./spi -p 1
    ./spi -p 0

## Requirements

- **Linux-based OS** (Ubuntu or similar)
- **SPI interface enabled**
- **`spidev` driver available**
- **GCC 9.4.0 compiler** for building the application

## Compilation

To compile the program, run the makefile:

```sh
make