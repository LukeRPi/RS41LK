# RS41LK - Amateur radio firmware for Vaisala RS41 radiosonde

**NOTE:** This firmware is a work in progress and some features might not work as expected yet!

This is a custom firmware for [Vaisala RS41 radiosondes](https://www.vaisala.com/en/products/instruments-sensors-and-other-measurement-devices/soundings-products/rs41).
The code is based on the RS41 firmware [RS41ng](https://github.com/mikaelnousiainen/RS41ng).


### What can I do with an RS41 radiosonde?

The [Vaisala RS41 radiosondes](https://www.vaisala.com/en/products/instruments-sensors-and-other-measurement-devices/soundings-products/rs41)
uses an off-the-shelf [STM32F100C8](https://www.st.com/en/microcontrollers-microprocessors/stm32f100c8.html)
32-bit microcontroller, which can be reprogrammed using an [ST-LINK v2 programmer](https://www.st.com/en/development-tools/st-link-v2.html)
or a smaller [ST-LINK v2 USB dongle](https://www.adafruit.com/product/2548).

The RS41 hardware can be programmed to transmit different kinds of RF modulations (morse code, APRS and different FSK modulations)
on the 70 cm (~433 MHz) amateur radio band. The radiosonde contains a [UBX-G6010](https://www.u-blox.com/en/product/ubx-g6010-st-chip)
GPS chip, so it can be used as a tracker device, e.g. for high-altitude balloons.

The RS41LK firmware is just one example of what can be achieved with the RS41 hardware!

## Why does the RS41LK firmware exist?

The motivation to develop this firmware is to provide a clean, customizable and
modular codebase for developing RS41 radiosonde-based experiments.

See the feature list below.

## Features

The main features the RS41LK firmware are:

* Support for multiple transmission modes:
  * Standard 1200-baud APRS
    * Option to transmit APRS weather reports using readings from an external BMP280 sensor
  * Morse code (CW)

* Support for custom sensors via the external I²C bus
* GPS NMEA data output via the external serial port pin 4 (see below). This disables use of I²C devices as the serial port pins are shared with the I²C bus pins.
  * This allows using the RS41 sonde GPS data in external tracker hardware, such as Raspberry Pi or other microcontrollers.
* Enhanced support for the internal Si4032 radio transmitter via PWM-based tone generation (and ultimately DMA-based symbol timing, if possible)
* Extensibility to allow easy addition of new transmission modes and new sensors

### Transmission modes

On the internal Si4032 transmitter:

* APRS (1200 baud)
* Morse code (CW)


#### Notes about APRS

* Bell 202 frequencies are generated via hardware PWM, but the symbol timing is created in a loop with delay
* There is also code available to use DMA transfers for symbol timing to achieve greater accuracy, but I have not been able to get the timings working correctly


### External sensors

It is possible to connect external sensors to the I²C bus of the RS41 radiosonde.

The following sensors are currently supported:

* Bosch BMP280 barometric pressure / temperature / humidity sensor

Sensor driver code contributions are welcome!

### Planned features
* Support for more I²C sensors

## Configuring the firmware

1. Configure your amateur radio call sign, transmission schedule (time sync),
   transmit frequencies and transmission mode parameters in `config.h`
2. Set up transmitted message templates in `config.c`, depending on the modes you use

## Building the firmware

Software requirements:

* [GNU GCC toolchain](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-a/downloads/9-2-2019-12)
  version 8.3.0 or higher for cross-compiling the firmware for the ARM Cortex-M3 architecture (`arm-none-eabi-gcc`)
* [CMake](https://cmake.org/) version 3.6 or higher for building the firmware
* [OpenOCD](http://openocd.org/) version 0.10.0 or higher for flashing the firmware

On a Red Hat/Fedora Linux installation, the following packages can be installed:
```bash
dnf install arm-none-eabi-gcc-cs arm-none-eabi-gcc-cs-c++ arm-none-eabi-binutils-cs arm-none-eabi-newlib cmake openocd
```

### Steps to build the firmware

1. Install the required software dependencies listed above
2. Build the firmware using the following commands
    ```
    mkdir build
    cd build
    cmake ..
    make
    ```
3. The firmware will be stored in file `build/src/RS41ng.elf`

## Flashing the firmware

Hardware requirements:

* A working [Vaisala RS41 radiosonde](https://www.vaisala.com/en/products/instruments-sensors-and-other-measurement-devices/soundings-products/rs41)
* An [ST-LINK v2 programmer for the STM32 microcontroller](https://www.st.com/en/development-tools/st-link-v2.html) in the RS41 radiosonde. 
  * These smaller [ST-LINK v2 USB dongles](https://www.adafruit.com/product/2548) also work well.

The pinout of the RS41 connector (by DF8OE) is the following:

```
______________________|           |______________________
|                                                       |
|   1           2           3           4           5   |
|                                                       |
|   6           7           8           9          10   |
|_______________________________________________________|

(View from the bottom of the sonde, pay attention to the gap in the connector)
```

* 1 - SWDIO (PA13)
* 2 - RST
* 3 - MCU switched 3.3V out to external device / Vcc (Boost out) 5.0V
  * This pin powers the device via 3.3V voltage from an ST-LINK programmer dongle
  * This pin can be used to supply power to external devices, e.g. Si5351, BMP280 or other sensors
* 4 - I2C2_SCL (PB10) / UART3 TX
  * This is the external I²C port clock pin for Si5351 and sensors
  * This pin can alternatively be used to output GPS NMEA data to external tracker hardware (e.g. Raspberry Pi or other microcontrollers)
* 5 - GND
* 6 - GND
* 7 - SWCLK (PA14)
* 8 - +U_Battery / VBAT 3.3V
* 9 - +VDD_MCU / PB1 * (10k + cap + 10k)
* 10 - I2C2_SDA (PB11) / UART3 RX
  * This is the external I²C port data pin for Si5351 and sensors

### Steps to flash the firmware

1. Remove batteries from the sonde
2. Connect an ST-LINK v2 programmer dongle to the sonde via the following pins:
  * SWDIO -> Pin 1
  * SWCLK -> Pin 7
  * GND -> Pin 5
  * 3.3V -> Pin 3
3. Unlock the flash protection - needed only before reprogramming the sonde for the first time
  * `openocd -f ./openocd_rs41.cfg -c "init; halt; flash protect 0 0 31 off; exit"`
4. Flash the firmware
  * `openocd -f ./openocd_rs41.cfg -c "program build/src/RS41ng.elf verify reset exit"`
5. Power cycle the sonde to start running the new firmware

## Developing / debugging the firmware

It is possible to receive log messages from the firmware program and to perform debugging of the firmware using GNU GDB.

Also, please note that Red Hat/Fedora do not provide GDB for ARM architectures, so you will need to manually download
and install GDB from [ARM GNU GCC toolchain](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-a/downloads/9-2-2019-12).

Semihosting allows the firmware to send log messages via special system calls to OpenOCD, so that you
can get real-time feedback and debug output from the application.

**Semihosting has to be disabled when the RS41 radiosonde is not connected to the STM32 programmer dongle,
otherwise the firmware will not run.**

### Steps to start firmware debugging and semihosting

1. Connect the RS41 radiosonde to a computer via the STM32 ST-LINK programmer dongle
2. Enable semihosting and logging in `config.h` by uncommenting the following lines
    ```
    #define SEMIHOSTING_ENABLE
    #define LOGGING_ENABLE
    ```
3. Start OpenOCD and leave it running in the background
    ```
    openocd -f ./openocd_rs41.cfg
    ```
4. Start ARM GDB
    ```
    arm-none-eabi-gdb
    ```
5. Connect GDB to OpenOCD for flashing and debugging (assumes you are in the `build` directory with Makefiles from CMake ready for build)
    ```
    target remote localhost:3333
    monitor arm semihosting enable
    make
    load src/RS41ng.elf
    monitor reset halt
    continue # this command runs the firmware
    ```
6. OpenOCD will output log messages from the firmware and GDB can be used to interrupt
   and inspect the firmware program.

To load debugging symbols for settings breakpoints and to perform more detailed inspection,
use command `file src/RS41ng.elf`.


## Debugging APRS

Here are some tools and command-line examples to receive and debug APRS messages using an
SDR receiver. There are examples for using both [rx_tools](https://github.com/rxseger/rx_tools)
and [rtl-sdr](https://github.com/osmocom/rtl-sdr) tools to interface with the SDR receiver.
The example commands assume you are using an RTL-SDR dongle, but `rx_fm` (from `rx_tools`)
supports other types of devices too, as it's based on SoapySDR.

### Dire Wolf

[Dire Wolf](https://github.com/wb2osz/direwolf) can decode APRS (and lots of other digital modes)
from audio streams.

rx_tools:

```bash
rx_fm -f 432500000 -M fm -s 250000 -r 48000 -g 22 -d driver=rtlsdr - | direwolf -n 1 -D 1 -r 48000 -b 16 -
```

rtl-sdr:

```bash
rtl_fm -f 432500000 -M fm -s 250k -r 48000 -g 22 - | direwolf -n 1 -D 1 -r 48000 -b 16 -
```


# Authors

* Luca Stella
* Original codebase: Mikael Nousiainen OH3BHX [RS41ng](https://github.com/mikaelnousiainen/RS41ng)
* Horus 4FSK code adapted from [darksidelemm fork of RS41HUP](https://github.com/darksidelemm/RS41HUP) project

# Additional documentation

## Vaisala RS41 hardware documentation

* https://github.com/bazjo/RS41_Hardware - Reverse-engineered documentation on the RS41 hardware
* https://github.com/bazjo/RS41_Decoding - Information about decoding the RS41 data transmission
* http://happysat.nl/RS-41/RS41.html - Vaisala RS-41 SGP Modification and info about the original firmware settings
* https://destevez.net/2018/06/flashing-a-vaisala-rs41-radiosonde/
* https://destevez.net/2017/11/tracking-an-rs41-sgp-radiosonde-and-reporting-to-aprs/

## Alternative firmware projects

* https://github.com/df8oe/RS41HUP - The original amateur radio firmware for RS41
* https://github.com/darksidelemm/RS41HUP - A fork of the original firmware that includes support for Horus 4FSK (but omits APRS)
* https://github.com/darksidelemm/RS41FOX - RS41-FOX - RS41 Amateur Radio Direction Finding (Foxhunting) Beacon
* http://www.om3bc.com/docs/rs41/rs41_en.html
