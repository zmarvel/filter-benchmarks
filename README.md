
# Filter benchmarks

The goal of this project is to provide energy benchmarks for various digital
filters implemented in the ARM CMSIS library.

The first step is to identify which filters should be benchmarked.

## Measurement strategy

For different filters,

1. [Count cycles][count_cycles]
2. Using a shunt resistor, amplifier, and oscilloscope, measure actual energy
consumption.

## Hardware

The initial target is an STM32L432KC Nucleo-32. This is an ARM Cortex M4, chosen
for the low-power architecture and the inclusion of a floating point unit.

## Building

To build the project, make sure you have `arm-none-eabi-gcc` installed. In
Debian, it can be found in the `gcc-arm-none-eabi` package.

In the `Makefile`, you will need to change the path to your STM32L4 HAL:

```
PERIFLIB_PATH = /home/zack/STM32Cube/Repository/STM32Cube_FW_L4_V1.11.0/
```

## Downloading code

To flash the board, you will need OpenOCD installed (in Debian, the package is
simply `openocd`). If OpenOCD is installed to an unusual location (if the
script cannot find it), you may specify it in `.openocd_env`, like so:

```
OPENOCD=/usr/local/bin/openocd
OPENOCD_SCRIPTS=/usr/local/share/openocd/scripts/
```

Then you can run the `download` script:

```
$ ./download build/filter_benchmarks.elf
```

To interact with OpenOCD over telnet or to connect a debugger to the target,
you can run the `openocd` script:

```
$ ./openocd
```


[count_cycles]: (http://embeddedb.blogspot.com/2013/10/how-to-count-cycles-on-arm-cortex-m.html)
