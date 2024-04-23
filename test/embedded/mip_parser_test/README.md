# Embedded Parser Test

This is a mini-project which tests the MIP parser performance on an STM32F767 Nucleo board.

## Program Description

A global array of packet bytes is constructed by repeatedly appending a static data packet.
This array is then fed into the parser by making one or more calls to `mip_parser_parse`.
During this process, the GPIO pin connected to the blue LED is raised high, turning the LED
on. This pin (PB7) can be probed with an oscilloscope to measure the total time taken.
Once the buffer has been completely parsed, the program waits for the user to press the
blue button before running again. The number of parsed packets is checked against the
expected value. If it matches, the green LED is lit, and otherwise the red LED is lit.

## Build process
To make changes to the MCU configuration (for example, to change clock speed), open the
`mip_parser_test.ioc` project in STM32CubeMX. Make your changes, then click "generate code".
This will regenerate all of the code and build files. User code will not be overwritten
if placed appropriately between the commented areas. CLion was used to compile and run the
project, though this shouldn't be necessary since it's based on CMake.

Note: STM32CubeMX tends to put hardcoded paths in `cmake/stm32cubemx/CMakeLists.txt`. This
is bad for portability and you should NOT commit changes in this file without first
reviewing and/or editing it. Most of the time you can just revert the file after
regenerating the project code.

## Requirements
* STM32F767ZI Nucleo-144 dev board and micro-USB cable
* STM32CubeMX if you want to reconfigure the peripherals using the UI
* STM32Cube Firmware package for STM32F7 (can be installed from STM32CubeMX)
* CMake (3.26 was tested, earlier versions may work)
* GNU Arm Embedded toolchain (arm-none-eabi-gcc) or other suitable compiler
* OpenOCD or another way to download the code to the MCU

