MSCL-Embedded
=============

Welcome to the Embedded version of the MicroStrain Communication Library.


Features
--------

* Send commands using a single function
* MIP packet creation
* MIP packet parsing and field iteration
* Data field deserialization
* Suitable for bare-metal microcontrollers
  * Small code size and memory footprint
  * No dynamic memory allocation
  * No dependence on any RTOS or threading
* Simple to interface with existing projects
* Can be used to parse offline binary files
* C and C++ interfaces


Examples
--------

* Get device information - queries the device strings and prints them to stdout.
* Watch IMU - Configures the IMU for streaming and prints the data to stdout.

You'll need to enable at least one of the communications interfaces in the CMake configuration (see below) to use the examples.

The examples take two parameters for the device connection:
* For a serial connection: Port and baudrate. Port must start with `/dev/` on Linux or `COM` on Windows.
* For a TCP connection: Hostname and port. Hostname can be either a hostname like `localhost` or an IPv4 address.


Communications Interfaces
-------------------------

### Serial Port

A serial port library is included via git submodule. To use it, you'll have to download this additional library:
`git submodule init`

Enable it in the CMake configuration with `-DWITH_SERIAL=1`.

### TCP Client

A TCP client connection is provided with MSCL-Embedded. This allows remote development over
a network cable and makes it possible to insert other software into the communications path for debugging.

Enable it in the CMake configuration with `-DWITH_SOCKETS=1`.


How to Build
------------

### Prerequisites

* CMake version 3.10 or later
* A working C compiler
* Doxygen, if building documentation

### Build configuration

The following options may be specified when configuring the build with CMake (e.g. `cmake .. -DOPTION=VALUE`):
* WITH_SERIAL - Builds the included serial port library (default enabled).
* WITH_SOCKETS - Builds the included socket library (default enabled).
* BUILD_EXAMPLES - If enabled (`-DBUILD_EXAMPLES=ON`), the example projects will be built (default disabled).
* BUILD_TESTING - If enabled (`-DBUILD_TESTING=ON`), the test programs in the /test directory will be compiled and linked. Run the tests with `ctest`.
* BUILD_DOCUMENTATION - If enabled, the documentation will be built with doxygen. You must have doxygen installed.
* BUILD_DOCUMENTATION_FULL - Builds internal documentation (default disabled).
* BUILD_DOCUMENTATION_QUIET - Suppress standard doxygen output (default enabled).

### Compilation for Linux

1. Create the build directory (e.g. `mkdir build`).
2. In the build directory, run `cmake .. -G 'Unix Makefiles' <options>`
   * Replace `<options>` with your configuration options, such as `-DWITH_SERIAL=1`.
   * You can use `cmake-gui ..` instead if you'd prefer to use the GUI tool (and have it installed).
   * An alternative generator may be used, such as ninja, code blocks, etc.
3. Invoke `make` in the build directory (if using the `Unix Makefiles` generator).
4. (Optional, if BUILD_TESTS was enabled) Run `ctest` to verify the test cases pass.

### Compilation for Windows

Todo

### Compilation for Embedded MCU

Todo
