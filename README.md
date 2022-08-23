libmip
======

Libmip is a library for communicating with MicroStrain inertial sensors via the MIP protocol.


Features
--------

* Send commands using a single function
* Suitable for bare-metal microcontrollers
  * Minimal code size and memory footprint
  * No dynamic memory allocation
  * No dependence on any RTOS or threading
* Simple to interface with existing projects
  * FindMip.cmake is included for CMake-based projects
* Can be used to parse offline binary files
* C API for those who can't use C++
* C++ API for safety, flexibility, and convenience.

* Advanced Features
  * MIP packet creation
  * MIP packet parsing and field iteration
  * Data field deserialization

Examples
--------

* Get device information [C++] - queries the device strings and prints them to stdout.
* Watch IMU [C, C++] - Configures the IMU for streaming and prints the data to stdout.
* Product-specific examples:
  * GQ7 setup [C, C++] - Configures the device for typical usage.
  * CV7 setup [C, C++] - Configures the device for typical usage.
  * CV7 event [] - Configures a data trigger to output a message.

You'll need to enable at least one of the communications interfaces in the CMake configuration (see below) to use the examples.

The examples take two parameters for the device connection:
* For a serial connection: Port and baudrate. Port must start with `/dev/` on Linux or `COM` on Windows.
* For a TCP connection: Hostname and port. Hostname can be either a hostname like `localhost` or an IPv4 address.


Communications Interfaces
-------------------------

### Serial Port

A basic serial port interface which works on desktop PCs is included. This enables building desktop apps and
makes it easy to get started without needing to code any infrastructure first.

Enable it in the CMake configuration with `-DWITH_SERIAL=1`. This option defaults to enabled.

Use this interface for either USB or serial connections.

### TCP Client

A TCP client connection is provided with libmip. This allows remote development over
a network cable and makes it possible to insert other software into the communications path for debugging.

Enable it in the CMake configuration with `-DWITH_SOCKETS=1`. This option defaults to enabled.


How to Build
------------

### Prerequisites

* CMake version 3.10 or later
* A working C compiler
  * C99 or later is required
* A working C++ compiler
  * For C++ API only. Define `MIP_DISABLE_CPP=ON` if you don't want to use any C++.
  * C++11 or later is required for the core mip library
  * C++14 or later is requried for the examples (currently CMakeLists.txt assumes C++14 is required regardless)
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
* MIP_DISABLE_CPP - Ignores .hpp/.cpp files during the build and does not add them to the project.

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


Implementation Notes
--------------------

### C and C++ APIs

The C++ API is implemented on top of the C API to provide additional features:
* Object-oriented interfaces
* Improved type safety and sanity checking
* Better clarity / reduced verbosity (e.g. with `using namespace mip`)
* Simpler registration of data and field callbacks via the use of templates.

The C++ API uses `TitleCase` for typenames and `camelCase` for functions and variables, while the C api uses `snake_case` naming for
everything. This makes it easy to tell which is being used when looking at the examples.

The C API can be accessed directly from C++ via the `mip::C` namesace.

### User-Implemented Functions

There are two C functions which the user must implement to use this library.

The first, `mip_interface_user_update()`, must fetch raw data bytes from the connected MIP device. Typically this means reading from
a serial port or TCP socket. Once the data is read, it should be passed to `mip_interface_receive_bytes()` along with the associated
timestamp.

The second, `mip_interface_send_to_device()`, must pass the provided data bytes directly to the connected MIP device.

If compiling your application for C++, declare them as `extern "C"` to avoid linking problems.

If using the `MipDeviceInterface` class, you should instead subclass it and override the pure virtual `update` and `sendToDevice`
methods. The `MipDeviceInterface` class implements the C functions itself.

### Command Results (mip_cmd_result / MipCmdResult)

Command results are divided into two categories:
* Reply codes are returned by the device, e.g.:
  * ACK / OK
  * Invalid parameter
  * Unknown command
* Status codes are set by this library, e.g.:
  * General ERROR
  * TIMEDOUT
  * CANCELLED
  * Other statuses are used while the command is still in process

### Timestamps and Timeouts

Timestamps (`timestamp_type` / `Timestamp`) represent the local time when data was received or a packet was parsed. These timestamps
are used to implement command timeouts and provide the user with an approximate timestamp of received data. It is not intended to be
a precise timestamp or used for synchronization, and it generally cannot be used instead of the timestamps from the connected MIP device.
In particular, if you limit the maximum number of packets processed per `update` call, the timestamp of some packets may be delayed.

Because different applications may keep track of time differently (especially on embedded platforms), it is up to the user to provide
the current time whenever data is received from the device. On a PC, this might come from the posix `time()` function or from the
`std::chrono` library. On ARM Cortex-M systems, it is often derived from the Systick timer.

Typically timestamps and timeouts are specified in milliseconds, but this is not required. Any unit of time can be used as long as
it is used consistently. It must be small enough to achieve the desired precision, but large enough to not overflow too often.
Whole seconds may be too coarse (command timeouts may be longer than desired), but nanoseconds will overflow a `uint32_t` timestamp
in just 4 seconds.

By default, timestamps are `typedef`'d to `uint64_t` for use on a PC. This simplifies debugging because it will not overflow in this
century, even if the unit is nanoseconds. Embedded systems may wish to use `uint32_t` instead to avoid 64-bit math. The value is
allowed to wrap around (overflow), as long as the time taken to overflow exceeds twice the maximum command timeout used in the
application. E.g. a `uint32_t` timestamp in milliseconds will overflow about every 50 days, and about once every 72 minutes if the
units are microseconds.

Timeouts for commands are broken down into two parts.
* A "base reply timeout" applies to all commands. This is useful to compensate for communication latency, such as over a TCP socket.
* "Additional time" which applies per command, because some commands may take longer to complete.

Currently, only the C++ api offers a way to set the additional time parameter, and only when using the command `struct`s with
`MipInterface::runCommand`.
