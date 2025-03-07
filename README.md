MIP SDK
=======

Welcome to the official MIP Software Development Kit.

If you have any questions or run into any issues, please let us know! [MicroStrain Support Portal](https://support.microstrain.com)


Features
--------

* Send commands using a single function
* Suitable for bare-metal microcontrollers
  * Minimal code size and memory footprint
  * No dynamic memory allocation
  * No dependence on any RTOS or threading
* Simple to interface with existing projects
  * FindMip.cmake is included for CMake-based projects
* It can be used to parse offline binary files
* C++ API for safety, flexibility, and convenience.
* C API for those who can't use C++

* Advanced Features
  * MIP packet creation
  * MIP packet parsing and field iteration
  * Data field deserialization

Examples
--------

| Example                          |                                                                   |                                                              | Description                                                                                  |
|----------------------------------|-------------------------------------------------------------------|--------------------------------------------------------------|----------------------------------------------------------------------------------------------|
| <b>Generic Examples</b>          |                                                                   |                                                              |                                                                                              |
| Get device information           | [C++](./examples/device_info.cpp)                                 |                                                              | Queries the device strings and prints them to stdout.                                        |
| Watch IMU                        | [C++](./examples/watch_imu.cpp)                                   | [C](./examples/watch_imu.c)                                  | Configures the IMU for streaming and prints the data to stdout.                              |
| Threading                        | [C++](./examples/threading.cpp)                                   |                                                              |                                                                                              |
| Ping                             | [C++](./examples/ping.cpp)                                        |                                                              |                                                                                              |
| <b>Product-specific examples</b> |                                                                   |                                                              |                                                                                              |
| CV7 setup                        | [C++](./examples/CV7/CV7_example.cpp)                             | [C](./examples/CV7/CV7_example.c)                            | Configures a CV7 device for typical usage and includes an example of using the event system. |
| CV7_INS setup                    | [C++](./examples/CV7_INS/CV7_INS_simple_example.cpp)              |                                                              | Configures a CV7_INS device for typical usage.                                               |
| CV7_INS with UBlox setup         | [C++](./examples/CV7_INS/CV7_INS_simple_ublox_example.cpp)        |                                                              | Configures a CV7_INS device for typical usage with aiding data from a UBlox GNSS receiver.   |
| GQ7 setup                        | [C++](./examples/GQ7/GQ7_example.cpp)                             | [C](./examples/GQ7/GQ7_example.c)                            | Configures a GQ7 device for typical usage in a wheeled-vehicle application.                  |
| GX5-45 setup                     | [C++](./examples/GX5_45/GX5_45_example.cpp)                       | [C](./examples/GX5_45/GX5_45_example.c)                      | Configures a GX5-45 device for typical usage in a wheeled-vehicle application.               |
| GX5/CX5/CV5 -15 or -25 setup     | [C++](./examples/CX5_GX5_CV5_15_25/CX5_GX5_CV5_15_25_example.cpp) | [C](./examples/CX5_GX5_CV5_15_25/CX5_GX5_CV5_15_25_example.c) | Configures a GX5-45 device for typical usage in a wheeled-vehicle application.               |

You'll need to enable at least one of the [communications interfaces](#communications-interfaces) in the CMake configuration to use the examples.

The examples take two parameters for the device connection:
* For a serial connection: Port and baudrate. Port must start with `/dev/` on Linux or `COM` on Windows.
* For a TCP connection: Hostname and port. Hostname can be either a hostname like `localhost` or an IPv4 address.


Documentation
-------------

Documentation for all released versions can be found [here](https://lord-microstrain.github.io/mip_sdk_documentation).

### C and C++ APIs

The C++ API is implemented on top of the C API to provide additional features:
* Object-oriented interfaces
* Improved type safety and sanity checking
* Better clarity / reduced verbosity (e.g. with `using namespace mip`)

The C++ API uses `TitleCase` for types and `camelCase` for functions and variables, while the C api uses `snake_case` naming for
everything. This makes it easy to tell which is being used when looking at the examples.

The C API can be accessed directly from C++ via the `mip::C` namespace.

### Command Results

MIP devices return an ack/nack field in response to commands to allow the user to determine if the command was
successfully executed. These fields contain a "reply code" which is defined by the MIP protocol. This library
additionally defines several "status codes" for situations where an ack/nack field is not applicable (i.e. if
the device doesn't respond to the command, if the command couldn't be transmitted, etc).

See the documentation page for [Command Results](https://lord-microstrain.github.io/mip_sdk_documentation/latest/command_results.html) for details.

### Timestamps

In order to implement command timeouts and provide time of arrival information, this library requires applications to
provide the time of received data. The time must be provided as an unsigned integral value with a reasonable precision,
typically milliseconds since program startup. By default the timestamp type is set to `uint64_t`, but some embedded
applications may wish to change this to `uint32_t` via the `MICROSTRAIN_TIMESTAMP_TYPE` define. Note that wraparound is
permissible if the wraparound period is longer than twice the longest timeout used by the application.

See the documentation page for [Timestamps](https://lord-microstrain.github.io/mip_sdk_documentation/latest/timestamps.html).


Communications Interfaces
-------------------------

Two connection types are provided with the MIP SDK to make it easy to run the examples on both Windows and Linux systems.

### Serial Port

A basic serial port interface is provided in C and C++ for Linux, Mac, and Windows. These can be modified for other platforms by the user.
The serial port connection will be used in most cases, when the MIP device is connected
via a serial or USB cable (the USB connection acts like a virtual serial port).

[Enable it](#build-configuration) in the CMake configuration with `-DMICROSTRAIN_ENABLE_SERIAL=1`.

### TCP Client

The TCP client connection allows you to connect to a MIP device remotely. The MIP device must be connected
via the normal serial or USB cable to a computer system running a TCP server which forwards data between
the serial port and TCP clients.

[Enable it](#build-configuration) in the CMake configuration with `-DMICROSTRAIN_ENABLE_TCP=1`.


How to Build
------------

### Prerequisites

* A working C compiler
  * C11 or later required
  * Define `MICROSTRAIN_ENABLE_CPP=OFF` if you don't want to use any C++. Note that some features are only available in C++.
* A working C++ compiler, if using any C++ features
  * C++14 or later is required.
  * C++20 or later for metadata and associated examples
* CMake version 3.10 or later (technically this is optional, see below)
* Doxygen, if building documentation

### CMake Build Configuration

The following options may be specified when configuring the build with CMake (e.g. `cmake .. -DOPTION=VALUE`):

| CMake Option                          | Default                    | Description                                                                                                                                                                                                                                                 |
|---------------------------------------|----------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| MICROSTRAIN_ENABLE_LOGGING            | ON                         | Builds logging functionality into the library. The user is responsible for configuring a logging callback.                                                                                                                                                  |
| MICROSTRAIN_LOGGING_MAX_LEVEL         | MICROSTRAIN_LOG_LEVEL_WARN | Max log level the SDK is allowed to log. If this is defined, any log level logged at a higher level than this will result in a noop regardless of runtime configuration. Useful if you want some logs, but do not want the overhead compiled into the code. |
| MICROSTRAIN_TIMESTAMP_TYPE            | uint64_t                   | Overrides the default timestamp type. See [Timestamps](https://lord-microstrain.github.io/mip_sdk_documentation/latest/timestamps.html) in the documentation.                                                                                                                                                                           |
| MICROSTRAIN_ENABLE_CPP                | ON                         | Causes the src/cpp directory to be included in the build. Disable to turn off the C++ api.                                                                                                                                                                  |
| MICROSTRAIN_ENABLE_EXTRAS             | ON                         | Builds some higher level utility classes and functions that may use dynamic memory.                                                                                                                                                                         |
| MICROSTRAIN_ENABLE_SERIAL             | ON                         | Builds the included serial port library.                                                                                                                                                                                                                    |
| MICROSTRAIN_ENABLE_TCP                | ON                         | Builds the included socket library (default enabled).                                                                                                                                                                                                       |
| MICROSTRAIN_BUILD_PACKAGE             | OFF                        | Adds a `package` target to the project that will build a `.deb`, `.rpm`, or `.zip` file containing the library                                                                                                                                              |
| MICROSTRAIN_BUILD_EXAMPLES            | OFF                        | If enabled, the example projects will be built.                                                                                                                                                                                                             |
| MICROSTRAIN_BUILD_TESTS               | OFF                        | If enabled, the test programs in the /test directory will be compiled and linked. Run the tests with `ctest`.                                                                                                                                               |
| MICROSTRAIN_BUILD_DOCUMENTATION       | OFF                        | If enabled, the documentation will be built with doxygen. You must have doxygen installed.                                                                                                                                                                  |
| MICROSTRAIN_BUILD_DOCUMENTATION_FULL  | OFF                        | Builds internal documentation.                                                                                                                                                                                                                              |
| MICROSTRAIN_BUILD_DOCUMENTATION_QUIET | ON                         | Suppress standard doxygen output.                                                                                                                                                                                                                           |
| MIP_ENABLE_DIAGNOSTICS                | ON                         | Adds some counters to various entities which can serve as a debugging aid.                                                                                                                                                                                  |
| MIP_ENABLE_METADATA                   | ON if supported            | Builds metadata for MIP commands. If not set, the system will try to determine if C++20 is available to enable it. C++20 is required for the metadata module.                                                                                               |
| MIP_ENABLE_EXTRAS                     | MICROSTRAIN_ENABLE_EXTRAS  | Builds some higher level utility classes and functions that may use dynamic memory. (default MICROSTRAIN_ENABLE_EXTRAS)                                                                                                                                     |

### Compilation 

1. Create the build directory (e.g. `mkdir build`).
2. In the build directory, run `cmake .. <options>`
   * Replace `<options>` with your configuration options, such as `-DMICROSTRAIN_ENABLE_SERIAL=1`.
   * You can use `cmake-gui ..` instead if you'd prefer to use the GUI tool (and have it installed).
   * An alternative generator may be used, such as ninja, code blocks, etc. by specifying `-G <generator>`
3. Invoke `cmake --build .` in the build directory
4. (Optional, if MICROSTRAIN_BUILD_PACKAGE was enabled) Run `cmake --build . --target package` to build the packages.

### Building without CMake

If your target platform doesn't support CMake, you can build the project without it. To do so,
include all the necessary files and define a few options.

#### Minimum Required Files for building without CMake
##### C only
* All source files in `src/c/microstrain`, except logging.c if logging is disabled
* Source files in `src/c/microstrain/connections` for your required connection types
* All source files in `src/c/mip` and `src/c/mip/utils`
* All source files in `src/c/mip/definitions` (or at least all the required descriptor sets)
##### C++
* The C files indicated above, except those in `definitions` (they can be added too but aren't required)
* Source files in `src/cpp/microstrain/connections` for your required connection types
* All source files in `src/cpp/microstrain`
* All source files in `src/cpp/mip/definitions` (or at least the required descriptor sets)
* Source files in `src/cpp/mip/extras` as needed for your project
* Source files in `src/cpp/mip/metadata` if using metadata

Note: When using MSVC with C++, build the C and C++ code separately as libraries. Some C and C++ files
have the same name, and MSVC will try to compile both to the same object file. This causes one to
be overwritten by the other. Building as separate libraries causes the object files to be placed in
different subdirectories, avoiding the conflict.

#### Required #defines for building without CMake

Pass these to your compiler as appropriate, e.g. `arm-none-eabi-gcc -DMICROSTRAIN_TIMESTAMP_TYPE=uint32_t -DMICROSTRAIN_ENABLE_LOGGING=0`

These defines must be set when building the MIP SDK sources and for any code that includes
MIP SDK headers.

| Name                          | Description                                                                          | 
|-------------------------------|--------------------------------------------------------------------------------------|
| MICROSTRAIN_ENABLE_LOGGING    | Enables logging facilities (e.g. via printf).                                        |
| MICROSTRAIN_LOGGING_MAX_LEVEL | Disables logging more detailed messages (e.g. debug tracing) to improve performance. |
| MICROSTRAIN_TIMESTAMP_TYPE    | Type to use for mip::Timestamp / mip::Timeout. Defaults to uint64_t if not defined.  |
| MICROSTRAIN_USE_STD_ENDIAN    | Define to 1 to enable the use of std::span from C++20.                               |
| MICROSTRAIN_USE_STD_SPAN      | Define to 1 to enable the use of std::endian from C++20.                             |
| MIP_ENABLE_DIAGNOSTICS        | Enables diagnostic counters.                                                         |

These options affect the compiled code interface and sizes of various structs. They
MUST be consistent between compiling the MIP SDK and any other code which includes
headers from the MIP SDK. (If you change them after building, make sure everything gets
rebuilt properly. Normally CMake takes care of this for you).

#### Include directories
* `src/c`
* `src/cpp` (if using C++)
* `examples` (if building/using example code)

#### Other compiler settings
* C++14 or later is required (if using C++)
  * For gcc: --std=c++14
  * For msvc: /std:c++14
  * Use C++17 or later if using metadata.
* If building TCP socket support on Windows, link against ws2_32.lib.

## How to use the pre-packaged libraries
* Available Modules:
  * microstrain
  * microstrain_extras (CPP only)
  * microstrain_recording_connection (CPP only)
  * microstrain_serial
  * microstrain_socket
  * mip
  * mip_extras (CPP only)
  * mip_metadata (CPP only)

### Including the packages using CMake

We have included CMake find_package config files with the package for easy project integration.<br>
The simplest way to include all desired modules is to find the MIP package and request the desired modules as components.<br>
Each module has an associated config file which defines `<NAME>_LIBRARY` `<NAME>_LIBRARIES` and `<NAME>_INCLUDE_DIRS`.<br>
When using find_package, the name of the package should be all lowercase.

#### Minimum requirements for CMake projects
* Include the installation directory in `CMAKE_PREFIX_PATH` (Usually not required on Linux)
* Find the package in `CONFIG` mode
* Link the libraries and include directories to your target/project

```cmake
# Find the desired package/module
find_package(mip COMPONENTS <list_of_modules> CONFIG)

# Link the libraries
target_link_libraries(<target_name> PUBLIC ${MIP_LIBRARIES})

# Add the include directories
target_include_directories(<target_name> PUBLIC ${MIP_INCLUDE_DIRS})
```

You may need to add the installation directory to the CMake prefix path if it cannot find the package.<br>
This is usually not required on Linux.

 ```cmake
# Set the installation directory to the package
set(MIP_SDK_DIR <path_to_installation_dir>)

# Add the directory to the CMake prefix paths to search for the package
list(APPEND CMAKE_PREFIX_PATH ${MIP_SDK_DIR})

# Find the desired package/module
find_package(...)
```

### Including the packages without CMake

If you want to include the MIP SDK without a CMake project, you will need to include the installation <br>
directory in your project configuration.

#### Minimum requirements for projects without CMake
* Include the library directory `<path_to_installed_package>/lib`
* Link the libraries/modules
* Add the include directory `<path_to_installed_package>/include/microstrain`

Please note, all libraries work with C, but only some work with C++

Known Issues
------------

* `suppress_ack=true` is not supported.
* The commanded BIT, device settings, and capture gyro bias commands can time out unless the timeout is increased.

See the documentation page for [Known Issues](https://lord-microstrain.github.io/mip_sdk_documentation/latest/other.html#known_issues).
