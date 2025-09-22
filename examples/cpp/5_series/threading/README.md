# 5 Series Threading Example (C++)

This example demonstrates how to implement multithreading for simultaneous data collection and command operations on
MicroStrain 5-series devices using the C++ API.

## Overview

The example showcases basic threading concepts for 5-series devices, including:
- Device initialization and communication
- Sensor message format configuration
- Concurrent data collection and command execution
- Thread-safe device communication
- Race condition prevention and stress testing
- Modern C++ threading primitives
- Proper thread synchronization and cleanup

## Configuration

The example uses the following default settings:

| Setting            | Value                                         | Description                             |
|--------------------|-----------------------------------------------|-----------------------------------------|
| `PORT_NAME`        | `"COM1"` (Windows)<br>`"/dev/ttyACM0"` (Unix) | Serial port for device communication    |
| `BAUDRATE`         | `115200`                                      | Communication baud rate                 |
| `SAMPLE_RATE_HZ`   | `1`                                           | Data output rate in Hz                  |
| `RUN_TIME_SECONDS` | `30`                                          | Example runtime duration                |
| `USE_THREADS`      | `true`                                        | Enable/disable threading (compile-time) |

## Key Functions

### Device Setup
- `initializeDevice()` - Establishes communication, validates device connection, and loads defaults

### Message Configuration
- `configureSensorMessageFormat()` - Configures IMU sensor data output including:
    - Scaled accelerometer

### Threading Infrastructure
- `updateDevice()` - Custom update function that handles thread context switching
- `dataCollectionThread()` - Dedicated thread function for continuous data collection
- `packetCallback()` - Processes complete MIP packets and displays field information

### Communication Interface
- Uses the `mip::Interface` class for device communication
- Serial connection handled by `microstrain::connections::SerialConnection`

## Threading Architecture

This example uses modern C++ threading features:
- **std::thread**: Standard C++ threading library
- **Thread References**: Safe reference passing to thread functions
- **Volatile Flags**: Thread-safe communication using volatile references
- **Automatic Cleanup**: RAII-based resource management
- **Exception Safety**: Proper error handling across threads

## Threading Implementation

This example showcases:
- **Modern C++ Threading**: std::thread, std::chrono, and threading utilities
- **RAII Thread Management**: Automatic thread resource management
- **Reference Semantics**: Safe reference passing between threads
- **Chrono Time Management**: Modern C++ time handling

## Thread Safety Features

The example includes comprehensive thread safety:
- **Single Command Channel**: Only one thread can send commands at a time
- **Data Collection Isolation**: Separate thread for continuous data processing
- **Connection State Management**: Safe handling of connection failures across threads
- **Graceful Shutdown**: Proper thread termination and resource cleanup

## Data Handling

This example uses modern C++ features including:
- **Type-Safe Callbacks**: Compile-time verified callback functions
- **Range-Based Loops**: Modern iteration over packet fields
- **String Handling**: Safe C++ string operations

## C++ Implementation Features

This example demonstrates:
- **MIP Interface**: Modern C++ interface for device communication (`mip::Interface`)
- **Modern C++ Connection Management**: RAII-based resource handling
- **Type-Safe MIP Command Interfaces**: Compile-time type checking
- **Exception Safety**: Proper error handling and resource cleanup
- **STL Integration**: Use of standard library containers and algorithms
- **Portability**: Cross-platform compatibility (Windows/Unix)

## Connection Management

This example uses modern C++ connection handling:
- **SerialConnection**: RAII-based serial connection management
- **Automatic Cleanup**: Connection automatically closed when the object goes out of scope

## Usage

1. Connect your 5-series device to the specified serial port
2. Update the `PORT_NAME` constant if using a different port
3. Set `USE_THREADS` to `true` to enable threading (default)
4. Compile and run the example
5. The program will:
    - Initialize the device and threading infrastructure
    - Start a dedicated data collection thread
    - Execute stress testing with rapid ping commands
    - Display received packet information
    - Perform thread cleanup and exit

## Error Handling

The example includes comprehensive error handling with:
- Command result checking using `mip::CmdResult`
- Connection failure detection and recovery
- Graceful termination functions for different error types
- Detailed error messages with context using built-in documentation strings
- Exception-safe thread operations

## C++ Features

This example demonstrates:
- Modern C++ connection management
- Type-safe MIP command interfaces
- Automatic data field extraction
- RAII resource management
- Standard library integration

## Type Safety and Documentation

This example provides additional C++ benefits:
- **Built-in Documentation**: Data structures include `DOC_NAME` constants for easy reference
- **Strongly Typed Enums**: C++ enum classes prevent accidental misuse
- **Automatic Descriptors**: `DESCRIPTOR` constants eliminate magic numbers

## Building

### With CMake

The project can be configured on its own using the supplied [CMakeLists.txt](CMakeLists.txt).
The file is configured to work directly in the MIP SDK project or as a standalone project.
If building outside the MIP SDK project, all that's needed is to define `MIP_SDK_ROOT_DIR`.
When building within the MIP SDK project, make sure to enable the examples using the `MICROSTRAIN_BUILD_EXAMPLES`
CMake option.

#### Standalone Command Line
```shell
mkdir build
cd build
cmake .. -DMIP_SDK_ROOT_DIR:PATH=<path_to_mip_sdk>
```

### Without CMake

When building manually, you need to configure the following:

#### Required Libraries

Link against these libraries:
- `mip` - Core MIP SDK library
- `microstrain` - Core MicroStrain SDK library
- `microstrain_serial` - MicroStrain serial communication library
- `pthread` - Threading library (not MSVC)

Make sure to include those library paths as additional link directories if needed

#### Include Directories

Add these include directories:
- `<path_to_mip_sdk_include>/c`
- `<path_to_mip_sdk_include>/cpp`
- `<path_to_project_root>`

<path_to_mip_sdk_include> can be installed paths or source paths:
- Unix - `/usr/include/microstrain`
- Windows - `C:/Program Files/MIP_SDK/include/microstrain`
- Source: `<mip_sdk_project_root>/src`

#### Compiler Definitions
Add these compiler definitions:
- `MICROSTRAIN_LOGGING_MAX_LEVEL=MICROSTRAIN_LOGGING_LEVEL_INFO_` Sets the logging level to info which is the minimum required for this example

## Requirements

- MicroStrain 5-series device
- Serial connection (USB or RS-232)
- MIP SDK library with C++ support
- C++11 or later compiler
- std::thread support

## See Also

- C version: `5_series_threading_example.c`
- Other examples in the `examples/` directory
- [MIP SDK documentation](https://lord-microstrain.github.io/mip_sdk_documentation/)
- C++ threading and concurrency documentation
