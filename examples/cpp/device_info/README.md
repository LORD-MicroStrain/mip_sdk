# Device Info Example (C++)

This example demonstrates how to retrieve and display device information from any MIP-enabled MicroStrain device with 
the MIP SDK using the C++ API.

## Overview

The example showcases basic device communication and information retrieval, including:
- Device initialization and communication
- Device information querying
- Display of comprehensive device details

## Configurable Options

The example uses the following default settings, which should be adjusted based on application requirements:

| Setting     | Value                                         | Description                          |
|-------------|-----------------------------------------------|--------------------------------------|
| `PORT_NAME` | `"COM1"` (Windows)<br>`"/dev/ttyACM0"` (Unix) | Serial port for device communication |
| `BAUDRATE`  | `115200`                                      | Communication baud rate              |

## Usage

1. Connect your MIP-enabled MicroStrain device to the specified serial port
2. Update the [configuration options](#configurable-options) based on your application needs
3. Compile and run the example
4. The program will:
    - Initialize the device
    - Retrieve device information
    - Display formatted device details
    - Clean up and exit

## Building

### With CMake (Recommended)

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

 If the project cannot be configured using CMake, then the following project configurations are required:

#### Required Libraries

Link against these libraries:
- `mip` - Core MIP SDK library
- `microstrain` - Core MicroStrain SDK library
- `microstrain_serial` - MicroStrain serial communication library

Make sure to include those library paths as additional link directories if needed

#### Include Directories

Add these include directories:
- `[path_to_mip_sdk_include]/c`
- `[path_to_mip_sdk_include]/cpp`
- `[path_to_project_root]`

`path_to_mip_sdk_include` can be installed paths or source paths:
- Unix - `/usr/include/microstrain`
- Windows - `C:/Program Files/MIP_SDK/include/microstrain`
- Source: `[mip_sdk_project_root]/src`

#### Compiler Definitions
Add these compiler definitions:
- `MICROSTRAIN_LOGGING_MAX_LEVEL=MICROSTRAIN_LOGGING_LEVEL_INFO_` Sets the logging level to info which is the minimum required for this example

## Key Functions

### Device Setup
- `initializeDevice()` - Establishes communication, and validates device connection
- `getDeviceInformation()` - Queries and retrieves comprehensive device information

### Device Information Display
- Displays formatted device details including:
    - Model name and number
    - Serial number and lot number
    - Device options and capabilities
    - Firmware version information

### Communication Interface
- Uses the `mip::Interface` class for device communication
- Serial connection handled by `microstrain::connections::SerialConnection`

## Data Handling

This example uses modern C++ features including:
- **Type Safety**: Strongly typed data structures for each message type
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

## Device Information Retrieved

The example displays the following device information:
- **Model Name**: Human-readable device model name
- **Model Number**: Specific model identifier
- **Serial Number**: Unique device serial number
- **Lot Number**: Manufacturing lot information
- **Device Options**: Available device features and capabilities
- **Firmware Version**: Current firmware version (formatted as x.x.xx)

### Output Format

The example displays the device information in the following format:
```
-------- Device Information --------
Name             |      3DM-CV7-INS
Model Number     |        6291-9960
Serial Number    |      6291.123456
Lot Number       |      6-5-2023_10
Options          |  4-16g,250-1kdps
Firmware Version |           1.2.04
------------------------------------
```

## Error Handling

The example includes comprehensive error handling with:
- Command result checking using `mip::CmdResult`
- Connection failure detection and recovery
- Graceful termination functions for different error types
- Detailed error messages with context using built-in documentation strings

## C++ Features

This example demonstrates:
- Modern C++ connection management
- Type-safe MIP command interfaces
- RAII resource management
- Standard library integration

## Type Safety and Documentation

This example provides additional C++ benefits:
- **Built-in Documentation**: Data structures include `DOC_NAME` constants for easy reference
- **Strongly Typed Enums**: C++ enum classes prevent accidental misuse
- **Automatic Descriptors**: `DESCRIPTOR` constants eliminate magic numbers

## Requirements

- Any MIP-enabled MicroStrain device
- Serial connection (USB or RS-232)
- MIP SDK library with C++ support
- C++11 or later compiler

## See Also

- C version: `device_info_example.c`
- Other examples in the `examples/` directory
- [MIP SDK documentation](https://lord-microstrain.github.io/mip_sdk_documentation/)
