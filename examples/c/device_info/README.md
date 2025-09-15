# Device Info Example (C)

This example demonstrates how to retrieve and display device information from any MIP-enabled MicroStrain device with
the MIP SDK using the C API.

## Overview

The example showcases basic device communication and information retrieval, including:
- Device initialization and communication
- Device information querying
- Display of comprehensive device details

## Configuration

The example uses the following default settings:

| Setting     | Value                                         | Description                          |
|-------------|-----------------------------------------------|--------------------------------------|
| `PORT_NAME` | `"COM1"` (Windows)<br>`"/dev/ttyACM0"` (Unix) | Serial port for device communication |
| `BAUDRATE`  | `115200`                                      | Communication baud rate              |

## Key Functions

### Device Setup
- `initialize_device()` - Establishes serial communication and validates device connection
- `get_device_information()` - Queries and retrieves comprehensive device information

### Device Information Display
- Displays formatted device details including:
    - Model name and number
    - Serial number and lot number
    - Device options and capabilities
    - Firmware version information

### Communication Interface
- Uses the `mip_interface` struct for device communication
- Serial connection handled by `serial_port`

## Data Handling

This example demonstrates traditional C programming patterns:
- **Manual Parsing**: Direct parsing of incoming MIP packets using the MIP parser
- **String Handling**: Safe C string operations for device information
- **Explicit Memory Management**: Manual buffer and resource management
- **Type Safety**: Uses C structs and enums for data type safety
- **Error Codes**: Command results handled through return codes and error checking

## C Implementation Features

This example showcases:
- **MIP Interface**: Core C interface for device communication (`mip_interface`)
- **Serial Port Management**: Low-level serial port operations
- **Command Handling**: Direct MIP command execution
- **Memory Safety**: Careful buffer management and bound checking
- **Portability**: Cross-platform compatibility (Windows/Unix)

## Device Information Retrieved

The example displays the following device information:
- **Model Name**: Human-readable device model name
- **Model Number**: Specific model identifier
- **Serial Number**: Unique device serial number
- **Lot Number**: Manufacturing lot information
- **Device Options**: Available device features and capabilities
- **Firmware Version**: Current firmware version (formatted as x.x.xx)

## Custom Communication Functions

The example implements custom communication handlers:
- **Send Function**: `mip_interface_user_send_to_device()` handles outgoing data
- **Receive Function**: `mip_interface_user_recv_from_device()` manages incoming data
- **Timeout Handling**: Configurable timeouts for reliable communication

## Usage

1. Connect your MIP-enabled MicroStrain device to the specified serial port
2. Update the `PORT_NAME` constant if using a different port
3. Compile and run the example
4. The program will:
    - Initialize the device
    - Retrieve device information
    - Display formatted device details
    - Clean up and exit

## Error Handling

The example includes comprehensive error handling with:
- Command result checking using `mip_cmd_result`
- Connection failure detection and recovery
- Graceful termination functions for different error types
- Detailed error messages with context

## C Features

This example demonstrates:
- Standard C11 programming practices
- Minimal external dependencies
- Direct hardware interface programming
- Efficient memory usage patterns
- Cross-platform serial communication

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
- `microstrain_serial` - MicroStrain serial communication library

Make sure to include those library paths as additional link directories if needed

#### Include Directories

Add these include directories:
- `<path_to_mip_sdk_include>/c`
- `<path_to_project_root>`

<path_to_mip_sdk_include> can be installed paths or source paths:
- Unix - `/usr/include/microstrain`
- Windows - `C:/Program Files/MIP_SDK/include/microstrain`
- Source: `<mip_sdk_project_root>/src`

#### Compiler Definitions
Add these compiler definitions:
- `MICROSTRAIN_ENABLE_LOGGING=1` Enables the logging feature
- `MICROSTRAIN_LOGGING_MAX_LEVEL=MICROSTRAIN_LOG_LEVEL_INFO` Sets the logging level to info which is the minimum required for this example

## Requirements

- Any MIP-enabled MicroStrain device
- Serial connection (USB or RS-232)
- MIP SDK library with C support
- C11 or later compiler

## See Also

- C++ version: `device_info_example.cpp`
- Other examples in the `examples/` directory
- [MIP SDK documentation](https://lord-microstrain.github.io/mip_sdk_documentation/)
