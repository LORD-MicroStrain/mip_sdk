# Device Info Example (C++)

This example demonstrates how to retrieve and display device information from any MIP-enabled MicroStrain device using
the C++ API.

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
- `initializeDevice()` - Establishes serial communication and validates device connection
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

The C++ version uses modern features including:
- **Type Safety**: Strongly typed data structures for device information
- **RAII**: Automatic resource management for connections
- **Exception Safety**: Proper error handling and resource cleanup
- **String Handling**: Safe C++ string operations

## C++ Implementation Features

This example showcases:
- **Modern C++ Connection Management**: RAII-based resource handling
- **Type-Safe MIP Command Interfaces**: Compile-time type checking
- **Automatic Resource Cleanup**: RAII for connection management
- **Exception Safety**: Proper error handling
- **STL Integration**: Use of standard library string operations

## Device Information Retrieved

The example displays the following device information:
- **Model Name**: Human-readable device model name
- **Model Number**: Specific model identifier
- **Serial Number**: Unique device serial number
- **Lot Number**: Manufacturing lot information
- **Device Options**: Available device features and capabilities
- **Firmware Version**: Current firmware version (formatted as x.x.xx)

## Modern C++ Features

This example showcases:
- **RAII Resource Management**: Automatic connection cleanup
- **Type-Safe Commands**: Compile-time verified command interfaces
- **STL Containers**: Standard library integration for data handling
- **Exception Safety**: Guaranteed cleanup and error handling

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
- Command result checking using `mip::CmdResult`
- Connection failure detection and recovery
- Graceful termination functions for different error types
- Detailed error messages with context

## C++ Features

This example demonstrates:
- Modern C++ connection management
- Type-safe MIP command interfaces
- RAII resource management
- Exception-safe programming
- Standard library integration
- Safe string handling

## Requirements

- Any MIP-enabled MicroStrain device
- Serial connection (USB or RS-232)
- MIP SDK library with C++ support
- C++11 or later compiler

## See Also

- C version: `device_info_example.c`
- Other examples in the `examples/` directory
- MIP SDK documentation
