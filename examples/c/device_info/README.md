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
- `mip_interface_user_send_to_device()` - Sends commands to the device
- `mip_interface_user_recv_from_device()` - Receives data from the device

## Data Handling

The C version demonstrates traditional C programming patterns:
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

## Requirements

- Any MIP-enabled MicroStrain device
- Serial connection (USB or RS-232)
- MIP SDK library with C support
- C11 or later compiler

## See Also

- C++ version: `device_info_example.cpp`
- Other examples in the `examples/` directory
- MIP SDK documentation
