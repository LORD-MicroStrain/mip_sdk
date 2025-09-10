# 7 Series AHRS Example (C)

This example demonstrates how to configure and use a MicroStrain 7-series AHRS device with the MIP SDK using the C API.

## Overview

The example showcases the basic setup and operation of a 7-series AHRS device, including:
- Device initialization and communication
- Filter message configuration
- Gyro bias capture
- Filter initialization and heading source configuration
- Real-time data streaming and display

## Configuration

The example uses the following default settings:

| Setting            | Value                                         | Description                          |
|--------------------|-----------------------------------------------|--------------------------------------|
| `PORT_NAME`        | `"COM1"` (Windows)<br>`"/dev/ttyACM0"` (Unix) | Serial port for device communication |
| `BAUDRATE`         | `115200`                                      | Communication baud rate              |
| `SAMPLE_RATE_HZ`   | `1`                                           | Data output rate in Hz               |
| `RUN_TIME_SECONDS` | `30`                                          | Example runtime duration             |

## Key Functions

### Device Setup
- `initialize_device()` - Establishes serial communication and validates device connection
- `capture_gyro_bias()` - Captures and applies gyroscope bias compensation
- `initialize_filter()` - Initializes the navigation filter with magnetometer as the heading source

### Message Configuration
- `configure_filter_message_format()` - Configures filter/navigation data output including:
    - GPS timestamp
    - Filter status
    - Euler angles (roll, pitch, yaw)

### Data Display
- `display_filter_state()` - Displays navigation filter operating mode changes

### Communication Interface
- `mip_interface_user_send_to_device()` - Sends commands to the device
- `mip_interface_user_recv_from_device()` - Receives data from the device

## Data Handling

The C version demonstrates traditional C programming patterns:
- **Manual Parsing**: Direct parsing of incoming MIP packets using the MIP parser
- **Callback Functions**: Function pointer-based callbacks for data processing
- **Explicit Memory Management**: Manual buffer and resource management
- **Type Safety**: Uses C structs and enums for data type safety
- **Error Codes**: Command results handled through return codes and error checking

## C Implementation Features

This example showcases:
- **MIP Interface**: Core C interface for device communication (`mip_interface`)
- **Serial Port Management**: Low-level serial port operations
- **Parser Integration**: Direct use of the MIP packet parser
- **Memory Safety**: Careful buffer management and bound checking
- **Portability**: Cross-platform compatibility (Windows/Unix)

## Custom Communication Functions

The example implements custom communication handlers:
- **Send Function**: `mip_interface_user_send_to_device()` handles outgoing data
- **Receive Function**: `mip_interface_user_recv_from_device()` manages incoming data
- **Timeout Handling**: Configurable timeouts for reliable communication

## Usage

1. Connect your 7-series AHRS device to the specified serial port
2. Update the `PORT_NAME` constant if using a different port
3. Compile and run the example
4. The program will:
    - Initialize the device
    - Configure data output
    - Stream data for the specified runtime
    - Display filter status changes
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

- MicroStrain 7-series AHRS device (3DM-CV7-AHRS or 3DM-GV7-AHRS)
- Serial connection (USB or RS-232)
- MIP SDK library with C support
- C11 or later compiler

## See Also

- C++ version: `7_series_ahrs_example.cpp`
- Other examples in the `examples/` directory
- MIP SDK documentation
