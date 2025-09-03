# 5 Series AHRS Example (C++)

This example demonstrates how to configure and use a MicroStrain 5-series AHRS device with the MIP SDK using the C++
API.

## Overview

The example showcases the basic setup and operation of a 5-series AHRS device, including:
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
- `initializeDevice()` - Establishes serial communication and validates device connection
- `captureGyroBias()` - Captures and applies gyroscope bias compensation
- `initializeFilter()` - Initializes the navigation filter with magnetometer as the heading source

### Message Configuration
- `configureFilterMessageFormat()` - Configures filter/navigation data output including:
    - Filter timestamps
    - Filter status
    - LLH position coordinates
    - NED velocity vectors
    - Euler angles (roll, pitch, yaw)

### Data Display
- `displayFilterState()` - Displays navigation filter operating mode changes

### Communication Interface
- Uses the `mip::Interface` class for device communication
- Serial connection handled by `microstrain::connections::SerialConnection`

## Data Handling

The C++ version uses modern features including:
- **Data Extractors**: Automatic parsing of incoming data fields
- **Type Safety**: Strongly typed data structures for each message type
- **Callbacks**: Automatic data callbacks for registered message types
- **RAII**: Automatic resource management for connections

## C++ Implementation Features

This example demonstrates:
- **Modern C++ Connection Management**: RAII-based resource handling
- **Type-Safe MIP Command Interfaces**: Compile-time type checking
- **Exception Safety**: Proper error handling and resource cleanup
- **STL Integration**: Use of standard library containers and algorithms

## Data Registration

The C++ version showcases automatic data handling:
- **Data Stores**: Automatic storage of incoming data fields
- **Extractor Registration**: Type-safe registration of data extractors
- **Callback Management**: Automatic callback invocation for new data

## Usage

1. Connect your 5-series AHRS device to the specified serial port
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
- Command result checking using `mip::CmdResult`
- Connection failure detection and recovery
- Graceful termination functions for different error types
- Detailed error messages with context

## C++ Features

This example demonstrates:
- Modern C++ connection management
- Type-safe MIP command interfaces
- Automatic data field extraction
- RAII resource management
- Standard library integration

## Requirements

- MicroStrain 5-series AHRS device (3DM-CX5-AHRS, 3DM-CV5-AHRS, or 3DM-GX5-AHRS)
- Serial connection (USB or RS-232)
- MIP SDK library with C++ support
- C++11 or later compiler

## See Also

- C version: `5_series_ahrs_example.c`
- Other examples in the `examples/` directory
- MIP SDK documentation
