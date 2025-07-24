# 5 Series GNSS/INS Example (C++)

This example demonstrates how to configure and use a MicroStrain 5-series GNSS/INS device with the MIP SDK using the C++ API.

## Overview

The example showcases basic setup and operation of a 5-series device, including:
- Device initialization and communication
- GNSS and filter message configuration
- Gyro bias capture
- Antenna offset configuration
- Filter initialization
- Real-time data streaming and display

## Configuration

The example uses the following default settings:

| Setting | Value                                         | Description |
|---------|-----------------------------------------------|-------------|
| `PORT_NAME` | `"COM1"` (Windows)<br>`"/dev/ttyACM0"` (Unix) | Serial port for device communication |
| `BAUDRATE` | `115200`                                      | Communication baud rate |
| `SAMPLE_RATE_HZ` | `1`                                           | Data output rate in Hz |
| `RUN_TIME_SECONDS` | `30`                                          | Example runtime duration |

## Key Functions

### Device Setup
- `initializeDevice()` - Establishes serial communication and validates device connection
- `captureGyroBias()` - Captures and applies gyroscope bias compensation
- `configureAntennaOffset()` - Sets GNSS antenna position relative to device
- `initializeFilter()` - Initializes the navigation filter

### Message Configuration
- `configureGnssMessageFormat()` - Sets up GNSS data output messages
- `configureFilterMessageFormat()` - Configures filter/navigation data output

### Data Display
- `displayGnssFixState()` - Shows current GNSS fix status and quality
- `displayFilterState()` - Displays navigation filter operating mode

### Communication Interface
- Uses the `mip::Interface` class for device communication
- Serial connection handled by `microstrain::connections::SerialConnection`

## Data Handling

The C++ version uses modern features including:
- **Data Extractors**: Automatic parsing of incoming data fields
- **Type Safety**: Strongly typed data structures for each message type
- **Callbacks**: Automatic data callbacks for registered message types
- **RAII**: Automatic resource management for connections

## Usage

1. Connect your 5-series device to the specified serial port
2. Update the `PORT_NAME` constant if using a different port
3. Compile and run the example
4. The program will:
   - Initialize the device
   - Configure data output
   - Stream data for the specified runtime
   - Display GNSS fix status and filter state
   - Clean up and exit

## Error Handling

The example includes comprehensive error handling with:
- Command result checking using `mip::CmdResult`
- Graceful termination functions for different error types
- Detailed error messages with context

## C++ Features

This example demonstrates:
- Modern C++ connection management
- Type-safe MIP command interfaces
- Automatic data field extraction
- RAII resource management
- Template-based type safety

## Requirements

- MicroStrain 5-series GNSS/INS device
- Serial connection (USB or RS-232)
- MIP SDK library with C++ support
- C++11 or later compiler

## See Also

- C version: `5_series_gnss_ins_example.c`
- Other examples in the `examples/` directory
- MIP SDK documentation
