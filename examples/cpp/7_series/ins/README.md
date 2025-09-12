# 7 Series INS Example (C++)

This example demonstrates how to configure and use a MicroStrain 7-series INS device with the MIP SDK using the C++ API.

## Overview

The example showcases the basic setup and operation of a 7-series INS device, including:
- Device initialization and communication
- Filter message format configuration
- Gyro bias capture
- External aiding measurements configuration
- Reference frame setup for external sensors
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

## External Aiding Configuration

The example configures three reference frames for external measurements:

| Frame ID | Purpose          | Translation | Rotation       |
|----------|------------------|-------------|----------------|
| `1`      | External Heading | [0, 0, 0] m | [0, 0, 0] deg  |
| `2`      | GNSS Antenna     | [0, 1, 0] m | [0, 0, 0] deg  |
| `3`      | Body Velocity    | [1, 0, 0] m | [0, 0, 90] deg |

## Key Functions

### Device Setup
- `initializeDevice()` - Establishes communication, validates device connection, and loads defaults
- `captureGyroBias()` - Captures and applies gyroscope bias compensation
- `configureExternalAidingHeading()` - Sets up a reference frame for external heading sensor data
- `configureExternalAidingGnssAntenna()` - Sets up a reference frame for external GNSS antenna sensor data
- `configureExternalAidingNedVelocity()` - Sets up a reference frame for external NED velocity sensor data
- `initializeFilter()` - Initializes the navigation filter with GNSS position and velocity, and external heading as the
  heading sources

### Message Configuration
- `configureFilterMessageFormat()` - Configures filter/navigation data output including:
    - GPS timestamp
    - Filter status
    - LLH position coordinates
    - NED velocity vectors
    - Euler angles (roll, pitch, yaw)

### Data Display
- `displayFilterState()` - Displays navigation filter operating mode changes

### Communication Interface
- Uses the `mip::Interface` class for device communication
- Serial connection handled by `microstrain::connections::SerialConnection`

### External Data Simulation
- `sendSimulatedExternalMeasurementsHeading()` - Simulates external heading measurements
- `sendSimulatedExternalMeasurementsPosition()` - Provides external position updates
- `sendSimulatedExternalMeasurementsNedVelocity()` - Sends NED velocity measurements
- `sendSimulatedExternalMeasurementsVehicleFrameVelocity()` - Provides body-frame velocity data

## Data Handling

This example uses modern C++ features including:
- **Data Extractors**: Automatic parsing of incoming data fields
- **Type Safety**: Strongly typed data structures for each message type
- **Callbacks**: Automatic data callbacks for registered message types
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

## External Measurement Simulation

The example provides simulated external data for demonstration:
- **Heading**: 0.0° (North) with 0.001 rad uncertainty
- **Position**: MicroStrain headquarters coordinates (44.437°N, 73.106°W, 122m)
- **Velocity**: Stationary [0, 0, 0] m/s with 0.1 m/s uncertainty (both NED and body frame)

## Filter Data Output

The example streams the following filter data:

### TOW Data
- **Units**: seconds
- **Description**: Time of Week - GPS time reference
- **Format**: Floating-point timestamp value

### Position LLH Data
- **Units**:
    - Latitude: degrees
    - Longitude: degrees
    - Ellipsoid Height: meters
- **Description**: Position in Latitude, Longitude, Height coordinate system
- **Format**: [Latitude, Longitude, Height] vector

### Velocity NED Data
- **Units**: m/s (meters per second)
- **Description**: Velocity in North, East, Down coordinate frame
- **Format**: [North, East, Down] velocity vector

### Euler Angles Data
- **Units**: radians
- **Description**: Orientation expressed as Euler angles
- **Format**: [Roll, Pitch, Yaw] angle vector

## Streaming Output Format

The example displays filter data in the following format:
```
TOW = 123456.789    Position LLH = [ 4.123456, -83.54321,  123.4567]    Velocity NED = [ 1.234567, -0.987654,  0.123456]     Euler Angles = [ 0.012345, -0.067890,  1.234567]
```

## Usage

1. Connect your 7-series INS device to the specified serial port
2. Update the `PORT_NAME` constant if using a different port
3. Compile and run the example
4. Follow the gyro bias capture prompt (keep the device stationary)
5. The program will:
    - Initialize the device
    - Configure data streaming
    - Wait for the filter to reach full navigation mode
    - Stream data for the specified runtime
    - Display filter state transitions
    - Display real-time filter data
    - Clean up and exit

## Filter State Progression

The example monitors and displays filter state transitions:
1. **Startup** - Filter startup
2. **Initialization** - Filter initialization
3. **Vertical Gyro** - Basic attitude estimation
4. **AHRS** - Full attitude and heading reference
5. **Full Navigation** - Complete navigation solution with position/velocity

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
- Automatic data field extraction
- RAII resource management
- Standard library integration

## Type Safety and Documentation

This example provides additional C++ benefits:
- **Built-in Documentation**: Data structures include `DOC_NAME` constants for easy reference
- **Strongly Typed Enums**: C++ enum classes prevent accidental misuse
- **Automatic Descriptors**: `DESCRIPTOR` constants eliminate magic numbers

## Requirements

- MicroStrain 7-series INS device (3DM-CV7-INS, or 3DM-GV7-INS)
- Serial connection (USB or RS-232)
- MIP SDK library with C++ support
- C++11 or later compiler
- External sensors for real-world implementation (optional for demo)

## See Also

- C version: `7_series_ins_example.c`
- Other examples in the `examples/` directory
- [MIP SDK documentation](https://lord-microstrain.github.io/mip_sdk_documentation/)
