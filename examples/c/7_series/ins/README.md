# 7 Series INS Example (C)

This example demonstrates how to configure and use a MicroStrain 7-series INS device with external aiding 
measurements using the MIP SDK C API.

## Overview

The example showcases basic INS setup and operation for 7-series devices, including:
- Device initialization and communication
- Filter message configuration with INS-specific data
- Gyro bias capture for improved accuracy
- External aiding measurements configuration
- Reference frame setup for external sensors
- Filter initialization with external aiding
- Real-time data streaming with external measurement injection

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
- `initialize_device()` - Establishes communication, validates connection, and loads defaults
- `capture_gyro_bias()` - Captures gyroscope bias for improved INS performance
- `configure_external_aiding()` - Sets up reference frames for external sensor data
- `initialize_filter()` - Configures and initializes the navigation filter with external aiding

### Message Configuration
- `configure_filter_message_format()` - Sets up INS filter data output messages including:
  - GPS timestamp
  - Filter status
  - LLH position
  - NED velocity
  - Euler angles

### External Data Simulation
- `send_simulated_external_measurements_heading()` - Simulates external heading measurements
- `send_simulated_external_measurements_position()` - Provides external position updates
- `send_simulated_external_measurements_ned_velocity()` - Sends NED velocity measurements
- `send_simulated_external_measurements_vehicle_frame_velocity()` - Provides body-frame velocity data

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
- **MIP Interface**: Core C interface for INS device communication (`mip_interface`)
- **Serial Port Management**: Low-level serial port operations
- **Parser Integration**: Direct use of the MIP packet parser
- **Memory Safety**: Careful buffer management and bound checking
- **External Aiding**: Integration of external sensor measurements into the INS filter
- **Portability**: Cross-platform compatibility (Windows/Unix)

## 7-Series INS Specific Features

This example demonstrates 7-series INS-specific capabilities:
- **External Aiding Support**: Integration of external position, velocity, and heading measurements
- **Advanced Filter Configuration**: Enhanced navigation filter with external measurement sources
- **Reference Frame Management**: Configurable coordinate transformations for external sensors
- **Kinematic Alignment**: Filter initialization with external aiding support

## External Measurement Simulation

The example provides simulated external data for demonstration:
- **Heading**: 0.0° (North) with 0.001 rad uncertainty
- **Position**: MicroStrain headquarters coordinates (44.437°N, 73.106°W, 122m)
- **Velocity**: Stationary [0, 0, 0] m/s with 0.1 m/s uncertainty (both NED and body frame)

## Custom Communication Functions

The example implements custom communication handlers:
- **Send Function**: `mip_interface_user_send_to_device()` handles outgoing data
- **Receive Function**: `mip_interface_user_recv_from_device()` manages incoming data
- **Timeout Handling**: Configurable timeouts for reliable communication

## Usage

1. Connect your 7-series INS device to the specified serial port
2. Update the `PORT_NAME` constant if using a different port
3. Compile and run the example
4. Follow the gyro bias capture prompt (keep the device stationary)
5. The program will:
    - Initialize the device and configure external aiding
    - Configure INS data output
    - Wait for the filter to reach full navigation mode
    - Stream data while simulating external measurements
    - Display filter state transitions and navigation data
    - Clean up and exit

## Filter State Progression

The example monitors and displays filter state transitions:
1. **Initialization** - Filter startup
2. **Vertical Gyro** - Basic attitude estimation
3. **AHRS** - Full attitude and heading reference
4. **Full Navigation** - Complete INS solution with position/velocity

## Error Handling

The example includes comprehensive error handling with:
- Command result checking using `mip_cmd_result`
- Graceful termination functions for different error types
- Detailed error messages with context
- External measurement failure warnings (non-fatal)

## C Features

This example demonstrates:
- Standard C11 programming practices
- Minimal external dependencies
- Direct hardware interface programming
- Efficient memory usage patterns
- Cross-platform serial communication
- Real-time external data integration

## Requirements

- MicroStrain 7-series INS device
- Serial connection (USB or RS-232)
- MIP SDK library with C support
- C11 or later compiler
- External sensors for real-world implementation (optional for demo)

## See Also

- C++ version: `7_series_ins_example.cpp`
- Other examples in the `examples/` directory
- MIP SDK documentation
