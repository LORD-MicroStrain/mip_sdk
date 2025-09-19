# 5 Series GNSS/INS Example (C)

This example demonstrates how to configure and use a MicroStrain 5-series GNSS/INS device with the MIP SDK using the
C API.

## Overview

The example showcases the basic setup and operation of a 5-series GNSS/INS device, including:
- Device initialization and communication
- GNSS and filter message format configuration
- Gyro bias capture
- Antenna offset configuration
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
- `initialize_device()` - Establishes communication, validates device connection, and loads defaults
- `capture_gyro_bias()` - Captures and applies gyroscope bias compensation
- `configure_antenna_offset()` - Sets GNSS antenna position relative to the device
- `initialize_filter()` - Initializes the navigation filter with GNSS velocity and magnetometer as the heading source

### Message Configuration
- `configure_filter_message_format()` - Configures filter/navigation data output including:
    - Filter timestamp
    - Filter status
    - LLH position coordinates
    - NED velocity vectors
    - Euler angles (roll, pitch, yaw)

### Data Display
- `display_filter_state()` - Displays navigation filter operating mode changes
- `display_gnss_fix_state()` - Shows current GNSS fix status and quality

### Communication Interface
- Uses the `mip_interface` struct for device communication
- Serial connection handled by `serial_port`

## Data Handling

This example demonstrates traditional C programming patterns:
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

1. Connect your 5-series GNSS/INS device to the specified serial port
2. Update the `PORT_NAME` constant if using a different port
3. Compile and run the example
4. Follow the gyro bias capture prompt (keep the device stationary)
5. The program will:
    - Initialize the device
    - Configure data streaming
    - Wait for the filter to initialize
    - Stream data for the specified runtime
    - Display filter state and GNSS fix transitions
    - Display real-time filter data
    - Clean up and exit

## Filter State Progression

The example monitors and displays filter state transitions:
1. **Startup** - Filter startup
2. **Initialization** - Filter initialization
3. **Run Solution Valid** - Valid filter solution
4. **Run Solution Invalid** - Invalid filter solution

## GNSS Fix State Progression

The example monitors and displays GNSS fix state transitions:
1. **No Fix** - Initial state with no satellite positioning
2. **Invalid** - Fix data is present but flagged as unreliable or corrupted
3. **Time Only** - Time synchronization established but no positioning
4. **2D Fix** - Horizontal positioning available (latitude/longitude)
5. **3D Fix** - Full positioning with altitude information

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
- `MICROSTRAIN_LOGGING_MAX_LEVEL=MICROSTRAIN_LOGGING_LEVEL_INFO_` Sets the logging level to info which is the minimum required for this example

## Requirements

- MicroStrain 5-series GNSS/INS device (3DM-CX5-GNSS/INS, or 3DM-GX5-GNSS/INS)
- Serial connection (USB or RS-232)
- MIP SDK library with C support
- C11 or later compiler

## See Also

- C++ version: `5_series_gnss_ins_example.cpp`
- Other examples in the `examples/` directory
- [MIP SDK documentation](https://lord-microstrain.github.io/mip_sdk_documentation/)
