# 7 Series GNSS/INS Example (C++)

This example demonstrates how to configure and use a MicroStrain 7-series GNSS/INS device with the MIP SDK using the
C++ API.

## Overview

The example showcases the basic setup and operation of a 7-series GNSS/INS device, including:
- Device initialization and communication
- Filter and GNSS message format configuration
- Gyro bias capture
- Antenna offset configuration
- Filter initialization and heading source configuration
- Real-time data streaming and display

## Configurable Options

The example uses the following default settings, which should be adjusted based on application requirements:

| Setting                       | Value                                         | Description                                                    |
|-------------------------------|-----------------------------------------------|----------------------------------------------------------------|
| `PORT_NAME`                   | `"COM1"` (Windows)<br>`"/dev/ttyACM0"` (Unix) | Serial port for device communication                           |
| `BAUDRATE`                    | `115200`                                      | Communication baud rate                                        |
| `SAMPLE_RATE_HZ`              | `1`                                           | Data output rate in Hz                                         |
| `RUN_TIME_SECONDS`            | `30`                                          | Example runtime duration                                       |
| `WHEELED_VEHICLE_APPLICATION` | `false`                                       | Enable/disable for wheeled-vehicle applications (compile-time) |

## GNSS Antenna Configuration

The example configures dual-antenna GNSS setup for enhanced navigation accuracy and heading determination:

| Antenna ID | Purpose                | Translation     | Description                                                           |
|------------|------------------------|-----------------|-----------------------------------------------------------------------|
| `1`        | Primary GNSS Antenna   | [-0.25, 0, 0] m | Main GNSS receiver positioned 0.25m behind the device center          |
| `2`        | Secondary GNSS Antenna | [ 0.25, 0, 0] m | Secondary GNSS receiver positioned 0.25m forward of the device center |

## Usage

1. Connect your 7-series GNSS/INS device to the specified serial port
2. Update the [configuration options](#configurable-options) based on your application needs
3. Update the `WHEELED_VEHICLE_APPLICATION` constant if using a wheeled-vehicle application
4. Compile and run the example
5. Follow the gyro bias capture prompt (keep the device stationary)
6. The program will:
    - Initialize the device
    - Configure data streaming
    - Wait for the filter to reach full navigation mode
    - Stream data for the specified runtime
    - Display filter state and GNSS fix transitions
    - Display real-time filter data
    - Clean up and exit

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
- `initializeDevice()` - Establishes communication, validates device connection, and loads defaults
- `captureGyroBias()` - Captures and applies gyroscope bias compensation
- `configureAntennaOffset()` - Sets GNSS antenna position relative to the device
- `initializeFilter()` - Initializes the navigation filter with GNSS position and velocity, and GNSS heading as the
  heading sources

### Message Configuration
- `configureFilterMessageFormat()` - Configures filter/navigation data output including:
    - GPS timestamp
    - Filter status
    - LLH position coordinates
    - NED velocity vectors
    - Euler angles (roll, pitch, yaw)
- `configureGnssMessageFormat()` - Configures GNSS data output including:
    - Fix info

### Data Display
- `displayFilterState()` - Displays navigation filter operating mode changes
- `displayGnssFixState()` - Shows current GNSS fix status and quality

### Communication Interface
- Uses the `mip::Interface` class for device communication
- Serial connection handled by `microstrain::connections::SerialConnection`

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

## Filter State Progression

The example monitors and displays filter state transitions:
1. **Startup** - Filter startup
2. **Initialization** - Filter initialization
3. **Vertical Gyro** - Basic attitude estimation
4. **AHRS** - Full attitude and heading reference
5. **Full Navigation** - Complete navigation solution with position/velocity

## GNSS Fix State Progression

The example monitors and displays GNSS fix state transitions:
1. **No Fix** - Initial state with no satellite positioning
2. **Invalid** - Fix data is present but flagged as unreliable or corrupted
3. **Time Only** - Time synchronization established but no positioning
4. **2D Fix** - Horizontal positioning available (latitude/longitude)
5. **3D Fix** - Full positioning with altitude information
6. **Differential** - Enhanced accuracy with correction data
7. **RTK Float** - High-precision mode with ambiguity resolution in progress
8. **RTK Fixed** - Centimeter-level accuracy with resolved ambiguities

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

### Configuration Details

The dual-antenna configuration provides:
- **Enhanced Heading Accuracy**: Two spatially separated antennas enable precise heading calculations without relying
  solely on vehicle motion
- **Baseline Vector**: 0.5m separation along the X-axis provides optimal heading determination
- **RTK Support**: Both antennas can receive differential corrections for centimeter-level positioning accuracy
- **Redundancy**: Backup positioning capability if one antenna experiences signal obstruction

### Antenna Offset Constraints

- **Minimum Separation**: 0.25m magnitude per antenna offset
- **Maximum Separation**: 10m magnitude per antenna offset
- **Coordinate System**: Device body frame
    - X: Forward
    - Y: Right
    - Z: Down
- **Precision Requirements**: Accurate physical measurements critical for proper heading calculations

### Multi-Antenna Benefits

This configuration enables:
- **GNSS Heading**: Direct heading measurements independent of vehicle dynamics
- **Improved Initialization**: Faster filter convergence with dual-antenna alignment
- **Signal Diversity**: Better performance in challenging GNSS environments
- **RTK Float/Fixed**: Support for high-precision positioning modes

## Requirements

- MicroStrain 7-series GNSS/INS device (3DM-GQ7-GNSS/INS, or 3DM-CV7-GNSS/INS)
- Serial connection (USB or RS-232)
- MIP SDK library with C++ support
- C++11 or later compiler

## See Also

- C version: `7_series_gnss_ins_example.c`
- Other examples in the `examples/` directory
- [MIP SDK documentation](https://lord-microstrain.github.io/mip_sdk_documentation/)
