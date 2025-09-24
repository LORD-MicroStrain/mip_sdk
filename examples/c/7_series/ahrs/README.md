# 7 Series AHRS Example (C)

This example demonstrates how to configure and use a MicroStrain 7-series AHRS device with the MIP SDK using the C API.

## Overview

The example showcases the basic setup and operation of a 7-series AHRS device, including:
- Device initialization and communication
- Filter message format configuration
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

## Usage

1. Connect your 7-series AHRS device to the specified serial port
2. Update the `PORT_NAME` constant if using a different port
3. Compile and run the example
4. Follow the gyro bias capture prompt (keep the device stationary)
5. The program will:
    - Initialize the device
    - Configure data streaming
    - Wait for the filter to reach AHRS mode
    - Stream data for the specified runtime
    - Display filter state transitions
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
- `initialize_device()` - Establishes communication, validates device connection, and loads defaults
- `capture_gyro_bias()` - Captures and applies gyroscope bias compensation
- `initialize_filter()` - Initializes the attitude filter with magnetometer as the heading source

### Message Configuration
- `configure_filter_message_format()` - Configures filter/attitude data output including:
    - GPS timestamp
    - Filter status
    - Euler angles (roll, pitch, yaw)

### Data Display
- `display_filter_state()` - Displays navigation filter operating mode changes

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

### Euler Angles Data
- **Units**: radians
- **Description**: Orientation expressed as Euler angles
- **Format**: [Roll, Pitch, Yaw] angle vector

## Streaming Output Format

The example displays filter data in the following format:
```
TOW = 123456.789     Euler Angles = [ 0.012345, -0.067890,  1.234567]
```

## Filter State Progression

The example monitors and displays filter state transitions:
1. **Startup** - Filter startup
2. **Initialization** - Filter initialization
3. **Vertical Gyro** - Basic attitude estimation
4. **AHRS** - Full attitude and heading reference
5. **Full Navigation** - Complete navigation solution with position/velocity

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
- [MIP SDK documentation](https://lord-microstrain.github.io/mip_sdk_documentation/)
