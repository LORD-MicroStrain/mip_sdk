# 7 Series IMU Streaming Example (C++)

This example demonstrates how to stream basic IMU sensor data from a MicroStrain 7-series device with the MIP SDK 
using the C++ API.

## Overview

The example showcases basic IMU sensor data streaming setup and operation of a 7-series device, including:
- Device initialization and communication
- Sensor message format configuration
- Real-time data streaming and display
- Dynamic magnetometer support detection
- Basic packet and field-level callbacks

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
- `initializeDevice()` - Establishes communication, validates device connection, and loads defaults
- `isDescriptorSupported()` - Checks device capabilities for sensor data types

### Message Configuration
- `configureSensorMessageFormat()` - Configures IMU sensor data output including:
    - Scaled accelerometer
    - Scaled gyroscope
    - Scaled magnetometer (if supported)

### Data Processing
- `packetCallback()` - Processes complete MIP packets and displays field information
- `accelFieldCallback()` - Handles accelerometer data fields
- `gyroFieldCallback()` - Handles gyroscope data fields
- `magFieldCallback()` - Handles magnetometer data fields (if supported)

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

## IMU Sensor Data Output

The example streams the following IMU sensor data:

### Accelerometer Data
- **Units**: g (gravitational acceleration)
- **Range**: Device-dependent (typically ?2g, ?4g, ?8g, ?16g)
- **Format**: [X, Y, Z] scaled acceleration vector

### Gyroscope Data
- **Units**: rad/s (radians per second)
- **Range**: Device-dependent (typically ?250?/s to ?2000?/s)
- **Format**: [X, Y, Z] scaled angular rate vector

### Magnetometer Data (if supported)
- **Units**: Gauss
- **Range**: Device-dependent
- **Format**: [X, Y, Z] scaled magnetic field vector
- **Note**: Automatically detected and configured if available

## Streaming Output Format

The example displays sensor data in the following format:
```
Scaled Accel Data (0x80, 0x04): [ 0.012345, -0.098765,  0.987654]
Scaled Gyro Data  (0x80, 0x05): [-0.001234,  0.005678, -0.002345]
Scaled Mag Data   (0x80, 0x06): [ 0.123456, -0.234567,  0.345678]
```

## Usage

1. Connect your 7-series device to the specified serial port
2. Update the `PORT_NAME` constant if using a different port
3. Compile and run the example
4. The program will:
    - Initialize the device and query supported descriptors
    - Configure data streaming
    - Register callbacks for data processing
    - Stream data for the specified runtime
    - Display real-time sensor data
    - Clean up and exit

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
- `<path_to_mip_sdk_include>/cpp`
- `<path_to_project_root>`

<path_to_mip_sdk_include> can be installed paths or source paths:
- Unix - `/usr/include/microstrain`
- Windows - `C:/Program Files/MIP_SDK/include/microstrain`
- Source: `<mip_sdk_project_root>/src`

#### Compiler Definitions
Add these compiler definitions:
- `MICROSTRAIN_ENABLE_LOGGING=1` Enables the logging feature
- `MICROSTRAIN_LOGGING_MAX_LEVEL=MICROSTRAIN_LOG_LEVEL_INFO` Sets the logging level to info which is the minimum required for this example

## Requirements

- MicroStrain 7-series device
- Serial connection (USB or RS-232)
- MIP SDK library with C++ support
- C++11 or later compiler

## See Also

- C version: `7_series_stream_imu_example.c`
- Other examples in the `examples/` directory
- [MIP SDK documentation](https://lord-microstrain.github.io/mip_sdk_documentation/)
