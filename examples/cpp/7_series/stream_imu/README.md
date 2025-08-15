# 7 Series IMU Streaming Example (C++)

This example demonstrates how to stream basic IMU sensor data from a MicroStrain 7-series device using the MIP SDK C++ API.

## Overview

The example showcases fundamental IMU data streaming setup and operation, including:
- Device initialization and communication using modern C++ interfaces
- Sensor message format configuration
- Real-time IMU data streaming and display
- Communication recording for analysis
- Dynamic magnetometer support detection
- Comprehensive packet and field-level callbacks

## Configuration

The example uses the following default settings:

| Setting            | Value                                         | Description                          |
|--------------------|-----------------------------------------------|--------------------------------------|
| `PORT_NAME`        | `"COM1"` (Windows)<br>`"/dev/ttyACM0"` (Unix) | Serial port for device communication |
| `BAUDRATE`         | `115200`                                      | Communication baud rate              |
| `SAMPLE_RATE_HZ`   | `1`                                           | Data output rate in Hz               |
| `RUN_TIME_SECONDS` | `30`                                          | Example runtime duration             |

## Recording Configuration

The example includes communication recording for debugging and analysis:

| Setting                | Value                 | Description                      |
|------------------------|-----------------------|----------------------------------|
| `RECEIVE_BYTES_BINARY` | `"receive_bytes.bin"` | File to record incoming data     |
| `SEND_BYTES_BINARY`    | `"send_bytes.bin"`    | File to record outgoing commands |

## Key Functions

### Device Setup
- `initializeDevice()` - Establishes communication, validates connection, and loads defaults
- `configureSensorMessageFormat()` - Sets up IMU sensor data output messages
- `isDescriptorSupported()` - Checks device capabilities for sensor data types

### Message Configuration
- Configures sensor message format with:
  - Scaled accelerometer data
  - Scaled gyroscope data  
  - Scaled magnetometer data (if supported)

### Data Processing
- `packetCallback()` - Processes complete MIP packets and displays field information
- `accelFieldCallback()` - Handles accelerometer data fields
- `gyroFieldCallback()` - Handles gyroscope data fields  
- `magFieldCallback()` - Handles magnetometer data fields (optional)

## Data Handling

The C++ version demonstrates modern C++ programming patterns:
- **Object-Oriented Design**: Uses C++ classes and objects for clean abstraction
- **Type Safety**: Strong typing with C++ classes and enums
- **RAII**: Automatic resource management through constructors/destructors
- **STL Integration**: Uses standard library containers and algorithms
- **Exception Safety**: Proper error handling with C++ exceptions where appropriate
- **Template-Based Parsing**: Type-safe data extraction using C++ templates

## C++ Implementation Features

This example showcases:
- **MIP Interface**: Modern C++ interface for device communication (`mip::Interface`)
- **Serial Connection**: C++ connection class with automatic resource management
- **Type-Safe Data Structures**: C++ data classes for sensor measurements
- **Template-Based Callbacks**: Compile-time type checking for callback registration
- **Standard Library**: Modern C++ features and STL usage
- **Recording Support**: Built-in communication recording with C++ streams

## Sensor Data Output

The example streams the following sensor data:

### Accelerometer Data
- **Units**: g (gravitational acceleration)
- **Range**: Device-dependent (typically ±2g, ±4g, ±8g, ±16g)
- **Format**: [X, Y, Z] scaled acceleration vector

### Gyroscope Data  
- **Units**: rad/s (radians per second)
- **Range**: Device-dependent (typically ±250°/s to ±2000°/s)
- **Format**: [X, Y, Z] scaled angular rate vector

### Magnetometer Data (if supported)
- **Units**: Gauss
- **Range**: Device-dependent
- **Format**: [X, Y, Z] scaled magnetic field vector
- **Note**: Automatically detected and configured if available

## Connection Management

The C++ version uses modern connection handling:
- **SerialConnection**: RAII-based serial connection management
- **Automatic Cleanup**: Connection automatically closed when object goes out of scope
- **Exception Safety**: Proper resource cleanup even when errors occur
- **Recording Integration**: Built-in recording support with file management

## Usage

1. Connect your 7-series device to the specified serial port
2. Update the `PORT_NAME` constant if using a different port
3. Compile and run the example
4. The program will:
    - Initialize the device and query supported descriptors
    - Configure sensor data streaming
    - Register callbacks for data processing
    - Stream data for the specified runtime
    - Display real-time sensor measurements
    - Record communication data to binary files
    - Clean up and exit

## Output Format

The example displays sensor data in the following format:
```
Scaled Accel Data (0x80, 0x04): [ 0.012345, -0.098765,  0.987654]
Scaled Gyro Data  (0x80, 0x05): [-0.001234,  0.005678, -0.002345]
Scaled Mag Data   (0x80, 0x06): [ 0.123456, -0.234567,  0.345678]
```

## Error Handling

The example includes comprehensive error handling with:
- Command result checking using `mip::CmdResult`
- Graceful termination functions for different error types
- Detailed error messages with context using built-in documentation strings
- RAII-based resource cleanup

## C++ Features

This example demonstrates:
- Modern C++11 programming practices
- Object-oriented design principles
- RAII resource management
- Type-safe interfaces
- Template-based data extraction
- Standard library integration
- Real-time sensor data processing

## Type Safety and Documentation

The C++ version provides additional benefits:
- **Compile-Time Checking**: Template-based callbacks catch type errors at compile time
- **Built-in Documentation**: Data structures include `DOC_NAME` constants for easy reference
- **Strongly Typed Enums**: C++ enum classes prevent accidental misuse
- **Automatic Descriptors**: `DESCRIPTOR` constants eliminate magic numbers
- **Range-Based Iteration**: Modern C++ iteration over packet fields

## Requirements

- MicroStrain 7-series device
- Serial connection (USB or RS-232)
- MIP SDK library with C++ support
- C++11 or later compiler

## See Also

- C version: `7_series_stream_imu_example.c`
- Other examples in the `examples/` directory
- MIP SDK documentation for sensor data commands
