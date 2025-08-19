# 5 Series IMU Streaming Example (C)

This example demonstrates how to stream basic IMU sensor data from a MicroStrain 5-series device using the MIP SDK C 
API.

## Overview

The example showcases fundamental IMU data streaming setup and operation, including:
- Device initialization and communication
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

| Setting                 | Value                  | Description                      |
|-------------------------|------------------------|----------------------------------|
| `RECEIVED_BYTES_BINARY` | `"received_bytes.bin"` | File to record incoming data     |
| `SENT_BYTES_BINARY`     | `"sent_bytes.bin"`     | File to record outgoing commands |

## Key Functions

### Device Setup
- `initialize_device()` - Establishes communication, validates connection, and loads defaults
- `configure_sensor_message_format()` - Sets up IMU sensor data output messages
- `is_descriptor_supported()` - Checks device capabilities for sensor data types

### Message Configuration
- Configures sensor message format with:
  - Scaled accelerometer data
  - Scaled gyroscope data
  - Scaled magnetometer data (if supported)

### Data Processing
- `packet_callback()` - Processes complete MIP packets and displays field information
- `accel_field_callback()` - Handles accelerometer data fields
- `gyro_field_callback()` - Handles gyroscope data fields
- `mag_field_callback()` - Handles magnetometer data fields (optional)

### Communication Interface
- `mip_interface_user_send_to_device()` - Sends commands to the device
- `mip_interface_user_recv_from_device()` - Receives data from the device

## Data Handling

The C version demonstrates traditional C programming patterns:
- **Manual Parsing**: Direct parsing of incoming MIP packets using the MIP parser
- **Callback Functions**: Function pointer-based callbacks for sensor data processing
- **Explicit Memory Management**: Manual buffer and resource management
- **Type Safety**: Uses C structs and enums for sensor data type safety
- **Error Codes**: Command results handled through return codes and error checking

## C Implementation Features

This example showcases:
- **MIP Interface**: Core C interface for device communication (`mip_interface`)
- **Serial Port Management**: Low-level serial port operations with recording
- **Parser Integration**: Direct use of the MIP packet parser for sensor data
- **Memory Safety**: Careful buffer management and bounds checking
- **Recording Support**: Built-in communication recording for debugging
- **Portability**: Cross-platform compatibility (Windows/Unix)

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

## Communication Recording

The example includes comprehensive recording features:
- **Binary Recording**: Raw communication data saved to binary files
- **Bidirectional**: Both send and receive data streams recorded
- **Analysis Support**: Recorded data can be analyzed for debugging
- **Optional Feature**: Can be disabled by removing recording initialization

## Usage

1. Connect your 5-series device to the specified serial port
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
- Command result checking using `mip_cmd_result`
- Graceful termination functions for different error types
- Detailed error messages with context
- Connection cleanup on exit

## C Features

This example demonstrates:
- Standard C11 programming practices
- Minimal external dependencies
- Direct hardware interface programming
- Efficient memory usage patterns
- Cross-platform serial communication
- Real-time sensor data processing

## Requirements

- MicroStrain 5-series device
- Serial connection (USB or RS-232)
- MIP SDK library with C support
- C11 or later compiler

## See Also

- C++ version: `5_series_stream_imu_example.cpp`
- Comprehensive recording example: `recording_example.c`
- Other examples in the `examples/` directory
- MIP SDK documentation for sensor data commands
