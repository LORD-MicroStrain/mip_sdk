# Recording Example (C)

This example demonstrates how to stream and record factory streaming data from a MicroStrain device using the MIP SDK
C API.

## Overview

The example showcases fundamental data streaming and recording setup and operation, including:
- Device initialization and communication with factory streaming
- Automatic device capability detection and data stream configuration
- Real-time recording of all supported sensor data streams to binary files
- Bidirectional communication recording for debugging and analysis
- Comprehensive device information gathering and logging
- Cross-platform recording support with flexible stream management

## Configuration

The example uses the following default settings:

| Setting     | Value                                         | Description                          |
|-------------|-----------------------------------------------|--------------------------------------|
| `PORT_NAME` | `"COM1"` (Windows)<br>`"/dev/ttyACM0"` (Unix) | Serial port for device communication |
| `BAUDRATE`  | `115200`                                      | Communication baud rate              |

## Recording Configuration

The example includes comprehensive recording setup for debugging and analysis:

| Setting                         | Value                  | Description                                       |
|---------------------------------|------------------------|---------------------------------------------------|
| `RECEIVED_BYTES_BINARY`         | `"received_bytes.bin"` | File to record incoming data                      |
| `SENT_BYTES_BINARY`             | `"sent_bytes.bin"`     | File to record outgoing commands                  |
| `CONNECTION_RECORDING_WRAPPERS` | `1`                    | Use connection wrapper functions vs raw recording |
| `USER_RECORDING_STREAMS`        | `0`                    | Use user-managed streams vs connection-managed    |

## Key Functions

### Device Setup and Configuration
- `initialize_device()` - Establishes communication, validates connection, loads defaults, and configures factory
  streaming
- `get_supported_data_descriptors()` - Extracts supported data descriptor sets from device capabilities
- Recording interface initialization with `recording_connection_init()`

### Factory Streaming Configuration
- Automatically queries device descriptors using `mip_base_get_device_descriptors()`
- Extends descriptor queries with `mip_base_get_extended_descriptors()` for comprehensive coverage
- Resets and enables factory streaming for all supported data descriptors
- Uses `mip_3dm_factory_streaming()` and `mip_3dm_write_datastream_control()`

### Communication Recording
- `recording_connection` interface for bidirectional data recording
- Flexible recording options: wrapper functions or raw recording interface
- Support for both connection-managed and user-managed stream recording
- Optional recording streams that can be enabled/disabled via preprocessor flags
- **Individual stream initialization**: Both receive and send streams/files are not required for recording and can
  be initialized and used independently of each other using the respective `receive` and `send` variants of
  functions within the connection and recording interfaces

### Communication Interface
- `mip_interface_user_send_to_device()` - Sends commands to the device with recording
- `mip_interface_user_recv_from_device()` - Receives data from the device with recording

## Recording Implementation Features

This example showcases two recording approaches:

### Wrapper Recording (`CONNECTION_RECORDING_WRAPPERS = 1`)
- **Connection-Level Functions**: Uses `serial_port_open_recording_files()` and related functions
- **Automatic Management**: Recording files opened and closed automatically
- **Simplified API**: Single function calls for recording setup
- **Integration**: Recording seamlessly integrated with the connection interface

### Raw Recording (`CONNECTION_RECORDING_WRAPPERS = 0`)
- **Direct Interface**: Uses `recording_connection_*()` functions directly
- **Manual Control**: Full control over recording interface operations
- **Flexibility**: Custom stream management and recording behavior
- **Advanced Usage**: For applications requiring specialized recording handling

## Stream Management Options

### Connection-Managed Streams (`USER_RECORDING_STREAMS = 0`)
- **Connection Managed**: Files opened and closed by the connection interface
- **Simple Setup**: Specify filenames for automatic recording
- **Default Behavior**: Recommended for most applications

### User-Managed Streams (`USER_RECORDING_STREAMS = 1`)
- **Manual Control**: User creates and manages FILE* streams
- **Custom Streams**: Can use any stream type (files, memory, network, etc.)
- **Advanced Usage**: For applications with specific stream requirements
- **Explicit Cleanup**: User responsible for closing streams

## Data Recording Process

The example performs the following recording sequence:

1. **Device Discovery**: Ping the device and verify communication
2. **Device Information**: Query and display complete device information
3. **Capability Detection**: Get all supported descriptors (standard and extended)
4. **Factory Reset**: Reset streaming to factory defaults
5. **Stream Configuration**: Enable streaming for all supported data descriptors
6. **Recording Start**: Begin recording all data streams to binary files
7. **Continuous Recording**: Record until signal interrupt (Ctrl+C)
8. **Graceful Shutdown**: Clean up connections and close recording files

## C Implementation Features

This example showcases:
- **Recording Interface**: Complete C recording interface usage
- **Factory Streaming**: Automatic configuration of all supported data streams
- **Device Capability Detection**: Dynamic discovery of device features
- **Flexible Recording**: Multiple recording configuration options
- **Signal Handling**: Proper interrupt handling for recording applications
- **Memory Management**: Careful resource management and cleanup
- **Binary Recording**: Raw communication data recording for analysis

## Usage

1. Connect your device to the specified serial port
2. Update the `PORT_NAME` constant if using a different port
3. Optionally modify recording filenames in `RECEIVED_BYTES_BINARY` and `SENT_BYTES_BINARY`
4. Choose recording configuration by setting `CONNECTION_RECORDING_WRAPPERS` and `USER_RECORDING_STREAMS` flags
5. Compile and run the example
6. The program will:
   - Initialize the device and query supported descriptors
   - Configure factory streaming for all supported data types
   - Begin recording all supported data streams to binary files
   - Display recording status with an animated progress indicator
   - Continue until interrupted with Ctrl+C
   - Clean up and close all recording files

## Recording Output

The example generates two binary files:

### Received Data (`received_bytes.bin`)
- **Content**: All data received from the device
- **Format**: Raw binary MIP packets
- **Includes**: Sensor data, status messages, command responses
- **Analysis**: Can be parsed using MIP packet parsing tools

### Sent Data (`sent_bytes.bin`)
- **Content**: All commands sent to the device
- **Format**: Raw binary MIP command packets
- **Includes**: Configuration commands, queries, control messages
- **Debugging**: Shows the exact command sequence used

## Error Handling

The example includes comprehensive error handling with:
- Command result checking using `mip_cmd_result`
- Graceful termination functions for different error types
- Detailed error messages with context using built-in documentation strings
- Recording stream error handling and cleanup
- Device communication error recovery

## Requirements

- Any MicroStrain device
- Serial connection (USB or RS-232)
- MIP SDK library with C support
- C11 or later compiler

## See Also

- C++ version: `recording_example.cpp`
- Other examples in the `examples/` directory
- MIP SDK documentation
