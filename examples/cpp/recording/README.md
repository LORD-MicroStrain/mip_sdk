# Recording Example (C++)

This example demonstrates how to stream and record factory streaming data from a MicroStrain device using the MIP SDK
C++ API.

## Overview

The example showcases fundamental data streaming and recording setup and operation, including:
- Device initialization and communication with factory streaming using modern C++ interfaces
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
| `CONNECTION_RECORDING_WRAPPERS` | `1`                    | Initialize recording with the connection vs after |
| `USER_RECORDING_STREAMS`        | `0`                    | Use user-managed streams vs connection-managed    |

## Key Functions

### Device Setup
- `initializeDevice()` - Establishes communication, validates connection, loads defaults, and configures factory
  streaming
- `getSupportedDataDescriptors()` - Extracts supported data descriptor sets from device capabilities
- Modern C++ connection initialization with `microstrain::connections::SerialConnection`

### Factory Streaming Configuration
- Automatically queries device descriptors using `mip::commands_base::getDeviceDescriptors()`
- Extends descriptor queries with `mip::commands_base::getExtendedDescriptors()` for comprehensive coverage
- Resets and enables factory streaming for all supported data descriptors
- Uses `mip::commands_3dm::factoryStreaming()` and `mip::commands_3dm::writeDatastreamControl()`

### Communication Recording
- Modern C++ recording interface with RAII resource management
- Flexible recording options: wrapper functions or raw recording interface
- Support for both connection-managed and user-managed stream recording
- Optional recording streams that can be enabled/disabled via preprocessor flags
- **Individual stream initialization**: Both receive and send streams/files are not required for recording and can
  be initialized and used independently of each other using the respective `Receive` and `Send` variants of
  functions within the connection and recording interfaces

## Recording Implementation Features

This example showcases two recording approaches:

### Wrapper Recording (`CONNECTION_RECORDING_WRAPPERS = 1`)
- **Constructor-Based**: Recording configured directly in connection constructor
- **Automatic Management**: Recording files opened and closed automatically
- **RAII Pattern**: Resources managed through object lifetime
- **Simplified API**: Single constructor call for recording setup

### Manual Initialization (`CONNECTION_RECORDING_WRAPPERS = 0`)
- **Two-Phase Setup**: Connection created first, then recording initialized
- **Explicit Control**: Separate calls for connection and recording setup
- **Flexibility**: Custom initialization sequences and error handling
- **Advanced Usage**: For applications requiring specialized setup

## Stream Management Options

### Connection-Managed Streams (`USER_RECORDING_STREAMS = 0`)
- **Connection Managed**: Files opened and closed by the connection object
- **RAII Cleanup**: Automatic cleanup when the connection object is destroyed
- **Simple Setup**: Specify filenames for automatic recording
- **Default Behavior**: Recommended for most C++ applications

### User-Managed Streams (`USER_RECORDING_STREAMS = 1`)
- **Manual Control**: User creates and manages FILE* streams
- **Custom Streams**: Can use any stream type (files, memory, network, etc.)
- **Advanced Usage**: For applications with specific stream requirements
- **Explicit Cleanup**: User responsible for closing streams

## Data Recording Process

The example performs the following recording sequence:

1. **Connection Setup**: Create SerialConnection with recording configuration
2. **Device Discovery**: Ping the device and verify communication
3. **Device Information**: Query and display complete device information
4. **Capability Detection**: Get all supported descriptors (standard and extended)
5. **Factory Reset**: Reset streaming to factory defaults
6. **Stream Configuration**: Enable streaming for all supported data descriptors
7. **Recording Start**: Begin recording all data streams to binary files
8. **Continuous Recording**: Record until signal interrupt (Ctrl+C)
9. **RAII Cleanup**: Automatic connection and recording cleanup

## C++ Implementation Features

This example showcases:
- **MIP Interface**: Modern C++ interface for device communication (`mip::Interface`)
- **Serial Connection**: C++ connection class with automatic resource management
- **Type-Safe Data Structures**: C++ data classes for sensor measurements
- **Template-Based Callbacks**: Compile-time type checking for callback registration
- **Standard Library**: Modern C++ features and STL usage
- **Recording Support**: Built-in communication recording with C++ streams

## Connection Management

The C++ version uses modern connection handling:
- **SerialConnection**: RAII-based serial connection management
- **Automatic Cleanup**: Connection automatically closed when the object goes out of scope
- **Exception Safety**: Proper resource cleanup even when errors occur
- **Recording Integration**: Built-in recording support with file management

## Usage

1. Connect your device to the specified serial port
2. Update the `PORT_NAME` constant if using a different port
3. Optionally modify recording filenames in `RECEIVED_BYTES_BINARY` and `SENT_BYTES_BINARY`
4. Choose recording configuration by setting `CONNECTION_RECORDING_WRAPPERS` and `USER_RECORDING_STREAMS` flags
5. Compile and run the example
6. The program will:
   - Initialize the connection with recording configuration
   - Initialize the device and query supported descriptors
   - Configure factory streaming for all supported data types
   - Begin recording all supported data streams to binary files
   - Display recording status with an animated progress indicator
   - Continue until interrupted with Ctrl+C
   - Automatically cleans up connections and recording files

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

- Any MicroStrain device
- Serial connection (USB or RS-232)
- MIP SDK library with C++ support
- C++11 or later compiler

## See Also

- C version: `recording_example.c`
- Other examples in the `examples/` directory
- MIP SDK documentation
