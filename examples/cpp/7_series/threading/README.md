# 7 Series Threading Example (C++)

This example demonstrates how to implement multithreading for simultaneous data collection and command operations on
MicroStrain 7-series devices using the C++ API.

## Overview

The example showcases advanced threading concepts for 7-series devices, including:
- Concurrent data collection and command execution
- Thread-safe device communication
- Race condition prevention and stress testing
- Modern C++ threading primitives
- Proper thread synchronization and cleanup

## Configuration

The example uses the following default settings:

| Setting            | Value                                         | Description                             |
|--------------------|-----------------------------------------------|-----------------------------------------|
| `PORT_NAME`        | `"COM1"` (Windows)<br>`"/dev/ttyACM0"` (Unix) | Serial port for device communication    |
| `BAUDRATE`         | `115200`                                      | Communication baud rate                 |
| `SAMPLE_RATE_HZ`   | `1`                                           | Data output rate in Hz                  |
| `RUN_TIME_SECONDS` | `30`                                          | Example runtime duration                |
| `USE_THREADS`      | `true`                                        | Enable/disable threading (compile-time) |

## Key Functions

### Device Setup
- `initializeDevice()` - Establishes serial communication and validates device connection

### Message Configuration
- `configureSensorMessageFormat()` - Configures sensor data output including:
    - Scaled accelerometer

### Threading Infrastructure
- `updateDevice()` - Custom update function that handles thread context switching
- `dataCollectionThread()` - Dedicated thread function for continuous data collection
- `packetCallback()` - Processes received MIP packets and displays field information

### Communication Interface
- Uses the `mip::Interface` class for device communication
- Serial connection handled by `microstrain::connections::SerialConnection`

## Threading Architecture

The C++ version uses modern threading features:
- **std::thread**: Standard C++ threading library
- **Thread References**: Safe reference passing to thread functions
- **Volatile Flags**: Thread-safe communication using volatile references
- **Automatic Cleanup**: RAII-based resource management
- **Exception Safety**: Proper error handling across threads

## Threading Implementation

This example showcases:
- **Modern C++ Threading**: std::thread, std::chrono, and threading utilities
- **RAII Thread Management**: Automatic thread resource management
- **Reference Semantics**: Safe reference passing between threads
- **Chrono Time Management**: Modern C++ time handling

## Thread Safety Features

The example includes comprehensive thread safety:
- **Single Command Channel**: Only one thread can send commands at a time
- **Data Collection Isolation**: Separate thread for continuous data processing
- **Connection State Management**: Safe handling of connection failures across threads
- **Graceful Shutdown**: Proper thread termination and resource cleanup

## Data Handling

The C++ version uses modern features including:
- **Type-Safe Callbacks**: Compile-time verified callback functions
- **Range-Based Loops**: Modern iteration over packet fields
- **RAII**: Automatic resource management for connections and threads

## C++ Implementation Features

This example demonstrates:
- **Modern C++ Connection Management**: RAII-based resource handling
- **Type-Safe MIP Command Interfaces**: Compile-time type checking
- **Exception Safety**: Proper error handling and resource cleanup
- **STL Integration**: Use of standard library containers and algorithms

## Usage

1. Connect your 7-series device to the specified serial port
2. Update the `PORT_NAME` constant if using a different port
3. Set `USE_THREADS` to `true` to enable threading (default)
4. Compile and run the example
5. The program will:
    - Initialize the device and threading infrastructure
    - Start a dedicated data collection thread
    - Execute stress testing with rapid ping commands
    - Display received packet information
    - Perform thread cleanup and exit

## Error Handling

The example includes comprehensive error handling with:
- Exception-safe thread operations
- Command result checking using `mip::CmdResult`
- Connection failure detection and recovery
- Graceful termination functions for different error types
- Detailed error messages with context

## C++ Features

This example demonstrates:
- Modern C++ connection management
- Type-safe MIP command interfaces
- Automatic data field extraction
- RAII resource management
- Standard library integration

## Platform Compatibility

The example uses standard C++ threading:
- **Cross-Platform**: std::thread works on all supported platforms
- **Standard Library**: No external threading library dependencies
- **Modern C++**: Requires C++11 or later compiler support

## Requirements

- MicroStrain 7-series device
- Serial connection (USB or RS-232)
- MIP SDK library with C++ support
- C++11 or later compiler
- std::thread support

## See Also

- C version: `7_series_threading_example.c`
- Other examples in the `examples/` directory
- MIP SDK documentation
- C++ threading and concurrency documentation
