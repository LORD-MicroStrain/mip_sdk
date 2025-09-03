# 7 Series Threading Example (C)

This example demonstrates how to implement multithreading for simultaneous data collection and command operations on
MicroStrain 7-series devices using the C API.

## Overview

The example showcases advanced threading concepts for 7-series devices, including:
- Concurrent data collection and command execution
- Thread-safe device communication
- Race condition prevention and stress testing
- Custom update function implementation
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
- `initialize_device()` - Establishes serial communication and validates device connection

### Message Configuration
- `configure_sensor_message_format()` - Configures sensor data output including:
    - Scaled accelerometer

### Threading Infrastructure
- `update_device()` - Custom update function that handles thread context switching
- `data_collection_thread()` - Dedicated thread function for continuous data collection
- `packet_callback()` - Processes received MIP packets and displays field information

### Communication Interface
- `mip_interface_user_send_to_device()` - Sends commands to the device
- `mip_interface_user_recv_from_device()` - Receives data from the device

## Threading Architecture

The C version demonstrates advanced threading patterns:
- **Dual-Context Updates**: Different behavior for command vs. data collection contexts
- **Race Condition Testing**: Stress testing with rapid ping commands
- **Thread Synchronization**: Proper thread creation, joining, and cleanup
- **Cross-Platform Threading**: Portable pthread implementation with MSVC compatibility
- **Resource Management**: Safe cleanup on connection failures

## Threading Implementation

This example showcases:
- **POSIX Threads**: Standard pthread library for cross-platform threading
- **Thread Data Structure**: Custom `thread_data_t` structure for thread communication
- **Volatile Flags**: Thread-safe communication using volatile boolean flags
- **Command Queue Management**: Prevention of deadlocks through queue clearing
- **Context Switching**: Different update behaviors based on calling context

## Thread Safety Features

The example includes comprehensive thread safety:
- **Single Command Channel**: Only one thread can send commands at a time
- **Data Collection Isolation**: Separate thread for continuous data processing
- **Connection State Management**: Safe handling of connection failures across threads
- **Graceful Shutdown**: Proper thread termination and resource cleanup

## C Implementation Features

This example showcases:
- **MIP Interface**: Core C interface for device communication (`mip_interface`)
- **Serial Port Management**: Low-level serial port operations
- **Memory Safety**: Careful buffer management and bound checking
- **Portability**: Cross-platform compatibility (Windows/Unix)

## Custom Communication Functions

The example implements thread-aware communication handlers:
- **Send Function**: Thread-safe packet transmission to the device
- **Receive Function**: Thread-safe packet reception with timestamping
- **Update Function**: Context-aware device state updates
- **Stress Testing**: Rapid command execution to test thread safety

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
- Thread creation and joining error checking
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

## Platform Compatibility

The example includes platform-specific threading support:
- **Unix/Linux**: Native pthread support
- **Windows (MSVC)**: Custom pthread wrapper using C11 threads.h
- **Cross-Platform**: Unified interface regardless of platform

## Requirements

- MicroStrain 7-series device
- Serial connection (USB or RS-232)
- MIP SDK library with C support
- C11 or later compiler
- pthread library (Unix) or threads.h support (MSVC)

## See Also

- C++ version: `7_series_threading_example.cpp`
- Other examples in the `examples/` directory
- MIP SDK documentation
- POSIX Threads programming guide
