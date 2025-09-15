# MIP Packet Example (C++)

This example demonstrates how to create, manipulate, and work with raw and buffered MIP packets with the MIP SDK 
using the C++ API.

## Overview

The example showcases basic MIP packet operations, including:
- Creating empty MIP packets
- Adding data fields to packets
- Computing and adding checksums
- Packet serialization and deserialization
- Working with different MIP descriptor sets
- Packet validation and error handling

## Configuration

This example does not require device connection and works entirely with in-memory packet operations.

## Key Functions

### Packet Creation and Management
- `initializeEmptyPacket()` - Creates an empty packet for a given descriptor set
- `addChecksumToPacket()` - Computes and adds the checksum to a packet
- `printPacket()` - Displays packet contents and structure

### Field Addition Functions
- Functions to add various MIP data fields to packets
- Support for different data types (sensor, shared, command data)
- Proper field serialization and formatting

### Packet Utilities
- Packet validation and integrity checking
- Buffer management for packet storage
- Cross-platform packet handling

## Data Handling

This example uses modern C++ features including:
- **Type Safety**: Strongly typed data structures for each message type
- **String Handling**: Safe C++ string operations

## C++ Implementation Features

This example demonstrates:
- **Modern C++ Packet API**: RAII-based packet management
- **Type-Safe MIP Command Interfaces**: Compile-time type checking
- **Exception Safety**: Proper error handling and resource cleanup
- **STL Integration**: Use of standard library containers and algorithms
- **Portability**: Cross-platform compatibility (Windows/Unix)

## Packet Operations Demonstrated

The example covers the following packet operations:
- **Empty Packet Creation**: Initialize packets for different descriptor sets
- **Field Addition**: Add various data fields to packets
- **Checksum Calculation**: Compute and validate packet checksums
- **Packet Validation**: Verify packet structure and integrity
- **Buffer Operations**: Manage packet buffers and memory
- **Data Extraction**: Parse and extract data from packets

## MIP Descriptor Sets

The example works with various MIP descriptor sets:
- **Base Commands**: Fundamental device commands
- **3DM Commands**: Device management commands
- **Sensor Data**: IMU and sensor data fields
- **Shared Data**: Common data structures across devices

## Usage

1. Compile and run the example (no device connection required)
2. The program will:
    - Create various MIP packets
    - Demonstrate packet manipulation
    - Show serialization/deserialization
    - Display packet contents
    - Validate packet integrity
    - Clean up and exit

## Error Handling

The example includes comprehensive error handling with:
- Packet validation checks
- Buffer overflow protection
- Serialization error detection

## C++ Features

This example demonstrates:
- Type-safe MIP command interfaces
- RAII resource management
- Standard library integration

## Type Safety and Documentation

This example provides additional C++ benefits:
- **Built-in Documentation**: Data structures include `DOC_NAME` constants for easy reference
- **Strongly Typed Enums**: C++ enum classes prevent accidental misuse
- **Automatic Descriptors**: `DESCRIPTOR` constants eliminate magic numbers

## Use Cases

This example is useful for:
- Understanding MIP packet structure
- Custom packet creation and parsing
- Debugging communication issues
- Learning MIP protocol internals
- Building custom MIP applications

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

## Requirements

- MIP SDK library with C++ support
- C++11 or later compiler
- No device connection required

## See Also

- C version: `mip_packet_example.c`
- Other examples in the `examples/` directory
- [MIP SDK documentation](https://lord-microstrain.github.io/mip_sdk_documentation/)
- MIP Protocol specification
