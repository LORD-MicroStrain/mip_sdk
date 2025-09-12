# Examples

This directory contains comprehensive examples demonstrating how to use the MIP SDK with MicroStrain inertial 
sensors. The examples are organized by device series and programming language, providing both C and C++ 
implementations for most use cases.

## MicroStrain Device Overview

### 5-Series Devices

The MicroStrain 5-series devices are industrial-grade inertial sensors featuring fully calibrated and
temperature-compensated sensors with high-performance capabilities.

| Device Model                                                                                                                                 | Description                                                              |
|----------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------|
| [3DM-CX5-IMU](https://www.hbkworld.com/en/products/transducers/inertial-sensors/inertial-measurement-units--imu-/3dm-cx5-imu)                | Smallest and lightest high performance IMU with fully calibrated sensors |
| [3DM-CX5-AR](https://www.hbkworld.com/en/products/transducers/inertial-sensors/vertical-reference-units--vru-/3dm-cx5-ar)                    | Embeddable high performance vertical reference unit and tilt sensor      |
| [3DM-CX5-AHRS](https://www.hbkworld.com/en/products/transducers/inertial-sensors/attitude-and-heading-reference-systems--ahrs-/3dm-cx5-ahrs) | Smallest and lightest industrial AHRS with adaptive Kalman filter        |
| [3DM-CX5-GNSS/INS](https://www.hbkworld.com/en/products/transducers/inertial-sensors/inertial-navigation-systems--ins-/3dm-cx5-gnss-ins)     | All-in-one navigation solution with multi-constellation GNSS receiver    |
| [3DM-CV5-IMU](https://www.hbkworld.com/en/products/transducers/inertial-sensors/inertial-measurement-units--imu-/3dm-cv5-imu)                | Value-focused embeddable IMU with calibrated sensors                     |
| [3DM-CV5-AR](https://www.hbkworld.com/en/products/transducers/inertial-sensors/vertical-reference-units--vru-/3dm-cv5-ar)                    | Vertical reference unit for attitude determination                       |
| [3DM-CV5-AHRS](https://www.hbkworld.com/en/products/transducers/inertial-sensors/attitude-and-heading-reference-systems--ahrs-/3dm-cv5-ahrs) | Smallest and lightest industrial AHRS with adaptive Kalman filter        |
| [3DM-GX5-IMU](https://www.hbkworld.com/en/products/transducers/inertial-sensors/inertial-measurement-units--imu-/3dm-gx5-imu)                | High performance IMU with precision-machined aluminum housing            |
| [3DM-GX5-AHRS](https://www.hbkworld.com/en/products/transducers/inertial-sensors/attitude-and-heading-reference-systems--ahrs-/3dm-gx5-ahrs) | Industry's most proven high performance industrial AHRS                  |
| [3DM-GX5-AR](https://www.hbkworld.com/en/products/transducers/inertial-sensors/vertical-reference-units--vru-/3dm-gx5-ar)                    | High performance vertical reference unit                                 |
| [3DM-GX5-GNSS/INS](https://www.hbkworld.com/en/products/transducers/inertial-sensors/inertial-navigation-systems--ins-/3dm-gx5-gnss-ins)     | Navigation solution with integrated GNSS receiver                        |


### 7-Series Devices

The MicroStrain 7-series devices represent the latest generation of tactical-grade inertial sensors with advanced 
capabilities, compact designs, and ruggedizedIP68 packages.

| Device Model                                                                                                                                 | Description                                                                      |
|----------------------------------------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------|
| [3DM-GQ7-GNSS/INS](https://www.hbkworld.com/en/products/transducers/inertial-sensors/inertial-navigation-systems--ins-/3dm-gq7-gnss-ins)     | Dual-antenna RTK GNSS/INS with centimeter-level accuracy                         |
| [3DM-CV7-AR](https://www.hbkworld.com/en/products/transducers/inertial-sensors/vertical-reference-units--vru-/3dm-cv7-ar)                    | Advanced vertical reference unit with cutting-edge algorithms                    |
| [3DM-CV7-AHRS](https://www.hbkworld.com/en/products/transducers/inertial-sensors/attitude-and-heading-reference-systems--ahrs-/3dm-cv7-ahrs) | Tactical grade AHRS with 1.5?/hr gyro bias instability                           |
| [3DM-CV7-INS](https://www.hbkworld.com/en/products/transducers/inertial-sensors/inertial-navigation-systems--ins-/3dm-cv7-ins)               | Compact INS solution with tactical grade performance                             |
| [3DM-CV7-GNSS/INS](https://www.hbkworld.com/en/products/transducers/inertial-sensors/inertial-navigation-systems--ins-/3dm-cv7-gnss-ins)     | RTK-capable GNSS/INS with dual-frequency receiver and tactical grade performance |
| [3DM-GV7-AR](https://www.hbkworld.com/en/products/transducers/inertial-sensors/vertical-reference-units--vru-/3dm-gv7-ar)                    | IP68 ruggedized vertical reference unit for harsh environments                   |
| [3DM-GV7-AHRS](https://www.hbkworld.com/en/products/transducers/inertial-sensors/attitude-and-heading-reference-systems--ahrs-/3dm-gv7-ahrs) | IP68 ruggedized tactical grade AHRS with precision machined enclosure            |
| [3DM-GV7-INS](https://www.hbkworld.com/en/products/transducers/inertial-sensors/inertial-navigation-systems--ins-/3dm-gv7-ins)               | Ruggedized tactical grade INS solution                                           |

## Examples Overview

The examples demonstrate various aspects of working with MicroStrain devices, from basic sensor data streaming to 
advanced navigation solutions. Each example includes comprehensive documentation and error handling.

| Example Name                 | C                          | C++                            | Description                                                                                           |
|------------------------------|----------------------------|--------------------------------|-------------------------------------------------------------------------------------------------------|
| **5-Series Examples**        |                            |                                |                                                                                                       |
| 5_series_ahrs_example        | [C](c/5_series/ahrs)       | [C++](cpp/5_series/ahrs)       | Configure and stream AHRS data from 5-series devices with filter initialization and gyro bias capture |
| 5_series_ar_example          | [C](c/5_series/ar)         | [C++](cpp/5_series/ar)         | Attitude reference system example demonstrating roll/pitch estimation from 5-series devices           |
| 5_series_gnss_ins_example    | [C](c/5_series/gnss_ins)   | [C++](cpp/5_series/gnss_ins)   | Full navigation solution combining GNSS and inertial data for position, velocity, and attitude        |
| 5_series_stream_imu_example  | [C](c/5_series/stream_imu) | [C++](cpp/5_series/stream_imu) | Basic IMU sensor data streaming with accelerometer, gyroscope, and magnetometer output                |
| 5_series_threading_example   | [C](c/5_series/threading)  | [C++](cpp/5_series/threading)  | Multi-threaded data streaming example showing concurrent data processing patterns                     |
| **7-Series Examples**        |                            |                                |                                                                                                       |
| 7_series_ahrs_example        | [C](c/7_series/ahrs)       | [C++](cpp/7_series/ahrs)       | AHRS configuration and streaming for tactical-grade 7-series devices                                  |
| 7_series_ar_example          | [C](c/7_series/ar)         | [C++](cpp/7_series/ar)         | Attitude reference system for 7-series devices with enhanced precision                                |
| 7_series_gnss_ins_example    | [C](c/7_series/gnss_ins)   | [C++](cpp/7_series/gnss_ins)   | Advanced navigation solution with 7-series tactical grade sensors                                     |
| 7_series_ins_example         | [C](c/7_series/ins)        | [C++](cpp/7_series/ins)        | Complete INS example with external aiding measurements and reference frame configuration              |
| 7_series_stream_imu_example  | [C](c/7_series/stream_imu) | [C++](cpp/7_series/stream_imu) | High-precision IMU data streaming from 7-series tactical grade sensors                                |
| 7_series_threading_example   | [C](c/7_series/threading)  | [C++](cpp/7_series/threading)  | Multi-threaded processing example optimized for 7-series device capabilities                          |
| **Device Agnostic Examples** |                            |                                |                                                                                                       |
| device_info_example          | [C](c/device_info)         | [C++](cpp/device_info)         | Retrieve and display comprehensive device information from any MIP-enabled device                     |
| mip_packet_example           | [C](c/mip_packet)          | [C++](cpp/mip_packet)          | Create, manipulate, and work with raw MIP packets for custom applications                             |

## Key Features Demonstrated

### Device Communication
- Serial port configuration and management
- MIP protocol packet handling
- Error detection and recovery
- Cross-platform compatibility (Windows/Unix)

### Data Streaming
- Real-time sensor data output
- Configurable sample rates
- Multiple data format support
- Dynamic sensor capability detection

### Filter Operations
- Gyroscope bias capture and compensation
- Filter initialization and configuration
- Heading source selection
- External aiding integration

### Programming Patterns
- **C Examples**: Traditional C patterns with manual memory management, explicit error checking, and direct hardware 
  interface programming
- **C++ Examples**: Modern C++ features including RAII, type safety, automatic data extraction, and STL integration

## Getting Started

1. **Hardware Setup**: Connect your MicroStrain device to a serial port (USB or RS-232)
2. **Build System**: Use the provided CMakeLists.txt files to build examples with CMake
3. **Configuration**: Update serial port settings in examples as needed
4. **Run Examples**: Execute compiled examples to see device interaction

## Common Configuration

Most examples use these default settings:
- **Port**: `"COM1"` (Windows) or `"/dev/ttyACM0"` (Unix)
- **Baud Rate**: `115200`
- **Sample Rate**: `1 Hz`
- **Runtime**: `30 seconds`

## Error Handling

All examples include comprehensive error handling:
- Command result validation
- Connection failure detection
- Graceful termination procedures
- Detailed error messages with context

## Requirements

- MicroStrain 5-series or 7-series device
- Serial connection capability
- MIP SDK library
- C11 or C++11 compiler (or later)

## Additional Resources

- [MIP SDK Documentation](https://lord-microstrain.github.io/mip_sdk_documentation/)
- [MicroStrain Inertial Products](https://www.hbkworld.com/en/products/transducers/inertial-sensors/)
- Individual example README files for detailed implementation information

Each example directory contains detailed documentation, source code, and build instructions.
Refer to the individual README files for specific implementation details and usage instructions.
