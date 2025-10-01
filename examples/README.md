# Examples

This directory contains comprehensive examples demonstrating how to use the MIP SDK with MicroStrain inertial 
sensors. The examples are organized by device series and programming language, providing both C and C++ 
implementations for most use cases.

## MicroStrain Inertial Device Overview

### 7-Series Devices

The MicroStrain 7-series devices represent the latest generation of tactical-grade inertial sensors with advanced
capabilities, compact designs, and flexible packaging options.

| Device Model                                                                                                        | Description                                                     |
|---------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------|
| [3DM-GQ7-GNSS/INS](https://www.hbkworld.com/en/products/transducers/inertial-sensors/navigation/3dm-gq7-gnss-ins)   | Dual-antenna RTK GNSS/INS with centimeter-level accuracy        |
| [3DM-CV7-AR](https://www.hbkworld.com/en/products/transducers/inertial-sensors/vertical-reference/3dm-cv7-ar)       | Embeddable tactical-grade IMU/VRU                               |
| [3DM-CV7-AHRS](https://www.hbkworld.com/en/products/transducers/inertial-sensors/attitude-and-heading/3dm-cv7-ahrs) | Embeddable tactical grade IMU/AHRS                              |
| [3DM-CV7-INS](https://www.hbkworld.com/en/products/transducers/inertial-sensors/navigation/3dm-cv7-ins)             | Embeddable tactical grade Inertial Navigation System            |
| [3DM-CV7-GNSS/INS](https://www.hbkworld.com/en/products/transducers/inertial-sensors/navigation/3dm-cv7-gnss-ins)   | Embeddable tactical grade GNSS-aided Inertial Navigation System |
| [3DM-GV7-AR](https://www.hbkworld.com/en/products/transducers/inertial-sensors/vertical-reference/3dm-gv7-ar)       | IP68 ruggedized tactical grade IMU/VRU                          |
| [3DM-GV7-AHRS](https://www.hbkworld.com/en/products/transducers/inertial-sensors/attitude-and-heading/3dm-gv7-ahrs) | IP68 ruggedized tactical grade IMU/AHRS                         |
| [3DM-GV7-INS](https://www.hbkworld.com/en/products/transducers/inertial-sensors/navigation/3dm-gv7-ins)             | IP68 ruggedized tactical grade Inertial Navigation System       |

### 5-Series Devices

The MicroStrain 5-series devices are industrial-grade inertial sensors featuring fully calibrated and
temperature-compensated sensors with high-performance capabilities.

| Device Model                                                                                                        | Description                                                                                     |
|---------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------|
| [3DM-CX5-IMU](https://www.hbkworld.com/en/products/transducers/inertial-sensors/measurement-units/3dm-cx5-imu)      | Embeddable high performance IMU                                                                 |
| [3DM-CX5-AR](https://www.hbkworld.com/en/products/transducers/inertial-sensors/vertical-reference/3dm-cx5-ar)       | Embeddable high performance IMU/VRU                                                             |
| [3DM-CX5-AHRS](https://www.hbkworld.com/en/products/transducers/inertial-sensors/attitude-and-heading/3dm-cx5-ahrs) | Embeddable high performance IMU/AHRS                                                            |
| [3DM-CX5-GNSS/INS](https://www.hbkworld.com/en/products/transducers/inertial-sensors/navigation/3dm-cx5-gnss-ins)   | Embeddable high performance GNSS-aided Inertial Navigation System                               |
| [3DM-CV5-IMU](https://www.hbkworld.com/en/products/transducers/inertial-sensors/measurement-units/3dm-cv5-imu)      | Embeddable industrial grade IMU                                                                 |
| [3DM-CV5-AR](https://www.hbkworld.com/en/products/transducers/inertial-sensors/vertical-reference/3dm-cv5-ar)       | Embeddable industrial grade IMU/VRU                                                             |
| [3DM-CV5-AHRS](https://www.hbkworld.com/en/products/transducers/inertial-sensors/attitude-and-heading/3dm-cv5-ahrs) | Embeddable industrial grade IMU/AHRS                                                            |
| [3DM-GX5-IMU](https://www.hbkworld.com/en/products/transducers/inertial-sensors/measurement-units/3dm-gx5-imu)      | High performance IMU in precision anodized aluminum enclosure                                   |
| [3DM-GX5-AR](https://www.hbkworld.com/en/products/transducers/inertial-sensors/vertical-reference/3dm-gx5-ar)       | High performance IMU/VRU in precision anodized aluminum enclosure                               |
| [3DM-GX5-AHRS](https://www.hbkworld.com/en/products/transducers/inertial-sensors/attitude-and-heading/3dm-gx5-ahrs) | High performance IMU/AHRS in precision anodized aluminum enclosure                              |
| [3DM-GX5-GNSS/INS](https://www.hbkworld.com/en/products/transducers/inertial-sensors/navigation/3dm-gx5-gnss-ins)   | High performance GNSS-aided Inertial Navigation System in precision anodized aluminum enclosure |

## Examples Overview

The examples demonstrate various aspects of working with MicroStrain devices, from basic sensor data streaming to 
advanced navigation solutions. Each example includes comprehensive documentation and error handling.

<table class="markdownTable">
  <thead class="markdownTableHead">
    <tr align="center">
      <th class="markdownTableHeadNone"> Name </th>
      <th class="markdownTableHeadNone"> C </th>
      <th class="markdownTableHeadNone"> C++ </th>
      <th class="markdownTableHeadNone"> Description </th>
    </tr>
  </thead>
  <tbody>
    <tr align="center" class="markdownTableRowOdd">
      <td rowspan="4"> AR Filter Examples </td>
      <td rowspan="2"><a href="c/5_series/ar">5_series_ar_example_c</a></td>
      <td rowspan="2"><a href="cpp/5_series/ar">5_series_ar_example_cpp</a></td>
      <td rowspan="4">Basic attitude filter configuration for streaming filter data</td>
    </tr>
    <tr></tr>
    <tr align="center" class="markdownTableRowOdd">
      <td rowspan="2"><a href="c/7_series/ar">7_series_ar_example_c</a></td>
      <td rowspan="2"><a href="cpp/7_series/ar">7_series_ar_example_cpp</a></td>
    </tr>
    <tr></tr>
    <tr></tr>
    <tr align="center" class="markdownTableRowEven">
      <td rowspan="4"> AHRS Filter Examples </td>
      <td rowspan="2"><a href="c/5_series/ahrs">5_series_ahrs_example_c</a></td>
      <td rowspan="2"><a href="cpp/5_series/ahrs">5_series_ahrs_example_cpp</a></td>
      <td rowspan="4">Basic attitude filter configuration with magnetometer heading for streaming filter data</td>
    </tr>
    <tr></tr>
    <tr align="center" class="markdownTableRowEven">
      <td rowspan="2"><a href="c/7_series/ahrs">7_series_ahrs_example_c</a></td>
      <td rowspan="2"><a href="cpp/7_series/ahrs">7_series_ahrs_example_cpp</a></td>
    </tr>
    <tr></tr>
    <tr></tr>
    <tr align="center" class="markdownTableRowOdd">
      <td> INS External Aiding Examples </td>
      <td><a href="c/7_series/ins">7_series_ins_example_c</a></td>
      <td><a href="cpp/7_series/ins">7_series_ins_example_cpp</a></td>
      <td>Complete INS example with external aiding measurements and reference frame configuration using simulated external data</td>
    </tr>
    <tr align="center" class="markdownTableRowEven">
      <td rowspan="4"> GNSS/INS Navigation Examples </td>
      <td rowspan="2"><a href="c/5_series/gnss_ins">5_series_gnss_ins_example_c</a></td>
      <td rowspan="2"><a href="cpp/5_series/gnss_ins">5_series_gnss_ins_example_cpp</a></td>
      <td rowspan="4">Full navigation solution combining GNSS and inertial data for position, velocity, and attitude</td>
    </tr>
    <tr></tr>
    <tr align="center" class="markdownTableRowEven">
      <td rowspan="2"><a href="c/7_series/gnss_ins">7_series_gnss_ins_example_c</a></td>
      <td rowspan="2"><a href="cpp/7_series/gnss_ins">7_series_gnss_ins_example_cpp</a></td>
    </tr>
    <tr></tr>
    <tr></tr>
    <tr align="center" class="markdownTableRowOdd">
      <td rowspan="4"> IMU Data Streaming Examples </td>
      <td rowspan="2"><a href="c/5_series/stream_imu">5_series_stream_imu_example_c</a></td>
      <td rowspan="2"><a href="cpp/5_series/stream_imu">5_series_stream_imu_example_cpp</a></td>
      <td rowspan="4">Basic IMU sensor data streaming with accelerometer, gyroscope, and magnetometer output</td>
    </tr>
    <tr></tr>
    <tr align="center" class="markdownTableRowOdd">
      <td rowspan="2"><a href="c/7_series/stream_imu">7_series_stream_imu_example_c</a></td>
      <td rowspan="2"><a href="cpp/7_series/stream_imu">7_series_stream_imu_example_cpp</a></td>
    </tr>
    <tr></tr>
    <tr></tr>
    <tr align="center" class="markdownTableRowEven">
      <td rowspan="4"> Multithreaded Examples </td>
      <td rowspan="2"><a href="c/5_series/threading">5_series_threading_example_c</a></td>
      <td rowspan="2"><a href="cpp/5_series/threading">5_series_threading_example_cpp</a></td>
      <td rowspan="4">Multithreaded data streaming example showing concurrent data processing patterns</td>
    </tr>
    <tr></tr>
    <tr align="center" class="markdownTableRowEven">
      <td rowspan="2"><a href="c/7_series/threading">7_series_threading_example_c</a></td>
      <td rowspan="2"><a href="cpp/7_series/threading">7_series_threading_example_cpp</a></td>
    </tr>
    <tr></tr>
    <tr></tr>
    <tr align="center" class="markdownTableRowOdd">
      <td> Device Information Examples </td>
      <td><a href="c/device_info">device_info_example_c</a></td>
      <td><a href="cpp/device_info">device_info_example_cpp</a></td>
      <td>Retrieve and display device information from any MIP-enabled device</td>
    </tr>
    <tr align="center" class="markdownTableRowEven">
      <td> MIP Packet Examples </td>
      <td><a href="c/mip_packet">mip_packet_example_c</a></td>
      <td><a href="cpp/mip_packet">mip_packet_example_cpp</a></td>
      <td>Create, manipulate, and work with raw MIP packets for custom applications</td>
    </tr>
  </tbody>
</table>

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
