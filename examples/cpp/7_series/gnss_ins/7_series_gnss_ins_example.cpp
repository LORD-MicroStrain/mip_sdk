////////////////////////////////////////////////////////////////////////////////
/// @file 7_series_gnss_ins_example.cpp
///
/// @defgroup _7_series_gnss_ins_example_cpp 7-Series GNSS/INS Example [CPP]
///
/// @ingroup examples_cpp
///
/// @brief Example setup program for the 3DM-GQ7-GNSS/INS, and 3DM-CV7-GNSS/INS
///        using C++
///
/// @details This example shows a basic setup to configure the navigation filter
///          with GNSS heading, and GNSS position and velocity as the heading
///          sources to stream filter data for the 3DM-GQ7-GNSS/INS, and
///          3DM-CV7-GNSS/INS devices using C++. This is not an exhaustive
///          example of all settings for those devices. If this example does not
///          meet your specific setup needs, please consult the MIP SDK API
///          documentation for the proper commands.
///
/// @section _7_series_gnss_ins_example_cpp_license License
///
/// @copyright Copyright (c) 2025 MicroStrain by HBK
///            Licensed under MIT License
///
/// @{
///

// Include the MicroStrain Serial connection header
#include <microstrain/connections/serial/serial_connection.hpp>

// Include the MicroStrain logging header for custom logging
#include <microstrain/logging.hpp>

// Include all necessary MIP headers
// Note: The MIP SDK has headers for each module to include all headers associated with the module
// I.E., #include <mip/mip_all.hpp>
#include <mip/mip_interface.hpp>
#include <mip/definitions/commands_3dm.hpp>
#include <mip/definitions/commands_base.hpp>
#include <mip/definitions/commands_filter.hpp>
#include <mip/definitions/data_filter.hpp>
#include <mip/definitions/data_gnss.hpp>
#include <mip/definitions/data_shared.hpp>

#include <chrono>
#include <cinttypes>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>

////////////////////////////////////////////////////////////////////////////////
// NOTE: Setting these globally for example purposes

// TODO: Update to the correct port name and baudrate
/// @brief Set the port name for the connection (Serial/USB)
#ifdef _WIN32
static constexpr const char* PORT_NAME = "COM1";
#else  // Unix
static constexpr const char* PORT_NAME = "/dev/ttyACM0";
#endif // _WIN32

/// @brief Set the baudrate for the connection (Serial/USB)
/// @note For native serial connections this needs to be 115200 due to the device default settings command
// Use mip::commands_base::*CommSpeed() to write and save the baudrate on the device
static constexpr uint32_t BAUDRATE = 115200;

// TODO: Update to the desired streaming rate. Setting low for readability purposes
/// @brief Streaming rate in Hz
static constexpr uint16_t SAMPLE_RATE_HZ = 1;

// TODO: Update to change the example run time
/// @brief Example run time
static constexpr uint32_t RUN_TIME_SECONDS = 30;

// TODO: Turn on for wheeled-vehicle applications
/// @brief Enable configuration for wheeled-vehicle constraints
#define WHEELED_VEHICLE_APPLICATION false
////////////////////////////////////////////////////////////////////////////////

///
/// @} group _7_series_gnss_ins_example_cpp
////////////////////////////////////////////////////////////////////////////////

// Custom logging handler callback
static void logCallback(void* _user, const microstrain_log_level _level, const char* _format, va_list _args);

// Capture gyro bias
static void captureGyroBias(mip::Interface& _device);

// GNSS message format configuration
static void configureGnssMessageFormat(mip::Interface& _device, const uint8_t _gnssDataDescriptorSet);

// Filter message format configuration
static void configureFilterMessageFormat(mip::Interface& _device);

// Antenna offset configuration
static void configureAntennaOffset(mip::Interface& _device, const mip::Vector3f _antennaOffset, const uint8_t _antennaId);

// Filter initialization
static void initializeFilter(mip::Interface& _device);

// Utilities to display filter and GNSS state changes
static void displayGnssFixState(const mip::data_gnss::FixInfo* _fixInfoArray, const uint8_t _arrayIndex);
static void displayFilterState(const mip::data_filter::FilterMode _filterState);

// Used for basic timestamping (since epoch in milliseconds)
// TODO: Update this to whatever timestamping method is desired
static mip::Timestamp getCurrentTimestamp();

// Common device initialization procedure
static void initializeDevice(mip::Interface& _device);

// Utility functions the handle application closing and printing error messages
static void terminate(microstrain::Connection* _connection, const char* _message, const bool _successful = false);
static void terminate(mip::Interface& _device, const mip::CmdResult _cmdResult, const char* _format, ...);

int main(const int argc, const char* argv[])
{
    // Unused parameters
    (void)argc;
    (void)argv;

// Note: This is a compile-time way of checking that the proper logging level is enabled
// Note: The max available logging level may differ in pre-packaged installations of the MIP SDK
#ifndef MICROSTRAIN_LOGGING_ENABLED_INFO
#error This example requires a logging level of at least MICROSTRAIN_LOGGING_LEVEL_INFO_ to work properly
#endif // !MICROSTRAIN_LOGGING_ENABLED_INFO

    // Initialize the custom logger to print messages/errors as they occur
    // Note: The logging level parameter doesn't need to match the max logging level.
    // If the parameter is higher than the max level, higher-level logging functions will be ignored
    MICROSTRAIN_LOG_INIT(&logCallback, MICROSTRAIN_LOG_LEVEL_INFO, nullptr);

    // Initialize the connection
    MICROSTRAIN_LOG_INFO("Initializing the connection.\n");
    microstrain::connections::SerialConnection connection(PORT_NAME, BAUDRATE);

    MICROSTRAIN_LOG_INFO("Connecting to the device on port %s with %d baudrate.\n", PORT_NAME, BAUDRATE);

    // Open the connection to the device
    if (!connection.connect())
    {
        terminate(&connection, "Could not open the connection!\n");
    }

    MICROSTRAIN_LOG_INFO("Initializing the device interface.\n");
    mip::Interface device(
        &connection,                                 // Connection for the device
        mip::C::mip_timeout_from_baudrate(BAUDRATE), // Set the base timeout for commands (milliseconds)
        2000                                         // Set the base timeout for command replies (milliseconds)
    );
    initializeDevice(device);

    // Capture gyro bias
    captureGyroBias(device);

    // Configure the message format for GNSS data
    configureGnssMessageFormat(device, mip::data_gnss::MIP_GNSS1_DATA_DESC_SET);
    configureGnssMessageFormat(device, mip::data_gnss::MIP_GNSS2_DATA_DESC_SET);

    // Configure the message format for filter data
    configureFilterMessageFormat(device);

    // Configure Sensor-to-Vehicle Transformation
    MICROSTRAIN_LOG_INFO("Configuring %s.\n", mip::commands_3dm::Sensor2VehicleTransformEuler::DOC_NAME);
    mip::CmdResult cmdResult = mip::commands_3dm::writeSensor2VehicleTransformEuler(
        device,
        0.0f, // Roll
        0.0f, // Pitch
        0.0f  // Yaw
    );

    if (!cmdResult.isAck())
    {
        terminate(device, cmdResult, "Could not configure %s!\n", mip::commands_3dm::Sensor2VehicleTransformEuler::DOC_NAME);
    }

#if WHEELED_VEHICLE_APPLICATION
    // Configure the wheeled-vehicle constraint
    MICROSTRAIN_LOG_INFO("Enabling the %s.\n", mip::commands_filter::WheeledVehicleConstraintControl::DOC_NAME);
    cmdResult = mip::commands_filter::writeWheeledVehicleConstraintControl(
        device,
        1 // Enabled
    );

    if (!cmdResult.isAck())
    {
        terminate(
            device,
            cmdResult,
            "Could not enable the %s!\n",
            mip::commands_filter::WheeledVehicleConstraintControl::DOC_NAME
        );
    }
#endif // WHEELED_VEHICLE_APPLICATION

    // Configure the GNSS antenna offsets
    // Note: Antenna offsets are limited to a magnitude of [0.25, 10] (meters)
    MICROSTRAIN_LOG_INFO("Configuring the GNSS antenna offsets for dual-antenna.\n");

    // GNSS 1 antenna offset (in meters)
    const mip::Vector3f antennaOffset1 = {
        -0.25f, // X
        0.0f,   // Y
        0.0f    // Z
    };

    // GNSS 2 antenna offset (in meters)
    const mip::Vector3f antennaOffset2 = {
        0.25f, // X
        0.0f,  // Y
        0.0f   // Z
    };

    configureAntennaOffset(device, antennaOffset1, 1);
    configureAntennaOffset(device, antennaOffset2, 2);

    // Initialize the navigation filter
    initializeFilter(device);

    // Register data callbacks

    // Register GNSS data callbacks
    MICROSTRAIN_LOG_INFO("Registering GNSS data callbacks.\n");

    mip::DispatchHandler gnssDataHandlers[2];

    // Data stores for GNSS data
    mip::data_gnss::FixInfo gnssFixInfo[2]; // GNSS 1 & 2

    // Register the callbacks for the GNSS fields

    device.registerExtractor(
        gnssDataHandlers[0],
        &gnssFixInfo[0],                        // Data field out
        mip::data_gnss::MIP_GNSS1_DATA_DESC_SET // Data descriptor set
    );

    device.registerExtractor(
        gnssDataHandlers[1],
        &gnssFixInfo[1],                        // Data field out
        mip::data_gnss::MIP_GNSS2_DATA_DESC_SET // Data descriptor set
    );

    // Register filter data callbacks
    MICROSTRAIN_LOG_INFO("Registering filter data callbacks.\n");

    mip::DispatchHandler filterDataHandlers[5];

    // Data stores for filter data
    mip::data_shared::GpsTimestamp filterGpsTimestamp;
    mip::data_filter::Status       filterStatus;
    mip::data_filter::PositionLlh  filterPositionLlh;
    mip::data_filter::VelocityNed  filterVelocityNed;
    mip::data_filter::EulerAngles  filterEulerAngles;

    // Register the callbacks for the filter fields

    device.registerExtractor(
        filterDataHandlers[0],
        &filterGpsTimestamp,             // Data field out
        mip::data_filter::DESCRIPTOR_SET // Data descriptor set
    );

    device.registerExtractor(
        filterDataHandlers[1],
        &filterStatus // Data field out
    );

    device.registerExtractor(
        filterDataHandlers[2],
        &filterPositionLlh // Data field out
    );

    device.registerExtractor(
        filterDataHandlers[3],
        &filterVelocityNed // Data field out
    );

    device.registerExtractor(
        filterDataHandlers[4],
        &filterEulerAngles // Data field out
    );

    // Resume the device
    // Note: Since the device was idled for configuration, it needs to be resumed to output the data streams
    MICROSTRAIN_LOG_INFO("Resuming the device.\n");
    cmdResult = mip::commands_base::resume(device);

    if (!cmdResult.isAck())
    {
        terminate(device, cmdResult, "Could not resume the device!\n");
    }

    MICROSTRAIN_LOG_INFO("The device is configured... waiting for the filter to enter full navigation mode.\n");

    mip::data_gnss::FixInfo::FixType currentFixType[2] = { gnssFixInfo[0].fix_type, gnssFixInfo[1].fix_type };
    mip::data_filter::FilterMode     currentState      = filterStatus.filter_state;

    // Wait for the device to initialize
    while (filterStatus.filter_state < mip::data_filter::FilterMode::FULL_NAV)
    {
        // Update the device state
        // Note: This will update the device callbacks to trigger the filter state change
        // Note: The recommended default wait time is 10 ms, but could be 0 for non-blocking read operations
        device.update(
            10 // Time to wait
        );

        // Check for fix type state changes for each antenna
        for (uint8_t gnssIndex = 0; gnssIndex < 2; ++gnssIndex)
        {
            // Fix type state change
            if (currentFixType[gnssIndex] != gnssFixInfo[gnssIndex].fix_type)
            {
                displayGnssFixState(gnssFixInfo, gnssIndex);
                currentFixType[gnssIndex] = gnssFixInfo[gnssIndex].fix_type;
            }
        }

        // Filter state change
        if (currentState != filterStatus.filter_state)
        {
            displayFilterState(filterStatus.filter_state);
            currentState = filterStatus.filter_state;
        }
    }

    // Get the start time of the device update loop to handle exiting the application
    const mip::Timestamp loopStartTime = getCurrentTimestamp();

    mip::Timestamp previousPrintTimestamp = 0;

    // Running loop
    // Exit after a predetermined time in seconds
    while (getCurrentTimestamp() - loopStartTime <= RUN_TIME_SECONDS * 1000)
    {
        // Update the device state
        // Note: This will update the device callbacks to trigger the filter state change
        // Note: The recommended default wait time is 10 ms, but could be 0 for non-blocking read operations
        device.update(
            10 // Time to wait
        );

        // Check for fix type state changes for each antenna
        for (uint8_t gnssIndex = 0; gnssIndex < 2; ++gnssIndex)
        {
            // Fix type state change
            if (currentFixType[gnssIndex] != gnssFixInfo[gnssIndex].fix_type)
            {
                displayGnssFixState(gnssFixInfo, gnssIndex);
                currentFixType[gnssIndex] = gnssFixInfo[gnssIndex].fix_type;
            }
        }

        // Filter state change
        if (currentState != filterStatus.filter_state)
        {
            displayFilterState(filterStatus.filter_state);
            currentState = filterStatus.filter_state;
        }

        const mip::Timestamp currentTimestamp = getCurrentTimestamp();

        // Print out data based on the sample rate (1000 ms / SAMPLE_RATE_HZ)
        if (currentTimestamp - previousPrintTimestamp >= 1000 / SAMPLE_RATE_HZ)
        {
            if (filterStatus.filter_state >= mip::data_filter::FilterMode::FULL_NAV)
            {
                MICROSTRAIN_LOG_INFO(
                    "%s = %10.3f%16s = [%9.6f, %9.6f, %9.6f]%16s = [%9.6f, %9.6f, %9.6f]%16s = [%9.6f, %9.6f, %9.6f]\n",
                    "TOW",
                    filterGpsTimestamp.tow,
                    mip::data_filter::PositionLlh::DOC_NAME, // Built-in metadata for easy printing
                    filterPositionLlh.latitude,
                    filterPositionLlh.longitude,
                    filterPositionLlh.ellipsoid_height,
                    mip::data_filter::VelocityNed::DOC_NAME, // Built-in metadata for easy printing
                    filterVelocityNed.north,
                    filterVelocityNed.east,
                    filterVelocityNed.down,
                    mip::data_filter::EulerAngles::DOC_NAME, // Built-in metadata for easy printing
                    filterEulerAngles.roll,
                    filterEulerAngles.pitch,
                    filterEulerAngles.yaw
                );
            }

            previousPrintTimestamp = currentTimestamp;
        }
    }

    terminate(&connection, "Example Completed Successfully.\n", true);

    return 0;
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Custom logging callback for MIP SDK message formatting and output
///
/// @details Processes and formats log messages from the MIP SDK based on
///          severity level. Routes messages to appropriate output streams -
///          errors and fatal messages go to stderr while other levels go to
///          stdout. Each message is prefixed with its severity level name.
///
/// @param _user Pointer to user data (unused in this implementation)
/// @param _level Log message severity level from microstrain_log_level enum
/// @param _format Printf-style format string for the message
/// @param _args Variable argument list containing message parameters
///
/// @ingroup _7_series_gnss_ins_example_cpp
///
static void logCallback(void* _user, const microstrain_log_level _level, const char* _format, va_list _args)
{
    // Unused parameter
    (void)_user;

    switch (_level)
    {
        case MICROSTRAIN_LOG_LEVEL_FATAL:
        case MICROSTRAIN_LOG_LEVEL_ERROR:
        {
            fprintf(stderr, "%s: ", microstrain_logging_level_name(_level));
            vfprintf(stderr, _format, _args);
            fflush(stderr);
            break;
        }
        case MICROSTRAIN_LOG_LEVEL_WARN:
        case MICROSTRAIN_LOG_LEVEL_INFO:
        case MICROSTRAIN_LOG_LEVEL_DEBUG:
        case MICROSTRAIN_LOG_LEVEL_TRACE:
        {
            fprintf(stdout, "%s: ", microstrain_logging_level_name(_level));
            vfprintf(stdout, _format, _args);
            fflush(stdout);
            break;
        }
        case MICROSTRAIN_LOG_LEVEL_OFF:
        default:
        {
            break;
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Captures and configures device gyro bias
///
/// @param _device Reference to the initialized MIP device interface
///
/// @ingroup _7_series_gnss_ins_example_cpp
///
static void captureGyroBias(mip::Interface& _device)
{
    // Get the command queue so we can increase the reply timeout during the capture duration,
    // then reset it afterward
    mip::CmdQueue&     cmdQueue        = _device.cmdQueue();
    const mip::Timeout previousTimeout = cmdQueue.baseReplyTimeout();
    MICROSTRAIN_LOG_INFO("Initial command reply timeout is %dms.\n", previousTimeout);

    // Note: The default is 15 s (15,000 ms)
    constexpr uint16_t captureDuration          = 15000;
    constexpr uint16_t increasedCmdReplyTimeout = captureDuration + 1000;

    MICROSTRAIN_LOG_INFO("Increasing command reply timeout to %dms for capture gyro bias.\n", increasedCmdReplyTimeout);
    cmdQueue.setBaseReplyTimeout(increasedCmdReplyTimeout);

    mip::Vector3f gyroBias = {
        0.0f, // X
        0.0f, // Y
        0.0f  // Z
    };

    // Note: When capturing gyro bias, the device needs to remain still on a flat surface
    MICROSTRAIN_LOG_WARN("About to capture gyro bias for %.2g seconds!\n", static_cast<float>(captureDuration) / 1000.0f);
    MICROSTRAIN_LOG_WARN("Please do not move the device during this time!\n");
    MICROSTRAIN_LOG_WARN("Press 'Enter' when ready...");

    // Wait for anything to be entered
    const int confirmCapture = getc(stdin);
    (void)confirmCapture; // Unused

    MICROSTRAIN_LOG_WARN("Capturing gyro bias...\n");
    const mip::CmdResult cmdResult = mip::commands_3dm::captureGyroBias(
        _device,
        captureDuration, // Capture duration (ms)
        gyroBias         // Gyro bias out (result of the capture)
    );

    if (!cmdResult.isAck())
    {
        terminate(_device, cmdResult, "Failed to capture gyro bias!\n");
    }

    MICROSTRAIN_LOG_INFO("Capture gyro bias completed with result: [%f, %f, %f]\n", gyroBias[0], gyroBias[1], gyroBias[2]);

    MICROSTRAIN_LOG_INFO("Reverting command reply timeout to %dms.\n", previousTimeout);
    cmdQueue.setBaseReplyTimeout(previousTimeout);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Configures message format for GNSS data streaming
///
/// @details Sets up GNSS data output by:
///          1. Querying device base rate
///          2. Validating desired sample rate against base rate
///          3. Calculating proper decimation
///          4. Configuring message format with:
///             - Fix information
///
/// @param _device Reference to the initialized MIP device interface
/// @param _gnssDataDescriptorSet Data descriptor set for the GNSS ID to
///                               configure
///
/// @ingroup _7_series_gnss_ins_example_cpp
///
static void configureGnssMessageFormat(mip::Interface& _device, const uint8_t _gnssDataDescriptorSet)
{
    // Make sure the multi-antenna GNSS data descriptors are being used
    // Note: mip::data_gnss::DESCRIPTOR_SET is a legacy descriptor set for 5-series devices
    if (!mip::isGnssDataDescriptorSet(_gnssDataDescriptorSet) || _gnssDataDescriptorSet == mip::data_gnss::DESCRIPTOR_SET)
    {
        terminate(
            _device,
            mip::CmdResult::NACK_INVALID_PARAM,
            "The mip::data_gnss::MIP_GNSS*_DATA_DESC_SET values should be used for 7-series antenna configuration!\n"
        );
    }

    // Multi-antenna GNSS IDs correspond to 0x91 as GNSS 1, 0x92 as GNSS 2, etc.
    const uint8_t gnssId = 0x90 - _gnssDataDescriptorSet;

    // Note: Querying the device base rate is only one way to calculate the descriptor decimation
    // We could have also set it directly with information from the datasheet

    MICROSTRAIN_LOG_INFO("Getting the base rate for GNSS %d data.\n", gnssId);
    uint16_t       gnssBaseRate;
    mip::CmdResult cmdResult = mip::commands_3dm::getBaseRate(
        _device,
        _gnssDataDescriptorSet, // Data descriptor set
        &gnssBaseRate           // Base rate out
    );

    if (!cmdResult.isAck())
    {
        terminate(_device, cmdResult, "Could not get the base rate for GNSS %d data!\n", gnssId);
    }

    // Supported sample rates can be any value from 1 up to the base rate
    // Note: Decimation can be anything from 1 to 65,565 (uint16_t::max)
    if (SAMPLE_RATE_HZ == 0 || SAMPLE_RATE_HZ > gnssBaseRate)
    {
        terminate(
            _device,
            mip::CmdResult::NACK_INVALID_PARAM,
            "Invalid sample rate of %dHz! Supported rates are [1, %d].\n",
            SAMPLE_RATE_HZ,
            gnssBaseRate
        );
    }

    // Calculate the decimation (stream rate) for the device based on its base rate
    const uint16_t gnssDecimation = gnssBaseRate / SAMPLE_RATE_HZ;
    MICROSTRAIN_LOG_INFO(
        "Decimating GNSS %d base rate %d by %d to stream data at %dHz.\n",
        gnssId,
        gnssBaseRate,
        gnssDecimation,
        SAMPLE_RATE_HZ
    );

    // Descriptor rate is a pair of data descriptor set and decimation
    const mip::DescriptorRate gnssDescriptors[1] = {
        { mip::data_gnss::FixInfo::FIELD_DESCRIPTOR, gnssDecimation }
    };

    MICROSTRAIN_LOG_INFO("Configuring %s for GNSS %d data.\n", mip::commands_3dm::MessageFormat::DOC_NAME, gnssId);
    cmdResult = mip::commands_3dm::writeMessageFormat(
        _device,
        mip::data_filter::DESCRIPTOR_SET,                     // Data descriptor set
        sizeof(gnssDescriptors) / sizeof(gnssDescriptors[0]), // Number of descriptors to include
        gnssDescriptors                                       // Descriptor array
    );

    if (!cmdResult.isAck())
    {
        terminate(
            _device,
            cmdResult,
            "Could not configure %s for GNSS %d data!\n",
            mip::commands_3dm::MessageFormat::DOC_NAME,
            gnssId
        );
    }
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Configures message format for filter data streaming
///
/// @details Sets up filter data output by:
///          1. Querying device base rate
///          2. Validating desired sample rate against base rate
///          3. Calculating proper decimation
///          4. Configuring message format with:
///             - GPS timestamp
///             - Filter status
///             - LLH position
///             - NED velocity
///             - Euler angles
///
/// @param _device Reference to the initialized MIP device interface
///
/// @ingroup _7_series_gnss_ins_example_cpp
///
static void configureFilterMessageFormat(mip::Interface& _device)
{
    // Note: Querying the device base rate is only one way to calculate the descriptor decimation
    // We could have also set it directly with information from the datasheet

    MICROSTRAIN_LOG_INFO("Getting the base rate for filter data.\n");
    uint16_t       filterBaseRate;
    mip::CmdResult cmdResult = mip::commands_3dm::getBaseRate(
        _device,
        mip::data_filter::DESCRIPTOR_SET, // Data descriptor set
        &filterBaseRate                   // Base rate out
    );

    if (!cmdResult.isAck())
    {
        terminate(_device, cmdResult, "Could not get the base rate for filter data!\n");
    }

    // Supported sample rates can be any value from 1 up to the base rate
    // Note: Decimation can be anything from 1 to 65,565 (uint16_t::max)
    if (SAMPLE_RATE_HZ == 0 || SAMPLE_RATE_HZ > filterBaseRate)
    {
        terminate(
            _device,
            mip::CmdResult::NACK_INVALID_PARAM,
            "Invalid sample rate of %dHz! Supported rates are [1, %d].\n",
            SAMPLE_RATE_HZ,
            filterBaseRate
        );
    }

    // Calculate the decimation (stream rate) for the device based on its base rate
    const uint16_t filterDecimation = filterBaseRate / SAMPLE_RATE_HZ;
    MICROSTRAIN_LOG_INFO(
        "Decimating filter base rate %d by %d to stream data at %dHz.\n",
        filterBaseRate,
        filterDecimation,
        SAMPLE_RATE_HZ
    );

    // Descriptor rate is a pair of data descriptor set and decimation
    const mip::DescriptorRate filterDescriptors[5] = {
        { mip::data_shared::GpsTimestamp::FIELD_DESCRIPTOR, filterDecimation },
        { mip::data_filter::Status::FIELD_DESCRIPTOR,       filterDecimation },
        { mip::data_filter::PositionLlh::FIELD_DESCRIPTOR,  filterDecimation },
        { mip::data_filter::VelocityNed::FIELD_DESCRIPTOR,  filterDecimation },
        { mip::data_filter::EulerAngles::FIELD_DESCRIPTOR,  filterDecimation }
    };

    MICROSTRAIN_LOG_INFO("Configuring %s for filter data.\n", mip::commands_3dm::MessageFormat::DOC_NAME);
    cmdResult = mip::commands_3dm::writeMessageFormat(
        _device,
        mip::data_filter::DESCRIPTOR_SET,                         // Data descriptor set
        sizeof(filterDescriptors) / sizeof(filterDescriptors[0]), // Number of descriptors to include
        filterDescriptors                                         // Descriptor array
    );

    if (!cmdResult.isAck())
    {
        terminate(
            _device,
            cmdResult,
            "Could not configure %s for filter data!\n",
            mip::commands_3dm::MessageFormat::DOC_NAME
        );
    }
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Configures the GNSS antenna offset parameters for the device
///
/// @details Sets up the physical antenna offset values relative to the device's
///          reference point. The offset is specified in meters using a 3D
///          vector:
///          - X: forward/back
///          - Y: left/right
///          - Z: up/down
///
/// @param _device Reference to the initialized MIP device interface
/// @param _antennaOffset Antenna offset to set
/// @param _antennaId Antenna ID to configure the offset for
///
/// @note Offset values are specific to physical device setup and may need to be
///       adjusted based on actual antenna placement
///
/// @ingroup _7_series_gnss_ins_example_cpp
///
static void configureAntennaOffset(mip::Interface& _device, const mip::Vector3f _antennaOffset, const uint8_t _antennaId)
{
    MICROSTRAIN_LOG_INFO(
        "Configuring the GNSS %d antenna offset for [%gm, %gm, %gm].\n",
        _antennaId,
        _antennaOffset[0],
        _antennaOffset[1],
        _antennaOffset[2]
    );
    const mip::CmdResult cmdResult = mip::commands_filter::writeMultiAntennaOffset(
        _device,
        _antennaId, // Receiver/Antenna ID
        _antennaOffset
    );

    if (!cmdResult.isAck())
    {
        terminate(_device, cmdResult, "Could not configure the GNSS %d antenna offset!\n", _antennaId);
    }
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Initializes and resets the navigation filter
///
/// @details Configures the navigation filter by:
///          1. Enabling GNSS position and velocity aiding measurements
///          2. Enabling dual-antenna GNSS heading aiding measurements
///          3. Configuring filter initialization settings:
///             - Setting initial position and velocity to zero
///             - Enabling automatic position/velocity/attitude determination
///             - Configuring dual-antenna kinematic alignment
///          4. Resetting the filter to apply new settings
///
/// @param _device Reference to the initialized MIP device interface
///
/// @ingroup _7_series_gnss_ins_example_cpp
///
static void initializeFilter(mip::Interface& _device)
{
    // Configure Filter Aiding Measurements
    MICROSTRAIN_LOG_INFO(
        "Enabling the %s for GNSS position and velocity.\n",
        mip::commands_filter::AidingMeasurementEnable::DOC_NAME
    );
    mip::CmdResult cmdResult = mip::commands_filter::writeAidingMeasurementEnable(
        _device,
        mip::commands_filter::AidingMeasurementEnable::AidingSource::GNSS_POS_VEL, // Aiding Source type
        true                                                                       // Enabled
    );

    if (!cmdResult.isAck())
    {
        terminate(
            _device,
            cmdResult,
            "Could not enable the %s for GNSS position and velocity!\n",
            mip::commands_filter::AidingMeasurementEnable::DOC_NAME
        );
    }

    MICROSTRAIN_LOG_INFO("Enabling the %s for GNSS heading.\n", mip::commands_filter::AidingMeasurementEnable::DOC_NAME);
    cmdResult = mip::commands_filter::writeAidingMeasurementEnable(
        _device,
        mip::commands_filter::AidingMeasurementEnable::AidingSource::GNSS_HEADING, // Aiding Source type
        true                                                                       // Enabled
    );

    if (!cmdResult.isAck())
    {
        terminate(
            _device,
            cmdResult,
            "Could not enable the %s for GNSS heading!\n",
            mip::commands_filter::AidingMeasurementEnable::DOC_NAME
        );
    }

    // Configure the filter initialization

    const mip::Vector3f initialPosition = {
        0.0f, // X
        0.0f, // Y
        0.0f  // Z
    };

    const mip::Vector3f initialVelocity = {
        0.0f, // X
        0.0f, // Y
        0.0f  // Z
    };

    // Note: This is the default setting on the device and will automatically configure
    // the initial conditions required to initialize the filter based on the alignment selector
    // and readings from the device's sensors
    constexpr mip::commands_filter::InitializationConfiguration::InitialConditionSource initialConditionSource =
        mip::commands_filter::InitializationConfiguration::InitialConditionSource::AUTO_POS_VEL_ATT;

    constexpr mip::commands_filter::InitializationConfiguration::AlignmentSelector initialAlignmentSelector =
        mip::commands_filter::InitializationConfiguration::AlignmentSelector::KINEMATIC |
        mip::commands_filter::InitializationConfiguration::AlignmentSelector::DUAL_ANTENNA;

    MICROSTRAIN_LOG_INFO("Setting the %s configuration.\n", mip::commands_filter::InitializationConfiguration::DOC_NAME);
    cmdResult = mip::commands_filter::writeInitializationConfiguration(
        _device,
        0, // Initialize the filter after receiving the filter run command (disabled)
        initialConditionSource,
        initialAlignmentSelector, // Bitfield value
        0.0f,                     // Initial heading
        0.0f,                     // Initial pitch
        0.0f,                     // Initial roll
        initialPosition,
        initialVelocity,
        mip::commands_filter::FilterReferenceFrame::LLH // Reference frame selector
    );

    if (!cmdResult.isAck())
    {
        terminate(
            _device,
            cmdResult,
            "Could not set the %s configuration!\n",
            mip::commands_filter::InitializationConfiguration::DOC_NAME
        );
    }

    // Reset the filter
    // Note: This is good to do after filter setup is complete
    MICROSTRAIN_LOG_INFO("Attempting to %s.\n", mip::commands_filter::Reset::DOC_NAME);
    cmdResult = mip::commands_filter::reset(_device);

    if (!cmdResult.isAck())
    {
        terminate(_device, cmdResult, "Could not %s!\n", mip::commands_filter::Reset::DOC_NAME);
    }
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Displays the current GNSS fix state for a specific antenna when
///        changes occur
///
/// @details Outputs readable messages for GNSS fix state transitions:
///          - 3D fix
///          - 2D fix
///          - Time-only fix
///          - No fix
///          - Invalid fix
///          - RTK float
///          - RTK fixed
///          - Differential fix
///          Also indicates whether the current fix is valid based on flag
///          checks.
///
/// @param _fixInfoArray Pointer to an array of GNSS fix information structures
///                      containing fix types and validity flags
/// @param _arrayIndex Zero-based index into the fix info array identifying
///                    which GNSS receiver to report (0 = primary antenna,
///                    1 = secondary antenna)
///
/// @ingroup _7_series_gnss_ins_example_cpp
///
static void displayGnssFixState(const mip::data_gnss::FixInfo* _fixInfoArray, const uint8_t _arrayIndex)
{
    const uint8_t antennaId      = _arrayIndex + 1;
    const uint8_t fixTypeValue   = static_cast<uint8_t>(_fixInfoArray[_arrayIndex].fix_type);
    const char*   fixDescription = "";
    const char*   fixType        = "";

    switch (_fixInfoArray[_arrayIndex].fix_type)
    {
        case mip::data_gnss::FixInfo::FixType::FIX_3D:
        {
            fixDescription = "a 3D";
            fixType        = "FIX_3D";
            break;
        }
        case mip::data_gnss::FixInfo::FixType::FIX_2D:
        {
            fixDescription = "a 2D";
            fixType        = "FIX_2D";
            break;
        }
        case mip::data_gnss::FixInfo::FixType::FIX_TIME_ONLY:
        {
            fixDescription = "a time only";
            fixType        = "FIX_TIME_ONLY";
            break;
        }
        case mip::data_gnss::FixInfo::FixType::FIX_NONE:
        {
            MICROSTRAIN_LOG_INFO("GNSS %d has no fix. (%d) FIX_NONE\n", antennaId, fixTypeValue);

            // No fix, exit early
            return;
        }
        case mip::data_gnss::FixInfo::FixType::FIX_INVALID:
        {
            MICROSTRAIN_LOG_INFO("GNSS %d acquired an invalid fix. (%d) FIX_INVALID\n", antennaId, fixTypeValue);

            // The fix is already invalid, exit early
            return;
        }
        case mip::data_gnss::FixInfo::FixType::FIX_RTK_FLOAT:
        {
            fixDescription = "an RTK float";
            fixType        = "FIX_RTK_FLOAT";
            break;
        }
        case mip::data_gnss::FixInfo::FixType::FIX_RTK_FIXED:
        {
            fixDescription = "an RTK fixed";
            fixType        = "FIX_RTK_FIXED";
            break;
        }
        case mip::data_gnss::FixInfo::FixType::FIX_DIFFERENTIAL:
        {
            fixDescription = "a differential";
            fixType        = "FIX_DIFFERENTIAL";
            break;
        }
        default:
        {
            // Any other value is invalid
            return;
        }
    }

    MICROSTRAIN_LOG_INFO("GNSS %d acquired %s fix. (%d) %s\n", antennaId, fixDescription, fixTypeValue, fixType);

    const char* validEntryDisplay = _fixInfoArray[_arrayIndex].valid_flags.fixType() ? "valid" : "invalid";

    // Confirm a valid fix was acquired
    MICROSTRAIN_LOG_INFO("The current GNSS %d fix is %s.\n", antennaId, validEntryDisplay);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Displays the current filter state when changes occur
///
/// @details Outputs readable messages for filter state transitions:
///          - Initialization mode
///          - Vertical gyro mode
///          - AHRS mode
///          - Full navigation mode
///
/// @param _filterState Current filter mode from the MIP device interface
///
/// @ingroup _7_series_gnss_ins_example_cpp
///
static void displayFilterState(const mip::data_filter::FilterMode _filterState)
{
    const char* modeDescription = "startup";
    const char* modeType        = "STARTUP";

    switch (_filterState)
    {
        case mip::data_filter::FilterMode::INIT:
        {
            modeDescription = "initialization";
            modeType        = "INIT";
            break;
        }
        case mip::data_filter::FilterMode::VERT_GYRO:
        {
            modeDescription = "vertical gyro";
            modeType        = "VERT_GYRO";
            break;
        }
        case mip::data_filter::FilterMode::AHRS:
        {
            modeDescription = "AHRS";
            modeType        = "AHRS";
            break;
        }
        case mip::data_filter::FilterMode::FULL_NAV:
        {
            modeDescription = "full navigation";
            modeType        = "FULL_NAV";
            break;
        }
        default:
        {
            break;
        }
    }

    MICROSTRAIN_LOG_INFO(
        "The filter has entered %s mode. (%d) %s\n",
        modeDescription,
        static_cast<uint8_t>(_filterState),
        modeType
    );
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Gets the current system timestamp in milliseconds
///
/// @details Provides system time measurement using std::chrono for milliseconds
///          since epoch. Uses system_clock to get wall-clock time that
///          corresponds to calendar time and can be synchronized with external
///          time sources.
///
/// @note Update this function to use a different time source if needed for
///       your specific application requirements
///
/// @return Current timestamp in milliseconds since epoch
///
/// @ingroup _7_series_gnss_ins_example_cpp
///
static mip::Timestamp getCurrentTimestamp()
{
    const std::chrono::nanoseconds timeSinceEpoch = std::chrono::system_clock::now().time_since_epoch();
    return static_cast<mip::Timestamp>(std::chrono::duration_cast<std::chrono::milliseconds>(timeSinceEpoch).count());
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Initializes and configures a MIP device interface
///
/// @details Performs a complete device initialization sequence:
///          1. Verifies device communication with a ping command
///          2. Sets the device to idle mode to ensure reliable configuration
///          3. Queries and displays detailed device information
///          4. Loads default device settings for a known state
///
/// @param _device Reference to a MIP device interface to initialize
///
/// @ingroup _7_series_gnss_ins_example_cpp
///
static void initializeDevice(mip::Interface& _device)
{
    // Ping the device
    // Note: This is a good first step to make sure the device is present
    MICROSTRAIN_LOG_INFO("Pinging the device.\n");
    mip::CmdResult cmdResult = mip::commands_base::ping(_device);

    if (!cmdResult.isAck())
    {
        terminate(_device, cmdResult, "Could not ping the device!\n");
    }

    // Set the device to Idle
    // Note: This is good to do during setup as high data traffic can cause commands to fail
    MICROSTRAIN_LOG_INFO("Setting the device to idle.\n");
    cmdResult = mip::commands_base::setIdle(_device);

    if (!cmdResult.isAck())
    {
        terminate(_device, cmdResult, "Could not set the device to idle!\n");
    }

    // Print device info to make sure the correct device is being used
    MICROSTRAIN_LOG_INFO("Getting the device information.\n");
    mip::commands_base::BaseDeviceInfo deviceInfo;
    cmdResult = mip::commands_base::getDeviceInfo(_device, &deviceInfo);

    if (!cmdResult.isAck())
    {
        terminate(_device, cmdResult, "Could not get the device information!\n");
    }

    // Extract the major minor and patch values
    const uint16_t major = deviceInfo.firmware_version / 1000;
    const uint16_t minor = deviceInfo.firmware_version / 100 % 10;
    const uint16_t patch = deviceInfo.firmware_version % 100;

    // Firmware version format is x.x.xx
    char firmwareVersion[16];
    snprintf(firmwareVersion, sizeof(firmwareVersion) / sizeof(firmwareVersion[0]), "%d.%d.%02d", major, minor, patch);

    MICROSTRAIN_LOG_INFO("-------- Device Information --------\n");
    MICROSTRAIN_LOG_INFO("%-16s | %.16s\n", "Name", deviceInfo.model_name);
    MICROSTRAIN_LOG_INFO("%-16s | %.16s\n", "Model Number", deviceInfo.model_number);
    MICROSTRAIN_LOG_INFO("%-16s | %.16s\n", "Serial Number", deviceInfo.serial_number);
    MICROSTRAIN_LOG_INFO("%-16s | %.16s\n", "Lot Number", deviceInfo.lot_number);
    MICROSTRAIN_LOG_INFO("%-16s | %.16s\n", "Options", deviceInfo.device_options);
    MICROSTRAIN_LOG_INFO("%-16s | %16s\n", "Firmware Version", firmwareVersion);
    MICROSTRAIN_LOG_INFO("------------------------------------\n");

    // Load the default settings on the device
    // Note: This guarantees the device is in a known state
    MICROSTRAIN_LOG_INFO("Loading %s.\n", mip::commands_3dm::DeviceSettings::DOC_NAME);
    cmdResult = mip::commands_3dm::defaultDeviceSettings(_device);

    if (!cmdResult.isAck())
    {
        // Note: Default settings will reset the baudrate to 115200 and may cause connection issues
        if (cmdResult == mip::CmdResult::STATUS_TIMEDOUT && BAUDRATE != 115200)
        {
            MICROSTRAIN_LOG_WARN("On a native serial connections the baudrate needs to be 115200 for this example to run.\n");
        }

        terminate(_device, cmdResult, "Could not load %s!\n", mip::commands_3dm::DeviceSettings::DOC_NAME);
    }
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Handles graceful program termination and cleanup
///
/// @details Handles graceful shutdown when errors occur:
///          - Outputs provided error message
///          - Closes device connection if open
///          - Exits with appropriate status code
///
/// @param _connection Pointer to the device connection to close
/// @param _message Error message to display
/// @param _successful Whether termination is due to success or failure
///
/// @ingroup _7_series_gnss_ins_example_cpp
///
static void terminate(microstrain::Connection* _connection, const char* _message, const bool _successful /* = false */)
{
    if (_message && strlen(_message) != 0)
    {
        if (_successful)
        {
            MICROSTRAIN_LOG_INFO("%s", _message);
        }
        else
        {
            MICROSTRAIN_LOG_ERROR("%s", _message);
        }
    }

    if (!_connection)
    {
        // Create the device interface with a connection or set it after creation
        MICROSTRAIN_LOG_ERROR("Connection not set for the device interface. Cannot close the connection.\n");
    }
    else
    {
        if (_connection->isConnected())
        {
            MICROSTRAIN_LOG_INFO("Closing the connection.\n");

            if (!_connection->disconnect())
            {
                MICROSTRAIN_LOG_ERROR("Failed to close the connection!\n");
            }
        }
    }

    MICROSTRAIN_LOG_INFO("Press 'Enter' to exit the program.\n");

    // Make sure the console remains open
    const int confirmExit = getc(stdin);
    (void)confirmExit; // Unused

    if (!_successful)
    {
        exit(1);
    }
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Handles graceful program termination and command failure cleanup
///
/// @details Handles command failure scenarios:
///          - Formats and displays an error message with command result
///          - Closes device connection
///          - Exits with failure status
///
/// @param _device MIP device interface for the command that failed
/// @param _cmdResult Result code from a failed command
/// @param _format Printf-style format string for error message
/// @param ... Variable arguments for format string
///
/// @ingroup _7_series_gnss_ins_example_cpp
///
static void terminate(mip::Interface& _device, const mip::CmdResult _cmdResult, const char* _format, ...)
{
    if (_format && strlen(_format) != 0)
    {
        va_list args;
        va_start(args, _format);
        MICROSTRAIN_LOG_ERROR_V(_format, args);
        va_end(args);
    }

    MICROSTRAIN_LOG_ERROR("Command Result: (%d) %s.\n", _cmdResult.value, _cmdResult.name());

    // Get the connection pointer that was set during device initialization
    microstrain::Connection* connection = static_cast<microstrain::Connection*>(_device.userPointer());

    terminate(connection, "");
}
