#pragma once

#include "descriptors.h"
#include "../mip_result.h"

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
namespace mscl {
#endif // __cplusplus

struct mip_interface;

////////////////////////////////////////////////////////////////////////////////
///@addtogroup MipCommands
///@{
///@defgroup BASECommands  BASE
///
///@{

////////////////////////////////////////////////////////////////////////////////
// Descriptors
////////////////////////////////////////////////////////////////////////////////

enum mip_base_commands_descriptors
{
    MIP_BASE_CMD_DESC_SET                        = 0x01,
    
    MIP_CMD_DESC_BASE_PING                       = 0x01,
    MIP_CMD_DESC_BASE_SET_TO_IDLE                = 0x02,
    MIP_CMD_DESC_BASE_GET_DEVICE_INFO            = 0x03,
    MIP_CMD_DESC_BASE_GET_DEVICE_DESCRIPTORS     = 0x04,
    MIP_CMD_DESC_BASE_BUILT_IN_TEST              = 0x05,
    MIP_CMD_DESC_BASE_RESUME                     = 0x06,
    MIP_CMD_DESC_BASE_GET_EXTENDED_DESCRIPTORS   = 0x07,
    MIP_CMD_DESC_BASE_CONTINUOUS_BIT             = 0x08,
    MIP_CMD_DESC_BASE_COMM_SPEED                 = 0x09,
    MIP_CMD_DESC_BASE_GPS_TIME_BROADCAST_NEW     = 0x72,
    MIP_CMD_DESC_BASE_SOFT_RESET                 = 0x7E,
    
    MIP_REPLY_DESC_BASE_DEVICE_INFO              = 0x81,
    MIP_REPLY_DESC_BASE_DEVICE_DESCRIPTORS       = 0x82,
    MIP_REPLY_DESC_BASE_BUILT_IN_TEST            = 0x83,
    MIP_REPLY_DESC_BASE_GET_EXTENDED_DESCRIPTORS = 0x86,
    MIP_REPLY_DESC_BASE_CONTINUOUS_BIT           = 0x88,
    MIP_REPLY_DESC_BASE_COMM_SPEED               = 0x89,
};
#ifdef __cplusplus
namespace C {
extern "C" {
#endif // __cplusplus


////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////

struct mip_base_device_info
{
    uint16_t                                          firmware_version;
    char                                              model_name[16];
    char                                              model_number[16];
    char                                              serial_number[16];
    char                                              lot_number[16];
    char                                              device_options[16];
};
size_t insert_mip_base_device_info(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_base_device_info* self);
size_t extract_mip_base_device_info(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_base_device_info* self);

enum mip_time_format
{
    MIP_TIME_FORMAT_GPS = 1,  ///<  GPS time, a = week number since 1980, b = time of week in milliseconds.
};
size_t insert_mip_time_format(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_time_format self);
size_t extract_mip_time_format(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_time_format* self);

enum mip_commanded_test_bits_gq7
{
    MIP_COMMANDED_TEST_BITS_GQ7_GENERAL_HARDWARE_FAULT = 0x01,
    MIP_COMMANDED_TEST_BITS_GQ7_GENERAL_FIRMWARE_FAULT = 0x02,
    MIP_COMMANDED_TEST_BITS_GQ7_TIMING_OVERLOAD        = 0x04,
    MIP_COMMANDED_TEST_BITS_GQ7_BUFFER_OVERRUN         = 0x08,
    MIP_COMMANDED_TEST_BITS_GQ7_RESERVED               = 0xF0,
    MIP_COMMANDED_TEST_BITS_GQ7_IPC_IMU_FAULT          = 0x100,
    MIP_COMMANDED_TEST_BITS_GQ7_IPC_NAV_FAULT          = 0x200,
    MIP_COMMANDED_TEST_BITS_GQ7_IPC_GNSS_FAULT         = 0x400,
    MIP_COMMANDED_TEST_BITS_GQ7_COMMS_FAULT            = 0x800,
    MIP_COMMANDED_TEST_BITS_GQ7_IMU_ACCEL_FAULT        = 0x1000,
    MIP_COMMANDED_TEST_BITS_GQ7_IMU_GYRO_FAULT         = 0x2000,
    MIP_COMMANDED_TEST_BITS_GQ7_IMU_MAG_FAULT          = 0x4000,
    MIP_COMMANDED_TEST_BITS_GQ7_IMU_PRESS_FAULT        = 0x8000,
    MIP_COMMANDED_TEST_BITS_GQ7_IMU_RESERVED           = 0x30000,
    MIP_COMMANDED_TEST_BITS_GQ7_IMU_CAL_ERROR          = 0x40000,
    MIP_COMMANDED_TEST_BITS_GQ7_IMU_GENERAL_FAULT      = 0x80000,
    MIP_COMMANDED_TEST_BITS_GQ7_FILT_RESERVED          = 0x300000,
    MIP_COMMANDED_TEST_BITS_GQ7_FILT_SOLUTION_FAULT    = 0x400000,
    MIP_COMMANDED_TEST_BITS_GQ7_FILT_GENERAL_FAULT     = 0x800000,
    MIP_COMMANDED_TEST_BITS_GQ7_GNSS_RECEIVER1_FAULT   = 0x1000000,
    MIP_COMMANDED_TEST_BITS_GQ7_GNSS_ANTENNA1_FAULT    = 0x2000000,
    MIP_COMMANDED_TEST_BITS_GQ7_GNSS_RECEIVER2_FAULT   = 0x4000000,
    MIP_COMMANDED_TEST_BITS_GQ7_GNSS_ANTENNA2_FAULT    = 0x8000000,
    MIP_COMMANDED_TEST_BITS_GQ7_GNSS_RTCM_FAILURE      = 0x10000000,
    MIP_COMMANDED_TEST_BITS_GQ7_GNSS_RTK_FAULT         = 0x20000000,
    MIP_COMMANDED_TEST_BITS_GQ7_GNSS_SOLUTION_FAULT    = 0x40000000,
    MIP_COMMANDED_TEST_BITS_GQ7_GNSS_GENERAL_FAULT     = 0x80000000,
};
size_t insert_mip_commanded_test_bits_gq7(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_commanded_test_bits_gq7 self);
size_t extract_mip_commanded_test_bits_gq7(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_commanded_test_bits_gq7* self);


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
///@defgroup ping  Ping
/// Test Communications with a device.
/// 
/// The Device will respond with an ACK, if present and operating correctly.
/// 
/// If the device is not in a normal operating mode, it may NACK.
///
///@{

struct mip_base_ping_command
{
};
size_t insert_mip_base_ping_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_base_ping_command* self);
size_t extract_mip_base_ping_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_base_ping_command* self);

mip_cmd_result mip_base_ping(struct mip_interface* device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup set_idle  Set to idle
/// Turn off all device data streams.
/// 
/// The Device will respond with an ACK, if present and operating correctly.
/// This command will suspend streaming (if enabled) or wake the device from sleep (if sleeping) to allow it to respond to status and setup commands.
/// You may restore the device mode by issuing the Resume command.
///
///@{

struct mip_base_set_idle_command
{
};
size_t insert_mip_base_set_idle_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_base_set_idle_command* self);
size_t extract_mip_base_set_idle_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_base_set_idle_command* self);

mip_cmd_result mip_base_set_idle(struct mip_interface* device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup get_device_info  Get device information
/// Get the device ID strings and firmware version number.
///
///@{

struct mip_base_get_device_info_command
{
};
size_t insert_mip_base_get_device_info_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_base_get_device_info_command* self);
size_t extract_mip_base_get_device_info_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_base_get_device_info_command* self);

struct mip_base_get_device_info_response
{
    struct mip_base_device_info                       device_info;
};
size_t insert_mip_base_get_device_info_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_base_get_device_info_response* self);
size_t extract_mip_base_get_device_info_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_base_get_device_info_response* self);

mip_cmd_result mip_base_get_device_info(struct mip_interface* device, struct mip_base_device_info* device_info);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup get_device_descriptors  Get device descriptors
/// Get the command and data descriptors supported by the device.
/// 
/// Reply has two fields: "ACK/NACK" and "Descriptors". The "Descriptors" field is an array of 16 bit values.
/// The MSB specifies the descriptor set and the LSB specifies the descriptor.
///
///@{

struct mip_base_get_device_descriptors_command
{
};
size_t insert_mip_base_get_device_descriptors_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_base_get_device_descriptors_command* self);
size_t extract_mip_base_get_device_descriptors_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_base_get_device_descriptors_command* self);

struct mip_base_get_device_descriptors_response
{
    uint8_t                                           descriptors_count;
    uint16_t*                                         descriptors;
};
size_t insert_mip_base_get_device_descriptors_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_base_get_device_descriptors_response* self);
size_t extract_mip_base_get_device_descriptors_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_base_get_device_descriptors_response* self);

mip_cmd_result mip_base_get_device_descriptors(struct mip_interface* device, uint8_t* descriptors_count, uint16_t* descriptors);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup built_in_test  Built in test
/// Run the device Built-In Test (BIT).
/// 
/// The Built-In Test command always returns a 32 bit value.
/// A value of 0 means that all tests passed.
/// A non-zero value indicates that not all tests passed.
/// Reference the device user manual to decode the result.
///
///@{

struct mip_base_built_in_test_command
{
};
size_t insert_mip_base_built_in_test_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_base_built_in_test_command* self);
size_t extract_mip_base_built_in_test_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_base_built_in_test_command* self);

struct mip_base_built_in_test_response
{
    uint32_t                                          result;
};
size_t insert_mip_base_built_in_test_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_base_built_in_test_response* self);
size_t extract_mip_base_built_in_test_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_base_built_in_test_response* self);

mip_cmd_result mip_base_built_in_test(struct mip_interface* device, uint32_t* result);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup resume  Resume
/// Take the device out of idle mode.
/// 
/// The device responds with ACK upon success.
///
///@{

struct mip_base_resume_command
{
};
size_t insert_mip_base_resume_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_base_resume_command* self);
size_t extract_mip_base_resume_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_base_resume_command* self);

mip_cmd_result mip_base_resume(struct mip_interface* device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup get_extended_descriptors  Get device descriptors (extended)
/// Get the command and data descriptors supported by the device.
/// 
/// Reply has two fields: "ACK/NACK" and "Descriptors". The "Descriptors" field is an array of 16 bit values.
/// The MSB specifies the descriptor set and the LSB specifies the descriptor.
///
///@{

struct mip_base_get_extended_descriptors_command
{
};
size_t insert_mip_base_get_extended_descriptors_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_base_get_extended_descriptors_command* self);
size_t extract_mip_base_get_extended_descriptors_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_base_get_extended_descriptors_command* self);

struct mip_base_get_extended_descriptors_response
{
    uint8_t                                           descriptors_count;
    uint16_t*                                         descriptors;
};
size_t insert_mip_base_get_extended_descriptors_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_base_get_extended_descriptors_response* self);
size_t extract_mip_base_get_extended_descriptors_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_base_get_extended_descriptors_response* self);

mip_cmd_result mip_base_get_extended_descriptors(struct mip_interface* device, uint8_t* descriptors_count, uint16_t* descriptors);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup continuous_bit  Continuous built-in test
/// Report result of continous built-in test.
/// 
/// This test is non-disruptive but is not as thorough as the commanded BIT.
///
///@{

struct mip_base_continuous_bit_command
{
};
size_t insert_mip_base_continuous_bit_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_base_continuous_bit_command* self);
size_t extract_mip_base_continuous_bit_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_base_continuous_bit_command* self);

struct mip_base_continuous_bit_response
{
    uint8_t                                           result[16];
};
size_t insert_mip_base_continuous_bit_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_base_continuous_bit_response* self);
size_t extract_mip_base_continuous_bit_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_base_continuous_bit_response* self);

mip_cmd_result mip_base_continuous_bit(struct mip_interface* device, uint8_t* result);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup comm_speed  Comm Port Speed
/// Controls the baud rate of a specific port on the device.
/// 
/// Please see the device user manual for supported baud rates on each port.
/// 
/// The device will wait until all incoming and outgoing data has been sent, up
/// to a maximum of 250 ms, before applying any change.
/// 
/// No guarantee is provided as to what happens to commands issued during this
/// delay period; They may or may not be processed and any responses aren't
/// guaranteed to be at one rate or the other. The same applies to data packets.
/// 
/// It is highly recommended that the device be idle before issuing this command
/// and that it be issued in its own packet. Users should wait 250 ms after
/// sending this command before further interaction.
///
///@{

struct mip_base_comm_speed_command
{
    enum mip_function_selector                        function;
    uint8_t                                           port;
    uint32_t                                          baud;
};
size_t insert_mip_base_comm_speed_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_base_comm_speed_command* self);
size_t extract_mip_base_comm_speed_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_base_comm_speed_command* self);

struct mip_base_comm_speed_response
{
    uint8_t                                           port;
    uint32_t                                          baud;
};
size_t insert_mip_base_comm_speed_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_base_comm_speed_response* self);
size_t extract_mip_base_comm_speed_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_base_comm_speed_response* self);

mip_cmd_result write_mip_base_comm_speed(struct mip_interface* device, uint8_t port, uint32_t baud);
mip_cmd_result read_mip_base_comm_speed(struct mip_interface* device, uint8_t port, uint32_t* baud);
mip_cmd_result save_mip_base_comm_speed(struct mip_interface* device, uint8_t port);
mip_cmd_result load_mip_base_comm_speed(struct mip_interface* device, uint8_t port);
mip_cmd_result default_mip_base_comm_speed(struct mip_interface* device, uint8_t port);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup gps_time_update  Time Broadcast Command
/// When combined with a PPS input signal applied to the I/O connector, this command enables complete synchronization of data outputs
/// with an external time base, such as GPS system time. Since the hardware PPS synchronization can only detect the fractional number of seconds when pulses arrive,
/// complete synchronization requires that the user provide the whole number of seconds via this command. After achieving PPS synchronization, this command should be sent twice: once to set the time-of-week and once to set the week number. PPS synchronization can be verified by monitoring the time sync status message (0xA0, 0x02) or the valid flags of any shared external timestamp (0x--, D7) data field.
///
///@{

enum mip_base_gps_time_update_command_field_id
{
    MIP_BASE_GPS_TIME_UPDATE_COMMAND_FIELD_ID_WEEK_NUMBER  = 1,  ///<  Week number.
    MIP_BASE_GPS_TIME_UPDATE_COMMAND_FIELD_ID_TIME_OF_WEEK = 2,  ///<  Time of week in seconds.
};
size_t insert_mip_base_gps_time_update_command_field_id(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_base_gps_time_update_command_field_id self);
size_t extract_mip_base_gps_time_update_command_field_id(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_base_gps_time_update_command_field_id* self);

struct mip_base_gps_time_update_command
{
    enum mip_function_selector                        function;
    enum mip_base_gps_time_update_command_field_id    field_id;
    uint32_t                                          value;
};
size_t insert_mip_base_gps_time_update_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_base_gps_time_update_command* self);
size_t extract_mip_base_gps_time_update_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_base_gps_time_update_command* self);

mip_cmd_result write_mip_base_gps_time_update(struct mip_interface* device, enum mip_base_gps_time_update_command_field_id field_id, uint32_t value);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup soft_reset  Reset device
/// Resets the device.
/// 
/// Device responds with ACK and immediately resets.
///
///@{

struct mip_base_soft_reset_command
{
};
size_t insert_mip_base_soft_reset_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_base_soft_reset_command* self);
size_t extract_mip_base_soft_reset_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_base_soft_reset_command* self);

mip_cmd_result mip_base_soft_reset(struct mip_interface* device);
///@}
///

///@}
///@}
///
////////////////////////////////////////////////////////////////////////////////

#ifdef __cplusplus
} // extern "C"
} // namespace C
namespace BaseCommands {


struct ping : C::mip_base_ping_command
{
    static const uint8_t descriptorSet = MIP_BASE_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_BASE_PING;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::insert_mip_base_ping_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_base_ping_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = false;
    
};
MipCmdResult ping(C::mip_interface& device)
{
    return C::mip_base_ping(&device);
}



struct setIdle : C::mip_base_set_idle_command
{
    static const uint8_t descriptorSet = MIP_BASE_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_BASE_SET_TO_IDLE;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::insert_mip_base_set_idle_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_base_set_idle_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = false;
    
};
MipCmdResult setIdle(C::mip_interface& device)
{
    return C::mip_base_set_idle(&device);
}



struct getDeviceInfo : C::mip_base_get_device_info_command
{
    static const uint8_t descriptorSet = MIP_BASE_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_BASE_GET_DEVICE_INFO;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::insert_mip_base_get_device_info_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_base_get_device_info_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = false;
    
    struct Response : C::mip_base_get_device_info_response
    {
        static const uint8_t descriptorSet = MIP_BASE_CMD_DESC_SET;
        static const uint8_t fieldDescriptor = MIP_REPLY_DESC_BASE_DEVICE_INFO;
        
        size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::insert_mip_base_get_device_info_response(buffer, bufferSize, offset, this);
        }
        size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::extract_mip_base_get_device_info_response(buffer, bufferSize, offset, this);
        }
    };
    
};
MipCmdResult getDeviceInfo(C::mip_interface& device, struct C::mip_base_device_info& device_info)
{
    return C::mip_base_get_device_info(&device, &device_info);
}



struct getDeviceDescriptors : C::mip_base_get_device_descriptors_command
{
    static const uint8_t descriptorSet = MIP_BASE_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_BASE_GET_DEVICE_DESCRIPTORS;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::insert_mip_base_get_device_descriptors_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_base_get_device_descriptors_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = false;
    
    struct Response : C::mip_base_get_device_descriptors_response
    {
        static const uint8_t descriptorSet = MIP_BASE_CMD_DESC_SET;
        static const uint8_t fieldDescriptor = MIP_REPLY_DESC_BASE_DEVICE_DESCRIPTORS;
        
        size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::insert_mip_base_get_device_descriptors_response(buffer, bufferSize, offset, this);
        }
        size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::extract_mip_base_get_device_descriptors_response(buffer, bufferSize, offset, this);
        }
    };
    
};
MipCmdResult getDeviceDescriptors(C::mip_interface& device, uint8_t& descriptors_count, uint16_t* descriptors)
{
    return C::mip_base_get_device_descriptors(&device, &descriptors_count, descriptors);
}



struct builtInTest : C::mip_base_built_in_test_command
{
    static const uint8_t descriptorSet = MIP_BASE_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_BASE_BUILT_IN_TEST;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::insert_mip_base_built_in_test_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_base_built_in_test_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = false;
    
    struct Response : C::mip_base_built_in_test_response
    {
        static const uint8_t descriptorSet = MIP_BASE_CMD_DESC_SET;
        static const uint8_t fieldDescriptor = MIP_REPLY_DESC_BASE_BUILT_IN_TEST;
        
        size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::insert_mip_base_built_in_test_response(buffer, bufferSize, offset, this);
        }
        size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::extract_mip_base_built_in_test_response(buffer, bufferSize, offset, this);
        }
    };
    
};
MipCmdResult builtInTest(C::mip_interface& device, uint32_t& result)
{
    return C::mip_base_built_in_test(&device, &result);
}



struct resume : C::mip_base_resume_command
{
    static const uint8_t descriptorSet = MIP_BASE_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_BASE_RESUME;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::insert_mip_base_resume_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_base_resume_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = false;
    
};
MipCmdResult resume(C::mip_interface& device)
{
    return C::mip_base_resume(&device);
}



struct getExtendedDescriptors : C::mip_base_get_extended_descriptors_command
{
    static const uint8_t descriptorSet = MIP_BASE_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_BASE_GET_EXTENDED_DESCRIPTORS;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::insert_mip_base_get_extended_descriptors_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_base_get_extended_descriptors_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = false;
    
    struct Response : C::mip_base_get_extended_descriptors_response
    {
        static const uint8_t descriptorSet = MIP_BASE_CMD_DESC_SET;
        static const uint8_t fieldDescriptor = MIP_REPLY_DESC_BASE_GET_EXTENDED_DESCRIPTORS;
        
        size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::insert_mip_base_get_extended_descriptors_response(buffer, bufferSize, offset, this);
        }
        size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::extract_mip_base_get_extended_descriptors_response(buffer, bufferSize, offset, this);
        }
    };
    
};
MipCmdResult getExtendedDescriptors(C::mip_interface& device, uint8_t& descriptors_count, uint16_t* descriptors)
{
    return C::mip_base_get_extended_descriptors(&device, &descriptors_count, descriptors);
}



struct continuousBit : C::mip_base_continuous_bit_command
{
    static const uint8_t descriptorSet = MIP_BASE_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_BASE_CONTINUOUS_BIT;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::insert_mip_base_continuous_bit_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_base_continuous_bit_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = false;
    
    struct Response : C::mip_base_continuous_bit_response
    {
        static const uint8_t descriptorSet = MIP_BASE_CMD_DESC_SET;
        static const uint8_t fieldDescriptor = MIP_REPLY_DESC_BASE_CONTINUOUS_BIT;
        
        size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::insert_mip_base_continuous_bit_response(buffer, bufferSize, offset, this);
        }
        size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::extract_mip_base_continuous_bit_response(buffer, bufferSize, offset, this);
        }
    };
    
};
MipCmdResult continuousBit(C::mip_interface& device, uint8_t* result)
{
    return C::mip_base_continuous_bit(&device, result);
}



struct commSpeed : C::mip_base_comm_speed_command
{
    static const uint8_t descriptorSet = MIP_BASE_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_BASE_COMM_SPEED;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::insert_mip_base_comm_speed_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_base_comm_speed_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = true;
    static const bool canWrite = true;
    static const bool canRead = true;
    static const bool canSave = true;
    static const bool canLoad = true;
    static const bool canReset = true;
    
    struct Response : C::mip_base_comm_speed_response
    {
        static const uint8_t descriptorSet = MIP_BASE_CMD_DESC_SET;
        static const uint8_t fieldDescriptor = MIP_REPLY_DESC_BASE_COMM_SPEED;
        
        size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::insert_mip_base_comm_speed_response(buffer, bufferSize, offset, this);
        }
        size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::extract_mip_base_comm_speed_response(buffer, bufferSize, offset, this);
        }
    };
    
};
MipCmdResult writeCommspeed(C::mip_interface& device, uint8_t port, uint32_t baud)
{
    return C::write_mip_base_comm_speed(&device, port, baud);
}
MipCmdResult readCommspeed(C::mip_interface& device, uint8_t port, uint32_t& baud)
{
    return C::read_mip_base_comm_speed(&device, port, &baud);
}
MipCmdResult saveCommspeed(C::mip_interface& device, uint8_t port)
{
    return C::save_mip_base_comm_speed(&device, port);
}
MipCmdResult loadCommspeed(C::mip_interface& device, uint8_t port)
{
    return C::load_mip_base_comm_speed(&device, port);
}
MipCmdResult defaultCommspeed(C::mip_interface& device, uint8_t port)
{
    return C::default_mip_base_comm_speed(&device, port);
}



struct gpsTimeUpdate : C::mip_base_gps_time_update_command
{
    static const uint8_t descriptorSet = MIP_BASE_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_BASE_GPS_TIME_BROADCAST_NEW;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::insert_mip_base_gps_time_update_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_base_gps_time_update_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = true;
    static const bool canWrite = true;
    static const bool canRead = false;
    static const bool canSave = false;
    static const bool canLoad = false;
    static const bool canReset = false;
    
};
MipCmdResult writeGpstimeupdate(C::mip_interface& device, C::mip_base_gps_time_update_command_field_id field_id, uint32_t value)
{
    return C::write_mip_base_gps_time_update(&device, field_id, value);
}



struct softReset : C::mip_base_soft_reset_command
{
    static const uint8_t descriptorSet = MIP_BASE_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_BASE_SOFT_RESET;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::insert_mip_base_soft_reset_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_base_soft_reset_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = false;
    
};
MipCmdResult softReset(C::mip_interface& device)
{
    return C::mip_base_soft_reset(&device);
}



} // namespace BaseCommands
} // namespace mscl
#endif // __cplusplus
