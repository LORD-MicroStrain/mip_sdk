#pragma once

#include "descriptors.h"
#include "../mip_result.h"

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
namespace mscl {
namespace C {
extern "C" {

#endif // __cplusplus
struct mip_interface;
struct mip_serializer;

////////////////////////////////////////////////////////////////////////////////
///@addtogroup MipCommands
///@{
///@defgroup base_commands_c  BASECommands
///
///@{

////////////////////////////////////////////////////////////////////////////////
// Descriptors
////////////////////////////////////////////////////////////////////////////////

enum 
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
    MIP_CMD_DESC_BASE_GPS_TIME_BROADCAST         = 0x71,
    MIP_CMD_DESC_BASE_GPS_TIME_BROADCAST_NEW     = 0x72,
    MIP_CMD_DESC_BASE_SYSTEM_TIME                = 0x73,
    MIP_CMD_DESC_BASE_SOFT_RESET                 = 0x7E,
    
    MIP_REPLY_DESC_BASE_DEVICE_INFO              = 0x81,
    MIP_REPLY_DESC_BASE_DEVICE_DESCRIPTORS       = 0x82,
    MIP_REPLY_DESC_BASE_BUILT_IN_TEST            = 0x83,
    MIP_REPLY_DESC_BASE_GPS_CORRELATION_WEEK     = 0x84,
    MIP_REPLY_DESC_BASE_GPS_CORRELATION_SECONDS  = 0x85,
    MIP_REPLY_DESC_BASE_GET_EXTENDED_DESCRIPTORS = 0x86,
    MIP_REPLY_DESC_BASE_CONTINUOUS_BIT           = 0x88,
    MIP_REPLY_DESC_BASE_COMM_SPEED               = 0x89,
};

////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////

struct mip_base_device_info
{
    uint16_t firmware_version;
    char model_name[16];
    char model_number[16];
    char serial_number[16];
    char lot_number[16];
    char device_options[16];
    
};
void insert_mip_base_device_info(struct mip_serializer* serializer, const struct mip_base_device_info* self);
void extract_mip_base_device_info(struct mip_serializer* serializer, struct mip_base_device_info* self);

enum mip_time_format
{
    MIP_TIME_FORMAT_GPS = 1,  ///<  GPS time, a = week number since 1980, b = time of week in milliseconds.
};

void insert_mip_time_format(struct mip_serializer* serializer, const enum mip_time_format self);
void extract_mip_time_format(struct mip_serializer* serializer, enum mip_time_format* self);

enum mip_commanded_test_bits_gq7
{
    MIP_COMMANDED_TEST_BITS_GQ7_NONE                   = 0x00000000,
    MIP_COMMANDED_TEST_BITS_GQ7_GENERAL_HARDWARE_FAULT = 0x00000001,
    MIP_COMMANDED_TEST_BITS_GQ7_GENERAL_FIRMWARE_FAULT = 0x00000002,
    MIP_COMMANDED_TEST_BITS_GQ7_TIMING_OVERLOAD        = 0x00000004,
    MIP_COMMANDED_TEST_BITS_GQ7_BUFFER_OVERRUN         = 0x00000008,
    MIP_COMMANDED_TEST_BITS_GQ7_RESERVED               = 0x000000F0,
    MIP_COMMANDED_TEST_BITS_GQ7_IPC_IMU_FAULT          = 0x00000100,
    MIP_COMMANDED_TEST_BITS_GQ7_IPC_NAV_FAULT          = 0x00000200,
    MIP_COMMANDED_TEST_BITS_GQ7_IPC_GNSS_FAULT         = 0x00000400,
    MIP_COMMANDED_TEST_BITS_GQ7_COMMS_FAULT            = 0x00000800,
    MIP_COMMANDED_TEST_BITS_GQ7_IMU_ACCEL_FAULT        = 0x00001000,
    MIP_COMMANDED_TEST_BITS_GQ7_IMU_GYRO_FAULT         = 0x00002000,
    MIP_COMMANDED_TEST_BITS_GQ7_IMU_MAG_FAULT          = 0x00004000,
    MIP_COMMANDED_TEST_BITS_GQ7_IMU_PRESS_FAULT        = 0x00008000,
    MIP_COMMANDED_TEST_BITS_GQ7_IMU_RESERVED           = 0x00030000,
    MIP_COMMANDED_TEST_BITS_GQ7_IMU_CAL_ERROR          = 0x00040000,
    MIP_COMMANDED_TEST_BITS_GQ7_IMU_GENERAL_FAULT      = 0x00080000,
    MIP_COMMANDED_TEST_BITS_GQ7_FILT_RESERVED          = 0x00300000,
    MIP_COMMANDED_TEST_BITS_GQ7_FILT_SOLUTION_FAULT    = 0x00400000,
    MIP_COMMANDED_TEST_BITS_GQ7_FILT_GENERAL_FAULT     = 0x00800000,
    MIP_COMMANDED_TEST_BITS_GQ7_GNSS_RECEIVER1_FAULT   = 0x01000000,
    MIP_COMMANDED_TEST_BITS_GQ7_GNSS_ANTENNA1_FAULT    = 0x02000000,
    MIP_COMMANDED_TEST_BITS_GQ7_GNSS_RECEIVER2_FAULT   = 0x04000000,
    MIP_COMMANDED_TEST_BITS_GQ7_GNSS_ANTENNA2_FAULT    = 0x08000000,
    MIP_COMMANDED_TEST_BITS_GQ7_GNSS_RTCM_FAILURE      = 0x10000000,
    MIP_COMMANDED_TEST_BITS_GQ7_GNSS_RTK_FAULT         = 0x20000000,
    MIP_COMMANDED_TEST_BITS_GQ7_GNSS_SOLUTION_FAULT    = 0x40000000,
    MIP_COMMANDED_TEST_BITS_GQ7_GNSS_GENERAL_FAULT     = 0x80000000,
};

void insert_mip_commanded_test_bits_gq7(struct mip_serializer* serializer, const enum mip_commanded_test_bits_gq7 self);
void extract_mip_commanded_test_bits_gq7(struct mip_serializer* serializer, enum mip_commanded_test_bits_gq7* self);


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
///@defgroup c_ping  Ping
/// Test Communications with a device.
/// 
/// The Device will respond with an ACK, if present and operating correctly.
/// 
/// If the device is not in a normal operating mode, it may NACK.
///
///@{

mip_cmd_result mip_base_ping(struct mip_interface* device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_set_idle  Set to idle
/// Turn off all device data streams.
/// 
/// The Device will respond with an ACK, if present and operating correctly.
/// This command will suspend streaming (if enabled) or wake the device from sleep (if sleeping) to allow it to respond to status and setup commands.
/// You may restore the device mode by issuing the Resume command.
///
///@{

mip_cmd_result mip_base_set_idle(struct mip_interface* device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_get_device_info  Get device information
/// Get the device ID strings and firmware version number.
///
///@{

struct mip_base_get_device_info_response
{
    struct mip_base_device_info device_info;
    
};
void insert_mip_base_get_device_info_response(struct mip_serializer* serializer, const struct mip_base_get_device_info_response* self);
void extract_mip_base_get_device_info_response(struct mip_serializer* serializer, struct mip_base_get_device_info_response* self);

mip_cmd_result mip_base_get_device_info(struct mip_interface* device, struct mip_base_device_info* device_info);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_get_device_descriptors  Get device descriptors
/// Get the command and data descriptors supported by the device.
/// 
/// Reply has two fields: "ACK/NACK" and "Descriptors". The "Descriptors" field is an array of 16 bit values.
/// The MSB specifies the descriptor set and the LSB specifies the descriptor.
///
///@{

struct mip_base_get_device_descriptors_response
{
    uint8_t descriptors_count;
    uint16_t descriptors[253];
    
};
void insert_mip_base_get_device_descriptors_response(struct mip_serializer* serializer, const struct mip_base_get_device_descriptors_response* self);
void extract_mip_base_get_device_descriptors_response(struct mip_serializer* serializer, struct mip_base_get_device_descriptors_response* self);

mip_cmd_result mip_base_get_device_descriptors(struct mip_interface* device, uint8_t* descriptors_count, uint16_t* descriptors);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_built_in_test  Built in test
/// Run the device Built-In Test (BIT).
/// 
/// The Built-In Test command always returns a 32 bit value.
/// A value of 0 means that all tests passed.
/// A non-zero value indicates that not all tests passed.
/// Reference the device user manual to decode the result.
///
///@{

struct mip_base_built_in_test_response
{
    uint32_t result;
    
};
void insert_mip_base_built_in_test_response(struct mip_serializer* serializer, const struct mip_base_built_in_test_response* self);
void extract_mip_base_built_in_test_response(struct mip_serializer* serializer, struct mip_base_built_in_test_response* self);

mip_cmd_result mip_base_built_in_test(struct mip_interface* device, uint32_t* result);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_resume  Resume
/// Take the device out of idle mode.
/// 
/// The device responds with ACK upon success.
///
///@{

mip_cmd_result mip_base_resume(struct mip_interface* device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_get_extended_descriptors  Get device descriptors (extended)
/// Get the command and data descriptors supported by the device.
/// 
/// Reply has two fields: "ACK/NACK" and "Descriptors". The "Descriptors" field is an array of 16 bit values.
/// The MSB specifies the descriptor set and the LSB specifies the descriptor.
///
///@{

struct mip_base_get_extended_descriptors_response
{
    uint8_t descriptors_count;
    uint16_t descriptors[253];
    
};
void insert_mip_base_get_extended_descriptors_response(struct mip_serializer* serializer, const struct mip_base_get_extended_descriptors_response* self);
void extract_mip_base_get_extended_descriptors_response(struct mip_serializer* serializer, struct mip_base_get_extended_descriptors_response* self);

mip_cmd_result mip_base_get_extended_descriptors(struct mip_interface* device, uint8_t* descriptors_count, uint16_t* descriptors);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_continuous_bit  Continuous built-in test
/// Report result of continous built-in test.
/// 
/// This test is non-disruptive but is not as thorough as the commanded BIT.
///
///@{

struct mip_base_continuous_bit_response
{
    uint8_t result[16];
    
};
void insert_mip_base_continuous_bit_response(struct mip_serializer* serializer, const struct mip_base_continuous_bit_response* self);
void extract_mip_base_continuous_bit_response(struct mip_serializer* serializer, struct mip_base_continuous_bit_response* self);

mip_cmd_result mip_base_continuous_bit(struct mip_interface* device, uint8_t* result);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_comm_speed  Comm Port Speed
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

enum { ALL_PORTS = 0 };
struct mip_base_comm_speed_command
{
    enum mip_function_selector function;
    uint8_t port;
    uint32_t baud;
    
};
void insert_mip_base_comm_speed_command(struct mip_serializer* serializer, const struct mip_base_comm_speed_command* self);
void extract_mip_base_comm_speed_command(struct mip_serializer* serializer, struct mip_base_comm_speed_command* self);

struct mip_base_comm_speed_response
{
    uint8_t port;
    uint32_t baud;
    
};
void insert_mip_base_comm_speed_response(struct mip_serializer* serializer, const struct mip_base_comm_speed_response* self);
void extract_mip_base_comm_speed_response(struct mip_serializer* serializer, struct mip_base_comm_speed_response* self);

mip_cmd_result mip_base_write_comm_speed(struct mip_interface* device, uint8_t port, uint32_t baud);
mip_cmd_result mip_base_read_comm_speed(struct mip_interface* device, uint8_t port, uint32_t* baud);
mip_cmd_result mip_base_save_comm_speed(struct mip_interface* device, uint8_t port);
mip_cmd_result mip_base_load_comm_speed(struct mip_interface* device, uint8_t port);
mip_cmd_result mip_base_default_comm_speed(struct mip_interface* device, uint8_t port);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_gps_time_update  Time Broadcast Command
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

struct mip_base_gps_time_update_command
{
    enum mip_function_selector function;
    enum mip_base_gps_time_update_command_field_id field_id;
    uint32_t value;
    
};
void insert_mip_base_gps_time_update_command(struct mip_serializer* serializer, const struct mip_base_gps_time_update_command* self);
void extract_mip_base_gps_time_update_command(struct mip_serializer* serializer, struct mip_base_gps_time_update_command* self);

void insert_mip_base_gps_time_update_command_field_id(struct mip_serializer* serializer, const enum mip_base_gps_time_update_command_field_id self);
void extract_mip_base_gps_time_update_command_field_id(struct mip_serializer* serializer, enum mip_base_gps_time_update_command_field_id* self);

mip_cmd_result mip_base_write_gps_time_update(struct mip_interface* device, enum mip_base_gps_time_update_command_field_id field_id, uint32_t value);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_soft_reset  Reset device
/// Resets the device.
/// 
/// Device responds with ACK and immediately resets.
///
///@{

mip_cmd_result mip_base_soft_reset(struct mip_interface* device);
///@}
///

///@}
///@}
///
////////////////////////////////////////////////////////////////////////////////
#ifdef __cplusplus
} // namespace C
} // namespace mscl
} // extern "C"
#endif // __cplusplus

