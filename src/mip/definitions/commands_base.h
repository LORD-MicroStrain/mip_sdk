#pragma once

#include "descriptors.h"
#include "../mip_result.h"

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
namespace mscl {
extern "C" {
#endif // __cplusplus

struct MipInterfaceState;

////////////////////////////////////////////////////////////////////////////////
///@addtogroup MipCommands
///@{
///@defgroup BASE_COMMAND  BASE COMMAND
///
///@{

////////////////////////////////////////////////////////////////////////////////
// Descriptors
////////////////////////////////////////////////////////////////////////////////

enum MipBaseCommandDescriptors
{
    MIP_BASE_COMMAND_DESC_SET                    = 0x01,
    
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

////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////

struct MipBaseDeviceInfo
{
    uint16_t                                          firmware_version;
    char                                              model_name[16];
    char                                              model_number[16];
    char                                              serial_number[16];
    char                                              lot_number[16];
    char                                              device_options[16];
};
size_t insert_MipBaseDeviceInfo(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipBaseDeviceInfo* self);
size_t extract_MipBaseDeviceInfo(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipBaseDeviceInfo* self);

enum MipTimeFormat
{
    MIPTIMEFORMAT_GPS = 1,  ///<  GPS time, a = week number since 1980, b = time of week in milliseconds.
};
size_t insert_MipTimeFormat(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipTimeFormat self);
size_t extract_MipTimeFormat(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipTimeFormat* self);

enum MipCommandedTestBitsGq7
{
    MIPCOMMANDEDTESTBITSGQ7_GENERAL_HARDWARE_FAULT = 0x01,
    MIPCOMMANDEDTESTBITSGQ7_GENERAL_FIRMWARE_FAULT = 0x02,
    MIPCOMMANDEDTESTBITSGQ7_TIMING_OVERLOAD        = 0x04,
    MIPCOMMANDEDTESTBITSGQ7_BUFFER_OVERRUN         = 0x08,
    MIPCOMMANDEDTESTBITSGQ7_RESERVED               = 0xF0,
    MIPCOMMANDEDTESTBITSGQ7_IPC_IMU_FAULT          = 0x100,
    MIPCOMMANDEDTESTBITSGQ7_IPC_NAV_FAULT          = 0x200,
    MIPCOMMANDEDTESTBITSGQ7_IPC_GNSS_FAULT         = 0x400,
    MIPCOMMANDEDTESTBITSGQ7_COMMS_FAULT            = 0x800,
    MIPCOMMANDEDTESTBITSGQ7_IMU_ACCEL_FAULT        = 0x1000,
    MIPCOMMANDEDTESTBITSGQ7_IMU_GYRO_FAULT         = 0x2000,
    MIPCOMMANDEDTESTBITSGQ7_IMU_MAG_FAULT          = 0x4000,
    MIPCOMMANDEDTESTBITSGQ7_IMU_PRESS_FAULT        = 0x8000,
    MIPCOMMANDEDTESTBITSGQ7_IMU_RESERVED           = 0x30000,
    MIPCOMMANDEDTESTBITSGQ7_IMU_CAL_ERROR          = 0x40000,
    MIPCOMMANDEDTESTBITSGQ7_IMU_GENERAL_FAULT      = 0x80000,
    MIPCOMMANDEDTESTBITSGQ7_FILT_RESERVED          = 0x300000,
    MIPCOMMANDEDTESTBITSGQ7_FILT_SOLUTION_FAULT    = 0x400000,
    MIPCOMMANDEDTESTBITSGQ7_FILT_GENERAL_FAULT     = 0x800000,
    MIPCOMMANDEDTESTBITSGQ7_GNSS_RECEIVER1_FAULT   = 0x1000000,
    MIPCOMMANDEDTESTBITSGQ7_GNSS_ANTENNA1_FAULT    = 0x2000000,
    MIPCOMMANDEDTESTBITSGQ7_GNSS_RECEIVER2_FAULT   = 0x4000000,
    MIPCOMMANDEDTESTBITSGQ7_GNSS_ANTENNA2_FAULT    = 0x8000000,
    MIPCOMMANDEDTESTBITSGQ7_GNSS_RTCM_FAILURE      = 0x10000000,
    MIPCOMMANDEDTESTBITSGQ7_GNSS_RTK_FAULT         = 0x20000000,
    MIPCOMMANDEDTESTBITSGQ7_GNSS_SOLUTION_FAULT    = 0x40000000,
    MIPCOMMANDEDTESTBITSGQ7_GNSS_GENERAL_FAULT     = 0x80000000,
};
size_t insert_MipCommandedTestBitsGq7(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipCommandedTestBitsGq7 self);
size_t extract_MipCommandedTestBitsGq7(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipCommandedTestBitsGq7* self);


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_cmd_base_ping  Base Ping
/// Test Communications with a device.
/// 
/// The Device will respond with an ACK, if present and operating correctly.
/// 
/// If the device is not in a normal operating mode, it may NACK.
///
///@{

struct MipCmd_Base_Ping
{
};
size_t insert_MipCmd_Base_Ping(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Base_Ping* self);
size_t extract_MipCmd_Base_Ping(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Base_Ping* self);

MipCmdResult ping(struct MipInterfaceState* device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_cmd_base_set_idle  Base Set Idle
/// Turn off all device data streams.
/// 
/// The Device will respond with an ACK, if present and operating correctly.
/// This command will suspend streaming (if enabled) or wake the device from sleep (if sleeping) to allow it to respond to status and setup commands.
/// You may restore the device mode by issuing the Resume command.
///
///@{

struct MipCmd_Base_SetIdle
{
};
size_t insert_MipCmd_Base_SetIdle(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Base_SetIdle* self);
size_t extract_MipCmd_Base_SetIdle(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Base_SetIdle* self);

MipCmdResult set_to_idle(struct MipInterfaceState* device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_cmd_base_get_device_info  Base Get Device Info
/// Get the device ID strings and firmware version number.
///
///@{

struct MipCmd_Base_GetDeviceInfo
{
};
size_t insert_MipCmd_Base_GetDeviceInfo(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Base_GetDeviceInfo* self);
size_t extract_MipCmd_Base_GetDeviceInfo(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Base_GetDeviceInfo* self);

struct MipCmd_Base_GetDeviceInfo_Response
{
    struct MipBaseDeviceInfo                          device_info;
};
size_t insert_MipCmd_Base_GetDeviceInfo_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Base_GetDeviceInfo_Response* self);
size_t extract_MipCmd_Base_GetDeviceInfo_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Base_GetDeviceInfo_Response* self);

MipCmdResult get_device_information(struct MipInterfaceState* device, struct MipBaseDeviceInfo* device_info);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_cmd_base_get_device_descriptors  Base Get Device Descriptors
/// Get the command and data descriptors supported by the device.
/// 
/// Reply has two fields: "ACK/NACK" and "Descriptors". The "Descriptors" field is an array of 16 bit values.
/// The MSB specifies the descriptor set and the LSB specifies the descriptor.
///
///@{

struct MipCmd_Base_GetDeviceDescriptors
{
};
size_t insert_MipCmd_Base_GetDeviceDescriptors(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Base_GetDeviceDescriptors* self);
size_t extract_MipCmd_Base_GetDeviceDescriptors(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Base_GetDeviceDescriptors* self);

struct MipCmd_Base_GetDeviceDescriptors_Response
{
    uint8_t                                           descriptors_count;
    uint16_t*                                         descriptors;
};
size_t insert_MipCmd_Base_GetDeviceDescriptors_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Base_GetDeviceDescriptors_Response* self);
size_t extract_MipCmd_Base_GetDeviceDescriptors_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Base_GetDeviceDescriptors_Response* self);

MipCmdResult get_device_descriptors(struct MipInterfaceState* device, uint8_t* descriptors_count, uint16_t* descriptors);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_cmd_base_built_in_test  Base Built In Test
/// Run the device Built-In Test (BIT).
/// 
/// The Built-In Test command always returns a 32 bit value.
/// A value of 0 means that all tests passed.
/// A non-zero value indicates that not all tests passed.
/// Reference the device user manual to decode the result.
///
///@{

struct MipCmd_Base_BuiltInTest
{
};
size_t insert_MipCmd_Base_BuiltInTest(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Base_BuiltInTest* self);
size_t extract_MipCmd_Base_BuiltInTest(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Base_BuiltInTest* self);

struct MipCmd_Base_BuiltInTest_Response
{
    uint32_t                                          result;
};
size_t insert_MipCmd_Base_BuiltInTest_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Base_BuiltInTest_Response* self);
size_t extract_MipCmd_Base_BuiltInTest_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Base_BuiltInTest_Response* self);

MipCmdResult built_in_test(struct MipInterfaceState* device, uint32_t* result);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_cmd_base_resume  Base Resume
/// Take the device out of idle mode.
/// 
/// The device responds with ACK upon success.
///
///@{

struct MipCmd_Base_Resume
{
};
size_t insert_MipCmd_Base_Resume(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Base_Resume* self);
size_t extract_MipCmd_Base_Resume(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Base_Resume* self);

MipCmdResult resume(struct MipInterfaceState* device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_cmd_base_get_extended_descriptors  Base Get Extended Descriptors
/// Get the command and data descriptors supported by the device.
/// 
/// Reply has two fields: "ACK/NACK" and "Descriptors". The "Descriptors" field is an array of 16 bit values.
/// The MSB specifies the descriptor set and the LSB specifies the descriptor.
///
///@{

struct MipCmd_Base_GetExtendedDescriptors
{
};
size_t insert_MipCmd_Base_GetExtendedDescriptors(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Base_GetExtendedDescriptors* self);
size_t extract_MipCmd_Base_GetExtendedDescriptors(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Base_GetExtendedDescriptors* self);

struct MipCmd_Base_GetExtendedDescriptors_Response
{
    uint8_t                                           descriptors_count;
    uint16_t*                                         descriptors;
};
size_t insert_MipCmd_Base_GetExtendedDescriptors_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Base_GetExtendedDescriptors_Response* self);
size_t extract_MipCmd_Base_GetExtendedDescriptors_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Base_GetExtendedDescriptors_Response* self);

MipCmdResult get_device_descriptors_extended(struct MipInterfaceState* device, uint8_t* descriptors_count, uint16_t* descriptors);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_cmd_base_continuous_bit  Base Continuous Bit
/// Report result of continous built-in test.
/// 
/// This test is non-disruptive but is not as thorough as the commanded BIT.
///
///@{

struct MipCmd_Base_ContinuousBit
{
};
size_t insert_MipCmd_Base_ContinuousBit(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Base_ContinuousBit* self);
size_t extract_MipCmd_Base_ContinuousBit(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Base_ContinuousBit* self);

struct MipCmd_Base_ContinuousBit_Response
{
    uint8_t                                           result[16];
};
size_t insert_MipCmd_Base_ContinuousBit_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Base_ContinuousBit_Response* self);
size_t extract_MipCmd_Base_ContinuousBit_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Base_ContinuousBit_Response* self);

MipCmdResult continuous_built_in_test(struct MipInterfaceState* device, uint8_t* result);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_cmd_base_comm_speed  Base Comm Speed
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

struct MipCmd_Base_CommSpeed
{
    enum MipFunctionSelector                          function;
    uint8_t                                           port;
    uint32_t                                          baud;
};
size_t insert_MipCmd_Base_CommSpeed(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Base_CommSpeed* self);
size_t extract_MipCmd_Base_CommSpeed(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Base_CommSpeed* self);

struct MipCmd_Base_CommSpeed_Response
{
    uint8_t                                           port;
    uint32_t                                          baud;
};
size_t insert_MipCmd_Base_CommSpeed_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Base_CommSpeed_Response* self);
size_t extract_MipCmd_Base_CommSpeed_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Base_CommSpeed_Response* self);

MipCmdResult write_comm_port_speed(struct MipInterfaceState* device, uint8_t port, uint32_t baud);
MipCmdResult read_comm_port_speed(struct MipInterfaceState* device, uint8_t port, uint32_t* baud);
MipCmdResult save_comm_port_speed(struct MipInterfaceState* device, uint8_t port);
MipCmdResult load_comm_port_speed(struct MipInterfaceState* device, uint8_t port);
MipCmdResult default_comm_port_speed(struct MipInterfaceState* device, uint8_t port);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_cmd_base_gps_time_update  Base Gps Time Update
/// When combined with a PPS input signal applied to the I/O connector, this command enables complete synchronization of data outputs
/// with an external time base, such as GPS system time. Since the hardware PPS synchronization can only detect the fractional number of seconds when pulses arrive,
/// complete synchronization requires that the user provide the whole number of seconds via this command. After achieving PPS synchronization, this command should be sent twice: once to set the time-of-week and once to set the week number. PPS synchronization can be verified by monitoring the time sync status message (0xA0, 0x02) or the valid flags of any shared external timestamp (0x--, D7) data field.
///
///@{

enum MipCmd_Base_GpsTimeUpdate_Fieldid
{
    MIPCMD_BASE_GPSTIMEUPDATE_FIELDID_WEEK_NUMBER  = 1,  ///<  Week number.
    MIPCMD_BASE_GPSTIMEUPDATE_FIELDID_TIME_OF_WEEK = 2,  ///<  Time of week in seconds.
};
size_t insert_MipCmd_Base_GpsTimeUpdate_Fieldid(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipCmd_Base_GpsTimeUpdate_Fieldid self);
size_t extract_MipCmd_Base_GpsTimeUpdate_Fieldid(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipCmd_Base_GpsTimeUpdate_Fieldid* self);

struct MipCmd_Base_GpsTimeUpdate
{
    enum MipFunctionSelector                          function;
    enum MipCmd_Base_GpsTimeUpdate_Fieldid            field_id;
    uint32_t                                          value;
};
size_t insert_MipCmd_Base_GpsTimeUpdate(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Base_GpsTimeUpdate* self);
size_t extract_MipCmd_Base_GpsTimeUpdate(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Base_GpsTimeUpdate* self);

MipCmdResult write_time_broadcast_command(struct MipInterfaceState* device, enum MipCmd_Base_GpsTimeUpdate_Fieldid field_id, uint32_t value);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_cmd_base_soft_reset  Base Soft Reset
/// Resets the device.
/// 
/// Device responds with ACK and immediately resets.
///
///@{

struct MipCmd_Base_SoftReset
{
};
size_t insert_MipCmd_Base_SoftReset(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Base_SoftReset* self);
size_t extract_MipCmd_Base_SoftReset(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Base_SoftReset* self);

MipCmdResult reset_device(struct MipInterfaceState* device);
///@}
///

///@}
///@}
///
////////////////////////////////////////////////////////////////////////////////

#ifdef __cplusplus
} // extern "C"


template<>
struct MipFieldInfo<MipCmd_Base_Ping>
{
    static const uint8_t descriptorSet = MIP_BASE_COMMAND_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_BASE_PING;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipCmd_Base_Ping& self)
    {
        return insert_MipCmd_Base_Ping(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipCmd_Base_Ping& self)
    {
        return extract_MipCmd_Base_Ping(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipCmd_Base_SetIdle>
{
    static const uint8_t descriptorSet = MIP_BASE_COMMAND_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_BASE_SET_TO_IDLE;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipCmd_Base_SetIdle& self)
    {
        return insert_MipCmd_Base_SetIdle(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipCmd_Base_SetIdle& self)
    {
        return extract_MipCmd_Base_SetIdle(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipCmd_Base_GetDeviceInfo>
{
    static const uint8_t descriptorSet = MIP_BASE_COMMAND_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_BASE_GET_DEVICE_INFO;
    static const uint8_t responseDescriptor = MIP_REPLY_DESC_BASE_DEVICE_INFO;
    typedef MipCmd_Base_GetDeviceInfo_Response Response;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipCmd_Base_GetDeviceInfo& self)
    {
        return insert_MipCmd_Base_GetDeviceInfo(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipCmd_Base_GetDeviceInfo& self)
    {
        return extract_MipCmd_Base_GetDeviceInfo(buffer, bufferSize, offset, &self);
    }
    static inline size_t insert_response(uint8_t* buffer, size_t bufferSize, size_t offset, const MipCmd_Base_GetDeviceInfo_Response& self)
    {
        return insert_MipCmd_Base_GetDeviceInfo_Response(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract_response(const uint8_t* buffer, size_t bufferSize, size_t offset, MipCmd_Base_GetDeviceInfo_Response& self)
    {
        return extract_MipCmd_Base_GetDeviceInfo_Response(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipCmd_Base_GetDeviceDescriptors>
{
    static const uint8_t descriptorSet = MIP_BASE_COMMAND_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_BASE_GET_DEVICE_DESCRIPTORS;
    static const uint8_t responseDescriptor = MIP_REPLY_DESC_BASE_DEVICE_DESCRIPTORS;
    typedef MipCmd_Base_GetDeviceDescriptors_Response Response;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipCmd_Base_GetDeviceDescriptors& self)
    {
        return insert_MipCmd_Base_GetDeviceDescriptors(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipCmd_Base_GetDeviceDescriptors& self)
    {
        return extract_MipCmd_Base_GetDeviceDescriptors(buffer, bufferSize, offset, &self);
    }
    static inline size_t insert_response(uint8_t* buffer, size_t bufferSize, size_t offset, const MipCmd_Base_GetDeviceDescriptors_Response& self)
    {
        return insert_MipCmd_Base_GetDeviceDescriptors_Response(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract_response(const uint8_t* buffer, size_t bufferSize, size_t offset, MipCmd_Base_GetDeviceDescriptors_Response& self)
    {
        return extract_MipCmd_Base_GetDeviceDescriptors_Response(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipCmd_Base_BuiltInTest>
{
    static const uint8_t descriptorSet = MIP_BASE_COMMAND_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_BASE_BUILT_IN_TEST;
    static const uint8_t responseDescriptor = MIP_REPLY_DESC_BASE_BUILT_IN_TEST;
    typedef MipCmd_Base_BuiltInTest_Response Response;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipCmd_Base_BuiltInTest& self)
    {
        return insert_MipCmd_Base_BuiltInTest(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipCmd_Base_BuiltInTest& self)
    {
        return extract_MipCmd_Base_BuiltInTest(buffer, bufferSize, offset, &self);
    }
    static inline size_t insert_response(uint8_t* buffer, size_t bufferSize, size_t offset, const MipCmd_Base_BuiltInTest_Response& self)
    {
        return insert_MipCmd_Base_BuiltInTest_Response(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract_response(const uint8_t* buffer, size_t bufferSize, size_t offset, MipCmd_Base_BuiltInTest_Response& self)
    {
        return extract_MipCmd_Base_BuiltInTest_Response(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipCmd_Base_Resume>
{
    static const uint8_t descriptorSet = MIP_BASE_COMMAND_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_BASE_RESUME;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipCmd_Base_Resume& self)
    {
        return insert_MipCmd_Base_Resume(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipCmd_Base_Resume& self)
    {
        return extract_MipCmd_Base_Resume(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipCmd_Base_GetExtendedDescriptors>
{
    static const uint8_t descriptorSet = MIP_BASE_COMMAND_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_BASE_GET_EXTENDED_DESCRIPTORS;
    static const uint8_t responseDescriptor = MIP_REPLY_DESC_BASE_GET_EXTENDED_DESCRIPTORS;
    typedef MipCmd_Base_GetExtendedDescriptors_Response Response;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipCmd_Base_GetExtendedDescriptors& self)
    {
        return insert_MipCmd_Base_GetExtendedDescriptors(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipCmd_Base_GetExtendedDescriptors& self)
    {
        return extract_MipCmd_Base_GetExtendedDescriptors(buffer, bufferSize, offset, &self);
    }
    static inline size_t insert_response(uint8_t* buffer, size_t bufferSize, size_t offset, const MipCmd_Base_GetExtendedDescriptors_Response& self)
    {
        return insert_MipCmd_Base_GetExtendedDescriptors_Response(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract_response(const uint8_t* buffer, size_t bufferSize, size_t offset, MipCmd_Base_GetExtendedDescriptors_Response& self)
    {
        return extract_MipCmd_Base_GetExtendedDescriptors_Response(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipCmd_Base_ContinuousBit>
{
    static const uint8_t descriptorSet = MIP_BASE_COMMAND_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_BASE_CONTINUOUS_BIT;
    static const uint8_t responseDescriptor = MIP_REPLY_DESC_BASE_CONTINUOUS_BIT;
    typedef MipCmd_Base_ContinuousBit_Response Response;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipCmd_Base_ContinuousBit& self)
    {
        return insert_MipCmd_Base_ContinuousBit(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipCmd_Base_ContinuousBit& self)
    {
        return extract_MipCmd_Base_ContinuousBit(buffer, bufferSize, offset, &self);
    }
    static inline size_t insert_response(uint8_t* buffer, size_t bufferSize, size_t offset, const MipCmd_Base_ContinuousBit_Response& self)
    {
        return insert_MipCmd_Base_ContinuousBit_Response(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract_response(const uint8_t* buffer, size_t bufferSize, size_t offset, MipCmd_Base_ContinuousBit_Response& self)
    {
        return extract_MipCmd_Base_ContinuousBit_Response(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipCmd_Base_CommSpeed>
{
    static const uint8_t descriptorSet = MIP_BASE_COMMAND_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_BASE_COMM_SPEED;
    static const uint8_t responseDescriptor = MIP_REPLY_DESC_BASE_COMM_SPEED;
    typedef MipCmd_Base_CommSpeed_Response Response;
    
    static const bool hasFunctionSelector = true;
    static const bool canWrite = true;
    static const bool canRead = true;
    static const bool canSave = true;
    static const bool canLoad = true;
    static const bool canReset = true;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipCmd_Base_CommSpeed& self)
    {
        return insert_MipCmd_Base_CommSpeed(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipCmd_Base_CommSpeed& self)
    {
        return extract_MipCmd_Base_CommSpeed(buffer, bufferSize, offset, &self);
    }
    static inline size_t insert_response(uint8_t* buffer, size_t bufferSize, size_t offset, const MipCmd_Base_CommSpeed_Response& self)
    {
        return insert_MipCmd_Base_CommSpeed_Response(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract_response(const uint8_t* buffer, size_t bufferSize, size_t offset, MipCmd_Base_CommSpeed_Response& self)
    {
        return extract_MipCmd_Base_CommSpeed_Response(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipCmd_Base_GpsTimeUpdate>
{
    static const uint8_t descriptorSet = MIP_BASE_COMMAND_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_BASE_GPS_TIME_BROADCAST_NEW;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = true;
    static const bool canWrite = true;
    static const bool canRead = false;
    static const bool canSave = false;
    static const bool canLoad = false;
    static const bool canReset = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipCmd_Base_GpsTimeUpdate& self)
    {
        return insert_MipCmd_Base_GpsTimeUpdate(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipCmd_Base_GpsTimeUpdate& self)
    {
        return extract_MipCmd_Base_GpsTimeUpdate(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipCmd_Base_SoftReset>
{
    static const uint8_t descriptorSet = MIP_BASE_COMMAND_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_BASE_SOFT_RESET;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipCmd_Base_SoftReset& self)
    {
        return insert_MipCmd_Base_SoftReset(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipCmd_Base_SoftReset& self)
    {
        return extract_MipCmd_Base_SoftReset(buffer, bufferSize, offset, &self);
    }
};



} // namespace mscl
#endif // __cplusplus
