#pragma once

#include "descriptors.h"
#include "../mip_result.h"

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

namespace mscl {
class MipSerializer;

namespace C {
struct mip_interface;
} // namespace C

namespace commands_base {

////////////////////////////////////////////////////////////////////////////////
///@addtogroup MipCommands
///@{
///@defgroup base_commands_cpp  BASECommands
///
///@{

////////////////////////////////////////////////////////////////////////////////
// Descriptors
////////////////////////////////////////////////////////////////////////////////

enum 
{
    DESCRIPTOR_SET                 = 0x01,
    
    CMD_PING                       = 0x01,
    CMD_SET_TO_IDLE                = 0x02,
    CMD_GET_DEVICE_INFO            = 0x03,
    CMD_GET_DEVICE_DESCRIPTORS     = 0x04,
    CMD_BUILT_IN_TEST              = 0x05,
    CMD_RESUME                     = 0x06,
    CMD_GET_EXTENDED_DESCRIPTORS   = 0x07,
    CMD_CONTINUOUS_BIT             = 0x08,
    CMD_COMM_SPEED                 = 0x09,
    CMD_GPS_TIME_BROADCAST         = 0x71,
    CMD_GPS_TIME_BROADCAST_NEW     = 0x72,
    CMD_SYSTEM_TIME                = 0x73,
    CMD_SOFT_RESET                 = 0x7E,
    
    REPLY_DEVICE_INFO              = 0x81,
    REPLY_DEVICE_DESCRIPTORS       = 0x82,
    REPLY_BUILT_IN_TEST            = 0x83,
    REPLY_GPS_CORRELATION_WEEK     = 0x84,
    REPLY_GPS_CORRELATION_SECONDS  = 0x85,
    REPLY_GET_EXTENDED_DESCRIPTORS = 0x86,
    REPLY_CONTINUOUS_BIT           = 0x88,
    REPLY_COMM_SPEED               = 0x89,
};

////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////

struct BaseDeviceInfo
{
    uint16_t firmware_version;
    char model_name[16];
    char model_number[16];
    char serial_number[16];
    char lot_number[16];
    char device_options[16];
    
};
void insert(MipSerializer& serializer, const BaseDeviceInfo& self);
void extract(MipSerializer& serializer, BaseDeviceInfo& self);

enum class TimeFormat : uint8_t
{
    GPS = 1,  ///<  GPS time, a = week number since 1980, b = time of week in milliseconds.
};

struct CommandedTestBitsGq7 : Bitfield<CommandedTestBitsGq7>
{
    enum _enumType : uint32_t
    {
        NONE                   = 0x00000000,
        GENERAL_HARDWARE_FAULT = 0x00000001,
        GENERAL_FIRMWARE_FAULT = 0x00000002,
        TIMING_OVERLOAD        = 0x00000004,
        BUFFER_OVERRUN         = 0x00000008,
        RESERVED               = 0x000000F0,
        IPC_IMU_FAULT          = 0x00000100,
        IPC_NAV_FAULT          = 0x00000200,
        IPC_GNSS_FAULT         = 0x00000400,
        COMMS_FAULT            = 0x00000800,
        IMU_ACCEL_FAULT        = 0x00001000,
        IMU_GYRO_FAULT         = 0x00002000,
        IMU_MAG_FAULT          = 0x00004000,
        IMU_PRESS_FAULT        = 0x00008000,
        IMU_RESERVED           = 0x00030000,
        IMU_CAL_ERROR          = 0x00040000,
        IMU_GENERAL_FAULT      = 0x00080000,
        FILT_RESERVED          = 0x00300000,
        FILT_SOLUTION_FAULT    = 0x00400000,
        FILT_GENERAL_FAULT     = 0x00800000,
        GNSS_RECEIVER1_FAULT   = 0x01000000,
        GNSS_ANTENNA1_FAULT    = 0x02000000,
        GNSS_RECEIVER2_FAULT   = 0x04000000,
        GNSS_ANTENNA2_FAULT    = 0x08000000,
        GNSS_RTCM_FAILURE      = 0x10000000,
        GNSS_RTK_FAULT         = 0x20000000,
        GNSS_SOLUTION_FAULT    = 0x40000000,
        GNSS_GENERAL_FAULT     = 0x80000000,
    };
    uint32_t value = NONE;
    
    operator uint32_t() const { return value; }
    CommandedTestBitsGq7& operator=(uint32_t val) { value = val; return *this; }
    CommandedTestBitsGq7& operator|=(uint32_t val) { return *this = value | val; }
    CommandedTestBitsGq7& operator&=(uint32_t val) { return *this = value & val; }
};


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_ping  Ping
/// Test Communications with a device.
/// 
/// The Device will respond with an ACK, if present and operating correctly.
/// 
/// If the device is not in a normal operating mode, it may NACK.
///
///@{

struct Ping
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::commands_base::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::commands_base::CMD_PING;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    
};
void insert(MipSerializer& serializer, const Ping& self);
void extract(MipSerializer& serializer, Ping& self);

MipCmdResult ping(C::mip_interface& device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_set_idle  Set to idle
/// Turn off all device data streams.
/// 
/// The Device will respond with an ACK, if present and operating correctly.
/// This command will suspend streaming (if enabled) or wake the device from sleep (if sleeping) to allow it to respond to status and setup commands.
/// You may restore the device mode by issuing the Resume command.
///
///@{

struct SetIdle
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::commands_base::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::commands_base::CMD_SET_TO_IDLE;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    
};
void insert(MipSerializer& serializer, const SetIdle& self);
void extract(MipSerializer& serializer, SetIdle& self);

MipCmdResult setIdle(C::mip_interface& device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_get_device_info  Get device information
/// Get the device ID strings and firmware version number.
///
///@{

struct GetDeviceInfo
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::commands_base::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::commands_base::CMD_GET_DEVICE_INFO;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mscl::commands_base::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mscl::commands_base::REPLY_DEVICE_INFO;
        
        BaseDeviceInfo device_info;
        
    };
};
void insert(MipSerializer& serializer, const GetDeviceInfo& self);
void extract(MipSerializer& serializer, GetDeviceInfo& self);

void insert(MipSerializer& serializer, const GetDeviceInfo::Response& self);
void extract(MipSerializer& serializer, GetDeviceInfo::Response& self);

MipCmdResult getDeviceInfo(C::mip_interface& device, BaseDeviceInfo& device_info);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_get_device_descriptors  Get device descriptors
/// Get the command and data descriptors supported by the device.
/// 
/// Reply has two fields: "ACK/NACK" and "Descriptors". The "Descriptors" field is an array of 16 bit values.
/// The MSB specifies the descriptor set and the LSB specifies the descriptor.
///
///@{

struct GetDeviceDescriptors
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::commands_base::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::commands_base::CMD_GET_DEVICE_DESCRIPTORS;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mscl::commands_base::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mscl::commands_base::REPLY_DEVICE_DESCRIPTORS;
        
        uint8_t descriptors_count;
        uint16_t descriptors[253];
        
    };
};
void insert(MipSerializer& serializer, const GetDeviceDescriptors& self);
void extract(MipSerializer& serializer, GetDeviceDescriptors& self);

void insert(MipSerializer& serializer, const GetDeviceDescriptors::Response& self);
void extract(MipSerializer& serializer, GetDeviceDescriptors::Response& self);

MipCmdResult getDeviceDescriptors(C::mip_interface& device, uint8_t& descriptors_count, uint16_t* descriptors);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_built_in_test  Built in test
/// Run the device Built-In Test (BIT).
/// 
/// The Built-In Test command always returns a 32 bit value.
/// A value of 0 means that all tests passed.
/// A non-zero value indicates that not all tests passed.
/// Reference the device user manual to decode the result.
///
///@{

struct BuiltInTest
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::commands_base::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::commands_base::CMD_BUILT_IN_TEST;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mscl::commands_base::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mscl::commands_base::REPLY_BUILT_IN_TEST;
        
        uint32_t result;
        
    };
};
void insert(MipSerializer& serializer, const BuiltInTest& self);
void extract(MipSerializer& serializer, BuiltInTest& self);

void insert(MipSerializer& serializer, const BuiltInTest::Response& self);
void extract(MipSerializer& serializer, BuiltInTest::Response& self);

MipCmdResult builtInTest(C::mip_interface& device, uint32_t& result);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_resume  Resume
/// Take the device out of idle mode.
/// 
/// The device responds with ACK upon success.
///
///@{

struct Resume
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::commands_base::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::commands_base::CMD_RESUME;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    
};
void insert(MipSerializer& serializer, const Resume& self);
void extract(MipSerializer& serializer, Resume& self);

MipCmdResult resume(C::mip_interface& device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_get_extended_descriptors  Get device descriptors (extended)
/// Get the command and data descriptors supported by the device.
/// 
/// Reply has two fields: "ACK/NACK" and "Descriptors". The "Descriptors" field is an array of 16 bit values.
/// The MSB specifies the descriptor set and the LSB specifies the descriptor.
///
///@{

struct GetExtendedDescriptors
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::commands_base::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::commands_base::CMD_GET_EXTENDED_DESCRIPTORS;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mscl::commands_base::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mscl::commands_base::REPLY_GET_EXTENDED_DESCRIPTORS;
        
        uint8_t descriptors_count;
        uint16_t descriptors[253];
        
    };
};
void insert(MipSerializer& serializer, const GetExtendedDescriptors& self);
void extract(MipSerializer& serializer, GetExtendedDescriptors& self);

void insert(MipSerializer& serializer, const GetExtendedDescriptors::Response& self);
void extract(MipSerializer& serializer, GetExtendedDescriptors::Response& self);

MipCmdResult getExtendedDescriptors(C::mip_interface& device, uint8_t& descriptors_count, uint16_t* descriptors);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_continuous_bit  Continuous built-in test
/// Report result of continous built-in test.
/// 
/// This test is non-disruptive but is not as thorough as the commanded BIT.
///
///@{

struct ContinuousBit
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::commands_base::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::commands_base::CMD_CONTINUOUS_BIT;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mscl::commands_base::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mscl::commands_base::REPLY_CONTINUOUS_BIT;
        
        uint8_t result[16];
        
    };
};
void insert(MipSerializer& serializer, const ContinuousBit& self);
void extract(MipSerializer& serializer, ContinuousBit& self);

void insert(MipSerializer& serializer, const ContinuousBit::Response& self);
void extract(MipSerializer& serializer, ContinuousBit::Response& self);

MipCmdResult continuousBit(C::mip_interface& device, uint8_t* result);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_comm_speed  Comm Port Speed
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

struct CommSpeed
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::commands_base::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::commands_base::CMD_COMM_SPEED;
    
    static const bool HAS_WRITE_FUNCTION = true;
    static const bool HAS_READ_FUNCTION = true;
    static const bool HAS_SAVE_FUNCTION = true;
    static const bool HAS_LOAD_FUNCTION = true;
    static const bool HAS_RESET_FUNCTION = true;
    
    static const uint32_t ALL_PORTS = 0;
    MipFunctionSelector function;
    uint8_t port;
    uint32_t baud;
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mscl::commands_base::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mscl::commands_base::REPLY_COMM_SPEED;
        
        uint8_t port;
        uint32_t baud;
        
    };
};
void insert(MipSerializer& serializer, const CommSpeed& self);
void extract(MipSerializer& serializer, CommSpeed& self);

void insert(MipSerializer& serializer, const CommSpeed::Response& self);
void extract(MipSerializer& serializer, CommSpeed::Response& self);

MipCmdResult writeCommSpeed(C::mip_interface& device, uint8_t port, uint32_t baud);
MipCmdResult readCommSpeed(C::mip_interface& device, uint8_t port, uint32_t& baud);
MipCmdResult saveCommSpeed(C::mip_interface& device, uint8_t port);
MipCmdResult loadCommSpeed(C::mip_interface& device, uint8_t port);
MipCmdResult defaultCommSpeed(C::mip_interface& device, uint8_t port);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_gps_time_update  Time Broadcast Command
/// When combined with a PPS input signal applied to the I/O connector, this command enables complete synchronization of data outputs
/// with an external time base, such as GPS system time. Since the hardware PPS synchronization can only detect the fractional number of seconds when pulses arrive,
/// complete synchronization requires that the user provide the whole number of seconds via this command. After achieving PPS synchronization, this command should be sent twice: once to set the time-of-week and once to set the week number. PPS synchronization can be verified by monitoring the time sync status message (0xA0, 0x02) or the valid flags of any shared external timestamp (0x--, D7) data field.
///
///@{

struct GpsTimeUpdate
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::commands_base::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::commands_base::CMD_GPS_TIME_BROADCAST_NEW;
    
    static const bool HAS_WRITE_FUNCTION = true;
    static const bool HAS_READ_FUNCTION = false;
    static const bool HAS_SAVE_FUNCTION = false;
    static const bool HAS_LOAD_FUNCTION = false;
    static const bool HAS_RESET_FUNCTION = false;
    
    enum class FieldId : uint8_t
    {
        WEEK_NUMBER  = 1,  ///<  Week number.
        TIME_OF_WEEK = 2,  ///<  Time of week in seconds.
    };
    
    MipFunctionSelector function;
    FieldId field_id;
    uint32_t value;
    
};
void insert(MipSerializer& serializer, const GpsTimeUpdate& self);
void extract(MipSerializer& serializer, GpsTimeUpdate& self);

MipCmdResult writeGpsTimeUpdate(C::mip_interface& device, GpsTimeUpdate::FieldId field_id, uint32_t value);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_soft_reset  Reset device
/// Resets the device.
/// 
/// Device responds with ACK and immediately resets.
///
///@{

struct SoftReset
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::commands_base::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::commands_base::CMD_SOFT_RESET;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    
};
void insert(MipSerializer& serializer, const SoftReset& self);
void extract(MipSerializer& serializer, SoftReset& self);

MipCmdResult softReset(C::mip_interface& device);
///@}
///

///@}
///@}
///
////////////////////////////////////////////////////////////////////////////////
} // namespace commands_base
} // namespace mscl

