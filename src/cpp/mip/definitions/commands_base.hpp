#pragma once

#include <mip/definitions/common.hpp>
#include <mip/mip_descriptors.hpp>
#include <mip/mip_result.hpp>
#include <mip/mip_interface.hpp>

#include <stdint.h>
#include <stddef.h>

namespace mip {
namespace C {
struct mip_interface;
} // namespace C

namespace commands_base {

////////////////////////////////////////////////////////////////////////////////
///@addtogroup MipCommands_cpp
///@{
///@defgroup base_commands_cpp  Base Commands
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
    CMD_GPS_TIME_UPDATE            = 0x72,
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
    /// Parameters
    uint16_t firmware_version = 0;
    char model_name[16] = {0};
    char model_number[16] = {0};
    char serial_number[16] = {0};
    char lot_number[16] = {0};
    char device_options[16] = {0};
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};
enum class TimeFormat : uint8_t
{
    GPS = 1,  ///<  GPS time, a = week number since 1980, b = time of week in milliseconds.
};

struct CommandedTestBitsGq7 : Bitfield<CommandedTestBitsGq7>
{
    typedef uint32_t Type;
    enum _enumType : uint32_t
    {
        NONE                   = 0x00000000,
        GENERAL_HARDWARE_FAULT = 0x00000001,  ///<  
        GENERAL_FIRMWARE_FAULT = 0x00000002,  ///<  
        TIMING_OVERLOAD        = 0x00000004,  ///<  
        BUFFER_OVERRUN         = 0x00000008,  ///<  
        RESERVED               = 0x000000F0,  ///<  
        IPC_IMU_FAULT          = 0x00000100,  ///<  
        IPC_NAV_FAULT          = 0x00000200,  ///<  
        IPC_GNSS_FAULT         = 0x00000400,  ///<  
        COMMS_FAULT            = 0x00000800,  ///<  
        IMU_ACCEL_FAULT        = 0x00001000,  ///<  
        IMU_GYRO_FAULT         = 0x00002000,  ///<  
        IMU_MAG_FAULT          = 0x00004000,  ///<  
        IMU_PRESS_FAULT        = 0x00008000,  ///<  
        IMU_RESERVED           = 0x00030000,  ///<  
        IMU_CAL_ERROR          = 0x00040000,  ///<  
        IMU_GENERAL_FAULT      = 0x00080000,  ///<  
        FILT_RESERVED          = 0x00300000,  ///<  
        FILT_SOLUTION_FAULT    = 0x00400000,  ///<  
        FILT_GENERAL_FAULT     = 0x00800000,  ///<  
        GNSS_RECEIVER1_FAULT   = 0x01000000,  ///<  
        GNSS_ANTENNA1_FAULT    = 0x02000000,  ///<  
        GNSS_RECEIVER2_FAULT   = 0x04000000,  ///<  
        GNSS_ANTENNA2_FAULT    = 0x08000000,  ///<  
        GNSS_RTCM_FAILURE      = 0x10000000,  ///<  
        GNSS_RTK_FAULT         = 0x20000000,  ///<  
        GNSS_SOLUTION_FAULT    = 0x40000000,  ///<  
        GNSS_GENERAL_FAULT     = 0x80000000,  ///<  
        ALL                    = 0xFFFFFFFF,
    };
    uint32_t value = NONE;
    
    constexpr CommandedTestBitsGq7() : value(NONE) {}
    constexpr CommandedTestBitsGq7(int val) : value((uint32_t)val) {}
    constexpr operator uint32_t() const { return value; }
    constexpr CommandedTestBitsGq7& operator=(uint32_t val) { value = val; return *this; }
    constexpr CommandedTestBitsGq7& operator=(int val) { value = uint32_t(val); return *this; }
    constexpr CommandedTestBitsGq7& operator|=(uint32_t val) { return *this = value | val; }
    constexpr CommandedTestBitsGq7& operator&=(uint32_t val) { return *this = value & val; }
    
    constexpr bool generalHardwareFault() const { return (value & GENERAL_HARDWARE_FAULT) > 0; }
    constexpr void generalHardwareFault(bool val) { value &= ~GENERAL_HARDWARE_FAULT; if(val) value |= GENERAL_HARDWARE_FAULT; }
    constexpr bool generalFirmwareFault() const { return (value & GENERAL_FIRMWARE_FAULT) > 0; }
    constexpr void generalFirmwareFault(bool val) { value &= ~GENERAL_FIRMWARE_FAULT; if(val) value |= GENERAL_FIRMWARE_FAULT; }
    constexpr bool timingOverload() const { return (value & TIMING_OVERLOAD) > 0; }
    constexpr void timingOverload(bool val) { value &= ~TIMING_OVERLOAD; if(val) value |= TIMING_OVERLOAD; }
    constexpr bool bufferOverrun() const { return (value & BUFFER_OVERRUN) > 0; }
    constexpr void bufferOverrun(bool val) { value &= ~BUFFER_OVERRUN; if(val) value |= BUFFER_OVERRUN; }
    constexpr uint32_t reserved() const { return (value & RESERVED) >> 4; }
    constexpr void reserved(uint32_t val) { value = (value & ~RESERVED) | (val << 4); }
    constexpr bool ipcImuFault() const { return (value & IPC_IMU_FAULT) > 0; }
    constexpr void ipcImuFault(bool val) { value &= ~IPC_IMU_FAULT; if(val) value |= IPC_IMU_FAULT; }
    constexpr bool ipcNavFault() const { return (value & IPC_NAV_FAULT) > 0; }
    constexpr void ipcNavFault(bool val) { value &= ~IPC_NAV_FAULT; if(val) value |= IPC_NAV_FAULT; }
    constexpr bool ipcGnssFault() const { return (value & IPC_GNSS_FAULT) > 0; }
    constexpr void ipcGnssFault(bool val) { value &= ~IPC_GNSS_FAULT; if(val) value |= IPC_GNSS_FAULT; }
    constexpr bool commsFault() const { return (value & COMMS_FAULT) > 0; }
    constexpr void commsFault(bool val) { value &= ~COMMS_FAULT; if(val) value |= COMMS_FAULT; }
    constexpr bool imuAccelFault() const { return (value & IMU_ACCEL_FAULT) > 0; }
    constexpr void imuAccelFault(bool val) { value &= ~IMU_ACCEL_FAULT; if(val) value |= IMU_ACCEL_FAULT; }
    constexpr bool imuGyroFault() const { return (value & IMU_GYRO_FAULT) > 0; }
    constexpr void imuGyroFault(bool val) { value &= ~IMU_GYRO_FAULT; if(val) value |= IMU_GYRO_FAULT; }
    constexpr bool imuMagFault() const { return (value & IMU_MAG_FAULT) > 0; }
    constexpr void imuMagFault(bool val) { value &= ~IMU_MAG_FAULT; if(val) value |= IMU_MAG_FAULT; }
    constexpr bool imuPressFault() const { return (value & IMU_PRESS_FAULT) > 0; }
    constexpr void imuPressFault(bool val) { value &= ~IMU_PRESS_FAULT; if(val) value |= IMU_PRESS_FAULT; }
    constexpr uint32_t imuReserved() const { return (value & IMU_RESERVED) >> 16; }
    constexpr void imuReserved(uint32_t val) { value = (value & ~IMU_RESERVED) | (val << 16); }
    constexpr bool imuCalError() const { return (value & IMU_CAL_ERROR) > 0; }
    constexpr void imuCalError(bool val) { value &= ~IMU_CAL_ERROR; if(val) value |= IMU_CAL_ERROR; }
    constexpr bool imuGeneralFault() const { return (value & IMU_GENERAL_FAULT) > 0; }
    constexpr void imuGeneralFault(bool val) { value &= ~IMU_GENERAL_FAULT; if(val) value |= IMU_GENERAL_FAULT; }
    constexpr uint32_t filtReserved() const { return (value & FILT_RESERVED) >> 20; }
    constexpr void filtReserved(uint32_t val) { value = (value & ~FILT_RESERVED) | (val << 20); }
    constexpr bool filtSolutionFault() const { return (value & FILT_SOLUTION_FAULT) > 0; }
    constexpr void filtSolutionFault(bool val) { value &= ~FILT_SOLUTION_FAULT; if(val) value |= FILT_SOLUTION_FAULT; }
    constexpr bool filtGeneralFault() const { return (value & FILT_GENERAL_FAULT) > 0; }
    constexpr void filtGeneralFault(bool val) { value &= ~FILT_GENERAL_FAULT; if(val) value |= FILT_GENERAL_FAULT; }
    constexpr bool gnssReceiver1Fault() const { return (value & GNSS_RECEIVER1_FAULT) > 0; }
    constexpr void gnssReceiver1Fault(bool val) { value &= ~GNSS_RECEIVER1_FAULT; if(val) value |= GNSS_RECEIVER1_FAULT; }
    constexpr bool gnssAntenna1Fault() const { return (value & GNSS_ANTENNA1_FAULT) > 0; }
    constexpr void gnssAntenna1Fault(bool val) { value &= ~GNSS_ANTENNA1_FAULT; if(val) value |= GNSS_ANTENNA1_FAULT; }
    constexpr bool gnssReceiver2Fault() const { return (value & GNSS_RECEIVER2_FAULT) > 0; }
    constexpr void gnssReceiver2Fault(bool val) { value &= ~GNSS_RECEIVER2_FAULT; if(val) value |= GNSS_RECEIVER2_FAULT; }
    constexpr bool gnssAntenna2Fault() const { return (value & GNSS_ANTENNA2_FAULT) > 0; }
    constexpr void gnssAntenna2Fault(bool val) { value &= ~GNSS_ANTENNA2_FAULT; if(val) value |= GNSS_ANTENNA2_FAULT; }
    constexpr bool gnssRtcmFailure() const { return (value & GNSS_RTCM_FAILURE) > 0; }
    constexpr void gnssRtcmFailure(bool val) { value &= ~GNSS_RTCM_FAILURE; if(val) value |= GNSS_RTCM_FAILURE; }
    constexpr bool gnssRtkFault() const { return (value & GNSS_RTK_FAULT) > 0; }
    constexpr void gnssRtkFault(bool val) { value &= ~GNSS_RTK_FAULT; if(val) value |= GNSS_RTK_FAULT; }
    constexpr bool gnssSolutionFault() const { return (value & GNSS_SOLUTION_FAULT) > 0; }
    constexpr void gnssSolutionFault(bool val) { value &= ~GNSS_SOLUTION_FAULT; if(val) value |= GNSS_SOLUTION_FAULT; }
    constexpr bool gnssGeneralFault() const { return (value & GNSS_GENERAL_FAULT) > 0; }
    constexpr void gnssGeneralFault(bool val) { value &= ~GNSS_GENERAL_FAULT; if(val) value |= GNSS_GENERAL_FAULT; }
    constexpr bool allSet() const { return value == ALL; }
    constexpr void setAll() { value |= ALL; }
};

////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
///@defgroup base_ping_cpp  (0x01,0x01) Ping
/// Test Communications with a device.
/// 
/// The Device will respond with an ACK, if present and operating correctly.
/// 
/// If the device is not in a normal operating mode, it may NACK.
///
///@{

struct Ping
{
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_base::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_base::CMD_PING;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "Ping";
    static constexpr const char* DOC_NAME = "Ping";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple();
    }
    
    auto asTuple()
    {
        return std::make_tuple();
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
    typedef void Response;
};
TypedResult<Ping> ping(C::mip_interface& device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup base_set_idle_cpp  (0x01,0x02) Set Idle
/// Turn off all device data streams.
/// 
/// The Device will respond with an ACK, if present and operating correctly.
/// This command will suspend streaming (if enabled) or wake the device from sleep (if sleeping) to allow it to respond to status and setup commands.
/// You may restore the device mode by issuing the Resume command.
///
///@{

struct SetIdle
{
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_base::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_base::CMD_SET_TO_IDLE;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "SetIdle";
    static constexpr const char* DOC_NAME = "Set to idle";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple();
    }
    
    auto asTuple()
    {
        return std::make_tuple();
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
    typedef void Response;
};
TypedResult<SetIdle> setIdle(C::mip_interface& device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup base_get_device_info_cpp  (0x01,0x03) Get Device Info
/// Get the device ID strings and firmware version number.
///
///@{

struct GetDeviceInfo
{
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_base::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_base::CMD_GET_DEVICE_INFO;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "GetDeviceInfo";
    static constexpr const char* DOC_NAME = "Get device information";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple();
    }
    
    auto asTuple()
    {
        return std::make_tuple();
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
    struct Response
    {
        /// Parameters
        BaseDeviceInfo device_info;
        
        /// Descriptors
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_base::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_base::REPLY_DEVICE_INFO;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "GetDeviceInfo::Response";
        static constexpr const char* DOC_NAME = "Get device information Response";
        static constexpr const bool HAS_FUNCTION_SELECTOR = false;
        
        auto asTuple() const
        {
            return std::make_tuple(device_info);
        }
        
        auto asTuple()
        {
            return std::make_tuple(std::ref(device_info));
        }
        
        /// Serialization
        void insert(Serializer& serializer) const;
        void extract(Serializer& serializer);
        
    };
};
TypedResult<GetDeviceInfo> getDeviceInfo(C::mip_interface& device, BaseDeviceInfo* deviceInfoOut);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup base_get_device_descriptors_cpp  (0x01,0x04) Get Device Descriptors
/// Get the command and data descriptors supported by the device.
/// 
/// Reply has two fields: "ACK/NACK" and "Descriptors". The "Descriptors" field is an array of 16 bit values.
/// The MSB specifies the descriptor set and the LSB specifies the descriptor.
///
///@{

struct GetDeviceDescriptors
{
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_base::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_base::CMD_GET_DEVICE_DESCRIPTORS;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "GetDeviceDescriptors";
    static constexpr const char* DOC_NAME = "Get device descriptors";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple();
    }
    
    auto asTuple()
    {
        return std::make_tuple();
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
    struct Response
    {
        /// Parameters
        uint16_t descriptors[253] = {0};
        uint8_t descriptors_count = 0;
        
        /// Descriptors
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_base::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_base::REPLY_DEVICE_DESCRIPTORS;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "GetDeviceDescriptors::Response";
        static constexpr const char* DOC_NAME = "Get device descriptors Response";
        static constexpr const bool HAS_FUNCTION_SELECTOR = false;
        
        auto asTuple() const
        {
            return std::make_tuple(descriptors,descriptors_count);
        }
        
        auto asTuple()
        {
            return std::make_tuple(std::ref(descriptors),std::ref(descriptors_count));
        }
        
        /// Serialization
        void insert(Serializer& serializer) const;
        void extract(Serializer& serializer);
        
    };
};
TypedResult<GetDeviceDescriptors> getDeviceDescriptors(C::mip_interface& device, uint16_t* descriptorsOut, size_t descriptorsOutMax, uint8_t* descriptorsOutCount);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup base_built_in_test_cpp  (0x01,0x05) Built In Test
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
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_base::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_base::CMD_BUILT_IN_TEST;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "BuiltInTest";
    static constexpr const char* DOC_NAME = "Built in test";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple();
    }
    
    auto asTuple()
    {
        return std::make_tuple();
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
    struct Response
    {
        /// Parameters
        uint32_t result = 0;
        
        /// Descriptors
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_base::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_base::REPLY_BUILT_IN_TEST;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "BuiltInTest::Response";
        static constexpr const char* DOC_NAME = "Built in test Response";
        static constexpr const bool HAS_FUNCTION_SELECTOR = false;
        
        auto asTuple() const
        {
            return std::make_tuple(result);
        }
        
        auto asTuple()
        {
            return std::make_tuple(std::ref(result));
        }
        
        /// Serialization
        void insert(Serializer& serializer) const;
        void extract(Serializer& serializer);
        
    };
};
TypedResult<BuiltInTest> builtInTest(C::mip_interface& device, uint32_t* resultOut);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup base_resume_cpp  (0x01,0x06) Resume
/// Take the device out of idle mode.
/// 
/// The device responds with ACK upon success.
///
///@{

struct Resume
{
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_base::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_base::CMD_RESUME;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "Resume";
    static constexpr const char* DOC_NAME = "Resume";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple();
    }
    
    auto asTuple()
    {
        return std::make_tuple();
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
    typedef void Response;
};
TypedResult<Resume> resume(C::mip_interface& device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup base_get_extended_descriptors_cpp  (0x01,0x07) Get Extended Descriptors
/// Get the command and data descriptors supported by the device.
/// 
/// Reply has two fields: "ACK/NACK" and "Descriptors". The "Descriptors" field is an array of 16 bit values.
/// The MSB specifies the descriptor set and the LSB specifies the descriptor.
///
///@{

struct GetExtendedDescriptors
{
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_base::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_base::CMD_GET_EXTENDED_DESCRIPTORS;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "GetExtendedDescriptors";
    static constexpr const char* DOC_NAME = "Get device descriptors (extended)";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple();
    }
    
    auto asTuple()
    {
        return std::make_tuple();
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
    struct Response
    {
        /// Parameters
        uint16_t descriptors[253] = {0};
        uint8_t descriptors_count = 0;
        
        /// Descriptors
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_base::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_base::REPLY_GET_EXTENDED_DESCRIPTORS;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "GetExtendedDescriptors::Response";
        static constexpr const char* DOC_NAME = "Get device descriptors (extended) Response";
        static constexpr const bool HAS_FUNCTION_SELECTOR = false;
        
        auto asTuple() const
        {
            return std::make_tuple(descriptors,descriptors_count);
        }
        
        auto asTuple()
        {
            return std::make_tuple(std::ref(descriptors),std::ref(descriptors_count));
        }
        
        /// Serialization
        void insert(Serializer& serializer) const;
        void extract(Serializer& serializer);
        
    };
};
TypedResult<GetExtendedDescriptors> getExtendedDescriptors(C::mip_interface& device, uint16_t* descriptorsOut, size_t descriptorsOutMax, uint8_t* descriptorsOutCount);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup base_continuous_bit_cpp  (0x01,0x08) Continuous Bit
/// Report result of continuous built-in test.
/// 
/// This test is non-disruptive but is not as thorough as the commanded BIT.
///
///@{

struct ContinuousBit
{
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_base::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_base::CMD_CONTINUOUS_BIT;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "ContinuousBit";
    static constexpr const char* DOC_NAME = "Continuous built-in test";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple();
    }
    
    auto asTuple()
    {
        return std::make_tuple();
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
    struct Response
    {
        /// Parameters
        uint8_t result[16] = {0}; ///< Device-specific bitfield (128 bits). See device user manual. Bits are least-significant-byte first. For example, bit 0 is located at bit 0 of result[0], bit 1 is located at bit 1 of result[0], bit 8 is located at bit 0 of result[1], and bit 127 is located at bit 7 of result[15].
        
        /// Descriptors
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_base::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_base::REPLY_CONTINUOUS_BIT;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "ContinuousBit::Response";
        static constexpr const char* DOC_NAME = "Continuous built-in test Response";
        static constexpr const bool HAS_FUNCTION_SELECTOR = false;
        
        auto asTuple() const
        {
            return std::make_tuple(result);
        }
        
        auto asTuple()
        {
            return std::make_tuple(std::ref(result));
        }
        
        /// Serialization
        void insert(Serializer& serializer) const;
        void extract(Serializer& serializer);
        
    };
};
TypedResult<ContinuousBit> continuousBit(C::mip_interface& device, uint8_t* resultOut);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup base_comm_speed_cpp  (0x01,0x09) Comm Speed
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
    static constexpr const uint32_t ALL_PORTS = 0;
    /// Parameters
    FunctionSelector function = static_cast<FunctionSelector>(0);
    uint8_t port = 0; ///< Port ID number, starting with 1. When function is SAVE, LOAD, or DEFAULT, this can be 0 to apply to all ports. See the device user manual for details.
    uint32_t baud = 0; ///< Port baud rate. Must be a supported rate.
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_base::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_base::CMD_COMM_SPEED;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "CommSpeed";
    static constexpr const char* DOC_NAME = "Comm Port Speed";
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    
    auto asTuple() const
    {
        return std::make_tuple(port,baud);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(port),std::ref(baud));
    }
    
    static CommSpeed create_sld_all(::mip::FunctionSelector function)
    {
        CommSpeed cmd;
        cmd.function = function;
        cmd.port = 0;
        return cmd;
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
    struct Response
    {
        /// Parameters
        uint8_t port = 0; ///< Port ID number, starting with 1. When function is SAVE, LOAD, or DEFAULT, this can be 0 to apply to all ports. See the device user manual for details.
        uint32_t baud = 0; ///< Port baud rate. Must be a supported rate.
        
        /// Descriptors
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_base::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_base::REPLY_COMM_SPEED;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "CommSpeed::Response";
        static constexpr const char* DOC_NAME = "Comm Port Speed Response";
        static constexpr const bool HAS_FUNCTION_SELECTOR = false;
        
        auto asTuple() const
        {
            return std::make_tuple(port,baud);
        }
        
        auto asTuple()
        {
            return std::make_tuple(std::ref(port),std::ref(baud));
        }
        
        /// Serialization
        void insert(Serializer& serializer) const;
        void extract(Serializer& serializer);
        
    };
};
TypedResult<CommSpeed> writeCommSpeed(C::mip_interface& device, uint8_t port, uint32_t baud);
TypedResult<CommSpeed> readCommSpeed(C::mip_interface& device, uint8_t port, uint32_t* baudOut);
TypedResult<CommSpeed> saveCommSpeed(C::mip_interface& device, uint8_t port);
TypedResult<CommSpeed> loadCommSpeed(C::mip_interface& device, uint8_t port);
TypedResult<CommSpeed> defaultCommSpeed(C::mip_interface& device, uint8_t port);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup base_gps_time_update_cpp  (0x01,0x72) Gps Time Update
/// Set device internal GPS time
/// When combined with a PPS input signal applied to the I/O connector, this command enables complete synchronization of data outputs
/// with an external time base, such as GPS system time. Since the hardware PPS synchronization can only detect the fractional number of seconds when pulses arrive,
/// complete synchronization requires that the user provide the whole number of seconds via this command. After achieving PPS synchronization, this command should be sent twice: once to set the time-of-week and once to set the week number. PPS synchronization can be verified by monitoring the time sync status message (0xA0, 0x02) or the valid flags of any shared external timestamp (0x--, D7) data field.
///
///@{

struct GpsTimeUpdate
{
    enum class FieldId : uint8_t
    {
        WEEK_NUMBER  = 1,  ///<  Week number.
        TIME_OF_WEEK = 2,  ///<  Time of week in seconds.
    };
    
    /// Parameters
    FunctionSelector function = static_cast<FunctionSelector>(0);
    FieldId field_id = static_cast<FieldId>(0); ///< Determines how to interpret value.
    uint32_t value = 0; ///< Week number or time of week, depending on the field_id.
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_base::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_base::CMD_GPS_TIME_UPDATE;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "GpsTimeUpdate";
    static constexpr const char* DOC_NAME = "GPS Time Update Command";
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    
    auto asTuple() const
    {
        return std::make_tuple(field_id,value);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(field_id),std::ref(value));
    }
    
    static GpsTimeUpdate create_sld_all(::mip::FunctionSelector function)
    {
        GpsTimeUpdate cmd;
        cmd.function = function;
        return cmd;
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
    typedef void Response;
};
TypedResult<GpsTimeUpdate> writeGpsTimeUpdate(C::mip_interface& device, GpsTimeUpdate::FieldId fieldId, uint32_t value);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup base_soft_reset_cpp  (0x01,0x7E) Soft Reset
/// Resets the device.
/// 
/// Device responds with ACK and immediately resets.
///
///@{

struct SoftReset
{
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_base::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_base::CMD_SOFT_RESET;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "SoftReset";
    static constexpr const char* DOC_NAME = "Reset device";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple();
    }
    
    auto asTuple()
    {
        return std::make_tuple();
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
    typedef void Response;
};
TypedResult<SoftReset> softReset(C::mip_interface& device);

///@}
///

///@}
///@}
///
////////////////////////////////////////////////////////////////////////////////
} // namespace commands_base
} // namespace mip

