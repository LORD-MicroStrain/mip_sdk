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

namespace commands_3dm {

////////////////////////////////////////////////////////////////////////////////
///@addtogroup MipCommands_cpp
///@{
///@defgroup 3dm_commands_cpp  3dm Commands
///
///@{

////////////////////////////////////////////////////////////////////////////////
// Descriptors
////////////////////////////////////////////////////////////////////////////////

enum 
{
    DESCRIPTOR_SET                      = 0x0C,
    
    CMD_POLL_IMU_MESSAGE                = 0x01,
    CMD_POLL_GNSS_MESSAGE               = 0x02,
    CMD_POLL_FILTER_MESSAGE             = 0x03,
    CMD_POLL_NMEA_MESSAGE               = 0x04,
    CMD_GET_IMU_BASE_RATE               = 0x06,
    CMD_GET_GNSS_BASE_RATE              = 0x07,
    CMD_IMU_MESSAGE_FORMAT              = 0x08,
    CMD_GNSS_MESSAGE_FORMAT             = 0x09,
    CMD_FILTER_MESSAGE_FORMAT           = 0x0A,
    CMD_GET_FILTER_BASE_RATE            = 0x0B,
    CMD_NMEA_MESSAGE_FORMAT             = 0x0C,
    CMD_POLL_DATA                       = 0x0D,
    CMD_GET_BASE_RATE                   = 0x0E,
    CMD_MESSAGE_FORMAT                  = 0x0F,
    CMD_CONFIGURE_FACTORY_STREAMING     = 0x10,
    CMD_CONTROL_DATA_STREAM             = 0x11,
    CMD_GNSS_CONSTELLATION_SETTINGS     = 0x21,
    CMD_GNSS_SBAS_SETTINGS              = 0x22,
    CMD_GNSS_ASSISTED_FIX_SETTINGS      = 0x23,
    CMD_GNSS_TIME_ASSISTANCE            = 0x24,
    CMD_PPS_SOURCE                      = 0x28,
    CMD_EVENT_SUPPORT                   = 0x2A,
    CMD_EVENT_CONTROL                   = 0x2B,
    CMD_EVENT_TRIGGER_STATUS            = 0x2C,
    CMD_EVENT_ACTION_STATUS             = 0x2D,
    CMD_EVENT_TRIGGER_CONFIG            = 0x2E,
    CMD_EVENT_ACTION_CONFIG             = 0x2F,
    CMD_DEVICE_STARTUP_SETTINGS         = 0x30,
    CMD_SENSOR2VEHICLE_TRANSFORM_EUL    = 0x31,
    CMD_SENSOR2VEHICLE_TRANSFORM_QUAT   = 0x32,
    CMD_SENSOR2VEHICLE_TRANSFORM_DCM    = 0x33,
    CMD_ACCEL_BIAS                      = 0x37,
    CMD_GYRO_BIAS                       = 0x38,
    CMD_CAPTURE_GYRO_BIAS               = 0x39,
    CMD_HARD_IRON_OFFSET                = 0x3A,
    CMD_SOFT_IRON_MATRIX                = 0x3B,
    CMD_CONING_AND_SCULLING_ENABLE      = 0x3E,
    CMD_UART_BAUDRATE                   = 0x40,
    CMD_GPIO_CONFIG                     = 0x41,
    CMD_GPIO_STATE                      = 0x42,
    CMD_ODOMETER_CONFIG                 = 0x43,
    CMD_IMU_LOWPASS_FILTER              = 0x50,
    CMD_LEGACY_COMP_FILTER              = 0x51,
    CMD_SENSOR_RANGE                    = 0x52,
    CMD_CALIBRATED_RANGES               = 0x53,
    CMD_LOWPASS_FILTER                  = 0x54,
    
    REPLY_IMU_MESSAGE_FORMAT            = 0x80,
    REPLY_GNSS_MESSAGE_FORMAT           = 0x81,
    REPLY_FILTER_MESSAGE_FORMAT         = 0x82,
    REPLY_IMU_BASE_RATE                 = 0x83,
    REPLY_GNSS_BASE_RATE                = 0x84,
    REPLY_DATASTREAM_ENABLE             = 0x85,
    REPLY_UART_BAUDRATE                 = 0x87,
    REPLY_FILTER_BASE_RATE              = 0x8A,
    REPLY_ADVANCED_DATA_FILTER          = 0x8B,
    REPLY_POLL_DATA                     = 0x8D,
    REPLY_BASE_RATE                     = 0x8E,
    REPLY_MESSAGE_FORMAT                = 0x8F,
    REPLY_LEGACY_COMP_FILTER            = 0x97,
    REPLY_ACCEL_BIAS_VECTOR             = 0x9A,
    REPLY_GYRO_BIAS_VECTOR              = 0x9B,
    REPLY_HARD_IRON_OFFSET_VECTOR       = 0x9C,
    REPLY_SOFT_IRON_COMP_MATRIX         = 0x9D,
    REPLY_CONING_AND_SCULLING_ENABLE    = 0x9E,
    REPLY_GNSS_CONSTELLATION_SETTINGS   = 0xA0,
    REPLY_GNSS_SBAS_SETTINGS            = 0xA1,
    REPLY_GNSS_ASSISTED_FIX_SETTINGS    = 0xA2,
    REPLY_GNSS_TIME_ASSISTANCE          = 0xA3,
    REPLY_SENSOR2VEHICLE_TRANSFORM_EUL  = 0xB1,
    REPLY_SENSOR2VEHICLE_TRANSFORM_QUAT = 0xB2,
    REPLY_SENSOR2VEHICLE_TRANSFORM_DCM  = 0xB3,
    REPLY_EVENT_SUPPORT                 = 0xB4,
    REPLY_EVENT_CONTROL                 = 0xB5,
    REPLY_EVENT_TRIGGER_STATUS          = 0xB6,
    REPLY_EVENT_ACTION_STATUS           = 0xB7,
    REPLY_EVENT_TRIGGER_CONFIG          = 0xB8,
    REPLY_EVENT_ACTION_CONFIG           = 0xB9,
    REPLY_NMEA_MESSAGE_FORMAT           = 0x8C,
    REPLY_PPS_SOURCE                    = 0xA8,
    REPLY_GPIO_CONFIG                   = 0xC1,
    REPLY_GPIO_STATE                    = 0xC2,
    REPLY_ODOMETER_CONFIG               = 0xC3,
    REPLY_SENSOR_RANGE                  = 0xD2,
    REPLY_CALIBRATED_RANGES             = 0xD3,
    REPLY_LOWPASS_FILTER                = 0xD4,
};

////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////

struct NmeaMessage
{
    enum class MessageID : uint8_t
    {
        GGA  = 1,  ///<  GPS System Fix Data. Source can be the Filter or GNSS1/2 datasets.
        GLL  = 2,  ///<  Geographic Position Lat/Lon. Source can be the Filter or GNSS1/2 datasets.
        GSV  = 3,  ///<  GNSS Satellites in View. Source must be either GNSS1 or GNSS2 datasets. The talker ID must be set to IGNORED.
        RMC  = 4,  ///<  Recommended Minimum Specific GNSS Data. Source can be the Filter or GNSS1/2 datasets.
        VTG  = 5,  ///<  Course over Ground. Source can be the Filter or GNSS1/2 datasets.
        HDT  = 6,  ///<  Heading, True. Source can be the Filter or GNSS1/2 datasets.
        ZDA  = 7,  ///<  Time & Date. Source must be the GNSS1 or GNSS2 datasets.
        GST  = 8,  ///<  Position Error Statistics. Source can be the Filter or GNSS1/2 datasets.
        MSRA = 129,  ///<  MicroStrain proprietary Euler angles. Source must be the Filter dataset. The talker ID must be set to IGNORED.
        MSRR = 130,  ///<  MicroStrain proprietary Angular Rate/Acceleration. Source must be the Sensor dataset. The talker ID must be set to IGNORED.
    };
    
    enum class TalkerID : uint8_t
    {
        IGNORED = 0,  ///<  Talker ID cannot be changed.
        GNSS    = 1,  ///<  NMEA message will be produced with talker id "GN".
        GPS     = 2,  ///<  NMEA message will be produced with talker id "GP".
        GALILEO = 3,  ///<  NMEA message will be produced with talker id "GA".
        GLONASS = 4,  ///<  NMEA message will be produced with talker id "GL".
    };
    
    /// Parameters
    MessageID message_id = static_cast<MessageID>(0); ///< NMEA sentence type.
    TalkerID talker_id = static_cast<TalkerID>(0); ///< NMEA talker ID. Ignored for proprietary sentences.
    uint8_t source_desc_set = 0; ///< Data descriptor set where the data will be sourced. Available options depend on the sentence.
    uint16_t decimation = 0; ///< Decimation from the base rate for source_desc_set. Frequency is limited to 10 Hz or the base rate, whichever is lower. Must be 0 when polling.
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};
enum class SensorRangeType : uint8_t
{
    ALL   = 0,  ///<  Only allowed for SAVE, LOAD, and DEFAULT function selectors.
    ACCEL = 1,  ///<  Accelerometer. Range is specified in g.
    GYRO  = 2,  ///<  Gyroscope. Range is specified in degrees/s.
    MAG   = 3,  ///<  Magnetometer. Range is specified in Gauss.
    PRESS = 4,  ///<  Pressure sensor. Range is specified in hPa.
};


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
///@defgroup 3dm_poll_imu_message_cpp  (0x0C,0x01) Poll Imu Message
/// Poll the device for an IMU message with the specified format
/// 
/// This function polls for an IMU message using the provided format. The resulting message
/// will maintain the order of descriptors sent in the command and any unrecognized
/// descriptors are ignored. If the format is not provided, the device will attempt to use the
/// stored format (set with the Set IMU Message Format command.) If no format is provided
/// and there is no stored format, the device will respond with a NACK. The reply packet contains
/// an ACK/NACK field. The polled data packet is sent separately as an IMU Data packet.
///
///@{

struct PollImuMessage
{
    /// Parameters
    bool suppress_ack = 0; ///< Suppress the usual ACK/NACK reply.
    uint8_t num_descriptors = 0; ///< Number of descriptors in the descriptor list.
    DescriptorRate descriptors[83]; ///< Descriptor list.
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_POLL_IMU_MESSAGE;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "PollImuMessage";
    static constexpr const char* DOC_NAME = "Poll IMU Message";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(suppress_ack,num_descriptors,descriptors);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(suppress_ack),std::ref(num_descriptors),std::ref(descriptors));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
    typedef void Response;
};
TypedResult<PollImuMessage> pollImuMessage(C::mip_interface& device, bool suppressAck, uint8_t numDescriptors, const DescriptorRate* descriptors);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup 3dm_poll_gnss_message_cpp  (0x0C,0x02) Poll Gnss Message
/// Poll the device for an GNSS message with the specified format
/// 
/// This function polls for a GNSS message using the provided format. The resulting message
/// will maintain the order of descriptors sent in the command and any unrecognized
/// descriptors are ignored. If the format is not provided, the device will attempt to use the
/// stored format (set with the Set GNSS Message Format command.) If no format is provided
/// and there is no stored format, the device will respond with a NACK. The reply packet contains
/// an ACK/NACK field. The polled data packet is sent separately as a GNSS Data packet.
///
///@{

struct PollGnssMessage
{
    /// Parameters
    bool suppress_ack = 0; ///< Suppress the usual ACK/NACK reply.
    uint8_t num_descriptors = 0; ///< Number of descriptors in the descriptor list.
    DescriptorRate descriptors[83]; ///< Descriptor list.
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_POLL_GNSS_MESSAGE;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "PollGnssMessage";
    static constexpr const char* DOC_NAME = "Poll GNSS Message";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(suppress_ack,num_descriptors,descriptors);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(suppress_ack),std::ref(num_descriptors),std::ref(descriptors));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
    typedef void Response;
};
TypedResult<PollGnssMessage> pollGnssMessage(C::mip_interface& device, bool suppressAck, uint8_t numDescriptors, const DescriptorRate* descriptors);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup 3dm_poll_filter_message_cpp  (0x0C,0x03) Poll Filter Message
/// Poll the device for an Estimation Filter message with the specified format
/// 
/// This function polls for an Estimation Filter message using the provided format. The resulting message
/// will maintain the order of descriptors sent in the command and any unrecognized
/// descriptors are ignored. If the format is not provided, the device will attempt to use the
/// stored format (set with the Set Estimation Filter Message Format command.) If no format is provided
/// and there is no stored format, the device will respond with a NACK. The reply packet contains
/// an ACK/NACK field. The polled data packet is sent separately as an Estimation Filter Data packet.
///
///@{

struct PollFilterMessage
{
    /// Parameters
    bool suppress_ack = 0; ///< Suppress the usual ACK/NACK reply.
    uint8_t num_descriptors = 0; ///< Number of descriptors in the format list.
    DescriptorRate descriptors[83]; ///< Descriptor format list.
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_POLL_FILTER_MESSAGE;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "PollFilterMessage";
    static constexpr const char* DOC_NAME = "Poll Estimation Filter Message";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(suppress_ack,num_descriptors,descriptors);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(suppress_ack),std::ref(num_descriptors),std::ref(descriptors));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
    typedef void Response;
};
TypedResult<PollFilterMessage> pollFilterMessage(C::mip_interface& device, bool suppressAck, uint8_t numDescriptors, const DescriptorRate* descriptors);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup 3dm_nmea_poll_data_cpp  (0x0C,0x04) Nmea Poll Data
/// Poll the device for a NMEA message with the specified format.
/// 
/// This function polls for a NMEA message using the provided format.
/// If the format is not provided, the device will attempt to use the
/// stored format (set with the Set NMEA Message Format command.) If no format is provided
/// and there is no stored format, the device will respond with a NACK. The reply packet contains
/// an ACK/NACK field. The polled data packet is sent separately as normal NMEA messages.
///
///@{

struct NmeaPollData
{
    /// Parameters
    bool suppress_ack = 0; ///< Suppress the usual ACK/NACK reply.
    uint8_t count = 0; ///< Number of format entries (limited by payload size)
    NmeaMessage format_entries[40]; ///< List of format entries.
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_POLL_NMEA_MESSAGE;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "NmeaPollData";
    static constexpr const char* DOC_NAME = "Poll NMEA Data";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(suppress_ack,count,format_entries);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(suppress_ack),std::ref(count),std::ref(format_entries));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
    typedef void Response;
};
TypedResult<NmeaPollData> nmeaPollData(C::mip_interface& device, bool suppressAck, uint8_t count, const NmeaMessage* formatEntries);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup 3dm_imu_get_base_rate_cpp  (0x0C,0x06) Imu Get Base Rate
/// Get the base rate for the IMU data in Hz
/// 
/// This is the fastest rate for this type of data available on the device.
/// This is used in conjunction with the IMU Message Format Command to set streaming data at a specified rate.
///
///@{

struct ImuGetBaseRate
{
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_GET_IMU_BASE_RATE;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "ImuGetBaseRate";
    static constexpr const char* DOC_NAME = "Get IMU Data Base Rate";
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
        uint16_t rate = 0; ///< [hz]
        
        /// Descriptors
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::REPLY_IMU_BASE_RATE;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "ImuGetBaseRate::Response";
        static constexpr const char* DOC_NAME = "Get IMU Data Base Rate Response";
        static constexpr const bool HAS_FUNCTION_SELECTOR = false;
        
        auto asTuple() const
        {
            return std::make_tuple(rate);
        }
        
        auto asTuple()
        {
            return std::make_tuple(std::ref(rate));
        }
        
        /// Serialization
        void insert(Serializer& serializer) const;
        void extract(Serializer& serializer);
        
    };
};
TypedResult<ImuGetBaseRate> imuGetBaseRate(C::mip_interface& device, uint16_t* rateOut);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup 3dm_gnss_get_base_rate_cpp  (0x0C,0x07) Gnss Get Base Rate
/// Get the base rate for the GNSS data in Hz
/// 
/// This is the fastest rate for this type of data available on the device.
/// This is used in conjunction with the GNSS Message Format Command to set streaming data at a specified rate.
///
///@{

struct GnssGetBaseRate
{
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_GET_GNSS_BASE_RATE;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "GnssGetBaseRate";
    static constexpr const char* DOC_NAME = "Get GNSS Data Base Rate";
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
        uint16_t rate = 0; ///< [hz]
        
        /// Descriptors
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::REPLY_GNSS_BASE_RATE;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "GnssGetBaseRate::Response";
        static constexpr const char* DOC_NAME = "Get GNSS Data Base Rate Response";
        static constexpr const bool HAS_FUNCTION_SELECTOR = false;
        
        auto asTuple() const
        {
            return std::make_tuple(rate);
        }
        
        auto asTuple()
        {
            return std::make_tuple(std::ref(rate));
        }
        
        /// Serialization
        void insert(Serializer& serializer) const;
        void extract(Serializer& serializer);
        
    };
};
TypedResult<GnssGetBaseRate> gnssGetBaseRate(C::mip_interface& device, uint16_t* rateOut);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup 3dm_imu_message_format_cpp  (0x0C,0x08) Imu Message Format
/// Set, read, or save the format of the IMU data packet.
/// 
/// The resulting data messages will maintain the order of descriptors sent in the command.
///
///@{

struct ImuMessageFormat
{
    /// Parameters
    FunctionSelector function = static_cast<FunctionSelector>(0);
    uint8_t num_descriptors = 0; ///< Number of descriptors
    DescriptorRate descriptors[82]; ///< Descriptor format list.
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_IMU_MESSAGE_FORMAT;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "ImuMessageFormat";
    static constexpr const char* DOC_NAME = "IMU Message Format";
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    
    auto asTuple() const
    {
        return std::make_tuple(num_descriptors,descriptors);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(num_descriptors),std::ref(descriptors));
    }
    
    static ImuMessageFormat create_sld_all(::mip::FunctionSelector function)
    {
        ImuMessageFormat cmd;
        cmd.function = function;
        return cmd;
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
    struct Response
    {
        /// Parameters
        uint8_t num_descriptors = 0; ///< Number of descriptors
        DescriptorRate descriptors[82]; ///< Descriptor format list.
        
        /// Descriptors
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::REPLY_IMU_MESSAGE_FORMAT;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "ImuMessageFormat::Response";
        static constexpr const char* DOC_NAME = "IMU Message Format Response";
        static constexpr const bool HAS_FUNCTION_SELECTOR = false;
        
        auto asTuple() const
        {
            return std::make_tuple(num_descriptors,descriptors);
        }
        
        auto asTuple()
        {
            return std::make_tuple(std::ref(num_descriptors),std::ref(descriptors));
        }
        
        /// Serialization
        void insert(Serializer& serializer) const;
        void extract(Serializer& serializer);
        
    };
};
TypedResult<ImuMessageFormat> writeImuMessageFormat(C::mip_interface& device, uint8_t numDescriptors, const DescriptorRate* descriptors);
TypedResult<ImuMessageFormat> readImuMessageFormat(C::mip_interface& device, uint8_t* numDescriptorsOut, uint8_t numDescriptorsOutMax, DescriptorRate* descriptorsOut);
TypedResult<ImuMessageFormat> saveImuMessageFormat(C::mip_interface& device);
TypedResult<ImuMessageFormat> loadImuMessageFormat(C::mip_interface& device);
TypedResult<ImuMessageFormat> defaultImuMessageFormat(C::mip_interface& device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup 3dm_gnss_message_format_cpp  (0x0C,0x09) Gnss Message Format
/// Set, read, or save the format of the GNSS data packet.
/// 
/// The resulting data messages will maintain the order of descriptors sent in the command.
///
///@{

struct GnssMessageFormat
{
    /// Parameters
    FunctionSelector function = static_cast<FunctionSelector>(0);
    uint8_t num_descriptors = 0; ///< Number of descriptors
    DescriptorRate descriptors[82]; ///< Descriptor format list.
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_GNSS_MESSAGE_FORMAT;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "GnssMessageFormat";
    static constexpr const char* DOC_NAME = "GNSS Message Format";
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    
    auto asTuple() const
    {
        return std::make_tuple(num_descriptors,descriptors);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(num_descriptors),std::ref(descriptors));
    }
    
    static GnssMessageFormat create_sld_all(::mip::FunctionSelector function)
    {
        GnssMessageFormat cmd;
        cmd.function = function;
        return cmd;
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
    struct Response
    {
        /// Parameters
        uint8_t num_descriptors = 0; ///< Number of descriptors
        DescriptorRate descriptors[82]; ///< Descriptor format list.
        
        /// Descriptors
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::REPLY_GNSS_MESSAGE_FORMAT;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "GnssMessageFormat::Response";
        static constexpr const char* DOC_NAME = "GNSS Message Format Response";
        static constexpr const bool HAS_FUNCTION_SELECTOR = false;
        
        auto asTuple() const
        {
            return std::make_tuple(num_descriptors,descriptors);
        }
        
        auto asTuple()
        {
            return std::make_tuple(std::ref(num_descriptors),std::ref(descriptors));
        }
        
        /// Serialization
        void insert(Serializer& serializer) const;
        void extract(Serializer& serializer);
        
    };
};
TypedResult<GnssMessageFormat> writeGnssMessageFormat(C::mip_interface& device, uint8_t numDescriptors, const DescriptorRate* descriptors);
TypedResult<GnssMessageFormat> readGnssMessageFormat(C::mip_interface& device, uint8_t* numDescriptorsOut, uint8_t numDescriptorsOutMax, DescriptorRate* descriptorsOut);
TypedResult<GnssMessageFormat> saveGnssMessageFormat(C::mip_interface& device);
TypedResult<GnssMessageFormat> loadGnssMessageFormat(C::mip_interface& device);
TypedResult<GnssMessageFormat> defaultGnssMessageFormat(C::mip_interface& device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup 3dm_filter_message_format_cpp  (0x0C,0x0A) Filter Message Format
/// Set, read, or save the format of the Estimation Filter data packet.
/// 
/// The resulting data messages will maintain the order of descriptors sent in the command.
///
///@{

struct FilterMessageFormat
{
    /// Parameters
    FunctionSelector function = static_cast<FunctionSelector>(0);
    uint8_t num_descriptors = 0; ///< Number of descriptors (limited by payload size)
    DescriptorRate descriptors[82];
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_FILTER_MESSAGE_FORMAT;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "FilterMessageFormat";
    static constexpr const char* DOC_NAME = "Estimation Filter Message Format";
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    
    auto asTuple() const
    {
        return std::make_tuple(num_descriptors,descriptors);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(num_descriptors),std::ref(descriptors));
    }
    
    static FilterMessageFormat create_sld_all(::mip::FunctionSelector function)
    {
        FilterMessageFormat cmd;
        cmd.function = function;
        return cmd;
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
    struct Response
    {
        /// Parameters
        uint8_t num_descriptors = 0; ///< Number of descriptors (limited by payload size)
        DescriptorRate descriptors[82];
        
        /// Descriptors
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::REPLY_FILTER_MESSAGE_FORMAT;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "FilterMessageFormat::Response";
        static constexpr const char* DOC_NAME = "Estimation Filter Message Format Response";
        static constexpr const bool HAS_FUNCTION_SELECTOR = false;
        
        auto asTuple() const
        {
            return std::make_tuple(num_descriptors,descriptors);
        }
        
        auto asTuple()
        {
            return std::make_tuple(std::ref(num_descriptors),std::ref(descriptors));
        }
        
        /// Serialization
        void insert(Serializer& serializer) const;
        void extract(Serializer& serializer);
        
    };
};
TypedResult<FilterMessageFormat> writeFilterMessageFormat(C::mip_interface& device, uint8_t numDescriptors, const DescriptorRate* descriptors);
TypedResult<FilterMessageFormat> readFilterMessageFormat(C::mip_interface& device, uint8_t* numDescriptorsOut, uint8_t numDescriptorsOutMax, DescriptorRate* descriptorsOut);
TypedResult<FilterMessageFormat> saveFilterMessageFormat(C::mip_interface& device);
TypedResult<FilterMessageFormat> loadFilterMessageFormat(C::mip_interface& device);
TypedResult<FilterMessageFormat> defaultFilterMessageFormat(C::mip_interface& device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup 3dm_filter_get_base_rate_cpp  (0x0C,0x0B) Filter Get Base Rate
/// Get the base rate for the Estimation Filter data in Hz
/// 
/// This is the fastest rate for this type of data available on the device.
/// This is used in conjunction with the Estimation Filter Message Format Command to set streaming data at a specified rate.
///
///@{

struct FilterGetBaseRate
{
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_GET_FILTER_BASE_RATE;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "FilterGetBaseRate";
    static constexpr const char* DOC_NAME = "Get Estimation Filter Data Base Rate";
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
        uint16_t rate = 0; ///< [hz]
        
        /// Descriptors
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::REPLY_FILTER_BASE_RATE;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "FilterGetBaseRate::Response";
        static constexpr const char* DOC_NAME = "Get Estimation Filter Data Base Rate Response";
        static constexpr const bool HAS_FUNCTION_SELECTOR = false;
        
        auto asTuple() const
        {
            return std::make_tuple(rate);
        }
        
        auto asTuple()
        {
            return std::make_tuple(std::ref(rate));
        }
        
        /// Serialization
        void insert(Serializer& serializer) const;
        void extract(Serializer& serializer);
        
    };
};
TypedResult<FilterGetBaseRate> filterGetBaseRate(C::mip_interface& device, uint16_t* rateOut);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup 3dm_nmea_message_format_cpp  (0x0C,0x0C) Nmea Message Format
/// Set, read, or save the NMEA message format.
///
///@{

struct NmeaMessageFormat
{
    /// Parameters
    FunctionSelector function = static_cast<FunctionSelector>(0);
    uint8_t count = 0; ///< Number of format entries (limited by payload size)
    NmeaMessage format_entries[40]; ///< List of format entries.
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_NMEA_MESSAGE_FORMAT;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "NmeaMessageFormat";
    static constexpr const char* DOC_NAME = "NMEA Message Format";
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    
    auto asTuple() const
    {
        return std::make_tuple(count,format_entries);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(count),std::ref(format_entries));
    }
    
    static NmeaMessageFormat create_sld_all(::mip::FunctionSelector function)
    {
        NmeaMessageFormat cmd;
        cmd.function = function;
        return cmd;
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
    struct Response
    {
        /// Parameters
        uint8_t count = 0; ///< Number of format entries (limited by payload size)
        NmeaMessage format_entries[40]; ///< List of format entries.
        
        /// Descriptors
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::REPLY_NMEA_MESSAGE_FORMAT;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "NmeaMessageFormat::Response";
        static constexpr const char* DOC_NAME = "NMEA Message Format Response";
        static constexpr const bool HAS_FUNCTION_SELECTOR = false;
        
        auto asTuple() const
        {
            return std::make_tuple(count,format_entries);
        }
        
        auto asTuple()
        {
            return std::make_tuple(std::ref(count),std::ref(format_entries));
        }
        
        /// Serialization
        void insert(Serializer& serializer) const;
        void extract(Serializer& serializer);
        
    };
};
TypedResult<NmeaMessageFormat> writeNmeaMessageFormat(C::mip_interface& device, uint8_t count, const NmeaMessage* formatEntries);
TypedResult<NmeaMessageFormat> readNmeaMessageFormat(C::mip_interface& device, uint8_t* countOut, uint8_t countOutMax, NmeaMessage* formatEntriesOut);
TypedResult<NmeaMessageFormat> saveNmeaMessageFormat(C::mip_interface& device);
TypedResult<NmeaMessageFormat> loadNmeaMessageFormat(C::mip_interface& device);
TypedResult<NmeaMessageFormat> defaultNmeaMessageFormat(C::mip_interface& device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup 3dm_poll_data_cpp  (0x0C,0x0D) Poll Data
/// Poll the device for a message with the specified descriptor set and format.
/// 
/// This function polls for a message using the provided format. The resulting message
/// will maintain the order of descriptors sent in the command and any unrecognized
/// descriptors are ignored. If the format is not provided, the device will attempt to use the
/// stored format (set with the Set Message Format command.) If no format is provided
/// and there is no stored format, the device will respond with a NACK. The reply packet contains
/// an ACK/NACK field. The polled data packet is sent separately as a normal Data packet.
///
///@{

struct PollData
{
    /// Parameters
    uint8_t desc_set = 0; ///< Data descriptor set. Must be supported.
    bool suppress_ack = 0; ///< Suppress the usual ACK/NACK reply.
    uint8_t num_descriptors = 0; ///< Number of descriptors in the format list.
    uint8_t descriptors[82] = {0}; ///< Descriptor format list.
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_POLL_DATA;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "PollData";
    static constexpr const char* DOC_NAME = "Poll Data";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(desc_set,suppress_ack,num_descriptors,descriptors);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(desc_set),std::ref(suppress_ack),std::ref(num_descriptors),std::ref(descriptors));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
    typedef void Response;
};
TypedResult<PollData> pollData(C::mip_interface& device, uint8_t descSet, bool suppressAck, uint8_t numDescriptors, const uint8_t* descriptors);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup 3dm_get_base_rate_cpp  (0x0C,0x0E) Get Base Rate
/// Get the base rate for the specified descriptor set in Hz.
///
///@{

struct GetBaseRate
{
    /// Parameters
    uint8_t desc_set = 0; ///< This is the data descriptor set. It must be a supported descriptor.
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_GET_BASE_RATE;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "GetBaseRate";
    static constexpr const char* DOC_NAME = "Get Data Base Rate";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(desc_set);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(desc_set));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
    struct Response
    {
        /// Parameters
        uint8_t desc_set = 0; ///< Echoes the parameter in the command.
        uint16_t rate = 0; ///< Base rate in Hz (0 = variable, unknown, or user-defined rate.  Data will be sent when received).
        
        /// Descriptors
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::REPLY_BASE_RATE;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "GetBaseRate::Response";
        static constexpr const char* DOC_NAME = "Get Data Base Rate Response";
        static constexpr const bool HAS_FUNCTION_SELECTOR = false;
        
        auto asTuple() const
        {
            return std::make_tuple(desc_set,rate);
        }
        
        auto asTuple()
        {
            return std::make_tuple(std::ref(desc_set),std::ref(rate));
        }
        
        /// Serialization
        void insert(Serializer& serializer) const;
        void extract(Serializer& serializer);
        
    };
};
TypedResult<GetBaseRate> getBaseRate(C::mip_interface& device, uint8_t descSet, uint16_t* rateOut);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup 3dm_message_format_cpp  (0x0C,0x0F) Message Format
/// Set, read, or save the format for a given data packet.
/// 
/// The resulting data messages will maintain the order of descriptors sent in the command.
///
///@{

struct MessageFormat
{
    /// Parameters
    FunctionSelector function = static_cast<FunctionSelector>(0);
    uint8_t desc_set = 0; ///< Data descriptor set. Must be supported. When function is SAVE, LOAD, or DEFAULT, can be 0 to apply to all descriptor sets.
    uint8_t num_descriptors = 0; ///< Number of descriptors (limited by payload size)
    DescriptorRate descriptors[82]; ///< List of descriptors and decimations.
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_MESSAGE_FORMAT;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "MessageFormat";
    static constexpr const char* DOC_NAME = "Message Format";
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    
    auto asTuple() const
    {
        return std::make_tuple(desc_set,num_descriptors,descriptors);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(desc_set),std::ref(num_descriptors),std::ref(descriptors));
    }
    
    static MessageFormat create_sld_all(::mip::FunctionSelector function)
    {
        MessageFormat cmd;
        cmd.function = function;
        cmd.desc_set = 0;
        return cmd;
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
    struct Response
    {
        /// Parameters
        uint8_t desc_set = 0; ///< Echoes the descriptor set from the command.
        uint8_t num_descriptors = 0; ///< Number of descriptors in the list.
        DescriptorRate descriptors[82]; ///< List of descriptors and decimations.
        
        /// Descriptors
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::REPLY_MESSAGE_FORMAT;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "MessageFormat::Response";
        static constexpr const char* DOC_NAME = "Message Format Response";
        static constexpr const bool HAS_FUNCTION_SELECTOR = false;
        
        auto asTuple() const
        {
            return std::make_tuple(desc_set,num_descriptors,descriptors);
        }
        
        auto asTuple()
        {
            return std::make_tuple(std::ref(desc_set),std::ref(num_descriptors),std::ref(descriptors));
        }
        
        /// Serialization
        void insert(Serializer& serializer) const;
        void extract(Serializer& serializer);
        
    };
};
TypedResult<MessageFormat> writeMessageFormat(C::mip_interface& device, uint8_t descSet, uint8_t numDescriptors, const DescriptorRate* descriptors);
TypedResult<MessageFormat> readMessageFormat(C::mip_interface& device, uint8_t descSet, uint8_t* numDescriptorsOut, uint8_t numDescriptorsOutMax, DescriptorRate* descriptorsOut);
TypedResult<MessageFormat> saveMessageFormat(C::mip_interface& device, uint8_t descSet);
TypedResult<MessageFormat> loadMessageFormat(C::mip_interface& device, uint8_t descSet);
TypedResult<MessageFormat> defaultMessageFormat(C::mip_interface& device, uint8_t descSet);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup 3dm_factory_streaming_cpp  (0x0C,0x10) Factory Streaming
/// Configures the device for recording data for technical support.
/// 
/// This command will configure all available data streams to predefined
/// formats designed to be used with technical support.
///
///@{

struct FactoryStreaming
{
    enum class Action : uint8_t
    {
        OVERWRITE = 0,  ///<  Replaces the message format(s), removing any existing descriptors.
        MERGE     = 1,  ///<  Merges support descriptors into existing format(s). May reorder descriptors.
        ADD       = 2,  ///<  Adds descriptors to the current message format(s) without changing existing descriptors. May result in duplicates.
    };
    
    /// Parameters
    Action action = static_cast<Action>(0);
    uint8_t reserved = 0; ///< Reserved. Set to 0x00.
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_CONFIGURE_FACTORY_STREAMING;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "FactoryStreaming";
    static constexpr const char* DOC_NAME = "Factory Streaming";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(action,reserved);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(action),std::ref(reserved));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
    typedef void Response;
};
TypedResult<FactoryStreaming> factoryStreaming(C::mip_interface& device, FactoryStreaming::Action action, uint8_t reserved);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup 3dm_datastream_control_cpp  (0x0C,0x11) Datastream Control
/// Enable/disable the selected data stream.
/// 
/// Each data stream (descriptor set) can be enabled or disabled.
/// The default for the device is all streams enabled.
/// For all functions except 0x01 (use new setting),
/// the new enable flag value is ignored and can be omitted.
///
///@{

struct DatastreamControl
{
    static constexpr const uint8_t LEGACY_IMU_STREAM = 0x01;
    static constexpr const uint8_t LEGACY_GNSS_STREAM = 0x02;
    static constexpr const uint8_t LEGACY_FILTER_STREAM = 0x03;
    static constexpr const uint8_t ALL_STREAMS = 0x00;
    /// Parameters
    FunctionSelector function = static_cast<FunctionSelector>(0);
    uint8_t desc_set = 0; ///< The descriptor set of the stream to control. When function is SAVE, LOAD, or DEFAULT, can be ALL_STREAMS(0) to apply to all descriptor sets. On Generation 5 products, this must be one of the above legacy constants.
    bool enable = 0; ///< True or false to enable or disable the stream.
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_CONTROL_DATA_STREAM;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "DatastreamControl";
    static constexpr const char* DOC_NAME = "Data Stream Control";
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    
    auto asTuple() const
    {
        return std::make_tuple(desc_set,enable);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(desc_set),std::ref(enable));
    }
    
    static DatastreamControl create_sld_all(::mip::FunctionSelector function)
    {
        DatastreamControl cmd;
        cmd.function = function;
        cmd.desc_set = 0;
        return cmd;
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
    struct Response
    {
        /// Parameters
        uint8_t desc_set = 0;
        bool enabled = 0;
        
        /// Descriptors
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::REPLY_DATASTREAM_ENABLE;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "DatastreamControl::Response";
        static constexpr const char* DOC_NAME = "Data Stream Control Response";
        static constexpr const bool HAS_FUNCTION_SELECTOR = false;
        
        auto asTuple() const
        {
            return std::make_tuple(desc_set,enabled);
        }
        
        auto asTuple()
        {
            return std::make_tuple(std::ref(desc_set),std::ref(enabled));
        }
        
        /// Serialization
        void insert(Serializer& serializer) const;
        void extract(Serializer& serializer);
        
    };
};
TypedResult<DatastreamControl> writeDatastreamControl(C::mip_interface& device, uint8_t descSet, bool enable);
TypedResult<DatastreamControl> readDatastreamControl(C::mip_interface& device, uint8_t descSet, bool* enabledOut);
TypedResult<DatastreamControl> saveDatastreamControl(C::mip_interface& device, uint8_t descSet);
TypedResult<DatastreamControl> loadDatastreamControl(C::mip_interface& device, uint8_t descSet);
TypedResult<DatastreamControl> defaultDatastreamControl(C::mip_interface& device, uint8_t descSet);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup 3dm_constellation_settings_cpp  (0x0C,0x21) Constellation Settings
/// This command configures which satellite constellations are enabled and how many channels are dedicated to tracking each constellation.
/// 
/// Maximum number of tracking channels to use (total for all constellations):
/// 0 to max_channels_available (from reply message)
/// 
/// For each constellation you wish to use, include a ConstellationSettings struct.  Note the following:
/// 
/// Total number of tracking channels (sum of "reserved_channels" for all constellations) must be <= 32:
/// 0 -> 32 Number of reserved channels
/// 0 -> 32 Max number of channels (>= reserved channels)
/// 
/// The factory default setting is: GPS and GLONASS enabled.  Min/Max for GPS = 8/16, GLONASS = 8/14, SBAS = 1/3, QZSS = 0/3.
/// 
/// Warning: SBAS functionality shall not be used in "safety of life" applications!
/// Warning: Any setting that causes the total reserved channels to exceed 32 will result in a NACK.
/// Warning: You cannot enable GLONASS and BeiDou at the same time.
/// Note:    Enabling SBAS and QZSS augments GPS accuracy.
/// Note:    It is recommended to disable GLONASS and BeiDou if a GPS-only antenna or GPS-only SAW filter is used.
///
///@{

struct ConstellationSettings
{
    enum class ConstellationId : uint8_t
    {
        GPS     = 0,  ///<  GPS (G1-G32)
        SBAS    = 1,  ///<  SBAS (S120-S158)
        GALILEO = 2,  ///<  GALILEO (E1-E36)
        BEIDOU  = 3,  ///<  BeiDou (B1-B37)
        QZSS    = 5,  ///<  QZSS (Q1-Q5)
        GLONASS = 6,  ///<  GLONASS (R1-R32)
    };
    
    struct OptionFlags : Bitfield<OptionFlags>
    {
        typedef uint16_t Type;
        enum _enumType : uint16_t
        {
            NONE   = 0x0000,
            L1SAIF = 0x0001,  ///<  Available only for QZSS
            ALL    = 0x0001,
        };
        uint16_t value = NONE;
        
        constexpr OptionFlags() : value(NONE) {}
        constexpr OptionFlags(int val) : value((uint16_t)val) {}
        constexpr operator uint16_t() const { return value; }
        constexpr OptionFlags& operator=(uint16_t val) { value = val; return *this; }
        constexpr OptionFlags& operator=(int val) { value = uint16_t(val); return *this; }
        constexpr OptionFlags& operator|=(uint16_t val) { return *this = value | val; }
        constexpr OptionFlags& operator&=(uint16_t val) { return *this = value & val; }
        
        constexpr bool l1saif() const { return (value & L1SAIF) > 0; }
        constexpr void l1saif(bool val) { value &= ~L1SAIF; if(val) value |= L1SAIF; }
        constexpr bool allSet() const { return value == ALL; }
        constexpr void setAll() { value |= ALL; }
    };
    struct Settings
    {
        /// Parameters
        ConstellationId constellation_id = static_cast<ConstellationId>(0); ///< Constellation ID
        uint8_t enable = 0; ///< Enable/Disable constellation
        uint8_t reserved_channels = 0; ///< Minimum number of channels reserved for this constellation
        uint8_t max_channels = 0; ///< Maximum number of channels to use for this constallation
        OptionFlags option_flags; ///< Constellation option Flags
        
        /// Serialization
        void insert(Serializer& serializer) const;
        void extract(Serializer& serializer);
        
    };
    /// Parameters
    FunctionSelector function = static_cast<FunctionSelector>(0);
    uint16_t max_channels = 0;
    uint8_t config_count = 0;
    Settings settings[42];
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_GNSS_CONSTELLATION_SETTINGS;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "ConstellationSettings";
    static constexpr const char* DOC_NAME = "Constellation Settings";
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    
    auto asTuple() const
    {
        return std::make_tuple(max_channels,config_count,settings);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(max_channels),std::ref(config_count),std::ref(settings));
    }
    
    static ConstellationSettings create_sld_all(::mip::FunctionSelector function)
    {
        ConstellationSettings cmd;
        cmd.function = function;
        return cmd;
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
    struct Response
    {
        /// Parameters
        uint16_t max_channels_available = 0; ///< Maximum channels available
        uint16_t max_channels_use = 0; ///< Maximum channels to use
        uint8_t config_count = 0; ///< Number of constellation configurations
        Settings settings[42]; ///< Constellation Settings
        
        /// Descriptors
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::REPLY_GNSS_CONSTELLATION_SETTINGS;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "ConstellationSettings::Response";
        static constexpr const char* DOC_NAME = "Constellation Settings Response";
        static constexpr const bool HAS_FUNCTION_SELECTOR = false;
        
        auto asTuple() const
        {
            return std::make_tuple(max_channels_available,max_channels_use,config_count,settings);
        }
        
        auto asTuple()
        {
            return std::make_tuple(std::ref(max_channels_available),std::ref(max_channels_use),std::ref(config_count),std::ref(settings));
        }
        
        /// Serialization
        void insert(Serializer& serializer) const;
        void extract(Serializer& serializer);
        
    };
};
TypedResult<ConstellationSettings> writeConstellationSettings(C::mip_interface& device, uint16_t maxChannels, uint8_t configCount, const ConstellationSettings::Settings* settings);
TypedResult<ConstellationSettings> readConstellationSettings(C::mip_interface& device, uint16_t* maxChannelsAvailableOut, uint16_t* maxChannelsUseOut, uint8_t* configCountOut, uint8_t configCountOutMax, ConstellationSettings::Settings* settingsOut);
TypedResult<ConstellationSettings> saveConstellationSettings(C::mip_interface& device);
TypedResult<ConstellationSettings> loadConstellationSettings(C::mip_interface& device);
TypedResult<ConstellationSettings> defaultConstellationSettings(C::mip_interface& device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup 3dm_gnss_sbas_settings_cpp  (0x0C,0x22) Gnss Sbas Settings
/// Configure the GNSS SBAS subsystem
///
///@{

struct GnssSbasSettings
{
    struct SBASOptions : Bitfield<SBASOptions>
    {
        typedef uint16_t Type;
        enum _enumType : uint16_t
        {
            NONE               = 0x0000,
            ENABLE_RANGING     = 0x0001,  ///<  Use SBAS pseudoranges in position solution
            ENABLE_CORRECTIONS = 0x0002,  ///<  Use SBAS differential corrections
            APPLY_INTEGRITY    = 0x0004,  ///<  Use SBAS integrity information.  If enabled, only GPS satellites for which integrity information is available will be used.
            ALL                = 0x0007,
        };
        uint16_t value = NONE;
        
        constexpr SBASOptions() : value(NONE) {}
        constexpr SBASOptions(int val) : value((uint16_t)val) {}
        constexpr operator uint16_t() const { return value; }
        constexpr SBASOptions& operator=(uint16_t val) { value = val; return *this; }
        constexpr SBASOptions& operator=(int val) { value = uint16_t(val); return *this; }
        constexpr SBASOptions& operator|=(uint16_t val) { return *this = value | val; }
        constexpr SBASOptions& operator&=(uint16_t val) { return *this = value & val; }
        
        constexpr bool enableRanging() const { return (value & ENABLE_RANGING) > 0; }
        constexpr void enableRanging(bool val) { value &= ~ENABLE_RANGING; if(val) value |= ENABLE_RANGING; }
        constexpr bool enableCorrections() const { return (value & ENABLE_CORRECTIONS) > 0; }
        constexpr void enableCorrections(bool val) { value &= ~ENABLE_CORRECTIONS; if(val) value |= ENABLE_CORRECTIONS; }
        constexpr bool applyIntegrity() const { return (value & APPLY_INTEGRITY) > 0; }
        constexpr void applyIntegrity(bool val) { value &= ~APPLY_INTEGRITY; if(val) value |= APPLY_INTEGRITY; }
        constexpr bool allSet() const { return value == ALL; }
        constexpr void setAll() { value |= ALL; }
    };
    /// Parameters
    FunctionSelector function = static_cast<FunctionSelector>(0);
    uint8_t enable_sbas = 0; ///< 0 - SBAS Disabled, 1 - SBAS enabled
    SBASOptions sbas_options; ///< SBAS options, see definition
    uint8_t num_included_prns = 0; ///< Number of SBAS PRNs to include in search (0 = include all)
    uint16_t included_prns[39] = {0}; ///< List of specific SBAS PRNs to search for
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_GNSS_SBAS_SETTINGS;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "GnssSbasSettings";
    static constexpr const char* DOC_NAME = "GNSS SBAS Settings";
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    
    auto asTuple() const
    {
        return std::make_tuple(enable_sbas,sbas_options,num_included_prns,included_prns);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(enable_sbas),std::ref(sbas_options),std::ref(num_included_prns),std::ref(included_prns));
    }
    
    static GnssSbasSettings create_sld_all(::mip::FunctionSelector function)
    {
        GnssSbasSettings cmd;
        cmd.function = function;
        return cmd;
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
    struct Response
    {
        /// Parameters
        uint8_t enable_sbas = 0; ///< 0 - SBAS Disabled, 1 - SBAS enabled
        SBASOptions sbas_options; ///< SBAS options, see definition
        uint8_t num_included_prns = 0; ///< Number of SBAS PRNs to include in search (0 = include all)
        uint16_t included_prns[39] = {0}; ///< List of specific SBAS PRNs to search for
        
        /// Descriptors
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::REPLY_GNSS_SBAS_SETTINGS;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "GnssSbasSettings::Response";
        static constexpr const char* DOC_NAME = "GNSS SBAS Settings Response";
        static constexpr const bool HAS_FUNCTION_SELECTOR = false;
        
        auto asTuple() const
        {
            return std::make_tuple(enable_sbas,sbas_options,num_included_prns,included_prns);
        }
        
        auto asTuple()
        {
            return std::make_tuple(std::ref(enable_sbas),std::ref(sbas_options),std::ref(num_included_prns),std::ref(included_prns));
        }
        
        /// Serialization
        void insert(Serializer& serializer) const;
        void extract(Serializer& serializer);
        
    };
};
TypedResult<GnssSbasSettings> writeGnssSbasSettings(C::mip_interface& device, uint8_t enableSbas, GnssSbasSettings::SBASOptions sbasOptions, uint8_t numIncludedPrns, const uint16_t* includedPrns);
TypedResult<GnssSbasSettings> readGnssSbasSettings(C::mip_interface& device, uint8_t* enableSbasOut, GnssSbasSettings::SBASOptions* sbasOptionsOut, uint8_t* numIncludedPrnsOut, uint8_t numIncludedPrnsOutMax, uint16_t* includedPrnsOut);
TypedResult<GnssSbasSettings> saveGnssSbasSettings(C::mip_interface& device);
TypedResult<GnssSbasSettings> loadGnssSbasSettings(C::mip_interface& device);
TypedResult<GnssSbasSettings> defaultGnssSbasSettings(C::mip_interface& device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup 3dm_gnss_assisted_fix_cpp  (0x0C,0x23) Gnss Assisted Fix
/// Set the options for assisted GNSS fix.
/// 
/// Devices that implement this command have a dedicated GNSS flash memory and a non-volatile FRAM.
/// These storage mechanisms are used to retain information about the last good GNSS fix. This can greatly reduces the TTFF (Time To First Fix) depending on the age of the stored information.
/// The TTFF can be as low as one second, or up to the equivalent of a cold start. There is a small increase in power used when enabling assisted fix.
/// 
/// The fastest fix will be obtained by supplying the device with a GNSS Assist Time Update message containing the current GPS time immediately after subsequent power up.
/// This allows the device to determine if the last GNSS information saved is still fresh enough to improve the TTFF.
/// 
/// NOTE: Non-volatile GNSS memory is cleared when going from an enabled state to a disabled state.
/// WARNING: The clearing operation results in an erase operation on the GNSS Flash. The flash has a limited durability of 100,000 write/erase cycles
///
///@{

struct GnssAssistedFix
{
    enum class AssistedFixOption : uint8_t
    {
        NONE    = 0,  ///<  No assisted fix (default)
        ENABLED = 1,  ///<  Enable assisted fix
    };
    
    /// Parameters
    FunctionSelector function = static_cast<FunctionSelector>(0);
    AssistedFixOption option = static_cast<AssistedFixOption>(0); ///< Assisted fix options
    uint8_t flags = 0; ///< Assisted fix flags (set to 0xFF)
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_GNSS_ASSISTED_FIX_SETTINGS;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "GnssAssistedFix";
    static constexpr const char* DOC_NAME = "GNSS Assisted Fix Settings";
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    
    auto asTuple() const
    {
        return std::make_tuple(option,flags);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(option),std::ref(flags));
    }
    
    static GnssAssistedFix create_sld_all(::mip::FunctionSelector function)
    {
        GnssAssistedFix cmd;
        cmd.function = function;
        return cmd;
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
    struct Response
    {
        /// Parameters
        AssistedFixOption option = static_cast<AssistedFixOption>(0); ///< Assisted fix options
        uint8_t flags = 0; ///< Assisted fix flags (set to 0xFF)
        
        /// Descriptors
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::REPLY_GNSS_ASSISTED_FIX_SETTINGS;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "GnssAssistedFix::Response";
        static constexpr const char* DOC_NAME = "GNSS Assisted Fix Settings Response";
        static constexpr const bool HAS_FUNCTION_SELECTOR = false;
        
        auto asTuple() const
        {
            return std::make_tuple(option,flags);
        }
        
        auto asTuple()
        {
            return std::make_tuple(std::ref(option),std::ref(flags));
        }
        
        /// Serialization
        void insert(Serializer& serializer) const;
        void extract(Serializer& serializer);
        
    };
};
TypedResult<GnssAssistedFix> writeGnssAssistedFix(C::mip_interface& device, GnssAssistedFix::AssistedFixOption option, uint8_t flags);
TypedResult<GnssAssistedFix> readGnssAssistedFix(C::mip_interface& device, GnssAssistedFix::AssistedFixOption* optionOut, uint8_t* flagsOut);
TypedResult<GnssAssistedFix> saveGnssAssistedFix(C::mip_interface& device);
TypedResult<GnssAssistedFix> loadGnssAssistedFix(C::mip_interface& device);
TypedResult<GnssAssistedFix> defaultGnssAssistedFix(C::mip_interface& device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup 3dm_gnss_time_assistance_cpp  (0x0C,0x24) Gnss Time Assistance
/// Provide the GNSS subsystem with initial time information.
/// 
/// This message is required immediately after power up if GNSS Assist was enabled when the device was powered off.
/// This will initialize the subsystem clock to help reduce the time to first fix (TTFF).
///
///@{

struct GnssTimeAssistance
{
    /// Parameters
    FunctionSelector function = static_cast<FunctionSelector>(0);
    double tow = 0; ///< GPS Time of week [seconds]
    uint16_t week_number = 0; ///< GPS Weeks since 1980 [weeks]
    float accuracy = 0; ///< Accuracy of time information [seconds]
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_GNSS_TIME_ASSISTANCE;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "GnssTimeAssistance";
    static constexpr const char* DOC_NAME = "GNSS Time Assistance";
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    
    auto asTuple() const
    {
        return std::make_tuple(tow,week_number,accuracy);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(tow),std::ref(week_number),std::ref(accuracy));
    }
    
    static GnssTimeAssistance create_sld_all(::mip::FunctionSelector function)
    {
        GnssTimeAssistance cmd;
        cmd.function = function;
        return cmd;
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
    struct Response
    {
        /// Parameters
        double tow = 0; ///< GPS Time of week [seconds]
        uint16_t week_number = 0; ///< GPS Weeks since 1980 [weeks]
        float accuracy = 0; ///< Accuracy of time information [seconds]
        
        /// Descriptors
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::REPLY_GNSS_TIME_ASSISTANCE;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "GnssTimeAssistance::Response";
        static constexpr const char* DOC_NAME = "GNSS Time Assistance Response";
        static constexpr const bool HAS_FUNCTION_SELECTOR = false;
        
        auto asTuple() const
        {
            return std::make_tuple(tow,week_number,accuracy);
        }
        
        auto asTuple()
        {
            return std::make_tuple(std::ref(tow),std::ref(week_number),std::ref(accuracy));
        }
        
        /// Serialization
        void insert(Serializer& serializer) const;
        void extract(Serializer& serializer);
        
    };
};
TypedResult<GnssTimeAssistance> writeGnssTimeAssistance(C::mip_interface& device, double tow, uint16_t weekNumber, float accuracy);
TypedResult<GnssTimeAssistance> readGnssTimeAssistance(C::mip_interface& device, double* towOut, uint16_t* weekNumberOut, float* accuracyOut);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup 3dm_pps_source_cpp  (0x0C,0x28) Pps Source
/// Controls the Pulse Per Second (PPS) source.
///
///@{

struct PpsSource
{
    enum class Source : uint8_t
    {
        DISABLED   = 0,  ///<  PPS output is disabled. Not valid for PPS source command.
        RECEIVER_1 = 1,  ///<  PPS is provided by GNSS receiver 1.
        RECEIVER_2 = 2,  ///<  PPS is provided by GNSS receiver 2.
        GPIO       = 3,  ///<  PPS is provided to an external GPIO pin. Use the GPIO Setup command to choose and configure the pin.
        GENERATED  = 4,  ///<  PPS is generated from the system oscillator.
    };
    
    /// Parameters
    FunctionSelector function = static_cast<FunctionSelector>(0);
    Source source = static_cast<Source>(0);
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_PPS_SOURCE;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "PpsSource";
    static constexpr const char* DOC_NAME = "PPS Source Control";
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    
    auto asTuple() const
    {
        return std::make_tuple(source);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(source));
    }
    
    static PpsSource create_sld_all(::mip::FunctionSelector function)
    {
        PpsSource cmd;
        cmd.function = function;
        return cmd;
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
    struct Response
    {
        /// Parameters
        Source source = static_cast<Source>(0);
        
        /// Descriptors
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::REPLY_PPS_SOURCE;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "PpsSource::Response";
        static constexpr const char* DOC_NAME = "PPS Source Control Response";
        static constexpr const bool HAS_FUNCTION_SELECTOR = false;
        
        auto asTuple() const
        {
            return std::make_tuple(source);
        }
        
        auto asTuple()
        {
            return std::make_tuple(std::ref(source));
        }
        
        /// Serialization
        void insert(Serializer& serializer) const;
        void extract(Serializer& serializer);
        
    };
};
TypedResult<PpsSource> writePpsSource(C::mip_interface& device, PpsSource::Source source);
TypedResult<PpsSource> readPpsSource(C::mip_interface& device, PpsSource::Source* sourceOut);
TypedResult<PpsSource> savePpsSource(C::mip_interface& device);
TypedResult<PpsSource> loadPpsSource(C::mip_interface& device);
TypedResult<PpsSource> defaultPpsSource(C::mip_interface& device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup 3dm_get_event_support_cpp  (0x0C,0x2A) Get Event Support
/// Lists the available trigger or action types.
/// 
/// There are a limited number of trigger and action slots available
/// in the device. Up to M triggers and N actions can be configured at once
/// in slots 1..M and 1..N respectively. M and N are identified by the
/// max_instances field in the response with the appropriate query selector.
/// 
/// Each slot can be configured as one of a variety of different types of
/// triggers or actions. The supported types are enumerated in the response
/// to this command. Additionally, there is a limit on the number of a given
/// type. In other words, while the device may support M triggers in total,
/// only a few of them maybe usable as a given type. This limit helps optimize
/// device resources. The limit is identified in the count field.
/// 
/// All of the information in this command is available in the user manual.
/// This command provides a programmatic method for obtaining the information.
/// 
///
///@{

struct GetEventSupport
{
    enum class Query : uint8_t
    {
        TRIGGER_TYPES = 1,  ///<  Query the supported trigger types and max count for each.
        ACTION_TYPES  = 2,  ///<  Query the supported action types and max count for each.
    };
    
    struct Info
    {
        /// Parameters
        uint8_t type = 0; ///< Trigger or action type, as defined in the respective setup command.
        uint8_t count = 0; ///< This is the maximum number of instances supported for this type.
        
        /// Serialization
        void insert(Serializer& serializer) const;
        void extract(Serializer& serializer);
        
    };
    /// Parameters
    Query query = static_cast<Query>(0); ///< What type of information to retrieve.
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_EVENT_SUPPORT;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "GetEventSupport";
    static constexpr const char* DOC_NAME = "Get Supported Events";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(query);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(query));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
    struct Response
    {
        /// Parameters
        Query query = static_cast<Query>(0); ///< Query type specified in the command.
        uint8_t max_instances = 0; ///< Number of slots available. The 'instance' number for the configuration or control commands must be between 1 and this value.
        uint8_t num_entries = 0; ///< Number of supported types.
        Info entries[126]; ///< List of supported types.
        
        /// Descriptors
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::REPLY_EVENT_SUPPORT;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "GetEventSupport::Response";
        static constexpr const char* DOC_NAME = "Get Supported Events Response";
        static constexpr const bool HAS_FUNCTION_SELECTOR = false;
        
        auto asTuple() const
        {
            return std::make_tuple(query,max_instances,num_entries,entries);
        }
        
        auto asTuple()
        {
            return std::make_tuple(std::ref(query),std::ref(max_instances),std::ref(num_entries),std::ref(entries));
        }
        
        /// Serialization
        void insert(Serializer& serializer) const;
        void extract(Serializer& serializer);
        
    };
};
TypedResult<GetEventSupport> getEventSupport(C::mip_interface& device, GetEventSupport::Query query, uint8_t* maxInstancesOut, uint8_t* numEntriesOut, uint8_t numEntriesOutMax, GetEventSupport::Info* entriesOut);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup 3dm_event_control_cpp  (0x0C,0x2B) Event Control
/// Enables or disables event triggers.
/// 
/// Triggers can be disabled, enabled, and tested. While disabled, a trigger will
/// not evaluate its logic and effective behave like no trigger is configured.
/// A disabled trigger will not activate any actions. Triggers are disabled by default.
/// 
/// Use this command to enable (or disable) a trigger, or to place it into a test mode.
/// When in test mode, the trigger logic is disabled but the output is forced to
/// the active state, meaning that it will behave as if the trigger logic is satisfied
/// and any associated actions will execute.
///
///@{

struct EventControl
{
    enum class Mode : uint8_t
    {
        DISABLED   = 0,  ///<  Trigger is disabled.
        ENABLED    = 1,  ///<  Trigger is enabled and will work normally.
        TEST       = 2,  ///<  Forces the trigger to the active state for testing purposes.
        TEST_PULSE = 3,  ///<  Trigger is forced to the active state for one event cycle only. After the test cycle, the mode reverts to the previous state (either enabled or disabled).
    };
    
    /// Parameters
    FunctionSelector function = static_cast<FunctionSelector>(0);
    uint8_t instance = 0; ///< Trigger instance to affect. 0 can be used to apply the mode to all configured triggers, except when the function selector is READ.
    Mode mode = static_cast<Mode>(0); ///< How to change the trigger state. Except when instance is 0, the corresponding trigger must be configured, i.e. not have type 0.
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_EVENT_CONTROL;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "EventControl";
    static constexpr const char* DOC_NAME = "Event Control";
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    
    auto asTuple() const
    {
        return std::make_tuple(instance,mode);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(instance),std::ref(mode));
    }
    
    static EventControl create_sld_all(::mip::FunctionSelector function)
    {
        EventControl cmd;
        cmd.function = function;
        cmd.instance = 0;
        return cmd;
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
    struct Response
    {
        /// Parameters
        uint8_t instance = 0; ///< Trigger instance to affect. 0 can be used to apply the mode to all configured triggers, except when the function selector is READ.
        Mode mode = static_cast<Mode>(0); ///< How to change the trigger state. Except when instance is 0, the corresponding trigger must be configured, i.e. not have type 0.
        
        /// Descriptors
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::REPLY_EVENT_CONTROL;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "EventControl::Response";
        static constexpr const char* DOC_NAME = "Event Control Response";
        static constexpr const bool HAS_FUNCTION_SELECTOR = false;
        
        auto asTuple() const
        {
            return std::make_tuple(instance,mode);
        }
        
        auto asTuple()
        {
            return std::make_tuple(std::ref(instance),std::ref(mode));
        }
        
        /// Serialization
        void insert(Serializer& serializer) const;
        void extract(Serializer& serializer);
        
    };
};
TypedResult<EventControl> writeEventControl(C::mip_interface& device, uint8_t instance, EventControl::Mode mode);
TypedResult<EventControl> readEventControl(C::mip_interface& device, uint8_t instance, EventControl::Mode* modeOut);
TypedResult<EventControl> saveEventControl(C::mip_interface& device, uint8_t instance);
TypedResult<EventControl> loadEventControl(C::mip_interface& device, uint8_t instance);
TypedResult<EventControl> defaultEventControl(C::mip_interface& device, uint8_t instance);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup 3dm_get_event_trigger_status_cpp  (0x0C,0x2C) Get Event Trigger Status
///
///@{

struct GetEventTriggerStatus
{
    struct Status : Bitfield<Status>
    {
        typedef uint8_t Type;
        enum _enumType : uint8_t
        {
            NONE    = 0x00,
            ACTIVE  = 0x01,  ///<  True if the trigger is currently active (either due to its logic or being in test mode).
            ENABLED = 0x02,  ///<  True if the trigger is enabled.
            TEST    = 0x04,  ///<  True if the trigger is in test mode.
            ALL     = 0x07,
        };
        uint8_t value = NONE;
        
        constexpr Status() : value(NONE) {}
        constexpr Status(int val) : value((uint8_t)val) {}
        constexpr operator uint8_t() const { return value; }
        constexpr Status& operator=(uint8_t val) { value = val; return *this; }
        constexpr Status& operator=(int val) { value = uint8_t(val); return *this; }
        constexpr Status& operator|=(uint8_t val) { return *this = value | val; }
        constexpr Status& operator&=(uint8_t val) { return *this = value & val; }
        
        constexpr bool active() const { return (value & ACTIVE) > 0; }
        constexpr void active(bool val) { value &= ~ACTIVE; if(val) value |= ACTIVE; }
        constexpr bool enabled() const { return (value & ENABLED) > 0; }
        constexpr void enabled(bool val) { value &= ~ENABLED; if(val) value |= ENABLED; }
        constexpr bool test() const { return (value & TEST) > 0; }
        constexpr void test(bool val) { value &= ~TEST; if(val) value |= TEST; }
        constexpr bool allSet() const { return value == ALL; }
        constexpr void setAll() { value |= ALL; }
    };
    struct Entry
    {
        /// Parameters
        uint8_t type = 0; ///< Configured trigger type.
        Status status; ///< Trigger status.
        
        /// Serialization
        void insert(Serializer& serializer) const;
        void extract(Serializer& serializer);
        
    };
    /// Parameters
    uint8_t requested_count = 0; ///< Number of entries requested. If 0, requests all trigger slots.
    uint8_t requested_instances[20] = {0}; ///< List of trigger instances to query.
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_EVENT_TRIGGER_STATUS;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "GetEventTriggerStatus";
    static constexpr const char* DOC_NAME = "Get Event Trigger Status";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(requested_count,requested_instances);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(requested_count),std::ref(requested_instances));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
    struct Response
    {
        /// Parameters
        uint8_t count = 0; ///< Number of entries requested. If requested_count was 0, this is the number of supported trigger slots.
        Entry triggers[20]; ///< A list of the configured triggers. Entries are in the order requested, or in increasing order if count was 0.
        
        /// Descriptors
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::REPLY_EVENT_TRIGGER_STATUS;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "GetEventTriggerStatus::Response";
        static constexpr const char* DOC_NAME = "Get Event Trigger Status Response";
        static constexpr const bool HAS_FUNCTION_SELECTOR = false;
        
        auto asTuple() const
        {
            return std::make_tuple(count,triggers);
        }
        
        auto asTuple()
        {
            return std::make_tuple(std::ref(count),std::ref(triggers));
        }
        
        /// Serialization
        void insert(Serializer& serializer) const;
        void extract(Serializer& serializer);
        
    };
};
TypedResult<GetEventTriggerStatus> getEventTriggerStatus(C::mip_interface& device, uint8_t requestedCount, const uint8_t* requestedInstances, uint8_t* countOut, uint8_t countOutMax, GetEventTriggerStatus::Entry* triggersOut);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup 3dm_get_event_action_status_cpp  (0x0C,0x2D) Get Event Action Status
///
///@{

struct GetEventActionStatus
{
    struct Entry
    {
        /// Parameters
        uint8_t action_type = 0; ///< Configured action type.
        uint8_t trigger_id = 0; ///< Associated trigger instance.
        
        /// Serialization
        void insert(Serializer& serializer) const;
        void extract(Serializer& serializer);
        
    };
    /// Parameters
    uint8_t requested_count = 0; ///< Number of entries requested. If 0, requests all action slots.
    uint8_t requested_instances[20] = {0}; ///< List of action instances to query.
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_EVENT_ACTION_STATUS;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "GetEventActionStatus";
    static constexpr const char* DOC_NAME = "Get Event Action Status";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(requested_count,requested_instances);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(requested_count),std::ref(requested_instances));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
    struct Response
    {
        /// Parameters
        uint8_t count = 0; ///< Number of entries requested. If requested_count was 0, this is the number of supported action slots.
        Entry actions[20]; ///< A list of the configured actions. Entries are in the order requested, or in increasing order if count was 0.
        
        /// Descriptors
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::REPLY_EVENT_ACTION_STATUS;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "GetEventActionStatus::Response";
        static constexpr const char* DOC_NAME = "Get Event Action Status Response";
        static constexpr const bool HAS_FUNCTION_SELECTOR = false;
        
        auto asTuple() const
        {
            return std::make_tuple(count,actions);
        }
        
        auto asTuple()
        {
            return std::make_tuple(std::ref(count),std::ref(actions));
        }
        
        /// Serialization
        void insert(Serializer& serializer) const;
        void extract(Serializer& serializer);
        
    };
};
TypedResult<GetEventActionStatus> getEventActionStatus(C::mip_interface& device, uint8_t requestedCount, const uint8_t* requestedInstances, uint8_t* countOut, uint8_t countOutMax, GetEventActionStatus::Entry* actionsOut);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup 3dm_event_trigger_cpp  (0x0C,0x2E) Event Trigger
/// Configures various types of event triggers.
///
///@{

struct EventTrigger
{
    struct GpioParams
    {
        enum class Mode : uint8_t
        {
            DISABLED   = 0,  ///<  The pin will have no effect and the trigger will never activate.
            WHILE_HIGH = 1,  ///<  The trigger will be active while the pin is high.
            WHILE_LOW  = 2,  ///<  The trigger will be active while the pin is low.
            EDGE       = 4,  ///<  Use if the pin is configured for timestamping via the 3DM Gpio Configuration command (0x0C41).
        };
        
        /// Parameters
        uint8_t pin = 0; ///< GPIO pin number.
        Mode mode = static_cast<Mode>(0); ///< How the pin state affects the trigger.
        
        /// Serialization
        void insert(Serializer& serializer) const;
        void extract(Serializer& serializer);
        
    };
    struct ThresholdParams
    {
        enum class Type : uint8_t
        {
            WINDOW   = 1,  ///<  Window comparison. Trigger is active if low_thres &lt;= value &lt;= high_thres. If the thresholds are reversed, the trigger is active when value &lt; high_thres or value &gt; low_thres.
            INTERVAL = 2,  ///<  Trigger at evenly spaced intervals. Normally used with time fields to trigger periodically. Trigger is active when (value % interval) &lt;= int_thres. If the thresholds are reversed (high_thres &lt; low_thres) then the trigger is active when (value % low_thres) &gt; high_thres.
        };
        
        /// Parameters
        uint8_t desc_set = 0; ///< Descriptor set of target data quantity.
        uint8_t field_desc = 0; ///< Field descriptor of target data quantity.
        uint8_t param_id = 0; ///< 1-based index of the target parameter within the MIP field. E.g. for Scaled Accel (0x80,0x04) a value of 2 would represent the Y axis.
        Type type = static_cast<Type>(0); ///< Determines the type of comparison.
        union
        {
            double low_thres;
            double int_thres;
        };
        union
        {
            double high_thres;
            double interval;
        };
        
        /// Serialization
        void insert(Serializer& serializer) const;
        void extract(Serializer& serializer);
        
    };
    struct CombinationParams
    {
        static constexpr const uint16_t LOGIC_NEVER = 0x0000;
        static constexpr const uint16_t LOGIC_ALWAYS = 0xFFFF;
        static constexpr const uint16_t LOGIC_NONE = 0x0001;
        static constexpr const uint16_t LOGIC_OR = 0xFFFE;
        static constexpr const uint16_t LOGIC_NAND = 0x7FFF;
        static constexpr const uint16_t LOGIC_XOR_ONE = 0x0116;
        static constexpr const uint16_t LOGIC_ONLY_A = 0x0002;
        static constexpr const uint16_t LOGIC_ONLY_B = 0x0004;
        static constexpr const uint16_t LOGIC_ONLY_C = 0x0010;
        static constexpr const uint16_t LOGIC_ONLY_D = 0x0100;
        static constexpr const uint16_t LOGIC_AND_AB = 0x8888;
        static constexpr const uint16_t LOGIC_AB_OR_C = 0xF8F8;
        static constexpr const uint16_t LOGIC_AND = 0x8000;
        /// Parameters
        uint16_t logic_table = 0; ///< The last column of a truth table describing the output given the state of each input.
        uint8_t input_triggers[4] = {0}; ///< List of trigger IDs for inputs. Use 0 for unused inputs.
        
        /// Serialization
        void insert(Serializer& serializer) const;
        void extract(Serializer& serializer);
        
    };
    enum class Type : uint8_t
    {
        NONE        = 0,  ///<  No trigger selected. The state will always be inactive.
        GPIO        = 1,  ///<  Trigger based on the state of a GPIO pin. See GpioParams.
        THRESHOLD   = 2,  ///<  Compare a data quantity against a high and low threshold. See ThresholdParams.
        COMBINATION = 3,  ///<  Logical combination of two or more triggers. See CombinationParams.
    };
    
    union Parameters
    {
        GpioParams gpio;
        ThresholdParams threshold;
        CombinationParams combination;
        
        Parameters() {}
    };
    /// Parameters
    FunctionSelector function = static_cast<FunctionSelector>(0);
    uint8_t instance = 0; ///< Trigger number. When function is SAVE, LOAD, or DEFAULT, this can be 0 to apply to all instances.
    Type type = static_cast<Type>(0); ///< Type of trigger to configure.
    Parameters parameters;
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_EVENT_TRIGGER_CONFIG;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "EventTrigger";
    static constexpr const char* DOC_NAME = "Event Trigger Configuration";
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    
    auto asTuple() const
    {
        return std::make_tuple(instance,type,parameters);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(instance),std::ref(type),std::ref(parameters));
    }
    
    static EventTrigger create_sld_all(::mip::FunctionSelector function)
    {
        EventTrigger cmd;
        cmd.function = function;
        cmd.instance = 0;
        return cmd;
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
    struct Response
    {
        /// Parameters
        uint8_t instance = 0; ///< Trigger number. When function is SAVE, LOAD, or DEFAULT, this can be 0 to apply to all instances.
        Type type = static_cast<Type>(0); ///< Type of trigger to configure.
        Parameters parameters;
        
        /// Descriptors
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::REPLY_EVENT_TRIGGER_CONFIG;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "EventTrigger::Response";
        static constexpr const char* DOC_NAME = "Event Trigger Configuration Response";
        static constexpr const bool HAS_FUNCTION_SELECTOR = false;
        
        auto asTuple() const
        {
            return std::make_tuple(instance,type,parameters);
        }
        
        auto asTuple()
        {
            return std::make_tuple(std::ref(instance),std::ref(type),std::ref(parameters));
        }
        
        /// Serialization
        void insert(Serializer& serializer) const;
        void extract(Serializer& serializer);
        
    };
};
TypedResult<EventTrigger> writeEventTrigger(C::mip_interface& device, uint8_t instance, EventTrigger::Type type, const EventTrigger::Parameters& parameters);
TypedResult<EventTrigger> readEventTrigger(C::mip_interface& device, uint8_t instance, EventTrigger::Type* typeOut, EventTrigger::Parameters* parametersOut);
TypedResult<EventTrigger> saveEventTrigger(C::mip_interface& device, uint8_t instance);
TypedResult<EventTrigger> loadEventTrigger(C::mip_interface& device, uint8_t instance);
TypedResult<EventTrigger> defaultEventTrigger(C::mip_interface& device, uint8_t instance);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup 3dm_event_action_cpp  (0x0C,0x2F) Event Action
/// Configures various types of event actions.
///
///@{

struct EventAction
{
    struct GpioParams
    {
        enum class Mode : uint8_t
        {
            DISABLED     = 0,  ///<  Pin state will not be changed.
            ACTIVE_HIGH  = 1,  ///<  Pin will be set high when the trigger is active and low otherwise.
            ACTIVE_LOW   = 2,  ///<  Pin will be set low when the trigger is active and high otherwise.
            ONESHOT_HIGH = 5,  ///<  Pin will be set high each time the trigger activates. It will not be set low.
            ONESHOT_LOW  = 6,  ///<  Pin will be set low each time the trigger activates. It will not be set high.
            TOGGLE       = 7,  ///<  Pin will change to the opposite state each time the trigger activates.
        };
        
        /// Parameters
        uint8_t pin = 0; ///< GPIO pin number.
        Mode mode = static_cast<Mode>(0); ///< Behavior of the pin.
        
        /// Serialization
        void insert(Serializer& serializer) const;
        void extract(Serializer& serializer);
        
    };
    struct MessageParams
    {
        /// Parameters
        uint8_t desc_set = 0; ///< MIP data descriptor set.
        uint16_t decimation = 0; ///< Decimation from the base rate. If 0, a packet is emitted each time the trigger activates. Otherwise, packets will be streamed while the trigger is active. The internal decimation counter is reset if the trigger deactivates.
        uint8_t num_fields = 0; ///< Number of mip fields in the packet. Limited to 12.
        uint8_t descriptors[20] = {0}; ///< List of field descriptors.
        
        /// Serialization
        void insert(Serializer& serializer) const;
        void extract(Serializer& serializer);
        
    };
    enum class Type : uint8_t
    {
        NONE    = 0,  ///<  No action. Parameters should be empty.
        GPIO    = 1,  ///<  Control the state of a GPIO pin. See GpioParameters.
        MESSAGE = 2,  ///<  Output a data packet. See MessageParameters.
    };
    
    union Parameters
    {
        GpioParams gpio;
        MessageParams message;
        
        Parameters() {}
    };
    /// Parameters
    FunctionSelector function = static_cast<FunctionSelector>(0);
    uint8_t instance = 0; ///< Action number. When function is SAVE, LOAD, or DEFAULT, this can be 0 to apply to all instances.
    uint8_t trigger = 0; ///< Trigger ID number.
    Type type = static_cast<Type>(0); ///< Type of action to configure.
    Parameters parameters;
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_EVENT_ACTION_CONFIG;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "EventAction";
    static constexpr const char* DOC_NAME = "Event Action Configuration";
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    
    auto asTuple() const
    {
        return std::make_tuple(instance,trigger,type,parameters);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(instance),std::ref(trigger),std::ref(type),std::ref(parameters));
    }
    
    static EventAction create_sld_all(::mip::FunctionSelector function)
    {
        EventAction cmd;
        cmd.function = function;
        cmd.instance = 0;
        return cmd;
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
    struct Response
    {
        /// Parameters
        uint8_t instance = 0; ///< Action number. When function is SAVE, LOAD, or DEFAULT, this can be 0 to apply to all instances.
        uint8_t trigger = 0; ///< Trigger ID number.
        Type type = static_cast<Type>(0); ///< Type of action to configure.
        Parameters parameters;
        
        /// Descriptors
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::REPLY_EVENT_ACTION_CONFIG;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "EventAction::Response";
        static constexpr const char* DOC_NAME = "Event Action Configuration Response";
        static constexpr const bool HAS_FUNCTION_SELECTOR = false;
        
        auto asTuple() const
        {
            return std::make_tuple(instance,trigger,type,parameters);
        }
        
        auto asTuple()
        {
            return std::make_tuple(std::ref(instance),std::ref(trigger),std::ref(type),std::ref(parameters));
        }
        
        /// Serialization
        void insert(Serializer& serializer) const;
        void extract(Serializer& serializer);
        
    };
};
TypedResult<EventAction> writeEventAction(C::mip_interface& device, uint8_t instance, uint8_t trigger, EventAction::Type type, const EventAction::Parameters& parameters);
TypedResult<EventAction> readEventAction(C::mip_interface& device, uint8_t instance, uint8_t* triggerOut, EventAction::Type* typeOut, EventAction::Parameters* parametersOut);
TypedResult<EventAction> saveEventAction(C::mip_interface& device, uint8_t instance);
TypedResult<EventAction> loadEventAction(C::mip_interface& device, uint8_t instance);
TypedResult<EventAction> defaultEventAction(C::mip_interface& device, uint8_t instance);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup 3dm_device_settings_cpp  (0x0C,0x30) Device Settings
/// Save, Load, or Reset to Default the values for all device settings.
/// 
/// When a save current settings command is issued, a brief data disturbance may occur while all settings are written to non-volatile memory.
/// 
/// This command should have a long timeout as it may take up to 1 second to complete.
///
///@{

struct DeviceSettings
{
    /// Parameters
    FunctionSelector function = static_cast<FunctionSelector>(0);
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_DEVICE_STARTUP_SETTINGS;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "DeviceSettings";
    static constexpr const char* DOC_NAME = "Device Start Up Settings";
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    
    auto asTuple() const
    {
        return std::make_tuple();
    }
    
    auto asTuple()
    {
        return std::make_tuple();
    }
    
    static DeviceSettings create_sld_all(::mip::FunctionSelector function)
    {
        DeviceSettings cmd;
        cmd.function = function;
        return cmd;
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
    typedef void Response;
};
TypedResult<DeviceSettings> saveDeviceSettings(C::mip_interface& device);
TypedResult<DeviceSettings> loadDeviceSettings(C::mip_interface& device);
TypedResult<DeviceSettings> defaultDeviceSettings(C::mip_interface& device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup 3dm_sensor_2_vehicle_transform_euler_cpp  (0x0C,0x31) Sensor 2 Vehicle Transform Euler
/// Sets the sensor-to-vehicle frame transformation using Yaw, Pitch, and Roll Euler angles.
/// 
/// These are the Yaw, Pitch, and Roll mounting angles of the sensor with respect to vehicle frame of reference,
/// and describe the transformation of vectors from the sensor body frame to the vehicle frame.<br/>
/// 
/// Note: This is the transformation, the inverse of the rotation defined in our legacy products.<br/>
/// 
/// The transformation may be stored in the device as a matrix or quaternion.  When Euler angles are read back from the device, they may not
/// be exactly equal to the Euler angles used to set the transformation, but they are functionally equivalent, such that they result in the same transformation.<br/>
/// <br/><br/>
/// This transformation to the vehicle frame will be applied to the following output quantities:<br/><br/>
/// IMU:<br/>
/// Scaled Acceleration<br/>
/// Scaled Gyro<br/>
/// Scaled Magnetometer<br/>
/// Delta Theta<br/>
/// Delta Velocity<br/>
/// Complementary Filter Orientation<br/>
/// <br/><br/>
/// Estimation Filter:<br/>
/// Estimated Orientation, Quaternion<br/>
/// Estimated Orientation, Matrix<br/>
/// Estimated Orientation, Euler Angles<br/>
/// Estimated Linear Acceleration<br/>
/// Estimated Angular Rate<br/>
/// Estimated Gravity Vector<br/>
/// <br/>
/// Changing this setting will force all low-pass filters, the complementary filter, and the estimation filter to reset.
///
///@{

struct Sensor2VehicleTransformEuler
{
    /// Parameters
    FunctionSelector function = static_cast<FunctionSelector>(0);
    float roll = 0; ///< [radians]
    float pitch = 0; ///< [radians]
    float yaw = 0; ///< [radians]
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_SENSOR2VEHICLE_TRANSFORM_EUL;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "Sensor2VehicleTransformEuler";
    static constexpr const char* DOC_NAME = "Sensor-to-Vehicle Frame Transformation Euler";
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    
    auto asTuple() const
    {
        return std::make_tuple(roll,pitch,yaw);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(roll),std::ref(pitch),std::ref(yaw));
    }
    
    static Sensor2VehicleTransformEuler create_sld_all(::mip::FunctionSelector function)
    {
        Sensor2VehicleTransformEuler cmd;
        cmd.function = function;
        return cmd;
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
    struct Response
    {
        /// Parameters
        float roll = 0; ///< [radians]
        float pitch = 0; ///< [radians]
        float yaw = 0; ///< [radians]
        
        /// Descriptors
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::REPLY_SENSOR2VEHICLE_TRANSFORM_EUL;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "Sensor2VehicleTransformEuler::Response";
        static constexpr const char* DOC_NAME = "Sensor-to-Vehicle Frame Transformation Euler Response";
        static constexpr const bool HAS_FUNCTION_SELECTOR = false;
        
        auto asTuple() const
        {
            return std::make_tuple(roll,pitch,yaw);
        }
        
        auto asTuple()
        {
            return std::make_tuple(std::ref(roll),std::ref(pitch),std::ref(yaw));
        }
        
        /// Serialization
        void insert(Serializer& serializer) const;
        void extract(Serializer& serializer);
        
    };
};
TypedResult<Sensor2VehicleTransformEuler> writeSensor2VehicleTransformEuler(C::mip_interface& device, float roll, float pitch, float yaw);
TypedResult<Sensor2VehicleTransformEuler> readSensor2VehicleTransformEuler(C::mip_interface& device, float* rollOut, float* pitchOut, float* yawOut);
TypedResult<Sensor2VehicleTransformEuler> saveSensor2VehicleTransformEuler(C::mip_interface& device);
TypedResult<Sensor2VehicleTransformEuler> loadSensor2VehicleTransformEuler(C::mip_interface& device);
TypedResult<Sensor2VehicleTransformEuler> defaultSensor2VehicleTransformEuler(C::mip_interface& device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup 3dm_sensor_2_vehicle_transform_quaternion_cpp  (0x0C,0x32) Sensor 2 Vehicle Transform Quaternion
/// Set the sensor-to-vehicle frame transformation using unit length quaternion.
/// 
/// Note: This is the transformation, the inverse of the rotation.
/// 
/// This quaternion describes the transformation of vectors from the sensor body frame to the vehicle frame of reference, and satisfies the following relationship:<br/>
/// 
/// EQSTART p^{veh} = q^{-1} p^{sen} q EQEND<br/>
/// 
/// Where:<br/>
/// EQSTART q = (q_w, q_x, q_y, q_z) EQEND is the quaternion describing the transformation. <br/>
/// EQSTART p^{sen} = (0, v^{sen}_x, v^{sen}_y, v^{sen}_z) EQEND and EQSTART v^{sen} EQEND is a 3-element vector expressed in the sensor body frame.<br/>
/// EQSTART p^{veh} = (0, v^{veh}_x, v^{veh}_y, v^{veh}_z) EQEND and EQSTART v^{veh} EQEND is a 3-element vector expressed in the vehicle frame.<br/>
/// 
/// The transformation may be stored in the device as a matrix or a quaternion.  When the quaternion is read back from the device, it may not
/// be exactly equal to the quaternion used to set the transformation, but it is functionally equivalent.<br/>
/// <br/><br/>
/// This transformation affects the following output quantities:<br/><br/>
/// IMU:<br/>
/// Scaled Acceleration<br/>
/// Scaled Gyro<br/>
/// Scaled Magnetometer<br/>
/// Delta Theta<br/>
/// Delta Velocity<br/>
/// <br/><br/>
/// Estimation Filter:<br/>
/// Estimated Orientation, Quaternion<br/>
/// Estimated Orientation, Matrix<br/>
/// Estimated Orientation, Euler Angles<br/>
/// Estimated Linear Acceleration<br/>
/// Estimated Angular Rate<br/>
/// Estimated Gravity Vector<br/>
/// <br/>
/// Changing this setting will force all low-pass filters, the complementary filter, and the estimation filter to reset.
///
///@{

struct Sensor2VehicleTransformQuaternion
{
    /// Parameters
    FunctionSelector function = static_cast<FunctionSelector>(0);
    Quatf q; ///< Unit length quaternion representing transform [w, i, j, k]
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_SENSOR2VEHICLE_TRANSFORM_QUAT;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "Sensor2VehicleTransformQuaternion";
    static constexpr const char* DOC_NAME = "Sensor-to-Vehicle Frame Transformation Quaternion";
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    
    auto asTuple() const
    {
        return std::make_tuple(q);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(q));
    }
    
    static Sensor2VehicleTransformQuaternion create_sld_all(::mip::FunctionSelector function)
    {
        Sensor2VehicleTransformQuaternion cmd;
        cmd.function = function;
        return cmd;
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
    struct Response
    {
        /// Parameters
        Quatf q; ///< Unit length quaternion representing transform [w, i, j, k]
        
        /// Descriptors
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::REPLY_SENSOR2VEHICLE_TRANSFORM_QUAT;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "Sensor2VehicleTransformQuaternion::Response";
        static constexpr const char* DOC_NAME = "Sensor-to-Vehicle Frame Transformation Quaternion Response";
        static constexpr const bool HAS_FUNCTION_SELECTOR = false;
        
        auto asTuple() const
        {
            return std::make_tuple(q);
        }
        
        auto asTuple()
        {
            return std::make_tuple(std::ref(q));
        }
        
        /// Serialization
        void insert(Serializer& serializer) const;
        void extract(Serializer& serializer);
        
    };
};
TypedResult<Sensor2VehicleTransformQuaternion> writeSensor2VehicleTransformQuaternion(C::mip_interface& device, const float* q);
TypedResult<Sensor2VehicleTransformQuaternion> readSensor2VehicleTransformQuaternion(C::mip_interface& device, float* qOut);
TypedResult<Sensor2VehicleTransformQuaternion> saveSensor2VehicleTransformQuaternion(C::mip_interface& device);
TypedResult<Sensor2VehicleTransformQuaternion> loadSensor2VehicleTransformQuaternion(C::mip_interface& device);
TypedResult<Sensor2VehicleTransformQuaternion> defaultSensor2VehicleTransformQuaternion(C::mip_interface& device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup 3dm_sensor_2_vehicle_transform_dcm_cpp  (0x0C,0x33) Sensor 2 Vehicle Transform Dcm
/// Set the sensor to vehicle frame transformation using a using a 3 x 3 direction cosine matrix EQSTART M_{ned}^{veh} EQEND, stored in row-major order in a 9-element array.
/// 
/// These angles define the transformation of vectors from the sensor body frame to the fixed vehicle frame, according to:<br/>
/// EQSTART v^{veh} = M_{sen}^{veh} v^{sen} EQEND<br/>
/// 
/// Where:<br/>
/// 
/// EQSTART v^{sen} EQEND is a 3-element vector expressed in the sensor body frame. <br/>
/// EQSTART v^{veh} EQEND is the same 3-element vector expressed in the vehicle frame.  <br/>
/// <br/>
/// The matrix elements are stored is row-major order: EQSTART M_{sen}^{veh} = \\begin{bmatrix} M_{11}, M_{12}, M_{13}, M_{21}, M_{22}, M_{23}, M_{31}, M_{32}, M_{33} \\end{bmatrix} EQEND
/// 
/// The transformation may be stored in the device as a matrix or a quaternion. When EQSTART M_{sen}^{veh} EQEND is read back from the device, it may not
/// be exactly equal to array used to set the transformation, but it is functionally equivalent.<br/>
/// <br/><br/>
/// This transformation affects the following output quantities:<br/><br/>
/// IMU:<br/>
/// Scaled Acceleration<br/>
/// Scaled Gyro<br/>
/// Scaled Magnetometer<br/>
/// Delta Theta<br/>
/// Delta Velocity<br/>
/// <br/><br/>
/// Estimation Filter:<br/>
/// Estimated Orientation, Quaternion<br/>
/// Estimated Orientation, Matrix<br/>
/// Estimated Orientation, Euler Angles<br/>
/// Estimated Linear Acceleration<br/>
/// Estimated Angular Rate<br/>
/// Estimated Gravity Vector<br/>
/// <br/>
/// Changing this setting will force all low-pass filters, the complementary filter, and the estimation filter to reset.
///
///@{

struct Sensor2VehicleTransformDcm
{
    /// Parameters
    FunctionSelector function = static_cast<FunctionSelector>(0);
    Matrix3f dcm; ///< 3 x 3 direction cosine matrix, stored in row-major order
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_SENSOR2VEHICLE_TRANSFORM_DCM;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "Sensor2VehicleTransformDcm";
    static constexpr const char* DOC_NAME = "Sensor-to-Vehicle Frame Transformation Direction Cosine Matrix";
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    
    auto asTuple() const
    {
        return std::make_tuple(dcm);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(dcm));
    }
    
    static Sensor2VehicleTransformDcm create_sld_all(::mip::FunctionSelector function)
    {
        Sensor2VehicleTransformDcm cmd;
        cmd.function = function;
        return cmd;
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
    struct Response
    {
        /// Parameters
        Matrix3f dcm; ///< 3 x 3 direction cosine matrix, stored in row-major order
        
        /// Descriptors
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::REPLY_SENSOR2VEHICLE_TRANSFORM_DCM;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "Sensor2VehicleTransformDcm::Response";
        static constexpr const char* DOC_NAME = "Sensor-to-Vehicle Frame Transformation Direction Cosine Matrix Response";
        static constexpr const bool HAS_FUNCTION_SELECTOR = false;
        
        auto asTuple() const
        {
            return std::make_tuple(dcm);
        }
        
        auto asTuple()
        {
            return std::make_tuple(std::ref(dcm));
        }
        
        /// Serialization
        void insert(Serializer& serializer) const;
        void extract(Serializer& serializer);
        
    };
};
TypedResult<Sensor2VehicleTransformDcm> writeSensor2VehicleTransformDcm(C::mip_interface& device, const float* dcm);
TypedResult<Sensor2VehicleTransformDcm> readSensor2VehicleTransformDcm(C::mip_interface& device, float* dcmOut);
TypedResult<Sensor2VehicleTransformDcm> saveSensor2VehicleTransformDcm(C::mip_interface& device);
TypedResult<Sensor2VehicleTransformDcm> loadSensor2VehicleTransformDcm(C::mip_interface& device);
TypedResult<Sensor2VehicleTransformDcm> defaultSensor2VehicleTransformDcm(C::mip_interface& device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup 3dm_accel_bias_cpp  (0x0C,0x37) Accel Bias
/// Configures the user specified accelerometer bias
/// 
/// The user specified bias is subtracted from the calibrated accelerometer output.  Value is input in the sensor frame.
///
///@{

struct AccelBias
{
    /// Parameters
    FunctionSelector function = static_cast<FunctionSelector>(0);
    Vector3f bias; ///< accelerometer bias in the sensor frame (x,y,z) [g]
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_ACCEL_BIAS;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "AccelBias";
    static constexpr const char* DOC_NAME = "Accelerometer Bias Configuration";
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    
    auto asTuple() const
    {
        return std::make_tuple(bias);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(bias));
    }
    
    static AccelBias create_sld_all(::mip::FunctionSelector function)
    {
        AccelBias cmd;
        cmd.function = function;
        return cmd;
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
    struct Response
    {
        /// Parameters
        Vector3f bias; ///< accelerometer bias in the sensor frame (x,y,z) [g]
        
        /// Descriptors
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::REPLY_ACCEL_BIAS_VECTOR;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "AccelBias::Response";
        static constexpr const char* DOC_NAME = "Accelerometer Bias Configuration Response";
        static constexpr const bool HAS_FUNCTION_SELECTOR = false;
        
        auto asTuple() const
        {
            return std::make_tuple(bias);
        }
        
        auto asTuple()
        {
            return std::make_tuple(std::ref(bias));
        }
        
        /// Serialization
        void insert(Serializer& serializer) const;
        void extract(Serializer& serializer);
        
    };
};
TypedResult<AccelBias> writeAccelBias(C::mip_interface& device, const float* bias);
TypedResult<AccelBias> readAccelBias(C::mip_interface& device, float* biasOut);
TypedResult<AccelBias> saveAccelBias(C::mip_interface& device);
TypedResult<AccelBias> loadAccelBias(C::mip_interface& device);
TypedResult<AccelBias> defaultAccelBias(C::mip_interface& device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup 3dm_gyro_bias_cpp  (0x0C,0x38) Gyro Bias
/// Configures the user specified gyroscope bias
/// 
/// The user specified bias is subtracted from the calibrated angular rate output.  Value is input in the sensor frame.
///
///@{

struct GyroBias
{
    /// Parameters
    FunctionSelector function = static_cast<FunctionSelector>(0);
    Vector3f bias; ///< gyro bias in the sensor frame (x,y,z) [radians/second]
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_GYRO_BIAS;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "GyroBias";
    static constexpr const char* DOC_NAME = "Gyroscope Bias Configuration";
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    
    auto asTuple() const
    {
        return std::make_tuple(bias);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(bias));
    }
    
    static GyroBias create_sld_all(::mip::FunctionSelector function)
    {
        GyroBias cmd;
        cmd.function = function;
        return cmd;
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
    struct Response
    {
        /// Parameters
        Vector3f bias; ///< gyro bias in the sensor frame (x,y,z) [radians/second]
        
        /// Descriptors
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::REPLY_GYRO_BIAS_VECTOR;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "GyroBias::Response";
        static constexpr const char* DOC_NAME = "Gyroscope Bias Configuration Response";
        static constexpr const bool HAS_FUNCTION_SELECTOR = false;
        
        auto asTuple() const
        {
            return std::make_tuple(bias);
        }
        
        auto asTuple()
        {
            return std::make_tuple(std::ref(bias));
        }
        
        /// Serialization
        void insert(Serializer& serializer) const;
        void extract(Serializer& serializer);
        
    };
};
TypedResult<GyroBias> writeGyroBias(C::mip_interface& device, const float* bias);
TypedResult<GyroBias> readGyroBias(C::mip_interface& device, float* biasOut);
TypedResult<GyroBias> saveGyroBias(C::mip_interface& device);
TypedResult<GyroBias> loadGyroBias(C::mip_interface& device);
TypedResult<GyroBias> defaultGyroBias(C::mip_interface& device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup 3dm_capture_gyro_bias_cpp  (0x0C,0x39) Capture Gyro Bias
/// Samples gyro for a specified time range and writes the averaged result to the Gyro Bias vector in RAM
/// 
/// The device will average the gyro output for the duration of "averaging_time_ms." To store the resulting vector
/// in non-volatile memory, use the Set Gyro Bias command.
/// IMPORTANT: The device must be stationary and experiencing minimum vibration for the duration of "averaging_time_ms"
/// Averaging Time range: 1000 to 30,000
///
///@{

struct CaptureGyroBias
{
    /// Parameters
    uint16_t averaging_time_ms = 0; ///< Averaging time [milliseconds]
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_CAPTURE_GYRO_BIAS;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "CaptureGyroBias";
    static constexpr const char* DOC_NAME = "Capture Gyroscope Bias";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(averaging_time_ms);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(averaging_time_ms));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
    struct Response
    {
        /// Parameters
        Vector3f bias; ///< gyro bias in the sensor frame (x,y,z) [radians/second]
        
        /// Descriptors
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::REPLY_GYRO_BIAS_VECTOR;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "CaptureGyroBias::Response";
        static constexpr const char* DOC_NAME = "Capture Gyroscope Bias Response";
        static constexpr const bool HAS_FUNCTION_SELECTOR = false;
        
        auto asTuple() const
        {
            return std::make_tuple(bias);
        }
        
        auto asTuple()
        {
            return std::make_tuple(std::ref(bias));
        }
        
        /// Serialization
        void insert(Serializer& serializer) const;
        void extract(Serializer& serializer);
        
    };
};
TypedResult<CaptureGyroBias> captureGyroBias(C::mip_interface& device, uint16_t averagingTimeMs, float* biasOut);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup 3dm_mag_hard_iron_offset_cpp  (0x0C,0x3A) Mag Hard Iron Offset
/// Configure the user specified magnetometer hard iron offset vector
/// 
/// The values for this offset are determined empirically by external software algorithms
/// based on calibration data taken after the device is installed in its application. These values
/// can be obtained and set by using Microstrain software tools.
/// Alternatively, on some systems, the auto-mag calibration feature may be used to capture these values in-run.
/// The offset is applied to the scaled magnetometer vector prior to output.
///
///@{

struct MagHardIronOffset
{
    /// Parameters
    FunctionSelector function = static_cast<FunctionSelector>(0);
    Vector3f offset; ///< hard iron offset in the sensor frame (x,y,z) [Gauss]
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_HARD_IRON_OFFSET;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "MagHardIronOffset";
    static constexpr const char* DOC_NAME = "Magnetometer Hard Iron Offset";
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    
    auto asTuple() const
    {
        return std::make_tuple(offset);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(offset));
    }
    
    static MagHardIronOffset create_sld_all(::mip::FunctionSelector function)
    {
        MagHardIronOffset cmd;
        cmd.function = function;
        return cmd;
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
    struct Response
    {
        /// Parameters
        Vector3f offset; ///< hard iron offset in the sensor frame (x,y,z) [Gauss]
        
        /// Descriptors
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::REPLY_HARD_IRON_OFFSET_VECTOR;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "MagHardIronOffset::Response";
        static constexpr const char* DOC_NAME = "Magnetometer Hard Iron Offset Response";
        static constexpr const bool HAS_FUNCTION_SELECTOR = false;
        
        auto asTuple() const
        {
            return std::make_tuple(offset);
        }
        
        auto asTuple()
        {
            return std::make_tuple(std::ref(offset));
        }
        
        /// Serialization
        void insert(Serializer& serializer) const;
        void extract(Serializer& serializer);
        
    };
};
TypedResult<MagHardIronOffset> writeMagHardIronOffset(C::mip_interface& device, const float* offset);
TypedResult<MagHardIronOffset> readMagHardIronOffset(C::mip_interface& device, float* offsetOut);
TypedResult<MagHardIronOffset> saveMagHardIronOffset(C::mip_interface& device);
TypedResult<MagHardIronOffset> loadMagHardIronOffset(C::mip_interface& device);
TypedResult<MagHardIronOffset> defaultMagHardIronOffset(C::mip_interface& device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup 3dm_mag_soft_iron_matrix_cpp  (0x0C,0x3B) Mag Soft Iron Matrix
/// Configure the user specified magnetometer soft iron offset matrix
/// 
/// The values for this matrix are determined empirically by external software algorithms
/// based on calibration data taken after the device is installed in its application. These values
/// can be obtained and set by using Microstrain software tools.
/// Alternatively, on some systems, the auto-mag calibration feature may be used to capture these values in-run.
/// The matrix is applied to the scaled magnetometer vector prior to output.
/// 
/// The matrix is in row major order:
/// EQSTART M = \\begin{bmatrix} 0 &amp; 1 &amp; 2 \\\\ 3 &amp; 4 &amp; 5 \\\\ 6 &amp; 7 &amp; 8 \\end{bmatrix} EQEND
///
///@{

struct MagSoftIronMatrix
{
    /// Parameters
    FunctionSelector function = static_cast<FunctionSelector>(0);
    Matrix3f offset; ///< soft iron matrix [dimensionless]
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_SOFT_IRON_MATRIX;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "MagSoftIronMatrix";
    static constexpr const char* DOC_NAME = "Magnetometer Soft Iron Matrix";
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    
    auto asTuple() const
    {
        return std::make_tuple(offset);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(offset));
    }
    
    static MagSoftIronMatrix create_sld_all(::mip::FunctionSelector function)
    {
        MagSoftIronMatrix cmd;
        cmd.function = function;
        return cmd;
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
    struct Response
    {
        /// Parameters
        Matrix3f offset; ///< soft iron matrix [dimensionless]
        
        /// Descriptors
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::REPLY_SOFT_IRON_COMP_MATRIX;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "MagSoftIronMatrix::Response";
        static constexpr const char* DOC_NAME = "Magnetometer Soft Iron Matrix Response";
        static constexpr const bool HAS_FUNCTION_SELECTOR = false;
        
        auto asTuple() const
        {
            return std::make_tuple(offset);
        }
        
        auto asTuple()
        {
            return std::make_tuple(std::ref(offset));
        }
        
        /// Serialization
        void insert(Serializer& serializer) const;
        void extract(Serializer& serializer);
        
    };
};
TypedResult<MagSoftIronMatrix> writeMagSoftIronMatrix(C::mip_interface& device, const float* offset);
TypedResult<MagSoftIronMatrix> readMagSoftIronMatrix(C::mip_interface& device, float* offsetOut);
TypedResult<MagSoftIronMatrix> saveMagSoftIronMatrix(C::mip_interface& device);
TypedResult<MagSoftIronMatrix> loadMagSoftIronMatrix(C::mip_interface& device);
TypedResult<MagSoftIronMatrix> defaultMagSoftIronMatrix(C::mip_interface& device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup 3dm_coning_sculling_enable_cpp  (0x0C,0x3E) Coning Sculling Enable
/// Controls the Coning and Sculling Compenstation setting.
///
///@{

struct ConingScullingEnable
{
    /// Parameters
    FunctionSelector function = static_cast<FunctionSelector>(0);
    bool enable = 0; ///< If true, coning and sculling compensation is enabled.
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_CONING_AND_SCULLING_ENABLE;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "ConingScullingEnable";
    static constexpr const char* DOC_NAME = "Coning and Sculling Enable";
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    
    auto asTuple() const
    {
        return std::make_tuple(enable);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(enable));
    }
    
    static ConingScullingEnable create_sld_all(::mip::FunctionSelector function)
    {
        ConingScullingEnable cmd;
        cmd.function = function;
        return cmd;
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
    struct Response
    {
        /// Parameters
        bool enable = 0; ///< If true, coning and sculling compensation is enabled.
        
        /// Descriptors
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::REPLY_CONING_AND_SCULLING_ENABLE;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "ConingScullingEnable::Response";
        static constexpr const char* DOC_NAME = "Coning and Sculling Enable Response";
        static constexpr const bool HAS_FUNCTION_SELECTOR = false;
        
        auto asTuple() const
        {
            return std::make_tuple(enable);
        }
        
        auto asTuple()
        {
            return std::make_tuple(std::ref(enable));
        }
        
        /// Serialization
        void insert(Serializer& serializer) const;
        void extract(Serializer& serializer);
        
    };
};
TypedResult<ConingScullingEnable> writeConingScullingEnable(C::mip_interface& device, bool enable);
TypedResult<ConingScullingEnable> readConingScullingEnable(C::mip_interface& device, bool* enableOut);
TypedResult<ConingScullingEnable> saveConingScullingEnable(C::mip_interface& device);
TypedResult<ConingScullingEnable> loadConingScullingEnable(C::mip_interface& device);
TypedResult<ConingScullingEnable> defaultConingScullingEnable(C::mip_interface& device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup 3dm_uart_baudrate_cpp  (0x0C,0x40) Uart Baudrate
/// Read, Save, Load, or Reset to Default the baud rate of the main communication channel.
/// 
/// For all functions except 0x01 (use new settings), the new baud rate value is ignored.
/// Please see the device user manual for supported baud rates.
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

struct UartBaudrate
{
    /// Parameters
    FunctionSelector function = static_cast<FunctionSelector>(0);
    uint32_t baud = 0;
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_UART_BAUDRATE;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "UartBaudrate";
    static constexpr const char* DOC_NAME = "UART Baudrate";
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    
    auto asTuple() const
    {
        return std::make_tuple(baud);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(baud));
    }
    
    static UartBaudrate create_sld_all(::mip::FunctionSelector function)
    {
        UartBaudrate cmd;
        cmd.function = function;
        return cmd;
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
    struct Response
    {
        /// Parameters
        uint32_t baud = 0;
        
        /// Descriptors
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::REPLY_UART_BAUDRATE;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "UartBaudrate::Response";
        static constexpr const char* DOC_NAME = "UART Baudrate Response";
        static constexpr const bool HAS_FUNCTION_SELECTOR = false;
        
        auto asTuple() const
        {
            return std::make_tuple(baud);
        }
        
        auto asTuple()
        {
            return std::make_tuple(std::ref(baud));
        }
        
        /// Serialization
        void insert(Serializer& serializer) const;
        void extract(Serializer& serializer);
        
    };
};
TypedResult<UartBaudrate> writeUartBaudrate(C::mip_interface& device, uint32_t baud);
TypedResult<UartBaudrate> readUartBaudrate(C::mip_interface& device, uint32_t* baudOut);
TypedResult<UartBaudrate> saveUartBaudrate(C::mip_interface& device);
TypedResult<UartBaudrate> loadUartBaudrate(C::mip_interface& device);
TypedResult<UartBaudrate> defaultUartBaudrate(C::mip_interface& device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup 3dm_gpio_config_cpp  (0x0C,0x41) Gpio Config
/// Configures the user GPIO pins on the connector for use with several built-in functions or for general input or output.
/// 
/// GPIO pins are device-dependent. Some features are only available on
/// certain pins. Some behaviors require specific configurations.
/// Consult the device user manual for restrictions and default settings.
/// 
/// To avoid glitches on GPIOs configured as an output in a mode other than
/// GPIO, always configure the relevant function before setting up the pin
/// with this command. Otherwise, the pin state will be undefined between
/// this command and the one to set up the feature. For input pins, use
/// this command first so the state is well-defined when the feature is
/// initialized.
/// 
/// Some configurations can only be active on one pin at a time. If such
/// configuration is applied to a second pin, the second one will take
/// precedence and the original pin's configuration will be reset.
/// 
///
///@{

struct GpioConfig
{
    enum class Feature : uint8_t
    {
        UNUSED    = 0,  ///<  The pin is not used. It may be technically possible to read the pin state in this mode, but this is not guaranteed to be true of all devices or pins.
        GPIO      = 1,  ///<  General purpose input or output. Use this for direct control of pin output state or to stream the state of the pin.
        PPS       = 2,  ///<  Pulse per second input or output.
        ENCODER   = 3,  ///<  Motor encoder/odometer input.
        TIMESTAMP = 4,  ///<  Precision Timestamping. Use with Event Trigger Configuration (0x0C,0x2E).
        UART      = 5,  ///<  UART data or control lines.
    };
    
    enum class Behavior : uint8_t
    {
        UNUSED            = 0,  ///<  Use 0 unless otherwise specified.
        GPIO_INPUT        = 1,  ///<  Pin will be an input. This can be used to stream or poll the value and is the default setting.
        GPIO_OUTPUT_LOW   = 2,  ///<  Pin is an output initially in the LOW state. This state will be restored during system startup if the configuration is saved.
        GPIO_OUTPUT_HIGH  = 3,  ///<  Pin is an output initially in the HIGH state. This state will be restored during system startup if the configuration is saved.
        PPS_INPUT         = 1,  ///<  Pin will receive the pulse-per-second signal. Only one pin can have this behavior. This will only work if the PPS Source command is configured to GPIO.
        PPS_OUTPUT        = 2,  ///<  Pin will transmit the pulse-per-second signal from the device.
        ENCODER_A         = 1,  ///<  Encoder "A" quadrature input. Only one pin can have this behavior. The last command to set this behavior will take precedence.
        ENCODER_B         = 2,  ///<  Encoder "B" quadrature input. Only one pin can have this behavior. The last command to set this behavior will take precedence.
        TIMESTAMP_RISING  = 1,  ///<  Rising edges will be timestamped.
        TIMESTAMP_FALLING = 2,  ///<  Falling edges will be timestamped.
        TIMESTAMP_EITHER  = 3,  ///<  Both rising and falling edges will be timestamped.
        UART_PORT2_TX     = 33,  ///<  (0x21) UART port 2 transmit.
        UART_PORT2_RX     = 34,  ///<  (0x22) UART port 2 receive.
        UART_PORT3_TX     = 49,  ///<  (0x31) UART port 3 transmit.
        UART_PORT3_RX     = 50,  ///<  (0x32) UART port 3 receive.
    };
    
    struct PinMode : Bitfield<PinMode>
    {
        typedef uint8_t Type;
        enum _enumType : uint8_t
        {
            NONE       = 0x00,
            OPEN_DRAIN = 0x01,  ///<  The pin will be an open-drain output. The state will be either LOW or FLOATING instead of LOW or HIGH, respectively. This is used to connect multiple open-drain outputs from several devices. An internal or external pull-up resistor is typically used in combination. The maximum voltage of an open drain output is subject to the device maximum input voltage range found in the specifications.
            PULLDOWN   = 0x02,  ///<  The pin will have an internal pull-down resistor enabled. This is useful for connecting inputs to signals which can only be pulled high such as mechanical switches. Cannot be used in combination with pull-up. See the device specifications for the resistance value.
            PULLUP     = 0x04,  ///<  The pin will have an internal pull-up resistor enabled. Useful for connecting inputs to signals which can only be pulled low such as mechanical switches, or in combination with an open drain output. Cannot be used in combination with pull-down. See the device specifications for the resistance value. Use of this mode may restrict the maximum allowed input voltage. See the device datasheet for details.
            ALL        = 0x07,
        };
        uint8_t value = NONE;
        
        constexpr PinMode() : value(NONE) {}
        constexpr PinMode(int val) : value((uint8_t)val) {}
        constexpr operator uint8_t() const { return value; }
        constexpr PinMode& operator=(uint8_t val) { value = val; return *this; }
        constexpr PinMode& operator=(int val) { value = uint8_t(val); return *this; }
        constexpr PinMode& operator|=(uint8_t val) { return *this = value | val; }
        constexpr PinMode& operator&=(uint8_t val) { return *this = value & val; }
        
        constexpr bool openDrain() const { return (value & OPEN_DRAIN) > 0; }
        constexpr void openDrain(bool val) { value &= ~OPEN_DRAIN; if(val) value |= OPEN_DRAIN; }
        constexpr bool pulldown() const { return (value & PULLDOWN) > 0; }
        constexpr void pulldown(bool val) { value &= ~PULLDOWN; if(val) value |= PULLDOWN; }
        constexpr bool pullup() const { return (value & PULLUP) > 0; }
        constexpr void pullup(bool val) { value &= ~PULLUP; if(val) value |= PULLUP; }
        constexpr bool allSet() const { return value == ALL; }
        constexpr void setAll() { value |= ALL; }
    };
    /// Parameters
    FunctionSelector function = static_cast<FunctionSelector>(0);
    uint8_t pin = 0; ///< GPIO pin number counting from 1. For save, load, and default function selectors, this can be 0 to select all pins.
    Feature feature = static_cast<Feature>(0); ///< Determines how the pin will be used.
    Behavior behavior = static_cast<Behavior>(0); ///< Select an appropriate value from the enumeration based on the selected feature (e.g. for PPS, select one of the values prefixed with PPS_.)
    PinMode pin_mode; ///< GPIO configuration. May be restricted depending on device, pin, feature, and behavior. See device user manual.
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_GPIO_CONFIG;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "GpioConfig";
    static constexpr const char* DOC_NAME = "GPIO Configuration";
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    
    auto asTuple() const
    {
        return std::make_tuple(pin,feature,behavior,pin_mode);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(pin),std::ref(feature),std::ref(behavior),std::ref(pin_mode));
    }
    
    static GpioConfig create_sld_all(::mip::FunctionSelector function)
    {
        GpioConfig cmd;
        cmd.function = function;
        cmd.pin = 0;
        return cmd;
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
    struct Response
    {
        /// Parameters
        uint8_t pin = 0; ///< GPIO pin number counting from 1. For save, load, and default function selectors, this can be 0 to select all pins.
        Feature feature = static_cast<Feature>(0); ///< Determines how the pin will be used.
        Behavior behavior = static_cast<Behavior>(0); ///< Select an appropriate value from the enumeration based on the selected feature (e.g. for PPS, select one of the values prefixed with PPS_.)
        PinMode pin_mode; ///< GPIO configuration. May be restricted depending on device, pin, feature, and behavior. See device user manual.
        
        /// Descriptors
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::REPLY_GPIO_CONFIG;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "GpioConfig::Response";
        static constexpr const char* DOC_NAME = "GPIO Configuration Response";
        static constexpr const bool HAS_FUNCTION_SELECTOR = false;
        
        auto asTuple() const
        {
            return std::make_tuple(pin,feature,behavior,pin_mode);
        }
        
        auto asTuple()
        {
            return std::make_tuple(std::ref(pin),std::ref(feature),std::ref(behavior),std::ref(pin_mode));
        }
        
        /// Serialization
        void insert(Serializer& serializer) const;
        void extract(Serializer& serializer);
        
    };
};
TypedResult<GpioConfig> writeGpioConfig(C::mip_interface& device, uint8_t pin, GpioConfig::Feature feature, GpioConfig::Behavior behavior, GpioConfig::PinMode pinMode);
TypedResult<GpioConfig> readGpioConfig(C::mip_interface& device, uint8_t pin, GpioConfig::Feature* featureOut, GpioConfig::Behavior* behaviorOut, GpioConfig::PinMode* pinModeOut);
TypedResult<GpioConfig> saveGpioConfig(C::mip_interface& device, uint8_t pin);
TypedResult<GpioConfig> loadGpioConfig(C::mip_interface& device, uint8_t pin);
TypedResult<GpioConfig> defaultGpioConfig(C::mip_interface& device, uint8_t pin);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup 3dm_gpio_state_cpp  (0x0C,0x42) Gpio State
/// Allows the state of the pin to be read or controlled.
/// 
/// This command serves two purposes: 1) To allow reading the state of a pin via command,
/// rather than polling a data quantity, and 2) to provide a way to set the output state
/// without also having to specify the operating mode.
/// 
/// The state read back from the pin is the physical state of the pin, rather than a
/// configuration value. The state can be read regardless of its configuration as long as
/// the device supports GPIO input on that pin. If the pin is set to an output, the read
/// value would match the output value.
/// 
/// While the state of a pin can always be set, it will only have an observable effect if
/// the pin is set to output mode.
/// 
/// This command does not support saving, loading, or resetting the state. Instead, use the
/// GPIO Configuration command, which allows the initial state to be configured.
///
///@{

struct GpioState
{
    /// Parameters
    FunctionSelector function = static_cast<FunctionSelector>(0);
    uint8_t pin = 0; ///< GPIO pin number counting from 1. Cannot be 0.
    bool state = 0; ///< The pin state.
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_GPIO_STATE;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "GpioState";
    static constexpr const char* DOC_NAME = "GPIO State";
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    
    auto asTuple() const
    {
        return std::make_tuple(pin,state);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(pin),std::ref(state));
    }
    
    static GpioState create_sld_all(::mip::FunctionSelector function)
    {
        GpioState cmd;
        cmd.function = function;
        return cmd;
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
    struct Response
    {
        /// Parameters
        uint8_t pin = 0; ///< GPIO pin number counting from 1. Cannot be 0.
        bool state = 0; ///< The pin state.
        
        /// Descriptors
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::REPLY_GPIO_STATE;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "GpioState::Response";
        static constexpr const char* DOC_NAME = "GPIO State Response";
        static constexpr const bool HAS_FUNCTION_SELECTOR = false;
        
        auto asTuple() const
        {
            return std::make_tuple(pin,state);
        }
        
        auto asTuple()
        {
            return std::make_tuple(std::ref(pin),std::ref(state));
        }
        
        /// Serialization
        void insert(Serializer& serializer) const;
        void extract(Serializer& serializer);
        
    };
};
TypedResult<GpioState> writeGpioState(C::mip_interface& device, uint8_t pin, bool state);
TypedResult<GpioState> readGpioState(C::mip_interface& device, uint8_t pin, bool* stateOut);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup 3dm_odometer_cpp  (0x0C,0x43) Odometer
/// Configures the hardware odometer interface.
///
///@{

struct Odometer
{
    enum class Mode : uint8_t
    {
        DISABLED   = 0,  ///<  Encoder is disabled.
        QUADRATURE = 2,  ///<  Quadrature encoder mode.
    };
    
    /// Parameters
    FunctionSelector function = static_cast<FunctionSelector>(0);
    Mode mode = static_cast<Mode>(0); ///< Mode setting.
    float scaling = 0; ///< Encoder pulses per meter of distance traveled [pulses/m]. Distance traveled is computed using the formula d = p / N * 2R * pi, where d is distance, p is the number of pulses received, N is the encoder resolution, and R is the wheel radius. By simplifying all of the parameters into one, the formula d = p / S is obtained, where s is the odometer scaling factor passed to this command. S is equivalent to N / (2R * pi) and has units of pulses / meter. N is in units of "A" pulses per revolution and R is in meters. Make this value negative if the odometer is mounted so that it rotates backwards.
    float uncertainty = 0; ///< Uncertainty in encoder counts to distance translation (1-sigma value) [m/m].
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_ODOMETER_CONFIG;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "Odometer";
    static constexpr const char* DOC_NAME = "Odometer Configuration";
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    
    auto asTuple() const
    {
        return std::make_tuple(mode,scaling,uncertainty);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(mode),std::ref(scaling),std::ref(uncertainty));
    }
    
    static Odometer create_sld_all(::mip::FunctionSelector function)
    {
        Odometer cmd;
        cmd.function = function;
        return cmd;
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
    struct Response
    {
        /// Parameters
        Mode mode = static_cast<Mode>(0); ///< Mode setting.
        float scaling = 0; ///< Encoder pulses per meter of distance traveled [pulses/m]. Distance traveled is computed using the formula d = p / N * 2R * pi, where d is distance, p is the number of pulses received, N is the encoder resolution, and R is the wheel radius. By simplifying all of the parameters into one, the formula d = p / S is obtained, where s is the odometer scaling factor passed to this command. S is equivalent to N / (2R * pi) and has units of pulses / meter. N is in units of "A" pulses per revolution and R is in meters. Make this value negative if the odometer is mounted so that it rotates backwards.
        float uncertainty = 0; ///< Uncertainty in encoder counts to distance translation (1-sigma value) [m/m].
        
        /// Descriptors
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::REPLY_ODOMETER_CONFIG;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "Odometer::Response";
        static constexpr const char* DOC_NAME = "Odometer Configuration Response";
        static constexpr const bool HAS_FUNCTION_SELECTOR = false;
        
        auto asTuple() const
        {
            return std::make_tuple(mode,scaling,uncertainty);
        }
        
        auto asTuple()
        {
            return std::make_tuple(std::ref(mode),std::ref(scaling),std::ref(uncertainty));
        }
        
        /// Serialization
        void insert(Serializer& serializer) const;
        void extract(Serializer& serializer);
        
    };
};
TypedResult<Odometer> writeOdometer(C::mip_interface& device, Odometer::Mode mode, float scaling, float uncertainty);
TypedResult<Odometer> readOdometer(C::mip_interface& device, Odometer::Mode* modeOut, float* scalingOut, float* uncertaintyOut);
TypedResult<Odometer> saveOdometer(C::mip_interface& device);
TypedResult<Odometer> loadOdometer(C::mip_interface& device);
TypedResult<Odometer> defaultOdometer(C::mip_interface& device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup 3dm_imu_lowpass_filter_cpp  (0x0C,0x50) Imu Lowpass Filter
/// Advanced configuration for the IMU data quantity low-pass filters.
/// 
/// Deprecated, use the lowpass filter (0x0C,0x54) command instead.
/// 
/// The scaled data quantities are by default filtered through a single-pole IIR low-pass filter
/// which is configured with a -3dB cutoff frequency of half the reporting frequency (set by
/// decimation factor in the IMU Message Format command) to prevent aliasing on a per data
/// quantity basis. This advanced configuration command allows for the cutoff frequency to
/// be configured independently of the data reporting frequency as well as allowing for a
/// complete bypass of the digital low-pass filter.
/// 
/// Possible data descriptors:
/// 0x04 - Scaled accelerometer data
/// 0x05 - Scaled gyro data
/// 0x06 - Scaled magnetometer data (if applicable)
/// 0x17 - Scaled pressure data (if applicable)
///
///@{

struct ImuLowpassFilter
{
    /// Parameters
    FunctionSelector function = static_cast<FunctionSelector>(0);
    uint8_t target_descriptor = 0; ///< Field descriptor of filtered quantity within the Sensor data set. Supported values are accel (0x04), gyro (0x05), mag (0x06), and pressure (0x17), provided the data is supported by the device. Except with the READ function selector, this can be 0 to apply to all of the above quantities.
    bool enable = 0; ///< The target data will be filtered if this is true.
    bool manual = 0; ///< If false, the cutoff frequency is set to half of the streaming rate as configured by the message format command. Otherwise, the cutoff frequency is set according to the following 'frequency' parameter.
    uint16_t frequency = 0; ///< -3dB cutoff frequency in Hz. Will not affect filtering if 'manual' is false.
    uint8_t reserved = 0; ///< Reserved, set to 0x00.
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_IMU_LOWPASS_FILTER;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "ImuLowpassFilter";
    static constexpr const char* DOC_NAME = "Advanced Low-Pass Filter Settings";
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    
    auto asTuple() const
    {
        return std::make_tuple(target_descriptor,enable,manual,frequency,reserved);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(target_descriptor),std::ref(enable),std::ref(manual),std::ref(frequency),std::ref(reserved));
    }
    
    static ImuLowpassFilter create_sld_all(::mip::FunctionSelector function)
    {
        ImuLowpassFilter cmd;
        cmd.function = function;
        cmd.target_descriptor = 0;
        return cmd;
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
    struct Response
    {
        /// Parameters
        uint8_t target_descriptor = 0;
        bool enable = 0; ///< True if the filter is currently enabled.
        bool manual = 0; ///< True if the filter cutoff was manually configured.
        uint16_t frequency = 0; ///< The cutoff frequency of the filter. If the filter is in auto mode, this value is unspecified.
        uint8_t reserved = 0; ///< Reserved and must be ignored.
        
        /// Descriptors
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::REPLY_ADVANCED_DATA_FILTER;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "ImuLowpassFilter::Response";
        static constexpr const char* DOC_NAME = "Advanced Low-Pass Filter Settings Response";
        static constexpr const bool HAS_FUNCTION_SELECTOR = false;
        
        auto asTuple() const
        {
            return std::make_tuple(target_descriptor,enable,manual,frequency,reserved);
        }
        
        auto asTuple()
        {
            return std::make_tuple(std::ref(target_descriptor),std::ref(enable),std::ref(manual),std::ref(frequency),std::ref(reserved));
        }
        
        /// Serialization
        void insert(Serializer& serializer) const;
        void extract(Serializer& serializer);
        
    };
};
TypedResult<ImuLowpassFilter> writeImuLowpassFilter(C::mip_interface& device, uint8_t targetDescriptor, bool enable, bool manual, uint16_t frequency, uint8_t reserved);
TypedResult<ImuLowpassFilter> readImuLowpassFilter(C::mip_interface& device, uint8_t targetDescriptor, bool* enableOut, bool* manualOut, uint16_t* frequencyOut, uint8_t* reservedOut);
TypedResult<ImuLowpassFilter> saveImuLowpassFilter(C::mip_interface& device, uint8_t targetDescriptor);
TypedResult<ImuLowpassFilter> loadImuLowpassFilter(C::mip_interface& device, uint8_t targetDescriptor);
TypedResult<ImuLowpassFilter> defaultImuLowpassFilter(C::mip_interface& device, uint8_t targetDescriptor);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup 3dm_complementary_filter_cpp  (0x0C,0x51) Complementary Filter
/// Configure the settings for the complementary filter which produces the following (0x80) descriptor set values: attitude matrix (0x80,09), quaternion (0x80,0A), and  Euler angle (0x80,0C) outputs.
/// 
/// The filter can be configured to correct for pitch and roll using the accelerometer (with the assumption that linear acceleration is minimal),
/// and to correct for heading using the magnetometer (with the assumption that the local magnetic field is dominated by the Earth's own magnetic field).
/// Pitch/roll and heading corrections each have their own configurable time constants, with a valid range of 1-1000 seconds. The default time constant is 10 seconds.
///
///@{

struct ComplementaryFilter
{
    /// Parameters
    FunctionSelector function = static_cast<FunctionSelector>(0);
    bool pitch_roll_enable = 0; ///< Enable Pitch/Roll corrections
    bool heading_enable = 0; ///< Enable Heading corrections (only available on devices with magnetometer)
    float pitch_roll_time_constant = 0; ///< Time constant associated with the pitch/roll corrections [s]
    float heading_time_constant = 0; ///< Time constant associated with the heading corrections [s]
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_LEGACY_COMP_FILTER;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "ComplementaryFilter";
    static constexpr const char* DOC_NAME = "Complementary Filter Configuration";
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    
    auto asTuple() const
    {
        return std::make_tuple(pitch_roll_enable,heading_enable,pitch_roll_time_constant,heading_time_constant);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(pitch_roll_enable),std::ref(heading_enable),std::ref(pitch_roll_time_constant),std::ref(heading_time_constant));
    }
    
    static ComplementaryFilter create_sld_all(::mip::FunctionSelector function)
    {
        ComplementaryFilter cmd;
        cmd.function = function;
        return cmd;
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
    struct Response
    {
        /// Parameters
        bool pitch_roll_enable = 0; ///< Enable Pitch/Roll corrections
        bool heading_enable = 0; ///< Enable Heading corrections (only available on devices with magnetometer)
        float pitch_roll_time_constant = 0; ///< Time constant associated with the pitch/roll corrections [s]
        float heading_time_constant = 0; ///< Time constant associated with the heading corrections [s]
        
        /// Descriptors
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::REPLY_LEGACY_COMP_FILTER;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "ComplementaryFilter::Response";
        static constexpr const char* DOC_NAME = "Complementary Filter Configuration Response";
        static constexpr const bool HAS_FUNCTION_SELECTOR = false;
        
        auto asTuple() const
        {
            return std::make_tuple(pitch_roll_enable,heading_enable,pitch_roll_time_constant,heading_time_constant);
        }
        
        auto asTuple()
        {
            return std::make_tuple(std::ref(pitch_roll_enable),std::ref(heading_enable),std::ref(pitch_roll_time_constant),std::ref(heading_time_constant));
        }
        
        /// Serialization
        void insert(Serializer& serializer) const;
        void extract(Serializer& serializer);
        
    };
};
TypedResult<ComplementaryFilter> writeComplementaryFilter(C::mip_interface& device, bool pitchRollEnable, bool headingEnable, float pitchRollTimeConstant, float headingTimeConstant);
TypedResult<ComplementaryFilter> readComplementaryFilter(C::mip_interface& device, bool* pitchRollEnableOut, bool* headingEnableOut, float* pitchRollTimeConstantOut, float* headingTimeConstantOut);
TypedResult<ComplementaryFilter> saveComplementaryFilter(C::mip_interface& device);
TypedResult<ComplementaryFilter> loadComplementaryFilter(C::mip_interface& device);
TypedResult<ComplementaryFilter> defaultComplementaryFilter(C::mip_interface& device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup 3dm_sensor_range_cpp  (0x0C,0x52) Sensor Range
/// Changes the IMU sensor gain.
/// 
/// This allows you to optimize the range to get the best accuracy and performance
/// while minimizing over-range events.
/// 
/// Use the 3DM Get Calibrated Sensor Ranges (0x0C,0x53) command to determine
/// the appropriate setting value for your application. Using values other than
/// those specified may result in a NACK or inaccurate measurement data.
///
///@{

struct SensorRange
{
    /// Parameters
    FunctionSelector function = static_cast<FunctionSelector>(0);
    SensorRangeType sensor = static_cast<SensorRangeType>(0); ///< Which type of sensor will get the new range value.
    uint8_t setting = 0; ///< Use the 3DM Get Calibrated Sensor Ranges (0x0C,0x53) command to determine this value.
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_SENSOR_RANGE;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "SensorRange";
    static constexpr const char* DOC_NAME = "Sensor Range";
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    
    auto asTuple() const
    {
        return std::make_tuple(sensor,setting);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(sensor),std::ref(setting));
    }
    
    static SensorRange create_sld_all(::mip::FunctionSelector function)
    {
        SensorRange cmd;
        cmd.function = function;
        cmd.sensor = ::mip::commands_3dm::SensorRangeType::ALL;
        return cmd;
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
    struct Response
    {
        /// Parameters
        SensorRangeType sensor = static_cast<SensorRangeType>(0); ///< Which type of sensor will get the new range value.
        uint8_t setting = 0; ///< Use the 3DM Get Calibrated Sensor Ranges (0x0C,0x53) command to determine this value.
        
        /// Descriptors
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::REPLY_SENSOR_RANGE;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "SensorRange::Response";
        static constexpr const char* DOC_NAME = "Sensor Range Response";
        static constexpr const bool HAS_FUNCTION_SELECTOR = false;
        
        auto asTuple() const
        {
            return std::make_tuple(sensor,setting);
        }
        
        auto asTuple()
        {
            return std::make_tuple(std::ref(sensor),std::ref(setting));
        }
        
        /// Serialization
        void insert(Serializer& serializer) const;
        void extract(Serializer& serializer);
        
    };
};
TypedResult<SensorRange> writeSensorRange(C::mip_interface& device, SensorRangeType sensor, uint8_t setting);
TypedResult<SensorRange> readSensorRange(C::mip_interface& device, SensorRangeType sensor, uint8_t* settingOut);
TypedResult<SensorRange> saveSensorRange(C::mip_interface& device, SensorRangeType sensor);
TypedResult<SensorRange> loadSensorRange(C::mip_interface& device, SensorRangeType sensor);
TypedResult<SensorRange> defaultSensorRange(C::mip_interface& device, SensorRangeType sensor);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup 3dm_calibrated_sensor_ranges_cpp  (0x0C,0x53) Calibrated Sensor Ranges
/// Returns the supported sensor ranges which may be used with the 3DM Sensor Range (0x0C,0x52) command.
/// 
/// The response includes an array of (u8, float) pairs which map each allowed setting
/// to the corresponding maximum range in physical units. See SensorRangeType for units.
///
///@{

struct CalibratedSensorRanges
{
    struct Entry
    {
        /// Parameters
        uint8_t setting = 0; ///< The value used in the 3DM Sensor Range command and response.
        float range = 0; ///< The actual range value. Units depend on the sensor type.
        
        /// Serialization
        void insert(Serializer& serializer) const;
        void extract(Serializer& serializer);
        
    };
    /// Parameters
    SensorRangeType sensor = static_cast<SensorRangeType>(0); ///< The sensor to query. Cannot be ALL.
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_CALIBRATED_RANGES;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "CalibratedSensorRanges";
    static constexpr const char* DOC_NAME = "Get Calibrated Sensor Ranges";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(sensor);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(sensor));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
    struct Response
    {
        /// Parameters
        SensorRangeType sensor = static_cast<SensorRangeType>(0); ///< The sensor type from the command.
        uint8_t num_ranges = 0; ///< Number of supported ranges.
        Entry ranges[50]; ///< List of possible range settings.
        
        /// Descriptors
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::REPLY_CALIBRATED_RANGES;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "CalibratedSensorRanges::Response";
        static constexpr const char* DOC_NAME = "Get Calibrated Sensor Ranges Response";
        static constexpr const bool HAS_FUNCTION_SELECTOR = false;
        
        auto asTuple() const
        {
            return std::make_tuple(sensor,num_ranges,ranges);
        }
        
        auto asTuple()
        {
            return std::make_tuple(std::ref(sensor),std::ref(num_ranges),std::ref(ranges));
        }
        
        /// Serialization
        void insert(Serializer& serializer) const;
        void extract(Serializer& serializer);
        
    };
};
TypedResult<CalibratedSensorRanges> calibratedSensorRanges(C::mip_interface& device, SensorRangeType sensor, uint8_t* numRangesOut, uint8_t numRangesOutMax, CalibratedSensorRanges::Entry* rangesOut);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup 3dm_lowpass_filter_cpp  (0x0C,0x54) Lowpass Filter
/// This command controls the low-pass anti-aliasing filter supported data quantities.
/// 
/// See the device user manual for data quantities which support the anti-aliasing filter.
/// 
/// If set to automatic mode, the frequency will track half of the transmission rate
/// of the target descriptor according to the configured message format (0x0C,0x0F).
/// For example, if scaled accel (0x80,0x04) is set to stream at 100 Hz, the filter would
/// be set to 50 Hz. Changing the message format to 200 Hz would automatically adjust the
/// filter to 100 Hz.
/// 
/// For WRITE, SAVE, LOAD, and DEFAULT function selectors, the descriptor set and/or field descriptor
/// may be 0x00 to set, save, load, or reset the setting for all supported descriptors. The
/// field descriptor must be 0x00 if the descriptor set is 0x00.
///
///@{

struct LowpassFilter
{
    /// Parameters
    FunctionSelector function = static_cast<FunctionSelector>(0);
    uint8_t desc_set = 0; ///< Descriptor set of the quantity to be filtered.
    uint8_t field_desc = 0; ///< Field descriptor of the quantity to be filtered.
    bool enable = 0; ///< The filter will be enabled if this is true.
    bool manual = 0; ///< If false, the frequency parameter is ignored and the filter will track to half of the configured message format frequency.
    float frequency = 0; ///< Cutoff frequency in Hz. This will return the actual frequency when read out in automatic mode.
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_LOWPASS_FILTER;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "LowpassFilter";
    static constexpr const char* DOC_NAME = "Low-Pass Anti-Aliasing Filter";
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    
    auto asTuple() const
    {
        return std::make_tuple(desc_set,field_desc,enable,manual,frequency);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(desc_set),std::ref(field_desc),std::ref(enable),std::ref(manual),std::ref(frequency));
    }
    
    static LowpassFilter create_sld_all(::mip::FunctionSelector function)
    {
        LowpassFilter cmd;
        cmd.function = function;
        cmd.desc_set = 0;
        cmd.field_desc = 0;
        return cmd;
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
    struct Response
    {
        /// Parameters
        uint8_t desc_set = 0; ///< Descriptor set of the quantity to be filtered.
        uint8_t field_desc = 0; ///< Field descriptor of the quantity to be filtered.
        bool enable = 0; ///< The filter will be enabled if this is true.
        bool manual = 0; ///< If false, the frequency parameter is ignored and the filter will track to half of the configured message format frequency.
        float frequency = 0; ///< Cutoff frequency in Hz. This will return the actual frequency when read out in automatic mode.
        
        /// Descriptors
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::REPLY_LOWPASS_FILTER;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "LowpassFilter::Response";
        static constexpr const char* DOC_NAME = "Low-Pass Anti-Aliasing Filter Response";
        static constexpr const bool HAS_FUNCTION_SELECTOR = false;
        
        auto asTuple() const
        {
            return std::make_tuple(desc_set,field_desc,enable,manual,frequency);
        }
        
        auto asTuple()
        {
            return std::make_tuple(std::ref(desc_set),std::ref(field_desc),std::ref(enable),std::ref(manual),std::ref(frequency));
        }
        
        /// Serialization
        void insert(Serializer& serializer) const;
        void extract(Serializer& serializer);
        
    };
};
TypedResult<LowpassFilter> writeLowpassFilter(C::mip_interface& device, uint8_t descSet, uint8_t fieldDesc, bool enable, bool manual, float frequency);
TypedResult<LowpassFilter> readLowpassFilter(C::mip_interface& device, uint8_t descSet, uint8_t fieldDesc, bool* enableOut, bool* manualOut, float* frequencyOut);
TypedResult<LowpassFilter> saveLowpassFilter(C::mip_interface& device, uint8_t descSet, uint8_t fieldDesc);
TypedResult<LowpassFilter> loadLowpassFilter(C::mip_interface& device, uint8_t descSet, uint8_t fieldDesc);
TypedResult<LowpassFilter> defaultLowpassFilter(C::mip_interface& device, uint8_t descSet, uint8_t fieldDesc);

///@}
///

///@}
///@}
///
////////////////////////////////////////////////////////////////////////////////
} // namespace commands_3dm
} // namespace mip

