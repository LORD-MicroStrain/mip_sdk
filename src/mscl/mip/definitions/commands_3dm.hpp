#pragma once

#include "descriptors.h"
#include "../mip_result.h"

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

namespace mip {
class Serializer;

namespace C {
struct mip_interface;
} // namespace C

namespace commands_3dm {

////////////////////////////////////////////////////////////////////////////////
///@addtogroup MipCommands
///@{
///@defgroup 3dm_commands_cpp  3DMCommands
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
    CMD_RAW_RTCM_2_3_MESSAGE            = 0x20,
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
    CMD_SET_GNSS_DYNAMICS_MODE          = 0x34,
    CMD_SET_IMU_SIGNAL_COND             = 0x35,
    CMD_SET_IMU_TIMESTAMP               = 0x36,
    CMD_ACCEL_BIAS                      = 0x37,
    CMD_GYRO_BIAS                       = 0x38,
    CMD_CAPTURE_GYRO_BIAS               = 0x39,
    CMD_HARD_IRON_OFFSET                = 0x3A,
    CMD_SOFT_IRON_MATRIX                = 0x3B,
    CMD_REALIGN_UP                      = 0x3C,
    CMD_REALIGN_NORTH                   = 0x3D,
    CMD_CONING_AND_SCULLING_ENABLE      = 0x3E,
    CMD_UART_BAUDRATE                   = 0x40,
    CMD_GPIO_CONFIG                     = 0x41,
    CMD_GPIO_STATE                      = 0x42,
    CMD_ODOMETER_CONFIG                 = 0x43,
    CMD_ADVANCED_DATA_FILTER            = 0x50,
    CMD_LEGACY_COMP_FILTER              = 0x51,
    CMD_SENSOR_RANGE                    = 0x52,
    CMD_CALIBRATED_RANGES               = 0x53,
    CMD_DATASTREAM_FORMAT               = 0x60,
    CMD_DEVICE_POWER_STATE              = 0x61,
    CMD_SAVE_RESTORE_GPS_SETTINGS       = 0x62,
    CMD_DEVICE_SETTINGS                 = 0x63,
    CMD_RAW_CLIP_SETTINGS               = 0x70,
    
    REPLY_IMU_MESSAGE_FORMAT            = 0x80,
    REPLY_GNSS_MESSAGE_FORMAT           = 0x81,
    REPLY_FILTER_MESSAGE_FORMAT         = 0x82,
    REPLY_IMU_BASE_RATE                 = 0x83,
    REPLY_GNSS_BASE_RATE                = 0x84,
    REPLY_DATASTREAM_ENABLE             = 0x85,
    REPLY_IMU_SIGNAL_SETTINGS           = 0x86,
    REPLY_UART_BAUDRATE                 = 0x87,
    REPLY_DATASTREAM_FORMAT             = 0x88,
    REPLY_POWER_STATE                   = 0x89,
    REPLY_FILTER_BASE_RATE              = 0x8A,
    REPLY_ADVANCED_DATA_FILTER          = 0x8B,
    REPLY_POLL_DATA                     = 0x8D,
    REPLY_BASE_RATE                     = 0x8E,
    REPLY_MESSAGE_FORMAT                = 0x8F,
    REPLY_COMMUNICATIONS_MODE           = 0x91,
    REPLY_GNSS_DYNAMICS_MODE            = 0x92,
    REPLY_IMU_TIMESTAMP_VALUE           = 0x93,
    REPLY_IMU_BASIC_STATUS              = 0x94,
    REPLY_IMU_ADVANCED_STATUS           = 0x95,
    REPLY_RAW_CLIP_SETTINGS             = 0x96,
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
};

////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////

struct NMEAMessageFormat
{
    enum class MessageID : uint8_t
    {
        GGA  = 1,  ///<  GPS System Fix Data
        GLL  = 2,  ///<  Geographic Position Lat/Lon
        GSV  = 3,  ///<  GNSS Satellites in View
        RMC  = 4,  ///<  Recommended Minimum Specific GNSS Data
        VTG  = 5,  ///<  Course over Ground
        HDT  = 6,  ///<  Heading, True
        ZDA  = 7,  ///<  Time & Date
        PRKA = 100,  ///<  Parker proprietary Euler angles
        PRKR = 101,  ///<  Parker proprietary Angular Rate/Acceleration
    };
    
    enum class TalkerID : uint8_t
    {
        GNSS    = 1,  ///<  NMEA message will be produced with talker id "GN"
        GPS     = 2,  ///<  NMEA message will be produced with talker id "GP"
        GALILEO = 3,  ///<  NMEA message will be produced with talker id "GA"
        GLONASS = 4,  ///<  NMEA message will be produced with talker id "GL"
    };
    
    enum class SourceID : uint8_t
    {
        FILTER = 1,  ///<  Data from the Kalman filter will be used to generate the NMEA message
        IMU    = 2,  ///<  Data from the IMU/IMU derived quantities will be used to generate the NMEA message
        GNSS1  = 3,  ///<  Data from GNSS1 will be used to generate the NMEA message
        GNSS2  = 4,  ///<  Data from GNSS2 will be used to generate the NMEA message
    };
    
    MessageID message_id;
    TalkerID talker_id;
    SourceID source_id;
    uint16_t decimation;
    
};
void insert(Serializer& serializer, const NMEAMessageFormat& self);
void extract(Serializer& serializer, NMEAMessageFormat& self);

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
///@defgroup cpp_poll_imu_message  None
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
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_POLL_IMU_MESSAGE;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    bool suppress_ack;
    uint8_t num_descriptors;
    DescriptorRate* descriptors;
    
};
void insert(Serializer& serializer, const PollImuMessage& self);
void extract(Serializer& serializer, PollImuMessage& self);

CmdResult pollImuMessage(C::mip_interface& device, bool suppress_ack, uint8_t num_descriptors, const DescriptorRate* descriptors);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_poll_gnss_message  None
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
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_POLL_GNSS_MESSAGE;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    bool suppress_ack;
    uint8_t num_descriptors;
    DescriptorRate* descriptors;
    
};
void insert(Serializer& serializer, const PollGnssMessage& self);
void extract(Serializer& serializer, PollGnssMessage& self);

CmdResult pollGnssMessage(C::mip_interface& device, bool suppress_ack, uint8_t num_descriptors, const DescriptorRate* descriptors);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_poll_filter_message  None
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
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_POLL_FILTER_MESSAGE;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    bool suppress_ack;
    uint8_t num_descriptors;
    DescriptorRate* descriptors;
    
};
void insert(Serializer& serializer, const PollFilterMessage& self);
void extract(Serializer& serializer, PollFilterMessage& self);

CmdResult pollFilterMessage(C::mip_interface& device, bool suppress_ack, uint8_t num_descriptors, const DescriptorRate* descriptors);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_imu_message_format  None
/// Set, read, or save the format of the IMU data packet.
/// 
/// The resulting data messages will maintain the order of descriptors sent in the command.
///
///@{

struct ImuMessageFormat
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_IMU_MESSAGE_FORMAT;
    
    static const bool HAS_WRITE_FUNCTION = true;
    static const bool HAS_READ_FUNCTION = true;
    static const bool HAS_SAVE_FUNCTION = true;
    static const bool HAS_LOAD_FUNCTION = true;
    static const bool HAS_RESET_FUNCTION = true;
    
    FunctionSelector function;
    uint8_t num_descriptors;
    DescriptorRate* descriptors;
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::REPLY_IMU_MESSAGE_FORMAT;
        
        uint8_t num_descriptors;
        DescriptorRate* descriptors;
        
    };
};
void insert(Serializer& serializer, const ImuMessageFormat& self);
void extract(Serializer& serializer, ImuMessageFormat& self);

void insert(Serializer& serializer, const ImuMessageFormat::Response& self);
void extract(Serializer& serializer, ImuMessageFormat::Response& self);

CmdResult writeImuMessageFormat(C::mip_interface& device, uint8_t num_descriptors, const DescriptorRate* descriptors);
CmdResult readImuMessageFormat(C::mip_interface& device, uint8_t& num_descriptors, DescriptorRate* descriptors);
CmdResult saveImuMessageFormat(C::mip_interface& device);
CmdResult loadImuMessageFormat(C::mip_interface& device);
CmdResult defaultImuMessageFormat(C::mip_interface& device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_gps_message_format  None
/// Set, read, or save the format of the GNSS data packet.
/// 
/// The resulting data messages will maintain the order of descriptors sent in the command.
///
///@{

struct GpsMessageFormat
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_GNSS_MESSAGE_FORMAT;
    
    static const bool HAS_WRITE_FUNCTION = true;
    static const bool HAS_READ_FUNCTION = true;
    static const bool HAS_SAVE_FUNCTION = true;
    static const bool HAS_LOAD_FUNCTION = true;
    static const bool HAS_RESET_FUNCTION = true;
    
    FunctionSelector function;
    uint8_t num_descriptors;
    DescriptorRate* descriptors;
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::REPLY_GNSS_MESSAGE_FORMAT;
        
        uint8_t num_descriptors;
        DescriptorRate* descriptors;
        
    };
};
void insert(Serializer& serializer, const GpsMessageFormat& self);
void extract(Serializer& serializer, GpsMessageFormat& self);

void insert(Serializer& serializer, const GpsMessageFormat::Response& self);
void extract(Serializer& serializer, GpsMessageFormat::Response& self);

CmdResult writeGpsMessageFormat(C::mip_interface& device, uint8_t num_descriptors, const DescriptorRate* descriptors);
CmdResult readGpsMessageFormat(C::mip_interface& device, uint8_t& num_descriptors, DescriptorRate* descriptors);
CmdResult saveGpsMessageFormat(C::mip_interface& device);
CmdResult loadGpsMessageFormat(C::mip_interface& device);
CmdResult defaultGpsMessageFormat(C::mip_interface& device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_message_format  None
/// Set, read, or save the format of the Estimation Filter data packet.
/// 
/// The resulting data messages will maintain the order of descriptors sent in the command.
///
///@{

struct FilterMessageFormat
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_FILTER_MESSAGE_FORMAT;
    
    static const bool HAS_WRITE_FUNCTION = true;
    static const bool HAS_READ_FUNCTION = true;
    static const bool HAS_SAVE_FUNCTION = true;
    static const bool HAS_LOAD_FUNCTION = true;
    static const bool HAS_RESET_FUNCTION = true;
    
    FunctionSelector function;
    uint8_t num_descriptors;
    DescriptorRate* descriptors;
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::REPLY_FILTER_MESSAGE_FORMAT;
        
        uint8_t num_descriptors;
        DescriptorRate* descriptors;
        
    };
};
void insert(Serializer& serializer, const FilterMessageFormat& self);
void extract(Serializer& serializer, FilterMessageFormat& self);

void insert(Serializer& serializer, const FilterMessageFormat::Response& self);
void extract(Serializer& serializer, FilterMessageFormat::Response& self);

CmdResult writeFilterMessageFormat(C::mip_interface& device, uint8_t num_descriptors, const DescriptorRate* descriptors);
CmdResult readFilterMessageFormat(C::mip_interface& device, uint8_t& num_descriptors, DescriptorRate* descriptors);
CmdResult saveFilterMessageFormat(C::mip_interface& device);
CmdResult loadFilterMessageFormat(C::mip_interface& device);
CmdResult defaultFilterMessageFormat(C::mip_interface& device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_imu_get_base_rate  Get IMU Data Base Rate
/// Get the base rate for the IMU data in Hz
/// 
/// This is the fastest rate for this type of data available on the device.
/// This is used in conjunction with the IMU Message Format Command to set streaming data at a specified rate.
///
///@{

struct ImuGetBaseRate
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_GET_IMU_BASE_RATE;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::REPLY_IMU_BASE_RATE;
        
        uint16_t rate;
        
    };
};
void insert(Serializer& serializer, const ImuGetBaseRate& self);
void extract(Serializer& serializer, ImuGetBaseRate& self);

void insert(Serializer& serializer, const ImuGetBaseRate::Response& self);
void extract(Serializer& serializer, ImuGetBaseRate::Response& self);

CmdResult imuGetBaseRate(C::mip_interface& device, uint16_t& rate);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_gps_get_base_rate  Get GNSS Data Base Rate
/// Get the base rate for the GNSS data in Hz
/// 
/// This is the fastest rate for this type of data available on the device.
/// This is used in conjunction with the GNSS Message Format Command to set streaming data at a specified rate.
///
///@{

struct GpsGetBaseRate
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_GET_GNSS_BASE_RATE;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::REPLY_GNSS_BASE_RATE;
        
        uint16_t rate;
        
    };
};
void insert(Serializer& serializer, const GpsGetBaseRate& self);
void extract(Serializer& serializer, GpsGetBaseRate& self);

void insert(Serializer& serializer, const GpsGetBaseRate::Response& self);
void extract(Serializer& serializer, GpsGetBaseRate::Response& self);

CmdResult gpsGetBaseRate(C::mip_interface& device, uint16_t& rate);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_get_base_rate  Get Estimation Filter Data Base Rate
/// Get the base rate for the Estimation Filter data in Hz
/// 
/// This is the fastest rate for this type of data available on the device.
/// This is used in conjunction with the Estimation Filter Message Format Command to set streaming data at a specified rate.
///
///@{

struct FilterGetBaseRate
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_GET_FILTER_BASE_RATE;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::REPLY_FILTER_BASE_RATE;
        
        uint16_t rate;
        
    };
};
void insert(Serializer& serializer, const FilterGetBaseRate& self);
void extract(Serializer& serializer, FilterGetBaseRate& self);

void insert(Serializer& serializer, const FilterGetBaseRate::Response& self);
void extract(Serializer& serializer, FilterGetBaseRate::Response& self);

CmdResult filterGetBaseRate(C::mip_interface& device, uint16_t& rate);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_poll_data  None
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
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_POLL_DATA;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    uint8_t desc_set;
    bool suppress_ack;
    uint8_t num_descriptors;
    uint8_t* descriptors;
    
};
void insert(Serializer& serializer, const PollData& self);
void extract(Serializer& serializer, PollData& self);

CmdResult pollData(C::mip_interface& device, uint8_t desc_set, bool suppress_ack, uint8_t num_descriptors, const uint8_t* descriptors);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_get_base_rate  Get Data Base Rate
/// Get the base rate for the specified descriptor set in Hz.
///
///@{

struct GetBaseRate
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_GET_BASE_RATE;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    uint8_t desc_set;
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::REPLY_BASE_RATE;
        
        uint8_t desc_set;
        uint16_t rate;
        
    };
};
void insert(Serializer& serializer, const GetBaseRate& self);
void extract(Serializer& serializer, GetBaseRate& self);

void insert(Serializer& serializer, const GetBaseRate::Response& self);
void extract(Serializer& serializer, GetBaseRate::Response& self);

CmdResult getBaseRate(C::mip_interface& device, uint8_t desc_set, uint16_t& rate);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_message_format  None
/// Set, read, or save the format for a given data packet.
/// 
/// The resulting data messages will maintain the order of descriptors sent in the command.
///
///@{

struct MessageFormat
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_MESSAGE_FORMAT;
    
    static const bool HAS_WRITE_FUNCTION = true;
    static const bool HAS_READ_FUNCTION = true;
    static const bool HAS_SAVE_FUNCTION = true;
    static const bool HAS_LOAD_FUNCTION = true;
    static const bool HAS_RESET_FUNCTION = true;
    
    FunctionSelector function;
    uint8_t desc_set;
    uint8_t num_descriptors;
    DescriptorRate* descriptors;
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::REPLY_MESSAGE_FORMAT;
        
        uint8_t desc_set;
        uint8_t num_descriptors;
        DescriptorRate* descriptors;
        
    };
};
void insert(Serializer& serializer, const MessageFormat& self);
void extract(Serializer& serializer, MessageFormat& self);

void insert(Serializer& serializer, const MessageFormat::Response& self);
void extract(Serializer& serializer, MessageFormat::Response& self);

CmdResult writeMessageFormat(C::mip_interface& device, uint8_t desc_set, uint8_t num_descriptors, const DescriptorRate* descriptors);
CmdResult readMessageFormat(C::mip_interface& device, uint8_t desc_set, uint8_t& num_descriptors, DescriptorRate* descriptors);
CmdResult saveMessageFormat(C::mip_interface& device, uint8_t desc_set);
CmdResult loadMessageFormat(C::mip_interface& device, uint8_t desc_set);
CmdResult defaultMessageFormat(C::mip_interface& device, uint8_t desc_set);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_nmea_poll_data  None
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
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_POLL_NMEA_MESSAGE;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    bool suppress_ack;
    uint8_t count;
    NMEAMessageFormat* format_entries;
    
};
void insert(Serializer& serializer, const NmeaPollData& self);
void extract(Serializer& serializer, NmeaPollData& self);

CmdResult nmeaPollData(C::mip_interface& device, bool suppress_ack, uint8_t count, const NMEAMessageFormat* format_entries);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_nmea_message_format  None
/// Set, read, or save the NMEA message format.
///
///@{

struct NmeaMessageFormat
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_NMEA_MESSAGE_FORMAT;
    
    static const bool HAS_WRITE_FUNCTION = true;
    static const bool HAS_READ_FUNCTION = true;
    static const bool HAS_SAVE_FUNCTION = true;
    static const bool HAS_LOAD_FUNCTION = true;
    static const bool HAS_RESET_FUNCTION = true;
    
    FunctionSelector function;
    uint8_t count;
    NMEAMessageFormat* format_entries;
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::REPLY_NMEA_MESSAGE_FORMAT;
        
        uint8_t count;
        NMEAMessageFormat* format_entries;
        
    };
};
void insert(Serializer& serializer, const NmeaMessageFormat& self);
void extract(Serializer& serializer, NmeaMessageFormat& self);

void insert(Serializer& serializer, const NmeaMessageFormat::Response& self);
void extract(Serializer& serializer, NmeaMessageFormat::Response& self);

CmdResult writeNmeaMessageFormat(C::mip_interface& device, uint8_t count, const NMEAMessageFormat* format_entries);
CmdResult readNmeaMessageFormat(C::mip_interface& device, uint8_t& count, NMEAMessageFormat* format_entries);
CmdResult saveNmeaMessageFormat(C::mip_interface& device);
CmdResult loadNmeaMessageFormat(C::mip_interface& device);
CmdResult defaultNmeaMessageFormat(C::mip_interface& device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_device_settings  None
/// Save, Load, or Reset to Default the values for all device settings.
/// 
/// When a save current settings command is issued, a brief data disturbance may occur while all settings are written to non-volatile memory.
/// 
/// This command should have a long timeout as it may take up to 1 second to complete.
///
///@{

struct DeviceSettings
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_DEVICE_STARTUP_SETTINGS;
    
    static const bool HAS_WRITE_FUNCTION = false;
    static const bool HAS_READ_FUNCTION = false;
    static const bool HAS_SAVE_FUNCTION = true;
    static const bool HAS_LOAD_FUNCTION = true;
    static const bool HAS_RESET_FUNCTION = true;
    
    FunctionSelector function;
    
};
void insert(Serializer& serializer, const DeviceSettings& self);
void extract(Serializer& serializer, DeviceSettings& self);

CmdResult saveDeviceSettings(C::mip_interface& device);
CmdResult loadDeviceSettings(C::mip_interface& device);
CmdResult defaultDeviceSettings(C::mip_interface& device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_uart_baudrate  None
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
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_UART_BAUDRATE;
    
    static const bool HAS_WRITE_FUNCTION = true;
    static const bool HAS_READ_FUNCTION = true;
    static const bool HAS_SAVE_FUNCTION = true;
    static const bool HAS_LOAD_FUNCTION = true;
    static const bool HAS_RESET_FUNCTION = true;
    
    FunctionSelector function;
    uint32_t baud;
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::REPLY_UART_BAUDRATE;
        
        uint32_t baud;
        
    };
};
void insert(Serializer& serializer, const UartBaudrate& self);
void extract(Serializer& serializer, UartBaudrate& self);

void insert(Serializer& serializer, const UartBaudrate::Response& self);
void extract(Serializer& serializer, UartBaudrate::Response& self);

CmdResult writeUartBaudrate(C::mip_interface& device, uint32_t baud);
CmdResult readUartBaudrate(C::mip_interface& device, uint32_t& baud);
CmdResult saveUartBaudrate(C::mip_interface& device);
CmdResult loadUartBaudrate(C::mip_interface& device);
CmdResult defaultUartBaudrate(C::mip_interface& device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_factory_streaming  None
/// Configures the device for recording data for technical support.
/// 
/// This command will configure all available data streams to predefined
/// formats designed to be used with technical support.
///
///@{

struct FactoryStreaming
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_CONFIGURE_FACTORY_STREAMING;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    enum class Action : uint8_t
    {
        OVERWRITE = 0,  ///<  Replaces the message format(s), removing any existing descriptors.
        MERGE     = 1,  ///<  Merges support descriptors into existing format(s). May reorder descriptors.
        ADD       = 2,  ///<  Adds descriptors to the current message format(s) without changing existing descriptors. May result in duplicates.
    };
    
    Action action;
    uint8_t reserved;
    
};
void insert(Serializer& serializer, const FactoryStreaming& self);
void extract(Serializer& serializer, FactoryStreaming& self);

CmdResult factoryStreaming(C::mip_interface& device, FactoryStreaming::Action action, uint8_t reserved);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_datastream_control  None
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
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_CONTROL_DATA_STREAM;
    
    static const bool HAS_WRITE_FUNCTION = true;
    static const bool HAS_READ_FUNCTION = true;
    static const bool HAS_SAVE_FUNCTION = true;
    static const bool HAS_LOAD_FUNCTION = true;
    static const bool HAS_RESET_FUNCTION = true;
    
    static const uint8_t LEGACY_IMU_STREAM = 0x01;
    static const uint8_t LEGACY_GNSS_STREAM = 0x02;
    static const uint8_t LEGACY_FILTER_STREAM = 0x03;
    static const uint8_t ALL_STREAMS = 0x00;
    FunctionSelector function;
    uint8_t desc_set;
    bool enable;
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::REPLY_DATASTREAM_ENABLE;
        
        uint8_t desc_set;
        bool enabled;
        
    };
};
void insert(Serializer& serializer, const DatastreamControl& self);
void extract(Serializer& serializer, DatastreamControl& self);

void insert(Serializer& serializer, const DatastreamControl::Response& self);
void extract(Serializer& serializer, DatastreamControl::Response& self);

CmdResult writeDatastreamControl(C::mip_interface& device, uint8_t desc_set, bool enable);
CmdResult readDatastreamControl(C::mip_interface& device, uint8_t desc_set, bool& enabled);
CmdResult saveDatastreamControl(C::mip_interface& device, uint8_t desc_set);
CmdResult loadDatastreamControl(C::mip_interface& device, uint8_t desc_set);
CmdResult defaultDatastreamControl(C::mip_interface& device, uint8_t desc_set);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_gnss_sbas_settings  SBAS Settings
/// Configure the SBAS subsystem
/// 
/// 
/// 
///
///@{

struct GnssSbasSettings
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_GNSS_SBAS_SETTINGS;
    
    static const bool HAS_WRITE_FUNCTION = true;
    static const bool HAS_READ_FUNCTION = true;
    static const bool HAS_SAVE_FUNCTION = true;
    static const bool HAS_LOAD_FUNCTION = true;
    static const bool HAS_RESET_FUNCTION = true;
    
    struct SBASOptions : Bitfield<SBASOptions>
    {
        enum _enumType : uint16_t
        {
            NONE               = 0x0000,
            ENABLE_RANGING     = 0x0001,
            ENABLE_CORRECTIONS = 0x0002,
            APPLY_INTEGRITY    = 0x0004,
        };
        uint16_t value = NONE;
        
        operator uint16_t() const { return value; }
        SBASOptions& operator=(uint16_t val) { value = val; return *this; }
        SBASOptions& operator|=(uint16_t val) { return *this = value | val; }
        SBASOptions& operator&=(uint16_t val) { return *this = value & val; }
    };
    
    FunctionSelector function;
    uint8_t enable_sbas;
    SBASOptions sbas_options;
    uint8_t num_included_prns;
    uint16_t* included_prns;
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::REPLY_GNSS_SBAS_SETTINGS;
        
        uint8_t enable_sbas;
        SBASOptions sbas_options;
        uint8_t num_included_prns;
        uint16_t* included_prns;
        
    };
};
void insert(Serializer& serializer, const GnssSbasSettings& self);
void extract(Serializer& serializer, GnssSbasSettings& self);

void insert(Serializer& serializer, const GnssSbasSettings::Response& self);
void extract(Serializer& serializer, GnssSbasSettings::Response& self);

CmdResult writeGnssSbasSettings(C::mip_interface& device, uint8_t enable_sbas, GnssSbasSettings::SBASOptions sbas_options, uint8_t num_included_prns, const uint16_t* included_prns);
CmdResult readGnssSbasSettings(C::mip_interface& device, uint8_t& enable_sbas, GnssSbasSettings::SBASOptions& sbas_options, uint8_t& num_included_prns, uint16_t* included_prns);
CmdResult saveGnssSbasSettings(C::mip_interface& device);
CmdResult loadGnssSbasSettings(C::mip_interface& device);
CmdResult defaultGnssSbasSettings(C::mip_interface& device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_gnss_time_assistance  None
/// Provide the GNSS subsystem with initial time information.
/// 
/// This message is required immediately after power up if GNSS Assist was enabled when the device was powered off.
/// This will initialize the subsystem clock to help reduce the time to first fix (TTFF).
///
///@{

struct GnssTimeAssistance
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_GNSS_TIME_ASSISTANCE;
    
    static const bool HAS_WRITE_FUNCTION = true;
    static const bool HAS_READ_FUNCTION = true;
    static const bool HAS_SAVE_FUNCTION = false;
    static const bool HAS_LOAD_FUNCTION = false;
    static const bool HAS_RESET_FUNCTION = false;
    
    FunctionSelector function;
    double tow;
    uint16_t week_number;
    float accuracy;
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::REPLY_GNSS_TIME_ASSISTANCE;
        
        double tow;
        uint16_t week_number;
        float accuracy;
        
    };
};
void insert(Serializer& serializer, const GnssTimeAssistance& self);
void extract(Serializer& serializer, GnssTimeAssistance& self);

void insert(Serializer& serializer, const GnssTimeAssistance::Response& self);
void extract(Serializer& serializer, GnssTimeAssistance::Response& self);

CmdResult writeGnssTimeAssistance(C::mip_interface& device, double tow, uint16_t week_number, float accuracy);
CmdResult readGnssTimeAssistance(C::mip_interface& device, double& tow, uint16_t& week_number, float& accuracy);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_adv_lowpass_filter  Advanced Low-Pass Filter Settings
/// Advanced configuration for the IMU data quantity low-pass filters.
/// 
/// The scaled data quantities are by default filtered through a single-pole IIR low-pass filter
/// which is configured with a -3dB cutoff frequency of half the reporting frequency (set by
/// decimation factor in the IMU Message Format command) to prevent aliasing on a per data
/// quantity basis. This advanced configuration command allows for the cutoff frequency to
/// be configured independently of the data reporting frequency as well as allowing for a
/// complete bypass of the digital low-pass filter.
/// 
/// Possible data descriptors:
/// 0x04 – Scaled accelerometer data
/// 0x05 – Scaled gyro data
/// 0x06 – Scaled magnetometer data (if applicable)
/// 0x17 – Scaled pressure data (if applicable)
///
///@{

struct AdvLowpassFilter
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_ADVANCED_DATA_FILTER;
    
    static const bool HAS_WRITE_FUNCTION = true;
    static const bool HAS_READ_FUNCTION = true;
    static const bool HAS_SAVE_FUNCTION = true;
    static const bool HAS_LOAD_FUNCTION = true;
    static const bool HAS_RESET_FUNCTION = true;
    
    FunctionSelector function;
    uint8_t target_descriptor;
    bool enable;
    bool manual;
    uint16_t frequency;
    uint8_t reserved;
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::REPLY_ADVANCED_DATA_FILTER;
        
        uint8_t target_descriptor;
        bool enable;
        bool manual;
        uint16_t frequency;
        uint8_t reserved;
        
    };
};
void insert(Serializer& serializer, const AdvLowpassFilter& self);
void extract(Serializer& serializer, AdvLowpassFilter& self);

void insert(Serializer& serializer, const AdvLowpassFilter::Response& self);
void extract(Serializer& serializer, AdvLowpassFilter::Response& self);

CmdResult writeAdvLowpassFilter(C::mip_interface& device, uint8_t target_descriptor, bool enable, bool manual, uint16_t frequency, uint8_t reserved);
CmdResult readAdvLowpassFilter(C::mip_interface& device, uint8_t target_descriptor, bool& enable, bool& manual, uint16_t& frequency, uint8_t& reserved);
CmdResult saveAdvLowpassFilter(C::mip_interface& device, uint8_t target_descriptor);
CmdResult loadAdvLowpassFilter(C::mip_interface& device, uint8_t target_descriptor);
CmdResult defaultAdvLowpassFilter(C::mip_interface& device, uint8_t target_descriptor);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_pps_source  None
/// Controls the Pulse Per Second (PPS) source.
///
///@{

struct PpsSource
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_PPS_SOURCE;
    
    static const bool HAS_WRITE_FUNCTION = true;
    static const bool HAS_READ_FUNCTION = true;
    static const bool HAS_SAVE_FUNCTION = true;
    static const bool HAS_LOAD_FUNCTION = true;
    static const bool HAS_RESET_FUNCTION = true;
    
    enum class Source : uint8_t
    {
        DISABLED   = 0,  ///<  PPS output is disabled. Not valid for PPS source command.
        RECEIVER_1 = 1,  ///<  PPS is provided by GNSS receiver 1.
        RECEIVER_2 = 2,  ///<  PPS is provided by GNSS receiver 2.
        GPIO       = 3,  ///<  PPS is provided to an external GPIO pin. Use the GPIO Setup command to choose and configure the pin.
        GENERATED  = 4,  ///<  PPS is generated from the system oscillator.
    };
    
    FunctionSelector function;
    Source source;
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::REPLY_PPS_SOURCE;
        
        Source source;
        
    };
};
void insert(Serializer& serializer, const PpsSource& self);
void extract(Serializer& serializer, PpsSource& self);

void insert(Serializer& serializer, const PpsSource::Response& self);
void extract(Serializer& serializer, PpsSource::Response& self);

CmdResult writePpsSource(C::mip_interface& device, PpsSource::Source source);
CmdResult readPpsSource(C::mip_interface& device, PpsSource::Source& source);
CmdResult savePpsSource(C::mip_interface& device);
CmdResult loadPpsSource(C::mip_interface& device);
CmdResult defaultPpsSource(C::mip_interface& device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_gpio_config  GPIO Configuration
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
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_GPIO_CONFIG;
    
    static const bool HAS_WRITE_FUNCTION = true;
    static const bool HAS_READ_FUNCTION = true;
    static const bool HAS_SAVE_FUNCTION = true;
    static const bool HAS_LOAD_FUNCTION = true;
    static const bool HAS_RESET_FUNCTION = true;
    
    enum class Feature : uint8_t
    {
        UNUSED    = 0,  ///<  The pin is not used. It may be technically possible to read the pin state in this mode, but this is not guaranteed to be true of all devices or pins.
        GPIO      = 1,  ///<  General purpose input or output. Use this for direct control of pin output state or to stream the state of the pin.
        PPS       = 2,  ///<  Pulse per second input or output.
        ENCODER   = 3,  ///<  Motor encoder/odometer input.
        TIMESTAMP = 4,  ///<  Precision Timestamping. Use with Event Trigger Configuration (0x0C,0x2E).
        POWER     = 5,  ///<  Controls the device power state (e.g. enter low power mode).
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
        POWER_SHUTDOWN    = 1,  ///<  A logic 1 applied to the pin will place the device in low-power mode. A full restart is executed after the signal is removed.
    };
    
    struct PinMode : Bitfield<PinMode>
    {
        enum _enumType : uint8_t
        {
            NONE       = 0x00,
            OPEN_DRAIN = 0x01,
            PULLDOWN   = 0x02,
            PULLUP     = 0x04,
        };
        uint8_t value = NONE;
        
        operator uint8_t() const { return value; }
        PinMode& operator=(uint8_t val) { value = val; return *this; }
        PinMode& operator|=(uint8_t val) { return *this = value | val; }
        PinMode& operator&=(uint8_t val) { return *this = value & val; }
    };
    
    FunctionSelector function;
    uint8_t pin;
    Feature feature;
    Behavior behavior;
    PinMode pin_mode;
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::REPLY_GPIO_CONFIG;
        
        uint8_t pin;
        Feature feature;
        Behavior behavior;
        PinMode pin_mode;
        
    };
};
void insert(Serializer& serializer, const GpioConfig& self);
void extract(Serializer& serializer, GpioConfig& self);

void insert(Serializer& serializer, const GpioConfig::Response& self);
void extract(Serializer& serializer, GpioConfig::Response& self);

CmdResult writeGpioConfig(C::mip_interface& device, uint8_t pin, GpioConfig::Feature feature, GpioConfig::Behavior behavior, GpioConfig::PinMode pin_mode);
CmdResult readGpioConfig(C::mip_interface& device, uint8_t pin, GpioConfig::Feature& feature, GpioConfig::Behavior& behavior, GpioConfig::PinMode& pin_mode);
CmdResult saveGpioConfig(C::mip_interface& device, uint8_t pin);
CmdResult loadGpioConfig(C::mip_interface& device, uint8_t pin);
CmdResult defaultGpioConfig(C::mip_interface& device, uint8_t pin);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_gpio_state  GPIO State
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
/// This command does not support saving, loading, or reseting the state. Instead, use the
/// GPIO Configuration command, which allows the initial state to be configured.
///
///@{

struct GpioState
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_GPIO_STATE;
    
    static const bool HAS_WRITE_FUNCTION = true;
    static const bool HAS_READ_FUNCTION = true;
    static const bool HAS_SAVE_FUNCTION = false;
    static const bool HAS_LOAD_FUNCTION = false;
    static const bool HAS_RESET_FUNCTION = false;
    
    FunctionSelector function;
    uint8_t pin;
    bool state;
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::REPLY_GPIO_STATE;
        
        uint8_t pin;
        bool state;
        
    };
};
void insert(Serializer& serializer, const GpioState& self);
void extract(Serializer& serializer, GpioState& self);

void insert(Serializer& serializer, const GpioState::Response& self);
void extract(Serializer& serializer, GpioState::Response& self);

CmdResult writeGpioState(C::mip_interface& device, uint8_t pin, bool state);
CmdResult readGpioState(C::mip_interface& device, uint8_t pin, bool& state);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_odometer  Odometer Settings
/// Configures the hardware odometer interface.
/// 
///
///@{

struct Odometer
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_ODOMETER_CONFIG;
    
    static const bool HAS_WRITE_FUNCTION = true;
    static const bool HAS_READ_FUNCTION = true;
    static const bool HAS_SAVE_FUNCTION = true;
    static const bool HAS_LOAD_FUNCTION = true;
    static const bool HAS_RESET_FUNCTION = true;
    
    enum class Mode : uint8_t
    {
        DISABLED   = 0,  ///<  Encoder is disabled.
        QUADRATURE = 2,  ///<  Quadrature encoder mode.
    };
    
    FunctionSelector function;
    Mode mode;
    float scaling;
    float uncertainty;
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::REPLY_ODOMETER_CONFIG;
        
        Mode mode;
        float scaling;
        float uncertainty;
        
    };
};
void insert(Serializer& serializer, const Odometer& self);
void extract(Serializer& serializer, Odometer& self);

void insert(Serializer& serializer, const Odometer::Response& self);
void extract(Serializer& serializer, Odometer::Response& self);

CmdResult writeOdometer(C::mip_interface& device, Odometer::Mode mode, float scaling, float uncertainty);
CmdResult readOdometer(C::mip_interface& device, Odometer::Mode& mode, float& scaling, float& uncertainty);
CmdResult saveOdometer(C::mip_interface& device);
CmdResult loadOdometer(C::mip_interface& device);
CmdResult defaultOdometer(C::mip_interface& device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_get_event_support  Get Supported Events
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
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_EVENT_SUPPORT;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    enum class Query : uint8_t
    {
        TRIGGER_TYPES = 1,  ///<  Query the supported trigger types and max count for each.
        ACTION_TYPES  = 2,  ///<  Query the supported action types and max count for each.
    };
    
    struct Info
    {
        uint8_t type;
        uint8_t count;
        
    };
    Query query;
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::REPLY_EVENT_SUPPORT;
        
        Query query;
        uint8_t max_instances;
        uint8_t num_entries;
        Info entries[126];
        
    };
};
void insert(Serializer& serializer, const GetEventSupport& self);
void extract(Serializer& serializer, GetEventSupport& self);

void insert(Serializer& serializer, const GetEventSupport::Info& self);
void extract(Serializer& serializer, GetEventSupport::Info& self);

void insert(Serializer& serializer, const GetEventSupport::Response& self);
void extract(Serializer& serializer, GetEventSupport::Response& self);

CmdResult getEventSupport(C::mip_interface& device, GetEventSupport::Query query, uint8_t& max_instances, uint8_t& num_entries, GetEventSupport::Info* entries);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_event_control  Event Control
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
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_EVENT_CONTROL;
    
    static const bool HAS_WRITE_FUNCTION = true;
    static const bool HAS_READ_FUNCTION = true;
    static const bool HAS_SAVE_FUNCTION = true;
    static const bool HAS_LOAD_FUNCTION = true;
    static const bool HAS_RESET_FUNCTION = true;
    
    enum class Mode : uint8_t
    {
        DISABLED   = 0,  ///<  Trigger is disabled.
        ENABLED    = 1,  ///<  Trigger is enabled and will work normally.
        TEST       = 2,  ///<  Forces the trigger to the active state for testing purposes.
        TEST_PULSE = 3,  ///<  Trigger is forced to the active state for one event cycle only. After the test cycle, the mode reverts to the previous state (either enabled or disabled).
    };
    
    FunctionSelector function;
    uint8_t instance;
    Mode mode;
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::REPLY_EVENT_CONTROL;
        
        uint8_t instance;
        Mode mode;
        
    };
};
void insert(Serializer& serializer, const EventControl& self);
void extract(Serializer& serializer, EventControl& self);

void insert(Serializer& serializer, const EventControl::Response& self);
void extract(Serializer& serializer, EventControl::Response& self);

CmdResult writeEventControl(C::mip_interface& device, uint8_t instance, EventControl::Mode mode);
CmdResult readEventControl(C::mip_interface& device, uint8_t instance, EventControl::Mode& mode);
CmdResult saveEventControl(C::mip_interface& device, uint8_t instance);
CmdResult loadEventControl(C::mip_interface& device, uint8_t instance);
CmdResult defaultEventControl(C::mip_interface& device, uint8_t instance);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_get_event_trigger_status  Get Trigger Status
///
///@{

struct GetEventTriggerStatus
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_EVENT_TRIGGER_STATUS;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    struct Status : Bitfield<Status>
    {
        enum _enumType : uint8_t
        {
            NONE    = 0x00,
            ACTIVE  = 0x01,
            ENABLED = 0x02,
            TEST    = 0x04,
        };
        uint8_t value = NONE;
        
        operator uint8_t() const { return value; }
        Status& operator=(uint8_t val) { value = val; return *this; }
        Status& operator|=(uint8_t val) { return *this = value | val; }
        Status& operator&=(uint8_t val) { return *this = value & val; }
    };
    
    struct Entry
    {
        uint8_t type;
        Status status;
        
    };
    uint8_t requested_count;
    uint8_t requested_instances[20];
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::REPLY_EVENT_TRIGGER_STATUS;
        
        uint8_t count;
        Entry triggers[20];
        
    };
};
void insert(Serializer& serializer, const GetEventTriggerStatus& self);
void extract(Serializer& serializer, GetEventTriggerStatus& self);

void insert(Serializer& serializer, const GetEventTriggerStatus::Entry& self);
void extract(Serializer& serializer, GetEventTriggerStatus::Entry& self);

void insert(Serializer& serializer, const GetEventTriggerStatus::Response& self);
void extract(Serializer& serializer, GetEventTriggerStatus::Response& self);

CmdResult getEventTriggerStatus(C::mip_interface& device, uint8_t requested_count, const uint8_t* requested_instances, uint8_t& count, GetEventTriggerStatus::Entry* triggers);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_get_event_action_status  Get Action Status
///
///@{

struct GetEventActionStatus
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_EVENT_ACTION_STATUS;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    struct Entry
    {
        uint8_t action_type;
        uint8_t trigger_id;
        
    };
    uint8_t requested_count;
    uint8_t requested_instances[20];
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::REPLY_EVENT_ACTION_STATUS;
        
        uint8_t count;
        Entry actions[20];
        
    };
};
void insert(Serializer& serializer, const GetEventActionStatus& self);
void extract(Serializer& serializer, GetEventActionStatus& self);

void insert(Serializer& serializer, const GetEventActionStatus::Entry& self);
void extract(Serializer& serializer, GetEventActionStatus::Entry& self);

void insert(Serializer& serializer, const GetEventActionStatus::Response& self);
void extract(Serializer& serializer, GetEventActionStatus::Response& self);

CmdResult getEventActionStatus(C::mip_interface& device, uint8_t requested_count, const uint8_t* requested_instances, uint8_t& count, GetEventActionStatus::Entry* actions);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_event_trigger  Event Trigger Configuration
/// Configures various types of event triggers.
///
///@{

struct EventTrigger
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_EVENT_TRIGGER_CONFIG;
    
    static const bool HAS_WRITE_FUNCTION = true;
    static const bool HAS_READ_FUNCTION = true;
    static const bool HAS_SAVE_FUNCTION = true;
    static const bool HAS_LOAD_FUNCTION = true;
    static const bool HAS_RESET_FUNCTION = true;
    
    struct GpioParams
    {
        enum class Mode : uint8_t
        {
            DISABLED   = 0,  ///<  The pin will have no effect and the trigger will never activate.
            WHILE_HIGH = 1,  ///<  The trigger will be active while the pin is high.
            WHILE_LOW  = 2,  ///<  The trigger will be active while the pin is low.
            EDGE       = 4,  ///<  Use if the pin is configured for timestamping via the 3DM Gpio Configuration command (0x0C41).
        };
        
        uint8_t pin;
        Mode mode;
        
    };
    struct ThresholdParams
    {
        enum class Type : uint8_t
        {
            WINDOW   = 1,  ///<  Window comparison. Trigger is active if low_thres &lt;= value &lt;= high_thres. If the thresholds are reversed, the trigger is active when value &lt; high_thres or value &gt; low_thres.
            INTERVAL = 2,  ///<  Trigger at evenly-spaced intervals. Normally used with time fields to trigger periodically. Trigger is active when (value % interval) &lt;= int_thres. If the thresholds are reversed (high_thres &lt; low_thres) then the trigger is active when (value % low_thres) &gt; high_thres.
        };
        
        uint8_t desc_set;
        uint8_t field_desc;
        uint8_t param_id;
        Type type;
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
        
    };
    struct CombinationParams
    {
        static const uint16_t LOGIC_NEVER = 0x0000;
        static const uint16_t LOGIC_ALWAYS = 0xFFFF;
        static const uint16_t LOGIC_NONE = 0x0001;
        static const uint16_t LOGIC_OR = 0xFFFE;
        static const uint16_t LOGIC_NAND = 0x7FFF;
        static const uint16_t LOGIC_XOR_ONE = 0x0116;
        static const uint16_t LOGIC_ONLY_A = 0x0002;
        static const uint16_t LOGIC_ONLY_B = 0x0004;
        static const uint16_t LOGIC_ONLY_C = 0x0010;
        static const uint16_t LOGIC_ONLY_D = 0x0100;
        static const uint16_t LOGIC_AND_AB = 0x8888;
        static const uint16_t LOGIC_AB_OR_C = 0xF8F8;
        static const uint16_t LOGIC_AND = 0x8000;
        uint16_t logic_table;
        uint8_t input_triggers[4];
        
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
    };
    
    FunctionSelector function;
    uint8_t instance;
    Type type;
    Parameters parameters;
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::REPLY_EVENT_TRIGGER_CONFIG;
        
        uint8_t instance;
        Type type;
        Parameters parameters;
        
    };
};
void insert(Serializer& serializer, const EventTrigger& self);
void extract(Serializer& serializer, EventTrigger& self);

void insert(Serializer& serializer, const EventTrigger::GpioParams& self);
void extract(Serializer& serializer, EventTrigger::GpioParams& self);

void insert(Serializer& serializer, const EventTrigger::ThresholdParams& self);
void extract(Serializer& serializer, EventTrigger::ThresholdParams& self);

void insert(Serializer& serializer, const EventTrigger::CombinationParams& self);
void extract(Serializer& serializer, EventTrigger::CombinationParams& self);

void insert(Serializer& serializer, const EventTrigger::Response& self);
void extract(Serializer& serializer, EventTrigger::Response& self);

CmdResult writeEventTrigger(C::mip_interface& device, uint8_t instance, EventTrigger::Type type, const EventTrigger::Parameters& parameters);
CmdResult readEventTrigger(C::mip_interface& device, uint8_t instance, EventTrigger::Type& type, EventTrigger::Parameters& parameters);
CmdResult saveEventTrigger(C::mip_interface& device, uint8_t instance);
CmdResult loadEventTrigger(C::mip_interface& device, uint8_t instance);
CmdResult defaultEventTrigger(C::mip_interface& device, uint8_t instance);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_event_action  Event Action Configuration
/// Configures various types of event actions.
///
///@{

struct EventAction
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_EVENT_ACTION_CONFIG;
    
    static const bool HAS_WRITE_FUNCTION = true;
    static const bool HAS_READ_FUNCTION = true;
    static const bool HAS_SAVE_FUNCTION = true;
    static const bool HAS_LOAD_FUNCTION = true;
    static const bool HAS_RESET_FUNCTION = true;
    
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
        
        uint8_t pin;
        Mode mode;
        
    };
    struct MessageParams
    {
        uint8_t desc_set;
        uint16_t decimation;
        uint8_t num_fields;
        uint8_t descriptors[20];
        
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
    };
    
    FunctionSelector function;
    uint8_t instance;
    uint8_t trigger;
    Type type;
    Parameters parameters;
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::REPLY_EVENT_ACTION_CONFIG;
        
        uint8_t instance;
        uint8_t trigger;
        Type type;
        Parameters parameters;
        
    };
};
void insert(Serializer& serializer, const EventAction& self);
void extract(Serializer& serializer, EventAction& self);

void insert(Serializer& serializer, const EventAction::GpioParams& self);
void extract(Serializer& serializer, EventAction::GpioParams& self);

void insert(Serializer& serializer, const EventAction::MessageParams& self);
void extract(Serializer& serializer, EventAction::MessageParams& self);

void insert(Serializer& serializer, const EventAction::Response& self);
void extract(Serializer& serializer, EventAction::Response& self);

CmdResult writeEventAction(C::mip_interface& device, uint8_t instance, uint8_t trigger, EventAction::Type type, const EventAction::Parameters& parameters);
CmdResult readEventAction(C::mip_interface& device, uint8_t instance, uint8_t& trigger, EventAction::Type& type, EventAction::Parameters& parameters);
CmdResult saveEventAction(C::mip_interface& device, uint8_t instance);
CmdResult loadEventAction(C::mip_interface& device, uint8_t instance);
CmdResult defaultEventAction(C::mip_interface& device, uint8_t instance);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_accel_bias  Configure Accel Bias
/// Configures the user specified accelerometer bias
/// 
/// The user specified bias is subtracted from the calibrated accelerometer output.  Value is input in the sensor frame.
///
///@{

struct AccelBias
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_ACCEL_BIAS;
    
    static const bool HAS_WRITE_FUNCTION = true;
    static const bool HAS_READ_FUNCTION = true;
    static const bool HAS_SAVE_FUNCTION = true;
    static const bool HAS_LOAD_FUNCTION = true;
    static const bool HAS_RESET_FUNCTION = true;
    
    FunctionSelector function;
    float bias[3];
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::REPLY_ACCEL_BIAS_VECTOR;
        
        float bias[3];
        
    };
};
void insert(Serializer& serializer, const AccelBias& self);
void extract(Serializer& serializer, AccelBias& self);

void insert(Serializer& serializer, const AccelBias::Response& self);
void extract(Serializer& serializer, AccelBias::Response& self);

CmdResult writeAccelBias(C::mip_interface& device, const float* bias);
CmdResult readAccelBias(C::mip_interface& device, float* bias);
CmdResult saveAccelBias(C::mip_interface& device);
CmdResult loadAccelBias(C::mip_interface& device);
CmdResult defaultAccelBias(C::mip_interface& device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_gyro_bias  Configure Gyro Bias
/// Configures the user specified gyroscope bias
/// 
/// The user specified bias is subtracted from the calibrated angular rate output.  Value is input in the sensor frame.
///
///@{

struct GyroBias
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_GYRO_BIAS;
    
    static const bool HAS_WRITE_FUNCTION = true;
    static const bool HAS_READ_FUNCTION = true;
    static const bool HAS_SAVE_FUNCTION = true;
    static const bool HAS_LOAD_FUNCTION = true;
    static const bool HAS_RESET_FUNCTION = true;
    
    FunctionSelector function;
    float bias[3];
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::REPLY_GYRO_BIAS_VECTOR;
        
        float bias[3];
        
    };
};
void insert(Serializer& serializer, const GyroBias& self);
void extract(Serializer& serializer, GyroBias& self);

void insert(Serializer& serializer, const GyroBias::Response& self);
void extract(Serializer& serializer, GyroBias::Response& self);

CmdResult writeGyroBias(C::mip_interface& device, const float* bias);
CmdResult readGyroBias(C::mip_interface& device, float* bias);
CmdResult saveGyroBias(C::mip_interface& device);
CmdResult loadGyroBias(C::mip_interface& device);
CmdResult defaultGyroBias(C::mip_interface& device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_capture_gyro_bias  Capture Gyro Bias
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
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_CAPTURE_GYRO_BIAS;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    uint16_t averaging_time_ms;
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::REPLY_GYRO_BIAS_VECTOR;
        
        float bias[3];
        
    };
};
void insert(Serializer& serializer, const CaptureGyroBias& self);
void extract(Serializer& serializer, CaptureGyroBias& self);

void insert(Serializer& serializer, const CaptureGyroBias::Response& self);
void extract(Serializer& serializer, CaptureGyroBias::Response& self);

CmdResult captureGyroBias(C::mip_interface& device, uint16_t averaging_time_ms, float* bias);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_mag_hard_iron_offset  Magnetometer Hard Iron Offset
/// Configure the user specified magnetometer hard iron offset vector
/// 
/// The values for this offset are determined empirically by external software algorithms
/// based on calibration data taken after the device is installed in its application. These values
/// can be obtained and set by using the LORD "MIP Iron Calibration" application.
/// Alternatively, on some systems, the auto-mag calibration feature may be used to capture these values in-run.
/// The offset is applied to the scaled magnetometer vector prior to output.
///
///@{

struct MagHardIronOffset
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_HARD_IRON_OFFSET;
    
    static const bool HAS_WRITE_FUNCTION = true;
    static const bool HAS_READ_FUNCTION = true;
    static const bool HAS_SAVE_FUNCTION = true;
    static const bool HAS_LOAD_FUNCTION = true;
    static const bool HAS_RESET_FUNCTION = true;
    
    FunctionSelector function;
    float offset[3];
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::REPLY_HARD_IRON_OFFSET_VECTOR;
        
        float offset[3];
        
    };
};
void insert(Serializer& serializer, const MagHardIronOffset& self);
void extract(Serializer& serializer, MagHardIronOffset& self);

void insert(Serializer& serializer, const MagHardIronOffset::Response& self);
void extract(Serializer& serializer, MagHardIronOffset::Response& self);

CmdResult writeMagHardIronOffset(C::mip_interface& device, const float* offset);
CmdResult readMagHardIronOffset(C::mip_interface& device, float* offset);
CmdResult saveMagHardIronOffset(C::mip_interface& device);
CmdResult loadMagHardIronOffset(C::mip_interface& device);
CmdResult defaultMagHardIronOffset(C::mip_interface& device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_mag_soft_iron_matrix  Magnetometer Soft Iron Matrix
/// Configure the user specified magnetometer soft iron offset matrix
/// 
/// The values for this matrix are determined empirically by external software algorithms
/// based on calibration data taken after the device is installed in its application. These values
/// can be obtained and set by using the LORD "MIP Iron Calibration" application.
/// Alternatively, on some systems, the auto-mag calibration feature may be used to capture these values in-run.
/// The matrix is applied to the scaled magnetometer vector prior to output.
/// 
/// The matrix is in row major order:
/// EQSTART M = \begin{bmatrix} 0 &amp; 1 &amp; 2 \\ 3 &amp; 4 &amp; 5 \\ 6 &amp; 7 &amp; 8 \end{bmatrix} EQEND
/// 
///
///@{

struct MagSoftIronMatrix
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_SOFT_IRON_MATRIX;
    
    static const bool HAS_WRITE_FUNCTION = true;
    static const bool HAS_READ_FUNCTION = true;
    static const bool HAS_SAVE_FUNCTION = true;
    static const bool HAS_LOAD_FUNCTION = true;
    static const bool HAS_RESET_FUNCTION = true;
    
    FunctionSelector function;
    float offset[9];
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::REPLY_SOFT_IRON_COMP_MATRIX;
        
        float offset[9];
        
    };
};
void insert(Serializer& serializer, const MagSoftIronMatrix& self);
void extract(Serializer& serializer, MagSoftIronMatrix& self);

void insert(Serializer& serializer, const MagSoftIronMatrix::Response& self);
void extract(Serializer& serializer, MagSoftIronMatrix::Response& self);

CmdResult writeMagSoftIronMatrix(C::mip_interface& device, const float* offset);
CmdResult readMagSoftIronMatrix(C::mip_interface& device, float* offset);
CmdResult saveMagSoftIronMatrix(C::mip_interface& device);
CmdResult loadMagSoftIronMatrix(C::mip_interface& device);
CmdResult defaultMagSoftIronMatrix(C::mip_interface& device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_sensor_2_vehicle_transform_euler  Sensor to Vehicle Frame Transformation Euler
/// Sets the sensor-to-vehicle frame transformation using Yaw, Pitch, Roll Euler angles.
/// These are the Yaw, Pitch, and Roll mounting angles of the sensor with respect to vehicle frame of reference,
/// and describe the transformation of vectors from the sensor body frame to the vehicle frame.<br/>
/// Note: This is the transformation, the inverse of the rotation defined in our legacy products.<br/>
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
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_SENSOR2VEHICLE_TRANSFORM_EUL;
    
    static const bool HAS_WRITE_FUNCTION = true;
    static const bool HAS_READ_FUNCTION = true;
    static const bool HAS_SAVE_FUNCTION = true;
    static const bool HAS_LOAD_FUNCTION = true;
    static const bool HAS_RESET_FUNCTION = true;
    
    FunctionSelector function;
    float roll;
    float pitch;
    float yaw;
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::REPLY_SENSOR2VEHICLE_TRANSFORM_EUL;
        
        float roll;
        float pitch;
        float yaw;
        
    };
};
void insert(Serializer& serializer, const Sensor2VehicleTransformEuler& self);
void extract(Serializer& serializer, Sensor2VehicleTransformEuler& self);

void insert(Serializer& serializer, const Sensor2VehicleTransformEuler::Response& self);
void extract(Serializer& serializer, Sensor2VehicleTransformEuler::Response& self);

CmdResult writeSensor2VehicleTransformEuler(C::mip_interface& device, float roll, float pitch, float yaw);
CmdResult readSensor2VehicleTransformEuler(C::mip_interface& device, float& roll, float& pitch, float& yaw);
CmdResult saveSensor2VehicleTransformEuler(C::mip_interface& device);
CmdResult loadSensor2VehicleTransformEuler(C::mip_interface& device);
CmdResult defaultSensor2VehicleTransformEuler(C::mip_interface& device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_sensor_2_vehicle_transform_quaternion  Sensor to Vehicle Frame Transformation Quaternion
/// Set the sensor to vehicle frame transformation using unit length quaternion.
/// 
/// Note: This is the transformation, the inverse of the rotation.
/// 
/// This quaternion describes the transformation of vectors from the sensor body frame to the vehicle frame of reference, and satisfies the following relationship:<br/>
/// 
/// EQSTART p^{veh} = q^{-1} p^{sen} q EQEND<br/>
/// 
/// Where:<br/>
/// EQSTART q = (q_w, q_x, q_y, q_z) EQEND is the quaternion desrcribing the transformation. <br/>
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
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_SENSOR2VEHICLE_TRANSFORM_QUAT;
    
    static const bool HAS_WRITE_FUNCTION = true;
    static const bool HAS_READ_FUNCTION = true;
    static const bool HAS_SAVE_FUNCTION = true;
    static const bool HAS_LOAD_FUNCTION = true;
    static const bool HAS_RESET_FUNCTION = true;
    
    FunctionSelector function;
    float q[4];
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::REPLY_SENSOR2VEHICLE_TRANSFORM_QUAT;
        
        float q[4];
        
    };
};
void insert(Serializer& serializer, const Sensor2VehicleTransformQuaternion& self);
void extract(Serializer& serializer, Sensor2VehicleTransformQuaternion& self);

void insert(Serializer& serializer, const Sensor2VehicleTransformQuaternion::Response& self);
void extract(Serializer& serializer, Sensor2VehicleTransformQuaternion::Response& self);

CmdResult writeSensor2VehicleTransformQuaternion(C::mip_interface& device, const float* q);
CmdResult readSensor2VehicleTransformQuaternion(C::mip_interface& device, float* q);
CmdResult saveSensor2VehicleTransformQuaternion(C::mip_interface& device);
CmdResult loadSensor2VehicleTransformQuaternion(C::mip_interface& device);
CmdResult defaultSensor2VehicleTransformQuaternion(C::mip_interface& device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_sensor_2_vehicle_transform_dcm  Sensor to Vehicle Frame Transformation Direction Cosine Matrix
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
/// The matrix elements are stored is row-major order: EQSTART M_{sen}^{veh} = \begin{bmatrix} M_{11}, M_{12}, M_{13}, M_{21}, M_{22}, M_{23}, M_{31}, M_{32}, M_{33} \end{bmatrix} EQEND
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
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_SENSOR2VEHICLE_TRANSFORM_DCM;
    
    static const bool HAS_WRITE_FUNCTION = true;
    static const bool HAS_READ_FUNCTION = true;
    static const bool HAS_SAVE_FUNCTION = true;
    static const bool HAS_LOAD_FUNCTION = true;
    static const bool HAS_RESET_FUNCTION = true;
    
    FunctionSelector function;
    float dcm[9];
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::REPLY_SENSOR2VEHICLE_TRANSFORM_DCM;
        
        float dcm[9];
        
    };
};
void insert(Serializer& serializer, const Sensor2VehicleTransformDcm& self);
void extract(Serializer& serializer, Sensor2VehicleTransformDcm& self);

void insert(Serializer& serializer, const Sensor2VehicleTransformDcm::Response& self);
void extract(Serializer& serializer, Sensor2VehicleTransformDcm::Response& self);

CmdResult writeSensor2VehicleTransformDcm(C::mip_interface& device, const float* dcm);
CmdResult readSensor2VehicleTransformDcm(C::mip_interface& device, float* dcm);
CmdResult saveSensor2VehicleTransformDcm(C::mip_interface& device);
CmdResult loadSensor2VehicleTransformDcm(C::mip_interface& device);
CmdResult defaultSensor2VehicleTransformDcm(C::mip_interface& device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_complementary_filter  Complementary filter settings
/// Configure the settings for the complementary filter which produces the following (0x80) descriptor set values: attitude matrix (0x80,09), quaternion (0x80,0A), and  Euler angle (0x80,0C) outputs.
/// 
/// The filter can be configured to correct for pitch and roll using the accelerometer (with the assumption that linear acceleration is minimal),
/// and to correct for heading using the magnetomer (with the assumption that the local magnetic field is dominated by the Earth's own magnetic field).
/// Pitch/roll and heading corrections each have their own configurable time constants, with a valid range of 1-1000 seconds. The default time constant is 10 seconds.
///
///@{

struct ComplementaryFilter
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_LEGACY_COMP_FILTER;
    
    static const bool HAS_WRITE_FUNCTION = true;
    static const bool HAS_READ_FUNCTION = true;
    static const bool HAS_SAVE_FUNCTION = true;
    static const bool HAS_LOAD_FUNCTION = true;
    static const bool HAS_RESET_FUNCTION = true;
    
    FunctionSelector function;
    bool pitch_roll_enable;
    bool heading_enable;
    float pitch_roll_time_constant;
    float heading_time_constant;
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::REPLY_LEGACY_COMP_FILTER;
        
        bool pitch_roll_enable;
        bool heading_enable;
        float pitch_roll_time_constant;
        float heading_time_constant;
        
    };
};
void insert(Serializer& serializer, const ComplementaryFilter& self);
void extract(Serializer& serializer, ComplementaryFilter& self);

void insert(Serializer& serializer, const ComplementaryFilter::Response& self);
void extract(Serializer& serializer, ComplementaryFilter::Response& self);

CmdResult writeComplementaryFilter(C::mip_interface& device, bool pitch_roll_enable, bool heading_enable, float pitch_roll_time_constant, float heading_time_constant);
CmdResult readComplementaryFilter(C::mip_interface& device, bool& pitch_roll_enable, bool& heading_enable, float& pitch_roll_time_constant, float& heading_time_constant);
CmdResult saveComplementaryFilter(C::mip_interface& device);
CmdResult loadComplementaryFilter(C::mip_interface& device);
CmdResult defaultComplementaryFilter(C::mip_interface& device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_sensor_range  Sensor Range
/// Changes the IMU sensor gain.
/// 
/// This allows you to optimize the range to get the best accuracy and performance
/// while minimizing overrange events.
/// 
/// Use the 3DM Get Calibrated Sensor Ranges (0x0C,0x53) command to determine
/// the appropriate setting value for your application. Using values other than
/// those specified may result in a NACK or inaccurate measurement data.
///
///@{

struct SensorRange
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_SENSOR_RANGE;
    
    static const bool HAS_WRITE_FUNCTION = true;
    static const bool HAS_READ_FUNCTION = true;
    static const bool HAS_SAVE_FUNCTION = true;
    static const bool HAS_LOAD_FUNCTION = true;
    static const bool HAS_RESET_FUNCTION = true;
    
    FunctionSelector function;
    SensorRangeType sensor;
    uint8_t setting;
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::REPLY_SENSOR_RANGE;
        
        SensorRangeType sensor;
        uint8_t setting;
        
    };
};
void insert(Serializer& serializer, const SensorRange& self);
void extract(Serializer& serializer, SensorRange& self);

void insert(Serializer& serializer, const SensorRange::Response& self);
void extract(Serializer& serializer, SensorRange::Response& self);

CmdResult writeSensorRange(C::mip_interface& device, SensorRangeType sensor, uint8_t setting);
CmdResult readSensorRange(C::mip_interface& device, SensorRangeType sensor, uint8_t& setting);
CmdResult saveSensorRange(C::mip_interface& device, SensorRangeType sensor);
CmdResult loadSensorRange(C::mip_interface& device, SensorRangeType sensor);
CmdResult defaultSensorRange(C::mip_interface& device, SensorRangeType sensor);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_calibrated_sensor_ranges  Get Calibrated Sensor Ranges
/// Returns the supported sensor ranges which may be used with the 3DM Sensor Range (0x0C,0x52) command.
/// 
/// The response includes an array of (u8, float) pairs which map each allowed setting
/// to the corresponding maximum range in physical units. See SensorRangeType for units.
///
///@{

struct CalibratedSensorRanges
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::CMD_CALIBRATED_RANGES;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    struct Entry
    {
        uint8_t setting;
        float range;
        
    };
    SensorRangeType sensor;
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_3dm::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_3dm::REPLY_CALIBRATED_RANGES;
        
        SensorRangeType sensor;
        uint8_t num_ranges;
        Entry ranges[50];
        
    };
};
void insert(Serializer& serializer, const CalibratedSensorRanges& self);
void extract(Serializer& serializer, CalibratedSensorRanges& self);

void insert(Serializer& serializer, const CalibratedSensorRanges::Entry& self);
void extract(Serializer& serializer, CalibratedSensorRanges::Entry& self);

void insert(Serializer& serializer, const CalibratedSensorRanges::Response& self);
void extract(Serializer& serializer, CalibratedSensorRanges::Response& self);

CmdResult calibratedSensorRanges(C::mip_interface& device, SensorRangeType sensor, uint8_t& num_ranges, CalibratedSensorRanges::Entry* ranges);
///@}
///

///@}
///@}
///
////////////////////////////////////////////////////////////////////////////////
} // namespace commands_3dm
} // namespace mip

