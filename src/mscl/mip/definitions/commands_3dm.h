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
///@defgroup 3DMCommands  3DM
///
///@{

////////////////////////////////////////////////////////////////////////////////
// Descriptors
////////////////////////////////////////////////////////////////////////////////

enum 
{
    MIP_3DM_CMD_DESC_SET                             = 0x0C,
    
    MIP_CMD_DESC_3DM_POLL_IMU_MESSAGE                = 0x01,
    MIP_CMD_DESC_3DM_POLL_GNSS_MESSAGE               = 0x02,
    MIP_CMD_DESC_3DM_POLL_FILTER_MESSAGE             = 0x03,
    MIP_CMD_DESC_3DM_POLL_NMEA_MESSAGE               = 0x04,
    MIP_CMD_DESC_3DM_GET_IMU_BASE_RATE               = 0x06,
    MIP_CMD_DESC_3DM_GET_GNSS_BASE_RATE              = 0x07,
    MIP_CMD_DESC_3DM_IMU_MESSAGE_FORMAT              = 0x08,
    MIP_CMD_DESC_3DM_GNSS_MESSAGE_FORMAT             = 0x09,
    MIP_CMD_DESC_3DM_FILTER_MESSAGE_FORMAT           = 0x0A,
    MIP_CMD_DESC_3DM_GET_FILTER_BASE_RATE            = 0x0B,
    MIP_CMD_DESC_3DM_NMEA_MESSAGE_FORMAT             = 0x0C,
    MIP_CMD_DESC_3DM_POLL_DATA                       = 0x0D,
    MIP_CMD_DESC_3DM_GET_BASE_RATE                   = 0x0E,
    MIP_CMD_DESC_3DM_MESSAGE_FORMAT                  = 0x0F,
    MIP_CMD_DESC_3DM_CONFIGURE_FACTORY_STREAMING     = 0x10,
    MIP_CMD_DESC_3DM_CONTROL_DATA_STREAM             = 0x11,
    MIP_CMD_DESC_3DM_RAW_RTCM_2_3_MESSAGE            = 0x20,
    MIP_CMD_DESC_3DM_GNSS_CONSTELLATION_SETTINGS     = 0x21,
    MIP_CMD_DESC_3DM_GNSS_SBAS_SETTINGS              = 0x22,
    MIP_CMD_DESC_3DM_GNSS_ASSISTED_FIX_SETTINGS      = 0x23,
    MIP_CMD_DESC_3DM_GNSS_TIME_ASSISTANCE            = 0x24,
    MIP_CMD_DESC_3DM_PPS_SOURCE                      = 0x28,
    MIP_CMD_DESC_3DM_EVENT_SUPPORT                   = 0x2A,
    MIP_CMD_DESC_3DM_EVENT_CONTROL                   = 0x2B,
    MIP_CMD_DESC_3DM_EVENT_TRIGGER_STATUS            = 0x2C,
    MIP_CMD_DESC_3DM_EVENT_ACTION_STATUS             = 0x2D,
    MIP_CMD_DESC_3DM_EVENT_TRIGGER_CONFIG            = 0x2E,
    MIP_CMD_DESC_3DM_EVENT_ACTION_CONFIG             = 0x2F,
    MIP_CMD_DESC_3DM_DEVICE_STARTUP_SETTINGS         = 0x30,
    MIP_CMD_DESC_3DM_SENSOR2VEHICLE_TRANSFORM_EUL    = 0x31,
    MIP_CMD_DESC_3DM_SENSOR2VEHICLE_TRANSFORM_QUAT   = 0x32,
    MIP_CMD_DESC_3DM_SENSOR2VEHICLE_TRANSFORM_DCM    = 0x33,
    MIP_CMD_DESC_3DM_SET_GNSS_DYNAMICS_MODE          = 0x34,
    MIP_CMD_DESC_3DM_SET_IMU_SIGNAL_COND             = 0x35,
    MIP_CMD_DESC_3DM_SET_IMU_TIMESTAMP               = 0x36,
    MIP_CMD_DESC_3DM_ACCEL_BIAS                      = 0x37,
    MIP_CMD_DESC_3DM_GYRO_BIAS                       = 0x38,
    MIP_CMD_DESC_3DM_CAPTURE_GYRO_BIAS               = 0x39,
    MIP_CMD_DESC_3DM_HARD_IRON_OFFSET                = 0x3A,
    MIP_CMD_DESC_3DM_SOFT_IRON_MATRIX                = 0x3B,
    MIP_CMD_DESC_3DM_REALIGN_UP                      = 0x3C,
    MIP_CMD_DESC_3DM_REALIGN_NORTH                   = 0x3D,
    MIP_CMD_DESC_3DM_CONING_AND_SCULLING_ENABLE      = 0x3E,
    MIP_CMD_DESC_3DM_UART_BAUDRATE                   = 0x40,
    MIP_CMD_DESC_3DM_GPIO_CONFIG                     = 0x41,
    MIP_CMD_DESC_3DM_GPIO_STATE                      = 0x42,
    MIP_CMD_DESC_3DM_ODOMETER_CONFIG                 = 0x43,
    MIP_CMD_DESC_3DM_ADVANCED_DATA_FILTER            = 0x50,
    MIP_CMD_DESC_3DM_LEGACY_COMP_FILTER              = 0x51,
    MIP_CMD_DESC_3DM_SENSOR_RANGE                    = 0x52,
    MIP_CMD_DESC_3DM_CALIBRATED_RANGES               = 0x53,
    MIP_CMD_DESC_3DM_DATASTREAM_FORMAT               = 0x60,
    MIP_CMD_DESC_3DM_DEVICE_POWER_STATE              = 0x61,
    MIP_CMD_DESC_3DM_SAVE_RESTORE_GPS_SETTINGS       = 0x62,
    MIP_CMD_DESC_3DM_DEVICE_SETTINGS                 = 0x63,
    MIP_CMD_DESC_3DM_RAW_CLIP_SETTINGS               = 0x70,
    
    MIP_REPLY_DESC_3DM_IMU_MESSAGE_FORMAT            = 0x80,
    MIP_REPLY_DESC_3DM_GNSS_MESSAGE_FORMAT           = 0x81,
    MIP_REPLY_DESC_3DM_FILTER_MESSAGE_FORMAT         = 0x82,
    MIP_REPLY_DESC_3DM_IMU_BASE_RATE                 = 0x83,
    MIP_REPLY_DESC_3DM_GNSS_BASE_RATE                = 0x84,
    MIP_REPLY_DESC_3DM_DATASTREAM_ENABLE             = 0x85,
    MIP_REPLY_DESC_3DM_IMU_SIGNAL_SETTINGS           = 0x86,
    MIP_REPLY_DESC_3DM_UART_BAUDRATE                 = 0x87,
    MIP_REPLY_DESC_3DM_DATASTREAM_FORMAT             = 0x88,
    MIP_REPLY_DESC_3DM_POWER_STATE                   = 0x89,
    MIP_REPLY_DESC_3DM_FILTER_BASE_RATE              = 0x8A,
    MIP_REPLY_DESC_3DM_ADVANCED_DATA_FILTER          = 0x8B,
    MIP_REPLY_DESC_3DM_POLL_DATA                     = 0x8D,
    MIP_REPLY_DESC_3DM_BASE_RATE                     = 0x8E,
    MIP_REPLY_DESC_3DM_MESSAGE_FORMAT                = 0x8F,
    MIP_REPLY_DESC_3DM_COMMUNICATIONS_MODE           = 0x91,
    MIP_REPLY_DESC_3DM_GNSS_DYNAMICS_MODE            = 0x92,
    MIP_REPLY_DESC_3DM_IMU_TIMESTAMP_VALUE           = 0x93,
    MIP_REPLY_DESC_3DM_IMU_BASIC_STATUS              = 0x94,
    MIP_REPLY_DESC_3DM_IMU_ADVANCED_STATUS           = 0x95,
    MIP_REPLY_DESC_3DM_RAW_CLIP_SETTINGS             = 0x96,
    MIP_REPLY_DESC_3DM_LEGACY_COMP_FILTER            = 0x97,
    MIP_REPLY_DESC_3DM_ACCEL_BIAS_VECTOR             = 0x9A,
    MIP_REPLY_DESC_3DM_GYRO_BIAS_VECTOR              = 0x9B,
    MIP_REPLY_DESC_3DM_HARD_IRON_OFFSET_VECTOR       = 0x9C,
    MIP_REPLY_DESC_3DM_SOFT_IRON_COMP_MATRIX         = 0x9D,
    MIP_REPLY_DESC_3DM_CONING_AND_SCULLING_ENABLE    = 0x9E,
    MIP_REPLY_DESC_3DM_GNSS_CONSTELLATION_SETTINGS   = 0xA0,
    MIP_REPLY_DESC_3DM_GNSS_SBAS_SETTINGS            = 0xA1,
    MIP_REPLY_DESC_3DM_GNSS_ASSISTED_FIX_SETTINGS    = 0xA2,
    MIP_REPLY_DESC_3DM_GNSS_TIME_ASSISTANCE          = 0xA3,
    MIP_REPLY_DESC_3DM_SENSOR2VEHICLE_TRANSFORM_EUL  = 0xB1,
    MIP_REPLY_DESC_3DM_SENSOR2VEHICLE_TRANSFORM_QUAT = 0xB2,
    MIP_REPLY_DESC_3DM_SENSOR2VEHICLE_TRANSFORM_DCM  = 0xB3,
    MIP_REPLY_DESC_3DM_EVENT_SUPPORT                 = 0xB4,
    MIP_REPLY_DESC_3DM_EVENT_CONTROL                 = 0xB5,
    MIP_REPLY_DESC_3DM_EVENT_TRIGGER_STATUS          = 0xB6,
    MIP_REPLY_DESC_3DM_EVENT_ACTION_STATUS           = 0xB7,
    MIP_REPLY_DESC_3DM_EVENT_TRIGGER_CONFIG          = 0xB8,
    MIP_REPLY_DESC_3DM_EVENT_ACTION_CONFIG           = 0xB9,
    MIP_REPLY_DESC_3DM_NMEA_MESSAGE_FORMAT           = 0x8C,
    MIP_REPLY_DESC_3DM_PPS_SOURCE                    = 0xA8,
    MIP_REPLY_DESC_3DM_GPIO_CONFIG                   = 0xC1,
    MIP_REPLY_DESC_3DM_GPIO_STATE                    = 0xC2,
    MIP_REPLY_DESC_3DM_ODOMETER_CONFIG               = 0xC3,
    MIP_REPLY_DESC_3DM_SENSOR_RANGE                  = 0xD2,
    MIP_REPLY_DESC_3DM_CALIBRATED_RANGES             = 0xD3,
};
#ifdef __cplusplus
namespace C {
extern "C" {
#endif // __cplusplus


////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////

enum mip_nmeamessage_format_message_id
{
    MIP_NMEAMESSAGE_FORMAT_MESSAGE_ID_GGA  = 1,  ///<  GPS System Fix Data
    MIP_NMEAMESSAGE_FORMAT_MESSAGE_ID_GLL  = 2,  ///<  Geographic Position Lat/Lon
    MIP_NMEAMESSAGE_FORMAT_MESSAGE_ID_GSV  = 3,  ///<  GNSS Satellites in View
    MIP_NMEAMESSAGE_FORMAT_MESSAGE_ID_RMC  = 4,  ///<  Recommended Minimum Specific GNSS Data
    MIP_NMEAMESSAGE_FORMAT_MESSAGE_ID_VTG  = 5,  ///<  Course over Ground
    MIP_NMEAMESSAGE_FORMAT_MESSAGE_ID_HDT  = 6,  ///<  Heading, True
    MIP_NMEAMESSAGE_FORMAT_MESSAGE_ID_ZDA  = 7,  ///<  Time & Date
    MIP_NMEAMESSAGE_FORMAT_MESSAGE_ID_PRKA = 100,  ///<  Parker proprietary Euler angles
    MIP_NMEAMESSAGE_FORMAT_MESSAGE_ID_PRKR = 101,  ///<  Parker proprietary Angular Rate/Acceleration
};
size_t insert_mip_nmeamessage_format_message_id(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_nmeamessage_format_message_id self);
size_t extract_mip_nmeamessage_format_message_id(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_nmeamessage_format_message_id* self);

enum mip_nmeamessage_format_talker_id
{
    MIP_NMEAMESSAGE_FORMAT_TALKER_ID_GNSS    = 1,  ///<  NMEA message will be produced with talker id "GN"
    MIP_NMEAMESSAGE_FORMAT_TALKER_ID_GPS     = 2,  ///<  NMEA message will be produced with talker id "GP"
    MIP_NMEAMESSAGE_FORMAT_TALKER_ID_GALILEO = 3,  ///<  NMEA message will be produced with talker id "GA"
    MIP_NMEAMESSAGE_FORMAT_TALKER_ID_GLONASS = 4,  ///<  NMEA message will be produced with talker id "GL"
};
size_t insert_mip_nmeamessage_format_talker_id(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_nmeamessage_format_talker_id self);
size_t extract_mip_nmeamessage_format_talker_id(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_nmeamessage_format_talker_id* self);

enum mip_nmeamessage_format_source_id
{
    MIP_NMEAMESSAGE_FORMAT_SOURCE_ID_FILTER = 1,  ///<  Data from the Kalman filter will be used to generate the NMEA message
    MIP_NMEAMESSAGE_FORMAT_SOURCE_ID_IMU    = 2,  ///<  Data from the IMU/IMU derived quantities will be used to generate the NMEA message
    MIP_NMEAMESSAGE_FORMAT_SOURCE_ID_GNSS1  = 3,  ///<  Data from GNSS1 will be used to generate the NMEA message
    MIP_NMEAMESSAGE_FORMAT_SOURCE_ID_GNSS2  = 4,  ///<  Data from GNSS2 will be used to generate the NMEA message
};
size_t insert_mip_nmeamessage_format_source_id(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_nmeamessage_format_source_id self);
size_t extract_mip_nmeamessage_format_source_id(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_nmeamessage_format_source_id* self);

struct mip_nmeamessage_format
{
    enum mip_nmeamessage_format_message_id            message_id;
    enum mip_nmeamessage_format_talker_id             talker_id;
    enum mip_nmeamessage_format_source_id             source_id;
    uint16_t                                          decimation;
};
size_t insert_mip_nmeamessage_format(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_nmeamessage_format* self);
size_t extract_mip_nmeamessage_format(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_nmeamessage_format* self);

enum mip_pps_source
{
    MIP_PPS_SOURCE_DISABLED   = 0,  ///<  PPS output is disabled. Not valid for PPS source command.
    MIP_PPS_SOURCE_RECEIVER_1 = 1,  ///<  PPS is provided by GNSS receiver 1.
    MIP_PPS_SOURCE_RECEIVER_2 = 2,  ///<  PPS is provided by GNSS receiver 2.
    MIP_PPS_SOURCE_GPIO       = 3,  ///<  PPS is provided to an external GPIO pin. Use the GPIO Setup command to choose and configure the pin.
    MIP_PPS_SOURCE_GENERATED  = 4,  ///<  PPS is generated from the system oscillator.
};
size_t insert_mip_pps_source(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_pps_source self);
size_t extract_mip_pps_source(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_pps_source* self);

enum mip_sensor_range_type
{
    MIP_SENSOR_RANGE_TYPE_ALL   = 0,  ///<  Only allowed for SAVE, LOAD, and DEFAULT function selectors.
    MIP_SENSOR_RANGE_TYPE_ACCEL = 1,  ///<  Accelerometer. Range is specified in g.
    MIP_SENSOR_RANGE_TYPE_GYRO  = 2,  ///<  Gyroscope. Range is specified in degrees/s.
    MIP_SENSOR_RANGE_TYPE_MAG   = 3,  ///<  Magnetometer. Range is specified in Gauss.
    MIP_SENSOR_RANGE_TYPE_PRESS = 4,  ///<  Pressure sensor. Range is specified in hPa.
};
size_t insert_mip_sensor_range_type(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_sensor_range_type self);
size_t extract_mip_sensor_range_type(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_sensor_range_type* self);


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
///@defgroup poll_imu_message  None
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

struct mip_3dm_poll_imu_message_command
{
    bool                                              suppress_ack;
    uint8_t                                           num_descriptors;
    struct mip_descriptor_rate*                       descriptors;
};
size_t insert_mip_3dm_poll_imu_message_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_3dm_poll_imu_message_command* self);
size_t extract_mip_3dm_poll_imu_message_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_3dm_poll_imu_message_command* self);

mip_cmd_result mip_3dm_poll_imu_message(struct mip_interface* device, bool suppress_ack, uint8_t num_descriptors, const struct mip_descriptor_rate* descriptors);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup poll_gnss_message  None
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

struct mip_3dm_poll_gnss_message_command
{
    bool                                              suppress_ack;
    uint8_t                                           num_descriptors;
    struct mip_descriptor_rate*                       descriptors;
};
size_t insert_mip_3dm_poll_gnss_message_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_3dm_poll_gnss_message_command* self);
size_t extract_mip_3dm_poll_gnss_message_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_3dm_poll_gnss_message_command* self);

mip_cmd_result mip_3dm_poll_gnss_message(struct mip_interface* device, bool suppress_ack, uint8_t num_descriptors, const struct mip_descriptor_rate* descriptors);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup poll_filter_message  None
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

struct mip_3dm_poll_filter_message_command
{
    bool                                              suppress_ack;
    uint8_t                                           num_descriptors;
    struct mip_descriptor_rate*                       descriptors;
};
size_t insert_mip_3dm_poll_filter_message_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_3dm_poll_filter_message_command* self);
size_t extract_mip_3dm_poll_filter_message_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_3dm_poll_filter_message_command* self);

mip_cmd_result mip_3dm_poll_filter_message(struct mip_interface* device, bool suppress_ack, uint8_t num_descriptors, const struct mip_descriptor_rate* descriptors);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup imu_message_format  None
/// Set, read, or save the format of the IMU data packet.
/// 
/// The resulting data messages will maintain the order of descriptors sent in the command.
///
///@{

struct mip_3dm_imu_message_format_command
{
    enum mip_function_selector                        function;
    uint8_t                                           num_descriptors;
    struct mip_descriptor_rate*                       descriptors;
};
size_t insert_mip_3dm_imu_message_format_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_3dm_imu_message_format_command* self);
size_t extract_mip_3dm_imu_message_format_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_3dm_imu_message_format_command* self);

struct mip_3dm_imu_message_format_response
{
    uint8_t                                           num_descriptors;
    struct mip_descriptor_rate*                       descriptors;
};
size_t insert_mip_3dm_imu_message_format_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_3dm_imu_message_format_response* self);
size_t extract_mip_3dm_imu_message_format_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_3dm_imu_message_format_response* self);

mip_cmd_result mip_3dm_write_imu_message_format(struct mip_interface* device, uint8_t num_descriptors, const struct mip_descriptor_rate* descriptors);
mip_cmd_result mip_3dm_read_imu_message_format(struct mip_interface* device, uint8_t* num_descriptors, struct mip_descriptor_rate* descriptors);
mip_cmd_result mip_3dm_save_imu_message_format(struct mip_interface* device);
mip_cmd_result mip_3dm_load_imu_message_format(struct mip_interface* device);
mip_cmd_result mip_3dm_default_imu_message_format(struct mip_interface* device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup gps_message_format  None
/// Set, read, or save the format of the GNSS data packet.
/// 
/// The resulting data messages will maintain the order of descriptors sent in the command.
///
///@{

struct mip_3dm_gps_message_format_command
{
    enum mip_function_selector                        function;
    uint8_t                                           num_descriptors;
    struct mip_descriptor_rate*                       descriptors;
};
size_t insert_mip_3dm_gps_message_format_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_3dm_gps_message_format_command* self);
size_t extract_mip_3dm_gps_message_format_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_3dm_gps_message_format_command* self);

struct mip_3dm_gps_message_format_response
{
    uint8_t                                           num_descriptors;
    struct mip_descriptor_rate*                       descriptors;
};
size_t insert_mip_3dm_gps_message_format_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_3dm_gps_message_format_response* self);
size_t extract_mip_3dm_gps_message_format_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_3dm_gps_message_format_response* self);

mip_cmd_result mip_3dm_write_gps_message_format(struct mip_interface* device, uint8_t num_descriptors, const struct mip_descriptor_rate* descriptors);
mip_cmd_result mip_3dm_read_gps_message_format(struct mip_interface* device, uint8_t* num_descriptors, struct mip_descriptor_rate* descriptors);
mip_cmd_result mip_3dm_save_gps_message_format(struct mip_interface* device);
mip_cmd_result mip_3dm_load_gps_message_format(struct mip_interface* device);
mip_cmd_result mip_3dm_default_gps_message_format(struct mip_interface* device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup filter_message_format  None
/// Set, read, or save the format of the Estimation Filter data packet.
/// 
/// The resulting data messages will maintain the order of descriptors sent in the command.
///
///@{

struct mip_3dm_filter_message_format_command
{
    enum mip_function_selector                        function;
    uint8_t                                           num_descriptors;
    struct mip_descriptor_rate*                       descriptors;
};
size_t insert_mip_3dm_filter_message_format_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_3dm_filter_message_format_command* self);
size_t extract_mip_3dm_filter_message_format_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_3dm_filter_message_format_command* self);

struct mip_3dm_filter_message_format_response
{
    uint8_t                                           num_descriptors;
    struct mip_descriptor_rate*                       descriptors;
};
size_t insert_mip_3dm_filter_message_format_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_3dm_filter_message_format_response* self);
size_t extract_mip_3dm_filter_message_format_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_3dm_filter_message_format_response* self);

mip_cmd_result mip_3dm_write_filter_message_format(struct mip_interface* device, uint8_t num_descriptors, const struct mip_descriptor_rate* descriptors);
mip_cmd_result mip_3dm_read_filter_message_format(struct mip_interface* device, uint8_t* num_descriptors, struct mip_descriptor_rate* descriptors);
mip_cmd_result mip_3dm_save_filter_message_format(struct mip_interface* device);
mip_cmd_result mip_3dm_load_filter_message_format(struct mip_interface* device);
mip_cmd_result mip_3dm_default_filter_message_format(struct mip_interface* device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup imu_get_base_rate  Get IMU Data Base Rate
/// Get the base rate for the IMU data in Hz
/// 
/// This is the fastest rate for this type of data available on the device.
/// This is used in conjunction with the IMU Message Format Command to set streaming data at a specified rate.
///
///@{

struct mip_3dm_imu_get_base_rate_command
{
};
size_t insert_mip_3dm_imu_get_base_rate_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_3dm_imu_get_base_rate_command* self);
size_t extract_mip_3dm_imu_get_base_rate_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_3dm_imu_get_base_rate_command* self);

struct mip_3dm_imu_get_base_rate_response
{
    uint16_t                                          rate;
};
size_t insert_mip_3dm_imu_get_base_rate_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_3dm_imu_get_base_rate_response* self);
size_t extract_mip_3dm_imu_get_base_rate_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_3dm_imu_get_base_rate_response* self);

mip_cmd_result mip_3dm_imu_get_base_rate(struct mip_interface* device, uint16_t* rate);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup gps_get_base_rate  Get GNSS Data Base Rate
/// Get the base rate for the GNSS data in Hz
/// 
/// This is the fastest rate for this type of data available on the device.
/// This is used in conjunction with the GNSS Message Format Command to set streaming data at a specified rate.
///
///@{

struct mip_3dm_gps_get_base_rate_command
{
};
size_t insert_mip_3dm_gps_get_base_rate_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_3dm_gps_get_base_rate_command* self);
size_t extract_mip_3dm_gps_get_base_rate_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_3dm_gps_get_base_rate_command* self);

struct mip_3dm_gps_get_base_rate_response
{
    uint16_t                                          rate;
};
size_t insert_mip_3dm_gps_get_base_rate_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_3dm_gps_get_base_rate_response* self);
size_t extract_mip_3dm_gps_get_base_rate_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_3dm_gps_get_base_rate_response* self);

mip_cmd_result mip_3dm_gps_get_base_rate(struct mip_interface* device, uint16_t* rate);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup filter_get_base_rate  Get Estimation Filter Data Base Rate
/// Get the base rate for the Estimation Filter data in Hz
/// 
/// This is the fastest rate for this type of data available on the device.
/// This is used in conjunction with the Estimation Filter Message Format Command to set streaming data at a specified rate.
///
///@{

struct mip_3dm_filter_get_base_rate_command
{
};
size_t insert_mip_3dm_filter_get_base_rate_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_3dm_filter_get_base_rate_command* self);
size_t extract_mip_3dm_filter_get_base_rate_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_3dm_filter_get_base_rate_command* self);

struct mip_3dm_filter_get_base_rate_response
{
    uint16_t                                          rate;
};
size_t insert_mip_3dm_filter_get_base_rate_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_3dm_filter_get_base_rate_response* self);
size_t extract_mip_3dm_filter_get_base_rate_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_3dm_filter_get_base_rate_response* self);

mip_cmd_result mip_3dm_filter_get_base_rate(struct mip_interface* device, uint16_t* rate);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup poll_data  None
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

struct mip_3dm_poll_data_command
{
    uint8_t                                           desc_set;
    bool                                              suppress_ack;
    uint8_t                                           num_descriptors;
    uint8_t*                                          descriptors;
};
size_t insert_mip_3dm_poll_data_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_3dm_poll_data_command* self);
size_t extract_mip_3dm_poll_data_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_3dm_poll_data_command* self);

mip_cmd_result mip_3dm_poll_data(struct mip_interface* device, uint8_t desc_set, bool suppress_ack, uint8_t num_descriptors, const uint8_t* descriptors);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup get_base_rate  Get Data Base Rate
/// Get the base rate for the specified descriptor set in Hz.
///
///@{

struct mip_3dm_get_base_rate_command
{
    uint8_t                                           desc_set;
};
size_t insert_mip_3dm_get_base_rate_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_3dm_get_base_rate_command* self);
size_t extract_mip_3dm_get_base_rate_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_3dm_get_base_rate_command* self);

struct mip_3dm_get_base_rate_response
{
    uint8_t                                           desc_set;
    uint16_t                                          rate;
};
size_t insert_mip_3dm_get_base_rate_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_3dm_get_base_rate_response* self);
size_t extract_mip_3dm_get_base_rate_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_3dm_get_base_rate_response* self);

mip_cmd_result mip_3dm_get_base_rate(struct mip_interface* device, uint8_t desc_set, uint16_t* rate);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup message_format  None
/// Set, read, or save the format for a given data packet.
/// 
/// The resulting data messages will maintain the order of descriptors sent in the command.
///
///@{

struct mip_3dm_message_format_command
{
    enum mip_function_selector                        function;
    uint8_t                                           desc_set;
    uint8_t                                           num_descriptors;
    struct mip_descriptor_rate*                       descriptors;
};
size_t insert_mip_3dm_message_format_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_3dm_message_format_command* self);
size_t extract_mip_3dm_message_format_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_3dm_message_format_command* self);

struct mip_3dm_message_format_response
{
    uint8_t                                           desc_set;
    uint8_t                                           num_descriptors;
    struct mip_descriptor_rate*                       descriptors;
};
size_t insert_mip_3dm_message_format_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_3dm_message_format_response* self);
size_t extract_mip_3dm_message_format_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_3dm_message_format_response* self);

mip_cmd_result mip_3dm_write_message_format(struct mip_interface* device, uint8_t desc_set, uint8_t num_descriptors, const struct mip_descriptor_rate* descriptors);
mip_cmd_result mip_3dm_read_message_format(struct mip_interface* device, uint8_t desc_set, uint8_t* num_descriptors, struct mip_descriptor_rate* descriptors);
mip_cmd_result mip_3dm_save_message_format(struct mip_interface* device, uint8_t desc_set);
mip_cmd_result mip_3dm_load_message_format(struct mip_interface* device, uint8_t desc_set);
mip_cmd_result mip_3dm_default_message_format(struct mip_interface* device, uint8_t desc_set);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup nmea_poll_data  None
/// Poll the device for a NMEA message with the specified format.
/// 
/// This function polls for a NMEA message using the provided format.
/// If the format is not provided, the device will attempt to use the
/// stored format (set with the Set NMEA Message Format command.) If no format is provided
/// and there is no stored format, the device will respond with a NACK. The reply packet contains
/// an ACK/NACK field. The polled data packet is sent separately as normal NMEA messages.
///
///@{

struct mip_3dm_nmea_poll_data_command
{
    bool                                              suppress_ack;
    uint8_t                                           count;
    struct mip_nmeamessage_format*                    format_entries;
};
size_t insert_mip_3dm_nmea_poll_data_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_3dm_nmea_poll_data_command* self);
size_t extract_mip_3dm_nmea_poll_data_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_3dm_nmea_poll_data_command* self);

mip_cmd_result mip_3dm_nmea_poll_data(struct mip_interface* device, bool suppress_ack, uint8_t count, const struct mip_nmeamessage_format* format_entries);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup nmea_message_format  None
/// Set, read, or save the NMEA message format.
///
///@{

struct mip_3dm_nmea_message_format_command
{
    enum mip_function_selector                        function;
    uint8_t                                           count;
    struct mip_nmeamessage_format*                    format_entries;
};
size_t insert_mip_3dm_nmea_message_format_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_3dm_nmea_message_format_command* self);
size_t extract_mip_3dm_nmea_message_format_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_3dm_nmea_message_format_command* self);

struct mip_3dm_nmea_message_format_response
{
    uint8_t                                           count;
    struct mip_nmeamessage_format*                    format_entries;
};
size_t insert_mip_3dm_nmea_message_format_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_3dm_nmea_message_format_response* self);
size_t extract_mip_3dm_nmea_message_format_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_3dm_nmea_message_format_response* self);

mip_cmd_result mip_3dm_write_nmea_message_format(struct mip_interface* device, uint8_t count, const struct mip_nmeamessage_format* format_entries);
mip_cmd_result mip_3dm_read_nmea_message_format(struct mip_interface* device, uint8_t* count, struct mip_nmeamessage_format* format_entries);
mip_cmd_result mip_3dm_save_nmea_message_format(struct mip_interface* device);
mip_cmd_result mip_3dm_load_nmea_message_format(struct mip_interface* device);
mip_cmd_result mip_3dm_default_nmea_message_format(struct mip_interface* device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup device_settings  None
/// Save, Load, or Reset to Default the values for all device settings.
/// 
/// When a save current settings command is issued, a brief data disturbance may occur while all settings are written to non-volatile memory.
/// 
/// This command should have a long timeout as it may take up to 1 second to complete.
///
///@{

struct mip_3dm_device_settings_command
{
    enum mip_function_selector                        function;
};
size_t insert_mip_3dm_device_settings_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_3dm_device_settings_command* self);
size_t extract_mip_3dm_device_settings_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_3dm_device_settings_command* self);

mip_cmd_result mip_3dm_save_device_settings(struct mip_interface* device);
mip_cmd_result mip_3dm_load_device_settings(struct mip_interface* device);
mip_cmd_result mip_3dm_default_device_settings(struct mip_interface* device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup uart_baudrate  None
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

struct mip_3dm_uart_baudrate_command
{
    enum mip_function_selector                        function;
    uint32_t                                          baud;
};
size_t insert_mip_3dm_uart_baudrate_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_3dm_uart_baudrate_command* self);
size_t extract_mip_3dm_uart_baudrate_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_3dm_uart_baudrate_command* self);

struct mip_3dm_uart_baudrate_response
{
    uint32_t                                          baud;
};
size_t insert_mip_3dm_uart_baudrate_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_3dm_uart_baudrate_response* self);
size_t extract_mip_3dm_uart_baudrate_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_3dm_uart_baudrate_response* self);

mip_cmd_result mip_3dm_write_uart_baudrate(struct mip_interface* device, uint32_t baud);
mip_cmd_result mip_3dm_read_uart_baudrate(struct mip_interface* device, uint32_t* baud);
mip_cmd_result mip_3dm_save_uart_baudrate(struct mip_interface* device);
mip_cmd_result mip_3dm_load_uart_baudrate(struct mip_interface* device);
mip_cmd_result mip_3dm_default_uart_baudrate(struct mip_interface* device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup factory_streaming  None
/// Configures the device for recording data for technical support.
/// 
/// This command will configure all available data streams to predefined
/// formats designed to be used with technical support.
///
///@{

enum mip_3dm_factory_streaming_command_action
{
    MIP_3DM_FACTORY_STREAMING_COMMAND_ACTION_OVERWRITE = 0,  ///<  Replaces the message format(s), removing any existing descriptors.
    MIP_3DM_FACTORY_STREAMING_COMMAND_ACTION_MERGE     = 1,  ///<  Merges support descriptors into existing format(s). May reorder descriptors.
    MIP_3DM_FACTORY_STREAMING_COMMAND_ACTION_ADD       = 2,  ///<  Adds descriptors to the current message format(s) without changing existing descriptors. May result in duplicates.
};
size_t insert_mip_3dm_factory_streaming_command_action(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_3dm_factory_streaming_command_action self);
size_t extract_mip_3dm_factory_streaming_command_action(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_3dm_factory_streaming_command_action* self);

struct mip_3dm_factory_streaming_command
{
    enum mip_3dm_factory_streaming_command_action     action;
    uint8_t                                           reserved;
};
size_t insert_mip_3dm_factory_streaming_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_3dm_factory_streaming_command* self);
size_t extract_mip_3dm_factory_streaming_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_3dm_factory_streaming_command* self);

mip_cmd_result mip_3dm_factory_streaming(struct mip_interface* device, enum mip_3dm_factory_streaming_command_action action, uint8_t reserved);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup datastream_control  None
/// Enable/disable the selected data stream.
/// 
/// Each data stream (descriptor set) can be enabled or disabled.
/// The default for the device is all streams enabled.
/// For all functions except 0x01 (use new setting),
/// the new enable flag value is ignored and can be omitted.
///
///@{

struct mip_3dm_datastream_control_command
{
    enum mip_function_selector                        function;
    uint8_t                                           desc_set;
    bool                                              enable;
};
size_t insert_mip_3dm_datastream_control_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_3dm_datastream_control_command* self);
size_t extract_mip_3dm_datastream_control_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_3dm_datastream_control_command* self);

struct mip_3dm_datastream_control_response
{
    uint8_t                                           desc_set;
    bool                                              enabled;
};
size_t insert_mip_3dm_datastream_control_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_3dm_datastream_control_response* self);
size_t extract_mip_3dm_datastream_control_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_3dm_datastream_control_response* self);

mip_cmd_result mip_3dm_write_datastream_control(struct mip_interface* device, uint8_t desc_set, bool enable);
mip_cmd_result mip_3dm_read_datastream_control(struct mip_interface* device, uint8_t desc_set, bool* enabled);
mip_cmd_result mip_3dm_save_datastream_control(struct mip_interface* device, uint8_t desc_set);
mip_cmd_result mip_3dm_load_datastream_control(struct mip_interface* device, uint8_t desc_set);
mip_cmd_result mip_3dm_default_datastream_control(struct mip_interface* device, uint8_t desc_set);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup gnss_sbas_settings  SBAS Settings
/// Configure the SBAS subsystem
/// 
/// 
/// 
///
///@{

enum mip_3dm_gnss_sbas_settings_command_sbasoptions
{
    MIP_3DM_GNSS_SBAS_SETTINGS_COMMAND_SBASOPTIONS_ENABLE_RANGING     = 0x01,
    MIP_3DM_GNSS_SBAS_SETTINGS_COMMAND_SBASOPTIONS_ENABLE_CORRECTIONS = 0x02,
    MIP_3DM_GNSS_SBAS_SETTINGS_COMMAND_SBASOPTIONS_APPLY_INTEGRITY    = 0x04,
};
size_t insert_mip_3dm_gnss_sbas_settings_command_sbasoptions(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_3dm_gnss_sbas_settings_command_sbasoptions self);
size_t extract_mip_3dm_gnss_sbas_settings_command_sbasoptions(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_3dm_gnss_sbas_settings_command_sbasoptions* self);

struct mip_3dm_gnss_sbas_settings_command
{
    enum mip_function_selector                        function;
    uint8_t                                           enable_sbas;
    enum mip_3dm_gnss_sbas_settings_command_sbasoptions sbas_options;
    uint8_t                                           num_included_prns;
    uint16_t*                                         included_prns;
};
size_t insert_mip_3dm_gnss_sbas_settings_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_3dm_gnss_sbas_settings_command* self);
size_t extract_mip_3dm_gnss_sbas_settings_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_3dm_gnss_sbas_settings_command* self);

struct mip_3dm_gnss_sbas_settings_response
{
    uint8_t                                           enable_sbas;
    enum mip_3dm_gnss_sbas_settings_command_sbasoptions sbas_options;
    uint8_t                                           num_included_prns;
    uint16_t*                                         included_prns;
};
size_t insert_mip_3dm_gnss_sbas_settings_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_3dm_gnss_sbas_settings_response* self);
size_t extract_mip_3dm_gnss_sbas_settings_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_3dm_gnss_sbas_settings_response* self);

mip_cmd_result mip_3dm_write_gnss_sbas_settings(struct mip_interface* device, uint8_t enable_sbas, enum mip_3dm_gnss_sbas_settings_command_sbasoptions sbas_options, uint8_t num_included_prns, const uint16_t* included_prns);
mip_cmd_result mip_3dm_read_gnss_sbas_settings(struct mip_interface* device, uint8_t* enable_sbas, enum mip_3dm_gnss_sbas_settings_command_sbasoptions* sbas_options, uint8_t* num_included_prns, uint16_t* included_prns);
mip_cmd_result mip_3dm_save_gnss_sbas_settings(struct mip_interface* device);
mip_cmd_result mip_3dm_load_gnss_sbas_settings(struct mip_interface* device);
mip_cmd_result mip_3dm_default_gnss_sbas_settings(struct mip_interface* device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup gnss_time_assistance  None
/// Provide the GNSS subsystem with initial time information.
/// 
/// This message is required immediately after power up if GNSS Assist was enabled when the device was powered off.
/// This will initialize the subsystem clock to help reduce the time to first fix (TTFF).
///
///@{

struct mip_3dm_gnss_time_assistance_command
{
    enum mip_function_selector                        function;
    double                                            tow;
    uint16_t                                          week_number;
    float                                             accuracy;
};
size_t insert_mip_3dm_gnss_time_assistance_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_3dm_gnss_time_assistance_command* self);
size_t extract_mip_3dm_gnss_time_assistance_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_3dm_gnss_time_assistance_command* self);

struct mip_3dm_gnss_time_assistance_response
{
    double                                            tow;
    uint16_t                                          week_number;
    float                                             accuracy;
};
size_t insert_mip_3dm_gnss_time_assistance_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_3dm_gnss_time_assistance_response* self);
size_t extract_mip_3dm_gnss_time_assistance_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_3dm_gnss_time_assistance_response* self);

mip_cmd_result mip_3dm_write_gnss_time_assistance(struct mip_interface* device, double tow, uint16_t week_number, float accuracy);
mip_cmd_result mip_3dm_read_gnss_time_assistance(struct mip_interface* device, double* tow, uint16_t* week_number, float* accuracy);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup adv_lowpass_filter  Advanced Low-Pass Filter Settings
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

struct mip_3dm_adv_lowpass_filter_command
{
    enum mip_function_selector                        function;
    uint8_t                                           target_descriptor;
    bool                                              enable;
    bool                                              manual;
    uint16_t                                          frequency;
    uint8_t                                           reserved;
};
size_t insert_mip_3dm_adv_lowpass_filter_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_3dm_adv_lowpass_filter_command* self);
size_t extract_mip_3dm_adv_lowpass_filter_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_3dm_adv_lowpass_filter_command* self);

struct mip_3dm_adv_lowpass_filter_response
{
    uint8_t                                           target_descriptor;
    bool                                              enable;
    bool                                              manual;
    uint16_t                                          frequency;
    uint8_t                                           reserved;
};
size_t insert_mip_3dm_adv_lowpass_filter_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_3dm_adv_lowpass_filter_response* self);
size_t extract_mip_3dm_adv_lowpass_filter_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_3dm_adv_lowpass_filter_response* self);

mip_cmd_result mip_3dm_write_adv_lowpass_filter(struct mip_interface* device, uint8_t target_descriptor, bool enable, bool manual, uint16_t frequency, uint8_t reserved);
mip_cmd_result mip_3dm_read_adv_lowpass_filter(struct mip_interface* device, uint8_t target_descriptor, bool* enable, bool* manual, uint16_t* frequency, uint8_t* reserved);
mip_cmd_result mip_3dm_save_adv_lowpass_filter(struct mip_interface* device, uint8_t target_descriptor);
mip_cmd_result mip_3dm_load_adv_lowpass_filter(struct mip_interface* device, uint8_t target_descriptor);
mip_cmd_result mip_3dm_default_adv_lowpass_filter(struct mip_interface* device, uint8_t target_descriptor);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup pps_source  None
/// Controls the Pulse Per Second (PPS) source.
///
///@{

struct mip_3dm_pps_source_command
{
    enum mip_function_selector                        function;
    enum mip_pps_source                               source;
};
size_t insert_mip_3dm_pps_source_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_3dm_pps_source_command* self);
size_t extract_mip_3dm_pps_source_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_3dm_pps_source_command* self);

struct mip_3dm_pps_source_response
{
    enum mip_pps_source                               source;
};
size_t insert_mip_3dm_pps_source_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_3dm_pps_source_response* self);
size_t extract_mip_3dm_pps_source_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_3dm_pps_source_response* self);

mip_cmd_result mip_3dm_write_pps_source(struct mip_interface* device, enum mip_pps_source source);
mip_cmd_result mip_3dm_read_pps_source(struct mip_interface* device, enum mip_pps_source* source);
mip_cmd_result mip_3dm_save_pps_source(struct mip_interface* device);
mip_cmd_result mip_3dm_load_pps_source(struct mip_interface* device);
mip_cmd_result mip_3dm_default_pps_source(struct mip_interface* device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup gpio_config  GPIO Configuration
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

enum mip_3dm_gpio_config_command_feature
{
    MIP_3DM_GPIO_CONFIG_COMMAND_FEATURE_UNUSED    = 0,  ///<  The pin is not used. It may be technically possible to read the pin state in this mode, but this is not guaranteed to be true of all devices or pins.
    MIP_3DM_GPIO_CONFIG_COMMAND_FEATURE_GPIO      = 1,  ///<  General purpose input or output. Use this for direct control of pin output state or to stream the state of the pin.
    MIP_3DM_GPIO_CONFIG_COMMAND_FEATURE_PPS       = 2,  ///<  Pulse per second input or output.
    MIP_3DM_GPIO_CONFIG_COMMAND_FEATURE_ENCODER   = 3,  ///<  Motor encoder/odometer input.
    MIP_3DM_GPIO_CONFIG_COMMAND_FEATURE_TIMESTAMP = 4,  ///<  Precision Timestamping. Use with Event Trigger Configuration (0x0C,0x2E).
    MIP_3DM_GPIO_CONFIG_COMMAND_FEATURE_POWER     = 5,  ///<  Controls the device power state (e.g. enter low power mode).
};
size_t insert_mip_3dm_gpio_config_command_feature(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_3dm_gpio_config_command_feature self);
size_t extract_mip_3dm_gpio_config_command_feature(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_3dm_gpio_config_command_feature* self);

enum mip_3dm_gpio_config_command_behavior
{
    MIP_3DM_GPIO_CONFIG_COMMAND_BEHAVIOR_UNUSED            = 0,  ///<  Use 0 unless otherwise specified.
    MIP_3DM_GPIO_CONFIG_COMMAND_BEHAVIOR_GPIO_INPUT        = 1,  ///<  Pin will be an input. This can be used to stream or poll the value and is the default setting.
    MIP_3DM_GPIO_CONFIG_COMMAND_BEHAVIOR_GPIO_OUTPUT_LOW   = 2,  ///<  Pin is an output initially in the LOW state. This state will be restored during system startup if the configuration is saved.
    MIP_3DM_GPIO_CONFIG_COMMAND_BEHAVIOR_GPIO_OUTPUT_HIGH  = 3,  ///<  Pin is an output initially in the HIGH state. This state will be restored during system startup if the configuration is saved.
    MIP_3DM_GPIO_CONFIG_COMMAND_BEHAVIOR_PPS_INPUT         = 1,  ///<  Pin will receive the pulse-per-second signal. Only one pin can have this behavior. This will only work if the PPS Source command is configured to GPIO.
    MIP_3DM_GPIO_CONFIG_COMMAND_BEHAVIOR_PPS_OUTPUT        = 2,  ///<  Pin will transmit the pulse-per-second signal from the device.
    MIP_3DM_GPIO_CONFIG_COMMAND_BEHAVIOR_ENCODER_A         = 1,  ///<  Encoder "A" quadrature input. Only one pin can have this behavior. The last command to set this behavior will take precedence.
    MIP_3DM_GPIO_CONFIG_COMMAND_BEHAVIOR_ENCODER_B         = 2,  ///<  Encoder "B" quadrature input. Only one pin can have this behavior. The last command to set this behavior will take precedence.
    MIP_3DM_GPIO_CONFIG_COMMAND_BEHAVIOR_TIMESTAMP_RISING  = 1,  ///<  Rising edges will be timestamped.
    MIP_3DM_GPIO_CONFIG_COMMAND_BEHAVIOR_TIMESTAMP_FALLING = 2,  ///<  Falling edges will be timestamped.
    MIP_3DM_GPIO_CONFIG_COMMAND_BEHAVIOR_TIMESTAMP_EITHER  = 3,  ///<  Both rising and falling edges will be timestamped.
    MIP_3DM_GPIO_CONFIG_COMMAND_BEHAVIOR_POWER_SHUTDOWN    = 1,  ///<  A logic 1 applied to the pin will place the device in low-power mode. A full restart is executed after the signal is removed.
};
size_t insert_mip_3dm_gpio_config_command_behavior(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_3dm_gpio_config_command_behavior self);
size_t extract_mip_3dm_gpio_config_command_behavior(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_3dm_gpio_config_command_behavior* self);

enum mip_3dm_gpio_config_command_pin_mode
{
    MIP_3DM_GPIO_CONFIG_COMMAND_PIN_MODE_OPEN_DRAIN = 0x01,
    MIP_3DM_GPIO_CONFIG_COMMAND_PIN_MODE_PULLDOWN   = 0x02,
    MIP_3DM_GPIO_CONFIG_COMMAND_PIN_MODE_PULLUP     = 0x04,
};
size_t insert_mip_3dm_gpio_config_command_pin_mode(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_3dm_gpio_config_command_pin_mode self);
size_t extract_mip_3dm_gpio_config_command_pin_mode(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_3dm_gpio_config_command_pin_mode* self);

struct mip_3dm_gpio_config_command
{
    enum mip_function_selector                        function;
    uint8_t                                           pin;
    enum mip_3dm_gpio_config_command_feature          feature;
    enum mip_3dm_gpio_config_command_behavior         behavior;
    enum mip_3dm_gpio_config_command_pin_mode         pin_mode;
};
size_t insert_mip_3dm_gpio_config_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_3dm_gpio_config_command* self);
size_t extract_mip_3dm_gpio_config_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_3dm_gpio_config_command* self);

struct mip_3dm_gpio_config_response
{
    uint8_t                                           pin;
    enum mip_3dm_gpio_config_command_feature          feature;
    enum mip_3dm_gpio_config_command_behavior         behavior;
    enum mip_3dm_gpio_config_command_pin_mode         pin_mode;
};
size_t insert_mip_3dm_gpio_config_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_3dm_gpio_config_response* self);
size_t extract_mip_3dm_gpio_config_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_3dm_gpio_config_response* self);

mip_cmd_result mip_3dm_write_gpio_config(struct mip_interface* device, uint8_t pin, enum mip_3dm_gpio_config_command_feature feature, enum mip_3dm_gpio_config_command_behavior behavior, enum mip_3dm_gpio_config_command_pin_mode pin_mode);
mip_cmd_result mip_3dm_read_gpio_config(struct mip_interface* device, uint8_t pin, enum mip_3dm_gpio_config_command_feature* feature, enum mip_3dm_gpio_config_command_behavior* behavior, enum mip_3dm_gpio_config_command_pin_mode* pin_mode);
mip_cmd_result mip_3dm_save_gpio_config(struct mip_interface* device, uint8_t pin);
mip_cmd_result mip_3dm_load_gpio_config(struct mip_interface* device, uint8_t pin);
mip_cmd_result mip_3dm_default_gpio_config(struct mip_interface* device, uint8_t pin);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup gpio_state  GPIO State
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

struct mip_3dm_gpio_state_command
{
    enum mip_function_selector                        function;
    uint8_t                                           pin;
    bool                                              state;
};
size_t insert_mip_3dm_gpio_state_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_3dm_gpio_state_command* self);
size_t extract_mip_3dm_gpio_state_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_3dm_gpio_state_command* self);

struct mip_3dm_gpio_state_response
{
    uint8_t                                           pin;
    bool                                              state;
};
size_t insert_mip_3dm_gpio_state_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_3dm_gpio_state_response* self);
size_t extract_mip_3dm_gpio_state_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_3dm_gpio_state_response* self);

mip_cmd_result mip_3dm_write_gpio_state(struct mip_interface* device, uint8_t pin, bool state);
mip_cmd_result mip_3dm_read_gpio_state(struct mip_interface* device, uint8_t pin, bool* state);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup odometer  Odometer Settings
/// Configures the hardware odometer interface.
/// 
///
///@{

enum mip_3dm_odometer_command_mode
{
    MIP_3DM_ODOMETER_COMMAND_MODE_DISABLED   = 0,  ///<  Encoder is disabled.
    MIP_3DM_ODOMETER_COMMAND_MODE_QUADRATURE = 2,  ///<  Quadrature encoder mode.
};
size_t insert_mip_3dm_odometer_command_mode(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_3dm_odometer_command_mode self);
size_t extract_mip_3dm_odometer_command_mode(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_3dm_odometer_command_mode* self);

struct mip_3dm_odometer_command
{
    enum mip_function_selector                        function;
    enum mip_3dm_odometer_command_mode                mode;
    float                                             scaling;
    float                                             uncertainty;
};
size_t insert_mip_3dm_odometer_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_3dm_odometer_command* self);
size_t extract_mip_3dm_odometer_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_3dm_odometer_command* self);

struct mip_3dm_odometer_response
{
    enum mip_3dm_odometer_command_mode                mode;
    float                                             scaling;
    float                                             uncertainty;
};
size_t insert_mip_3dm_odometer_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_3dm_odometer_response* self);
size_t extract_mip_3dm_odometer_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_3dm_odometer_response* self);

mip_cmd_result mip_3dm_write_odometer(struct mip_interface* device, enum mip_3dm_odometer_command_mode mode, float scaling, float uncertainty);
mip_cmd_result mip_3dm_read_odometer(struct mip_interface* device, enum mip_3dm_odometer_command_mode* mode, float* scaling, float* uncertainty);
mip_cmd_result mip_3dm_save_odometer(struct mip_interface* device);
mip_cmd_result mip_3dm_load_odometer(struct mip_interface* device);
mip_cmd_result mip_3dm_default_odometer(struct mip_interface* device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup get_event_support  Get Supported Events
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

enum mip_3dm_get_event_support_command_query
{
    MIP_3DM_GET_EVENT_SUPPORT_COMMAND_QUERY_TRIGGER_TYPES = 1,  ///<  Query the supported trigger types and max count for each.
    MIP_3DM_GET_EVENT_SUPPORT_COMMAND_QUERY_ACTION_TYPES  = 2,  ///<  Query the supported action types and max count for each.
};
size_t insert_mip_3dm_get_event_support_command_query(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_3dm_get_event_support_command_query self);
size_t extract_mip_3dm_get_event_support_command_query(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_3dm_get_event_support_command_query* self);

struct mip_3dm_get_event_support_command
{
    enum mip_3dm_get_event_support_command_query      query;
};
size_t insert_mip_3dm_get_event_support_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_3dm_get_event_support_command* self);
size_t extract_mip_3dm_get_event_support_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_3dm_get_event_support_command* self);

struct mip_3dm_get_event_support_command_info
{
    uint8_t                                           type;
    uint8_t                                           count;
};
size_t insert_mip_3dm_get_event_support_command_info(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_3dm_get_event_support_command_info* self);
size_t extract_mip_3dm_get_event_support_command_info(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_3dm_get_event_support_command_info* self);

struct mip_3dm_get_event_support_response
{
    enum mip_3dm_get_event_support_command_query      query;
    uint8_t                                           max_instances;
    uint8_t                                           num_entries;
    struct mip_3dm_get_event_support_command_info     entries[126];
};
size_t insert_mip_3dm_get_event_support_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_3dm_get_event_support_response* self);
size_t extract_mip_3dm_get_event_support_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_3dm_get_event_support_response* self);

mip_cmd_result mip_3dm_get_event_support(struct mip_interface* device, enum mip_3dm_get_event_support_command_query query, uint8_t* max_instances, uint8_t* num_entries, struct mip_3dm_get_event_support_command_info* entries);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup event_control  Event Control
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

enum mip_3dm_event_control_command_mode
{
    MIP_3DM_EVENT_CONTROL_COMMAND_MODE_DISABLED   = 0,  ///<  Trigger is disabled.
    MIP_3DM_EVENT_CONTROL_COMMAND_MODE_ENABLED    = 1,  ///<  Trigger is enabled and will work normally.
    MIP_3DM_EVENT_CONTROL_COMMAND_MODE_TEST       = 2,  ///<  Forces the trigger to the active state for testing purposes.
    MIP_3DM_EVENT_CONTROL_COMMAND_MODE_TEST_PULSE = 3,  ///<  Trigger is forced to the active state for one event cycle only. After the test cycle, the mode reverts to the previous state (either enabled or disabled).
};
size_t insert_mip_3dm_event_control_command_mode(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_3dm_event_control_command_mode self);
size_t extract_mip_3dm_event_control_command_mode(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_3dm_event_control_command_mode* self);

struct mip_3dm_event_control_command
{
    enum mip_function_selector                        function;
    uint8_t                                           instance;
    enum mip_3dm_event_control_command_mode           mode;
};
size_t insert_mip_3dm_event_control_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_3dm_event_control_command* self);
size_t extract_mip_3dm_event_control_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_3dm_event_control_command* self);

struct mip_3dm_event_control_response
{
    uint8_t                                           instance;
    enum mip_3dm_event_control_command_mode           mode;
};
size_t insert_mip_3dm_event_control_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_3dm_event_control_response* self);
size_t extract_mip_3dm_event_control_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_3dm_event_control_response* self);

mip_cmd_result mip_3dm_write_event_control(struct mip_interface* device, uint8_t instance, enum mip_3dm_event_control_command_mode mode);
mip_cmd_result mip_3dm_read_event_control(struct mip_interface* device, uint8_t instance, enum mip_3dm_event_control_command_mode* mode);
mip_cmd_result mip_3dm_save_event_control(struct mip_interface* device, uint8_t instance);
mip_cmd_result mip_3dm_load_event_control(struct mip_interface* device, uint8_t instance);
mip_cmd_result mip_3dm_default_event_control(struct mip_interface* device, uint8_t instance);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup get_event_trigger_status  Get Trigger Status
///
///@{

struct mip_3dm_get_event_trigger_status_command
{
    uint8_t                                           requested_count;
    uint8_t                                           requested_instances[20];
};
size_t insert_mip_3dm_get_event_trigger_status_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_3dm_get_event_trigger_status_command* self);
size_t extract_mip_3dm_get_event_trigger_status_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_3dm_get_event_trigger_status_command* self);

enum mip_3dm_get_event_trigger_status_command_status
{
    MIP_3DM_GET_EVENT_TRIGGER_STATUS_COMMAND_STATUS_ACTIVE  = 0x01,
    MIP_3DM_GET_EVENT_TRIGGER_STATUS_COMMAND_STATUS_ENABLED = 0x02,
    MIP_3DM_GET_EVENT_TRIGGER_STATUS_COMMAND_STATUS_TEST    = 0x04,
};
size_t insert_mip_3dm_get_event_trigger_status_command_status(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_3dm_get_event_trigger_status_command_status self);
size_t extract_mip_3dm_get_event_trigger_status_command_status(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_3dm_get_event_trigger_status_command_status* self);

struct mip_3dm_get_event_trigger_status_command_entry
{
    uint8_t                                           type;
    enum mip_3dm_get_event_trigger_status_command_status status;
};
size_t insert_mip_3dm_get_event_trigger_status_command_entry(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_3dm_get_event_trigger_status_command_entry* self);
size_t extract_mip_3dm_get_event_trigger_status_command_entry(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_3dm_get_event_trigger_status_command_entry* self);

struct mip_3dm_get_event_trigger_status_response
{
    uint8_t                                           count;
    struct mip_3dm_get_event_trigger_status_command_entry triggers[20];
};
size_t insert_mip_3dm_get_event_trigger_status_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_3dm_get_event_trigger_status_response* self);
size_t extract_mip_3dm_get_event_trigger_status_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_3dm_get_event_trigger_status_response* self);

mip_cmd_result mip_3dm_get_event_trigger_status(struct mip_interface* device, uint8_t requested_count, const uint8_t* requested_instances, uint8_t* count, struct mip_3dm_get_event_trigger_status_command_entry* triggers);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup get_event_action_status  Get Action Status
///
///@{

struct mip_3dm_get_event_action_status_command
{
    uint8_t                                           requested_count;
    uint8_t                                           requested_instances[20];
};
size_t insert_mip_3dm_get_event_action_status_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_3dm_get_event_action_status_command* self);
size_t extract_mip_3dm_get_event_action_status_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_3dm_get_event_action_status_command* self);

struct mip_3dm_get_event_action_status_command_entry
{
    uint8_t                                           action_type;
    uint8_t                                           trigger_id;
};
size_t insert_mip_3dm_get_event_action_status_command_entry(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_3dm_get_event_action_status_command_entry* self);
size_t extract_mip_3dm_get_event_action_status_command_entry(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_3dm_get_event_action_status_command_entry* self);

struct mip_3dm_get_event_action_status_response
{
    uint8_t                                           count;
    struct mip_3dm_get_event_action_status_command_entry actions[20];
};
size_t insert_mip_3dm_get_event_action_status_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_3dm_get_event_action_status_response* self);
size_t extract_mip_3dm_get_event_action_status_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_3dm_get_event_action_status_response* self);

mip_cmd_result mip_3dm_get_event_action_status(struct mip_interface* device, uint8_t requested_count, const uint8_t* requested_instances, uint8_t* count, struct mip_3dm_get_event_action_status_command_entry* actions);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup event_trigger  Event Trigger Configuration
/// Configures various types of event triggers.
///
///@{

enum mip_3dm_event_trigger_command_type
{
    MIP_3DM_EVENT_TRIGGER_COMMAND_TYPE_NONE        = 0,  ///<  No trigger selected. The state will always be inactive.
    MIP_3DM_EVENT_TRIGGER_COMMAND_TYPE_GPIO        = 1,  ///<  Trigger based on the state of a GPIO pin. See GpioParams.
    MIP_3DM_EVENT_TRIGGER_COMMAND_TYPE_THRESHOLD   = 2,  ///<  Compare a data quantity against a high and low threshold. See ThresholdParams.
    MIP_3DM_EVENT_TRIGGER_COMMAND_TYPE_COMBINATION = 3,  ///<  Logical combination of two or more triggers. See CombinationParams.
};
size_t insert_mip_3dm_event_trigger_command_type(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_3dm_event_trigger_command_type self);
size_t extract_mip_3dm_event_trigger_command_type(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_3dm_event_trigger_command_type* self);

enum mip_3dm_event_trigger_command_gpio_params_mode
{
    MIP_3DM_EVENT_TRIGGER_COMMAND_GPIO_PARAMS_MODE_DISABLED   = 0,  ///<  The pin will have no effect and the trigger will never activate.
    MIP_3DM_EVENT_TRIGGER_COMMAND_GPIO_PARAMS_MODE_WHILE_HIGH = 1,  ///<  The trigger will be active while the pin is high.
    MIP_3DM_EVENT_TRIGGER_COMMAND_GPIO_PARAMS_MODE_WHILE_LOW  = 2,  ///<  The trigger will be active while the pin is low.
    MIP_3DM_EVENT_TRIGGER_COMMAND_GPIO_PARAMS_MODE_EDGE       = 4,  ///<  Use if the pin is configured for timestamping via the 3DM Gpio Configuration command (0x0C41).
};
size_t insert_mip_3dm_event_trigger_command_gpio_params_mode(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_3dm_event_trigger_command_gpio_params_mode self);
size_t extract_mip_3dm_event_trigger_command_gpio_params_mode(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_3dm_event_trigger_command_gpio_params_mode* self);

struct mip_3dm_event_trigger_command_gpio_params
{
    uint8_t                                           pin;
    enum mip_3dm_event_trigger_command_gpio_params_mode mode;
};
size_t insert_mip_3dm_event_trigger_command_gpio_params(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_3dm_event_trigger_command_gpio_params* self);
size_t extract_mip_3dm_event_trigger_command_gpio_params(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_3dm_event_trigger_command_gpio_params* self);

enum mip_3dm_event_trigger_command_threshold_params_type
{
    MIP_3DM_EVENT_TRIGGER_COMMAND_THRESHOLD_PARAMS_TYPE_WINDOW   = 1,  ///<  Window comparison. Trigger is active if low_thres &lt;= value &lt;= high_thres. If the thresholds are reversed, the trigger is active when value &lt; high_thres or value &gt; low_thres.
    MIP_3DM_EVENT_TRIGGER_COMMAND_THRESHOLD_PARAMS_TYPE_INTERVAL = 2,  ///<  Trigger at evenly-spaced intervals. Normally used with time fields to trigger periodically. Trigger is active when (value % interval) &lt;= int_thres. If the thresholds are reversed (high_thres &lt; low_thres) then the trigger is active when (value % low_thres) &gt; high_thres.
};
size_t insert_mip_3dm_event_trigger_command_threshold_params_type(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_3dm_event_trigger_command_threshold_params_type self);
size_t extract_mip_3dm_event_trigger_command_threshold_params_type(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_3dm_event_trigger_command_threshold_params_type* self);

struct mip_3dm_event_trigger_command_threshold_params
{
    uint8_t                                           desc_set;
    uint8_t                                           field_desc;
    uint8_t                                           param_id;
    enum mip_3dm_event_trigger_command_threshold_params_type type;
    union
    {
        double                                            low_thres;
        double                                            int_thres;
    };
    union
    {
        double                                            high_thres;
        double                                            interval;
    };
};
size_t insert_mip_3dm_event_trigger_command_threshold_params(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_3dm_event_trigger_command_threshold_params* self);
size_t extract_mip_3dm_event_trigger_command_threshold_params(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_3dm_event_trigger_command_threshold_params* self);

struct mip_3dm_event_trigger_command_combination_params
{
    uint16_t                                          logic_table;
    uint8_t                                           input_triggers[4];
};
size_t insert_mip_3dm_event_trigger_command_combination_params(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_3dm_event_trigger_command_combination_params* self);
size_t extract_mip_3dm_event_trigger_command_combination_params(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_3dm_event_trigger_command_combination_params* self);

struct mip_3dm_event_trigger_command
{
    enum mip_function_selector                        function;
    uint8_t                                           instance;
    enum mip_3dm_event_trigger_command_type           type;
    union
    {
        struct mip_3dm_event_trigger_command_gpio_params  gpio;
        struct mip_3dm_event_trigger_command_threshold_params threshold;
        struct mip_3dm_event_trigger_command_combination_params combination;
    };
};
size_t insert_mip_3dm_event_trigger_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_3dm_event_trigger_command* self);
size_t extract_mip_3dm_event_trigger_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_3dm_event_trigger_command* self);

struct mip_3dm_event_trigger_response
{
    uint8_t                                           instance;
    enum mip_3dm_event_trigger_command_type           type;
    union
    {
        struct mip_3dm_event_trigger_command_gpio_params  gpio;
        struct mip_3dm_event_trigger_command_threshold_params threshold;
        struct mip_3dm_event_trigger_command_combination_params combination;
    };
};
size_t insert_mip_3dm_event_trigger_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_3dm_event_trigger_response* self);
size_t extract_mip_3dm_event_trigger_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_3dm_event_trigger_response* self);

mip_cmd_result mip_3dm_write_event_trigger(struct mip_interface* device, uint8_t instance, enum mip_3dm_event_trigger_command_type type, const void* gpio_threshold_combination);
mip_cmd_result mip_3dm_read_event_trigger(struct mip_interface* device, uint8_t instance, enum mip_3dm_event_trigger_command_type* type, void* gpio_threshold_combination);
mip_cmd_result mip_3dm_save_event_trigger(struct mip_interface* device, uint8_t instance);
mip_cmd_result mip_3dm_load_event_trigger(struct mip_interface* device, uint8_t instance);
mip_cmd_result mip_3dm_default_event_trigger(struct mip_interface* device, uint8_t instance);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup event_action  Event Action Configuration
/// Configures various types of event actions.
///
///@{

enum mip_3dm_event_action_command_type
{
    MIP_3DM_EVENT_ACTION_COMMAND_TYPE_NONE    = 0,  ///<  No action. Parameters should be empty.
    MIP_3DM_EVENT_ACTION_COMMAND_TYPE_GPIO    = 1,  ///<  Control the state of a GPIO pin. See GpioParameters.
    MIP_3DM_EVENT_ACTION_COMMAND_TYPE_MESSAGE = 2,  ///<  Output a data packet. See MessageParameters.
};
size_t insert_mip_3dm_event_action_command_type(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_3dm_event_action_command_type self);
size_t extract_mip_3dm_event_action_command_type(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_3dm_event_action_command_type* self);

enum mip_3dm_event_action_command_gpio_params_mode
{
    MIP_3DM_EVENT_ACTION_COMMAND_GPIO_PARAMS_MODE_DISABLED     = 0,  ///<  Pin state will not be changed.
    MIP_3DM_EVENT_ACTION_COMMAND_GPIO_PARAMS_MODE_ACTIVE_HIGH  = 1,  ///<  Pin will be set high when the trigger is active and low otherwise.
    MIP_3DM_EVENT_ACTION_COMMAND_GPIO_PARAMS_MODE_ACTIVE_LOW   = 2,  ///<  Pin will be set low when the trigger is active and high otherwise.
    MIP_3DM_EVENT_ACTION_COMMAND_GPIO_PARAMS_MODE_ONESHOT_HIGH = 5,  ///<  Pin will be set high each time the trigger activates. It will not be set low.
    MIP_3DM_EVENT_ACTION_COMMAND_GPIO_PARAMS_MODE_ONESHOT_LOW  = 6,  ///<  Pin will be set low each time the trigger activates. It will not be set high.
    MIP_3DM_EVENT_ACTION_COMMAND_GPIO_PARAMS_MODE_TOGGLE       = 7,  ///<  Pin will change to the opposite state each time the trigger activates.
};
size_t insert_mip_3dm_event_action_command_gpio_params_mode(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_3dm_event_action_command_gpio_params_mode self);
size_t extract_mip_3dm_event_action_command_gpio_params_mode(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_3dm_event_action_command_gpio_params_mode* self);

struct mip_3dm_event_action_command_gpio_params
{
    uint8_t                                           pin;
    enum mip_3dm_event_action_command_gpio_params_mode mode;
};
size_t insert_mip_3dm_event_action_command_gpio_params(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_3dm_event_action_command_gpio_params* self);
size_t extract_mip_3dm_event_action_command_gpio_params(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_3dm_event_action_command_gpio_params* self);

struct mip_3dm_event_action_command_message_params
{
    uint8_t                                           desc_set;
    uint16_t                                          decimation;
    uint8_t                                           num_fields;
    uint8_t                                           descriptors[20];
};
size_t insert_mip_3dm_event_action_command_message_params(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_3dm_event_action_command_message_params* self);
size_t extract_mip_3dm_event_action_command_message_params(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_3dm_event_action_command_message_params* self);

struct mip_3dm_event_action_command
{
    enum mip_function_selector                        function;
    uint8_t                                           instance;
    uint8_t                                           trigger;
    enum mip_3dm_event_action_command_type            type;
    union
    {
        struct mip_3dm_event_action_command_gpio_params   gpio;
        struct mip_3dm_event_action_command_message_params message;
    };
};
size_t insert_mip_3dm_event_action_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_3dm_event_action_command* self);
size_t extract_mip_3dm_event_action_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_3dm_event_action_command* self);

struct mip_3dm_event_action_response
{
    uint8_t                                           instance;
    uint8_t                                           trigger;
    enum mip_3dm_event_action_command_type            type;
    union
    {
        struct mip_3dm_event_action_command_gpio_params   gpio;
        struct mip_3dm_event_action_command_message_params message;
    };
};
size_t insert_mip_3dm_event_action_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_3dm_event_action_response* self);
size_t extract_mip_3dm_event_action_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_3dm_event_action_response* self);

mip_cmd_result mip_3dm_write_event_action(struct mip_interface* device, uint8_t instance, uint8_t trigger, enum mip_3dm_event_action_command_type type, const void* gpio_message);
mip_cmd_result mip_3dm_read_event_action(struct mip_interface* device, uint8_t instance, uint8_t* trigger, enum mip_3dm_event_action_command_type* type, void* gpio_message);
mip_cmd_result mip_3dm_save_event_action(struct mip_interface* device, uint8_t instance);
mip_cmd_result mip_3dm_load_event_action(struct mip_interface* device, uint8_t instance);
mip_cmd_result mip_3dm_default_event_action(struct mip_interface* device, uint8_t instance);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup accel_bias  Configure Accel Bias
/// Configures the user specified accelerometer bias
/// 
/// The user specified bias is subtracted from the calibrated accelerometer output.  Value is input in the sensor frame.
///
///@{

struct mip_3dm_accel_bias_command
{
    enum mip_function_selector                        function;
    float                                             bias[3];
};
size_t insert_mip_3dm_accel_bias_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_3dm_accel_bias_command* self);
size_t extract_mip_3dm_accel_bias_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_3dm_accel_bias_command* self);

struct mip_3dm_accel_bias_response
{
    float                                             bias[3];
};
size_t insert_mip_3dm_accel_bias_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_3dm_accel_bias_response* self);
size_t extract_mip_3dm_accel_bias_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_3dm_accel_bias_response* self);

mip_cmd_result mip_3dm_write_accel_bias(struct mip_interface* device, const float* bias);
mip_cmd_result mip_3dm_read_accel_bias(struct mip_interface* device, float* bias);
mip_cmd_result mip_3dm_save_accel_bias(struct mip_interface* device);
mip_cmd_result mip_3dm_load_accel_bias(struct mip_interface* device);
mip_cmd_result mip_3dm_default_accel_bias(struct mip_interface* device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup gyro_bias  Configure Gyro Bias
/// Configures the user specified gyroscope bias
/// 
/// The user specified bias is subtracted from the calibrated angular rate output.  Value is input in the sensor frame.
///
///@{

struct mip_3dm_gyro_bias_command
{
    enum mip_function_selector                        function;
    float                                             bias[3];
};
size_t insert_mip_3dm_gyro_bias_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_3dm_gyro_bias_command* self);
size_t extract_mip_3dm_gyro_bias_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_3dm_gyro_bias_command* self);

struct mip_3dm_gyro_bias_response
{
    float                                             bias[3];
};
size_t insert_mip_3dm_gyro_bias_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_3dm_gyro_bias_response* self);
size_t extract_mip_3dm_gyro_bias_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_3dm_gyro_bias_response* self);

mip_cmd_result mip_3dm_write_gyro_bias(struct mip_interface* device, const float* bias);
mip_cmd_result mip_3dm_read_gyro_bias(struct mip_interface* device, float* bias);
mip_cmd_result mip_3dm_save_gyro_bias(struct mip_interface* device);
mip_cmd_result mip_3dm_load_gyro_bias(struct mip_interface* device);
mip_cmd_result mip_3dm_default_gyro_bias(struct mip_interface* device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup capture_gyro_bias  Capture Gyro Bias
/// Samples gyro for a specified time range and writes the averaged result to the Gyro Bias vector in RAM
/// 
/// The device will average the gyro output for the duration of "averaging_time_ms." To store the resulting vector
/// in non-volatile memory, use the Set Gyro Bias command.
/// IMPORTANT: The device must be stationary and experiencing minimum vibration for the duration of "averaging_time_ms"
/// Averaging Time range: 1000 to 30,000
///
///@{

struct mip_3dm_capture_gyro_bias_command
{
    uint16_t                                          averaging_time_ms;
};
size_t insert_mip_3dm_capture_gyro_bias_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_3dm_capture_gyro_bias_command* self);
size_t extract_mip_3dm_capture_gyro_bias_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_3dm_capture_gyro_bias_command* self);

struct mip_3dm_capture_gyro_bias_response
{
    float                                             bias[3];
};
size_t insert_mip_3dm_capture_gyro_bias_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_3dm_capture_gyro_bias_response* self);
size_t extract_mip_3dm_capture_gyro_bias_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_3dm_capture_gyro_bias_response* self);

mip_cmd_result mip_3dm_capture_gyro_bias(struct mip_interface* device, uint16_t averaging_time_ms, float* bias);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mag_hard_iron_offset  Magnetometer Hard Iron Offset
/// Configure the user specified magnetometer hard iron offset vector
/// 
/// The values for this offset are determined empirically by external software algorithms
/// based on calibration data taken after the device is installed in its application. These values
/// can be obtained and set by using the LORD "MIP Iron Calibration" application.
/// Alternatively, on some systems, the auto-mag calibration feature may be used to capture these values in-run.
/// The offset is applied to the scaled magnetometer vector prior to output.
///
///@{

struct mip_3dm_mag_hard_iron_offset_command
{
    enum mip_function_selector                        function;
    float                                             offset[3];
};
size_t insert_mip_3dm_mag_hard_iron_offset_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_3dm_mag_hard_iron_offset_command* self);
size_t extract_mip_3dm_mag_hard_iron_offset_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_3dm_mag_hard_iron_offset_command* self);

struct mip_3dm_mag_hard_iron_offset_response
{
    float                                             offset[3];
};
size_t insert_mip_3dm_mag_hard_iron_offset_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_3dm_mag_hard_iron_offset_response* self);
size_t extract_mip_3dm_mag_hard_iron_offset_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_3dm_mag_hard_iron_offset_response* self);

mip_cmd_result mip_3dm_write_mag_hard_iron_offset(struct mip_interface* device, const float* offset);
mip_cmd_result mip_3dm_read_mag_hard_iron_offset(struct mip_interface* device, float* offset);
mip_cmd_result mip_3dm_save_mag_hard_iron_offset(struct mip_interface* device);
mip_cmd_result mip_3dm_load_mag_hard_iron_offset(struct mip_interface* device);
mip_cmd_result mip_3dm_default_mag_hard_iron_offset(struct mip_interface* device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mag_soft_iron_matrix  Magnetometer Soft Iron Matrix
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

struct mip_3dm_mag_soft_iron_matrix_command
{
    enum mip_function_selector                        function;
    float                                             offset[9];
};
size_t insert_mip_3dm_mag_soft_iron_matrix_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_3dm_mag_soft_iron_matrix_command* self);
size_t extract_mip_3dm_mag_soft_iron_matrix_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_3dm_mag_soft_iron_matrix_command* self);

struct mip_3dm_mag_soft_iron_matrix_response
{
    float                                             offset[9];
};
size_t insert_mip_3dm_mag_soft_iron_matrix_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_3dm_mag_soft_iron_matrix_response* self);
size_t extract_mip_3dm_mag_soft_iron_matrix_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_3dm_mag_soft_iron_matrix_response* self);

mip_cmd_result mip_3dm_write_mag_soft_iron_matrix(struct mip_interface* device, const float* offset);
mip_cmd_result mip_3dm_read_mag_soft_iron_matrix(struct mip_interface* device, float* offset);
mip_cmd_result mip_3dm_save_mag_soft_iron_matrix(struct mip_interface* device);
mip_cmd_result mip_3dm_load_mag_soft_iron_matrix(struct mip_interface* device);
mip_cmd_result mip_3dm_default_mag_soft_iron_matrix(struct mip_interface* device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup sensor_2_vehicle_transform_euler  Sensor to Vehicle Frame Transformation Euler
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

struct mip_3dm_sensor_2_vehicle_transform_euler_command
{
    enum mip_function_selector                        function;
    float                                             roll;
    float                                             pitch;
    float                                             yaw;
};
size_t insert_mip_3dm_sensor_2_vehicle_transform_euler_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_3dm_sensor_2_vehicle_transform_euler_command* self);
size_t extract_mip_3dm_sensor_2_vehicle_transform_euler_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_3dm_sensor_2_vehicle_transform_euler_command* self);

struct mip_3dm_sensor_2_vehicle_transform_euler_response
{
    float                                             roll;
    float                                             pitch;
    float                                             yaw;
};
size_t insert_mip_3dm_sensor_2_vehicle_transform_euler_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_3dm_sensor_2_vehicle_transform_euler_response* self);
size_t extract_mip_3dm_sensor_2_vehicle_transform_euler_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_3dm_sensor_2_vehicle_transform_euler_response* self);

mip_cmd_result mip_3dm_write_sensor_2_vehicle_transform_euler(struct mip_interface* device, float roll, float pitch, float yaw);
mip_cmd_result mip_3dm_read_sensor_2_vehicle_transform_euler(struct mip_interface* device, float* roll, float* pitch, float* yaw);
mip_cmd_result mip_3dm_save_sensor_2_vehicle_transform_euler(struct mip_interface* device);
mip_cmd_result mip_3dm_load_sensor_2_vehicle_transform_euler(struct mip_interface* device);
mip_cmd_result mip_3dm_default_sensor_2_vehicle_transform_euler(struct mip_interface* device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup sensor_2_vehicle_transform_quaternion  Sensor to Vehicle Frame Transformation Quaternion
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

struct mip_3dm_sensor_2_vehicle_transform_quaternion_command
{
    enum mip_function_selector                        function;
    float                                             q[4];
};
size_t insert_mip_3dm_sensor_2_vehicle_transform_quaternion_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_3dm_sensor_2_vehicle_transform_quaternion_command* self);
size_t extract_mip_3dm_sensor_2_vehicle_transform_quaternion_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_3dm_sensor_2_vehicle_transform_quaternion_command* self);

struct mip_3dm_sensor_2_vehicle_transform_quaternion_response
{
    float                                             q[4];
};
size_t insert_mip_3dm_sensor_2_vehicle_transform_quaternion_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_3dm_sensor_2_vehicle_transform_quaternion_response* self);
size_t extract_mip_3dm_sensor_2_vehicle_transform_quaternion_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_3dm_sensor_2_vehicle_transform_quaternion_response* self);

mip_cmd_result mip_3dm_write_sensor_2_vehicle_transform_quaternion(struct mip_interface* device, const float* q);
mip_cmd_result mip_3dm_read_sensor_2_vehicle_transform_quaternion(struct mip_interface* device, float* q);
mip_cmd_result mip_3dm_save_sensor_2_vehicle_transform_quaternion(struct mip_interface* device);
mip_cmd_result mip_3dm_load_sensor_2_vehicle_transform_quaternion(struct mip_interface* device);
mip_cmd_result mip_3dm_default_sensor_2_vehicle_transform_quaternion(struct mip_interface* device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup sensor_2_vehicle_transform_dcm  Sensor to Vehicle Frame Transformation Direction Cosine Matrix
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

struct mip_3dm_sensor_2_vehicle_transform_dcm_command
{
    enum mip_function_selector                        function;
    float                                             dcm[9];
};
size_t insert_mip_3dm_sensor_2_vehicle_transform_dcm_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_3dm_sensor_2_vehicle_transform_dcm_command* self);
size_t extract_mip_3dm_sensor_2_vehicle_transform_dcm_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_3dm_sensor_2_vehicle_transform_dcm_command* self);

struct mip_3dm_sensor_2_vehicle_transform_dcm_response
{
    float                                             dcm[9];
};
size_t insert_mip_3dm_sensor_2_vehicle_transform_dcm_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_3dm_sensor_2_vehicle_transform_dcm_response* self);
size_t extract_mip_3dm_sensor_2_vehicle_transform_dcm_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_3dm_sensor_2_vehicle_transform_dcm_response* self);

mip_cmd_result mip_3dm_write_sensor_2_vehicle_transform_dcm(struct mip_interface* device, const float* dcm);
mip_cmd_result mip_3dm_read_sensor_2_vehicle_transform_dcm(struct mip_interface* device, float* dcm);
mip_cmd_result mip_3dm_save_sensor_2_vehicle_transform_dcm(struct mip_interface* device);
mip_cmd_result mip_3dm_load_sensor_2_vehicle_transform_dcm(struct mip_interface* device);
mip_cmd_result mip_3dm_default_sensor_2_vehicle_transform_dcm(struct mip_interface* device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup complementary_filter  Complementary filter settings
/// Configure the settings for the complementary filter which produces the following (0x80) descriptor set values: attitude matrix (0x80,09), quaternion (0x80,0A), and  Euler angle (0x80,0C) outputs.
/// 
/// The filter can be configured to correct for pitch and roll using the accelerometer (with the assumption that linear acceleration is minimal),
/// and to correct for heading using the magnetomer (with the assumption that the local magnetic field is dominated by the Earth's own magnetic field).
/// Pitch/roll and heading corrections each have their own configurable time constants, with a valid range of 1-1000 seconds. The default time constant is 10 seconds.
///
///@{

struct mip_3dm_complementary_filter_command
{
    enum mip_function_selector                        function;
    bool                                              pitch_roll_enable;
    bool                                              heading_enable;
    float                                             pitch_roll_time_constant;
    float                                             heading_time_constant;
};
size_t insert_mip_3dm_complementary_filter_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_3dm_complementary_filter_command* self);
size_t extract_mip_3dm_complementary_filter_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_3dm_complementary_filter_command* self);

struct mip_3dm_complementary_filter_response
{
    bool                                              pitch_roll_enable;
    bool                                              heading_enable;
    float                                             pitch_roll_time_constant;
    float                                             heading_time_constant;
};
size_t insert_mip_3dm_complementary_filter_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_3dm_complementary_filter_response* self);
size_t extract_mip_3dm_complementary_filter_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_3dm_complementary_filter_response* self);

mip_cmd_result mip_3dm_write_complementary_filter(struct mip_interface* device, bool pitch_roll_enable, bool heading_enable, float pitch_roll_time_constant, float heading_time_constant);
mip_cmd_result mip_3dm_read_complementary_filter(struct mip_interface* device, bool* pitch_roll_enable, bool* heading_enable, float* pitch_roll_time_constant, float* heading_time_constant);
mip_cmd_result mip_3dm_save_complementary_filter(struct mip_interface* device);
mip_cmd_result mip_3dm_load_complementary_filter(struct mip_interface* device);
mip_cmd_result mip_3dm_default_complementary_filter(struct mip_interface* device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup sensor_range  Sensor Range
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

struct mip_3dm_sensor_range_command
{
    enum mip_function_selector                        function;
    enum mip_sensor_range_type                        sensor;
    uint8_t                                           setting;
};
size_t insert_mip_3dm_sensor_range_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_3dm_sensor_range_command* self);
size_t extract_mip_3dm_sensor_range_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_3dm_sensor_range_command* self);

struct mip_3dm_sensor_range_response
{
    enum mip_sensor_range_type                        sensor;
    uint8_t                                           setting;
};
size_t insert_mip_3dm_sensor_range_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_3dm_sensor_range_response* self);
size_t extract_mip_3dm_sensor_range_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_3dm_sensor_range_response* self);

mip_cmd_result mip_3dm_write_sensor_range(struct mip_interface* device, enum mip_sensor_range_type sensor, uint8_t setting);
mip_cmd_result mip_3dm_read_sensor_range(struct mip_interface* device, enum mip_sensor_range_type sensor, uint8_t* setting);
mip_cmd_result mip_3dm_save_sensor_range(struct mip_interface* device, enum mip_sensor_range_type sensor);
mip_cmd_result mip_3dm_load_sensor_range(struct mip_interface* device, enum mip_sensor_range_type sensor);
mip_cmd_result mip_3dm_default_sensor_range(struct mip_interface* device, enum mip_sensor_range_type sensor);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup calibrated_sensor_ranges  Get Calibrated Sensor Ranges
/// Returns the supported sensor ranges which may be used with the 3DM Sensor Range (0x0C,0x52) command.
/// 
/// The response includes an array of (u8, float) pairs which map each allowed setting
/// to the corresponding maximum range in physical units. See SensorRangeType for units.
///
///@{

struct mip_3dm_calibrated_sensor_ranges_command
{
    enum mip_sensor_range_type                        sensor;
};
size_t insert_mip_3dm_calibrated_sensor_ranges_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_3dm_calibrated_sensor_ranges_command* self);
size_t extract_mip_3dm_calibrated_sensor_ranges_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_3dm_calibrated_sensor_ranges_command* self);

struct mip_3dm_calibrated_sensor_ranges_command_entry
{
    uint8_t                                           setting;
    float                                             range;
};
size_t insert_mip_3dm_calibrated_sensor_ranges_command_entry(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_3dm_calibrated_sensor_ranges_command_entry* self);
size_t extract_mip_3dm_calibrated_sensor_ranges_command_entry(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_3dm_calibrated_sensor_ranges_command_entry* self);

struct mip_3dm_calibrated_sensor_ranges_response
{
    enum mip_sensor_range_type                        sensor;
    uint8_t                                           num_ranges;
    struct mip_3dm_calibrated_sensor_ranges_command_entry ranges[50];
};
size_t insert_mip_3dm_calibrated_sensor_ranges_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_3dm_calibrated_sensor_ranges_response* self);
size_t extract_mip_3dm_calibrated_sensor_ranges_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_3dm_calibrated_sensor_ranges_response* self);

mip_cmd_result mip_3dm_calibrated_sensor_ranges(struct mip_interface* device, enum mip_sensor_range_type sensor, uint8_t* num_ranges, struct mip_3dm_calibrated_sensor_ranges_command_entry* ranges);
///@}
///

///@}
///@}
///
////////////////////////////////////////////////////////////////////////////////

#ifdef __cplusplus
} // extern "C"
} // namespace C
namespace TdmCommands {

static const uint8_t DESCRIPTOR_SET = MIP_3DM_CMD_DESC_SET;


struct PollImuMessage : C::mip_3dm_poll_imu_message_command
{
    static const uint8_t descriptorSet = MIP_3DM_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_3DM_POLL_IMU_MESSAGE;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0) const
    {
        return C::insert_mip_3dm_poll_imu_message_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_3dm_poll_imu_message_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = false;
    
};
MipCmdResult pollImuMessage(C::mip_interface& device, bool suppress_ack, uint8_t num_descriptors, const C::mip_descriptor_rate* descriptors)
{
    return C::mip_3dm_poll_imu_message(&device, suppress_ack, num_descriptors, descriptors);
}



struct PollGnssMessage : C::mip_3dm_poll_gnss_message_command
{
    static const uint8_t descriptorSet = MIP_3DM_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_3DM_POLL_GNSS_MESSAGE;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0) const
    {
        return C::insert_mip_3dm_poll_gnss_message_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_3dm_poll_gnss_message_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = false;
    
};
MipCmdResult pollGnssMessage(C::mip_interface& device, bool suppress_ack, uint8_t num_descriptors, const C::mip_descriptor_rate* descriptors)
{
    return C::mip_3dm_poll_gnss_message(&device, suppress_ack, num_descriptors, descriptors);
}



struct PollFilterMessage : C::mip_3dm_poll_filter_message_command
{
    static const uint8_t descriptorSet = MIP_3DM_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_3DM_POLL_FILTER_MESSAGE;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0) const
    {
        return C::insert_mip_3dm_poll_filter_message_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_3dm_poll_filter_message_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = false;
    
};
MipCmdResult pollFilterMessage(C::mip_interface& device, bool suppress_ack, uint8_t num_descriptors, const C::mip_descriptor_rate* descriptors)
{
    return C::mip_3dm_poll_filter_message(&device, suppress_ack, num_descriptors, descriptors);
}



struct ImuMessageFormat : C::mip_3dm_imu_message_format_command
{
    static const uint8_t descriptorSet = MIP_3DM_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_3DM_IMU_MESSAGE_FORMAT;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0) const
    {
        return C::insert_mip_3dm_imu_message_format_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_3dm_imu_message_format_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = true;
    static const bool canWrite = true;
    static const bool canRead = true;
    static const bool canSave = true;
    static const bool canLoad = true;
    static const bool canReset = true;
    
    struct Response : C::mip_3dm_imu_message_format_response
    {
        static const uint8_t descriptorSet = MIP_3DM_CMD_DESC_SET;
        static const uint8_t fieldDescriptor = MIP_REPLY_DESC_3DM_IMU_MESSAGE_FORMAT;
        
        size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::insert_mip_3dm_imu_message_format_response(buffer, bufferSize, offset, this);
        }
        size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::extract_mip_3dm_imu_message_format_response(buffer, bufferSize, offset, this);
        }
    };
    
};
MipCmdResult writeImuMessageFormat(C::mip_interface& device, uint8_t num_descriptors, const C::mip_descriptor_rate* descriptors)
{
    return C::mip_3dm_write_imu_message_format(&device, num_descriptors, descriptors);
}
MipCmdResult readImuMessageFormat(C::mip_interface& device, uint8_t& num_descriptors, struct C::mip_descriptor_rate* descriptors)
{
    return C::mip_3dm_read_imu_message_format(&device, &num_descriptors, descriptors);
}
MipCmdResult saveImuMessageFormat(C::mip_interface& device)
{
    return C::mip_3dm_save_imu_message_format(&device);
}
MipCmdResult loadImuMessageFormat(C::mip_interface& device)
{
    return C::mip_3dm_load_imu_message_format(&device);
}
MipCmdResult defaultImuMessageFormat(C::mip_interface& device)
{
    return C::mip_3dm_default_imu_message_format(&device);
}



struct GpsMessageFormat : C::mip_3dm_gps_message_format_command
{
    static const uint8_t descriptorSet = MIP_3DM_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_3DM_GNSS_MESSAGE_FORMAT;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0) const
    {
        return C::insert_mip_3dm_gps_message_format_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_3dm_gps_message_format_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = true;
    static const bool canWrite = true;
    static const bool canRead = true;
    static const bool canSave = true;
    static const bool canLoad = true;
    static const bool canReset = true;
    
    struct Response : C::mip_3dm_gps_message_format_response
    {
        static const uint8_t descriptorSet = MIP_3DM_CMD_DESC_SET;
        static const uint8_t fieldDescriptor = MIP_REPLY_DESC_3DM_GNSS_MESSAGE_FORMAT;
        
        size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::insert_mip_3dm_gps_message_format_response(buffer, bufferSize, offset, this);
        }
        size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::extract_mip_3dm_gps_message_format_response(buffer, bufferSize, offset, this);
        }
    };
    
};
MipCmdResult writeGpsMessageFormat(C::mip_interface& device, uint8_t num_descriptors, const C::mip_descriptor_rate* descriptors)
{
    return C::mip_3dm_write_gps_message_format(&device, num_descriptors, descriptors);
}
MipCmdResult readGpsMessageFormat(C::mip_interface& device, uint8_t& num_descriptors, struct C::mip_descriptor_rate* descriptors)
{
    return C::mip_3dm_read_gps_message_format(&device, &num_descriptors, descriptors);
}
MipCmdResult saveGpsMessageFormat(C::mip_interface& device)
{
    return C::mip_3dm_save_gps_message_format(&device);
}
MipCmdResult loadGpsMessageFormat(C::mip_interface& device)
{
    return C::mip_3dm_load_gps_message_format(&device);
}
MipCmdResult defaultGpsMessageFormat(C::mip_interface& device)
{
    return C::mip_3dm_default_gps_message_format(&device);
}



struct FilterMessageFormat : C::mip_3dm_filter_message_format_command
{
    static const uint8_t descriptorSet = MIP_3DM_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_3DM_FILTER_MESSAGE_FORMAT;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0) const
    {
        return C::insert_mip_3dm_filter_message_format_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_3dm_filter_message_format_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = true;
    static const bool canWrite = true;
    static const bool canRead = true;
    static const bool canSave = true;
    static const bool canLoad = true;
    static const bool canReset = true;
    
    struct Response : C::mip_3dm_filter_message_format_response
    {
        static const uint8_t descriptorSet = MIP_3DM_CMD_DESC_SET;
        static const uint8_t fieldDescriptor = MIP_REPLY_DESC_3DM_FILTER_MESSAGE_FORMAT;
        
        size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::insert_mip_3dm_filter_message_format_response(buffer, bufferSize, offset, this);
        }
        size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::extract_mip_3dm_filter_message_format_response(buffer, bufferSize, offset, this);
        }
    };
    
};
MipCmdResult writeFilterMessageFormat(C::mip_interface& device, uint8_t num_descriptors, const C::mip_descriptor_rate* descriptors)
{
    return C::mip_3dm_write_filter_message_format(&device, num_descriptors, descriptors);
}
MipCmdResult readFilterMessageFormat(C::mip_interface& device, uint8_t& num_descriptors, struct C::mip_descriptor_rate* descriptors)
{
    return C::mip_3dm_read_filter_message_format(&device, &num_descriptors, descriptors);
}
MipCmdResult saveFilterMessageFormat(C::mip_interface& device)
{
    return C::mip_3dm_save_filter_message_format(&device);
}
MipCmdResult loadFilterMessageFormat(C::mip_interface& device)
{
    return C::mip_3dm_load_filter_message_format(&device);
}
MipCmdResult defaultFilterMessageFormat(C::mip_interface& device)
{
    return C::mip_3dm_default_filter_message_format(&device);
}



struct ImuGetBaseRate : C::mip_3dm_imu_get_base_rate_command
{
    static const uint8_t descriptorSet = MIP_3DM_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_3DM_GET_IMU_BASE_RATE;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0) const
    {
        return C::insert_mip_3dm_imu_get_base_rate_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_3dm_imu_get_base_rate_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = false;
    
    struct Response : C::mip_3dm_imu_get_base_rate_response
    {
        static const uint8_t descriptorSet = MIP_3DM_CMD_DESC_SET;
        static const uint8_t fieldDescriptor = MIP_REPLY_DESC_3DM_IMU_BASE_RATE;
        
        size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::insert_mip_3dm_imu_get_base_rate_response(buffer, bufferSize, offset, this);
        }
        size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::extract_mip_3dm_imu_get_base_rate_response(buffer, bufferSize, offset, this);
        }
    };
    
};
MipCmdResult imuGetBaseRate(C::mip_interface& device, uint16_t& rate)
{
    return C::mip_3dm_imu_get_base_rate(&device, &rate);
}



struct GpsGetBaseRate : C::mip_3dm_gps_get_base_rate_command
{
    static const uint8_t descriptorSet = MIP_3DM_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_3DM_GET_GNSS_BASE_RATE;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0) const
    {
        return C::insert_mip_3dm_gps_get_base_rate_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_3dm_gps_get_base_rate_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = false;
    
    struct Response : C::mip_3dm_gps_get_base_rate_response
    {
        static const uint8_t descriptorSet = MIP_3DM_CMD_DESC_SET;
        static const uint8_t fieldDescriptor = MIP_REPLY_DESC_3DM_GNSS_BASE_RATE;
        
        size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::insert_mip_3dm_gps_get_base_rate_response(buffer, bufferSize, offset, this);
        }
        size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::extract_mip_3dm_gps_get_base_rate_response(buffer, bufferSize, offset, this);
        }
    };
    
};
MipCmdResult gpsGetBaseRate(C::mip_interface& device, uint16_t& rate)
{
    return C::mip_3dm_gps_get_base_rate(&device, &rate);
}



struct FilterGetBaseRate : C::mip_3dm_filter_get_base_rate_command
{
    static const uint8_t descriptorSet = MIP_3DM_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_3DM_GET_FILTER_BASE_RATE;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0) const
    {
        return C::insert_mip_3dm_filter_get_base_rate_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_3dm_filter_get_base_rate_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = false;
    
    struct Response : C::mip_3dm_filter_get_base_rate_response
    {
        static const uint8_t descriptorSet = MIP_3DM_CMD_DESC_SET;
        static const uint8_t fieldDescriptor = MIP_REPLY_DESC_3DM_FILTER_BASE_RATE;
        
        size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::insert_mip_3dm_filter_get_base_rate_response(buffer, bufferSize, offset, this);
        }
        size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::extract_mip_3dm_filter_get_base_rate_response(buffer, bufferSize, offset, this);
        }
    };
    
};
MipCmdResult filterGetBaseRate(C::mip_interface& device, uint16_t& rate)
{
    return C::mip_3dm_filter_get_base_rate(&device, &rate);
}



struct PollData : C::mip_3dm_poll_data_command
{
    static const uint8_t descriptorSet = MIP_3DM_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_3DM_POLL_DATA;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0) const
    {
        return C::insert_mip_3dm_poll_data_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_3dm_poll_data_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = false;
    
};
MipCmdResult pollData(C::mip_interface& device, uint8_t desc_set, bool suppress_ack, uint8_t num_descriptors, const uint8_t* descriptors)
{
    return C::mip_3dm_poll_data(&device, desc_set, suppress_ack, num_descriptors, descriptors);
}



struct GetBaseRate : C::mip_3dm_get_base_rate_command
{
    static const uint8_t descriptorSet = MIP_3DM_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_3DM_GET_BASE_RATE;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0) const
    {
        return C::insert_mip_3dm_get_base_rate_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_3dm_get_base_rate_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = false;
    
    struct Response : C::mip_3dm_get_base_rate_response
    {
        static const uint8_t descriptorSet = MIP_3DM_CMD_DESC_SET;
        static const uint8_t fieldDescriptor = MIP_REPLY_DESC_3DM_BASE_RATE;
        
        size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::insert_mip_3dm_get_base_rate_response(buffer, bufferSize, offset, this);
        }
        size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::extract_mip_3dm_get_base_rate_response(buffer, bufferSize, offset, this);
        }
    };
    
};
MipCmdResult getBaseRate(C::mip_interface& device, uint8_t desc_set, uint16_t& rate)
{
    return C::mip_3dm_get_base_rate(&device, desc_set, &rate);
}



struct MessageFormat : C::mip_3dm_message_format_command
{
    static const uint8_t descriptorSet = MIP_3DM_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_3DM_MESSAGE_FORMAT;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0) const
    {
        return C::insert_mip_3dm_message_format_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_3dm_message_format_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = true;
    static const bool canWrite = true;
    static const bool canRead = true;
    static const bool canSave = true;
    static const bool canLoad = true;
    static const bool canReset = true;
    
    struct Response : C::mip_3dm_message_format_response
    {
        static const uint8_t descriptorSet = MIP_3DM_CMD_DESC_SET;
        static const uint8_t fieldDescriptor = MIP_REPLY_DESC_3DM_MESSAGE_FORMAT;
        
        size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::insert_mip_3dm_message_format_response(buffer, bufferSize, offset, this);
        }
        size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::extract_mip_3dm_message_format_response(buffer, bufferSize, offset, this);
        }
    };
    
};
MipCmdResult writeMessageFormat(C::mip_interface& device, uint8_t desc_set, uint8_t num_descriptors, const C::mip_descriptor_rate* descriptors)
{
    return C::mip_3dm_write_message_format(&device, desc_set, num_descriptors, descriptors);
}
MipCmdResult readMessageFormat(C::mip_interface& device, uint8_t desc_set, uint8_t& num_descriptors, struct C::mip_descriptor_rate* descriptors)
{
    return C::mip_3dm_read_message_format(&device, desc_set, &num_descriptors, descriptors);
}
MipCmdResult saveMessageFormat(C::mip_interface& device, uint8_t desc_set)
{
    return C::mip_3dm_save_message_format(&device, desc_set);
}
MipCmdResult loadMessageFormat(C::mip_interface& device, uint8_t desc_set)
{
    return C::mip_3dm_load_message_format(&device, desc_set);
}
MipCmdResult defaultMessageFormat(C::mip_interface& device, uint8_t desc_set)
{
    return C::mip_3dm_default_message_format(&device, desc_set);
}



struct NmeaPollData : C::mip_3dm_nmea_poll_data_command
{
    static const uint8_t descriptorSet = MIP_3DM_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_3DM_POLL_NMEA_MESSAGE;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0) const
    {
        return C::insert_mip_3dm_nmea_poll_data_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_3dm_nmea_poll_data_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = false;
    
};
MipCmdResult nmeaPollData(C::mip_interface& device, bool suppress_ack, uint8_t count, const C::mip_nmeamessage_format* format_entries)
{
    return C::mip_3dm_nmea_poll_data(&device, suppress_ack, count, format_entries);
}



struct NmeaMessageFormat : C::mip_3dm_nmea_message_format_command
{
    static const uint8_t descriptorSet = MIP_3DM_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_3DM_NMEA_MESSAGE_FORMAT;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0) const
    {
        return C::insert_mip_3dm_nmea_message_format_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_3dm_nmea_message_format_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = true;
    static const bool canWrite = true;
    static const bool canRead = true;
    static const bool canSave = true;
    static const bool canLoad = true;
    static const bool canReset = true;
    
    struct Response : C::mip_3dm_nmea_message_format_response
    {
        static const uint8_t descriptorSet = MIP_3DM_CMD_DESC_SET;
        static const uint8_t fieldDescriptor = MIP_REPLY_DESC_3DM_NMEA_MESSAGE_FORMAT;
        
        size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::insert_mip_3dm_nmea_message_format_response(buffer, bufferSize, offset, this);
        }
        size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::extract_mip_3dm_nmea_message_format_response(buffer, bufferSize, offset, this);
        }
    };
    
};
MipCmdResult writeNmeaMessageFormat(C::mip_interface& device, uint8_t count, const C::mip_nmeamessage_format* format_entries)
{
    return C::mip_3dm_write_nmea_message_format(&device, count, format_entries);
}
MipCmdResult readNmeaMessageFormat(C::mip_interface& device, uint8_t& count, struct C::mip_nmeamessage_format* format_entries)
{
    return C::mip_3dm_read_nmea_message_format(&device, &count, format_entries);
}
MipCmdResult saveNmeaMessageFormat(C::mip_interface& device)
{
    return C::mip_3dm_save_nmea_message_format(&device);
}
MipCmdResult loadNmeaMessageFormat(C::mip_interface& device)
{
    return C::mip_3dm_load_nmea_message_format(&device);
}
MipCmdResult defaultNmeaMessageFormat(C::mip_interface& device)
{
    return C::mip_3dm_default_nmea_message_format(&device);
}



struct DeviceSettings : C::mip_3dm_device_settings_command
{
    static const uint8_t descriptorSet = MIP_3DM_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_3DM_DEVICE_STARTUP_SETTINGS;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0) const
    {
        return C::insert_mip_3dm_device_settings_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_3dm_device_settings_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = true;
    static const bool canWrite = false;
    static const bool canRead = false;
    static const bool canSave = true;
    static const bool canLoad = true;
    static const bool canReset = true;
    
};
MipCmdResult saveDeviceSettings(C::mip_interface& device)
{
    return C::mip_3dm_save_device_settings(&device);
}
MipCmdResult loadDeviceSettings(C::mip_interface& device)
{
    return C::mip_3dm_load_device_settings(&device);
}
MipCmdResult defaultDeviceSettings(C::mip_interface& device)
{
    return C::mip_3dm_default_device_settings(&device);
}



struct UartBaudrate : C::mip_3dm_uart_baudrate_command
{
    static const uint8_t descriptorSet = MIP_3DM_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_3DM_UART_BAUDRATE;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0) const
    {
        return C::insert_mip_3dm_uart_baudrate_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_3dm_uart_baudrate_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = true;
    static const bool canWrite = true;
    static const bool canRead = true;
    static const bool canSave = true;
    static const bool canLoad = true;
    static const bool canReset = true;
    
    struct Response : C::mip_3dm_uart_baudrate_response
    {
        static const uint8_t descriptorSet = MIP_3DM_CMD_DESC_SET;
        static const uint8_t fieldDescriptor = MIP_REPLY_DESC_3DM_UART_BAUDRATE;
        
        size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::insert_mip_3dm_uart_baudrate_response(buffer, bufferSize, offset, this);
        }
        size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::extract_mip_3dm_uart_baudrate_response(buffer, bufferSize, offset, this);
        }
    };
    
};
MipCmdResult writeUartBaudrate(C::mip_interface& device, uint32_t baud)
{
    return C::mip_3dm_write_uart_baudrate(&device, baud);
}
MipCmdResult readUartBaudrate(C::mip_interface& device, uint32_t& baud)
{
    return C::mip_3dm_read_uart_baudrate(&device, &baud);
}
MipCmdResult saveUartBaudrate(C::mip_interface& device)
{
    return C::mip_3dm_save_uart_baudrate(&device);
}
MipCmdResult loadUartBaudrate(C::mip_interface& device)
{
    return C::mip_3dm_load_uart_baudrate(&device);
}
MipCmdResult defaultUartBaudrate(C::mip_interface& device)
{
    return C::mip_3dm_default_uart_baudrate(&device);
}



struct FactoryStreaming : C::mip_3dm_factory_streaming_command
{
    static const uint8_t descriptorSet = MIP_3DM_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_3DM_CONFIGURE_FACTORY_STREAMING;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0) const
    {
        return C::insert_mip_3dm_factory_streaming_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_3dm_factory_streaming_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = false;
    
};
MipCmdResult factoryStreaming(C::mip_interface& device, C::mip_3dm_factory_streaming_command_action action, uint8_t reserved)
{
    return C::mip_3dm_factory_streaming(&device, action, reserved);
}



struct DatastreamControl : C::mip_3dm_datastream_control_command
{
    static const uint8_t descriptorSet = MIP_3DM_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_3DM_CONTROL_DATA_STREAM;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0) const
    {
        return C::insert_mip_3dm_datastream_control_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_3dm_datastream_control_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = true;
    static const bool canWrite = true;
    static const bool canRead = true;
    static const bool canSave = true;
    static const bool canLoad = true;
    static const bool canReset = true;
    
    struct Response : C::mip_3dm_datastream_control_response
    {
        static const uint8_t descriptorSet = MIP_3DM_CMD_DESC_SET;
        static const uint8_t fieldDescriptor = MIP_REPLY_DESC_3DM_DATASTREAM_ENABLE;
        
        size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::insert_mip_3dm_datastream_control_response(buffer, bufferSize, offset, this);
        }
        size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::extract_mip_3dm_datastream_control_response(buffer, bufferSize, offset, this);
        }
    };
    
};
MipCmdResult writeDatastreamControl(C::mip_interface& device, uint8_t desc_set, bool enable)
{
    return C::mip_3dm_write_datastream_control(&device, desc_set, enable);
}
MipCmdResult readDatastreamControl(C::mip_interface& device, uint8_t desc_set, bool& enabled)
{
    return C::mip_3dm_read_datastream_control(&device, desc_set, &enabled);
}
MipCmdResult saveDatastreamControl(C::mip_interface& device, uint8_t desc_set)
{
    return C::mip_3dm_save_datastream_control(&device, desc_set);
}
MipCmdResult loadDatastreamControl(C::mip_interface& device, uint8_t desc_set)
{
    return C::mip_3dm_load_datastream_control(&device, desc_set);
}
MipCmdResult defaultDatastreamControl(C::mip_interface& device, uint8_t desc_set)
{
    return C::mip_3dm_default_datastream_control(&device, desc_set);
}



struct GnssSbasSettings : C::mip_3dm_gnss_sbas_settings_command
{
    static const uint8_t descriptorSet = MIP_3DM_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_3DM_GNSS_SBAS_SETTINGS;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0) const
    {
        return C::insert_mip_3dm_gnss_sbas_settings_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_3dm_gnss_sbas_settings_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = true;
    static const bool canWrite = true;
    static const bool canRead = true;
    static const bool canSave = true;
    static const bool canLoad = true;
    static const bool canReset = true;
    
    struct Response : C::mip_3dm_gnss_sbas_settings_response
    {
        static const uint8_t descriptorSet = MIP_3DM_CMD_DESC_SET;
        static const uint8_t fieldDescriptor = MIP_REPLY_DESC_3DM_GNSS_SBAS_SETTINGS;
        
        size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::insert_mip_3dm_gnss_sbas_settings_response(buffer, bufferSize, offset, this);
        }
        size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::extract_mip_3dm_gnss_sbas_settings_response(buffer, bufferSize, offset, this);
        }
    };
    
};
MipCmdResult writeGnssSbasSettings(C::mip_interface& device, uint8_t enable_sbas, C::mip_3dm_gnss_sbas_settings_command_sbasoptions sbas_options, uint8_t num_included_prns, const uint16_t* included_prns)
{
    return C::mip_3dm_write_gnss_sbas_settings(&device, enable_sbas, sbas_options, num_included_prns, included_prns);
}
MipCmdResult readGnssSbasSettings(C::mip_interface& device, uint8_t& enable_sbas, enum C::mip_3dm_gnss_sbas_settings_command_sbasoptions& sbas_options, uint8_t& num_included_prns, uint16_t* included_prns)
{
    return C::mip_3dm_read_gnss_sbas_settings(&device, &enable_sbas, &sbas_options, &num_included_prns, included_prns);
}
MipCmdResult saveGnssSbasSettings(C::mip_interface& device)
{
    return C::mip_3dm_save_gnss_sbas_settings(&device);
}
MipCmdResult loadGnssSbasSettings(C::mip_interface& device)
{
    return C::mip_3dm_load_gnss_sbas_settings(&device);
}
MipCmdResult defaultGnssSbasSettings(C::mip_interface& device)
{
    return C::mip_3dm_default_gnss_sbas_settings(&device);
}



struct GnssTimeAssistance : C::mip_3dm_gnss_time_assistance_command
{
    static const uint8_t descriptorSet = MIP_3DM_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_3DM_GNSS_TIME_ASSISTANCE;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0) const
    {
        return C::insert_mip_3dm_gnss_time_assistance_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_3dm_gnss_time_assistance_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = true;
    static const bool canWrite = true;
    static const bool canRead = true;
    static const bool canSave = false;
    static const bool canLoad = false;
    static const bool canReset = false;
    
    struct Response : C::mip_3dm_gnss_time_assistance_response
    {
        static const uint8_t descriptorSet = MIP_3DM_CMD_DESC_SET;
        static const uint8_t fieldDescriptor = MIP_REPLY_DESC_3DM_GNSS_TIME_ASSISTANCE;
        
        size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::insert_mip_3dm_gnss_time_assistance_response(buffer, bufferSize, offset, this);
        }
        size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::extract_mip_3dm_gnss_time_assistance_response(buffer, bufferSize, offset, this);
        }
    };
    
};
MipCmdResult writeGnssTimeAssistance(C::mip_interface& device, double tow, uint16_t week_number, float accuracy)
{
    return C::mip_3dm_write_gnss_time_assistance(&device, tow, week_number, accuracy);
}
MipCmdResult readGnssTimeAssistance(C::mip_interface& device, double& tow, uint16_t& week_number, float& accuracy)
{
    return C::mip_3dm_read_gnss_time_assistance(&device, &tow, &week_number, &accuracy);
}



struct AdvLowpassFilter : C::mip_3dm_adv_lowpass_filter_command
{
    static const uint8_t descriptorSet = MIP_3DM_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_3DM_ADVANCED_DATA_FILTER;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0) const
    {
        return C::insert_mip_3dm_adv_lowpass_filter_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_3dm_adv_lowpass_filter_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = true;
    static const bool canWrite = true;
    static const bool canRead = true;
    static const bool canSave = true;
    static const bool canLoad = true;
    static const bool canReset = true;
    
    struct Response : C::mip_3dm_adv_lowpass_filter_response
    {
        static const uint8_t descriptorSet = MIP_3DM_CMD_DESC_SET;
        static const uint8_t fieldDescriptor = MIP_REPLY_DESC_3DM_ADVANCED_DATA_FILTER;
        
        size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::insert_mip_3dm_adv_lowpass_filter_response(buffer, bufferSize, offset, this);
        }
        size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::extract_mip_3dm_adv_lowpass_filter_response(buffer, bufferSize, offset, this);
        }
    };
    
};
MipCmdResult writeAdvLowpassFilter(C::mip_interface& device, uint8_t target_descriptor, bool enable, bool manual, uint16_t frequency, uint8_t reserved)
{
    return C::mip_3dm_write_adv_lowpass_filter(&device, target_descriptor, enable, manual, frequency, reserved);
}
MipCmdResult readAdvLowpassFilter(C::mip_interface& device, uint8_t target_descriptor, bool& enable, bool& manual, uint16_t& frequency, uint8_t& reserved)
{
    return C::mip_3dm_read_adv_lowpass_filter(&device, target_descriptor, &enable, &manual, &frequency, &reserved);
}
MipCmdResult saveAdvLowpassFilter(C::mip_interface& device, uint8_t target_descriptor)
{
    return C::mip_3dm_save_adv_lowpass_filter(&device, target_descriptor);
}
MipCmdResult loadAdvLowpassFilter(C::mip_interface& device, uint8_t target_descriptor)
{
    return C::mip_3dm_load_adv_lowpass_filter(&device, target_descriptor);
}
MipCmdResult defaultAdvLowpassFilter(C::mip_interface& device, uint8_t target_descriptor)
{
    return C::mip_3dm_default_adv_lowpass_filter(&device, target_descriptor);
}



struct PpsSource : C::mip_3dm_pps_source_command
{
    static const uint8_t descriptorSet = MIP_3DM_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_3DM_PPS_SOURCE;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0) const
    {
        return C::insert_mip_3dm_pps_source_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_3dm_pps_source_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = true;
    static const bool canWrite = true;
    static const bool canRead = true;
    static const bool canSave = true;
    static const bool canLoad = true;
    static const bool canReset = true;
    
    struct Response : C::mip_3dm_pps_source_response
    {
        static const uint8_t descriptorSet = MIP_3DM_CMD_DESC_SET;
        static const uint8_t fieldDescriptor = MIP_REPLY_DESC_3DM_PPS_SOURCE;
        
        size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::insert_mip_3dm_pps_source_response(buffer, bufferSize, offset, this);
        }
        size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::extract_mip_3dm_pps_source_response(buffer, bufferSize, offset, this);
        }
    };
    
};
MipCmdResult writePpsSource(C::mip_interface& device, C::mip_pps_source source)
{
    return C::mip_3dm_write_pps_source(&device, source);
}
MipCmdResult readPpsSource(C::mip_interface& device, enum C::mip_pps_source& source)
{
    return C::mip_3dm_read_pps_source(&device, &source);
}
MipCmdResult savePpsSource(C::mip_interface& device)
{
    return C::mip_3dm_save_pps_source(&device);
}
MipCmdResult loadPpsSource(C::mip_interface& device)
{
    return C::mip_3dm_load_pps_source(&device);
}
MipCmdResult defaultPpsSource(C::mip_interface& device)
{
    return C::mip_3dm_default_pps_source(&device);
}



struct GpioConfig : C::mip_3dm_gpio_config_command
{
    static const uint8_t descriptorSet = MIP_3DM_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_3DM_GPIO_CONFIG;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0) const
    {
        return C::insert_mip_3dm_gpio_config_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_3dm_gpio_config_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = true;
    static const bool canWrite = true;
    static const bool canRead = true;
    static const bool canSave = true;
    static const bool canLoad = true;
    static const bool canReset = true;
    
    struct Response : C::mip_3dm_gpio_config_response
    {
        static const uint8_t descriptorSet = MIP_3DM_CMD_DESC_SET;
        static const uint8_t fieldDescriptor = MIP_REPLY_DESC_3DM_GPIO_CONFIG;
        
        size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::insert_mip_3dm_gpio_config_response(buffer, bufferSize, offset, this);
        }
        size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::extract_mip_3dm_gpio_config_response(buffer, bufferSize, offset, this);
        }
    };
    
};
MipCmdResult writeGpioConfig(C::mip_interface& device, uint8_t pin, C::mip_3dm_gpio_config_command_feature feature, C::mip_3dm_gpio_config_command_behavior behavior, C::mip_3dm_gpio_config_command_pin_mode pin_mode)
{
    return C::mip_3dm_write_gpio_config(&device, pin, feature, behavior, pin_mode);
}
MipCmdResult readGpioConfig(C::mip_interface& device, uint8_t pin, enum C::mip_3dm_gpio_config_command_feature& feature, enum C::mip_3dm_gpio_config_command_behavior& behavior, enum C::mip_3dm_gpio_config_command_pin_mode& pin_mode)
{
    return C::mip_3dm_read_gpio_config(&device, pin, &feature, &behavior, &pin_mode);
}
MipCmdResult saveGpioConfig(C::mip_interface& device, uint8_t pin)
{
    return C::mip_3dm_save_gpio_config(&device, pin);
}
MipCmdResult loadGpioConfig(C::mip_interface& device, uint8_t pin)
{
    return C::mip_3dm_load_gpio_config(&device, pin);
}
MipCmdResult defaultGpioConfig(C::mip_interface& device, uint8_t pin)
{
    return C::mip_3dm_default_gpio_config(&device, pin);
}



struct GpioState : C::mip_3dm_gpio_state_command
{
    static const uint8_t descriptorSet = MIP_3DM_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_3DM_GPIO_STATE;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0) const
    {
        return C::insert_mip_3dm_gpio_state_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_3dm_gpio_state_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = true;
    static const bool canWrite = true;
    static const bool canRead = true;
    static const bool canSave = false;
    static const bool canLoad = false;
    static const bool canReset = false;
    
    struct Response : C::mip_3dm_gpio_state_response
    {
        static const uint8_t descriptorSet = MIP_3DM_CMD_DESC_SET;
        static const uint8_t fieldDescriptor = MIP_REPLY_DESC_3DM_GPIO_STATE;
        
        size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::insert_mip_3dm_gpio_state_response(buffer, bufferSize, offset, this);
        }
        size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::extract_mip_3dm_gpio_state_response(buffer, bufferSize, offset, this);
        }
    };
    
};
MipCmdResult writeGpioState(C::mip_interface& device, uint8_t pin, bool state)
{
    return C::mip_3dm_write_gpio_state(&device, pin, state);
}
MipCmdResult readGpioState(C::mip_interface& device, uint8_t pin, bool& state)
{
    return C::mip_3dm_read_gpio_state(&device, pin, &state);
}



struct Odometer : C::mip_3dm_odometer_command
{
    static const uint8_t descriptorSet = MIP_3DM_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_3DM_ODOMETER_CONFIG;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0) const
    {
        return C::insert_mip_3dm_odometer_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_3dm_odometer_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = true;
    static const bool canWrite = true;
    static const bool canRead = true;
    static const bool canSave = true;
    static const bool canLoad = true;
    static const bool canReset = true;
    
    struct Response : C::mip_3dm_odometer_response
    {
        static const uint8_t descriptorSet = MIP_3DM_CMD_DESC_SET;
        static const uint8_t fieldDescriptor = MIP_REPLY_DESC_3DM_ODOMETER_CONFIG;
        
        size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::insert_mip_3dm_odometer_response(buffer, bufferSize, offset, this);
        }
        size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::extract_mip_3dm_odometer_response(buffer, bufferSize, offset, this);
        }
    };
    
};
MipCmdResult writeOdometer(C::mip_interface& device, C::mip_3dm_odometer_command_mode mode, float scaling, float uncertainty)
{
    return C::mip_3dm_write_odometer(&device, mode, scaling, uncertainty);
}
MipCmdResult readOdometer(C::mip_interface& device, enum C::mip_3dm_odometer_command_mode& mode, float& scaling, float& uncertainty)
{
    return C::mip_3dm_read_odometer(&device, &mode, &scaling, &uncertainty);
}
MipCmdResult saveOdometer(C::mip_interface& device)
{
    return C::mip_3dm_save_odometer(&device);
}
MipCmdResult loadOdometer(C::mip_interface& device)
{
    return C::mip_3dm_load_odometer(&device);
}
MipCmdResult defaultOdometer(C::mip_interface& device)
{
    return C::mip_3dm_default_odometer(&device);
}



struct GetEventSupport : C::mip_3dm_get_event_support_command
{
    static const uint8_t descriptorSet = MIP_3DM_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_3DM_EVENT_SUPPORT;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0) const
    {
        return C::insert_mip_3dm_get_event_support_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_3dm_get_event_support_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = false;
    
    struct Response : C::mip_3dm_get_event_support_response
    {
        static const uint8_t descriptorSet = MIP_3DM_CMD_DESC_SET;
        static const uint8_t fieldDescriptor = MIP_REPLY_DESC_3DM_EVENT_SUPPORT;
        
        size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::insert_mip_3dm_get_event_support_response(buffer, bufferSize, offset, this);
        }
        size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::extract_mip_3dm_get_event_support_response(buffer, bufferSize, offset, this);
        }
    };
    
};
MipCmdResult getEventSupport(C::mip_interface& device, C::mip_3dm_get_event_support_command_query query, uint8_t& max_instances, uint8_t& num_entries, struct C::mip_3dm_get_event_support_command_info* entries)
{
    return C::mip_3dm_get_event_support(&device, query, &max_instances, &num_entries, entries);
}



struct EventControl : C::mip_3dm_event_control_command
{
    static const uint8_t descriptorSet = MIP_3DM_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_3DM_EVENT_CONTROL;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0) const
    {
        return C::insert_mip_3dm_event_control_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_3dm_event_control_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = true;
    static const bool canWrite = true;
    static const bool canRead = true;
    static const bool canSave = true;
    static const bool canLoad = true;
    static const bool canReset = true;
    
    struct Response : C::mip_3dm_event_control_response
    {
        static const uint8_t descriptorSet = MIP_3DM_CMD_DESC_SET;
        static const uint8_t fieldDescriptor = MIP_REPLY_DESC_3DM_EVENT_CONTROL;
        
        size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::insert_mip_3dm_event_control_response(buffer, bufferSize, offset, this);
        }
        size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::extract_mip_3dm_event_control_response(buffer, bufferSize, offset, this);
        }
    };
    
};
MipCmdResult writeEventControl(C::mip_interface& device, uint8_t instance, C::mip_3dm_event_control_command_mode mode)
{
    return C::mip_3dm_write_event_control(&device, instance, mode);
}
MipCmdResult readEventControl(C::mip_interface& device, uint8_t instance, enum C::mip_3dm_event_control_command_mode& mode)
{
    return C::mip_3dm_read_event_control(&device, instance, &mode);
}
MipCmdResult saveEventControl(C::mip_interface& device, uint8_t instance)
{
    return C::mip_3dm_save_event_control(&device, instance);
}
MipCmdResult loadEventControl(C::mip_interface& device, uint8_t instance)
{
    return C::mip_3dm_load_event_control(&device, instance);
}
MipCmdResult defaultEventControl(C::mip_interface& device, uint8_t instance)
{
    return C::mip_3dm_default_event_control(&device, instance);
}



struct GetEventTriggerStatus : C::mip_3dm_get_event_trigger_status_command
{
    static const uint8_t descriptorSet = MIP_3DM_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_3DM_EVENT_TRIGGER_STATUS;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0) const
    {
        return C::insert_mip_3dm_get_event_trigger_status_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_3dm_get_event_trigger_status_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = false;
    
    struct Response : C::mip_3dm_get_event_trigger_status_response
    {
        static const uint8_t descriptorSet = MIP_3DM_CMD_DESC_SET;
        static const uint8_t fieldDescriptor = MIP_REPLY_DESC_3DM_EVENT_TRIGGER_STATUS;
        
        size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::insert_mip_3dm_get_event_trigger_status_response(buffer, bufferSize, offset, this);
        }
        size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::extract_mip_3dm_get_event_trigger_status_response(buffer, bufferSize, offset, this);
        }
    };
    
};
MipCmdResult getEventTriggerStatus(C::mip_interface& device, uint8_t requested_count, const uint8_t* requested_instances, uint8_t& count, struct C::mip_3dm_get_event_trigger_status_command_entry* triggers)
{
    return C::mip_3dm_get_event_trigger_status(&device, requested_count, requested_instances, &count, triggers);
}



struct GetEventActionStatus : C::mip_3dm_get_event_action_status_command
{
    static const uint8_t descriptorSet = MIP_3DM_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_3DM_EVENT_ACTION_STATUS;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0) const
    {
        return C::insert_mip_3dm_get_event_action_status_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_3dm_get_event_action_status_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = false;
    
    struct Response : C::mip_3dm_get_event_action_status_response
    {
        static const uint8_t descriptorSet = MIP_3DM_CMD_DESC_SET;
        static const uint8_t fieldDescriptor = MIP_REPLY_DESC_3DM_EVENT_ACTION_STATUS;
        
        size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::insert_mip_3dm_get_event_action_status_response(buffer, bufferSize, offset, this);
        }
        size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::extract_mip_3dm_get_event_action_status_response(buffer, bufferSize, offset, this);
        }
    };
    
};
MipCmdResult getEventActionStatus(C::mip_interface& device, uint8_t requested_count, const uint8_t* requested_instances, uint8_t& count, struct C::mip_3dm_get_event_action_status_command_entry* actions)
{
    return C::mip_3dm_get_event_action_status(&device, requested_count, requested_instances, &count, actions);
}



struct EventTrigger : C::mip_3dm_event_trigger_command
{
    static const uint8_t descriptorSet = MIP_3DM_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_3DM_EVENT_TRIGGER_CONFIG;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0) const
    {
        return C::insert_mip_3dm_event_trigger_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_3dm_event_trigger_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = true;
    static const bool canWrite = true;
    static const bool canRead = true;
    static const bool canSave = true;
    static const bool canLoad = true;
    static const bool canReset = true;
    
    struct Response : C::mip_3dm_event_trigger_response
    {
        static const uint8_t descriptorSet = MIP_3DM_CMD_DESC_SET;
        static const uint8_t fieldDescriptor = MIP_REPLY_DESC_3DM_EVENT_TRIGGER_CONFIG;
        
        size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::insert_mip_3dm_event_trigger_response(buffer, bufferSize, offset, this);
        }
        size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::extract_mip_3dm_event_trigger_response(buffer, bufferSize, offset, this);
        }
    };
    
};
MipCmdResult writeEventTrigger(C::mip_interface& device, uint8_t instance, C::mip_3dm_event_trigger_command_type type, const void* gpio_threshold_combination)
{
    return C::mip_3dm_write_event_trigger(&device, instance, type, gpio_threshold_combination);
}
MipCmdResult readEventTrigger(C::mip_interface& device, uint8_t instance, enum C::mip_3dm_event_trigger_command_type& type, void* gpio_threshold_combination)
{
    return C::mip_3dm_read_event_trigger(&device, instance, &type, gpio_threshold_combination);
}
MipCmdResult saveEventTrigger(C::mip_interface& device, uint8_t instance)
{
    return C::mip_3dm_save_event_trigger(&device, instance);
}
MipCmdResult loadEventTrigger(C::mip_interface& device, uint8_t instance)
{
    return C::mip_3dm_load_event_trigger(&device, instance);
}
MipCmdResult defaultEventTrigger(C::mip_interface& device, uint8_t instance)
{
    return C::mip_3dm_default_event_trigger(&device, instance);
}



struct EventAction : C::mip_3dm_event_action_command
{
    static const uint8_t descriptorSet = MIP_3DM_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_3DM_EVENT_ACTION_CONFIG;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0) const
    {
        return C::insert_mip_3dm_event_action_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_3dm_event_action_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = true;
    static const bool canWrite = true;
    static const bool canRead = true;
    static const bool canSave = true;
    static const bool canLoad = true;
    static const bool canReset = true;
    
    struct Response : C::mip_3dm_event_action_response
    {
        static const uint8_t descriptorSet = MIP_3DM_CMD_DESC_SET;
        static const uint8_t fieldDescriptor = MIP_REPLY_DESC_3DM_EVENT_ACTION_CONFIG;
        
        size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::insert_mip_3dm_event_action_response(buffer, bufferSize, offset, this);
        }
        size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::extract_mip_3dm_event_action_response(buffer, bufferSize, offset, this);
        }
    };
    
};
MipCmdResult writeEventAction(C::mip_interface& device, uint8_t instance, uint8_t trigger, C::mip_3dm_event_action_command_type type, const void* gpio_message)
{
    return C::mip_3dm_write_event_action(&device, instance, trigger, type, gpio_message);
}
MipCmdResult readEventAction(C::mip_interface& device, uint8_t instance, uint8_t& trigger, enum C::mip_3dm_event_action_command_type& type, void* gpio_message)
{
    return C::mip_3dm_read_event_action(&device, instance, &trigger, &type, gpio_message);
}
MipCmdResult saveEventAction(C::mip_interface& device, uint8_t instance)
{
    return C::mip_3dm_save_event_action(&device, instance);
}
MipCmdResult loadEventAction(C::mip_interface& device, uint8_t instance)
{
    return C::mip_3dm_load_event_action(&device, instance);
}
MipCmdResult defaultEventAction(C::mip_interface& device, uint8_t instance)
{
    return C::mip_3dm_default_event_action(&device, instance);
}



struct AccelBias : C::mip_3dm_accel_bias_command
{
    static const uint8_t descriptorSet = MIP_3DM_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_3DM_ACCEL_BIAS;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0) const
    {
        return C::insert_mip_3dm_accel_bias_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_3dm_accel_bias_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = true;
    static const bool canWrite = true;
    static const bool canRead = true;
    static const bool canSave = true;
    static const bool canLoad = true;
    static const bool canReset = true;
    
    struct Response : C::mip_3dm_accel_bias_response
    {
        static const uint8_t descriptorSet = MIP_3DM_CMD_DESC_SET;
        static const uint8_t fieldDescriptor = MIP_REPLY_DESC_3DM_ACCEL_BIAS_VECTOR;
        
        size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::insert_mip_3dm_accel_bias_response(buffer, bufferSize, offset, this);
        }
        size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::extract_mip_3dm_accel_bias_response(buffer, bufferSize, offset, this);
        }
    };
    
};
MipCmdResult writeAccelBias(C::mip_interface& device, const float* bias)
{
    return C::mip_3dm_write_accel_bias(&device, bias);
}
MipCmdResult readAccelBias(C::mip_interface& device, float* bias)
{
    return C::mip_3dm_read_accel_bias(&device, bias);
}
MipCmdResult saveAccelBias(C::mip_interface& device)
{
    return C::mip_3dm_save_accel_bias(&device);
}
MipCmdResult loadAccelBias(C::mip_interface& device)
{
    return C::mip_3dm_load_accel_bias(&device);
}
MipCmdResult defaultAccelBias(C::mip_interface& device)
{
    return C::mip_3dm_default_accel_bias(&device);
}



struct GyroBias : C::mip_3dm_gyro_bias_command
{
    static const uint8_t descriptorSet = MIP_3DM_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_3DM_GYRO_BIAS;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0) const
    {
        return C::insert_mip_3dm_gyro_bias_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_3dm_gyro_bias_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = true;
    static const bool canWrite = true;
    static const bool canRead = true;
    static const bool canSave = true;
    static const bool canLoad = true;
    static const bool canReset = true;
    
    struct Response : C::mip_3dm_gyro_bias_response
    {
        static const uint8_t descriptorSet = MIP_3DM_CMD_DESC_SET;
        static const uint8_t fieldDescriptor = MIP_REPLY_DESC_3DM_GYRO_BIAS_VECTOR;
        
        size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::insert_mip_3dm_gyro_bias_response(buffer, bufferSize, offset, this);
        }
        size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::extract_mip_3dm_gyro_bias_response(buffer, bufferSize, offset, this);
        }
    };
    
};
MipCmdResult writeGyroBias(C::mip_interface& device, const float* bias)
{
    return C::mip_3dm_write_gyro_bias(&device, bias);
}
MipCmdResult readGyroBias(C::mip_interface& device, float* bias)
{
    return C::mip_3dm_read_gyro_bias(&device, bias);
}
MipCmdResult saveGyroBias(C::mip_interface& device)
{
    return C::mip_3dm_save_gyro_bias(&device);
}
MipCmdResult loadGyroBias(C::mip_interface& device)
{
    return C::mip_3dm_load_gyro_bias(&device);
}
MipCmdResult defaultGyroBias(C::mip_interface& device)
{
    return C::mip_3dm_default_gyro_bias(&device);
}



struct CaptureGyroBias : C::mip_3dm_capture_gyro_bias_command
{
    static const uint8_t descriptorSet = MIP_3DM_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_3DM_CAPTURE_GYRO_BIAS;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0) const
    {
        return C::insert_mip_3dm_capture_gyro_bias_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_3dm_capture_gyro_bias_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = false;
    
    struct Response : C::mip_3dm_capture_gyro_bias_response
    {
        static const uint8_t descriptorSet = MIP_3DM_CMD_DESC_SET;
        static const uint8_t fieldDescriptor = MIP_REPLY_DESC_3DM_GYRO_BIAS_VECTOR;
        
        size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::insert_mip_3dm_capture_gyro_bias_response(buffer, bufferSize, offset, this);
        }
        size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::extract_mip_3dm_capture_gyro_bias_response(buffer, bufferSize, offset, this);
        }
    };
    
};
MipCmdResult captureGyroBias(C::mip_interface& device, uint16_t averaging_time_ms, float* bias)
{
    return C::mip_3dm_capture_gyro_bias(&device, averaging_time_ms, bias);
}



struct MagHardIronOffset : C::mip_3dm_mag_hard_iron_offset_command
{
    static const uint8_t descriptorSet = MIP_3DM_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_3DM_HARD_IRON_OFFSET;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0) const
    {
        return C::insert_mip_3dm_mag_hard_iron_offset_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_3dm_mag_hard_iron_offset_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = true;
    static const bool canWrite = true;
    static const bool canRead = true;
    static const bool canSave = true;
    static const bool canLoad = true;
    static const bool canReset = true;
    
    struct Response : C::mip_3dm_mag_hard_iron_offset_response
    {
        static const uint8_t descriptorSet = MIP_3DM_CMD_DESC_SET;
        static const uint8_t fieldDescriptor = MIP_REPLY_DESC_3DM_HARD_IRON_OFFSET_VECTOR;
        
        size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::insert_mip_3dm_mag_hard_iron_offset_response(buffer, bufferSize, offset, this);
        }
        size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::extract_mip_3dm_mag_hard_iron_offset_response(buffer, bufferSize, offset, this);
        }
    };
    
};
MipCmdResult writeMagHardIronOffset(C::mip_interface& device, const float* offset)
{
    return C::mip_3dm_write_mag_hard_iron_offset(&device, offset);
}
MipCmdResult readMagHardIronOffset(C::mip_interface& device, float* offset)
{
    return C::mip_3dm_read_mag_hard_iron_offset(&device, offset);
}
MipCmdResult saveMagHardIronOffset(C::mip_interface& device)
{
    return C::mip_3dm_save_mag_hard_iron_offset(&device);
}
MipCmdResult loadMagHardIronOffset(C::mip_interface& device)
{
    return C::mip_3dm_load_mag_hard_iron_offset(&device);
}
MipCmdResult defaultMagHardIronOffset(C::mip_interface& device)
{
    return C::mip_3dm_default_mag_hard_iron_offset(&device);
}



struct MagSoftIronMatrix : C::mip_3dm_mag_soft_iron_matrix_command
{
    static const uint8_t descriptorSet = MIP_3DM_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_3DM_SOFT_IRON_MATRIX;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0) const
    {
        return C::insert_mip_3dm_mag_soft_iron_matrix_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_3dm_mag_soft_iron_matrix_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = true;
    static const bool canWrite = true;
    static const bool canRead = true;
    static const bool canSave = true;
    static const bool canLoad = true;
    static const bool canReset = true;
    
    struct Response : C::mip_3dm_mag_soft_iron_matrix_response
    {
        static const uint8_t descriptorSet = MIP_3DM_CMD_DESC_SET;
        static const uint8_t fieldDescriptor = MIP_REPLY_DESC_3DM_SOFT_IRON_COMP_MATRIX;
        
        size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::insert_mip_3dm_mag_soft_iron_matrix_response(buffer, bufferSize, offset, this);
        }
        size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::extract_mip_3dm_mag_soft_iron_matrix_response(buffer, bufferSize, offset, this);
        }
    };
    
};
MipCmdResult writeMagSoftIronMatrix(C::mip_interface& device, const float* offset)
{
    return C::mip_3dm_write_mag_soft_iron_matrix(&device, offset);
}
MipCmdResult readMagSoftIronMatrix(C::mip_interface& device, float* offset)
{
    return C::mip_3dm_read_mag_soft_iron_matrix(&device, offset);
}
MipCmdResult saveMagSoftIronMatrix(C::mip_interface& device)
{
    return C::mip_3dm_save_mag_soft_iron_matrix(&device);
}
MipCmdResult loadMagSoftIronMatrix(C::mip_interface& device)
{
    return C::mip_3dm_load_mag_soft_iron_matrix(&device);
}
MipCmdResult defaultMagSoftIronMatrix(C::mip_interface& device)
{
    return C::mip_3dm_default_mag_soft_iron_matrix(&device);
}



struct Sensor2VehicleTransformEuler : C::mip_3dm_sensor_2_vehicle_transform_euler_command
{
    static const uint8_t descriptorSet = MIP_3DM_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_3DM_SENSOR2VEHICLE_TRANSFORM_EUL;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0) const
    {
        return C::insert_mip_3dm_sensor_2_vehicle_transform_euler_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_3dm_sensor_2_vehicle_transform_euler_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = true;
    static const bool canWrite = true;
    static const bool canRead = true;
    static const bool canSave = true;
    static const bool canLoad = true;
    static const bool canReset = true;
    
    struct Response : C::mip_3dm_sensor_2_vehicle_transform_euler_response
    {
        static const uint8_t descriptorSet = MIP_3DM_CMD_DESC_SET;
        static const uint8_t fieldDescriptor = MIP_REPLY_DESC_3DM_SENSOR2VEHICLE_TRANSFORM_EUL;
        
        size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::insert_mip_3dm_sensor_2_vehicle_transform_euler_response(buffer, bufferSize, offset, this);
        }
        size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::extract_mip_3dm_sensor_2_vehicle_transform_euler_response(buffer, bufferSize, offset, this);
        }
    };
    
};
MipCmdResult writeSensor2VehicleTransformEuler(C::mip_interface& device, float roll, float pitch, float yaw)
{
    return C::mip_3dm_write_sensor_2_vehicle_transform_euler(&device, roll, pitch, yaw);
}
MipCmdResult readSensor2VehicleTransformEuler(C::mip_interface& device, float& roll, float& pitch, float& yaw)
{
    return C::mip_3dm_read_sensor_2_vehicle_transform_euler(&device, &roll, &pitch, &yaw);
}
MipCmdResult saveSensor2VehicleTransformEuler(C::mip_interface& device)
{
    return C::mip_3dm_save_sensor_2_vehicle_transform_euler(&device);
}
MipCmdResult loadSensor2VehicleTransformEuler(C::mip_interface& device)
{
    return C::mip_3dm_load_sensor_2_vehicle_transform_euler(&device);
}
MipCmdResult defaultSensor2VehicleTransformEuler(C::mip_interface& device)
{
    return C::mip_3dm_default_sensor_2_vehicle_transform_euler(&device);
}



struct Sensor2VehicleTransformQuaternion : C::mip_3dm_sensor_2_vehicle_transform_quaternion_command
{
    static const uint8_t descriptorSet = MIP_3DM_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_3DM_SENSOR2VEHICLE_TRANSFORM_QUAT;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0) const
    {
        return C::insert_mip_3dm_sensor_2_vehicle_transform_quaternion_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_3dm_sensor_2_vehicle_transform_quaternion_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = true;
    static const bool canWrite = true;
    static const bool canRead = true;
    static const bool canSave = true;
    static const bool canLoad = true;
    static const bool canReset = true;
    
    struct Response : C::mip_3dm_sensor_2_vehicle_transform_quaternion_response
    {
        static const uint8_t descriptorSet = MIP_3DM_CMD_DESC_SET;
        static const uint8_t fieldDescriptor = MIP_REPLY_DESC_3DM_SENSOR2VEHICLE_TRANSFORM_QUAT;
        
        size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::insert_mip_3dm_sensor_2_vehicle_transform_quaternion_response(buffer, bufferSize, offset, this);
        }
        size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::extract_mip_3dm_sensor_2_vehicle_transform_quaternion_response(buffer, bufferSize, offset, this);
        }
    };
    
};
MipCmdResult writeSensor2VehicleTransformQuaternion(C::mip_interface& device, const float* q)
{
    return C::mip_3dm_write_sensor_2_vehicle_transform_quaternion(&device, q);
}
MipCmdResult readSensor2VehicleTransformQuaternion(C::mip_interface& device, float* q)
{
    return C::mip_3dm_read_sensor_2_vehicle_transform_quaternion(&device, q);
}
MipCmdResult saveSensor2VehicleTransformQuaternion(C::mip_interface& device)
{
    return C::mip_3dm_save_sensor_2_vehicle_transform_quaternion(&device);
}
MipCmdResult loadSensor2VehicleTransformQuaternion(C::mip_interface& device)
{
    return C::mip_3dm_load_sensor_2_vehicle_transform_quaternion(&device);
}
MipCmdResult defaultSensor2VehicleTransformQuaternion(C::mip_interface& device)
{
    return C::mip_3dm_default_sensor_2_vehicle_transform_quaternion(&device);
}



struct Sensor2VehicleTransformDcm : C::mip_3dm_sensor_2_vehicle_transform_dcm_command
{
    static const uint8_t descriptorSet = MIP_3DM_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_3DM_SENSOR2VEHICLE_TRANSFORM_DCM;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0) const
    {
        return C::insert_mip_3dm_sensor_2_vehicle_transform_dcm_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_3dm_sensor_2_vehicle_transform_dcm_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = true;
    static const bool canWrite = true;
    static const bool canRead = true;
    static const bool canSave = true;
    static const bool canLoad = true;
    static const bool canReset = true;
    
    struct Response : C::mip_3dm_sensor_2_vehicle_transform_dcm_response
    {
        static const uint8_t descriptorSet = MIP_3DM_CMD_DESC_SET;
        static const uint8_t fieldDescriptor = MIP_REPLY_DESC_3DM_SENSOR2VEHICLE_TRANSFORM_DCM;
        
        size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::insert_mip_3dm_sensor_2_vehicle_transform_dcm_response(buffer, bufferSize, offset, this);
        }
        size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::extract_mip_3dm_sensor_2_vehicle_transform_dcm_response(buffer, bufferSize, offset, this);
        }
    };
    
};
MipCmdResult writeSensor2VehicleTransformDcm(C::mip_interface& device, const float* dcm)
{
    return C::mip_3dm_write_sensor_2_vehicle_transform_dcm(&device, dcm);
}
MipCmdResult readSensor2VehicleTransformDcm(C::mip_interface& device, float* dcm)
{
    return C::mip_3dm_read_sensor_2_vehicle_transform_dcm(&device, dcm);
}
MipCmdResult saveSensor2VehicleTransformDcm(C::mip_interface& device)
{
    return C::mip_3dm_save_sensor_2_vehicle_transform_dcm(&device);
}
MipCmdResult loadSensor2VehicleTransformDcm(C::mip_interface& device)
{
    return C::mip_3dm_load_sensor_2_vehicle_transform_dcm(&device);
}
MipCmdResult defaultSensor2VehicleTransformDcm(C::mip_interface& device)
{
    return C::mip_3dm_default_sensor_2_vehicle_transform_dcm(&device);
}



struct ComplementaryFilter : C::mip_3dm_complementary_filter_command
{
    static const uint8_t descriptorSet = MIP_3DM_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_3DM_LEGACY_COMP_FILTER;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0) const
    {
        return C::insert_mip_3dm_complementary_filter_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_3dm_complementary_filter_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = true;
    static const bool canWrite = true;
    static const bool canRead = true;
    static const bool canSave = true;
    static const bool canLoad = true;
    static const bool canReset = true;
    
    struct Response : C::mip_3dm_complementary_filter_response
    {
        static const uint8_t descriptorSet = MIP_3DM_CMD_DESC_SET;
        static const uint8_t fieldDescriptor = MIP_REPLY_DESC_3DM_LEGACY_COMP_FILTER;
        
        size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::insert_mip_3dm_complementary_filter_response(buffer, bufferSize, offset, this);
        }
        size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::extract_mip_3dm_complementary_filter_response(buffer, bufferSize, offset, this);
        }
    };
    
};
MipCmdResult writeComplementaryFilter(C::mip_interface& device, bool pitch_roll_enable, bool heading_enable, float pitch_roll_time_constant, float heading_time_constant)
{
    return C::mip_3dm_write_complementary_filter(&device, pitch_roll_enable, heading_enable, pitch_roll_time_constant, heading_time_constant);
}
MipCmdResult readComplementaryFilter(C::mip_interface& device, bool& pitch_roll_enable, bool& heading_enable, float& pitch_roll_time_constant, float& heading_time_constant)
{
    return C::mip_3dm_read_complementary_filter(&device, &pitch_roll_enable, &heading_enable, &pitch_roll_time_constant, &heading_time_constant);
}
MipCmdResult saveComplementaryFilter(C::mip_interface& device)
{
    return C::mip_3dm_save_complementary_filter(&device);
}
MipCmdResult loadComplementaryFilter(C::mip_interface& device)
{
    return C::mip_3dm_load_complementary_filter(&device);
}
MipCmdResult defaultComplementaryFilter(C::mip_interface& device)
{
    return C::mip_3dm_default_complementary_filter(&device);
}



struct SensorRange : C::mip_3dm_sensor_range_command
{
    static const uint8_t descriptorSet = MIP_3DM_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_3DM_SENSOR_RANGE;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0) const
    {
        return C::insert_mip_3dm_sensor_range_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_3dm_sensor_range_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = true;
    static const bool canWrite = true;
    static const bool canRead = true;
    static const bool canSave = true;
    static const bool canLoad = true;
    static const bool canReset = true;
    
    struct Response : C::mip_3dm_sensor_range_response
    {
        static const uint8_t descriptorSet = MIP_3DM_CMD_DESC_SET;
        static const uint8_t fieldDescriptor = MIP_REPLY_DESC_3DM_SENSOR_RANGE;
        
        size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::insert_mip_3dm_sensor_range_response(buffer, bufferSize, offset, this);
        }
        size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::extract_mip_3dm_sensor_range_response(buffer, bufferSize, offset, this);
        }
    };
    
};
MipCmdResult writeSensorRange(C::mip_interface& device, C::mip_sensor_range_type sensor, uint8_t setting)
{
    return C::mip_3dm_write_sensor_range(&device, sensor, setting);
}
MipCmdResult readSensorRange(C::mip_interface& device, C::mip_sensor_range_type sensor, uint8_t& setting)
{
    return C::mip_3dm_read_sensor_range(&device, sensor, &setting);
}
MipCmdResult saveSensorRange(C::mip_interface& device, C::mip_sensor_range_type sensor)
{
    return C::mip_3dm_save_sensor_range(&device, sensor);
}
MipCmdResult loadSensorRange(C::mip_interface& device, C::mip_sensor_range_type sensor)
{
    return C::mip_3dm_load_sensor_range(&device, sensor);
}
MipCmdResult defaultSensorRange(C::mip_interface& device, C::mip_sensor_range_type sensor)
{
    return C::mip_3dm_default_sensor_range(&device, sensor);
}



struct CalibratedSensorRanges : C::mip_3dm_calibrated_sensor_ranges_command
{
    static const uint8_t descriptorSet = MIP_3DM_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_3DM_CALIBRATED_RANGES;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0) const
    {
        return C::insert_mip_3dm_calibrated_sensor_ranges_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_3dm_calibrated_sensor_ranges_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = false;
    
    struct Response : C::mip_3dm_calibrated_sensor_ranges_response
    {
        static const uint8_t descriptorSet = MIP_3DM_CMD_DESC_SET;
        static const uint8_t fieldDescriptor = MIP_REPLY_DESC_3DM_CALIBRATED_RANGES;
        
        size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::insert_mip_3dm_calibrated_sensor_ranges_response(buffer, bufferSize, offset, this);
        }
        size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::extract_mip_3dm_calibrated_sensor_ranges_response(buffer, bufferSize, offset, this);
        }
    };
    
};
MipCmdResult calibratedSensorRanges(C::mip_interface& device, C::mip_sensor_range_type sensor, uint8_t& num_ranges, struct C::mip_3dm_calibrated_sensor_ranges_command_entry* ranges)
{
    return C::mip_3dm_calibrated_sensor_ranges(&device, sensor, &num_ranges, ranges);
}



} // namespace TdmCommands
} // namespace mscl
#endif // __cplusplus
