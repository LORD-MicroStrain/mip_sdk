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
///@addtogroup MipData
///@{
///@defgroup GNSSData  GNSS
///
///@{

////////////////////////////////////////////////////////////////////////////////
// Descriptors
////////////////////////////////////////////////////////////////////////////////

enum mip_gnss_data_descriptors
{
    MIP_GNSS_DATA_DESC_SET                    = 0x81,
    
    MIP_DATA_DESC_GNSS_POSITION_LLH           = 0x03,
    MIP_DATA_DESC_GNSS_POSITION_ECEF          = 0x04,
    MIP_DATA_DESC_GNSS_VELOCITY_NED           = 0x05,
    MIP_DATA_DESC_GNSS_VELOCITY_ECEF          = 0x06,
    MIP_DATA_DESC_GNSS_DOP                    = 0x07,
    MIP_DATA_DESC_GNSS_UTC_TIME               = 0x08,
    MIP_DATA_DESC_GNSS_GPS_TIME               = 0x09,
    MIP_DATA_DESC_GNSS_CLOCK_INFO             = 0x0A,
    MIP_DATA_DESC_GNSS_FIX_INFO               = 0x0B,
    MIP_DATA_DESC_GNSS_SV_INFO                = 0x0C,
    MIP_DATA_DESC_GNSS_HW_STATUS              = 0x0D,
    MIP_DATA_DESC_GNSS_DGPS_INFO              = 0x0E,
    MIP_DATA_DESC_GNSS_DGPS_CHANNEL_STATUS    = 0x0F,
    MIP_DATA_DESC_GNSS_CLOCK_INFO_2           = 0x10,
    MIP_DATA_DESC_GNSS_GPS_LEAP_SECONDS       = 0x11,
    MIP_DATA_DESC_GNSS_SBAS_INFO              = 0x12,
    MIP_DATA_DESC_GNSS_SBAS_CORRECTION        = 0x13,
    MIP_DATA_DESC_GNSS_RF_ERROR_DETECTION     = 0x14,
    MIP_DATA_DESC_GNSS_BASE_STATION_INFO      = 0x30,
    MIP_DATA_DESC_GNSS_RTK_CORRECTIONS_STATUS = 0x31,
    MIP_DATA_DESC_GNSS_SATELLITE_STATUS       = 0x20,
    MIP_DATA_DESC_GNSS_RAW                    = 0x22,
    MIP_DATA_DESC_GNSS_GPS_EPHEMERIS          = 0x61,
    MIP_DATA_DESC_GNSS_GLONASS_EPHEMERIS      = 0x62,
    MIP_DATA_DESC_GNSS_GPS_IONO_CORR          = 0x71,
    MIP_DATA_DESC_GNSS_GALILEO_IONO_CORR      = 0x73,
    
};
#ifdef __cplusplus
namespace C {
extern "C" {
#endif // __cplusplus


////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////

#define MIP_GNSS1_DATA_DESC_SET_MIP_GNSS1_DATA_DESC_SET 0x91
#define MIP_GNSS2_DATA_DESC_SET_MIP_GNSS2_DATA_DESC_SET 0x92
#define MIP_GNSS3_DATA_DESC_SET_MIP_GNSS3_DATA_DESC_SET 0x93
#define MIP_GNSS4_DATA_DESC_SET_MIP_GNSS4_DATA_DESC_SET 0x94
#define MIP_GNSS5_DATA_DESC_SET_MIP_GNSS5_DATA_DESC_SET 0x95
enum mip_gnss_constellation_id
{
    MIP_GNSS_CONSTELLATION_ID_UNKNOWN = 0,  ///<  
    MIP_GNSS_CONSTELLATION_ID_GPS     = 1,  ///<  
    MIP_GNSS_CONSTELLATION_ID_GLONASS = 2,  ///<  
    MIP_GNSS_CONSTELLATION_ID_GALILEO = 3,  ///<  
    MIP_GNSS_CONSTELLATION_ID_BEIDOU  = 4,  ///<  
    MIP_GNSS_CONSTELLATION_ID_SBAS    = 5,  ///<  
};
size_t insert_mip_gnss_constellation_id(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_gnss_constellation_id self);
size_t extract_mip_gnss_constellation_id(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_gnss_constellation_id* self);

enum mip_gnss_signal_id
{
    MIP_GNSS_SIGNAL_ID_UNKNOWN        = 0,  ///<  
    MIP_GNSS_SIGNAL_ID_GPS_L1CA       = 1,  ///<  
    MIP_GNSS_SIGNAL_ID_GPS_L1P        = 2,  ///<  
    MIP_GNSS_SIGNAL_ID_GPS_L1Z        = 3,  ///<  
    MIP_GNSS_SIGNAL_ID_GPS_L2CA       = 4,  ///<  
    MIP_GNSS_SIGNAL_ID_GPS_L2P        = 5,  ///<  
    MIP_GNSS_SIGNAL_ID_GPS_L2Z        = 6,  ///<  
    MIP_GNSS_SIGNAL_ID_GPS_L2CL       = 7,  ///<  
    MIP_GNSS_SIGNAL_ID_GPS_L2CM       = 8,  ///<  
    MIP_GNSS_SIGNAL_ID_GPS_L2CML      = 9,  ///<  
    MIP_GNSS_SIGNAL_ID_GPS_L5I        = 10,  ///<  
    MIP_GNSS_SIGNAL_ID_GPS_L5Q        = 11,  ///<  
    MIP_GNSS_SIGNAL_ID_GPS_L5IQ       = 12,  ///<  
    MIP_GNSS_SIGNAL_ID_GPS_L1CD       = 13,  ///<  
    MIP_GNSS_SIGNAL_ID_GPS_L1CP       = 14,  ///<  
    MIP_GNSS_SIGNAL_ID_GPS_L1CDP      = 15,  ///<  
    MIP_GNSS_SIGNAL_ID_GLONASS_G1CA   = 32,  ///<  
    MIP_GNSS_SIGNAL_ID_GLONASS_G1P    = 33,  ///<  
    MIP_GNSS_SIGNAL_ID_GLONASS_G2C    = 34,  ///<  
    MIP_GNSS_SIGNAL_ID_GLONASS_G2P    = 35,  ///<  
    MIP_GNSS_SIGNAL_ID_GALILEO_E1C    = 64,  ///<  
    MIP_GNSS_SIGNAL_ID_GALILEO_E1A    = 65,  ///<  
    MIP_GNSS_SIGNAL_ID_GALILEO_E1B    = 66,  ///<  
    MIP_GNSS_SIGNAL_ID_GALILEO_E1BC   = 67,  ///<  
    MIP_GNSS_SIGNAL_ID_GALILEO_E1ABC  = 68,  ///<  
    MIP_GNSS_SIGNAL_ID_GALILEO_E6C    = 69,  ///<  
    MIP_GNSS_SIGNAL_ID_GALILEO_E6A    = 70,  ///<  
    MIP_GNSS_SIGNAL_ID_GALILEO_E6B    = 71,  ///<  
    MIP_GNSS_SIGNAL_ID_GALILEO_E6BC   = 72,  ///<  
    MIP_GNSS_SIGNAL_ID_GALILEO_E6ABC  = 73,  ///<  
    MIP_GNSS_SIGNAL_ID_GALILEO_E5BI   = 74,  ///<  
    MIP_GNSS_SIGNAL_ID_GALILEO_E5BQ   = 75,  ///<  
    MIP_GNSS_SIGNAL_ID_GALILEO_E5BIQ  = 76,  ///<  
    MIP_GNSS_SIGNAL_ID_GALILEO_E5ABI  = 77,  ///<  
    MIP_GNSS_SIGNAL_ID_GALILEO_E5ABQ  = 78,  ///<  
    MIP_GNSS_SIGNAL_ID_GALILEO_E5ABIQ = 79,  ///<  
    MIP_GNSS_SIGNAL_ID_GALILEO_E5AI   = 80,  ///<  
    MIP_GNSS_SIGNAL_ID_GALILEO_E5AQ   = 81,  ///<  
    MIP_GNSS_SIGNAL_ID_GALILEO_E5AIQ  = 82,  ///<  
    MIP_GNSS_SIGNAL_ID_SBAS_L1CA      = 96,  ///<  
    MIP_GNSS_SIGNAL_ID_SBAS_L5I       = 97,  ///<  
    MIP_GNSS_SIGNAL_ID_SBAS_L5Q       = 98,  ///<  
    MIP_GNSS_SIGNAL_ID_SBAS_L5IQ      = 99,  ///<  
    MIP_GNSS_SIGNAL_ID_QZSS_L1CA      = 128,  ///<  
    MIP_GNSS_SIGNAL_ID_QZSS_LEXS      = 129,  ///<  
    MIP_GNSS_SIGNAL_ID_QZSS_LEXL      = 130,  ///<  
    MIP_GNSS_SIGNAL_ID_QZSS_LEXSL     = 131,  ///<  
    MIP_GNSS_SIGNAL_ID_QZSS_L2CM      = 132,  ///<  
    MIP_GNSS_SIGNAL_ID_QZSS_L2CL      = 133,  ///<  
    MIP_GNSS_SIGNAL_ID_QZSS_L2CML     = 134,  ///<  
    MIP_GNSS_SIGNAL_ID_QZSS_L5I       = 135,  ///<  
    MIP_GNSS_SIGNAL_ID_QZSS_L5Q       = 136,  ///<  
    MIP_GNSS_SIGNAL_ID_QZSS_L5IQ      = 137,  ///<  
    MIP_GNSS_SIGNAL_ID_QZSS_L1CD      = 138,  ///<  
    MIP_GNSS_SIGNAL_ID_QZSS_L1CP      = 139,  ///<  
    MIP_GNSS_SIGNAL_ID_QZSS_L1CDP     = 140,  ///<  
    MIP_GNSS_SIGNAL_ID_BEIDOU_B1I     = 160,  ///<  
    MIP_GNSS_SIGNAL_ID_BEIDOU_B1Q     = 161,  ///<  
    MIP_GNSS_SIGNAL_ID_BEIDOU_B1IQ    = 162,  ///<  
    MIP_GNSS_SIGNAL_ID_BEIDOU_B3I     = 163,  ///<  
    MIP_GNSS_SIGNAL_ID_BEIDOU_B3Q     = 164,  ///<  
    MIP_GNSS_SIGNAL_ID_BEIDOU_B3IQ    = 165,  ///<  
    MIP_GNSS_SIGNAL_ID_BEIDOU_B2I     = 166,  ///<  
    MIP_GNSS_SIGNAL_ID_BEIDOU_B2Q     = 167,  ///<  
    MIP_GNSS_SIGNAL_ID_BEIDOU_B2IQ    = 168,  ///<  
};
size_t insert_mip_gnss_signal_id(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_gnss_signal_id self);
size_t extract_mip_gnss_signal_id(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_gnss_signal_id* self);

enum mip_sbas_system
{
    MIP_SBAS_SYSTEM_UNKNOWN = 0,  ///<  
    MIP_SBAS_SYSTEM_WAAS    = 1,  ///<  
    MIP_SBAS_SYSTEM_EGNOS   = 2,  ///<  
    MIP_SBAS_SYSTEM_MSAS    = 3,  ///<  
    MIP_SBAS_SYSTEM_GAGAN   = 4,  ///<  
};
size_t insert_mip_sbas_system(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_sbas_system self);
size_t extract_mip_sbas_system(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_sbas_system* self);

#define MIP_GNSS_DGPS_INFO_MAX_CHANNEL_NUMBER_GNSS_DGPS_INFO_MAX_CHANNEL_NUMBER 32
#define MIP_GNSS_SV_INFO_MAX_SV_NUMBER_GNSS_SV_INFO_MAX_SV_NUMBER 32

////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
///@defgroup llh_pos  GNSS LLH Position
/// GNSS reported position in the WGS84 geodetic frame
///
///@{

enum mip_gnss_llh_pos_data_valid_flags
{
    MIP_GNSS_LLH_POS_DATA_VALID_FLAGS_LAT_LON             = 0x01,
    MIP_GNSS_LLH_POS_DATA_VALID_FLAGS_ELLIPSOID_HEIGHT    = 0x02,
    MIP_GNSS_LLH_POS_DATA_VALID_FLAGS_MSL_HEIGHT          = 0x04,
    MIP_GNSS_LLH_POS_DATA_VALID_FLAGS_HORIZONTAL_ACCURACY = 0x08,
    MIP_GNSS_LLH_POS_DATA_VALID_FLAGS_VERTICAL_ACCURACY   = 0x10,
    MIP_GNSS_LLH_POS_DATA_VALID_FLAGS_FLAGS               = 0x1F,
};
size_t insert_mip_gnss_llh_pos_data_valid_flags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_gnss_llh_pos_data_valid_flags self);
size_t extract_mip_gnss_llh_pos_data_valid_flags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_gnss_llh_pos_data_valid_flags* self);

struct mip_gnss_llh_pos_data
{
    double                                            latitude;
    double                                            longitude;
    double                                            ellipsoid_height;
    double                                            msl_height;
    float                                             horizontal_accuracy;
    float                                             vertical_accuracy;
    enum mip_gnss_llh_pos_data_valid_flags            valid_flags;
};
size_t insert_mip_gnss_llh_pos_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_gnss_llh_pos_data* self);
size_t extract_mip_gnss_llh_pos_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_gnss_llh_pos_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup ecef_pos  GNSS ECEF Position
/// GNSS reported position in the Earth-centered, Earth-Fixed (ECEF) frame
///
///@{

enum mip_gnss_ecef_pos_data_valid_flags
{
    MIP_GNSS_ECEF_POS_DATA_VALID_FLAGS_POSITION          = 0x01,
    MIP_GNSS_ECEF_POS_DATA_VALID_FLAGS_POSITION_ACCURACY = 0x02,
    MIP_GNSS_ECEF_POS_DATA_VALID_FLAGS_FLAGS             = 0x03,
};
size_t insert_mip_gnss_ecef_pos_data_valid_flags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_gnss_ecef_pos_data_valid_flags self);
size_t extract_mip_gnss_ecef_pos_data_valid_flags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_gnss_ecef_pos_data_valid_flags* self);

struct mip_gnss_ecef_pos_data
{
    double                                            x[3];
    float                                             x_accuracy;
    enum mip_gnss_ecef_pos_data_valid_flags           valid_flags;
};
size_t insert_mip_gnss_ecef_pos_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_gnss_ecef_pos_data* self);
size_t extract_mip_gnss_ecef_pos_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_gnss_ecef_pos_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup ned_vel  NED Velocity
/// GNSS reported velocity in the NED frame
///
///@{

enum mip_gnss_ned_vel_data_valid_flags
{
    MIP_GNSS_NED_VEL_DATA_VALID_FLAGS_VELOCITY         = 0x01,
    MIP_GNSS_NED_VEL_DATA_VALID_FLAGS_SPEED_3D         = 0x02,
    MIP_GNSS_NED_VEL_DATA_VALID_FLAGS_GROUND_SPEED     = 0x04,
    MIP_GNSS_NED_VEL_DATA_VALID_FLAGS_HEADING          = 0x08,
    MIP_GNSS_NED_VEL_DATA_VALID_FLAGS_SPEED_ACCURACY   = 0x10,
    MIP_GNSS_NED_VEL_DATA_VALID_FLAGS_HEADING_ACCURACY = 0x20,
    MIP_GNSS_NED_VEL_DATA_VALID_FLAGS_FLAGS            = 0x3F,
};
size_t insert_mip_gnss_ned_vel_data_valid_flags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_gnss_ned_vel_data_valid_flags self);
size_t extract_mip_gnss_ned_vel_data_valid_flags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_gnss_ned_vel_data_valid_flags* self);

struct mip_gnss_ned_vel_data
{
    float                                             v[3];
    float                                             speed;
    float                                             ground_speed;
    float                                             heading;
    float                                             speed_accuracy;
    float                                             heading_accuracy;
    enum mip_gnss_ned_vel_data_valid_flags            valid_flags;
};
size_t insert_mip_gnss_ned_vel_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_gnss_ned_vel_data* self);
size_t extract_mip_gnss_ned_vel_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_gnss_ned_vel_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup ecef_vel  GNSS ECEF Velocity
/// GNSS reported velocity in the Earth-centered, Earth-Fixed (ECEF) frame
///
///@{

enum mip_gnss_ecef_vel_data_valid_flags
{
    MIP_GNSS_ECEF_VEL_DATA_VALID_FLAGS_VELOCITY          = 0x01,
    MIP_GNSS_ECEF_VEL_DATA_VALID_FLAGS_VELOCITY_ACCURACY = 0x02,
    MIP_GNSS_ECEF_VEL_DATA_VALID_FLAGS_FLAGS             = 0x03,
};
size_t insert_mip_gnss_ecef_vel_data_valid_flags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_gnss_ecef_vel_data_valid_flags self);
size_t extract_mip_gnss_ecef_vel_data_valid_flags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_gnss_ecef_vel_data_valid_flags* self);

struct mip_gnss_ecef_vel_data
{
    float                                             v[3];
    float                                             v_accuracy;
    enum mip_gnss_ecef_vel_data_valid_flags           valid_flags;
};
size_t insert_mip_gnss_ecef_vel_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_gnss_ecef_vel_data* self);
size_t extract_mip_gnss_ecef_vel_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_gnss_ecef_vel_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup dop  None
/// GNSS reported dilution of precision information.
///
///@{

enum mip_gnss_dop_data_valid_flags
{
    MIP_GNSS_DOP_DATA_VALID_FLAGS_GDOP  = 0x01,
    MIP_GNSS_DOP_DATA_VALID_FLAGS_PDOP  = 0x02,
    MIP_GNSS_DOP_DATA_VALID_FLAGS_HDOP  = 0x04,
    MIP_GNSS_DOP_DATA_VALID_FLAGS_VDOP  = 0x08,
    MIP_GNSS_DOP_DATA_VALID_FLAGS_TDOP  = 0x10,
    MIP_GNSS_DOP_DATA_VALID_FLAGS_NDOP  = 0x20,
    MIP_GNSS_DOP_DATA_VALID_FLAGS_EDOP  = 0x40,
    MIP_GNSS_DOP_DATA_VALID_FLAGS_FLAGS = 0x7F,
};
size_t insert_mip_gnss_dop_data_valid_flags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_gnss_dop_data_valid_flags self);
size_t extract_mip_gnss_dop_data_valid_flags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_gnss_dop_data_valid_flags* self);

struct mip_gnss_dop_data
{
    float                                             gdop;
    float                                             pdop;
    float                                             hdop;
    float                                             vdop;
    float                                             tdop;
    float                                             ndop;
    float                                             edop;
    enum mip_gnss_dop_data_valid_flags                valid_flags;
};
size_t insert_mip_gnss_dop_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_gnss_dop_data* self);
size_t extract_mip_gnss_dop_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_gnss_dop_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup utc_time  None
/// GNSS reported Coordinated Universal Time
///
///@{

enum mip_gnss_utc_time_data_valid_flags
{
    MIP_GNSS_UTC_TIME_DATA_VALID_FLAGS_GNSS_DATE_TIME     = 0x01,
    MIP_GNSS_UTC_TIME_DATA_VALID_FLAGS_LEAP_SECONDS_KNOWN = 0x02,
    MIP_GNSS_UTC_TIME_DATA_VALID_FLAGS_FLAGS              = 0x03,
};
size_t insert_mip_gnss_utc_time_data_valid_flags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_gnss_utc_time_data_valid_flags self);
size_t extract_mip_gnss_utc_time_data_valid_flags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_gnss_utc_time_data_valid_flags* self);

struct mip_gnss_utc_time_data
{
    uint16_t                                          year;
    uint8_t                                           month;
    uint8_t                                           day;
    uint8_t                                           hour;
    uint8_t                                           min;
    uint8_t                                           sec;
    uint32_t                                          msec;
    enum mip_gnss_utc_time_data_valid_flags           valid_flags;
};
size_t insert_mip_gnss_utc_time_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_gnss_utc_time_data* self);
size_t extract_mip_gnss_utc_time_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_gnss_utc_time_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup gps_time  None
/// GNSS reported GPS Time
///
///@{

enum mip_gnss_gps_time_data_valid_flags
{
    MIP_GNSS_GPS_TIME_DATA_VALID_FLAGS_TOW         = 0x01,
    MIP_GNSS_GPS_TIME_DATA_VALID_FLAGS_WEEK_NUMBER = 0x02,
    MIP_GNSS_GPS_TIME_DATA_VALID_FLAGS_FLAGS       = 0x03,
};
size_t insert_mip_gnss_gps_time_data_valid_flags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_gnss_gps_time_data_valid_flags self);
size_t extract_mip_gnss_gps_time_data_valid_flags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_gnss_gps_time_data_valid_flags* self);

struct mip_gnss_gps_time_data
{
    double                                            tow;
    uint16_t                                          week_number;
    enum mip_gnss_gps_time_data_valid_flags           valid_flags;
};
size_t insert_mip_gnss_gps_time_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_gnss_gps_time_data* self);
size_t extract_mip_gnss_gps_time_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_gnss_gps_time_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup clock_info  None
/// GNSS reported receiver clock parameters
///
///@{

enum mip_gnss_clock_info_data_valid_flags
{
    MIP_GNSS_CLOCK_INFO_DATA_VALID_FLAGS_BIAS              = 0x01,
    MIP_GNSS_CLOCK_INFO_DATA_VALID_FLAGS_DRIFT             = 0x02,
    MIP_GNSS_CLOCK_INFO_DATA_VALID_FLAGS_ACCURACY_ESTIMATE = 0x04,
    MIP_GNSS_CLOCK_INFO_DATA_VALID_FLAGS_FLAGS             = 0x07,
};
size_t insert_mip_gnss_clock_info_data_valid_flags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_gnss_clock_info_data_valid_flags self);
size_t extract_mip_gnss_clock_info_data_valid_flags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_gnss_clock_info_data_valid_flags* self);

struct mip_gnss_clock_info_data
{
    double                                            bias;
    double                                            drift;
    double                                            accuracy_estimate;
    enum mip_gnss_clock_info_data_valid_flags         valid_flags;
};
size_t insert_mip_gnss_clock_info_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_gnss_clock_info_data* self);
size_t extract_mip_gnss_clock_info_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_gnss_clock_info_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup fix_info  None
/// GNSS reported position fix type
///
///@{

enum mip_gnss_fix_info_data_fix_type
{
    MIP_GNSS_FIX_INFO_DATA_FIX_TYPE_FIX_3D        = 0,  ///<  
    MIP_GNSS_FIX_INFO_DATA_FIX_TYPE_FIX_2D        = 1,  ///<  
    MIP_GNSS_FIX_INFO_DATA_FIX_TYPE_FIX_TIME_ONLY = 2,  ///<  
    MIP_GNSS_FIX_INFO_DATA_FIX_TYPE_FIX_NONE      = 3,  ///<  
    MIP_GNSS_FIX_INFO_DATA_FIX_TYPE_FIX_INVALID   = 4,  ///<  
    MIP_GNSS_FIX_INFO_DATA_FIX_TYPE_FIX_RTK_FLOAT = 5,  ///<  
    MIP_GNSS_FIX_INFO_DATA_FIX_TYPE_FIX_RTK_FIXED = 6,  ///<  
};
size_t insert_mip_gnss_fix_info_data_fix_type(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_gnss_fix_info_data_fix_type self);
size_t extract_mip_gnss_fix_info_data_fix_type(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_gnss_fix_info_data_fix_type* self);

enum mip_gnss_fix_info_data_fix_flags
{
    MIP_GNSS_FIX_INFO_DATA_FIX_FLAGS_SBAS_USED  = 0x01,
    MIP_GNSS_FIX_INFO_DATA_FIX_FLAGS_DNGSS_USED = 0x02,
};
size_t insert_mip_gnss_fix_info_data_fix_flags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_gnss_fix_info_data_fix_flags self);
size_t extract_mip_gnss_fix_info_data_fix_flags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_gnss_fix_info_data_fix_flags* self);

enum mip_gnss_fix_info_data_valid_flags
{
    MIP_GNSS_FIX_INFO_DATA_VALID_FLAGS_FIX_TYPE  = 0x01,
    MIP_GNSS_FIX_INFO_DATA_VALID_FLAGS_NUM_SV    = 0x02,
    MIP_GNSS_FIX_INFO_DATA_VALID_FLAGS_FIX_FLAGS = 0x04,
    MIP_GNSS_FIX_INFO_DATA_VALID_FLAGS_FLAGS     = 0x07,
};
size_t insert_mip_gnss_fix_info_data_valid_flags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_gnss_fix_info_data_valid_flags self);
size_t extract_mip_gnss_fix_info_data_valid_flags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_gnss_fix_info_data_valid_flags* self);

struct mip_gnss_fix_info_data
{
    enum mip_gnss_fix_info_data_fix_type              fix_type;
    uint8_t                                           num_sv;
    enum mip_gnss_fix_info_data_fix_flags             fix_flags;
    enum mip_gnss_fix_info_data_valid_flags           valid_flags;
};
size_t insert_mip_gnss_fix_info_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_gnss_fix_info_data* self);
size_t extract_mip_gnss_fix_info_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_gnss_fix_info_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup sv_info  None
/// GNSS reported space vehicle information
/// 
/// When enabled, these fields will arrive in separate MIP packets
///
///@{

enum mip_gnss_sv_info_data_svflags
{
    MIP_GNSS_SV_INFO_DATA_SVFLAGS_USED_FOR_NAVIGATION = 0x01,
    MIP_GNSS_SV_INFO_DATA_SVFLAGS_HEALTHY             = 0x02,
};
size_t insert_mip_gnss_sv_info_data_svflags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_gnss_sv_info_data_svflags self);
size_t extract_mip_gnss_sv_info_data_svflags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_gnss_sv_info_data_svflags* self);

enum mip_gnss_sv_info_data_valid_flags
{
    MIP_GNSS_SV_INFO_DATA_VALID_FLAGS_CHANNEL             = 0x01,
    MIP_GNSS_SV_INFO_DATA_VALID_FLAGS_SV_ID               = 0x02,
    MIP_GNSS_SV_INFO_DATA_VALID_FLAGS_CARRIER_NOISE_RATIO = 0x04,
    MIP_GNSS_SV_INFO_DATA_VALID_FLAGS_AZIMUTH             = 0x08,
    MIP_GNSS_SV_INFO_DATA_VALID_FLAGS_ELEVATION           = 0x10,
    MIP_GNSS_SV_INFO_DATA_VALID_FLAGS_SV_FLAGS            = 0x20,
    MIP_GNSS_SV_INFO_DATA_VALID_FLAGS_FLAGS               = 0x3F,
};
size_t insert_mip_gnss_sv_info_data_valid_flags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_gnss_sv_info_data_valid_flags self);
size_t extract_mip_gnss_sv_info_data_valid_flags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_gnss_sv_info_data_valid_flags* self);

struct mip_gnss_sv_info_data
{
    uint8_t                                           channel;
    uint8_t                                           sv_id;
    uint16_t                                          carrier_noise_ratio;
    int16_t                                           azimuth;
    int16_t                                           elevation;
    enum mip_gnss_sv_info_data_svflags                sv_flags;
    enum mip_gnss_sv_info_data_valid_flags            valid_flags;
};
size_t insert_mip_gnss_sv_info_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_gnss_sv_info_data* self);
size_t extract_mip_gnss_sv_info_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_gnss_sv_info_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup hw_status  GNSS Hardware Status
/// GNSS reported hardware status
///
///@{

enum mip_gnss_hw_status_data_receiver_state
{
    MIP_GNSS_HW_STATUS_DATA_RECEIVER_STATE_OFF     = 0,  ///<  
    MIP_GNSS_HW_STATUS_DATA_RECEIVER_STATE_ON      = 1,  ///<  
    MIP_GNSS_HW_STATUS_DATA_RECEIVER_STATE_UNKNOWN = 2,  ///<  
};
size_t insert_mip_gnss_hw_status_data_receiver_state(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_gnss_hw_status_data_receiver_state self);
size_t extract_mip_gnss_hw_status_data_receiver_state(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_gnss_hw_status_data_receiver_state* self);

enum mip_gnss_hw_status_data_antenna_state
{
    MIP_GNSS_HW_STATUS_DATA_ANTENNA_STATE_INIT    = 1,  ///<  
    MIP_GNSS_HW_STATUS_DATA_ANTENNA_STATE_SHORT   = 2,  ///<  
    MIP_GNSS_HW_STATUS_DATA_ANTENNA_STATE_OPEN    = 3,  ///<  
    MIP_GNSS_HW_STATUS_DATA_ANTENNA_STATE_GOOD    = 4,  ///<  
    MIP_GNSS_HW_STATUS_DATA_ANTENNA_STATE_UNKNOWN = 5,  ///<  
};
size_t insert_mip_gnss_hw_status_data_antenna_state(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_gnss_hw_status_data_antenna_state self);
size_t extract_mip_gnss_hw_status_data_antenna_state(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_gnss_hw_status_data_antenna_state* self);

enum mip_gnss_hw_status_data_antenna_power
{
    MIP_GNSS_HW_STATUS_DATA_ANTENNA_POWER_OFF     = 0,  ///<  
    MIP_GNSS_HW_STATUS_DATA_ANTENNA_POWER_ON      = 1,  ///<  
    MIP_GNSS_HW_STATUS_DATA_ANTENNA_POWER_UNKNOWN = 2,  ///<  
};
size_t insert_mip_gnss_hw_status_data_antenna_power(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_gnss_hw_status_data_antenna_power self);
size_t extract_mip_gnss_hw_status_data_antenna_power(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_gnss_hw_status_data_antenna_power* self);

enum mip_gnss_hw_status_data_valid_flags
{
    MIP_GNSS_HW_STATUS_DATA_VALID_FLAGS_SENSOR_STATE  = 0x01,
    MIP_GNSS_HW_STATUS_DATA_VALID_FLAGS_ANTENNA_STATE = 0x02,
    MIP_GNSS_HW_STATUS_DATA_VALID_FLAGS_ANTENNA_POWER = 0x04,
    MIP_GNSS_HW_STATUS_DATA_VALID_FLAGS_FLAGS         = 0x07,
};
size_t insert_mip_gnss_hw_status_data_valid_flags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_gnss_hw_status_data_valid_flags self);
size_t extract_mip_gnss_hw_status_data_valid_flags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_gnss_hw_status_data_valid_flags* self);

struct mip_gnss_hw_status_data
{
    enum mip_gnss_hw_status_data_receiver_state       receiver_state;
    enum mip_gnss_hw_status_data_antenna_state        antenna_state;
    enum mip_gnss_hw_status_data_antenna_power        antenna_power;
    enum mip_gnss_hw_status_data_valid_flags          valid_flags;
};
size_t insert_mip_gnss_hw_status_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_gnss_hw_status_data* self);
size_t extract_mip_gnss_hw_status_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_gnss_hw_status_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup dgps_info  None
/// GNSS reported DGNSS status
/// 
/// Possible Base Station Status Values:
/// 0 – UDRE Scale Factor = 1.0
/// 1 – UDRE Scale Factor = 0.75
/// 2 – UDRE Scale Factor = 0.5
/// 3 – UDRE Scale Factor = 0.3
/// 4 – UDRE Scale Factor = 0.2
/// 5 – UDRE Scale Factor = 0.1
/// 6 – Reference Station Transmission Not Monitored
/// 7 – Reference Station Not Working
/// 
/// (UDRE = User Differential Range Error)
///
///@{

enum mip_gnss_dgps_info_data_valid_flags
{
    MIP_GNSS_DGPS_INFO_DATA_VALID_FLAGS_AGE                 = 0x01,
    MIP_GNSS_DGPS_INFO_DATA_VALID_FLAGS_BASE_STATION_ID     = 0x02,
    MIP_GNSS_DGPS_INFO_DATA_VALID_FLAGS_BASE_STATION_STATUS = 0x04,
    MIP_GNSS_DGPS_INFO_DATA_VALID_FLAGS_NUM_CHANNELS        = 0x08,
    MIP_GNSS_DGPS_INFO_DATA_VALID_FLAGS_FLAGS               = 0x0F,
};
size_t insert_mip_gnss_dgps_info_data_valid_flags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_gnss_dgps_info_data_valid_flags self);
size_t extract_mip_gnss_dgps_info_data_valid_flags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_gnss_dgps_info_data_valid_flags* self);

struct mip_gnss_dgps_info_data
{
    uint8_t                                           sv_id;
    float                                             age;
    float                                             range_correction;
    float                                             range_rate_correction;
    enum mip_gnss_dgps_info_data_valid_flags          valid_flags;
};
size_t insert_mip_gnss_dgps_info_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_gnss_dgps_info_data* self);
size_t extract_mip_gnss_dgps_info_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_gnss_dgps_info_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup dgps_channel  None
/// GNSS reported DGPS Channel Status status
/// 
/// When enabled, a separate field for each active space vehicle will be sent in the packet.
///
///@{

enum mip_gnss_dgps_channel_data_valid_flags
{
    MIP_GNSS_DGPS_CHANNEL_DATA_VALID_FLAGS_ID                    = 0x01,
    MIP_GNSS_DGPS_CHANNEL_DATA_VALID_FLAGS_AGE                   = 0x02,
    MIP_GNSS_DGPS_CHANNEL_DATA_VALID_FLAGS_RANGE_CORRECTION      = 0x04,
    MIP_GNSS_DGPS_CHANNEL_DATA_VALID_FLAGS_RANGE_RATE_CORRECTION = 0x08,
    MIP_GNSS_DGPS_CHANNEL_DATA_VALID_FLAGS_FLAGS                 = 0x0F,
};
size_t insert_mip_gnss_dgps_channel_data_valid_flags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_gnss_dgps_channel_data_valid_flags self);
size_t extract_mip_gnss_dgps_channel_data_valid_flags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_gnss_dgps_channel_data_valid_flags* self);

struct mip_gnss_dgps_channel_data
{
    uint8_t                                           sv_id;
    float                                             age;
    float                                             range_correction;
    float                                             range_rate_correction;
    enum mip_gnss_dgps_channel_data_valid_flags       valid_flags;
};
size_t insert_mip_gnss_dgps_channel_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_gnss_dgps_channel_data* self);
size_t extract_mip_gnss_dgps_channel_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_gnss_dgps_channel_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup clock_info_2  None
/// GNSS reported receiver clock parameters
/// 
/// This supersedes MIP_DATA_DESC_GNSS_CLOCK_INFO with additional information.
///
///@{

enum mip_gnss_clock_info_2_data_valid_flags
{
    MIP_GNSS_CLOCK_INFO_2_DATA_VALID_FLAGS_BIAS           = 0x01,
    MIP_GNSS_CLOCK_INFO_2_DATA_VALID_FLAGS_DRIFT          = 0x02,
    MIP_GNSS_CLOCK_INFO_2_DATA_VALID_FLAGS_BIAS_ACCURACY  = 0x04,
    MIP_GNSS_CLOCK_INFO_2_DATA_VALID_FLAGS_DRIFT_ACCURACY = 0x08,
    MIP_GNSS_CLOCK_INFO_2_DATA_VALID_FLAGS_FLAGS          = 0x0F,
};
size_t insert_mip_gnss_clock_info_2_data_valid_flags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_gnss_clock_info_2_data_valid_flags self);
size_t extract_mip_gnss_clock_info_2_data_valid_flags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_gnss_clock_info_2_data_valid_flags* self);

struct mip_gnss_clock_info_2_data
{
    double                                            bias;
    double                                            drift;
    double                                            bias_accuracy_estimate;
    double                                            drift_accuracy_estimate;
    enum mip_gnss_clock_info_2_data_valid_flags       valid_flags;
};
size_t insert_mip_gnss_clock_info_2_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_gnss_clock_info_2_data* self);
size_t extract_mip_gnss_clock_info_2_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_gnss_clock_info_2_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup gps_leap_seconds  None
/// GNSS reported leap seconds (difference between GPS and UTC Time)
///
///@{

enum mip_gnss_gps_leap_seconds_data_valid_flags
{
    MIP_GNSS_GPS_LEAP_SECONDS_DATA_VALID_FLAGS_LEAP_SECONDS = 0x02,
};
size_t insert_mip_gnss_gps_leap_seconds_data_valid_flags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_gnss_gps_leap_seconds_data_valid_flags self);
size_t extract_mip_gnss_gps_leap_seconds_data_valid_flags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_gnss_gps_leap_seconds_data_valid_flags* self);

struct mip_gnss_gps_leap_seconds_data
{
    uint8_t                                           leap_seconds;
    enum mip_gnss_gps_leap_seconds_data_valid_flags   valid_flags;
};
size_t insert_mip_gnss_gps_leap_seconds_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_gnss_gps_leap_seconds_data* self);
size_t extract_mip_gnss_gps_leap_seconds_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_gnss_gps_leap_seconds_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup sbas_info  None
/// GNSS SBAS status
///
///@{

enum mip_gnss_sbas_info_data_sbas_status
{
    MIP_GNSS_SBAS_INFO_DATA_SBAS_STATUS_RANGE_AVAILABLE       = 0x01,
    MIP_GNSS_SBAS_INFO_DATA_SBAS_STATUS_CORRECTIONS_AVAILABLE = 0x02,
    MIP_GNSS_SBAS_INFO_DATA_SBAS_STATUS_INTEGRITY_AVAILABLE   = 0x04,
    MIP_GNSS_SBAS_INFO_DATA_SBAS_STATUS_TEST_MODE             = 0x08,
};
size_t insert_mip_gnss_sbas_info_data_sbas_status(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_gnss_sbas_info_data_sbas_status self);
size_t extract_mip_gnss_sbas_info_data_sbas_status(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_gnss_sbas_info_data_sbas_status* self);

enum mip_gnss_sbas_info_data_valid_flags
{
    MIP_GNSS_SBAS_INFO_DATA_VALID_FLAGS_TOW         = 0x01,
    MIP_GNSS_SBAS_INFO_DATA_VALID_FLAGS_WEEK_NUMBER = 0x02,
    MIP_GNSS_SBAS_INFO_DATA_VALID_FLAGS_SBAS_SYSTEM = 0x04,
    MIP_GNSS_SBAS_INFO_DATA_VALID_FLAGS_SBAS_ID     = 0x08,
    MIP_GNSS_SBAS_INFO_DATA_VALID_FLAGS_COUNT       = 0x10,
    MIP_GNSS_SBAS_INFO_DATA_VALID_FLAGS_SBAS_STATUS = 0x20,
    MIP_GNSS_SBAS_INFO_DATA_VALID_FLAGS_FLAGS       = 0x3F,
};
size_t insert_mip_gnss_sbas_info_data_valid_flags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_gnss_sbas_info_data_valid_flags self);
size_t extract_mip_gnss_sbas_info_data_valid_flags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_gnss_sbas_info_data_valid_flags* self);

struct mip_gnss_sbas_info_data
{
    double                                            time_of_week;
    uint16_t                                          week_number;
    enum mip_sbas_system                              sbas_system;
    uint8_t                                           sbas_id;
    uint8_t                                           count;
    enum mip_gnss_sbas_info_data_sbas_status          sbas_status;
    enum mip_gnss_sbas_info_data_valid_flags          valid_flags;
};
size_t insert_mip_gnss_sbas_info_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_gnss_sbas_info_data* self);
size_t extract_mip_gnss_sbas_info_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_gnss_sbas_info_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup sbas_correction  None
/// GNSS calculated SBAS Correction
/// 
/// UDREI - the variance of a normal distribution associated with the user differential range errors for a
/// satellite after application of fast and long-term corrections, excluding atmospheric effects
/// 
/// UDREI  Variance
/// -----------------------
/// 0      0.0520 m^2
/// 1      0.0924 m^2
/// 2      0.1444 m^2
/// 3      0.2830 m^2
/// 4      0.4678 m^2
/// 5      0.8315 m^2
/// 6      1.2992 m^2
/// 7      1.8709 m^2
/// 8      2.5465 m^2
/// 9      3.3260 m^2
/// 10     5.1968 m^2
/// 11     20.7870 m^2
/// 12     230.9661 m^2
/// 13     2078.695 m^2
/// 14     "Not Monitored"
/// 15     "Do Not Use"
///
///@{

enum mip_gnss_sbas_correction_data_valid_flags
{
    MIP_GNSS_SBAS_CORRECTION_DATA_VALID_FLAGS_UDREI                  = 0x01,
    MIP_GNSS_SBAS_CORRECTION_DATA_VALID_FLAGS_PSEUDORANGE_CORRECTION = 0x02,
    MIP_GNSS_SBAS_CORRECTION_DATA_VALID_FLAGS_IONO_CORRECTION        = 0x04,
    MIP_GNSS_SBAS_CORRECTION_DATA_VALID_FLAGS_FLAGS                  = 0x07,
};
size_t insert_mip_gnss_sbas_correction_data_valid_flags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_gnss_sbas_correction_data_valid_flags self);
size_t extract_mip_gnss_sbas_correction_data_valid_flags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_gnss_sbas_correction_data_valid_flags* self);

struct mip_gnss_sbas_correction_data
{
    uint8_t                                           index;
    uint8_t                                           count;
    double                                            time_of_week;
    uint16_t                                          week_number;
    enum mip_gnss_constellation_id                    gnss_id;
    uint8_t                                           sv_id;
    uint8_t                                           udrei;
    float                                             pseudorange_correction;
    float                                             iono_correction;
    enum mip_gnss_sbas_correction_data_valid_flags    valid_flags;
};
size_t insert_mip_gnss_sbas_correction_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_gnss_sbas_correction_data* self);
size_t extract_mip_gnss_sbas_correction_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_gnss_sbas_correction_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup rf_error_detection  None
/// GNSS Error Detection subsystem status
///
///@{

enum mip_gnss_rf_error_detection_data_rfband
{
    MIP_GNSS_RF_ERROR_DETECTION_DATA_RFBAND_UNKNOWN = 0,  ///<  
    MIP_GNSS_RF_ERROR_DETECTION_DATA_RFBAND_L1      = 1,  ///<  
    MIP_GNSS_RF_ERROR_DETECTION_DATA_RFBAND_L2      = 2,  ///<  
    MIP_GNSS_RF_ERROR_DETECTION_DATA_RFBAND_L5      = 5,  ///<  
};
size_t insert_mip_gnss_rf_error_detection_data_rfband(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_gnss_rf_error_detection_data_rfband self);
size_t extract_mip_gnss_rf_error_detection_data_rfband(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_gnss_rf_error_detection_data_rfband* self);

enum mip_gnss_rf_error_detection_data_jamming_state
{
    MIP_GNSS_RF_ERROR_DETECTION_DATA_JAMMING_STATE_UNKNOWN     = 0,  ///<  
    MIP_GNSS_RF_ERROR_DETECTION_DATA_JAMMING_STATE_NONE        = 1,  ///<  
    MIP_GNSS_RF_ERROR_DETECTION_DATA_JAMMING_STATE_PARTIAL     = 2,  ///<  
    MIP_GNSS_RF_ERROR_DETECTION_DATA_JAMMING_STATE_SIGNIFICANT = 3,  ///<  
};
size_t insert_mip_gnss_rf_error_detection_data_jamming_state(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_gnss_rf_error_detection_data_jamming_state self);
size_t extract_mip_gnss_rf_error_detection_data_jamming_state(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_gnss_rf_error_detection_data_jamming_state* self);

enum mip_gnss_rf_error_detection_data_spoofing_state
{
    MIP_GNSS_RF_ERROR_DETECTION_DATA_SPOOFING_STATE_UNKNOWN     = 0,  ///<  
    MIP_GNSS_RF_ERROR_DETECTION_DATA_SPOOFING_STATE_NONE        = 1,  ///<  
    MIP_GNSS_RF_ERROR_DETECTION_DATA_SPOOFING_STATE_PARTIAL     = 2,  ///<  
    MIP_GNSS_RF_ERROR_DETECTION_DATA_SPOOFING_STATE_SIGNIFICANT = 3,  ///<  
};
size_t insert_mip_gnss_rf_error_detection_data_spoofing_state(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_gnss_rf_error_detection_data_spoofing_state self);
size_t extract_mip_gnss_rf_error_detection_data_spoofing_state(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_gnss_rf_error_detection_data_spoofing_state* self);

enum mip_gnss_rf_error_detection_data_valid_flags
{
    MIP_GNSS_RF_ERROR_DETECTION_DATA_VALID_FLAGS_RF_BAND        = 0x01,
    MIP_GNSS_RF_ERROR_DETECTION_DATA_VALID_FLAGS_JAMMING_STATE  = 0x02,
    MIP_GNSS_RF_ERROR_DETECTION_DATA_VALID_FLAGS_SPOOFING_STATE = 0x04,
    MIP_GNSS_RF_ERROR_DETECTION_DATA_VALID_FLAGS_FLAGS          = 0x07,
};
size_t insert_mip_gnss_rf_error_detection_data_valid_flags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_gnss_rf_error_detection_data_valid_flags self);
size_t extract_mip_gnss_rf_error_detection_data_valid_flags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_gnss_rf_error_detection_data_valid_flags* self);

struct mip_gnss_rf_error_detection_data
{
    enum mip_gnss_rf_error_detection_data_rfband      rf_band;
    enum mip_gnss_rf_error_detection_data_jamming_state jamming_state;
    enum mip_gnss_rf_error_detection_data_spoofing_state spoofing_state;
    uint8_t                                           reserved[4];
    enum mip_gnss_rf_error_detection_data_valid_flags valid_flags;
};
size_t insert_mip_gnss_rf_error_detection_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_gnss_rf_error_detection_data* self);
size_t extract_mip_gnss_rf_error_detection_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_gnss_rf_error_detection_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup base_station_info  None
/// RTCM reported base station information (sourced from RTCM Message 1005 or 1006)
/// 
/// Valid Flag Mapping:
///
///@{

enum mip_gnss_base_station_info_data_indicator_flags
{
    MIP_GNSS_BASE_STATION_INFO_DATA_INDICATOR_FLAGS_GPS                = 0x01,
    MIP_GNSS_BASE_STATION_INFO_DATA_INDICATOR_FLAGS_GLONASS            = 0x02,
    MIP_GNSS_BASE_STATION_INFO_DATA_INDICATOR_FLAGS_GALILEO            = 0x04,
    MIP_GNSS_BASE_STATION_INFO_DATA_INDICATOR_FLAGS_BEIDOU             = 0x08,
    MIP_GNSS_BASE_STATION_INFO_DATA_INDICATOR_FLAGS_REF_STATION        = 0x10,
    MIP_GNSS_BASE_STATION_INFO_DATA_INDICATOR_FLAGS_SINGLE_RECEIVER    = 0x20,
    MIP_GNSS_BASE_STATION_INFO_DATA_INDICATOR_FLAGS_QUARTER_CYCLE_BIT1 = 0x40,
    MIP_GNSS_BASE_STATION_INFO_DATA_INDICATOR_FLAGS_QUARTER_CYCLE_BIT2 = 0x80,
    MIP_GNSS_BASE_STATION_INFO_DATA_INDICATOR_FLAGS_QUARTER_CYCLE_BITS = 0xC0,
};
size_t insert_mip_gnss_base_station_info_data_indicator_flags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_gnss_base_station_info_data_indicator_flags self);
size_t extract_mip_gnss_base_station_info_data_indicator_flags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_gnss_base_station_info_data_indicator_flags* self);

enum mip_gnss_base_station_info_data_valid_flags
{
    MIP_GNSS_BASE_STATION_INFO_DATA_VALID_FLAGS_TOW           = 0x01,
    MIP_GNSS_BASE_STATION_INFO_DATA_VALID_FLAGS_WEEK_NUMBER   = 0x02,
    MIP_GNSS_BASE_STATION_INFO_DATA_VALID_FLAGS_ECEF_POSITION = 0x04,
    MIP_GNSS_BASE_STATION_INFO_DATA_VALID_FLAGS_HEIGHT        = 0x08,
    MIP_GNSS_BASE_STATION_INFO_DATA_VALID_FLAGS_STATION_ID    = 0x10,
    MIP_GNSS_BASE_STATION_INFO_DATA_VALID_FLAGS_INDICATORS    = 0x20,
    MIP_GNSS_BASE_STATION_INFO_DATA_VALID_FLAGS_FLAGS         = 0x3F,
};
size_t insert_mip_gnss_base_station_info_data_valid_flags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_gnss_base_station_info_data_valid_flags self);
size_t extract_mip_gnss_base_station_info_data_valid_flags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_gnss_base_station_info_data_valid_flags* self);

struct mip_gnss_base_station_info_data
{
    double                                            time_of_week;
    uint16_t                                          week_number;
    double                                            ecef_pos[3];
    float                                             height;
    uint16_t                                          station_id;
    enum mip_gnss_base_station_info_data_indicator_flags indicators;
    enum mip_gnss_base_station_info_data_valid_flags  valid_flags;
};
size_t insert_mip_gnss_base_station_info_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_gnss_base_station_info_data* self);
size_t extract_mip_gnss_base_station_info_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_gnss_base_station_info_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup rtk_corrections_status  None
///
///@{

enum mip_gnss_rtk_corrections_status_data_epoch_status
{
    MIP_GNSS_RTK_CORRECTIONS_STATUS_DATA_EPOCH_STATUS_ANTENNA_LOCATION_RECEIVED    = 0x01,
    MIP_GNSS_RTK_CORRECTIONS_STATUS_DATA_EPOCH_STATUS_ANTENNA_DESCRIPTION_RECEIVED = 0x02,
    MIP_GNSS_RTK_CORRECTIONS_STATUS_DATA_EPOCH_STATUS_GPS_RECEIVED                 = 0x04,
    MIP_GNSS_RTK_CORRECTIONS_STATUS_DATA_EPOCH_STATUS_GLONASS_RECEIVED             = 0x08,
    MIP_GNSS_RTK_CORRECTIONS_STATUS_DATA_EPOCH_STATUS_GALILEO_RECEIVED             = 0x10,
    MIP_GNSS_RTK_CORRECTIONS_STATUS_DATA_EPOCH_STATUS_BEIDOU_RECEIVED              = 0x20,
    MIP_GNSS_RTK_CORRECTIONS_STATUS_DATA_EPOCH_STATUS_USING_GPS_MSM_MESSAGES       = 0x40,
    MIP_GNSS_RTK_CORRECTIONS_STATUS_DATA_EPOCH_STATUS_USING_GLONASS_MSM_MESSAGES   = 0x80,
    MIP_GNSS_RTK_CORRECTIONS_STATUS_DATA_EPOCH_STATUS_DONGLE_STATUS_READ_FAILED    = 0x100,
};
size_t insert_mip_gnss_rtk_corrections_status_data_epoch_status(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_gnss_rtk_corrections_status_data_epoch_status self);
size_t extract_mip_gnss_rtk_corrections_status_data_epoch_status(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_gnss_rtk_corrections_status_data_epoch_status* self);

enum mip_gnss_rtk_corrections_status_data_valid_flags
{
    MIP_GNSS_RTK_CORRECTIONS_STATUS_DATA_VALID_FLAGS_TOW             = 0x01,
    MIP_GNSS_RTK_CORRECTIONS_STATUS_DATA_VALID_FLAGS_WEEK_NUMBER     = 0x02,
    MIP_GNSS_RTK_CORRECTIONS_STATUS_DATA_VALID_FLAGS_EPOCH_STATUS    = 0x04,
    MIP_GNSS_RTK_CORRECTIONS_STATUS_DATA_VALID_FLAGS_DONGLE_STATUS   = 0x08,
    MIP_GNSS_RTK_CORRECTIONS_STATUS_DATA_VALID_FLAGS_GPS_LATENCY     = 0x10,
    MIP_GNSS_RTK_CORRECTIONS_STATUS_DATA_VALID_FLAGS_GLONASS_LATENCY = 0x20,
    MIP_GNSS_RTK_CORRECTIONS_STATUS_DATA_VALID_FLAGS_GALILEO_LATENCY = 0x40,
    MIP_GNSS_RTK_CORRECTIONS_STATUS_DATA_VALID_FLAGS_BEIDOU_LATENCY  = 0x80,
    MIP_GNSS_RTK_CORRECTIONS_STATUS_DATA_VALID_FLAGS_FLAGS           = 0xFF,
};
size_t insert_mip_gnss_rtk_corrections_status_data_valid_flags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_gnss_rtk_corrections_status_data_valid_flags self);
size_t extract_mip_gnss_rtk_corrections_status_data_valid_flags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_gnss_rtk_corrections_status_data_valid_flags* self);

struct mip_gnss_rtk_corrections_status_data
{
    double                                            time_of_week;
    uint16_t                                          week_number;
    enum mip_gnss_rtk_corrections_status_data_epoch_status epoch_status;
    uint32_t                                          dongle_status;
    float                                             gps_correction_latency;
    float                                             glonass_correction_latency;
    float                                             galileo_correction_latency;
    float                                             beidou_correction_latency;
    uint32_t                                          reserved[4];
    enum mip_gnss_rtk_corrections_status_data_valid_flags valid_flags;
};
size_t insert_mip_gnss_rtk_corrections_status_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_gnss_rtk_corrections_status_data* self);
size_t extract_mip_gnss_rtk_corrections_status_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_gnss_rtk_corrections_status_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup satellite_status  None
/// Status information for a GNSS satellite.
///
///@{

enum mip_gnss_satellite_status_data_valid_flags
{
    MIP_GNSS_SATELLITE_STATUS_DATA_VALID_FLAGS_TOW          = 0x01,
    MIP_GNSS_SATELLITE_STATUS_DATA_VALID_FLAGS_WEEK_NUMBER  = 0x02,
    MIP_GNSS_SATELLITE_STATUS_DATA_VALID_FLAGS_GNSS_ID      = 0x04,
    MIP_GNSS_SATELLITE_STATUS_DATA_VALID_FLAGS_SATELLITE_ID = 0x08,
    MIP_GNSS_SATELLITE_STATUS_DATA_VALID_FLAGS_ELEVATION    = 0x10,
    MIP_GNSS_SATELLITE_STATUS_DATA_VALID_FLAGS_AZIMUTH      = 0x20,
    MIP_GNSS_SATELLITE_STATUS_DATA_VALID_FLAGS_HEALTH       = 0x40,
    MIP_GNSS_SATELLITE_STATUS_DATA_VALID_FLAGS_FLAGS        = 0x7F,
};
size_t insert_mip_gnss_satellite_status_data_valid_flags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_gnss_satellite_status_data_valid_flags self);
size_t extract_mip_gnss_satellite_status_data_valid_flags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_gnss_satellite_status_data_valid_flags* self);

struct mip_gnss_satellite_status_data
{
    uint8_t                                           index;
    uint8_t                                           count;
    double                                            time_of_week;
    uint16_t                                          week_number;
    enum mip_gnss_constellation_id                    gnss_id;
    uint8_t                                           satellite_id;
    float                                             elevation;
    float                                             azimuth;
    bool                                              health;
    enum mip_gnss_satellite_status_data_valid_flags   valid_flags;
};
size_t insert_mip_gnss_satellite_status_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_gnss_satellite_status_data* self);
size_t extract_mip_gnss_satellite_status_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_gnss_satellite_status_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup raw  None
/// GNSS Raw observation.
///
///@{

enum mip_gnss_raw_data_gnss_signal_quality
{
    MIP_GNSS_RAW_DATA_GNSS_SIGNAL_QUALITY_NONE         = 0,  ///<  
    MIP_GNSS_RAW_DATA_GNSS_SIGNAL_QUALITY_SEARCHING    = 1,  ///<  
    MIP_GNSS_RAW_DATA_GNSS_SIGNAL_QUALITY_ACQUIRED     = 2,  ///<  
    MIP_GNSS_RAW_DATA_GNSS_SIGNAL_QUALITY_UNUSABLE     = 3,  ///<  
    MIP_GNSS_RAW_DATA_GNSS_SIGNAL_QUALITY_TIME_LOCKED  = 4,  ///<  
    MIP_GNSS_RAW_DATA_GNSS_SIGNAL_QUALITY_FULLY_LOCKED = 5,  ///<  
};
size_t insert_mip_gnss_raw_data_gnss_signal_quality(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_gnss_raw_data_gnss_signal_quality self);
size_t extract_mip_gnss_raw_data_gnss_signal_quality(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_gnss_raw_data_gnss_signal_quality* self);

enum mip_gnss_raw_data_valid_flags
{
    MIP_GNSS_RAW_DATA_VALID_FLAGS_TOW                       = 0x01,
    MIP_GNSS_RAW_DATA_VALID_FLAGS_WEEK_NUMBER               = 0x02,
    MIP_GNSS_RAW_DATA_VALID_FLAGS_RECEIVER_ID               = 0x04,
    MIP_GNSS_RAW_DATA_VALID_FLAGS_TRACKING_CHANNEL          = 0x08,
    MIP_GNSS_RAW_DATA_VALID_FLAGS_GNSS_ID                   = 0x10,
    MIP_GNSS_RAW_DATA_VALID_FLAGS_SATELLITE_ID              = 0x20,
    MIP_GNSS_RAW_DATA_VALID_FLAGS_SIGNAL_ID                 = 0x40,
    MIP_GNSS_RAW_DATA_VALID_FLAGS_SIGNAL_STRENGTH           = 0x80,
    MIP_GNSS_RAW_DATA_VALID_FLAGS_QUALITY                   = 0x100,
    MIP_GNSS_RAW_DATA_VALID_FLAGS_PSEUDORANGE               = 0x200,
    MIP_GNSS_RAW_DATA_VALID_FLAGS_CARRIER_PHASE             = 0x400,
    MIP_GNSS_RAW_DATA_VALID_FLAGS_DOPPLER                   = 0x800,
    MIP_GNSS_RAW_DATA_VALID_FLAGS_RANGE_UNCERTAINTY         = 0x1000,
    MIP_GNSS_RAW_DATA_VALID_FLAGS_CARRIER_PHASE_UNCERTAINTY = 0x2000,
    MIP_GNSS_RAW_DATA_VALID_FLAGS_DOPPLER_UNCERTAINTY       = 0x4000,
    MIP_GNSS_RAW_DATA_VALID_FLAGS_LOCK_TIME                 = 0x8000,
    MIP_GNSS_RAW_DATA_VALID_FLAGS_FLAGS                     = 0xFFFF,
};
size_t insert_mip_gnss_raw_data_valid_flags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_gnss_raw_data_valid_flags self);
size_t extract_mip_gnss_raw_data_valid_flags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_gnss_raw_data_valid_flags* self);

struct mip_gnss_raw_data
{
    uint8_t                                           index;
    uint8_t                                           count;
    double                                            time_of_week;
    uint16_t                                          week_number;
    uint16_t                                          receiver_id;
    uint8_t                                           tracking_channel;
    enum mip_gnss_constellation_id                    gnss_id;
    uint8_t                                           satellite_id;
    enum mip_gnss_signal_id                           signal_id;
    float                                             signal_strength;
    enum mip_gnss_raw_data_gnss_signal_quality        quality;
    double                                            pseudorange;
    double                                            carrier_phase;
    float                                             doppler;
    float                                             range_uncert;
    float                                             phase_uncert;
    float                                             doppler_uncert;
    float                                             lock_time;
    enum mip_gnss_raw_data_valid_flags                valid_flags;
};
size_t insert_mip_gnss_raw_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_gnss_raw_data* self);
size_t extract_mip_gnss_raw_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_gnss_raw_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup gps_ephemeris  None
/// GPS/Galileo Ephemeris Data
///
///@{

enum mip_gnss_gps_ephemeris_data_valid_flags
{
    MIP_GNSS_GPS_EPHEMERIS_DATA_VALID_FLAGS_EPHEMERIS   = 0x01,
    MIP_GNSS_GPS_EPHEMERIS_DATA_VALID_FLAGS_MODERN_DATA = 0x02,
    MIP_GNSS_GPS_EPHEMERIS_DATA_VALID_FLAGS_FLAGS       = 0x03,
};
size_t insert_mip_gnss_gps_ephemeris_data_valid_flags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_gnss_gps_ephemeris_data_valid_flags self);
size_t extract_mip_gnss_gps_ephemeris_data_valid_flags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_gnss_gps_ephemeris_data_valid_flags* self);

struct mip_gnss_gps_ephemeris_data
{
    uint8_t                                           index;
    uint8_t                                           count;
    double                                            time_of_week;
    uint16_t                                          week_number;
    uint8_t                                           satellite_id;
    uint8_t                                           health;
    uint8_t                                           iodc;
    uint8_t                                           iode;
    double                                            t_oc;
    double                                            af0;
    double                                            af1;
    double                                            af2;
    double                                            t_gd;
    double                                            ISC_L1CA;
    double                                            ISC_L2C;
    double                                            t_oe;
    double                                            a;
    double                                            a_dot;
    double                                            mean_anomaly;
    double                                            delta_mean_motion;
    double                                            delta_mean_motion_dot;
    double                                            eccentricity;
    double                                            argument_of_perigee;
    double                                            omega;
    double                                            omega_dot;
    double                                            inclination;
    double                                            inclination_dot;
    double                                            c_ic;
    double                                            c_is;
    double                                            c_uc;
    double                                            c_us;
    double                                            c_rc;
    double                                            c_rs;
    enum mip_gnss_gps_ephemeris_data_valid_flags      valid_flags;
};
size_t insert_mip_gnss_gps_ephemeris_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_gnss_gps_ephemeris_data* self);
size_t extract_mip_gnss_gps_ephemeris_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_gnss_gps_ephemeris_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup glo_ephemeris  Glonass Ephemeris
/// Glonass Ephemeris Data
///
///@{

enum mip_gnss_glo_ephemeris_data_valid_flags
{
    MIP_GNSS_GLO_EPHEMERIS_DATA_VALID_FLAGS_EPHEMERIS = 0x01,
    MIP_GNSS_GLO_EPHEMERIS_DATA_VALID_FLAGS_FLAGS     = 0x01,
};
size_t insert_mip_gnss_glo_ephemeris_data_valid_flags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_gnss_glo_ephemeris_data_valid_flags self);
size_t extract_mip_gnss_glo_ephemeris_data_valid_flags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_gnss_glo_ephemeris_data_valid_flags* self);

struct mip_gnss_glo_ephemeris_data
{
    uint8_t                                           index;
    uint8_t                                           count;
    double                                            time_of_week;
    uint16_t                                          week_number;
    uint8_t                                           satellite_id;
    int8_t                                            freq_number;
    uint32_t                                          tk;
    uint32_t                                          tb;
    uint8_t                                           sat_type;
    double                                            gamma;
    double                                            tau_n;
    double                                            x[3];
    float                                             v[3];
    float                                             a[3];
    uint8_t                                           health;
    uint8_t                                           P;
    uint8_t                                           NT;
    float                                             delta_tau_n;
    uint8_t                                           Ft;
    uint8_t                                           En;
    uint8_t                                           P1;
    uint8_t                                           P2;
    uint8_t                                           P3;
    uint8_t                                           P4;
    enum mip_gnss_glo_ephemeris_data_valid_flags      valid_flags;
};
size_t insert_mip_gnss_glo_ephemeris_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_gnss_glo_ephemeris_data* self);
size_t extract_mip_gnss_glo_ephemeris_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_gnss_glo_ephemeris_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup gps_iono_corr  GPS Ionospheric Correction
/// Ionospheric Correction Terms for GNSS
///
///@{

enum mip_gnss_gps_iono_corr_data_valid_flags
{
    MIP_GNSS_GPS_IONO_CORR_DATA_VALID_FLAGS_TOW         = 0x01,
    MIP_GNSS_GPS_IONO_CORR_DATA_VALID_FLAGS_WEEK_NUMBER = 0x02,
    MIP_GNSS_GPS_IONO_CORR_DATA_VALID_FLAGS_ALPHA       = 0x04,
    MIP_GNSS_GPS_IONO_CORR_DATA_VALID_FLAGS_BETA        = 0x08,
    MIP_GNSS_GPS_IONO_CORR_DATA_VALID_FLAGS_FLAGS       = 0x0F,
};
size_t insert_mip_gnss_gps_iono_corr_data_valid_flags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_gnss_gps_iono_corr_data_valid_flags self);
size_t extract_mip_gnss_gps_iono_corr_data_valid_flags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_gnss_gps_iono_corr_data_valid_flags* self);

struct mip_gnss_gps_iono_corr_data
{
    double                                            time_of_week;
    uint16_t                                          week_number;
    double                                            alpha[4];
    double                                            beta[4];
    enum mip_gnss_gps_iono_corr_data_valid_flags      valid_flags;
};
size_t insert_mip_gnss_gps_iono_corr_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_gnss_gps_iono_corr_data* self);
size_t extract_mip_gnss_gps_iono_corr_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_gnss_gps_iono_corr_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup galileo_iono_corr  Galileo Ionospheric Correction
/// Ionospheric Correction Terms for Galileo
///
///@{

enum mip_gnss_galileo_iono_corr_data_valid_flags
{
    MIP_GNSS_GALILEO_IONO_CORR_DATA_VALID_FLAGS_TOW               = 0x01,
    MIP_GNSS_GALILEO_IONO_CORR_DATA_VALID_FLAGS_WEEK_NUMBER       = 0x02,
    MIP_GNSS_GALILEO_IONO_CORR_DATA_VALID_FLAGS_ALPHA             = 0x04,
    MIP_GNSS_GALILEO_IONO_CORR_DATA_VALID_FLAGS_DISTURBANCE_FLAGS = 0x08,
    MIP_GNSS_GALILEO_IONO_CORR_DATA_VALID_FLAGS_FLAGS             = 0x0F,
};
size_t insert_mip_gnss_galileo_iono_corr_data_valid_flags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_gnss_galileo_iono_corr_data_valid_flags self);
size_t extract_mip_gnss_galileo_iono_corr_data_valid_flags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_gnss_galileo_iono_corr_data_valid_flags* self);

struct mip_gnss_galileo_iono_corr_data
{
    double                                            time_of_week;
    uint16_t                                          week_number;
    double                                            alpha[3];
    uint8_t                                           disturbance_flags;
    enum mip_gnss_galileo_iono_corr_data_valid_flags  valid_flags;
};
size_t insert_mip_gnss_galileo_iono_corr_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_gnss_galileo_iono_corr_data* self);
size_t extract_mip_gnss_galileo_iono_corr_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_gnss_galileo_iono_corr_data* self);

///@}
///

///@}
///@}
///
////////////////////////////////////////////////////////////////////////////////

#ifdef __cplusplus
} // extern "C"
} // namespace C
namespace GnssData {


struct LlhPos : C::mip_gnss_llh_pos_data
{
    static const uint8_t descriptorSet = MIP_GNSS_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_GNSS_POSITION_LLH;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0) const
    {
        return C::insert_mip_gnss_llh_pos_data(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_gnss_llh_pos_data(buffer, bufferSize, offset, this);
    }
};



struct EcefPos : C::mip_gnss_ecef_pos_data
{
    static const uint8_t descriptorSet = MIP_GNSS_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_GNSS_POSITION_ECEF;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0) const
    {
        return C::insert_mip_gnss_ecef_pos_data(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_gnss_ecef_pos_data(buffer, bufferSize, offset, this);
    }
};



struct NedVel : C::mip_gnss_ned_vel_data
{
    static const uint8_t descriptorSet = MIP_GNSS_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_GNSS_VELOCITY_NED;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0) const
    {
        return C::insert_mip_gnss_ned_vel_data(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_gnss_ned_vel_data(buffer, bufferSize, offset, this);
    }
};



struct EcefVel : C::mip_gnss_ecef_vel_data
{
    static const uint8_t descriptorSet = MIP_GNSS_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_GNSS_VELOCITY_ECEF;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0) const
    {
        return C::insert_mip_gnss_ecef_vel_data(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_gnss_ecef_vel_data(buffer, bufferSize, offset, this);
    }
};



struct Dop : C::mip_gnss_dop_data
{
    static const uint8_t descriptorSet = MIP_GNSS_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_GNSS_DOP;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0) const
    {
        return C::insert_mip_gnss_dop_data(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_gnss_dop_data(buffer, bufferSize, offset, this);
    }
};



struct UtcTime : C::mip_gnss_utc_time_data
{
    static const uint8_t descriptorSet = MIP_GNSS_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_GNSS_UTC_TIME;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0) const
    {
        return C::insert_mip_gnss_utc_time_data(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_gnss_utc_time_data(buffer, bufferSize, offset, this);
    }
};



struct GpsTime : C::mip_gnss_gps_time_data
{
    static const uint8_t descriptorSet = MIP_GNSS_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_GNSS_GPS_TIME;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0) const
    {
        return C::insert_mip_gnss_gps_time_data(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_gnss_gps_time_data(buffer, bufferSize, offset, this);
    }
};



struct ClockInfo : C::mip_gnss_clock_info_data
{
    static const uint8_t descriptorSet = MIP_GNSS_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_GNSS_CLOCK_INFO;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0) const
    {
        return C::insert_mip_gnss_clock_info_data(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_gnss_clock_info_data(buffer, bufferSize, offset, this);
    }
};



struct FixInfo : C::mip_gnss_fix_info_data
{
    static const uint8_t descriptorSet = MIP_GNSS_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_GNSS_FIX_INFO;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0) const
    {
        return C::insert_mip_gnss_fix_info_data(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_gnss_fix_info_data(buffer, bufferSize, offset, this);
    }
};



struct SvInfo : C::mip_gnss_sv_info_data
{
    static const uint8_t descriptorSet = MIP_GNSS_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_GNSS_SV_INFO;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0) const
    {
        return C::insert_mip_gnss_sv_info_data(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_gnss_sv_info_data(buffer, bufferSize, offset, this);
    }
};



struct HwStatus : C::mip_gnss_hw_status_data
{
    static const uint8_t descriptorSet = MIP_GNSS_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_GNSS_HW_STATUS;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0) const
    {
        return C::insert_mip_gnss_hw_status_data(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_gnss_hw_status_data(buffer, bufferSize, offset, this);
    }
};



struct DgpsInfo : C::mip_gnss_dgps_info_data
{
    static const uint8_t descriptorSet = MIP_GNSS_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_GNSS_DGPS_INFO;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0) const
    {
        return C::insert_mip_gnss_dgps_info_data(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_gnss_dgps_info_data(buffer, bufferSize, offset, this);
    }
};



struct DgpsChannel : C::mip_gnss_dgps_channel_data
{
    static const uint8_t descriptorSet = MIP_GNSS_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_GNSS_DGPS_CHANNEL_STATUS;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0) const
    {
        return C::insert_mip_gnss_dgps_channel_data(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_gnss_dgps_channel_data(buffer, bufferSize, offset, this);
    }
};



struct ClockInfo2 : C::mip_gnss_clock_info_2_data
{
    static const uint8_t descriptorSet = MIP_GNSS_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_GNSS_CLOCK_INFO_2;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0) const
    {
        return C::insert_mip_gnss_clock_info_2_data(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_gnss_clock_info_2_data(buffer, bufferSize, offset, this);
    }
};



struct GpsLeapSeconds : C::mip_gnss_gps_leap_seconds_data
{
    static const uint8_t descriptorSet = MIP_GNSS_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_GNSS_GPS_LEAP_SECONDS;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0) const
    {
        return C::insert_mip_gnss_gps_leap_seconds_data(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_gnss_gps_leap_seconds_data(buffer, bufferSize, offset, this);
    }
};



struct SbasInfo : C::mip_gnss_sbas_info_data
{
    static const uint8_t descriptorSet = MIP_GNSS_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_GNSS_SBAS_INFO;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0) const
    {
        return C::insert_mip_gnss_sbas_info_data(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_gnss_sbas_info_data(buffer, bufferSize, offset, this);
    }
};



struct SbasCorrection : C::mip_gnss_sbas_correction_data
{
    static const uint8_t descriptorSet = MIP_GNSS_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_GNSS_SBAS_CORRECTION;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0) const
    {
        return C::insert_mip_gnss_sbas_correction_data(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_gnss_sbas_correction_data(buffer, bufferSize, offset, this);
    }
};



struct RfErrorDetection : C::mip_gnss_rf_error_detection_data
{
    static const uint8_t descriptorSet = MIP_GNSS_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_GNSS_RF_ERROR_DETECTION;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0) const
    {
        return C::insert_mip_gnss_rf_error_detection_data(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_gnss_rf_error_detection_data(buffer, bufferSize, offset, this);
    }
};



struct BaseStationInfo : C::mip_gnss_base_station_info_data
{
    static const uint8_t descriptorSet = MIP_GNSS_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_GNSS_BASE_STATION_INFO;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0) const
    {
        return C::insert_mip_gnss_base_station_info_data(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_gnss_base_station_info_data(buffer, bufferSize, offset, this);
    }
};



struct RtkCorrectionsStatus : C::mip_gnss_rtk_corrections_status_data
{
    static const uint8_t descriptorSet = MIP_GNSS_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_GNSS_RTK_CORRECTIONS_STATUS;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0) const
    {
        return C::insert_mip_gnss_rtk_corrections_status_data(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_gnss_rtk_corrections_status_data(buffer, bufferSize, offset, this);
    }
};



struct SatelliteStatus : C::mip_gnss_satellite_status_data
{
    static const uint8_t descriptorSet = MIP_GNSS_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_GNSS_SATELLITE_STATUS;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0) const
    {
        return C::insert_mip_gnss_satellite_status_data(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_gnss_satellite_status_data(buffer, bufferSize, offset, this);
    }
};



struct Raw : C::mip_gnss_raw_data
{
    static const uint8_t descriptorSet = MIP_GNSS_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_GNSS_RAW;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0) const
    {
        return C::insert_mip_gnss_raw_data(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_gnss_raw_data(buffer, bufferSize, offset, this);
    }
};



struct GpsEphemeris : C::mip_gnss_gps_ephemeris_data
{
    static const uint8_t descriptorSet = MIP_GNSS_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_GNSS_GPS_EPHEMERIS;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0) const
    {
        return C::insert_mip_gnss_gps_ephemeris_data(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_gnss_gps_ephemeris_data(buffer, bufferSize, offset, this);
    }
};



struct GloEphemeris : C::mip_gnss_glo_ephemeris_data
{
    static const uint8_t descriptorSet = MIP_GNSS_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_GNSS_GLONASS_EPHEMERIS;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0) const
    {
        return C::insert_mip_gnss_glo_ephemeris_data(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_gnss_glo_ephemeris_data(buffer, bufferSize, offset, this);
    }
};



struct GpsIonoCorr : C::mip_gnss_gps_iono_corr_data
{
    static const uint8_t descriptorSet = MIP_GNSS_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_GNSS_GPS_IONO_CORR;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0) const
    {
        return C::insert_mip_gnss_gps_iono_corr_data(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_gnss_gps_iono_corr_data(buffer, bufferSize, offset, this);
    }
};



struct GalileoIonoCorr : C::mip_gnss_galileo_iono_corr_data
{
    static const uint8_t descriptorSet = MIP_GNSS_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_GNSS_GALILEO_IONO_CORR;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0) const
    {
        return C::insert_mip_gnss_galileo_iono_corr_data(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_gnss_galileo_iono_corr_data(buffer, bufferSize, offset, this);
    }
};



} // namespace GnssData
} // namespace mscl
#endif // __cplusplus
