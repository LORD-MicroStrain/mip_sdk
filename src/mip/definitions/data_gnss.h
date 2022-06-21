#pragma once

#include "descriptors.h"

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
namespace mscl {
extern "C" {
#endif // __cplusplus

////////////////////////////////////////////////////////////////////////////////
///@addtogroup MipData
///@{
///@defgroup GNSS_DATA  GNSS DATA
///
///@{

////////////////////////////////////////////////////////////////////////////////
// Descriptors
////////////////////////////////////////////////////////////////////////////////

enum MipGnssDataDescriptors
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

////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////

#define MIPMIPGNSS1DATADESCSET_MIP_GNSS1_DATA_DESC_SET 0x91
#define MIPMIPGNSS2DATADESCSET_MIP_GNSS2_DATA_DESC_SET 0x92
#define MIPMIPGNSS3DATADESCSET_MIP_GNSS3_DATA_DESC_SET 0x93
#define MIPMIPGNSS4DATADESCSET_MIP_GNSS4_DATA_DESC_SET 0x94
#define MIPMIPGNSS5DATADESCSET_MIP_GNSS5_DATA_DESC_SET 0x95
enum MipGnssConstellationId
{
    MIPGNSSCONSTELLATIONID_UNKNOWN = 0,  ///<  
    MIPGNSSCONSTELLATIONID_GPS     = 1,  ///<  
    MIPGNSSCONSTELLATIONID_GLONASS = 2,  ///<  
    MIPGNSSCONSTELLATIONID_GALILEO = 3,  ///<  
    MIPGNSSCONSTELLATIONID_BEIDOU  = 4,  ///<  
    MIPGNSSCONSTELLATIONID_SBAS    = 5,  ///<  
};
size_t insert_MipGnssConstellationId(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipGnssConstellationId self);
size_t extract_MipGnssConstellationId(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipGnssConstellationId* self);

enum MipGnssSignalId
{
    MIPGNSSSIGNALID_UNKNOWN        = 0,  ///<  
    MIPGNSSSIGNALID_GPS_L1CA       = 1,  ///<  
    MIPGNSSSIGNALID_GPS_L1P        = 2,  ///<  
    MIPGNSSSIGNALID_GPS_L1Z        = 3,  ///<  
    MIPGNSSSIGNALID_GPS_L2CA       = 4,  ///<  
    MIPGNSSSIGNALID_GPS_L2P        = 5,  ///<  
    MIPGNSSSIGNALID_GPS_L2Z        = 6,  ///<  
    MIPGNSSSIGNALID_GPS_L2CL       = 7,  ///<  
    MIPGNSSSIGNALID_GPS_L2CM       = 8,  ///<  
    MIPGNSSSIGNALID_GPS_L2CML      = 9,  ///<  
    MIPGNSSSIGNALID_GPS_L5I        = 10,  ///<  
    MIPGNSSSIGNALID_GPS_L5Q        = 11,  ///<  
    MIPGNSSSIGNALID_GPS_L5IQ       = 12,  ///<  
    MIPGNSSSIGNALID_GPS_L1CD       = 13,  ///<  
    MIPGNSSSIGNALID_GPS_L1CP       = 14,  ///<  
    MIPGNSSSIGNALID_GPS_L1CDP      = 15,  ///<  
    MIPGNSSSIGNALID_GLONASS_G1CA   = 32,  ///<  
    MIPGNSSSIGNALID_GLONASS_G1P    = 33,  ///<  
    MIPGNSSSIGNALID_GLONASS_G2C    = 34,  ///<  
    MIPGNSSSIGNALID_GLONASS_G2P    = 35,  ///<  
    MIPGNSSSIGNALID_GALILEO_E1C    = 64,  ///<  
    MIPGNSSSIGNALID_GALILEO_E1A    = 65,  ///<  
    MIPGNSSSIGNALID_GALILEO_E1B    = 66,  ///<  
    MIPGNSSSIGNALID_GALILEO_E1BC   = 67,  ///<  
    MIPGNSSSIGNALID_GALILEO_E1ABC  = 68,  ///<  
    MIPGNSSSIGNALID_GALILEO_E6C    = 69,  ///<  
    MIPGNSSSIGNALID_GALILEO_E6A    = 70,  ///<  
    MIPGNSSSIGNALID_GALILEO_E6B    = 71,  ///<  
    MIPGNSSSIGNALID_GALILEO_E6BC   = 72,  ///<  
    MIPGNSSSIGNALID_GALILEO_E6ABC  = 73,  ///<  
    MIPGNSSSIGNALID_GALILEO_E5BI   = 74,  ///<  
    MIPGNSSSIGNALID_GALILEO_E5BQ   = 75,  ///<  
    MIPGNSSSIGNALID_GALILEO_E5BIQ  = 76,  ///<  
    MIPGNSSSIGNALID_GALILEO_E5ABI  = 77,  ///<  
    MIPGNSSSIGNALID_GALILEO_E5ABQ  = 78,  ///<  
    MIPGNSSSIGNALID_GALILEO_E5ABIQ = 79,  ///<  
    MIPGNSSSIGNALID_GALILEO_E5AI   = 80,  ///<  
    MIPGNSSSIGNALID_GALILEO_E5AQ   = 81,  ///<  
    MIPGNSSSIGNALID_GALILEO_E5AIQ  = 82,  ///<  
    MIPGNSSSIGNALID_SBAS_L1CA      = 96,  ///<  
    MIPGNSSSIGNALID_SBAS_L5I       = 97,  ///<  
    MIPGNSSSIGNALID_SBAS_L5Q       = 98,  ///<  
    MIPGNSSSIGNALID_SBAS_L5IQ      = 99,  ///<  
    MIPGNSSSIGNALID_QZSS_L1CA      = 128,  ///<  
    MIPGNSSSIGNALID_QZSS_LEXS      = 129,  ///<  
    MIPGNSSSIGNALID_QZSS_LEXL      = 130,  ///<  
    MIPGNSSSIGNALID_QZSS_LEXSL     = 131,  ///<  
    MIPGNSSSIGNALID_QZSS_L2CM      = 132,  ///<  
    MIPGNSSSIGNALID_QZSS_L2CL      = 133,  ///<  
    MIPGNSSSIGNALID_QZSS_L2CML     = 134,  ///<  
    MIPGNSSSIGNALID_QZSS_L5I       = 135,  ///<  
    MIPGNSSSIGNALID_QZSS_L5Q       = 136,  ///<  
    MIPGNSSSIGNALID_QZSS_L5IQ      = 137,  ///<  
    MIPGNSSSIGNALID_QZSS_L1CD      = 138,  ///<  
    MIPGNSSSIGNALID_QZSS_L1CP      = 139,  ///<  
    MIPGNSSSIGNALID_QZSS_L1CDP     = 140,  ///<  
    MIPGNSSSIGNALID_BEIDOU_B1I     = 160,  ///<  
    MIPGNSSSIGNALID_BEIDOU_B1Q     = 161,  ///<  
    MIPGNSSSIGNALID_BEIDOU_B1IQ    = 162,  ///<  
    MIPGNSSSIGNALID_BEIDOU_B3I     = 163,  ///<  
    MIPGNSSSIGNALID_BEIDOU_B3Q     = 164,  ///<  
    MIPGNSSSIGNALID_BEIDOU_B3IQ    = 165,  ///<  
    MIPGNSSSIGNALID_BEIDOU_B2I     = 166,  ///<  
    MIPGNSSSIGNALID_BEIDOU_B2Q     = 167,  ///<  
    MIPGNSSSIGNALID_BEIDOU_B2IQ    = 168,  ///<  
};
size_t insert_MipGnssSignalId(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipGnssSignalId self);
size_t extract_MipGnssSignalId(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipGnssSignalId* self);

enum MipSbasSystem
{
    MIPSBASSYSTEM_UNKNOWN = 0,  ///<  
    MIPSBASSYSTEM_WAAS    = 1,  ///<  
    MIPSBASSYSTEM_EGNOS   = 2,  ///<  
    MIPSBASSYSTEM_MSAS    = 3,  ///<  
    MIPSBASSYSTEM_GAGAN   = 4,  ///<  
};
size_t insert_MipSbasSystem(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipSbasSystem self);
size_t extract_MipSbasSystem(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipSbasSystem* self);

#define MIPGNSSDGPSINFOMAXCHANNELNUMBER_GNSS_DGPS_INFO_MAX_CHANNEL_NUMBER 32
#define MIPGNSSSVINFOMAXSVNUMBER_GNSS_SV_INFO_MAX_SV_NUMBER 32

////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_gnss_llh_pos  Gnss Llh Pos
/// GNSS reported position in the WGS84 geodetic frame
///
///@{

enum MipData_Gnss_LlhPos_Validflags
{
    MIPDATA_GNSS_LLHPOS_VALIDFLAGS_LAT_LON             = 0x01,
    MIPDATA_GNSS_LLHPOS_VALIDFLAGS_ELLIPSOID_HEIGHT    = 0x02,
    MIPDATA_GNSS_LLHPOS_VALIDFLAGS_MSL_HEIGHT          = 0x04,
    MIPDATA_GNSS_LLHPOS_VALIDFLAGS_HORIZONTAL_ACCURACY = 0x08,
    MIPDATA_GNSS_LLHPOS_VALIDFLAGS_VERTICAL_ACCURACY   = 0x10,
    MIPDATA_GNSS_LLHPOS_VALIDFLAGS_FLAGS               = 0x1F,
};
size_t insert_MipData_Gnss_LlhPos_Validflags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Gnss_LlhPos_Validflags self);
size_t extract_MipData_Gnss_LlhPos_Validflags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Gnss_LlhPos_Validflags* self);

struct MipData_Gnss_LlhPos
{
    double                                            latitude;
    double                                            longitude;
    double                                            ellipsoid_height;
    double                                            msl_height;
    float                                             horizontal_accuracy;
    float                                             vertical_accuracy;
    enum MipData_Gnss_LlhPos_Validflags               valid_flags;
};
size_t insert_MipData_Gnss_LlhPos(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Gnss_LlhPos* self);
size_t extract_MipData_Gnss_LlhPos(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Gnss_LlhPos* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_gnss_ecef_pos  Gnss Ecef Pos
/// GNSS reported position in the Earth-centered, Earth-Fixed (ECEF) frame
///
///@{

enum MipData_Gnss_EcefPos_Validflags
{
    MIPDATA_GNSS_ECEFPOS_VALIDFLAGS_POSITION          = 0x01,
    MIPDATA_GNSS_ECEFPOS_VALIDFLAGS_POSITION_ACCURACY = 0x02,
    MIPDATA_GNSS_ECEFPOS_VALIDFLAGS_FLAGS             = 0x03,
};
size_t insert_MipData_Gnss_EcefPos_Validflags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Gnss_EcefPos_Validflags self);
size_t extract_MipData_Gnss_EcefPos_Validflags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Gnss_EcefPos_Validflags* self);

struct MipData_Gnss_EcefPos
{
    double                                            x[3];
    float                                             x_accuracy;
    enum MipData_Gnss_EcefPos_Validflags              valid_flags;
};
size_t insert_MipData_Gnss_EcefPos(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Gnss_EcefPos* self);
size_t extract_MipData_Gnss_EcefPos(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Gnss_EcefPos* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_gnss_ned_vel  Gnss Ned Vel
/// GNSS reported velocity in the NED frame
///
///@{

enum MipData_Gnss_NedVel_Validflags
{
    MIPDATA_GNSS_NEDVEL_VALIDFLAGS_VELOCITY         = 0x01,
    MIPDATA_GNSS_NEDVEL_VALIDFLAGS_SPEED_3D         = 0x02,
    MIPDATA_GNSS_NEDVEL_VALIDFLAGS_GROUND_SPEED     = 0x04,
    MIPDATA_GNSS_NEDVEL_VALIDFLAGS_HEADING          = 0x08,
    MIPDATA_GNSS_NEDVEL_VALIDFLAGS_SPEED_ACCURACY   = 0x10,
    MIPDATA_GNSS_NEDVEL_VALIDFLAGS_HEADING_ACCURACY = 0x20,
    MIPDATA_GNSS_NEDVEL_VALIDFLAGS_FLAGS            = 0x3F,
};
size_t insert_MipData_Gnss_NedVel_Validflags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Gnss_NedVel_Validflags self);
size_t extract_MipData_Gnss_NedVel_Validflags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Gnss_NedVel_Validflags* self);

struct MipData_Gnss_NedVel
{
    float                                             v[3];
    float                                             speed;
    float                                             ground_speed;
    float                                             heading;
    float                                             speed_accuracy;
    float                                             heading_accuracy;
    enum MipData_Gnss_NedVel_Validflags               valid_flags;
};
size_t insert_MipData_Gnss_NedVel(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Gnss_NedVel* self);
size_t extract_MipData_Gnss_NedVel(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Gnss_NedVel* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_gnss_ecef_vel  Gnss Ecef Vel
/// GNSS reported velocity in the Earth-centered, Earth-Fixed (ECEF) frame
///
///@{

enum MipData_Gnss_EcefVel_Validflags
{
    MIPDATA_GNSS_ECEFVEL_VALIDFLAGS_VELOCITY          = 0x01,
    MIPDATA_GNSS_ECEFVEL_VALIDFLAGS_VELOCITY_ACCURACY = 0x02,
    MIPDATA_GNSS_ECEFVEL_VALIDFLAGS_FLAGS             = 0x03,
};
size_t insert_MipData_Gnss_EcefVel_Validflags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Gnss_EcefVel_Validflags self);
size_t extract_MipData_Gnss_EcefVel_Validflags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Gnss_EcefVel_Validflags* self);

struct MipData_Gnss_EcefVel
{
    float                                             v[3];
    float                                             v_accuracy;
    enum MipData_Gnss_EcefVel_Validflags              valid_flags;
};
size_t insert_MipData_Gnss_EcefVel(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Gnss_EcefVel* self);
size_t extract_MipData_Gnss_EcefVel(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Gnss_EcefVel* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_gnss_dop  Gnss Dop
/// GNSS reported dilution of precision information.
///
///@{

enum MipData_Gnss_Dop_Validflags
{
    MIPDATA_GNSS_DOP_VALIDFLAGS_GDOP  = 0x01,
    MIPDATA_GNSS_DOP_VALIDFLAGS_PDOP  = 0x02,
    MIPDATA_GNSS_DOP_VALIDFLAGS_HDOP  = 0x04,
    MIPDATA_GNSS_DOP_VALIDFLAGS_VDOP  = 0x08,
    MIPDATA_GNSS_DOP_VALIDFLAGS_TDOP  = 0x10,
    MIPDATA_GNSS_DOP_VALIDFLAGS_NDOP  = 0x20,
    MIPDATA_GNSS_DOP_VALIDFLAGS_EDOP  = 0x40,
    MIPDATA_GNSS_DOP_VALIDFLAGS_FLAGS = 0x7F,
};
size_t insert_MipData_Gnss_Dop_Validflags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Gnss_Dop_Validflags self);
size_t extract_MipData_Gnss_Dop_Validflags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Gnss_Dop_Validflags* self);

struct MipData_Gnss_Dop
{
    float                                             gdop;
    float                                             pdop;
    float                                             hdop;
    float                                             vdop;
    float                                             tdop;
    float                                             ndop;
    float                                             edop;
    enum MipData_Gnss_Dop_Validflags                  valid_flags;
};
size_t insert_MipData_Gnss_Dop(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Gnss_Dop* self);
size_t extract_MipData_Gnss_Dop(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Gnss_Dop* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_gnss_utc_time  Gnss Utc Time
/// GNSS reported Coordinated Universal Time
///
///@{

enum MipData_Gnss_UtcTime_Validflags
{
    MIPDATA_GNSS_UTCTIME_VALIDFLAGS_GNSS_DATE_TIME     = 0x01,
    MIPDATA_GNSS_UTCTIME_VALIDFLAGS_LEAP_SECONDS_KNOWN = 0x02,
    MIPDATA_GNSS_UTCTIME_VALIDFLAGS_FLAGS              = 0x03,
};
size_t insert_MipData_Gnss_UtcTime_Validflags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Gnss_UtcTime_Validflags self);
size_t extract_MipData_Gnss_UtcTime_Validflags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Gnss_UtcTime_Validflags* self);

struct MipData_Gnss_UtcTime
{
    uint16_t                                          year;
    uint8_t                                           month;
    uint8_t                                           day;
    uint8_t                                           hour;
    uint8_t                                           min;
    uint8_t                                           sec;
    uint32_t                                          msec;
    enum MipData_Gnss_UtcTime_Validflags              valid_flags;
};
size_t insert_MipData_Gnss_UtcTime(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Gnss_UtcTime* self);
size_t extract_MipData_Gnss_UtcTime(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Gnss_UtcTime* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_gnss_gps_time  Gnss Gps Time
/// GNSS reported GPS Time
///
///@{

enum MipData_Gnss_GpsTime_Validflags
{
    MIPDATA_GNSS_GPSTIME_VALIDFLAGS_TOW         = 0x01,
    MIPDATA_GNSS_GPSTIME_VALIDFLAGS_WEEK_NUMBER = 0x02,
    MIPDATA_GNSS_GPSTIME_VALIDFLAGS_FLAGS       = 0x03,
};
size_t insert_MipData_Gnss_GpsTime_Validflags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Gnss_GpsTime_Validflags self);
size_t extract_MipData_Gnss_GpsTime_Validflags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Gnss_GpsTime_Validflags* self);

struct MipData_Gnss_GpsTime
{
    double                                            tow;
    uint16_t                                          week_number;
    enum MipData_Gnss_GpsTime_Validflags              valid_flags;
};
size_t insert_MipData_Gnss_GpsTime(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Gnss_GpsTime* self);
size_t extract_MipData_Gnss_GpsTime(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Gnss_GpsTime* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_gnss_clock_info  Gnss Clock Info
/// GNSS reported receiver clock parameters
///
///@{

enum MipData_Gnss_ClockInfo_Validflags
{
    MIPDATA_GNSS_CLOCKINFO_VALIDFLAGS_BIAS              = 0x01,
    MIPDATA_GNSS_CLOCKINFO_VALIDFLAGS_DRIFT             = 0x02,
    MIPDATA_GNSS_CLOCKINFO_VALIDFLAGS_ACCURACY_ESTIMATE = 0x04,
    MIPDATA_GNSS_CLOCKINFO_VALIDFLAGS_FLAGS             = 0x07,
};
size_t insert_MipData_Gnss_ClockInfo_Validflags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Gnss_ClockInfo_Validflags self);
size_t extract_MipData_Gnss_ClockInfo_Validflags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Gnss_ClockInfo_Validflags* self);

struct MipData_Gnss_ClockInfo
{
    double                                            bias;
    double                                            drift;
    double                                            accuracy_estimate;
    enum MipData_Gnss_ClockInfo_Validflags            valid_flags;
};
size_t insert_MipData_Gnss_ClockInfo(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Gnss_ClockInfo* self);
size_t extract_MipData_Gnss_ClockInfo(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Gnss_ClockInfo* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_gnss_fix_info  Gnss Fix Info
/// GNSS reported position fix type
///
///@{

enum MipData_Gnss_FixInfo_Fixtype
{
    MIPDATA_GNSS_FIXINFO_FIXTYPE_FIX_3D        = 0,  ///<  
    MIPDATA_GNSS_FIXINFO_FIXTYPE_FIX_2D        = 1,  ///<  
    MIPDATA_GNSS_FIXINFO_FIXTYPE_FIX_TIME_ONLY = 2,  ///<  
    MIPDATA_GNSS_FIXINFO_FIXTYPE_FIX_NONE      = 3,  ///<  
    MIPDATA_GNSS_FIXINFO_FIXTYPE_FIX_INVALID   = 4,  ///<  
    MIPDATA_GNSS_FIXINFO_FIXTYPE_FIX_RTK_FLOAT = 5,  ///<  
    MIPDATA_GNSS_FIXINFO_FIXTYPE_FIX_RTK_FIXED = 6,  ///<  
};
size_t insert_MipData_Gnss_FixInfo_Fixtype(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Gnss_FixInfo_Fixtype self);
size_t extract_MipData_Gnss_FixInfo_Fixtype(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Gnss_FixInfo_Fixtype* self);

enum MipData_Gnss_FixInfo_Fixflags
{
    MIPDATA_GNSS_FIXINFO_FIXFLAGS_SBAS_USED  = 0x01,
    MIPDATA_GNSS_FIXINFO_FIXFLAGS_DNGSS_USED = 0x02,
};
size_t insert_MipData_Gnss_FixInfo_Fixflags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Gnss_FixInfo_Fixflags self);
size_t extract_MipData_Gnss_FixInfo_Fixflags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Gnss_FixInfo_Fixflags* self);

enum MipData_Gnss_FixInfo_Validflags
{
    MIPDATA_GNSS_FIXINFO_VALIDFLAGS_FIX_TYPE  = 0x01,
    MIPDATA_GNSS_FIXINFO_VALIDFLAGS_NUM_SV    = 0x02,
    MIPDATA_GNSS_FIXINFO_VALIDFLAGS_FIX_FLAGS = 0x04,
    MIPDATA_GNSS_FIXINFO_VALIDFLAGS_FLAGS     = 0x07,
};
size_t insert_MipData_Gnss_FixInfo_Validflags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Gnss_FixInfo_Validflags self);
size_t extract_MipData_Gnss_FixInfo_Validflags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Gnss_FixInfo_Validflags* self);

struct MipData_Gnss_FixInfo
{
    enum MipData_Gnss_FixInfo_Fixtype                 fix_type;
    uint8_t                                           num_sv;
    enum MipData_Gnss_FixInfo_Fixflags                fix_flags;
    enum MipData_Gnss_FixInfo_Validflags              valid_flags;
};
size_t insert_MipData_Gnss_FixInfo(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Gnss_FixInfo* self);
size_t extract_MipData_Gnss_FixInfo(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Gnss_FixInfo* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_gnss_sv_info  Gnss Sv Info
/// GNSS reported space vehicle information
/// 
/// When enabled, these fields will arrive in separate MIP packets
///
///@{

enum MipData_Gnss_SvInfo_Svflags
{
    MIPDATA_GNSS_SVINFO_SVFLAGS_USED_FOR_NAVIGATION = 0x01,
    MIPDATA_GNSS_SVINFO_SVFLAGS_HEALTHY             = 0x02,
};
size_t insert_MipData_Gnss_SvInfo_Svflags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Gnss_SvInfo_Svflags self);
size_t extract_MipData_Gnss_SvInfo_Svflags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Gnss_SvInfo_Svflags* self);

enum MipData_Gnss_SvInfo_Validflags
{
    MIPDATA_GNSS_SVINFO_VALIDFLAGS_CHANNEL             = 0x01,
    MIPDATA_GNSS_SVINFO_VALIDFLAGS_SV_ID               = 0x02,
    MIPDATA_GNSS_SVINFO_VALIDFLAGS_CARRIER_NOISE_RATIO = 0x04,
    MIPDATA_GNSS_SVINFO_VALIDFLAGS_AZIMUTH             = 0x08,
    MIPDATA_GNSS_SVINFO_VALIDFLAGS_ELEVATION           = 0x10,
    MIPDATA_GNSS_SVINFO_VALIDFLAGS_SV_FLAGS            = 0x20,
    MIPDATA_GNSS_SVINFO_VALIDFLAGS_FLAGS               = 0x3F,
};
size_t insert_MipData_Gnss_SvInfo_Validflags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Gnss_SvInfo_Validflags self);
size_t extract_MipData_Gnss_SvInfo_Validflags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Gnss_SvInfo_Validflags* self);

struct MipData_Gnss_SvInfo
{
    uint8_t                                           channel;
    uint8_t                                           sv_id;
    uint16_t                                          carrier_noise_ratio;
    int16_t                                           azimuth;
    int16_t                                           elevation;
    enum MipData_Gnss_SvInfo_Svflags                  sv_flags;
    enum MipData_Gnss_SvInfo_Validflags               valid_flags;
};
size_t insert_MipData_Gnss_SvInfo(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Gnss_SvInfo* self);
size_t extract_MipData_Gnss_SvInfo(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Gnss_SvInfo* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_gnss_hw_status  Gnss Hw Status
/// GNSS reported hardware status
///
///@{

enum MipData_Gnss_HwStatus_Receiverstate
{
    MIPDATA_GNSS_HWSTATUS_RECEIVERSTATE_OFF     = 0,  ///<  
    MIPDATA_GNSS_HWSTATUS_RECEIVERSTATE_ON      = 1,  ///<  
    MIPDATA_GNSS_HWSTATUS_RECEIVERSTATE_UNKNOWN = 2,  ///<  
};
size_t insert_MipData_Gnss_HwStatus_Receiverstate(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Gnss_HwStatus_Receiverstate self);
size_t extract_MipData_Gnss_HwStatus_Receiverstate(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Gnss_HwStatus_Receiverstate* self);

enum MipData_Gnss_HwStatus_Antennastate
{
    MIPDATA_GNSS_HWSTATUS_ANTENNASTATE_INIT    = 1,  ///<  
    MIPDATA_GNSS_HWSTATUS_ANTENNASTATE_SHORT   = 2,  ///<  
    MIPDATA_GNSS_HWSTATUS_ANTENNASTATE_OPEN    = 3,  ///<  
    MIPDATA_GNSS_HWSTATUS_ANTENNASTATE_GOOD    = 4,  ///<  
    MIPDATA_GNSS_HWSTATUS_ANTENNASTATE_UNKNOWN = 5,  ///<  
};
size_t insert_MipData_Gnss_HwStatus_Antennastate(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Gnss_HwStatus_Antennastate self);
size_t extract_MipData_Gnss_HwStatus_Antennastate(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Gnss_HwStatus_Antennastate* self);

enum MipData_Gnss_HwStatus_Antennapower
{
    MIPDATA_GNSS_HWSTATUS_ANTENNAPOWER_OFF     = 0,  ///<  
    MIPDATA_GNSS_HWSTATUS_ANTENNAPOWER_ON      = 1,  ///<  
    MIPDATA_GNSS_HWSTATUS_ANTENNAPOWER_UNKNOWN = 2,  ///<  
};
size_t insert_MipData_Gnss_HwStatus_Antennapower(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Gnss_HwStatus_Antennapower self);
size_t extract_MipData_Gnss_HwStatus_Antennapower(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Gnss_HwStatus_Antennapower* self);

enum MipData_Gnss_HwStatus_Validflags
{
    MIPDATA_GNSS_HWSTATUS_VALIDFLAGS_SENSOR_STATE  = 0x01,
    MIPDATA_GNSS_HWSTATUS_VALIDFLAGS_ANTENNA_STATE = 0x02,
    MIPDATA_GNSS_HWSTATUS_VALIDFLAGS_ANTENNA_POWER = 0x04,
    MIPDATA_GNSS_HWSTATUS_VALIDFLAGS_FLAGS         = 0x07,
};
size_t insert_MipData_Gnss_HwStatus_Validflags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Gnss_HwStatus_Validflags self);
size_t extract_MipData_Gnss_HwStatus_Validflags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Gnss_HwStatus_Validflags* self);

struct MipData_Gnss_HwStatus
{
    enum MipData_Gnss_HwStatus_Receiverstate          receiver_state;
    enum MipData_Gnss_HwStatus_Antennastate           antenna_state;
    enum MipData_Gnss_HwStatus_Antennapower           antenna_power;
    enum MipData_Gnss_HwStatus_Validflags             valid_flags;
};
size_t insert_MipData_Gnss_HwStatus(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Gnss_HwStatus* self);
size_t extract_MipData_Gnss_HwStatus(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Gnss_HwStatus* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_gnss_dgps_info  Gnss Dgps Info
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

enum MipData_Gnss_DgpsInfo_Validflags
{
    MIPDATA_GNSS_DGPSINFO_VALIDFLAGS_AGE                 = 0x01,
    MIPDATA_GNSS_DGPSINFO_VALIDFLAGS_BASE_STATION_ID     = 0x02,
    MIPDATA_GNSS_DGPSINFO_VALIDFLAGS_BASE_STATION_STATUS = 0x04,
    MIPDATA_GNSS_DGPSINFO_VALIDFLAGS_NUM_CHANNELS        = 0x08,
    MIPDATA_GNSS_DGPSINFO_VALIDFLAGS_FLAGS               = 0x0F,
};
size_t insert_MipData_Gnss_DgpsInfo_Validflags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Gnss_DgpsInfo_Validflags self);
size_t extract_MipData_Gnss_DgpsInfo_Validflags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Gnss_DgpsInfo_Validflags* self);

struct MipData_Gnss_DgpsInfo
{
    uint8_t                                           sv_id;
    float                                             age;
    float                                             range_correction;
    float                                             range_rate_correction;
    enum MipData_Gnss_DgpsInfo_Validflags             valid_flags;
};
size_t insert_MipData_Gnss_DgpsInfo(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Gnss_DgpsInfo* self);
size_t extract_MipData_Gnss_DgpsInfo(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Gnss_DgpsInfo* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_gnss_dgps_channel  Gnss Dgps Channel
/// GNSS reported DGPS Channel Status status
/// 
/// When enabled, a separate field for each active space vehicle will be sent in the packet.
///
///@{

enum MipData_Gnss_DgpsChannel_Validflags
{
    MIPDATA_GNSS_DGPSCHANNEL_VALIDFLAGS_ID                    = 0x01,
    MIPDATA_GNSS_DGPSCHANNEL_VALIDFLAGS_AGE                   = 0x02,
    MIPDATA_GNSS_DGPSCHANNEL_VALIDFLAGS_RANGE_CORRECTION      = 0x04,
    MIPDATA_GNSS_DGPSCHANNEL_VALIDFLAGS_RANGE_RATE_CORRECTION = 0x08,
    MIPDATA_GNSS_DGPSCHANNEL_VALIDFLAGS_FLAGS                 = 0x0F,
};
size_t insert_MipData_Gnss_DgpsChannel_Validflags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Gnss_DgpsChannel_Validflags self);
size_t extract_MipData_Gnss_DgpsChannel_Validflags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Gnss_DgpsChannel_Validflags* self);

struct MipData_Gnss_DgpsChannel
{
    uint8_t                                           sv_id;
    float                                             age;
    float                                             range_correction;
    float                                             range_rate_correction;
    enum MipData_Gnss_DgpsChannel_Validflags          valid_flags;
};
size_t insert_MipData_Gnss_DgpsChannel(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Gnss_DgpsChannel* self);
size_t extract_MipData_Gnss_DgpsChannel(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Gnss_DgpsChannel* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_gnss_clock_info_2  Gnss Clock Info 2
/// GNSS reported receiver clock parameters
/// 
/// This supersedes MIP_DATA_DESC_GNSS_CLOCK_INFO with additional information.
///
///@{

enum MipData_Gnss_ClockInfo2_Validflags
{
    MIPDATA_GNSS_CLOCKINFO2_VALIDFLAGS_BIAS           = 0x01,
    MIPDATA_GNSS_CLOCKINFO2_VALIDFLAGS_DRIFT          = 0x02,
    MIPDATA_GNSS_CLOCKINFO2_VALIDFLAGS_BIAS_ACCURACY  = 0x04,
    MIPDATA_GNSS_CLOCKINFO2_VALIDFLAGS_DRIFT_ACCURACY = 0x08,
    MIPDATA_GNSS_CLOCKINFO2_VALIDFLAGS_FLAGS          = 0x0F,
};
size_t insert_MipData_Gnss_ClockInfo2_Validflags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Gnss_ClockInfo2_Validflags self);
size_t extract_MipData_Gnss_ClockInfo2_Validflags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Gnss_ClockInfo2_Validflags* self);

struct MipData_Gnss_ClockInfo2
{
    double                                            bias;
    double                                            drift;
    double                                            bias_accuracy_estimate;
    double                                            drift_accuracy_estimate;
    enum MipData_Gnss_ClockInfo2_Validflags           valid_flags;
};
size_t insert_MipData_Gnss_ClockInfo2(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Gnss_ClockInfo2* self);
size_t extract_MipData_Gnss_ClockInfo2(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Gnss_ClockInfo2* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_gnss_gps_leap_seconds  Gnss Gps Leap Seconds
/// GNSS reported leap seconds (difference between GPS and UTC Time)
///
///@{

enum MipData_Gnss_GpsLeapSeconds_Validflags
{
    MIPDATA_GNSS_GPSLEAPSECONDS_VALIDFLAGS_LEAP_SECONDS = 0x02,
};
size_t insert_MipData_Gnss_GpsLeapSeconds_Validflags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Gnss_GpsLeapSeconds_Validflags self);
size_t extract_MipData_Gnss_GpsLeapSeconds_Validflags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Gnss_GpsLeapSeconds_Validflags* self);

struct MipData_Gnss_GpsLeapSeconds
{
    uint8_t                                           leap_seconds;
    enum MipData_Gnss_GpsLeapSeconds_Validflags       valid_flags;
};
size_t insert_MipData_Gnss_GpsLeapSeconds(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Gnss_GpsLeapSeconds* self);
size_t extract_MipData_Gnss_GpsLeapSeconds(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Gnss_GpsLeapSeconds* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_gnss_sbas_info  Gnss Sbas Info
/// GNSS SBAS status
///
///@{

enum MipData_Gnss_SbasInfo_Sbasstatus
{
    MIPDATA_GNSS_SBASINFO_SBASSTATUS_RANGE_AVAILABLE       = 0x01,
    MIPDATA_GNSS_SBASINFO_SBASSTATUS_CORRECTIONS_AVAILABLE = 0x02,
    MIPDATA_GNSS_SBASINFO_SBASSTATUS_INTEGRITY_AVAILABLE   = 0x04,
    MIPDATA_GNSS_SBASINFO_SBASSTATUS_TEST_MODE             = 0x08,
};
size_t insert_MipData_Gnss_SbasInfo_Sbasstatus(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Gnss_SbasInfo_Sbasstatus self);
size_t extract_MipData_Gnss_SbasInfo_Sbasstatus(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Gnss_SbasInfo_Sbasstatus* self);

enum MipData_Gnss_SbasInfo_Validflags
{
    MIPDATA_GNSS_SBASINFO_VALIDFLAGS_TOW         = 0x01,
    MIPDATA_GNSS_SBASINFO_VALIDFLAGS_WEEK_NUMBER = 0x02,
    MIPDATA_GNSS_SBASINFO_VALIDFLAGS_SBAS_SYSTEM = 0x04,
    MIPDATA_GNSS_SBASINFO_VALIDFLAGS_SBAS_ID     = 0x08,
    MIPDATA_GNSS_SBASINFO_VALIDFLAGS_COUNT       = 0x10,
    MIPDATA_GNSS_SBASINFO_VALIDFLAGS_SBAS_STATUS = 0x20,
    MIPDATA_GNSS_SBASINFO_VALIDFLAGS_FLAGS       = 0x3F,
};
size_t insert_MipData_Gnss_SbasInfo_Validflags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Gnss_SbasInfo_Validflags self);
size_t extract_MipData_Gnss_SbasInfo_Validflags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Gnss_SbasInfo_Validflags* self);

struct MipData_Gnss_SbasInfo
{
    double                                            time_of_week;
    uint16_t                                          week_number;
    enum MipSbasSystem                                sbas_system;
    uint8_t                                           sbas_id;
    uint8_t                                           count;
    enum MipData_Gnss_SbasInfo_Sbasstatus             sbas_status;
    enum MipData_Gnss_SbasInfo_Validflags             valid_flags;
};
size_t insert_MipData_Gnss_SbasInfo(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Gnss_SbasInfo* self);
size_t extract_MipData_Gnss_SbasInfo(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Gnss_SbasInfo* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_gnss_sbas_correction  Gnss Sbas Correction
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

enum MipData_Gnss_SbasCorrection_Validflags
{
    MIPDATA_GNSS_SBASCORRECTION_VALIDFLAGS_UDREI                  = 0x01,
    MIPDATA_GNSS_SBASCORRECTION_VALIDFLAGS_PSEUDORANGE_CORRECTION = 0x02,
    MIPDATA_GNSS_SBASCORRECTION_VALIDFLAGS_IONO_CORRECTION        = 0x04,
    MIPDATA_GNSS_SBASCORRECTION_VALIDFLAGS_FLAGS                  = 0x07,
};
size_t insert_MipData_Gnss_SbasCorrection_Validflags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Gnss_SbasCorrection_Validflags self);
size_t extract_MipData_Gnss_SbasCorrection_Validflags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Gnss_SbasCorrection_Validflags* self);

struct MipData_Gnss_SbasCorrection
{
    uint8_t                                           index;
    uint8_t                                           count;
    double                                            time_of_week;
    uint16_t                                          week_number;
    enum MipGnssConstellationId                       gnss_id;
    uint8_t                                           sv_id;
    uint8_t                                           udrei;
    float                                             pseudorange_correction;
    float                                             iono_correction;
    enum MipData_Gnss_SbasCorrection_Validflags       valid_flags;
};
size_t insert_MipData_Gnss_SbasCorrection(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Gnss_SbasCorrection* self);
size_t extract_MipData_Gnss_SbasCorrection(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Gnss_SbasCorrection* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_gnss_rf_error_detection  Gnss Rf Error Detection
/// GNSS Error Detection subsystem status
///
///@{

enum MipData_Gnss_RfErrorDetection_Rfband
{
    MIPDATA_GNSS_RFERRORDETECTION_RFBAND_UNKNOWN = 0,  ///<  
    MIPDATA_GNSS_RFERRORDETECTION_RFBAND_L1      = 1,  ///<  
    MIPDATA_GNSS_RFERRORDETECTION_RFBAND_L2      = 2,  ///<  
    MIPDATA_GNSS_RFERRORDETECTION_RFBAND_L5      = 5,  ///<  
};
size_t insert_MipData_Gnss_RfErrorDetection_Rfband(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Gnss_RfErrorDetection_Rfband self);
size_t extract_MipData_Gnss_RfErrorDetection_Rfband(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Gnss_RfErrorDetection_Rfband* self);

enum MipData_Gnss_RfErrorDetection_Jammingstate
{
    MIPDATA_GNSS_RFERRORDETECTION_JAMMINGSTATE_UNKNOWN     = 0,  ///<  
    MIPDATA_GNSS_RFERRORDETECTION_JAMMINGSTATE_NONE        = 1,  ///<  
    MIPDATA_GNSS_RFERRORDETECTION_JAMMINGSTATE_PARTIAL     = 2,  ///<  
    MIPDATA_GNSS_RFERRORDETECTION_JAMMINGSTATE_SIGNIFICANT = 3,  ///<  
};
size_t insert_MipData_Gnss_RfErrorDetection_Jammingstate(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Gnss_RfErrorDetection_Jammingstate self);
size_t extract_MipData_Gnss_RfErrorDetection_Jammingstate(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Gnss_RfErrorDetection_Jammingstate* self);

enum MipData_Gnss_RfErrorDetection_Spoofingstate
{
    MIPDATA_GNSS_RFERRORDETECTION_SPOOFINGSTATE_UNKNOWN     = 0,  ///<  
    MIPDATA_GNSS_RFERRORDETECTION_SPOOFINGSTATE_NONE        = 1,  ///<  
    MIPDATA_GNSS_RFERRORDETECTION_SPOOFINGSTATE_PARTIAL     = 2,  ///<  
    MIPDATA_GNSS_RFERRORDETECTION_SPOOFINGSTATE_SIGNIFICANT = 3,  ///<  
};
size_t insert_MipData_Gnss_RfErrorDetection_Spoofingstate(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Gnss_RfErrorDetection_Spoofingstate self);
size_t extract_MipData_Gnss_RfErrorDetection_Spoofingstate(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Gnss_RfErrorDetection_Spoofingstate* self);

enum MipData_Gnss_RfErrorDetection_Validflags
{
    MIPDATA_GNSS_RFERRORDETECTION_VALIDFLAGS_RF_BAND        = 0x01,
    MIPDATA_GNSS_RFERRORDETECTION_VALIDFLAGS_JAMMING_STATE  = 0x02,
    MIPDATA_GNSS_RFERRORDETECTION_VALIDFLAGS_SPOOFING_STATE = 0x04,
    MIPDATA_GNSS_RFERRORDETECTION_VALIDFLAGS_FLAGS          = 0x07,
};
size_t insert_MipData_Gnss_RfErrorDetection_Validflags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Gnss_RfErrorDetection_Validflags self);
size_t extract_MipData_Gnss_RfErrorDetection_Validflags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Gnss_RfErrorDetection_Validflags* self);

struct MipData_Gnss_RfErrorDetection
{
    enum MipData_Gnss_RfErrorDetection_Rfband         rf_band;
    enum MipData_Gnss_RfErrorDetection_Jammingstate   jamming_state;
    enum MipData_Gnss_RfErrorDetection_Spoofingstate  spoofing_state;
    uint8_t                                           reserved[4];
    enum MipData_Gnss_RfErrorDetection_Validflags     valid_flags;
};
size_t insert_MipData_Gnss_RfErrorDetection(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Gnss_RfErrorDetection* self);
size_t extract_MipData_Gnss_RfErrorDetection(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Gnss_RfErrorDetection* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_gnss_base_station_info  Gnss Base Station Info
/// RTCM reported base station information (sourced from RTCM Message 1005 or 1006)
/// 
/// Valid Flag Mapping:
///
///@{

enum MipData_Gnss_BaseStationInfo_Indicatorflags
{
    MIPDATA_GNSS_BASESTATIONINFO_INDICATORFLAGS_GPS                = 0x01,
    MIPDATA_GNSS_BASESTATIONINFO_INDICATORFLAGS_GLONASS            = 0x02,
    MIPDATA_GNSS_BASESTATIONINFO_INDICATORFLAGS_GALILEO            = 0x04,
    MIPDATA_GNSS_BASESTATIONINFO_INDICATORFLAGS_BEIDOU             = 0x08,
    MIPDATA_GNSS_BASESTATIONINFO_INDICATORFLAGS_REF_STATION        = 0x10,
    MIPDATA_GNSS_BASESTATIONINFO_INDICATORFLAGS_SINGLE_RECEIVER    = 0x20,
    MIPDATA_GNSS_BASESTATIONINFO_INDICATORFLAGS_QUARTER_CYCLE_BIT1 = 0x40,
    MIPDATA_GNSS_BASESTATIONINFO_INDICATORFLAGS_QUARTER_CYCLE_BIT2 = 0x80,
    MIPDATA_GNSS_BASESTATIONINFO_INDICATORFLAGS_QUARTER_CYCLE_BITS = 0xC0,
};
size_t insert_MipData_Gnss_BaseStationInfo_Indicatorflags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Gnss_BaseStationInfo_Indicatorflags self);
size_t extract_MipData_Gnss_BaseStationInfo_Indicatorflags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Gnss_BaseStationInfo_Indicatorflags* self);

enum MipData_Gnss_BaseStationInfo_Validflags
{
    MIPDATA_GNSS_BASESTATIONINFO_VALIDFLAGS_TOW           = 0x01,
    MIPDATA_GNSS_BASESTATIONINFO_VALIDFLAGS_WEEK_NUMBER   = 0x02,
    MIPDATA_GNSS_BASESTATIONINFO_VALIDFLAGS_ECEF_POSITION = 0x04,
    MIPDATA_GNSS_BASESTATIONINFO_VALIDFLAGS_HEIGHT        = 0x08,
    MIPDATA_GNSS_BASESTATIONINFO_VALIDFLAGS_STATION_ID    = 0x10,
    MIPDATA_GNSS_BASESTATIONINFO_VALIDFLAGS_INDICATORS    = 0x20,
    MIPDATA_GNSS_BASESTATIONINFO_VALIDFLAGS_FLAGS         = 0x3F,
};
size_t insert_MipData_Gnss_BaseStationInfo_Validflags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Gnss_BaseStationInfo_Validflags self);
size_t extract_MipData_Gnss_BaseStationInfo_Validflags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Gnss_BaseStationInfo_Validflags* self);

struct MipData_Gnss_BaseStationInfo
{
    double                                            time_of_week;
    uint16_t                                          week_number;
    double                                            ecef_pos[3];
    float                                             height;
    uint16_t                                          station_id;
    enum MipData_Gnss_BaseStationInfo_Indicatorflags  indicators;
    enum MipData_Gnss_BaseStationInfo_Validflags      valid_flags;
};
size_t insert_MipData_Gnss_BaseStationInfo(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Gnss_BaseStationInfo* self);
size_t extract_MipData_Gnss_BaseStationInfo(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Gnss_BaseStationInfo* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_gnss_rtk_corrections_status  Gnss Rtk Corrections Status
///
///@{

enum MipData_Gnss_RtkCorrectionsStatus_Epochstatus
{
    MIPDATA_GNSS_RTKCORRECTIONSSTATUS_EPOCHSTATUS_ANTENNA_LOCATION_RECEIVED    = 0x01,
    MIPDATA_GNSS_RTKCORRECTIONSSTATUS_EPOCHSTATUS_ANTENNA_DESCRIPTION_RECEIVED = 0x02,
    MIPDATA_GNSS_RTKCORRECTIONSSTATUS_EPOCHSTATUS_GPS_RECEIVED                 = 0x04,
    MIPDATA_GNSS_RTKCORRECTIONSSTATUS_EPOCHSTATUS_GLONASS_RECEIVED             = 0x08,
    MIPDATA_GNSS_RTKCORRECTIONSSTATUS_EPOCHSTATUS_GALILEO_RECEIVED             = 0x10,
    MIPDATA_GNSS_RTKCORRECTIONSSTATUS_EPOCHSTATUS_BEIDOU_RECEIVED              = 0x20,
    MIPDATA_GNSS_RTKCORRECTIONSSTATUS_EPOCHSTATUS_USING_GPS_MSM_MESSAGES       = 0x40,
    MIPDATA_GNSS_RTKCORRECTIONSSTATUS_EPOCHSTATUS_USING_GLONASS_MSM_MESSAGES   = 0x80,
    MIPDATA_GNSS_RTKCORRECTIONSSTATUS_EPOCHSTATUS_DONGLE_STATUS_READ_FAILED    = 0x100,
};
size_t insert_MipData_Gnss_RtkCorrectionsStatus_Epochstatus(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Gnss_RtkCorrectionsStatus_Epochstatus self);
size_t extract_MipData_Gnss_RtkCorrectionsStatus_Epochstatus(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Gnss_RtkCorrectionsStatus_Epochstatus* self);

enum MipData_Gnss_RtkCorrectionsStatus_Validflags
{
    MIPDATA_GNSS_RTKCORRECTIONSSTATUS_VALIDFLAGS_TOW             = 0x01,
    MIPDATA_GNSS_RTKCORRECTIONSSTATUS_VALIDFLAGS_WEEK_NUMBER     = 0x02,
    MIPDATA_GNSS_RTKCORRECTIONSSTATUS_VALIDFLAGS_EPOCH_STATUS    = 0x04,
    MIPDATA_GNSS_RTKCORRECTIONSSTATUS_VALIDFLAGS_DONGLE_STATUS   = 0x08,
    MIPDATA_GNSS_RTKCORRECTIONSSTATUS_VALIDFLAGS_GPS_LATENCY     = 0x10,
    MIPDATA_GNSS_RTKCORRECTIONSSTATUS_VALIDFLAGS_GLONASS_LATENCY = 0x20,
    MIPDATA_GNSS_RTKCORRECTIONSSTATUS_VALIDFLAGS_GALILEO_LATENCY = 0x40,
    MIPDATA_GNSS_RTKCORRECTIONSSTATUS_VALIDFLAGS_BEIDOU_LATENCY  = 0x80,
    MIPDATA_GNSS_RTKCORRECTIONSSTATUS_VALIDFLAGS_FLAGS           = 0xFF,
};
size_t insert_MipData_Gnss_RtkCorrectionsStatus_Validflags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Gnss_RtkCorrectionsStatus_Validflags self);
size_t extract_MipData_Gnss_RtkCorrectionsStatus_Validflags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Gnss_RtkCorrectionsStatus_Validflags* self);

struct MipData_Gnss_RtkCorrectionsStatus
{
    double                                            time_of_week;
    uint16_t                                          week_number;
    enum MipData_Gnss_RtkCorrectionsStatus_Epochstatus epoch_status;
    uint32_t                                          dongle_status;
    float                                             gps_correction_latency;
    float                                             glonass_correction_latency;
    float                                             galileo_correction_latency;
    float                                             beidou_correction_latency;
    uint32_t                                          reserved[4];
    enum MipData_Gnss_RtkCorrectionsStatus_Validflags valid_flags;
};
size_t insert_MipData_Gnss_RtkCorrectionsStatus(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Gnss_RtkCorrectionsStatus* self);
size_t extract_MipData_Gnss_RtkCorrectionsStatus(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Gnss_RtkCorrectionsStatus* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_gnss_satellite_status  Gnss Satellite Status
/// Status information for a GNSS satellite.
///
///@{

enum MipData_Gnss_SatelliteStatus_Validflags
{
    MIPDATA_GNSS_SATELLITESTATUS_VALIDFLAGS_TOW          = 0x01,
    MIPDATA_GNSS_SATELLITESTATUS_VALIDFLAGS_WEEK_NUMBER  = 0x02,
    MIPDATA_GNSS_SATELLITESTATUS_VALIDFLAGS_GNSS_ID      = 0x04,
    MIPDATA_GNSS_SATELLITESTATUS_VALIDFLAGS_SATELLITE_ID = 0x08,
    MIPDATA_GNSS_SATELLITESTATUS_VALIDFLAGS_ELEVATION    = 0x10,
    MIPDATA_GNSS_SATELLITESTATUS_VALIDFLAGS_AZIMUTH      = 0x20,
    MIPDATA_GNSS_SATELLITESTATUS_VALIDFLAGS_HEALTH       = 0x40,
    MIPDATA_GNSS_SATELLITESTATUS_VALIDFLAGS_FLAGS        = 0x7F,
};
size_t insert_MipData_Gnss_SatelliteStatus_Validflags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Gnss_SatelliteStatus_Validflags self);
size_t extract_MipData_Gnss_SatelliteStatus_Validflags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Gnss_SatelliteStatus_Validflags* self);

struct MipData_Gnss_SatelliteStatus
{
    uint8_t                                           index;
    uint8_t                                           count;
    double                                            time_of_week;
    uint16_t                                          week_number;
    enum MipGnssConstellationId                       gnss_id;
    uint8_t                                           satellite_id;
    float                                             elevation;
    float                                             azimuth;
    bool                                              health;
    enum MipData_Gnss_SatelliteStatus_Validflags      valid_flags;
};
size_t insert_MipData_Gnss_SatelliteStatus(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Gnss_SatelliteStatus* self);
size_t extract_MipData_Gnss_SatelliteStatus(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Gnss_SatelliteStatus* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_gnss_raw  Gnss Raw
/// GNSS Raw observation.
///
///@{

enum MipData_Gnss_Raw_Gnsssignalquality
{
    MIPDATA_GNSS_RAW_GNSSSIGNALQUALITY_NONE         = 0,  ///<  
    MIPDATA_GNSS_RAW_GNSSSIGNALQUALITY_SEARCHING    = 1,  ///<  
    MIPDATA_GNSS_RAW_GNSSSIGNALQUALITY_ACQUIRED     = 2,  ///<  
    MIPDATA_GNSS_RAW_GNSSSIGNALQUALITY_UNUSABLE     = 3,  ///<  
    MIPDATA_GNSS_RAW_GNSSSIGNALQUALITY_TIME_LOCKED  = 4,  ///<  
    MIPDATA_GNSS_RAW_GNSSSIGNALQUALITY_FULLY_LOCKED = 5,  ///<  
};
size_t insert_MipData_Gnss_Raw_Gnsssignalquality(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Gnss_Raw_Gnsssignalquality self);
size_t extract_MipData_Gnss_Raw_Gnsssignalquality(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Gnss_Raw_Gnsssignalquality* self);

enum MipData_Gnss_Raw_Validflags
{
    MIPDATA_GNSS_RAW_VALIDFLAGS_TOW                       = 0x01,
    MIPDATA_GNSS_RAW_VALIDFLAGS_WEEK_NUMBER               = 0x02,
    MIPDATA_GNSS_RAW_VALIDFLAGS_RECEIVER_ID               = 0x04,
    MIPDATA_GNSS_RAW_VALIDFLAGS_TRACKING_CHANNEL          = 0x08,
    MIPDATA_GNSS_RAW_VALIDFLAGS_GNSS_ID                   = 0x10,
    MIPDATA_GNSS_RAW_VALIDFLAGS_SATELLITE_ID              = 0x20,
    MIPDATA_GNSS_RAW_VALIDFLAGS_SIGNAL_ID                 = 0x40,
    MIPDATA_GNSS_RAW_VALIDFLAGS_SIGNAL_STRENGTH           = 0x80,
    MIPDATA_GNSS_RAW_VALIDFLAGS_QUALITY                   = 0x100,
    MIPDATA_GNSS_RAW_VALIDFLAGS_PSEUDORANGE               = 0x200,
    MIPDATA_GNSS_RAW_VALIDFLAGS_CARRIER_PHASE             = 0x400,
    MIPDATA_GNSS_RAW_VALIDFLAGS_DOPPLER                   = 0x800,
    MIPDATA_GNSS_RAW_VALIDFLAGS_RANGE_UNCERTAINTY         = 0x1000,
    MIPDATA_GNSS_RAW_VALIDFLAGS_CARRIER_PHASE_UNCERTAINTY = 0x2000,
    MIPDATA_GNSS_RAW_VALIDFLAGS_DOPPLER_UNCERTAINTY       = 0x4000,
    MIPDATA_GNSS_RAW_VALIDFLAGS_LOCK_TIME                 = 0x8000,
    MIPDATA_GNSS_RAW_VALIDFLAGS_FLAGS                     = 0xFFFF,
};
size_t insert_MipData_Gnss_Raw_Validflags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Gnss_Raw_Validflags self);
size_t extract_MipData_Gnss_Raw_Validflags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Gnss_Raw_Validflags* self);

struct MipData_Gnss_Raw
{
    uint8_t                                           index;
    uint8_t                                           count;
    double                                            time_of_week;
    uint16_t                                          week_number;
    uint16_t                                          receiver_id;
    uint8_t                                           tracking_channel;
    enum MipGnssConstellationId                       gnss_id;
    uint8_t                                           satellite_id;
    enum MipGnssSignalId                              signal_id;
    float                                             signal_strength;
    enum MipData_Gnss_Raw_Gnsssignalquality           quality;
    double                                            pseudorange;
    double                                            carrier_phase;
    float                                             doppler;
    float                                             range_uncert;
    float                                             phase_uncert;
    float                                             doppler_uncert;
    float                                             lock_time;
    enum MipData_Gnss_Raw_Validflags                  valid_flags;
};
size_t insert_MipData_Gnss_Raw(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Gnss_Raw* self);
size_t extract_MipData_Gnss_Raw(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Gnss_Raw* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_gnss_gps_ephemeris  Gnss Gps Ephemeris
/// GPS/Galileo Ephemeris Data
///
///@{

enum MipData_Gnss_GpsEphemeris_Validflags
{
    MIPDATA_GNSS_GPSEPHEMERIS_VALIDFLAGS_EPHEMERIS   = 0x01,
    MIPDATA_GNSS_GPSEPHEMERIS_VALIDFLAGS_MODERN_DATA = 0x02,
    MIPDATA_GNSS_GPSEPHEMERIS_VALIDFLAGS_FLAGS       = 0x03,
};
size_t insert_MipData_Gnss_GpsEphemeris_Validflags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Gnss_GpsEphemeris_Validflags self);
size_t extract_MipData_Gnss_GpsEphemeris_Validflags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Gnss_GpsEphemeris_Validflags* self);

struct MipData_Gnss_GpsEphemeris
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
    enum MipData_Gnss_GpsEphemeris_Validflags         valid_flags;
};
size_t insert_MipData_Gnss_GpsEphemeris(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Gnss_GpsEphemeris* self);
size_t extract_MipData_Gnss_GpsEphemeris(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Gnss_GpsEphemeris* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_gnss_glo_ephemeris  Gnss Glo Ephemeris
/// Glonass Ephemeris Data
///
///@{

enum MipData_Gnss_GloEphemeris_Validflags
{
    MIPDATA_GNSS_GLOEPHEMERIS_VALIDFLAGS_EPHEMERIS = 0x01,
    MIPDATA_GNSS_GLOEPHEMERIS_VALIDFLAGS_FLAGS     = 0x01,
};
size_t insert_MipData_Gnss_GloEphemeris_Validflags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Gnss_GloEphemeris_Validflags self);
size_t extract_MipData_Gnss_GloEphemeris_Validflags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Gnss_GloEphemeris_Validflags* self);

struct MipData_Gnss_GloEphemeris
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
    enum MipData_Gnss_GloEphemeris_Validflags         valid_flags;
};
size_t insert_MipData_Gnss_GloEphemeris(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Gnss_GloEphemeris* self);
size_t extract_MipData_Gnss_GloEphemeris(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Gnss_GloEphemeris* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_gnss_gps_iono_corr  Gnss Gps Iono Corr
/// Ionospheric Correction Terms for GNSS
///
///@{

enum MipData_Gnss_GpsIonoCorr_Validflags
{
    MIPDATA_GNSS_GPSIONOCORR_VALIDFLAGS_TOW         = 0x01,
    MIPDATA_GNSS_GPSIONOCORR_VALIDFLAGS_WEEK_NUMBER = 0x02,
    MIPDATA_GNSS_GPSIONOCORR_VALIDFLAGS_ALPHA       = 0x04,
    MIPDATA_GNSS_GPSIONOCORR_VALIDFLAGS_BETA        = 0x08,
    MIPDATA_GNSS_GPSIONOCORR_VALIDFLAGS_FLAGS       = 0x0F,
};
size_t insert_MipData_Gnss_GpsIonoCorr_Validflags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Gnss_GpsIonoCorr_Validflags self);
size_t extract_MipData_Gnss_GpsIonoCorr_Validflags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Gnss_GpsIonoCorr_Validflags* self);

struct MipData_Gnss_GpsIonoCorr
{
    double                                            time_of_week;
    uint16_t                                          week_number;
    double                                            alpha[4];
    double                                            beta[4];
    enum MipData_Gnss_GpsIonoCorr_Validflags          valid_flags;
};
size_t insert_MipData_Gnss_GpsIonoCorr(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Gnss_GpsIonoCorr* self);
size_t extract_MipData_Gnss_GpsIonoCorr(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Gnss_GpsIonoCorr* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_gnss_galileo_iono_corr  Gnss Galileo Iono Corr
/// Ionospheric Correction Terms for Galileo
///
///@{

enum MipData_Gnss_GalileoIonoCorr_Validflags
{
    MIPDATA_GNSS_GALILEOIONOCORR_VALIDFLAGS_TOW               = 0x01,
    MIPDATA_GNSS_GALILEOIONOCORR_VALIDFLAGS_WEEK_NUMBER       = 0x02,
    MIPDATA_GNSS_GALILEOIONOCORR_VALIDFLAGS_ALPHA             = 0x04,
    MIPDATA_GNSS_GALILEOIONOCORR_VALIDFLAGS_DISTURBANCE_FLAGS = 0x08,
    MIPDATA_GNSS_GALILEOIONOCORR_VALIDFLAGS_FLAGS             = 0x0F,
};
size_t insert_MipData_Gnss_GalileoIonoCorr_Validflags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Gnss_GalileoIonoCorr_Validflags self);
size_t extract_MipData_Gnss_GalileoIonoCorr_Validflags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Gnss_GalileoIonoCorr_Validflags* self);

struct MipData_Gnss_GalileoIonoCorr
{
    double                                            time_of_week;
    uint16_t                                          week_number;
    double                                            alpha[3];
    uint8_t                                           disturbance_flags;
    enum MipData_Gnss_GalileoIonoCorr_Validflags      valid_flags;
};
size_t insert_MipData_Gnss_GalileoIonoCorr(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Gnss_GalileoIonoCorr* self);
size_t extract_MipData_Gnss_GalileoIonoCorr(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Gnss_GalileoIonoCorr* self);

///@}
///

///@}
///@}
///
////////////////////////////////////////////////////////////////////////////////

#ifdef __cplusplus
} // extern "C"


template<>
struct MipFieldInfo<MipData_Gnss_LlhPos>
{
    static const uint8_t descriptorSet = MIP_GNSS_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_GNSS_POSITION_LLH;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Gnss_LlhPos& self)
    {
        return insert_MipData_Gnss_LlhPos(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Gnss_LlhPos& self)
    {
        return extract_MipData_Gnss_LlhPos(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Gnss_EcefPos>
{
    static const uint8_t descriptorSet = MIP_GNSS_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_GNSS_POSITION_ECEF;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Gnss_EcefPos& self)
    {
        return insert_MipData_Gnss_EcefPos(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Gnss_EcefPos& self)
    {
        return extract_MipData_Gnss_EcefPos(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Gnss_NedVel>
{
    static const uint8_t descriptorSet = MIP_GNSS_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_GNSS_VELOCITY_NED;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Gnss_NedVel& self)
    {
        return insert_MipData_Gnss_NedVel(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Gnss_NedVel& self)
    {
        return extract_MipData_Gnss_NedVel(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Gnss_EcefVel>
{
    static const uint8_t descriptorSet = MIP_GNSS_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_GNSS_VELOCITY_ECEF;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Gnss_EcefVel& self)
    {
        return insert_MipData_Gnss_EcefVel(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Gnss_EcefVel& self)
    {
        return extract_MipData_Gnss_EcefVel(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Gnss_Dop>
{
    static const uint8_t descriptorSet = MIP_GNSS_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_GNSS_DOP;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Gnss_Dop& self)
    {
        return insert_MipData_Gnss_Dop(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Gnss_Dop& self)
    {
        return extract_MipData_Gnss_Dop(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Gnss_UtcTime>
{
    static const uint8_t descriptorSet = MIP_GNSS_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_GNSS_UTC_TIME;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Gnss_UtcTime& self)
    {
        return insert_MipData_Gnss_UtcTime(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Gnss_UtcTime& self)
    {
        return extract_MipData_Gnss_UtcTime(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Gnss_GpsTime>
{
    static const uint8_t descriptorSet = MIP_GNSS_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_GNSS_GPS_TIME;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Gnss_GpsTime& self)
    {
        return insert_MipData_Gnss_GpsTime(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Gnss_GpsTime& self)
    {
        return extract_MipData_Gnss_GpsTime(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Gnss_ClockInfo>
{
    static const uint8_t descriptorSet = MIP_GNSS_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_GNSS_CLOCK_INFO;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Gnss_ClockInfo& self)
    {
        return insert_MipData_Gnss_ClockInfo(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Gnss_ClockInfo& self)
    {
        return extract_MipData_Gnss_ClockInfo(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Gnss_FixInfo>
{
    static const uint8_t descriptorSet = MIP_GNSS_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_GNSS_FIX_INFO;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Gnss_FixInfo& self)
    {
        return insert_MipData_Gnss_FixInfo(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Gnss_FixInfo& self)
    {
        return extract_MipData_Gnss_FixInfo(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Gnss_SvInfo>
{
    static const uint8_t descriptorSet = MIP_GNSS_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_GNSS_SV_INFO;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Gnss_SvInfo& self)
    {
        return insert_MipData_Gnss_SvInfo(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Gnss_SvInfo& self)
    {
        return extract_MipData_Gnss_SvInfo(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Gnss_HwStatus>
{
    static const uint8_t descriptorSet = MIP_GNSS_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_GNSS_HW_STATUS;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Gnss_HwStatus& self)
    {
        return insert_MipData_Gnss_HwStatus(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Gnss_HwStatus& self)
    {
        return extract_MipData_Gnss_HwStatus(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Gnss_DgpsInfo>
{
    static const uint8_t descriptorSet = MIP_GNSS_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_GNSS_DGPS_INFO;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Gnss_DgpsInfo& self)
    {
        return insert_MipData_Gnss_DgpsInfo(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Gnss_DgpsInfo& self)
    {
        return extract_MipData_Gnss_DgpsInfo(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Gnss_DgpsChannel>
{
    static const uint8_t descriptorSet = MIP_GNSS_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_GNSS_DGPS_CHANNEL_STATUS;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Gnss_DgpsChannel& self)
    {
        return insert_MipData_Gnss_DgpsChannel(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Gnss_DgpsChannel& self)
    {
        return extract_MipData_Gnss_DgpsChannel(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Gnss_ClockInfo2>
{
    static const uint8_t descriptorSet = MIP_GNSS_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_GNSS_CLOCK_INFO_2;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Gnss_ClockInfo2& self)
    {
        return insert_MipData_Gnss_ClockInfo2(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Gnss_ClockInfo2& self)
    {
        return extract_MipData_Gnss_ClockInfo2(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Gnss_GpsLeapSeconds>
{
    static const uint8_t descriptorSet = MIP_GNSS_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_GNSS_GPS_LEAP_SECONDS;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Gnss_GpsLeapSeconds& self)
    {
        return insert_MipData_Gnss_GpsLeapSeconds(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Gnss_GpsLeapSeconds& self)
    {
        return extract_MipData_Gnss_GpsLeapSeconds(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Gnss_SbasInfo>
{
    static const uint8_t descriptorSet = MIP_GNSS_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_GNSS_SBAS_INFO;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Gnss_SbasInfo& self)
    {
        return insert_MipData_Gnss_SbasInfo(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Gnss_SbasInfo& self)
    {
        return extract_MipData_Gnss_SbasInfo(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Gnss_SbasCorrection>
{
    static const uint8_t descriptorSet = MIP_GNSS_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_GNSS_SBAS_CORRECTION;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Gnss_SbasCorrection& self)
    {
        return insert_MipData_Gnss_SbasCorrection(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Gnss_SbasCorrection& self)
    {
        return extract_MipData_Gnss_SbasCorrection(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Gnss_RfErrorDetection>
{
    static const uint8_t descriptorSet = MIP_GNSS_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_GNSS_RF_ERROR_DETECTION;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Gnss_RfErrorDetection& self)
    {
        return insert_MipData_Gnss_RfErrorDetection(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Gnss_RfErrorDetection& self)
    {
        return extract_MipData_Gnss_RfErrorDetection(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Gnss_BaseStationInfo>
{
    static const uint8_t descriptorSet = MIP_GNSS_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_GNSS_BASE_STATION_INFO;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Gnss_BaseStationInfo& self)
    {
        return insert_MipData_Gnss_BaseStationInfo(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Gnss_BaseStationInfo& self)
    {
        return extract_MipData_Gnss_BaseStationInfo(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Gnss_RtkCorrectionsStatus>
{
    static const uint8_t descriptorSet = MIP_GNSS_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_GNSS_RTK_CORRECTIONS_STATUS;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Gnss_RtkCorrectionsStatus& self)
    {
        return insert_MipData_Gnss_RtkCorrectionsStatus(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Gnss_RtkCorrectionsStatus& self)
    {
        return extract_MipData_Gnss_RtkCorrectionsStatus(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Gnss_SatelliteStatus>
{
    static const uint8_t descriptorSet = MIP_GNSS_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_GNSS_SATELLITE_STATUS;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Gnss_SatelliteStatus& self)
    {
        return insert_MipData_Gnss_SatelliteStatus(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Gnss_SatelliteStatus& self)
    {
        return extract_MipData_Gnss_SatelliteStatus(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Gnss_Raw>
{
    static const uint8_t descriptorSet = MIP_GNSS_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_GNSS_RAW;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Gnss_Raw& self)
    {
        return insert_MipData_Gnss_Raw(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Gnss_Raw& self)
    {
        return extract_MipData_Gnss_Raw(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Gnss_GpsEphemeris>
{
    static const uint8_t descriptorSet = MIP_GNSS_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_GNSS_GPS_EPHEMERIS;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Gnss_GpsEphemeris& self)
    {
        return insert_MipData_Gnss_GpsEphemeris(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Gnss_GpsEphemeris& self)
    {
        return extract_MipData_Gnss_GpsEphemeris(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Gnss_GloEphemeris>
{
    static const uint8_t descriptorSet = MIP_GNSS_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_GNSS_GLONASS_EPHEMERIS;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Gnss_GloEphemeris& self)
    {
        return insert_MipData_Gnss_GloEphemeris(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Gnss_GloEphemeris& self)
    {
        return extract_MipData_Gnss_GloEphemeris(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Gnss_GpsIonoCorr>
{
    static const uint8_t descriptorSet = MIP_GNSS_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_GNSS_GPS_IONO_CORR;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Gnss_GpsIonoCorr& self)
    {
        return insert_MipData_Gnss_GpsIonoCorr(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Gnss_GpsIonoCorr& self)
    {
        return extract_MipData_Gnss_GpsIonoCorr(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Gnss_GalileoIonoCorr>
{
    static const uint8_t descriptorSet = MIP_GNSS_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_GNSS_GALILEO_IONO_CORR;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Gnss_GalileoIonoCorr& self)
    {
        return insert_MipData_Gnss_GalileoIonoCorr(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Gnss_GalileoIonoCorr& self)
    {
        return extract_MipData_Gnss_GalileoIonoCorr(buffer, bufferSize, offset, &self);
    }
};



} // namespace mscl
#endif // __cplusplus
