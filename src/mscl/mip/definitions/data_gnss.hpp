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

namespace data_gnss {

////////////////////////////////////////////////////////////////////////////////
///@addtogroup MipData
///@{
///@defgroup gnss_data_cpp  GNSSData
///
///@{

////////////////////////////////////////////////////////////////////////////////
// Descriptors
////////////////////////////////////////////////////////////////////////////////

enum 
{
    DESCRIPTOR_SET               = 0x81,
    
    DATA_POSITION_LLH            = 0x03,
    DATA_POSITION_ECEF           = 0x04,
    DATA_VELOCITY_NED            = 0x05,
    DATA_VELOCITY_ECEF           = 0x06,
    DATA_DOP                     = 0x07,
    DATA_UTC_TIME                = 0x08,
    DATA_GPS_TIME                = 0x09,
    DATA_CLOCK_INFO              = 0x0A,
    DATA_FIX_INFO                = 0x0B,
    DATA_SV_INFO                 = 0x0C,
    DATA_HW_STATUS               = 0x0D,
    DATA_DGPS_INFO               = 0x0E,
    DATA_DGPS_CHANNEL_STATUS     = 0x0F,
    DATA_CLOCK_INFO_2            = 0x10,
    DATA_GPS_LEAP_SECONDS        = 0x11,
    DATA_SBAS_INFO               = 0x12,
    DATA_SBAS_CORRECTION         = 0x13,
    DATA_RF_ERROR_DETECTION      = 0x14,
    DATA_SATELLITE_STATUS        = 0x20,
    DATA_SATELLITE_SIGNAL_STATUS = 0x21,
    DATA_RAW                     = 0x22,
    DATA_BASE_STATION_INFO       = 0x30,
    DATA_RTK_CORRECTIONS_STATUS  = 0x31,
    DATA_GPS_EPHEMERIS           = 0x61,
    DATA_GLONASS_EPHEMERIS       = 0x62,
    DATA_GALILEO_EPHEMERIS       = 0x63,
    DATA_GPS_IONO_CORR           = 0x71,
    DATA_GLONASS_IONO_CORR       = 0x72,
    DATA_GALILEO_IONO_CORR       = 0x73,
    
};

////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////

static const uint8_t MIP_GNSS1_DATA_DESC_SET = 0x91;
static const uint8_t MIP_GNSS2_DATA_DESC_SET = 0x92;
static const uint8_t MIP_GNSS3_DATA_DESC_SET = 0x93;
static const uint8_t MIP_GNSS4_DATA_DESC_SET = 0x94;
static const uint8_t MIP_GNSS5_DATA_DESC_SET = 0x95;
enum class GnssConstellationId : uint8_t
{
    UNKNOWN = 0,  ///<  
    GPS     = 1,  ///<  
    GLONASS = 2,  ///<  
    GALILEO = 3,  ///<  
    BEIDOU  = 4,  ///<  
    SBAS    = 5,  ///<  
};

enum class GnssSignalId : uint8_t
{
    UNKNOWN        = 0,  ///<  
    GPS_L1CA       = 1,  ///<  
    GPS_L1P        = 2,  ///<  
    GPS_L1Z        = 3,  ///<  
    GPS_L2CA       = 4,  ///<  
    GPS_L2P        = 5,  ///<  
    GPS_L2Z        = 6,  ///<  
    GPS_L2CL       = 7,  ///<  
    GPS_L2CM       = 8,  ///<  
    GPS_L2CML      = 9,  ///<  
    GPS_L5I        = 10,  ///<  
    GPS_L5Q        = 11,  ///<  
    GPS_L5IQ       = 12,  ///<  
    GPS_L1CD       = 13,  ///<  
    GPS_L1CP       = 14,  ///<  
    GPS_L1CDP      = 15,  ///<  
    GLONASS_G1CA   = 32,  ///<  
    GLONASS_G1P    = 33,  ///<  
    GLONASS_G2C    = 34,  ///<  
    GLONASS_G2P    = 35,  ///<  
    GALILEO_E1C    = 64,  ///<  
    GALILEO_E1A    = 65,  ///<  
    GALILEO_E1B    = 66,  ///<  
    GALILEO_E1BC   = 67,  ///<  
    GALILEO_E1ABC  = 68,  ///<  
    GALILEO_E6C    = 69,  ///<  
    GALILEO_E6A    = 70,  ///<  
    GALILEO_E6B    = 71,  ///<  
    GALILEO_E6BC   = 72,  ///<  
    GALILEO_E6ABC  = 73,  ///<  
    GALILEO_E5BI   = 74,  ///<  
    GALILEO_E5BQ   = 75,  ///<  
    GALILEO_E5BIQ  = 76,  ///<  
    GALILEO_E5ABI  = 77,  ///<  
    GALILEO_E5ABQ  = 78,  ///<  
    GALILEO_E5ABIQ = 79,  ///<  
    GALILEO_E5AI   = 80,  ///<  
    GALILEO_E5AQ   = 81,  ///<  
    GALILEO_E5AIQ  = 82,  ///<  
    SBAS_L1CA      = 96,  ///<  
    SBAS_L5I       = 97,  ///<  
    SBAS_L5Q       = 98,  ///<  
    SBAS_L5IQ      = 99,  ///<  
    QZSS_L1CA      = 128,  ///<  
    QZSS_LEXS      = 129,  ///<  
    QZSS_LEXL      = 130,  ///<  
    QZSS_LEXSL     = 131,  ///<  
    QZSS_L2CM      = 132,  ///<  
    QZSS_L2CL      = 133,  ///<  
    QZSS_L2CML     = 134,  ///<  
    QZSS_L5I       = 135,  ///<  
    QZSS_L5Q       = 136,  ///<  
    QZSS_L5IQ      = 137,  ///<  
    QZSS_L1CD      = 138,  ///<  
    QZSS_L1CP      = 139,  ///<  
    QZSS_L1CDP     = 140,  ///<  
    BEIDOU_B1I     = 160,  ///<  
    BEIDOU_B1Q     = 161,  ///<  
    BEIDOU_B1IQ    = 162,  ///<  
    BEIDOU_B3I     = 163,  ///<  
    BEIDOU_B3Q     = 164,  ///<  
    BEIDOU_B3IQ    = 165,  ///<  
    BEIDOU_B2I     = 166,  ///<  
    BEIDOU_B2Q     = 167,  ///<  
    BEIDOU_B2IQ    = 168,  ///<  
};

enum class SbasSystem : uint8_t
{
    UNKNOWN = 0,  ///<  
    WAAS    = 1,  ///<  
    EGNOS   = 2,  ///<  
    MSAS    = 3,  ///<  
    GAGAN   = 4,  ///<  
};

static const uint32_t GNSS_DGPS_INFO_MAX_CHANNEL_NUMBER = 32;
static const uint32_t GNSS_SV_INFO_MAX_SV_NUMBER = 32;

////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_pos_llh  GNSS LLH Position
/// GNSS reported position in the WGS84 geodetic frame
///
///@{

struct PosLlh
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::data_gnss::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::data_gnss::DATA_POSITION_LLH;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    enum class ValidFlags : uint16_t
    {
        LAT_LON             = 0x01,
        ELLIPSOID_HEIGHT    = 0x02,
        MSL_HEIGHT          = 0x04,
        HORIZONTAL_ACCURACY = 0x08,
        VERTICAL_ACCURACY   = 0x10,
        FLAGS               = 0x1F,
    };
    
    double latitude;
    double longitude;
    double ellipsoid_height;
    double msl_height;
    float horizontal_accuracy;
    float vertical_accuracy;
    ValidFlags valid_flags;
    
};
void insert(MipSerializer& serializer, const PosLlh& self);
void extract(MipSerializer& serializer, PosLlh& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_pos_ecef  GNSS ECEF Position
/// GNSS reported position in the Earth-centered, Earth-Fixed (ECEF) frame
///
///@{

struct PosEcef
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::data_gnss::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::data_gnss::DATA_POSITION_ECEF;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    enum class ValidFlags : uint16_t
    {
        POSITION          = 0x01,
        POSITION_ACCURACY = 0x02,
        FLAGS             = 0x03,
    };
    
    double x[3];
    float x_accuracy;
    ValidFlags valid_flags;
    
};
void insert(MipSerializer& serializer, const PosEcef& self);
void extract(MipSerializer& serializer, PosEcef& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_vel_ned  NED Velocity
/// GNSS reported velocity in the NED frame
///
///@{

struct VelNed
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::data_gnss::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::data_gnss::DATA_VELOCITY_NED;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    enum class ValidFlags : uint16_t
    {
        VELOCITY         = 0x01,
        SPEED_3D         = 0x02,
        GROUND_SPEED     = 0x04,
        HEADING          = 0x08,
        SPEED_ACCURACY   = 0x10,
        HEADING_ACCURACY = 0x20,
        FLAGS            = 0x3F,
    };
    
    float v[3];
    float speed;
    float ground_speed;
    float heading;
    float speed_accuracy;
    float heading_accuracy;
    ValidFlags valid_flags;
    
};
void insert(MipSerializer& serializer, const VelNed& self);
void extract(MipSerializer& serializer, VelNed& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_vel_ecef  GNSS ECEF Velocity
/// GNSS reported velocity in the Earth-centered, Earth-Fixed (ECEF) frame
///
///@{

struct VelEcef
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::data_gnss::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::data_gnss::DATA_VELOCITY_ECEF;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    enum class ValidFlags : uint16_t
    {
        VELOCITY          = 0x01,
        VELOCITY_ACCURACY = 0x02,
        FLAGS             = 0x03,
    };
    
    float v[3];
    float v_accuracy;
    ValidFlags valid_flags;
    
};
void insert(MipSerializer& serializer, const VelEcef& self);
void extract(MipSerializer& serializer, VelEcef& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_dop  None
/// GNSS reported dilution of precision information.
///
///@{

struct Dop
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::data_gnss::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::data_gnss::DATA_DOP;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    enum class ValidFlags : uint16_t
    {
        GDOP  = 0x01,
        PDOP  = 0x02,
        HDOP  = 0x04,
        VDOP  = 0x08,
        TDOP  = 0x10,
        NDOP  = 0x20,
        EDOP  = 0x40,
        FLAGS = 0x7F,
    };
    
    float gdop;
    float pdop;
    float hdop;
    float vdop;
    float tdop;
    float ndop;
    float edop;
    ValidFlags valid_flags;
    
};
void insert(MipSerializer& serializer, const Dop& self);
void extract(MipSerializer& serializer, Dop& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_utc_time  None
/// GNSS reported Coordinated Universal Time
///
///@{

struct UtcTime
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::data_gnss::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::data_gnss::DATA_UTC_TIME;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    enum class ValidFlags : uint16_t
    {
        GNSS_DATE_TIME     = 0x01,
        LEAP_SECONDS_KNOWN = 0x02,
        FLAGS              = 0x03,
    };
    
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
    uint32_t msec;
    ValidFlags valid_flags;
    
};
void insert(MipSerializer& serializer, const UtcTime& self);
void extract(MipSerializer& serializer, UtcTime& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_gps_time  None
/// GNSS reported GPS Time
///
///@{

struct GpsTime
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::data_gnss::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::data_gnss::DATA_GPS_TIME;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    enum class ValidFlags : uint16_t
    {
        TOW         = 0x01,
        WEEK_NUMBER = 0x02,
        FLAGS       = 0x03,
    };
    
    double tow;
    uint16_t week_number;
    ValidFlags valid_flags;
    
};
void insert(MipSerializer& serializer, const GpsTime& self);
void extract(MipSerializer& serializer, GpsTime& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_clock_info  None
/// GNSS reported receiver clock parameters
///
///@{

struct ClockInfo
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::data_gnss::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::data_gnss::DATA_CLOCK_INFO;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    enum class ValidFlags : uint16_t
    {
        BIAS              = 0x01,
        DRIFT             = 0x02,
        ACCURACY_ESTIMATE = 0x04,
        FLAGS             = 0x07,
    };
    
    double bias;
    double drift;
    double accuracy_estimate;
    ValidFlags valid_flags;
    
};
void insert(MipSerializer& serializer, const ClockInfo& self);
void extract(MipSerializer& serializer, ClockInfo& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_fix_info  None
/// GNSS reported position fix type
///
///@{

struct FixInfo
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::data_gnss::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::data_gnss::DATA_FIX_INFO;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    enum class FixType : uint8_t
    {
        FIX_3D        = 0,  ///<  
        FIX_2D        = 1,  ///<  
        FIX_TIME_ONLY = 2,  ///<  
        FIX_NONE      = 3,  ///<  
        FIX_INVALID   = 4,  ///<  
        FIX_RTK_FLOAT = 5,  ///<  
        FIX_RTK_FIXED = 6,  ///<  
    };
    
    enum class FixFlags : uint16_t
    {
        SBAS_USED  = 0x01,
        DNGSS_USED = 0x02,
    };
    
    enum class ValidFlags : uint16_t
    {
        FIX_TYPE  = 0x01,
        NUM_SV    = 0x02,
        FIX_FLAGS = 0x04,
        FLAGS     = 0x07,
    };
    
    FixType fix_type;
    uint8_t num_sv;
    FixFlags fix_flags;
    ValidFlags valid_flags;
    
};
void insert(MipSerializer& serializer, const FixInfo& self);
void extract(MipSerializer& serializer, FixInfo& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_sv_info  None
/// GNSS reported space vehicle information
/// 
/// When enabled, these fields will arrive in separate MIP packets
///
///@{

struct SvInfo
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::data_gnss::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::data_gnss::DATA_SV_INFO;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    enum class SVFlags : uint16_t
    {
        USED_FOR_NAVIGATION = 0x01,
        HEALTHY             = 0x02,
    };
    
    enum class ValidFlags : uint16_t
    {
        CHANNEL             = 0x01,
        SV_ID               = 0x02,
        CARRIER_NOISE_RATIO = 0x04,
        AZIMUTH             = 0x08,
        ELEVATION           = 0x10,
        SV_FLAGS            = 0x20,
        FLAGS               = 0x3F,
    };
    
    uint8_t channel;
    uint8_t sv_id;
    uint16_t carrier_noise_ratio;
    int16_t azimuth;
    int16_t elevation;
    SVFlags sv_flags;
    ValidFlags valid_flags;
    
};
void insert(MipSerializer& serializer, const SvInfo& self);
void extract(MipSerializer& serializer, SvInfo& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_hw_status  GNSS Hardware Status
/// GNSS reported hardware status
///
///@{

struct HwStatus
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::data_gnss::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::data_gnss::DATA_HW_STATUS;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    enum class ReceiverState : uint8_t
    {
        OFF     = 0,  ///<  
        ON      = 1,  ///<  
        UNKNOWN = 2,  ///<  
    };
    
    enum class AntennaState : uint8_t
    {
        INIT    = 1,  ///<  
        SHORT   = 2,  ///<  
        OPEN    = 3,  ///<  
        GOOD    = 4,  ///<  
        UNKNOWN = 5,  ///<  
    };
    
    enum class AntennaPower : uint8_t
    {
        OFF     = 0,  ///<  
        ON      = 1,  ///<  
        UNKNOWN = 2,  ///<  
    };
    
    enum class ValidFlags : uint16_t
    {
        SENSOR_STATE  = 0x01,
        ANTENNA_STATE = 0x02,
        ANTENNA_POWER = 0x04,
        FLAGS         = 0x07,
    };
    
    ReceiverState receiver_state;
    AntennaState antenna_state;
    AntennaPower antenna_power;
    ValidFlags valid_flags;
    
};
void insert(MipSerializer& serializer, const HwStatus& self);
void extract(MipSerializer& serializer, HwStatus& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_dgps_info  None
/// GNSS reported DGNSS status
/// 
/// <pre>Possible Base Station Status Values:</pre>
/// <pre>  0 – UDRE Scale Factor = 1.0</pre>
/// <pre>  1 – UDRE Scale Factor = 0.75</pre>
/// <pre>  2 – UDRE Scale Factor = 0.5</pre>
/// <pre>  3 – UDRE Scale Factor = 0.3</pre>
/// <pre>  4 – UDRE Scale Factor = 0.2</pre>
/// <pre>  5 – UDRE Scale Factor = 0.1</pre>
/// <pre>  6 – Reference Station Transmission Not Monitored</pre>
/// <pre>  7 – Reference Station Not Working</pre>
/// 
/// (UDRE = User Differential Range Error)
///
///@{

struct DgpsInfo
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::data_gnss::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::data_gnss::DATA_DGPS_INFO;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    enum class ValidFlags : uint16_t
    {
        AGE                 = 0x01,
        BASE_STATION_ID     = 0x02,
        BASE_STATION_STATUS = 0x04,
        NUM_CHANNELS        = 0x08,
        FLAGS               = 0x0F,
    };
    
    uint8_t sv_id;
    float age;
    float range_correction;
    float range_rate_correction;
    ValidFlags valid_flags;
    
};
void insert(MipSerializer& serializer, const DgpsInfo& self);
void extract(MipSerializer& serializer, DgpsInfo& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_dgps_channel  None
/// GNSS reported DGPS Channel Status status
/// 
/// When enabled, a separate field for each active space vehicle will be sent in the packet.
///
///@{

struct DgpsChannel
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::data_gnss::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::data_gnss::DATA_DGPS_CHANNEL_STATUS;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    enum class ValidFlags : uint16_t
    {
        ID                    = 0x01,
        AGE                   = 0x02,
        RANGE_CORRECTION      = 0x04,
        RANGE_RATE_CORRECTION = 0x08,
        FLAGS                 = 0x0F,
    };
    
    uint8_t sv_id;
    float age;
    float range_correction;
    float range_rate_correction;
    ValidFlags valid_flags;
    
};
void insert(MipSerializer& serializer, const DgpsChannel& self);
void extract(MipSerializer& serializer, DgpsChannel& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_clock_info_2  None
/// GNSS reported receiver clock parameters
/// 
/// This supersedes MIP_DATA_DESC_GNSS_CLOCK_INFO with additional information.
///
///@{

struct ClockInfo2
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::data_gnss::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::data_gnss::DATA_CLOCK_INFO_2;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    enum class ValidFlags : uint16_t
    {
        BIAS           = 0x01,
        DRIFT          = 0x02,
        BIAS_ACCURACY  = 0x04,
        DRIFT_ACCURACY = 0x08,
        FLAGS          = 0x0F,
    };
    
    double bias;
    double drift;
    double bias_accuracy_estimate;
    double drift_accuracy_estimate;
    ValidFlags valid_flags;
    
};
void insert(MipSerializer& serializer, const ClockInfo2& self);
void extract(MipSerializer& serializer, ClockInfo2& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_gps_leap_seconds  None
/// GNSS reported leap seconds (difference between GPS and UTC Time)
///
///@{

struct GpsLeapSeconds
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::data_gnss::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::data_gnss::DATA_GPS_LEAP_SECONDS;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    enum class ValidFlags : uint16_t
    {
        LEAP_SECONDS = 0x02,
    };
    
    uint8_t leap_seconds;
    ValidFlags valid_flags;
    
};
void insert(MipSerializer& serializer, const GpsLeapSeconds& self);
void extract(MipSerializer& serializer, GpsLeapSeconds& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_sbas_info  None
/// GNSS SBAS status
///
///@{

struct SbasInfo
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::data_gnss::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::data_gnss::DATA_SBAS_INFO;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    enum class SbasStatus : uint8_t
    {
        RANGE_AVAILABLE       = 0x01,
        CORRECTIONS_AVAILABLE = 0x02,
        INTEGRITY_AVAILABLE   = 0x04,
        TEST_MODE             = 0x08,
    };
    
    enum class ValidFlags : uint16_t
    {
        TOW         = 0x01,
        WEEK_NUMBER = 0x02,
        SBAS_SYSTEM = 0x04,
        SBAS_ID     = 0x08,
        COUNT       = 0x10,
        SBAS_STATUS = 0x20,
        FLAGS       = 0x3F,
    };
    
    double time_of_week;
    uint16_t week_number;
    SbasSystem sbas_system;
    uint8_t sbas_id;
    uint8_t count;
    SbasStatus sbas_status;
    ValidFlags valid_flags;
    
};
void insert(MipSerializer& serializer, const SbasInfo& self);
void extract(MipSerializer& serializer, SbasInfo& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_sbas_correction  None
/// GNSS calculated SBAS Correction
/// 
/// UDREI - the variance of a normal distribution associated with the user differential range errors for a
/// satellite after application of fast and long-term corrections, excluding atmospheric effects
/// 
/// <pre>UDREI  Variance</pre>
/// <pre>-----------------------</pre>
/// <pre>0      0.0520 m^2</pre>
/// <pre>1      0.0924 m^2</pre>
/// <pre>2      0.1444 m^2</pre>
/// <pre>3      0.2830 m^2</pre>
/// <pre>4      0.4678 m^2</pre>
/// <pre>5      0.8315 m^2</pre>
/// <pre>6      1.2992 m^2</pre>
/// <pre>7      1.8709 m^2</pre>
/// <pre>8      2.5465 m^2</pre>
/// <pre>9      3.3260 m^2</pre>
/// <pre>10     5.1968 m^2</pre>
/// <pre>11     20.7870 m^2</pre>
/// <pre>12     230.9661 m^2</pre>
/// <pre>13     2078.695 m^2</pre>
/// <pre>14     "Not Monitored"</pre>
/// <pre>15     "Do Not Use"</pre>
///
///@{

struct SbasCorrection
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::data_gnss::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::data_gnss::DATA_SBAS_CORRECTION;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    enum class ValidFlags : uint16_t
    {
        UDREI                  = 0x01,
        PSEUDORANGE_CORRECTION = 0x02,
        IONO_CORRECTION        = 0x04,
        FLAGS                  = 0x07,
    };
    
    uint8_t index;
    uint8_t count;
    double time_of_week;
    uint16_t week_number;
    GnssConstellationId gnss_id;
    uint8_t sv_id;
    uint8_t udrei;
    float pseudorange_correction;
    float iono_correction;
    ValidFlags valid_flags;
    
};
void insert(MipSerializer& serializer, const SbasCorrection& self);
void extract(MipSerializer& serializer, SbasCorrection& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_rf_error_detection  None
/// GNSS Error Detection subsystem status
///
///@{

struct RfErrorDetection
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::data_gnss::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::data_gnss::DATA_RF_ERROR_DETECTION;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    enum class RFBand : uint8_t
    {
        UNKNOWN = 0,  ///<  
        L1      = 1,  ///<  
        L2      = 2,  ///<  
        L5      = 5,  ///<  
    };
    
    enum class JammingState : uint8_t
    {
        UNKNOWN     = 0,  ///<  
        NONE        = 1,  ///<  
        PARTIAL     = 2,  ///<  
        SIGNIFICANT = 3,  ///<  
    };
    
    enum class SpoofingState : uint8_t
    {
        UNKNOWN     = 0,  ///<  
        NONE        = 1,  ///<  
        PARTIAL     = 2,  ///<  
        SIGNIFICANT = 3,  ///<  
    };
    
    enum class ValidFlags : uint16_t
    {
        RF_BAND        = 0x01,
        JAMMING_STATE  = 0x02,
        SPOOFING_STATE = 0x04,
        FLAGS          = 0x07,
    };
    
    RFBand rf_band;
    JammingState jamming_state;
    SpoofingState spoofing_state;
    uint8_t reserved[4];
    ValidFlags valid_flags;
    
};
void insert(MipSerializer& serializer, const RfErrorDetection& self);
void extract(MipSerializer& serializer, RfErrorDetection& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_base_station_info  None
/// RTCM reported base station information (sourced from RTCM Message 1005 or 1006)
/// 
/// Valid Flag Mapping:
///
///@{

struct BaseStationInfo
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::data_gnss::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::data_gnss::DATA_BASE_STATION_INFO;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    enum class IndicatorFlags : uint16_t
    {
        GPS                = 0x01,
        GLONASS            = 0x02,
        GALILEO            = 0x04,
        BEIDOU             = 0x08,
        REF_STATION        = 0x10,
        SINGLE_RECEIVER    = 0x20,
        QUARTER_CYCLE_BIT1 = 0x40,
        QUARTER_CYCLE_BIT2 = 0x80,
        QUARTER_CYCLE_BITS = 0xC0,
    };
    
    enum class ValidFlags : uint16_t
    {
        TOW           = 0x01,
        WEEK_NUMBER   = 0x02,
        ECEF_POSITION = 0x04,
        HEIGHT        = 0x08,
        STATION_ID    = 0x10,
        INDICATORS    = 0x20,
        FLAGS         = 0x3F,
    };
    
    double time_of_week;
    uint16_t week_number;
    double ecef_pos[3];
    float height;
    uint16_t station_id;
    IndicatorFlags indicators;
    ValidFlags valid_flags;
    
};
void insert(MipSerializer& serializer, const BaseStationInfo& self);
void extract(MipSerializer& serializer, BaseStationInfo& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_rtk_corrections_status  None
///
///@{

struct RtkCorrectionsStatus
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::data_gnss::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::data_gnss::DATA_RTK_CORRECTIONS_STATUS;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    enum class ValidFlags : uint16_t
    {
        TOW             = 0x01,
        WEEK_NUMBER     = 0x02,
        EPOCH_STATUS    = 0x04,
        DONGLE_STATUS   = 0x08,
        GPS_LATENCY     = 0x10,
        GLONASS_LATENCY = 0x20,
        GALILEO_LATENCY = 0x40,
        BEIDOU_LATENCY  = 0x80,
        FLAGS           = 0xFF,
    };
    
    enum class EpochStatus : uint16_t
    {
        ANTENNA_LOCATION_RECEIVED    = 0x01,
        ANTENNA_DESCRIPTION_RECEIVED = 0x02,
        GPS_RECEIVED                 = 0x04,
        GLONASS_RECEIVED             = 0x08,
        GALILEO_RECEIVED             = 0x10,
        BEIDOU_RECEIVED              = 0x20,
        USING_GPS_MSM_MESSAGES       = 0x40,
        USING_GLONASS_MSM_MESSAGES   = 0x80,
        DONGLE_STATUS_READ_FAILED    = 0x100,
    };
    
    double time_of_week;
    uint16_t week_number;
    EpochStatus epoch_status;
    uint32_t dongle_status;
    float gps_correction_latency;
    float glonass_correction_latency;
    float galileo_correction_latency;
    float beidou_correction_latency;
    uint32_t reserved[4];
    ValidFlags valid_flags;
    
};
void insert(MipSerializer& serializer, const RtkCorrectionsStatus& self);
void extract(MipSerializer& serializer, RtkCorrectionsStatus& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_satellite_status  None
/// Status information for a GNSS satellite.
///
///@{

struct SatelliteStatus
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::data_gnss::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::data_gnss::DATA_SATELLITE_STATUS;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    enum class ValidFlags : uint16_t
    {
        TOW          = 0x01,
        WEEK_NUMBER  = 0x02,
        GNSS_ID      = 0x04,
        SATELLITE_ID = 0x08,
        ELEVATION    = 0x10,
        AZIMUTH      = 0x20,
        HEALTH       = 0x40,
        FLAGS        = 0x7F,
    };
    
    uint8_t index;
    uint8_t count;
    double time_of_week;
    uint16_t week_number;
    GnssConstellationId gnss_id;
    uint8_t satellite_id;
    float elevation;
    float azimuth;
    bool health;
    ValidFlags valid_flags;
    
};
void insert(MipSerializer& serializer, const SatelliteStatus& self);
void extract(MipSerializer& serializer, SatelliteStatus& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_raw  None
/// GNSS Raw observation.
///
///@{

struct Raw
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::data_gnss::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::data_gnss::DATA_RAW;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    enum class GnssSignalQuality : uint8_t
    {
        NONE         = 0,  ///<  
        SEARCHING    = 1,  ///<  
        ACQUIRED     = 2,  ///<  
        UNUSABLE     = 3,  ///<  
        TIME_LOCKED  = 4,  ///<  
        FULLY_LOCKED = 5,  ///<  
    };
    
    enum class ValidFlags : uint16_t
    {
        TOW                       = 0x01,
        WEEK_NUMBER               = 0x02,
        RECEIVER_ID               = 0x04,
        TRACKING_CHANNEL          = 0x08,
        GNSS_ID                   = 0x10,
        SATELLITE_ID              = 0x20,
        SIGNAL_ID                 = 0x40,
        SIGNAL_STRENGTH           = 0x80,
        QUALITY                   = 0x100,
        PSEUDORANGE               = 0x200,
        CARRIER_PHASE             = 0x400,
        DOPPLER                   = 0x800,
        RANGE_UNCERTAINTY         = 0x1000,
        CARRIER_PHASE_UNCERTAINTY = 0x2000,
        DOPPLER_UNCERTAINTY       = 0x4000,
        LOCK_TIME                 = 0x8000,
        FLAGS                     = 0xFFFF,
    };
    
    uint8_t index;
    uint8_t count;
    double time_of_week;
    uint16_t week_number;
    uint16_t receiver_id;
    uint8_t tracking_channel;
    GnssConstellationId gnss_id;
    uint8_t satellite_id;
    GnssSignalId signal_id;
    float signal_strength;
    GnssSignalQuality quality;
    double pseudorange;
    double carrier_phase;
    float doppler;
    float range_uncert;
    float phase_uncert;
    float doppler_uncert;
    float lock_time;
    ValidFlags valid_flags;
    
};
void insert(MipSerializer& serializer, const Raw& self);
void extract(MipSerializer& serializer, Raw& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_gps_ephemeris  None
/// GPS/Galileo Ephemeris Data
///
///@{

struct GpsEphemeris
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::data_gnss::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::data_gnss::DATA_GPS_EPHEMERIS;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    enum class ValidFlags : uint16_t
    {
        EPHEMERIS   = 0x01,
        MODERN_DATA = 0x02,
        FLAGS       = 0x03,
    };
    
    uint8_t index;
    uint8_t count;
    double time_of_week;
    uint16_t week_number;
    uint8_t satellite_id;
    uint8_t health;
    uint8_t iodc;
    uint8_t iode;
    double t_oc;
    double af0;
    double af1;
    double af2;
    double t_gd;
    double ISC_L1CA;
    double ISC_L2C;
    double t_oe;
    double a;
    double a_dot;
    double mean_anomaly;
    double delta_mean_motion;
    double delta_mean_motion_dot;
    double eccentricity;
    double argument_of_perigee;
    double omega;
    double omega_dot;
    double inclination;
    double inclination_dot;
    double c_ic;
    double c_is;
    double c_uc;
    double c_us;
    double c_rc;
    double c_rs;
    ValidFlags valid_flags;
    
};
void insert(MipSerializer& serializer, const GpsEphemeris& self);
void extract(MipSerializer& serializer, GpsEphemeris& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_glo_ephemeris  Glonass Ephemeris
/// Glonass Ephemeris Data
///
///@{

struct GloEphemeris
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::data_gnss::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::data_gnss::DATA_GLONASS_EPHEMERIS;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    enum class ValidFlags : uint16_t
    {
        EPHEMERIS = 0x01,
        FLAGS     = 0x01,
    };
    
    uint8_t index;
    uint8_t count;
    double time_of_week;
    uint16_t week_number;
    uint8_t satellite_id;
    int8_t freq_number;
    uint32_t tk;
    uint32_t tb;
    uint8_t sat_type;
    double gamma;
    double tau_n;
    double x[3];
    float v[3];
    float a[3];
    uint8_t health;
    uint8_t P;
    uint8_t NT;
    float delta_tau_n;
    uint8_t Ft;
    uint8_t En;
    uint8_t P1;
    uint8_t P2;
    uint8_t P3;
    uint8_t P4;
    ValidFlags valid_flags;
    
};
void insert(MipSerializer& serializer, const GloEphemeris& self);
void extract(MipSerializer& serializer, GloEphemeris& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_gps_iono_corr  GPS Ionospheric Correction
/// Ionospheric Correction Terms for GNSS
///
///@{

struct GpsIonoCorr
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::data_gnss::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::data_gnss::DATA_GPS_IONO_CORR;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    enum class ValidFlags : uint16_t
    {
        TOW         = 0x01,
        WEEK_NUMBER = 0x02,
        ALPHA       = 0x04,
        BETA        = 0x08,
        FLAGS       = 0x0F,
    };
    
    double time_of_week;
    uint16_t week_number;
    double alpha[4];
    double beta[4];
    ValidFlags valid_flags;
    
};
void insert(MipSerializer& serializer, const GpsIonoCorr& self);
void extract(MipSerializer& serializer, GpsIonoCorr& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_galileo_iono_corr  Galileo Ionospheric Correction
/// Ionospheric Correction Terms for Galileo
///
///@{

struct GalileoIonoCorr
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::data_gnss::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::data_gnss::DATA_GALILEO_IONO_CORR;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    enum class ValidFlags : uint16_t
    {
        TOW               = 0x01,
        WEEK_NUMBER       = 0x02,
        ALPHA             = 0x04,
        DISTURBANCE_FLAGS = 0x08,
        FLAGS             = 0x0F,
    };
    
    double time_of_week;
    uint16_t week_number;
    double alpha[3];
    uint8_t disturbance_flags;
    ValidFlags valid_flags;
    
};
void insert(MipSerializer& serializer, const GalileoIonoCorr& self);
void extract(MipSerializer& serializer, GalileoIonoCorr& self);

///@}
///

///@}
///@}
///
////////////////////////////////////////////////////////////////////////////////
} // namespace data_gnss
} // namespace mscl

