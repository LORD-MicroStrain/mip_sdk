#pragma once

#include "common.hpp"
#include <mip/mip_descriptors.hpp>
#include <mip/mip_result.hpp>
#include <mip/mip_interface.hpp>

#include <stdint.h>
#include <stddef.h>

namespace mip {
namespace C {
struct mip_interface;
} // namespace C

namespace data_gnss {

////////////////////////////////////////////////////////////////////////////////
///@addtogroup MipData_cpp  MIP Data [CPP]
///@{
///@defgroup gnss_data_cpp  Gnss Data [CPP]
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

static constexpr const uint8_t MIP_GNSS1_DATA_DESC_SET = 0x91;
static constexpr const uint8_t MIP_GNSS2_DATA_DESC_SET = 0x92;
static constexpr const uint8_t MIP_GNSS3_DATA_DESC_SET = 0x93;
static constexpr const uint8_t MIP_GNSS4_DATA_DESC_SET = 0x94;
static constexpr const uint8_t MIP_GNSS5_DATA_DESC_SET = 0x95;
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

static constexpr const uint32_t GNSS_DGPS_INFO_MAX_CHANNEL_NUMBER = 32;
static constexpr const uint32_t GNSS_SV_INFO_MAX_SV_NUMBER = 32;

////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_gnss_pos_llh  (0x81,0x03) Pos Llh [CPP]
/// GNSS reported position in the WGS84 geodetic frame
///
///@{

struct PosLlh
{
    struct ValidFlags : Bitfield<ValidFlags>
    {
        typedef uint16_t Type;
        enum _enumType : uint16_t
        {
            NONE                = 0x0000,
            LAT_LON             = 0x0001,  ///<  
            ELLIPSOID_HEIGHT    = 0x0002,  ///<  
            MSL_HEIGHT          = 0x0004,  ///<  
            HORIZONTAL_ACCURACY = 0x0008,  ///<  
            VERTICAL_ACCURACY   = 0x0010,  ///<  
            FLAGS               = 0x001F,  ///<  
            ALL                 = 0x001F,
        };
        uint16_t value = NONE;
        
        ValidFlags() : value(NONE) {}
        ValidFlags(int val) : value((uint16_t)val) {}
        operator uint16_t() const { return value; }
        ValidFlags& operator=(uint16_t val) { value = val; return *this; }
        ValidFlags& operator=(int val) { value = uint16_t(val); return *this; }
        ValidFlags& operator|=(uint16_t val) { return *this = value | val; }
        ValidFlags& operator&=(uint16_t val) { return *this = value & val; }
        
        bool allSet() const { return value == ALL; }
        void setAll() { value |= ALL; }
    };
    /// Parameters
    double latitude = 0; ///< [degrees]
    double longitude = 0; ///< [degrees]
    double ellipsoid_height = 0; ///< [meters]
    double msl_height = 0; ///< [meters]
    float horizontal_accuracy = 0; ///< [meters]
    float vertical_accuracy = 0; ///< [meters]
    ValidFlags valid_flags;
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_gnss::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_gnss::DATA_POSITION_LLH;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "PosLlh";
    static constexpr const char* DOC_NAME = "GNSS LLH Position";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(latitude,longitude,ellipsoid_height,msl_height,horizontal_accuracy,vertical_accuracy,valid_flags);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(latitude),std::ref(longitude),std::ref(ellipsoid_height),std::ref(msl_height),std::ref(horizontal_accuracy),std::ref(vertical_accuracy),std::ref(valid_flags));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_gnss_pos_ecef  (0x81,0x04) Pos Ecef [CPP]
/// GNSS reported position in the Earth-centered, Earth-Fixed (ECEF) frame
///
///@{

struct PosEcef
{
    struct ValidFlags : Bitfield<ValidFlags>
    {
        typedef uint16_t Type;
        enum _enumType : uint16_t
        {
            NONE              = 0x0000,
            POSITION          = 0x0001,  ///<  
            POSITION_ACCURACY = 0x0002,  ///<  
            FLAGS             = 0x0003,  ///<  
            ALL               = 0x0003,
        };
        uint16_t value = NONE;
        
        ValidFlags() : value(NONE) {}
        ValidFlags(int val) : value((uint16_t)val) {}
        operator uint16_t() const { return value; }
        ValidFlags& operator=(uint16_t val) { value = val; return *this; }
        ValidFlags& operator=(int val) { value = uint16_t(val); return *this; }
        ValidFlags& operator|=(uint16_t val) { return *this = value | val; }
        ValidFlags& operator&=(uint16_t val) { return *this = value & val; }
        
        bool allSet() const { return value == ALL; }
        void setAll() { value |= ALL; }
    };
    /// Parameters
    Vector3d x; ///< [meters]
    float x_accuracy = 0; ///< [meters]
    ValidFlags valid_flags;
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_gnss::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_gnss::DATA_POSITION_ECEF;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "PosEcef";
    static constexpr const char* DOC_NAME = "GNSS ECEF Position";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(x[0],x[1],x[2],x_accuracy,valid_flags);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(x[0]),std::ref(x[1]),std::ref(x[2]),std::ref(x_accuracy),std::ref(valid_flags));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_gnss_vel_ned  (0x81,0x05) Vel Ned [CPP]
/// GNSS reported velocity in the NED frame
///
///@{

struct VelNed
{
    struct ValidFlags : Bitfield<ValidFlags>
    {
        typedef uint16_t Type;
        enum _enumType : uint16_t
        {
            NONE             = 0x0000,
            VELOCITY         = 0x0001,  ///<  
            SPEED_3D         = 0x0002,  ///<  
            GROUND_SPEED     = 0x0004,  ///<  
            HEADING          = 0x0008,  ///<  
            SPEED_ACCURACY   = 0x0010,  ///<  
            HEADING_ACCURACY = 0x0020,  ///<  
            FLAGS            = 0x003F,  ///<  
            ALL              = 0x003F,
        };
        uint16_t value = NONE;
        
        ValidFlags() : value(NONE) {}
        ValidFlags(int val) : value((uint16_t)val) {}
        operator uint16_t() const { return value; }
        ValidFlags& operator=(uint16_t val) { value = val; return *this; }
        ValidFlags& operator=(int val) { value = uint16_t(val); return *this; }
        ValidFlags& operator|=(uint16_t val) { return *this = value | val; }
        ValidFlags& operator&=(uint16_t val) { return *this = value & val; }
        
        bool allSet() const { return value == ALL; }
        void setAll() { value |= ALL; }
    };
    /// Parameters
    Vector3f v; ///< [meters/second]
    float speed = 0; ///< [meters/second]
    float ground_speed = 0; ///< [meters/second]
    float heading = 0; ///< [degrees]
    float speed_accuracy = 0; ///< [meters/second]
    float heading_accuracy = 0; ///< [degrees]
    ValidFlags valid_flags;
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_gnss::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_gnss::DATA_VELOCITY_NED;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "VelNed";
    static constexpr const char* DOC_NAME = "NED Velocity";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(v[0],v[1],v[2],speed,ground_speed,heading,speed_accuracy,heading_accuracy,valid_flags);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(v[0]),std::ref(v[1]),std::ref(v[2]),std::ref(speed),std::ref(ground_speed),std::ref(heading),std::ref(speed_accuracy),std::ref(heading_accuracy),std::ref(valid_flags));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_gnss_vel_ecef  (0x81,0x06) Vel Ecef [CPP]
/// GNSS reported velocity in the Earth-centered, Earth-Fixed (ECEF) frame
///
///@{

struct VelEcef
{
    struct ValidFlags : Bitfield<ValidFlags>
    {
        typedef uint16_t Type;
        enum _enumType : uint16_t
        {
            NONE              = 0x0000,
            VELOCITY          = 0x0001,  ///<  
            VELOCITY_ACCURACY = 0x0002,  ///<  
            FLAGS             = 0x0003,  ///<  
            ALL               = 0x0003,
        };
        uint16_t value = NONE;
        
        ValidFlags() : value(NONE) {}
        ValidFlags(int val) : value((uint16_t)val) {}
        operator uint16_t() const { return value; }
        ValidFlags& operator=(uint16_t val) { value = val; return *this; }
        ValidFlags& operator=(int val) { value = uint16_t(val); return *this; }
        ValidFlags& operator|=(uint16_t val) { return *this = value | val; }
        ValidFlags& operator&=(uint16_t val) { return *this = value & val; }
        
        bool allSet() const { return value == ALL; }
        void setAll() { value |= ALL; }
    };
    /// Parameters
    Vector3f v; ///< [meters/second]
    float v_accuracy = 0; ///< [meters/second]
    ValidFlags valid_flags;
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_gnss::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_gnss::DATA_VELOCITY_ECEF;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "VelEcef";
    static constexpr const char* DOC_NAME = "GNSS ECEF Velocity";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(v[0],v[1],v[2],v_accuracy,valid_flags);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(v[0]),std::ref(v[1]),std::ref(v[2]),std::ref(v_accuracy),std::ref(valid_flags));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_gnss_dop  (0x81,0x07) Dop [CPP]
/// GNSS reported dilution of precision information.
///
///@{

struct Dop
{
    struct ValidFlags : Bitfield<ValidFlags>
    {
        typedef uint16_t Type;
        enum _enumType : uint16_t
        {
            NONE  = 0x0000,
            GDOP  = 0x0001,  ///<  
            PDOP  = 0x0002,  ///<  
            HDOP  = 0x0004,  ///<  
            VDOP  = 0x0008,  ///<  
            TDOP  = 0x0010,  ///<  
            NDOP  = 0x0020,  ///<  
            EDOP  = 0x0040,  ///<  
            FLAGS = 0x007F,  ///<  
            ALL   = 0x007F,
        };
        uint16_t value = NONE;
        
        ValidFlags() : value(NONE) {}
        ValidFlags(int val) : value((uint16_t)val) {}
        operator uint16_t() const { return value; }
        ValidFlags& operator=(uint16_t val) { value = val; return *this; }
        ValidFlags& operator=(int val) { value = uint16_t(val); return *this; }
        ValidFlags& operator|=(uint16_t val) { return *this = value | val; }
        ValidFlags& operator&=(uint16_t val) { return *this = value & val; }
        
        bool allSet() const { return value == ALL; }
        void setAll() { value |= ALL; }
    };
    /// Parameters
    float gdop = 0; ///< Geometric DOP
    float pdop = 0; ///< Position DOP
    float hdop = 0; ///< Horizontal DOP
    float vdop = 0; ///< Vertical DOP
    float tdop = 0; ///< Time DOP
    float ndop = 0; ///< Northing DOP
    float edop = 0; ///< Easting DOP
    ValidFlags valid_flags;
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_gnss::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_gnss::DATA_DOP;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "Dop";
    static constexpr const char* DOC_NAME = "Dop";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(gdop,pdop,hdop,vdop,tdop,ndop,edop,valid_flags);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(gdop),std::ref(pdop),std::ref(hdop),std::ref(vdop),std::ref(tdop),std::ref(ndop),std::ref(edop),std::ref(valid_flags));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_gnss_utc_time  (0x81,0x08) Utc Time [CPP]
/// GNSS reported Coordinated Universal Time
///
///@{

struct UtcTime
{
    struct ValidFlags : Bitfield<ValidFlags>
    {
        typedef uint16_t Type;
        enum _enumType : uint16_t
        {
            NONE               = 0x0000,
            GNSS_DATE_TIME     = 0x0001,  ///<  
            LEAP_SECONDS_KNOWN = 0x0002,  ///<  
            FLAGS              = 0x0003,  ///<  
            ALL                = 0x0003,
        };
        uint16_t value = NONE;
        
        ValidFlags() : value(NONE) {}
        ValidFlags(int val) : value((uint16_t)val) {}
        operator uint16_t() const { return value; }
        ValidFlags& operator=(uint16_t val) { value = val; return *this; }
        ValidFlags& operator=(int val) { value = uint16_t(val); return *this; }
        ValidFlags& operator|=(uint16_t val) { return *this = value | val; }
        ValidFlags& operator&=(uint16_t val) { return *this = value & val; }
        
        bool allSet() const { return value == ALL; }
        void setAll() { value |= ALL; }
    };
    /// Parameters
    uint16_t year = 0;
    uint8_t month = 0; ///< Month (1-12)
    uint8_t day = 0; ///< Day (1-31)
    uint8_t hour = 0; ///< Hour (0-23)
    uint8_t min = 0; ///< Minute (0-59)
    uint8_t sec = 0; ///< Second (0-59)
    uint32_t msec = 0; ///< Millisecond(0-999)
    ValidFlags valid_flags;
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_gnss::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_gnss::DATA_UTC_TIME;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "UtcTime";
    static constexpr const char* DOC_NAME = "UtcTime";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(year,month,day,hour,min,sec,msec,valid_flags);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(year),std::ref(month),std::ref(day),std::ref(hour),std::ref(min),std::ref(sec),std::ref(msec),std::ref(valid_flags));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_gnss_gps_time  (0x81,0x09) Gps Time [CPP]
/// GNSS reported GPS Time
///
///@{

struct GpsTime
{
    struct ValidFlags : Bitfield<ValidFlags>
    {
        typedef uint16_t Type;
        enum _enumType : uint16_t
        {
            NONE        = 0x0000,
            TOW         = 0x0001,  ///<  
            WEEK_NUMBER = 0x0002,  ///<  
            FLAGS       = 0x0003,  ///<  
            ALL         = 0x0003,
        };
        uint16_t value = NONE;
        
        ValidFlags() : value(NONE) {}
        ValidFlags(int val) : value((uint16_t)val) {}
        operator uint16_t() const { return value; }
        ValidFlags& operator=(uint16_t val) { value = val; return *this; }
        ValidFlags& operator=(int val) { value = uint16_t(val); return *this; }
        ValidFlags& operator|=(uint16_t val) { return *this = value | val; }
        ValidFlags& operator&=(uint16_t val) { return *this = value & val; }
        
        bool allSet() const { return value == ALL; }
        void setAll() { value |= ALL; }
    };
    /// Parameters
    double tow = 0; ///< GPS Time of week [seconds]
    uint16_t week_number = 0; ///< GPS Week since 1980 [weeks]
    ValidFlags valid_flags;
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_gnss::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_gnss::DATA_GPS_TIME;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "GpsTime";
    static constexpr const char* DOC_NAME = "GpsTime";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(tow,week_number,valid_flags);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(tow),std::ref(week_number),std::ref(valid_flags));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_gnss_clock_info  (0x81,0x0A) Clock Info [CPP]
/// GNSS reported receiver clock parameters
///
///@{

struct ClockInfo
{
    struct ValidFlags : Bitfield<ValidFlags>
    {
        typedef uint16_t Type;
        enum _enumType : uint16_t
        {
            NONE              = 0x0000,
            BIAS              = 0x0001,  ///<  
            DRIFT             = 0x0002,  ///<  
            ACCURACY_ESTIMATE = 0x0004,  ///<  
            FLAGS             = 0x0007,  ///<  
            ALL               = 0x0007,
        };
        uint16_t value = NONE;
        
        ValidFlags() : value(NONE) {}
        ValidFlags(int val) : value((uint16_t)val) {}
        operator uint16_t() const { return value; }
        ValidFlags& operator=(uint16_t val) { value = val; return *this; }
        ValidFlags& operator=(int val) { value = uint16_t(val); return *this; }
        ValidFlags& operator|=(uint16_t val) { return *this = value | val; }
        ValidFlags& operator&=(uint16_t val) { return *this = value & val; }
        
        bool allSet() const { return value == ALL; }
        void setAll() { value |= ALL; }
    };
    /// Parameters
    double bias = 0; ///< [seconds]
    double drift = 0; ///< [seconds/second]
    double accuracy_estimate = 0; ///< [seconds]
    ValidFlags valid_flags;
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_gnss::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_gnss::DATA_CLOCK_INFO;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "ClockInfo";
    static constexpr const char* DOC_NAME = "ClockInfo";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(bias,drift,accuracy_estimate,valid_flags);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(bias),std::ref(drift),std::ref(accuracy_estimate),std::ref(valid_flags));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_gnss_fix_info  (0x81,0x0B) Fix Info [CPP]
/// GNSS reported position fix type
///
///@{

struct FixInfo
{
    enum class FixType : uint8_t
    {
        FIX_3D           = 0,  ///<  
        FIX_2D           = 1,  ///<  
        FIX_TIME_ONLY    = 2,  ///<  
        FIX_NONE         = 3,  ///<  
        FIX_INVALID      = 4,  ///<  
        FIX_RTK_FLOAT    = 5,  ///<  
        FIX_RTK_FIXED    = 6,  ///<  
        FIX_DIFFERENTIAL = 7,  ///<  
    };
    
    struct FixFlags : Bitfield<FixFlags>
    {
        typedef uint16_t Type;
        enum _enumType : uint16_t
        {
            NONE       = 0x0000,
            SBAS_USED  = 0x0001,  ///<  
            DGNSS_USED = 0x0002,  ///<  
            ALL        = 0x0003,
        };
        uint16_t value = NONE;
        
        FixFlags() : value(NONE) {}
        FixFlags(int val) : value((uint16_t)val) {}
        operator uint16_t() const { return value; }
        FixFlags& operator=(uint16_t val) { value = val; return *this; }
        FixFlags& operator=(int val) { value = uint16_t(val); return *this; }
        FixFlags& operator|=(uint16_t val) { return *this = value | val; }
        FixFlags& operator&=(uint16_t val) { return *this = value & val; }
        
        bool allSet() const { return value == ALL; }
        void setAll() { value |= ALL; }
    };
    struct ValidFlags : Bitfield<ValidFlags>
    {
        typedef uint16_t Type;
        enum _enumType : uint16_t
        {
            NONE      = 0x0000,
            FIX_TYPE  = 0x0001,  ///<  
            NUM_SV    = 0x0002,  ///<  
            FIX_FLAGS = 0x0004,  ///<  
            FLAGS     = 0x0007,  ///<  
            ALL       = 0x0007,
        };
        uint16_t value = NONE;
        
        ValidFlags() : value(NONE) {}
        ValidFlags(int val) : value((uint16_t)val) {}
        operator uint16_t() const { return value; }
        ValidFlags& operator=(uint16_t val) { value = val; return *this; }
        ValidFlags& operator=(int val) { value = uint16_t(val); return *this; }
        ValidFlags& operator|=(uint16_t val) { return *this = value | val; }
        ValidFlags& operator&=(uint16_t val) { return *this = value & val; }
        
        bool allSet() const { return value == ALL; }
        void setAll() { value |= ALL; }
    };
    /// Parameters
    FixType fix_type = static_cast<FixType>(0);
    uint8_t num_sv = 0;
    FixFlags fix_flags;
    ValidFlags valid_flags;
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_gnss::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_gnss::DATA_FIX_INFO;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "FixInfo";
    static constexpr const char* DOC_NAME = "FixInfo";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(fix_type,num_sv,fix_flags,valid_flags);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(fix_type),std::ref(num_sv),std::ref(fix_flags),std::ref(valid_flags));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_gnss_sv_info  (0x81,0x0C) Sv Info [CPP]
/// GNSS reported space vehicle information
/// 
/// When enabled, these fields will arrive in separate MIP packets
///
///@{

struct SvInfo
{
    struct SVFlags : Bitfield<SVFlags>
    {
        typedef uint16_t Type;
        enum _enumType : uint16_t
        {
            NONE                = 0x0000,
            USED_FOR_NAVIGATION = 0x0001,  ///<  
            HEALTHY             = 0x0002,  ///<  
            ALL                 = 0x0003,
        };
        uint16_t value = NONE;
        
        SVFlags() : value(NONE) {}
        SVFlags(int val) : value((uint16_t)val) {}
        operator uint16_t() const { return value; }
        SVFlags& operator=(uint16_t val) { value = val; return *this; }
        SVFlags& operator=(int val) { value = uint16_t(val); return *this; }
        SVFlags& operator|=(uint16_t val) { return *this = value | val; }
        SVFlags& operator&=(uint16_t val) { return *this = value & val; }
        
        bool allSet() const { return value == ALL; }
        void setAll() { value |= ALL; }
    };
    struct ValidFlags : Bitfield<ValidFlags>
    {
        typedef uint16_t Type;
        enum _enumType : uint16_t
        {
            NONE                = 0x0000,
            CHANNEL             = 0x0001,  ///<  
            SV_ID               = 0x0002,  ///<  
            CARRIER_NOISE_RATIO = 0x0004,  ///<  
            AZIMUTH             = 0x0008,  ///<  
            ELEVATION           = 0x0010,  ///<  
            SV_FLAGS            = 0x0020,  ///<  
            FLAGS               = 0x003F,  ///<  
            ALL                 = 0x003F,
        };
        uint16_t value = NONE;
        
        ValidFlags() : value(NONE) {}
        ValidFlags(int val) : value((uint16_t)val) {}
        operator uint16_t() const { return value; }
        ValidFlags& operator=(uint16_t val) { value = val; return *this; }
        ValidFlags& operator=(int val) { value = uint16_t(val); return *this; }
        ValidFlags& operator|=(uint16_t val) { return *this = value | val; }
        ValidFlags& operator&=(uint16_t val) { return *this = value & val; }
        
        bool allSet() const { return value == ALL; }
        void setAll() { value |= ALL; }
    };
    /// Parameters
    uint8_t channel = 0; ///< Receiver channel number
    uint8_t sv_id = 0; ///< GNSS Satellite ID
    uint16_t carrier_noise_ratio = 0; ///< [dBHz]
    int16_t azimuth = 0; ///< [deg]
    int16_t elevation = 0; ///< [deg]
    SVFlags sv_flags;
    ValidFlags valid_flags;
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_gnss::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_gnss::DATA_SV_INFO;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "SvInfo";
    static constexpr const char* DOC_NAME = "SvInfo";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(channel,sv_id,carrier_noise_ratio,azimuth,elevation,sv_flags,valid_flags);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(channel),std::ref(sv_id),std::ref(carrier_noise_ratio),std::ref(azimuth),std::ref(elevation),std::ref(sv_flags),std::ref(valid_flags));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_gnss_hw_status  (0x81,0x0D) Hw Status [CPP]
/// GNSS reported hardware status
///
///@{

struct HwStatus
{
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
    
    struct ValidFlags : Bitfield<ValidFlags>
    {
        typedef uint16_t Type;
        enum _enumType : uint16_t
        {
            NONE          = 0x0000,
            SENSOR_STATE  = 0x0001,  ///<  
            ANTENNA_STATE = 0x0002,  ///<  
            ANTENNA_POWER = 0x0004,  ///<  
            FLAGS         = 0x0007,  ///<  
            ALL           = 0x0007,
        };
        uint16_t value = NONE;
        
        ValidFlags() : value(NONE) {}
        ValidFlags(int val) : value((uint16_t)val) {}
        operator uint16_t() const { return value; }
        ValidFlags& operator=(uint16_t val) { value = val; return *this; }
        ValidFlags& operator=(int val) { value = uint16_t(val); return *this; }
        ValidFlags& operator|=(uint16_t val) { return *this = value | val; }
        ValidFlags& operator&=(uint16_t val) { return *this = value & val; }
        
        bool allSet() const { return value == ALL; }
        void setAll() { value |= ALL; }
    };
    /// Parameters
    ReceiverState receiver_state = static_cast<ReceiverState>(0);
    AntennaState antenna_state = static_cast<AntennaState>(0);
    AntennaPower antenna_power = static_cast<AntennaPower>(0);
    ValidFlags valid_flags;
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_gnss::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_gnss::DATA_HW_STATUS;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "HwStatus";
    static constexpr const char* DOC_NAME = "GNSS Hardware Status";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(receiver_state,antenna_state,antenna_power,valid_flags);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(receiver_state),std::ref(antenna_state),std::ref(antenna_power),std::ref(valid_flags));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_gnss_dgps_info  (0x81,0x0E) Dgps Info [CPP]
/// GNSS reported DGNSS status
/// 
/// <pre>Possible Base Station Status Values:</pre>
/// <pre>  0 - UDRE Scale Factor = 1.0</pre>
/// <pre>  1 - UDRE Scale Factor = 0.75</pre>
/// <pre>  2 - UDRE Scale Factor = 0.5</pre>
/// <pre>  3 - UDRE Scale Factor = 0.3</pre>
/// <pre>  4 - UDRE Scale Factor = 0.2</pre>
/// <pre>  5 - UDRE Scale Factor = 0.1</pre>
/// <pre>  6 - Reference Station Transmission Not Monitored</pre>
/// <pre>  7 - Reference Station Not Working</pre>
/// 
/// (UDRE = User Differential Range Error)
///
///@{

struct DgpsInfo
{
    struct ValidFlags : Bitfield<ValidFlags>
    {
        typedef uint16_t Type;
        enum _enumType : uint16_t
        {
            NONE                = 0x0000,
            AGE                 = 0x0001,  ///<  
            BASE_STATION_ID     = 0x0002,  ///<  
            BASE_STATION_STATUS = 0x0004,  ///<  
            NUM_CHANNELS        = 0x0008,  ///<  
            FLAGS               = 0x000F,  ///<  
            ALL                 = 0x000F,
        };
        uint16_t value = NONE;
        
        ValidFlags() : value(NONE) {}
        ValidFlags(int val) : value((uint16_t)val) {}
        operator uint16_t() const { return value; }
        ValidFlags& operator=(uint16_t val) { value = val; return *this; }
        ValidFlags& operator=(int val) { value = uint16_t(val); return *this; }
        ValidFlags& operator|=(uint16_t val) { return *this = value | val; }
        ValidFlags& operator&=(uint16_t val) { return *this = value & val; }
        
        bool allSet() const { return value == ALL; }
        void setAll() { value |= ALL; }
    };
    /// Parameters
    uint8_t sv_id = 0;
    float age = 0;
    float range_correction = 0;
    float range_rate_correction = 0;
    ValidFlags valid_flags;
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_gnss::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_gnss::DATA_DGPS_INFO;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "DgpsInfo";
    static constexpr const char* DOC_NAME = "DgpsInfo";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(sv_id,age,range_correction,range_rate_correction,valid_flags);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(sv_id),std::ref(age),std::ref(range_correction),std::ref(range_rate_correction),std::ref(valid_flags));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_gnss_dgps_channel  (0x81,0x0F) Dgps Channel [CPP]
/// GNSS reported DGPS Channel Status status
/// 
/// When enabled, a separate field for each active space vehicle will be sent in the packet.
///
///@{

struct DgpsChannel
{
    struct ValidFlags : Bitfield<ValidFlags>
    {
        typedef uint16_t Type;
        enum _enumType : uint16_t
        {
            NONE                  = 0x0000,
            ID                    = 0x0001,  ///<  
            AGE                   = 0x0002,  ///<  
            RANGE_CORRECTION      = 0x0004,  ///<  
            RANGE_RATE_CORRECTION = 0x0008,  ///<  
            FLAGS                 = 0x000F,  ///<  
            ALL                   = 0x000F,
        };
        uint16_t value = NONE;
        
        ValidFlags() : value(NONE) {}
        ValidFlags(int val) : value((uint16_t)val) {}
        operator uint16_t() const { return value; }
        ValidFlags& operator=(uint16_t val) { value = val; return *this; }
        ValidFlags& operator=(int val) { value = uint16_t(val); return *this; }
        ValidFlags& operator|=(uint16_t val) { return *this = value | val; }
        ValidFlags& operator&=(uint16_t val) { return *this = value & val; }
        
        bool allSet() const { return value == ALL; }
        void setAll() { value |= ALL; }
    };
    /// Parameters
    uint8_t sv_id = 0;
    float age = 0; ///< [s]
    float range_correction = 0; ///< [m]
    float range_rate_correction = 0; ///< [m/s]
    ValidFlags valid_flags;
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_gnss::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_gnss::DATA_DGPS_CHANNEL_STATUS;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "DgpsChannel";
    static constexpr const char* DOC_NAME = "DgpsChannel";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(sv_id,age,range_correction,range_rate_correction,valid_flags);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(sv_id),std::ref(age),std::ref(range_correction),std::ref(range_rate_correction),std::ref(valid_flags));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_gnss_clock_info_2  (0x81,0x10) Clock Info 2 [CPP]
/// GNSS reported receiver clock parameters
/// 
/// This supersedes MIP_DATA_DESC_GNSS_CLOCK_INFO with additional information.
///
///@{

struct ClockInfo2
{
    struct ValidFlags : Bitfield<ValidFlags>
    {
        typedef uint16_t Type;
        enum _enumType : uint16_t
        {
            NONE           = 0x0000,
            BIAS           = 0x0001,  ///<  
            DRIFT          = 0x0002,  ///<  
            BIAS_ACCURACY  = 0x0004,  ///<  
            DRIFT_ACCURACY = 0x0008,  ///<  
            FLAGS          = 0x000F,  ///<  
            ALL            = 0x000F,
        };
        uint16_t value = NONE;
        
        ValidFlags() : value(NONE) {}
        ValidFlags(int val) : value((uint16_t)val) {}
        operator uint16_t() const { return value; }
        ValidFlags& operator=(uint16_t val) { value = val; return *this; }
        ValidFlags& operator=(int val) { value = uint16_t(val); return *this; }
        ValidFlags& operator|=(uint16_t val) { return *this = value | val; }
        ValidFlags& operator&=(uint16_t val) { return *this = value & val; }
        
        bool allSet() const { return value == ALL; }
        void setAll() { value |= ALL; }
    };
    /// Parameters
    double bias = 0;
    double drift = 0;
    double bias_accuracy_estimate = 0;
    double drift_accuracy_estimate = 0;
    ValidFlags valid_flags;
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_gnss::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_gnss::DATA_CLOCK_INFO_2;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "ClockInfo2";
    static constexpr const char* DOC_NAME = "ClockInfo2";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(bias,drift,bias_accuracy_estimate,drift_accuracy_estimate,valid_flags);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(bias),std::ref(drift),std::ref(bias_accuracy_estimate),std::ref(drift_accuracy_estimate),std::ref(valid_flags));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_gnss_gps_leap_seconds  (0x81,0x11) Gps Leap Seconds [CPP]
/// GNSS reported leap seconds (difference between GPS and UTC Time)
///
///@{

struct GpsLeapSeconds
{
    struct ValidFlags : Bitfield<ValidFlags>
    {
        typedef uint16_t Type;
        enum _enumType : uint16_t
        {
            NONE         = 0x0000,
            LEAP_SECONDS = 0x0002,  ///<  
            ALL          = 0x0002,
        };
        uint16_t value = NONE;
        
        ValidFlags() : value(NONE) {}
        ValidFlags(int val) : value((uint16_t)val) {}
        operator uint16_t() const { return value; }
        ValidFlags& operator=(uint16_t val) { value = val; return *this; }
        ValidFlags& operator=(int val) { value = uint16_t(val); return *this; }
        ValidFlags& operator|=(uint16_t val) { return *this = value | val; }
        ValidFlags& operator&=(uint16_t val) { return *this = value & val; }
        
        bool allSet() const { return value == ALL; }
        void setAll() { value |= ALL; }
    };
    /// Parameters
    uint8_t leap_seconds = 0; ///< [s]
    ValidFlags valid_flags;
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_gnss::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_gnss::DATA_GPS_LEAP_SECONDS;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "GpsLeapSeconds";
    static constexpr const char* DOC_NAME = "GpsLeapSeconds";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(leap_seconds,valid_flags);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(leap_seconds),std::ref(valid_flags));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_gnss_sbas_info  (0x81,0x12) Sbas Info [CPP]
/// GNSS SBAS status
///
///@{

struct SbasInfo
{
    struct SbasStatus : Bitfield<SbasStatus>
    {
        typedef uint8_t Type;
        enum _enumType : uint8_t
        {
            NONE                  = 0x00,
            RANGE_AVAILABLE       = 0x01,  ///<  
            CORRECTIONS_AVAILABLE = 0x02,  ///<  
            INTEGRITY_AVAILABLE   = 0x04,  ///<  
            TEST_MODE             = 0x08,  ///<  
            ALL                   = 0x0F,
        };
        uint8_t value = NONE;
        
        SbasStatus() : value(NONE) {}
        SbasStatus(int val) : value((uint8_t)val) {}
        operator uint8_t() const { return value; }
        SbasStatus& operator=(uint8_t val) { value = val; return *this; }
        SbasStatus& operator=(int val) { value = uint8_t(val); return *this; }
        SbasStatus& operator|=(uint8_t val) { return *this = value | val; }
        SbasStatus& operator&=(uint8_t val) { return *this = value & val; }
        
        bool allSet() const { return value == ALL; }
        void setAll() { value |= ALL; }
    };
    struct ValidFlags : Bitfield<ValidFlags>
    {
        typedef uint16_t Type;
        enum _enumType : uint16_t
        {
            NONE        = 0x0000,
            TOW         = 0x0001,  ///<  
            WEEK_NUMBER = 0x0002,  ///<  
            SBAS_SYSTEM = 0x0004,  ///<  
            SBAS_ID     = 0x0008,  ///<  
            COUNT       = 0x0010,  ///<  
            SBAS_STATUS = 0x0020,  ///<  
            FLAGS       = 0x003F,  ///<  
            ALL         = 0x003F,
        };
        uint16_t value = NONE;
        
        ValidFlags() : value(NONE) {}
        ValidFlags(int val) : value((uint16_t)val) {}
        operator uint16_t() const { return value; }
        ValidFlags& operator=(uint16_t val) { value = val; return *this; }
        ValidFlags& operator=(int val) { value = uint16_t(val); return *this; }
        ValidFlags& operator|=(uint16_t val) { return *this = value | val; }
        ValidFlags& operator&=(uint16_t val) { return *this = value & val; }
        
        bool allSet() const { return value == ALL; }
        void setAll() { value |= ALL; }
    };
    /// Parameters
    double time_of_week = 0; ///< GPS Time of week [seconds]
    uint16_t week_number = 0; ///< GPS Week since 1980 [weeks]
    SbasSystem sbas_system = static_cast<SbasSystem>(0); ///< SBAS system id
    uint8_t sbas_id = 0; ///< SBAS satellite id.
    uint8_t count = 0; ///< Number of SBAS corrections
    SbasStatus sbas_status; ///< Status of the SBAS service
    ValidFlags valid_flags;
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_gnss::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_gnss::DATA_SBAS_INFO;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "SbasInfo";
    static constexpr const char* DOC_NAME = "SbasInfo";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(time_of_week,week_number,sbas_system,sbas_id,count,sbas_status,valid_flags);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(time_of_week),std::ref(week_number),std::ref(sbas_system),std::ref(sbas_id),std::ref(count),std::ref(sbas_status),std::ref(valid_flags));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_gnss_sbas_correction  (0x81,0x13) Sbas Correction [CPP]
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
    struct ValidFlags : Bitfield<ValidFlags>
    {
        typedef uint16_t Type;
        enum _enumType : uint16_t
        {
            NONE                   = 0x0000,
            UDREI                  = 0x0001,  ///<  
            PSEUDORANGE_CORRECTION = 0x0002,  ///<  
            IONO_CORRECTION        = 0x0004,  ///<  
            FLAGS                  = 0x0007,  ///<  
            ALL                    = 0x0007,
        };
        uint16_t value = NONE;
        
        ValidFlags() : value(NONE) {}
        ValidFlags(int val) : value((uint16_t)val) {}
        operator uint16_t() const { return value; }
        ValidFlags& operator=(uint16_t val) { value = val; return *this; }
        ValidFlags& operator=(int val) { value = uint16_t(val); return *this; }
        ValidFlags& operator|=(uint16_t val) { return *this = value | val; }
        ValidFlags& operator&=(uint16_t val) { return *this = value & val; }
        
        bool allSet() const { return value == ALL; }
        void setAll() { value |= ALL; }
    };
    /// Parameters
    uint8_t index = 0; ///< Index of this field in this epoch.
    uint8_t count = 0; ///< Total number of fields in this epoch.
    double time_of_week = 0; ///< GPS Time of week the message was received [seconds]
    uint16_t week_number = 0; ///< GPS Week since 1980 [weeks]
    GnssConstellationId gnss_id = static_cast<GnssConstellationId>(0); ///< GNSS constellation id
    uint8_t sv_id = 0; ///< GNSS satellite id within the constellation.
    uint8_t udrei = 0; ///< [See above 0-13 usable, 14 not monitored, 15 - do not use]
    float pseudorange_correction = 0; ///< Pseudo-range correction [meters].
    float iono_correction = 0; ///< Ionospheric correction [meters].
    ValidFlags valid_flags;
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_gnss::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_gnss::DATA_SBAS_CORRECTION;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "SbasCorrection";
    static constexpr const char* DOC_NAME = "SbasCorrection";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(index,count,time_of_week,week_number,gnss_id,sv_id,udrei,pseudorange_correction,iono_correction,valid_flags);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(index),std::ref(count),std::ref(time_of_week),std::ref(week_number),std::ref(gnss_id),std::ref(sv_id),std::ref(udrei),std::ref(pseudorange_correction),std::ref(iono_correction),std::ref(valid_flags));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_gnss_rf_error_detection  (0x81,0x14) Rf Error Detection [CPP]
/// GNSS Error Detection subsystem status
///
///@{

struct RfErrorDetection
{
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
    
    struct ValidFlags : Bitfield<ValidFlags>
    {
        typedef uint16_t Type;
        enum _enumType : uint16_t
        {
            NONE           = 0x0000,
            RF_BAND        = 0x0001,  ///<  
            JAMMING_STATE  = 0x0002,  ///<  
            SPOOFING_STATE = 0x0004,  ///<  
            FLAGS          = 0x0007,  ///<  
            ALL            = 0x0007,
        };
        uint16_t value = NONE;
        
        ValidFlags() : value(NONE) {}
        ValidFlags(int val) : value((uint16_t)val) {}
        operator uint16_t() const { return value; }
        ValidFlags& operator=(uint16_t val) { value = val; return *this; }
        ValidFlags& operator=(int val) { value = uint16_t(val); return *this; }
        ValidFlags& operator|=(uint16_t val) { return *this = value | val; }
        ValidFlags& operator&=(uint16_t val) { return *this = value & val; }
        
        bool allSet() const { return value == ALL; }
        void setAll() { value |= ALL; }
    };
    /// Parameters
    RFBand rf_band = static_cast<RFBand>(0); ///< RF Band of the reported information
    JammingState jamming_state = static_cast<JammingState>(0); ///< GNSS Jamming State (as reported by the GNSS module)
    SpoofingState spoofing_state = static_cast<SpoofingState>(0); ///< GNSS Spoofing State (as reported by the GNSS module)
    uint8_t reserved[4] = {0}; ///< Reserved for future use
    ValidFlags valid_flags;
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_gnss::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_gnss::DATA_RF_ERROR_DETECTION;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "RfErrorDetection";
    static constexpr const char* DOC_NAME = "RfErrorDetection";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(rf_band,jamming_state,spoofing_state,reserved,valid_flags);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(rf_band),std::ref(jamming_state),std::ref(spoofing_state),std::ref(reserved),std::ref(valid_flags));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_gnss_base_station_info  (0x81,0x30) Base Station Info [CPP]
/// RTCM reported base station information (sourced from RTCM Message 1005 or 1006)
/// 
/// Valid Flag Mapping:
///
///@{

struct BaseStationInfo
{
    struct IndicatorFlags : Bitfield<IndicatorFlags>
    {
        typedef uint16_t Type;
        enum _enumType : uint16_t
        {
            NONE               = 0x0000,
            GPS                = 0x0001,  ///<  
            GLONASS            = 0x0002,  ///<  
            GALILEO            = 0x0004,  ///<  
            BEIDOU             = 0x0008,  ///<  
            REF_STATION        = 0x0010,  ///<  
            SINGLE_RECEIVER    = 0x0020,  ///<  
            QUARTER_CYCLE_BIT1 = 0x0040,  ///<  
            QUARTER_CYCLE_BIT2 = 0x0080,  ///<  
            QUARTER_CYCLE_BITS = 0x00C0,  ///<  
            ALL                = 0x00FF,
        };
        uint16_t value = NONE;
        
        IndicatorFlags() : value(NONE) {}
        IndicatorFlags(int val) : value((uint16_t)val) {}
        operator uint16_t() const { return value; }
        IndicatorFlags& operator=(uint16_t val) { value = val; return *this; }
        IndicatorFlags& operator=(int val) { value = uint16_t(val); return *this; }
        IndicatorFlags& operator|=(uint16_t val) { return *this = value | val; }
        IndicatorFlags& operator&=(uint16_t val) { return *this = value & val; }
        
        bool allSet() const { return value == ALL; }
        void setAll() { value |= ALL; }
    };
    struct ValidFlags : Bitfield<ValidFlags>
    {
        typedef uint16_t Type;
        enum _enumType : uint16_t
        {
            NONE          = 0x0000,
            TOW           = 0x0001,  ///<  
            WEEK_NUMBER   = 0x0002,  ///<  
            ECEF_POSITION = 0x0004,  ///<  
            HEIGHT        = 0x0008,  ///<  
            STATION_ID    = 0x0010,  ///<  
            INDICATORS    = 0x0020,  ///<  
            FLAGS         = 0x003F,  ///<  
            ALL           = 0x003F,
        };
        uint16_t value = NONE;
        
        ValidFlags() : value(NONE) {}
        ValidFlags(int val) : value((uint16_t)val) {}
        operator uint16_t() const { return value; }
        ValidFlags& operator=(uint16_t val) { value = val; return *this; }
        ValidFlags& operator=(int val) { value = uint16_t(val); return *this; }
        ValidFlags& operator|=(uint16_t val) { return *this = value | val; }
        ValidFlags& operator&=(uint16_t val) { return *this = value & val; }
        
        bool allSet() const { return value == ALL; }
        void setAll() { value |= ALL; }
    };
    /// Parameters
    double time_of_week = 0; ///< GPS Time of week the message was received [seconds]
    uint16_t week_number = 0; ///< GPS Week since 1980 [weeks]
    Vector3d ecef_pos; ///< Earth-centered, Earth-fixed [m]
    float height = 0; ///< Antenna Height above the marker used in the survey [m]
    uint16_t station_id = 0; ///< Range: 0-4095
    IndicatorFlags indicators; ///< Bitfield
    ValidFlags valid_flags;
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_gnss::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_gnss::DATA_BASE_STATION_INFO;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "BaseStationInfo";
    static constexpr const char* DOC_NAME = "BaseStationInfo";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(time_of_week,week_number,ecef_pos[0],ecef_pos[1],ecef_pos[2],height,station_id,indicators,valid_flags);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(time_of_week),std::ref(week_number),std::ref(ecef_pos[0]),std::ref(ecef_pos[1]),std::ref(ecef_pos[2]),std::ref(height),std::ref(station_id),std::ref(indicators),std::ref(valid_flags));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_gnss_rtk_corrections_status  (0x81,0x31) Rtk Corrections Status [CPP]
///
///@{

struct RtkCorrectionsStatus
{
    struct ValidFlags : Bitfield<ValidFlags>
    {
        typedef uint16_t Type;
        enum _enumType : uint16_t
        {
            NONE            = 0x0000,
            TOW             = 0x0001,  ///<  
            WEEK_NUMBER     = 0x0002,  ///<  
            EPOCH_STATUS    = 0x0004,  ///<  
            DONGLE_STATUS   = 0x0008,  ///<  
            GPS_LATENCY     = 0x0010,  ///<  
            GLONASS_LATENCY = 0x0020,  ///<  
            GALILEO_LATENCY = 0x0040,  ///<  
            BEIDOU_LATENCY  = 0x0080,  ///<  
            FLAGS           = 0x00FF,  ///<  
            ALL             = 0x00FF,
        };
        uint16_t value = NONE;
        
        ValidFlags() : value(NONE) {}
        ValidFlags(int val) : value((uint16_t)val) {}
        operator uint16_t() const { return value; }
        ValidFlags& operator=(uint16_t val) { value = val; return *this; }
        ValidFlags& operator=(int val) { value = uint16_t(val); return *this; }
        ValidFlags& operator|=(uint16_t val) { return *this = value | val; }
        ValidFlags& operator&=(uint16_t val) { return *this = value & val; }
        
        bool allSet() const { return value == ALL; }
        void setAll() { value |= ALL; }
    };
    struct EpochStatus : Bitfield<EpochStatus>
    {
        typedef uint16_t Type;
        enum _enumType : uint16_t
        {
            NONE                         = 0x0000,
            ANTENNA_LOCATION_RECEIVED    = 0x0001,  ///<  
            ANTENNA_DESCRIPTION_RECEIVED = 0x0002,  ///<  
            GPS_RECEIVED                 = 0x0004,  ///<  
            GLONASS_RECEIVED             = 0x0008,  ///<  
            GALILEO_RECEIVED             = 0x0010,  ///<  
            BEIDOU_RECEIVED              = 0x0020,  ///<  
            USING_GPS_MSM_MESSAGES       = 0x0040,  ///<  Using MSM messages for GPS corrections instead of RTCM messages 1001-1004
            USING_GLONASS_MSM_MESSAGES   = 0x0080,  ///<  Using MSM messages for GLONASS corrections instead of RTCM messages 1009-1012
            DONGLE_STATUS_READ_FAILED    = 0x0100,  ///<  A read of the dongle status was attempted, but failed
            ALL                          = 0x01FF,
        };
        uint16_t value = NONE;
        
        EpochStatus() : value(NONE) {}
        EpochStatus(int val) : value((uint16_t)val) {}
        operator uint16_t() const { return value; }
        EpochStatus& operator=(uint16_t val) { value = val; return *this; }
        EpochStatus& operator=(int val) { value = uint16_t(val); return *this; }
        EpochStatus& operator|=(uint16_t val) { return *this = value | val; }
        EpochStatus& operator&=(uint16_t val) { return *this = value & val; }
        
        bool allSet() const { return value == ALL; }
        void setAll() { value |= ALL; }
    };
    /// Parameters
    double time_of_week = 0; ///< GPS Time of week [seconds]
    uint16_t week_number = 0; ///< GPS Week since 1980 [weeks]
    EpochStatus epoch_status; ///< Status of the corrections received during this epoch
    uint32_t dongle_status = 0; ///< RTK Dongle Status Flags (valid only when using RTK dongle, see Get RTK Device Status Flags (0x0F,0x01) for details)
    float gps_correction_latency = 0; ///< Latency of last GPS correction [seconds]
    float glonass_correction_latency = 0; ///< Latency of last GLONASS correction [seconds]
    float galileo_correction_latency = 0; ///< Latency of last Galileo correction [seconds]
    float beidou_correction_latency = 0; ///< Latency of last Beidou correction [seconds]
    uint32_t reserved[4] = {0}; ///< Reserved for future use
    ValidFlags valid_flags;
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_gnss::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_gnss::DATA_RTK_CORRECTIONS_STATUS;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "RtkCorrectionsStatus";
    static constexpr const char* DOC_NAME = "RtkCorrectionsStatus";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(time_of_week,week_number,epoch_status,dongle_status,gps_correction_latency,glonass_correction_latency,galileo_correction_latency,beidou_correction_latency,reserved,valid_flags);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(time_of_week),std::ref(week_number),std::ref(epoch_status),std::ref(dongle_status),std::ref(gps_correction_latency),std::ref(glonass_correction_latency),std::ref(galileo_correction_latency),std::ref(beidou_correction_latency),std::ref(reserved),std::ref(valid_flags));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_gnss_satellite_status  (0x81,0x20) Satellite Status [CPP]
/// Status information for a GNSS satellite.
///
///@{

struct SatelliteStatus
{
    struct ValidFlags : Bitfield<ValidFlags>
    {
        typedef uint16_t Type;
        enum _enumType : uint16_t
        {
            NONE         = 0x0000,
            TOW          = 0x0001,  ///<  
            WEEK_NUMBER  = 0x0002,  ///<  
            GNSS_ID      = 0x0004,  ///<  
            SATELLITE_ID = 0x0008,  ///<  
            ELEVATION    = 0x0010,  ///<  
            AZIMUTH      = 0x0020,  ///<  
            HEALTH       = 0x0040,  ///<  
            FLAGS        = 0x007F,  ///<  
            ALL          = 0x007F,
        };
        uint16_t value = NONE;
        
        ValidFlags() : value(NONE) {}
        ValidFlags(int val) : value((uint16_t)val) {}
        operator uint16_t() const { return value; }
        ValidFlags& operator=(uint16_t val) { value = val; return *this; }
        ValidFlags& operator=(int val) { value = uint16_t(val); return *this; }
        ValidFlags& operator|=(uint16_t val) { return *this = value | val; }
        ValidFlags& operator&=(uint16_t val) { return *this = value & val; }
        
        bool allSet() const { return value == ALL; }
        void setAll() { value |= ALL; }
    };
    /// Parameters
    uint8_t index = 0; ///< Index of this field in this epoch.
    uint8_t count = 0; ///< Total number of fields in this epoch.
    double time_of_week = 0; ///< GPS Time of week [seconds]
    uint16_t week_number = 0; ///< GPS Week since 1980 [weeks]
    GnssConstellationId gnss_id = static_cast<GnssConstellationId>(0);
    uint8_t satellite_id = 0; ///< GNSS satellite id within the constellation
    float elevation = 0; ///< Elevation of the satellite relative to the rover [degrees]
    float azimuth = 0; ///< Azimuth of the satellite relative to the rover [degrees]
    bool health = 0; ///< True if the satellite is healthy.
    ValidFlags valid_flags;
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_gnss::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_gnss::DATA_SATELLITE_STATUS;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "SatelliteStatus";
    static constexpr const char* DOC_NAME = "SatelliteStatus";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(index,count,time_of_week,week_number,gnss_id,satellite_id,elevation,azimuth,health,valid_flags);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(index),std::ref(count),std::ref(time_of_week),std::ref(week_number),std::ref(gnss_id),std::ref(satellite_id),std::ref(elevation),std::ref(azimuth),std::ref(health),std::ref(valid_flags));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_gnss_raw  (0x81,0x22) Raw [CPP]
/// GNSS Raw observation.
///
///@{

struct Raw
{
    enum class GnssSignalQuality : uint8_t
    {
        NONE         = 0,  ///<  
        SEARCHING    = 1,  ///<  
        ACQUIRED     = 2,  ///<  
        UNUSABLE     = 3,  ///<  
        TIME_LOCKED  = 4,  ///<  
        FULLY_LOCKED = 5,  ///<  
    };
    
    struct ValidFlags : Bitfield<ValidFlags>
    {
        typedef uint16_t Type;
        enum _enumType : uint16_t
        {
            NONE                      = 0x0000,
            TOW                       = 0x0001,  ///<  
            WEEK_NUMBER               = 0x0002,  ///<  
            RECEIVER_ID               = 0x0004,  ///<  
            TRACKING_CHANNEL          = 0x0008,  ///<  
            GNSS_ID                   = 0x0010,  ///<  
            SATELLITE_ID              = 0x0020,  ///<  
            SIGNAL_ID                 = 0x0040,  ///<  
            SIGNAL_STRENGTH           = 0x0080,  ///<  
            QUALITY                   = 0x0100,  ///<  
            PSEUDORANGE               = 0x0200,  ///<  
            CARRIER_PHASE             = 0x0400,  ///<  
            DOPPLER                   = 0x0800,  ///<  
            RANGE_UNCERTAINTY         = 0x1000,  ///<  
            CARRIER_PHASE_UNCERTAINTY = 0x2000,  ///<  
            DOPPLER_UNCERTAINTY       = 0x4000,  ///<  
            LOCK_TIME                 = 0x8000,  ///<  
            FLAGS                     = 0xFFFF,  ///<  
            ALL                       = 0xFFFF,
        };
        uint16_t value = NONE;
        
        ValidFlags() : value(NONE) {}
        ValidFlags(int val) : value((uint16_t)val) {}
        operator uint16_t() const { return value; }
        ValidFlags& operator=(uint16_t val) { value = val; return *this; }
        ValidFlags& operator=(int val) { value = uint16_t(val); return *this; }
        ValidFlags& operator|=(uint16_t val) { return *this = value | val; }
        ValidFlags& operator&=(uint16_t val) { return *this = value & val; }
        
        bool allSet() const { return value == ALL; }
        void setAll() { value |= ALL; }
    };
    /// Parameters
    uint8_t index = 0; ///< Index of this field in this epoch.
    uint8_t count = 0; ///< Total number of fields in this epoch.
    double time_of_week = 0; ///< GPS Time of week [seconds]
    uint16_t week_number = 0; ///< GPS Week since 1980 [weeks]
    uint16_t receiver_id = 0; ///< When the measurement comes from RTCM, this will be the reference station ID; otherwise, it's the receiver number (1,2,...)
    uint8_t tracking_channel = 0; ///< Channel the receiver is using to track this satellite.
    GnssConstellationId gnss_id = static_cast<GnssConstellationId>(0);
    uint8_t satellite_id = 0; ///< GNSS satellite id within the constellation.
    GnssSignalId signal_id = static_cast<GnssSignalId>(0); ///< Signal identifier for the satellite.
    float signal_strength = 0; ///< Carrier to noise ratio [dBHz].
    GnssSignalQuality quality = static_cast<GnssSignalQuality>(0); ///< Indicator of signal quality.
    double pseudorange = 0; ///< Pseudo-range measurement [meters].
    double carrier_phase = 0; ///< Carrier phase measurement [Carrier periods].
    float doppler = 0; ///< Measured doppler shift [Hz].
    float range_uncert = 0; ///< Uncertainty of the pseudo-range measurement [m].
    float phase_uncert = 0; ///< Uncertainty of the phase measurement [Carrier periods].
    float doppler_uncert = 0; ///< Uncertainty of the measured doppler shift [Hz].
    float lock_time = 0; ///< DOC Minimum carrier phase lock time [s].  Note: the maximum value is dependent on the receiver.
    ValidFlags valid_flags;
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_gnss::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_gnss::DATA_RAW;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "Raw";
    static constexpr const char* DOC_NAME = "Raw";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(index,count,time_of_week,week_number,receiver_id,tracking_channel,gnss_id,satellite_id,signal_id,signal_strength,quality,pseudorange,carrier_phase,doppler,range_uncert,phase_uncert,doppler_uncert,lock_time,valid_flags);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(index),std::ref(count),std::ref(time_of_week),std::ref(week_number),std::ref(receiver_id),std::ref(tracking_channel),std::ref(gnss_id),std::ref(satellite_id),std::ref(signal_id),std::ref(signal_strength),std::ref(quality),std::ref(pseudorange),std::ref(carrier_phase),std::ref(doppler),std::ref(range_uncert),std::ref(phase_uncert),std::ref(doppler_uncert),std::ref(lock_time),std::ref(valid_flags));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_gnss_gps_ephemeris  (0x81,0x61) Gps Ephemeris [CPP]
/// GPS Ephemeris Data
///
///@{

struct GpsEphemeris
{
    struct ValidFlags : Bitfield<ValidFlags>
    {
        typedef uint16_t Type;
        enum _enumType : uint16_t
        {
            NONE        = 0x0000,
            EPHEMERIS   = 0x0001,  ///<  
            MODERN_DATA = 0x0002,  ///<  
            FLAGS       = 0x0003,  ///<  
            ALL         = 0x0003,
        };
        uint16_t value = NONE;
        
        ValidFlags() : value(NONE) {}
        ValidFlags(int val) : value((uint16_t)val) {}
        operator uint16_t() const { return value; }
        ValidFlags& operator=(uint16_t val) { value = val; return *this; }
        ValidFlags& operator=(int val) { value = uint16_t(val); return *this; }
        ValidFlags& operator|=(uint16_t val) { return *this = value | val; }
        ValidFlags& operator&=(uint16_t val) { return *this = value & val; }
        
        bool allSet() const { return value == ALL; }
        void setAll() { value |= ALL; }
    };
    /// Parameters
    uint8_t index = 0; ///< Index of this field in this epoch.
    uint8_t count = 0; ///< Total number of fields in this epoch.
    double time_of_week = 0; ///< GPS Time of week [seconds]
    uint16_t week_number = 0; ///< GPS Week since 1980 [weeks]
    uint8_t satellite_id = 0; ///< GNSS satellite id within the constellation.
    uint8_t health = 0; ///< Satellite and signal health
    uint8_t iodc = 0; ///< Issue of Data Clock. This increments each time the data changes and rolls over at 4. It is used to make sure various raw data elements from different sources line up correctly.
    uint8_t iode = 0; ///< Issue of Data Ephemeris.
    double t_oc = 0; ///< Reference time for clock data.
    double af0 = 0; ///< Clock bias in [s].
    double af1 = 0; ///< Clock drift in [s/s].
    double af2 = 0; ///< Clock drift rate in [s/s^2].
    double t_gd = 0; ///< T Group Delay [s].
    double ISC_L1CA = 0;
    double ISC_L2C = 0;
    double t_oe = 0; ///< Reference time for ephemeris in [s].
    double a = 0; ///< Semi-major axis [m].
    double a_dot = 0; ///< Semi-major axis rate [m/s].
    double mean_anomaly = 0; ///< [rad].
    double delta_mean_motion = 0; ///< [rad].
    double delta_mean_motion_dot = 0; ///< [rad/s].
    double eccentricity = 0;
    double argument_of_perigee = 0; ///< [rad].
    double omega = 0; ///< Longitude of Ascending Node [rad].
    double omega_dot = 0; ///< Rate of Right Ascension [rad/s].
    double inclination = 0; ///< Inclination angle [rad].
    double inclination_dot = 0; ///< Inclination angle rate of change [rad/s].
    double c_ic = 0; ///< Harmonic Correction Term.
    double c_is = 0; ///< Harmonic Correction Term.
    double c_uc = 0; ///< Harmonic Correction Term.
    double c_us = 0; ///< Harmonic Correction Term.
    double c_rc = 0; ///< Harmonic Correction Term.
    double c_rs = 0; ///< Harmonic Correction Term.
    ValidFlags valid_flags;
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_gnss::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_gnss::DATA_GPS_EPHEMERIS;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "GpsEphemeris";
    static constexpr const char* DOC_NAME = "GpsEphemeris";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(index,count,time_of_week,week_number,satellite_id,health,iodc,iode,t_oc,af0,af1,af2,t_gd,ISC_L1CA,ISC_L2C,t_oe,a,a_dot,mean_anomaly,delta_mean_motion,delta_mean_motion_dot,eccentricity,argument_of_perigee,omega,omega_dot,inclination,inclination_dot,c_ic,c_is,c_uc,c_us,c_rc,c_rs,valid_flags);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(index),std::ref(count),std::ref(time_of_week),std::ref(week_number),std::ref(satellite_id),std::ref(health),std::ref(iodc),std::ref(iode),std::ref(t_oc),std::ref(af0),std::ref(af1),std::ref(af2),std::ref(t_gd),std::ref(ISC_L1CA),std::ref(ISC_L2C),std::ref(t_oe),std::ref(a),std::ref(a_dot),std::ref(mean_anomaly),std::ref(delta_mean_motion),std::ref(delta_mean_motion_dot),std::ref(eccentricity),std::ref(argument_of_perigee),std::ref(omega),std::ref(omega_dot),std::ref(inclination),std::ref(inclination_dot),std::ref(c_ic),std::ref(c_is),std::ref(c_uc),std::ref(c_us),std::ref(c_rc),std::ref(c_rs),std::ref(valid_flags));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_gnss_galileo_ephemeris  (0x81,0x63) Galileo Ephemeris [CPP]
/// Galileo Ephemeris Data
///
///@{

struct GalileoEphemeris
{
    struct ValidFlags : Bitfield<ValidFlags>
    {
        typedef uint16_t Type;
        enum _enumType : uint16_t
        {
            NONE        = 0x0000,
            EPHEMERIS   = 0x0001,  ///<  
            MODERN_DATA = 0x0002,  ///<  
            FLAGS       = 0x0003,  ///<  
            ALL         = 0x0003,
        };
        uint16_t value = NONE;
        
        ValidFlags() : value(NONE) {}
        ValidFlags(int val) : value((uint16_t)val) {}
        operator uint16_t() const { return value; }
        ValidFlags& operator=(uint16_t val) { value = val; return *this; }
        ValidFlags& operator=(int val) { value = uint16_t(val); return *this; }
        ValidFlags& operator|=(uint16_t val) { return *this = value | val; }
        ValidFlags& operator&=(uint16_t val) { return *this = value & val; }
        
        bool allSet() const { return value == ALL; }
        void setAll() { value |= ALL; }
    };
    /// Parameters
    uint8_t index = 0; ///< Index of this field in this epoch.
    uint8_t count = 0; ///< Total number of fields in this epoch.
    double time_of_week = 0; ///< GPS Time of week [seconds]
    uint16_t week_number = 0; ///< GPS Week since 1980 [weeks]
    uint8_t satellite_id = 0; ///< GNSS satellite id within the constellation.
    uint8_t health = 0; ///< Satellite and signal health
    uint8_t iodc = 0; ///< Issue of Data Clock. This increments each time the data changes and rolls over at 4. It is used to make sure various raw data elements from different sources line up correctly.
    uint8_t iode = 0; ///< Issue of Data Ephemeris.
    double t_oc = 0; ///< Reference time for clock data.
    double af0 = 0; ///< Clock bias in [s].
    double af1 = 0; ///< Clock drift in [s/s].
    double af2 = 0; ///< Clock drift rate in [s/s^2].
    double t_gd = 0; ///< T Group Delay [s].
    double ISC_L1CA = 0;
    double ISC_L2C = 0;
    double t_oe = 0; ///< Reference time for ephemeris in [s].
    double a = 0; ///< Semi-major axis [m].
    double a_dot = 0; ///< Semi-major axis rate [m/s].
    double mean_anomaly = 0; ///< [rad].
    double delta_mean_motion = 0; ///< [rad].
    double delta_mean_motion_dot = 0; ///< [rad/s].
    double eccentricity = 0;
    double argument_of_perigee = 0; ///< [rad].
    double omega = 0; ///< Longitude of Ascending Node [rad].
    double omega_dot = 0; ///< Rate of Right Ascension [rad/s].
    double inclination = 0; ///< Inclination angle [rad].
    double inclination_dot = 0; ///< Inclination angle rate of change [rad/s].
    double c_ic = 0; ///< Harmonic Correction Term.
    double c_is = 0; ///< Harmonic Correction Term.
    double c_uc = 0; ///< Harmonic Correction Term.
    double c_us = 0; ///< Harmonic Correction Term.
    double c_rc = 0; ///< Harmonic Correction Term.
    double c_rs = 0; ///< Harmonic Correction Term.
    ValidFlags valid_flags;
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_gnss::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_gnss::DATA_GALILEO_EPHEMERIS;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "GalileoEphemeris";
    static constexpr const char* DOC_NAME = "GalileoEphemeris";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(index,count,time_of_week,week_number,satellite_id,health,iodc,iode,t_oc,af0,af1,af2,t_gd,ISC_L1CA,ISC_L2C,t_oe,a,a_dot,mean_anomaly,delta_mean_motion,delta_mean_motion_dot,eccentricity,argument_of_perigee,omega,omega_dot,inclination,inclination_dot,c_ic,c_is,c_uc,c_us,c_rc,c_rs,valid_flags);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(index),std::ref(count),std::ref(time_of_week),std::ref(week_number),std::ref(satellite_id),std::ref(health),std::ref(iodc),std::ref(iode),std::ref(t_oc),std::ref(af0),std::ref(af1),std::ref(af2),std::ref(t_gd),std::ref(ISC_L1CA),std::ref(ISC_L2C),std::ref(t_oe),std::ref(a),std::ref(a_dot),std::ref(mean_anomaly),std::ref(delta_mean_motion),std::ref(delta_mean_motion_dot),std::ref(eccentricity),std::ref(argument_of_perigee),std::ref(omega),std::ref(omega_dot),std::ref(inclination),std::ref(inclination_dot),std::ref(c_ic),std::ref(c_is),std::ref(c_uc),std::ref(c_us),std::ref(c_rc),std::ref(c_rs),std::ref(valid_flags));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_gnss_glo_ephemeris  (0x81,0x62) Glo Ephemeris [CPP]
/// Glonass Ephemeris Data
///
///@{

struct GloEphemeris
{
    struct ValidFlags : Bitfield<ValidFlags>
    {
        typedef uint16_t Type;
        enum _enumType : uint16_t
        {
            NONE      = 0x0000,
            EPHEMERIS = 0x0001,  ///<  
            FLAGS     = 0x0001,  ///<  
            ALL       = 0x0001,
        };
        uint16_t value = NONE;
        
        ValidFlags() : value(NONE) {}
        ValidFlags(int val) : value((uint16_t)val) {}
        operator uint16_t() const { return value; }
        ValidFlags& operator=(uint16_t val) { value = val; return *this; }
        ValidFlags& operator=(int val) { value = uint16_t(val); return *this; }
        ValidFlags& operator|=(uint16_t val) { return *this = value | val; }
        ValidFlags& operator&=(uint16_t val) { return *this = value & val; }
        
        bool allSet() const { return value == ALL; }
        void setAll() { value |= ALL; }
    };
    /// Parameters
    uint8_t index = 0; ///< Index of this field in this epoch.
    uint8_t count = 0; ///< Total number of fields in this epoch.
    double time_of_week = 0; ///< GPS Time of week [seconds]
    uint16_t week_number = 0; ///< GPS Week since 1980 [weeks]
    uint8_t satellite_id = 0; ///< GNSS satellite id within the constellation.
    int8_t freq_number = 0; ///< GLONASS frequency number (-7 to 24)
    uint32_t tk = 0; ///< Frame start time within current day [seconds]
    uint32_t tb = 0; ///< Ephemeris reference time [seconds]
    uint8_t sat_type = 0; ///< Type of satellite (M) GLONASS = 0, GLONASS-M = 1
    double gamma = 0; ///< Relative deviation of carrier frequency from nominal [dimensionless]
    double tau_n = 0; ///< Time correction relative to GLONASS Time [seconds]
    Vector3d x; ///< Satellite PE-90 position [m]
    Vector3f v; ///< Satellite PE-90 velocity [m/s]
    Vector3f a; ///< Satellite PE-90 acceleration due to perturbations [m/s^2]
    uint8_t health = 0; ///< Satellite Health (Bn), Non-zero indicates satellite malfunction
    uint8_t P = 0; ///< Satellite operation mode (See GLONASS ICD)
    uint8_t NT = 0; ///< Day number within a 4 year period.
    float delta_tau_n = 0; ///< Time difference between L1 and L2[m/s]
    uint8_t Ft = 0; ///< User Range Accuracy (See GLONASS ICD)
    uint8_t En = 0; ///< Age of current information [days]
    uint8_t P1 = 0; ///< Time interval between adjacent values of tb [minutes]
    uint8_t P2 = 0; ///< Oddness "1" or evenness "0" of the value of tb.
    uint8_t P3 = 0; ///< Number of satellites in almanac for this frame
    uint8_t P4 = 0; ///< Flag indicating ephemeris parameters are present
    ValidFlags valid_flags;
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_gnss::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_gnss::DATA_GLONASS_EPHEMERIS;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "GloEphemeris";
    static constexpr const char* DOC_NAME = "Glonass Ephemeris";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(index,count,time_of_week,week_number,satellite_id,freq_number,tk,tb,sat_type,gamma,tau_n,x[0],x[1],x[2],v[0],v[1],v[2],a[0],a[1],a[2],health,P,NT,delta_tau_n,Ft,En,P1,P2,P3,P4,valid_flags);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(index),std::ref(count),std::ref(time_of_week),std::ref(week_number),std::ref(satellite_id),std::ref(freq_number),std::ref(tk),std::ref(tb),std::ref(sat_type),std::ref(gamma),std::ref(tau_n),std::ref(x[0]),std::ref(x[1]),std::ref(x[2]),std::ref(v[0]),std::ref(v[1]),std::ref(v[2]),std::ref(a[0]),std::ref(a[1]),std::ref(a[2]),std::ref(health),std::ref(P),std::ref(NT),std::ref(delta_tau_n),std::ref(Ft),std::ref(En),std::ref(P1),std::ref(P2),std::ref(P3),std::ref(P4),std::ref(valid_flags));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_gnss_gps_iono_corr  (0x81,0x71) Gps Iono Corr [CPP]
/// Ionospheric Correction Terms for GNSS
///
///@{

struct GpsIonoCorr
{
    struct ValidFlags : Bitfield<ValidFlags>
    {
        typedef uint16_t Type;
        enum _enumType : uint16_t
        {
            NONE        = 0x0000,
            TOW         = 0x0001,  ///<  
            WEEK_NUMBER = 0x0002,  ///<  
            ALPHA       = 0x0004,  ///<  
            BETA        = 0x0008,  ///<  
            FLAGS       = 0x000F,  ///<  
            ALL         = 0x000F,
        };
        uint16_t value = NONE;
        
        ValidFlags() : value(NONE) {}
        ValidFlags(int val) : value((uint16_t)val) {}
        operator uint16_t() const { return value; }
        ValidFlags& operator=(uint16_t val) { value = val; return *this; }
        ValidFlags& operator=(int val) { value = uint16_t(val); return *this; }
        ValidFlags& operator|=(uint16_t val) { return *this = value | val; }
        ValidFlags& operator&=(uint16_t val) { return *this = value & val; }
        
        bool allSet() const { return value == ALL; }
        void setAll() { value |= ALL; }
    };
    /// Parameters
    double time_of_week = 0; ///< GPS Time of week [seconds]
    uint16_t week_number = 0; ///< GPS Week since 1980 [weeks]
    double alpha[4] = {0}; ///< Ionospheric Correction Terms.
    double beta[4] = {0}; ///< Ionospheric Correction Terms.
    ValidFlags valid_flags;
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_gnss::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_gnss::DATA_GPS_IONO_CORR;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "GpsIonoCorr";
    static constexpr const char* DOC_NAME = "GPS Ionospheric Correction";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(time_of_week,week_number,alpha,beta,valid_flags);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(time_of_week),std::ref(week_number),std::ref(alpha),std::ref(beta),std::ref(valid_flags));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_gnss_galileo_iono_corr  (0x81,0x73) Galileo Iono Corr [CPP]
/// Ionospheric Correction Terms for Galileo
///
///@{

struct GalileoIonoCorr
{
    struct ValidFlags : Bitfield<ValidFlags>
    {
        typedef uint16_t Type;
        enum _enumType : uint16_t
        {
            NONE              = 0x0000,
            TOW               = 0x0001,  ///<  
            WEEK_NUMBER       = 0x0002,  ///<  
            ALPHA             = 0x0004,  ///<  
            DISTURBANCE_FLAGS = 0x0008,  ///<  
            FLAGS             = 0x000F,  ///<  
            ALL               = 0x000F,
        };
        uint16_t value = NONE;
        
        ValidFlags() : value(NONE) {}
        ValidFlags(int val) : value((uint16_t)val) {}
        operator uint16_t() const { return value; }
        ValidFlags& operator=(uint16_t val) { value = val; return *this; }
        ValidFlags& operator=(int val) { value = uint16_t(val); return *this; }
        ValidFlags& operator|=(uint16_t val) { return *this = value | val; }
        ValidFlags& operator&=(uint16_t val) { return *this = value & val; }
        
        bool allSet() const { return value == ALL; }
        void setAll() { value |= ALL; }
    };
    /// Parameters
    double time_of_week = 0; ///< GPS Time of week [seconds]
    uint16_t week_number = 0; ///< GPS Week since 1980 [weeks]
    Vector3d alpha; ///< Coefficients for the model.
    uint8_t disturbance_flags = 0; ///< Region disturbance flags (bits 1-5).
    ValidFlags valid_flags;
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_gnss::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_gnss::DATA_GALILEO_IONO_CORR;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "GalileoIonoCorr";
    static constexpr const char* DOC_NAME = "Galileo Ionospheric Correction";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(time_of_week,week_number,alpha[0],alpha[1],alpha[2],disturbance_flags,valid_flags);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(time_of_week),std::ref(week_number),std::ref(alpha[0]),std::ref(alpha[1]),std::ref(alpha[2]),std::ref(disturbance_flags),std::ref(valid_flags));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///

///@}
///@}
///
////////////////////////////////////////////////////////////////////////////////
} // namespace data_gnss
} // namespace mip

