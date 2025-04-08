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

namespace data_gnss {

////////////////////////////////////////////////////////////////////////////////
///@addtogroup MipData_cpp
///@{
///@defgroup gnss_data_cpp  Gnss Data
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
    DATA_BEIDOU_EPHEMERIS        = 0x64,
    DATA_GPS_IONO_CORR           = 0x71,
    DATA_GALILEO_IONO_CORR       = 0x73,
    DATA_BEIDOU_IONO_CORR        = 0x74,
    
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
    BEIDOU_B2A     = 169,  ///<  
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
///@defgroup gnss_pos_llh_cpp  (0x81,0x03) Pos Llh
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
        
        constexpr ValidFlags() : value(NONE) {}
        constexpr ValidFlags(int val) : value((uint16_t)val) {}
        constexpr operator uint16_t() const { return value; }
        constexpr ValidFlags& operator=(uint16_t val) { value = val; return *this; }
        constexpr ValidFlags& operator=(int val) { value = uint16_t(val); return *this; }
        constexpr ValidFlags& operator|=(uint16_t val) { return *this = value | val; }
        constexpr ValidFlags& operator&=(uint16_t val) { return *this = value & val; }
        
        constexpr bool latLon() const { return (value & LAT_LON) > 0; }
        constexpr void latLon(bool val) { value &= ~LAT_LON; if(val) value |= LAT_LON; }
        constexpr bool ellipsoidHeight() const { return (value & ELLIPSOID_HEIGHT) > 0; }
        constexpr void ellipsoidHeight(bool val) { value &= ~ELLIPSOID_HEIGHT; if(val) value |= ELLIPSOID_HEIGHT; }
        constexpr bool mslHeight() const { return (value & MSL_HEIGHT) > 0; }
        constexpr void mslHeight(bool val) { value &= ~MSL_HEIGHT; if(val) value |= MSL_HEIGHT; }
        constexpr bool horizontalAccuracy() const { return (value & HORIZONTAL_ACCURACY) > 0; }
        constexpr void horizontalAccuracy(bool val) { value &= ~HORIZONTAL_ACCURACY; if(val) value |= HORIZONTAL_ACCURACY; }
        constexpr bool verticalAccuracy() const { return (value & VERTICAL_ACCURACY) > 0; }
        constexpr void verticalAccuracy(bool val) { value &= ~VERTICAL_ACCURACY; if(val) value |= VERTICAL_ACCURACY; }
        constexpr uint16_t flags() const { return (value & FLAGS) >> 0; }
        constexpr void flags(uint16_t val) { value = (value & ~FLAGS) | (val << 0); }
        constexpr bool allSet() const { return value == ALL; }
        constexpr void setAll() { value |= ALL; }
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
///@defgroup gnss_pos_ecef_cpp  (0x81,0x04) Pos Ecef
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
        
        constexpr ValidFlags() : value(NONE) {}
        constexpr ValidFlags(int val) : value((uint16_t)val) {}
        constexpr operator uint16_t() const { return value; }
        constexpr ValidFlags& operator=(uint16_t val) { value = val; return *this; }
        constexpr ValidFlags& operator=(int val) { value = uint16_t(val); return *this; }
        constexpr ValidFlags& operator|=(uint16_t val) { return *this = value | val; }
        constexpr ValidFlags& operator&=(uint16_t val) { return *this = value & val; }
        
        constexpr bool position() const { return (value & POSITION) > 0; }
        constexpr void position(bool val) { value &= ~POSITION; if(val) value |= POSITION; }
        constexpr bool positionAccuracy() const { return (value & POSITION_ACCURACY) > 0; }
        constexpr void positionAccuracy(bool val) { value &= ~POSITION_ACCURACY; if(val) value |= POSITION_ACCURACY; }
        constexpr uint16_t flags() const { return (value & FLAGS) >> 0; }
        constexpr void flags(uint16_t val) { value = (value & ~FLAGS) | (val << 0); }
        constexpr bool allSet() const { return value == ALL; }
        constexpr void setAll() { value |= ALL; }
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
///@defgroup gnss_vel_ned_cpp  (0x81,0x05) Vel Ned
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
        
        constexpr ValidFlags() : value(NONE) {}
        constexpr ValidFlags(int val) : value((uint16_t)val) {}
        constexpr operator uint16_t() const { return value; }
        constexpr ValidFlags& operator=(uint16_t val) { value = val; return *this; }
        constexpr ValidFlags& operator=(int val) { value = uint16_t(val); return *this; }
        constexpr ValidFlags& operator|=(uint16_t val) { return *this = value | val; }
        constexpr ValidFlags& operator&=(uint16_t val) { return *this = value & val; }
        
        constexpr bool velocity() const { return (value & VELOCITY) > 0; }
        constexpr void velocity(bool val) { value &= ~VELOCITY; if(val) value |= VELOCITY; }
        constexpr bool speed3d() const { return (value & SPEED_3D) > 0; }
        constexpr void speed3d(bool val) { value &= ~SPEED_3D; if(val) value |= SPEED_3D; }
        constexpr bool groundSpeed() const { return (value & GROUND_SPEED) > 0; }
        constexpr void groundSpeed(bool val) { value &= ~GROUND_SPEED; if(val) value |= GROUND_SPEED; }
        constexpr bool heading() const { return (value & HEADING) > 0; }
        constexpr void heading(bool val) { value &= ~HEADING; if(val) value |= HEADING; }
        constexpr bool speedAccuracy() const { return (value & SPEED_ACCURACY) > 0; }
        constexpr void speedAccuracy(bool val) { value &= ~SPEED_ACCURACY; if(val) value |= SPEED_ACCURACY; }
        constexpr bool headingAccuracy() const { return (value & HEADING_ACCURACY) > 0; }
        constexpr void headingAccuracy(bool val) { value &= ~HEADING_ACCURACY; if(val) value |= HEADING_ACCURACY; }
        constexpr uint16_t flags() const { return (value & FLAGS) >> 0; }
        constexpr void flags(uint16_t val) { value = (value & ~FLAGS) | (val << 0); }
        constexpr bool allSet() const { return value == ALL; }
        constexpr void setAll() { value |= ALL; }
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
///@defgroup gnss_vel_ecef_cpp  (0x81,0x06) Vel Ecef
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
        
        constexpr ValidFlags() : value(NONE) {}
        constexpr ValidFlags(int val) : value((uint16_t)val) {}
        constexpr operator uint16_t() const { return value; }
        constexpr ValidFlags& operator=(uint16_t val) { value = val; return *this; }
        constexpr ValidFlags& operator=(int val) { value = uint16_t(val); return *this; }
        constexpr ValidFlags& operator|=(uint16_t val) { return *this = value | val; }
        constexpr ValidFlags& operator&=(uint16_t val) { return *this = value & val; }
        
        constexpr bool velocity() const { return (value & VELOCITY) > 0; }
        constexpr void velocity(bool val) { value &= ~VELOCITY; if(val) value |= VELOCITY; }
        constexpr bool velocityAccuracy() const { return (value & VELOCITY_ACCURACY) > 0; }
        constexpr void velocityAccuracy(bool val) { value &= ~VELOCITY_ACCURACY; if(val) value |= VELOCITY_ACCURACY; }
        constexpr uint16_t flags() const { return (value & FLAGS) >> 0; }
        constexpr void flags(uint16_t val) { value = (value & ~FLAGS) | (val << 0); }
        constexpr bool allSet() const { return value == ALL; }
        constexpr void setAll() { value |= ALL; }
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
///@defgroup gnss_dop_cpp  (0x81,0x07) Dop
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
        
        constexpr ValidFlags() : value(NONE) {}
        constexpr ValidFlags(int val) : value((uint16_t)val) {}
        constexpr operator uint16_t() const { return value; }
        constexpr ValidFlags& operator=(uint16_t val) { value = val; return *this; }
        constexpr ValidFlags& operator=(int val) { value = uint16_t(val); return *this; }
        constexpr ValidFlags& operator|=(uint16_t val) { return *this = value | val; }
        constexpr ValidFlags& operator&=(uint16_t val) { return *this = value & val; }
        
        constexpr bool gdop() const { return (value & GDOP) > 0; }
        constexpr void gdop(bool val) { value &= ~GDOP; if(val) value |= GDOP; }
        constexpr bool pdop() const { return (value & PDOP) > 0; }
        constexpr void pdop(bool val) { value &= ~PDOP; if(val) value |= PDOP; }
        constexpr bool hdop() const { return (value & HDOP) > 0; }
        constexpr void hdop(bool val) { value &= ~HDOP; if(val) value |= HDOP; }
        constexpr bool vdop() const { return (value & VDOP) > 0; }
        constexpr void vdop(bool val) { value &= ~VDOP; if(val) value |= VDOP; }
        constexpr bool tdop() const { return (value & TDOP) > 0; }
        constexpr void tdop(bool val) { value &= ~TDOP; if(val) value |= TDOP; }
        constexpr bool ndop() const { return (value & NDOP) > 0; }
        constexpr void ndop(bool val) { value &= ~NDOP; if(val) value |= NDOP; }
        constexpr bool edop() const { return (value & EDOP) > 0; }
        constexpr void edop(bool val) { value &= ~EDOP; if(val) value |= EDOP; }
        constexpr uint16_t flags() const { return (value & FLAGS) >> 0; }
        constexpr void flags(uint16_t val) { value = (value & ~FLAGS) | (val << 0); }
        constexpr bool allSet() const { return value == ALL; }
        constexpr void setAll() { value |= ALL; }
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
///@defgroup gnss_utc_time_cpp  (0x81,0x08) Utc Time
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
        
        constexpr ValidFlags() : value(NONE) {}
        constexpr ValidFlags(int val) : value((uint16_t)val) {}
        constexpr operator uint16_t() const { return value; }
        constexpr ValidFlags& operator=(uint16_t val) { value = val; return *this; }
        constexpr ValidFlags& operator=(int val) { value = uint16_t(val); return *this; }
        constexpr ValidFlags& operator|=(uint16_t val) { return *this = value | val; }
        constexpr ValidFlags& operator&=(uint16_t val) { return *this = value & val; }
        
        constexpr bool gnssDateTime() const { return (value & GNSS_DATE_TIME) > 0; }
        constexpr void gnssDateTime(bool val) { value &= ~GNSS_DATE_TIME; if(val) value |= GNSS_DATE_TIME; }
        constexpr bool leapSecondsKnown() const { return (value & LEAP_SECONDS_KNOWN) > 0; }
        constexpr void leapSecondsKnown(bool val) { value &= ~LEAP_SECONDS_KNOWN; if(val) value |= LEAP_SECONDS_KNOWN; }
        constexpr uint16_t flags() const { return (value & FLAGS) >> 0; }
        constexpr void flags(uint16_t val) { value = (value & ~FLAGS) | (val << 0); }
        constexpr bool allSet() const { return value == ALL; }
        constexpr void setAll() { value |= ALL; }
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
///@defgroup gnss_gps_time_cpp  (0x81,0x09) Gps Time
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
        
        constexpr ValidFlags() : value(NONE) {}
        constexpr ValidFlags(int val) : value((uint16_t)val) {}
        constexpr operator uint16_t() const { return value; }
        constexpr ValidFlags& operator=(uint16_t val) { value = val; return *this; }
        constexpr ValidFlags& operator=(int val) { value = uint16_t(val); return *this; }
        constexpr ValidFlags& operator|=(uint16_t val) { return *this = value | val; }
        constexpr ValidFlags& operator&=(uint16_t val) { return *this = value & val; }
        
        constexpr bool tow() const { return (value & TOW) > 0; }
        constexpr void tow(bool val) { value &= ~TOW; if(val) value |= TOW; }
        constexpr bool weekNumber() const { return (value & WEEK_NUMBER) > 0; }
        constexpr void weekNumber(bool val) { value &= ~WEEK_NUMBER; if(val) value |= WEEK_NUMBER; }
        constexpr uint16_t flags() const { return (value & FLAGS) >> 0; }
        constexpr void flags(uint16_t val) { value = (value & ~FLAGS) | (val << 0); }
        constexpr bool allSet() const { return value == ALL; }
        constexpr void setAll() { value |= ALL; }
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
///@defgroup gnss_clock_info_cpp  (0x81,0x0A) Clock Info
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
        
        constexpr ValidFlags() : value(NONE) {}
        constexpr ValidFlags(int val) : value((uint16_t)val) {}
        constexpr operator uint16_t() const { return value; }
        constexpr ValidFlags& operator=(uint16_t val) { value = val; return *this; }
        constexpr ValidFlags& operator=(int val) { value = uint16_t(val); return *this; }
        constexpr ValidFlags& operator|=(uint16_t val) { return *this = value | val; }
        constexpr ValidFlags& operator&=(uint16_t val) { return *this = value & val; }
        
        constexpr bool bias() const { return (value & BIAS) > 0; }
        constexpr void bias(bool val) { value &= ~BIAS; if(val) value |= BIAS; }
        constexpr bool drift() const { return (value & DRIFT) > 0; }
        constexpr void drift(bool val) { value &= ~DRIFT; if(val) value |= DRIFT; }
        constexpr bool accuracyEstimate() const { return (value & ACCURACY_ESTIMATE) > 0; }
        constexpr void accuracyEstimate(bool val) { value &= ~ACCURACY_ESTIMATE; if(val) value |= ACCURACY_ESTIMATE; }
        constexpr uint16_t flags() const { return (value & FLAGS) >> 0; }
        constexpr void flags(uint16_t val) { value = (value & ~FLAGS) | (val << 0); }
        constexpr bool allSet() const { return value == ALL; }
        constexpr void setAll() { value |= ALL; }
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
///@defgroup gnss_fix_info_cpp  (0x81,0x0B) Fix Info
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
        
        constexpr FixFlags() : value(NONE) {}
        constexpr FixFlags(int val) : value((uint16_t)val) {}
        constexpr operator uint16_t() const { return value; }
        constexpr FixFlags& operator=(uint16_t val) { value = val; return *this; }
        constexpr FixFlags& operator=(int val) { value = uint16_t(val); return *this; }
        constexpr FixFlags& operator|=(uint16_t val) { return *this = value | val; }
        constexpr FixFlags& operator&=(uint16_t val) { return *this = value & val; }
        
        constexpr bool sbasUsed() const { return (value & SBAS_USED) > 0; }
        constexpr void sbasUsed(bool val) { value &= ~SBAS_USED; if(val) value |= SBAS_USED; }
        constexpr bool dgnssUsed() const { return (value & DGNSS_USED) > 0; }
        constexpr void dgnssUsed(bool val) { value &= ~DGNSS_USED; if(val) value |= DGNSS_USED; }
        constexpr bool allSet() const { return value == ALL; }
        constexpr void setAll() { value |= ALL; }
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
        
        constexpr ValidFlags() : value(NONE) {}
        constexpr ValidFlags(int val) : value((uint16_t)val) {}
        constexpr operator uint16_t() const { return value; }
        constexpr ValidFlags& operator=(uint16_t val) { value = val; return *this; }
        constexpr ValidFlags& operator=(int val) { value = uint16_t(val); return *this; }
        constexpr ValidFlags& operator|=(uint16_t val) { return *this = value | val; }
        constexpr ValidFlags& operator&=(uint16_t val) { return *this = value & val; }
        
        constexpr bool fixType() const { return (value & FIX_TYPE) > 0; }
        constexpr void fixType(bool val) { value &= ~FIX_TYPE; if(val) value |= FIX_TYPE; }
        constexpr bool numSv() const { return (value & NUM_SV) > 0; }
        constexpr void numSv(bool val) { value &= ~NUM_SV; if(val) value |= NUM_SV; }
        constexpr bool fixFlags() const { return (value & FIX_FLAGS) > 0; }
        constexpr void fixFlags(bool val) { value &= ~FIX_FLAGS; if(val) value |= FIX_FLAGS; }
        constexpr uint16_t flags() const { return (value & FLAGS) >> 0; }
        constexpr void flags(uint16_t val) { value = (value & ~FLAGS) | (val << 0); }
        constexpr bool allSet() const { return value == ALL; }
        constexpr void setAll() { value |= ALL; }
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
///@defgroup gnss_sv_info_cpp  (0x81,0x0C) Sv Info
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
        
        constexpr SVFlags() : value(NONE) {}
        constexpr SVFlags(int val) : value((uint16_t)val) {}
        constexpr operator uint16_t() const { return value; }
        constexpr SVFlags& operator=(uint16_t val) { value = val; return *this; }
        constexpr SVFlags& operator=(int val) { value = uint16_t(val); return *this; }
        constexpr SVFlags& operator|=(uint16_t val) { return *this = value | val; }
        constexpr SVFlags& operator&=(uint16_t val) { return *this = value & val; }
        
        constexpr bool usedForNavigation() const { return (value & USED_FOR_NAVIGATION) > 0; }
        constexpr void usedForNavigation(bool val) { value &= ~USED_FOR_NAVIGATION; if(val) value |= USED_FOR_NAVIGATION; }
        constexpr bool healthy() const { return (value & HEALTHY) > 0; }
        constexpr void healthy(bool val) { value &= ~HEALTHY; if(val) value |= HEALTHY; }
        constexpr bool allSet() const { return value == ALL; }
        constexpr void setAll() { value |= ALL; }
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
        
        constexpr ValidFlags() : value(NONE) {}
        constexpr ValidFlags(int val) : value((uint16_t)val) {}
        constexpr operator uint16_t() const { return value; }
        constexpr ValidFlags& operator=(uint16_t val) { value = val; return *this; }
        constexpr ValidFlags& operator=(int val) { value = uint16_t(val); return *this; }
        constexpr ValidFlags& operator|=(uint16_t val) { return *this = value | val; }
        constexpr ValidFlags& operator&=(uint16_t val) { return *this = value & val; }
        
        constexpr bool channel() const { return (value & CHANNEL) > 0; }
        constexpr void channel(bool val) { value &= ~CHANNEL; if(val) value |= CHANNEL; }
        constexpr bool svId() const { return (value & SV_ID) > 0; }
        constexpr void svId(bool val) { value &= ~SV_ID; if(val) value |= SV_ID; }
        constexpr bool carrierNoiseRatio() const { return (value & CARRIER_NOISE_RATIO) > 0; }
        constexpr void carrierNoiseRatio(bool val) { value &= ~CARRIER_NOISE_RATIO; if(val) value |= CARRIER_NOISE_RATIO; }
        constexpr bool azimuth() const { return (value & AZIMUTH) > 0; }
        constexpr void azimuth(bool val) { value &= ~AZIMUTH; if(val) value |= AZIMUTH; }
        constexpr bool elevation() const { return (value & ELEVATION) > 0; }
        constexpr void elevation(bool val) { value &= ~ELEVATION; if(val) value |= ELEVATION; }
        constexpr bool svFlags() const { return (value & SV_FLAGS) > 0; }
        constexpr void svFlags(bool val) { value &= ~SV_FLAGS; if(val) value |= SV_FLAGS; }
        constexpr uint16_t flags() const { return (value & FLAGS) >> 0; }
        constexpr void flags(uint16_t val) { value = (value & ~FLAGS) | (val << 0); }
        constexpr bool allSet() const { return value == ALL; }
        constexpr void setAll() { value |= ALL; }
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
///@defgroup gnss_hw_status_cpp  (0x81,0x0D) Hw Status
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
        
        constexpr ValidFlags() : value(NONE) {}
        constexpr ValidFlags(int val) : value((uint16_t)val) {}
        constexpr operator uint16_t() const { return value; }
        constexpr ValidFlags& operator=(uint16_t val) { value = val; return *this; }
        constexpr ValidFlags& operator=(int val) { value = uint16_t(val); return *this; }
        constexpr ValidFlags& operator|=(uint16_t val) { return *this = value | val; }
        constexpr ValidFlags& operator&=(uint16_t val) { return *this = value & val; }
        
        constexpr bool sensorState() const { return (value & SENSOR_STATE) > 0; }
        constexpr void sensorState(bool val) { value &= ~SENSOR_STATE; if(val) value |= SENSOR_STATE; }
        constexpr bool antennaState() const { return (value & ANTENNA_STATE) > 0; }
        constexpr void antennaState(bool val) { value &= ~ANTENNA_STATE; if(val) value |= ANTENNA_STATE; }
        constexpr bool antennaPower() const { return (value & ANTENNA_POWER) > 0; }
        constexpr void antennaPower(bool val) { value &= ~ANTENNA_POWER; if(val) value |= ANTENNA_POWER; }
        constexpr uint16_t flags() const { return (value & FLAGS) >> 0; }
        constexpr void flags(uint16_t val) { value = (value & ~FLAGS) | (val << 0); }
        constexpr bool allSet() const { return value == ALL; }
        constexpr void setAll() { value |= ALL; }
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
///@defgroup gnss_dgps_info_cpp  (0x81,0x0E) Dgps Info
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
        
        constexpr ValidFlags() : value(NONE) {}
        constexpr ValidFlags(int val) : value((uint16_t)val) {}
        constexpr operator uint16_t() const { return value; }
        constexpr ValidFlags& operator=(uint16_t val) { value = val; return *this; }
        constexpr ValidFlags& operator=(int val) { value = uint16_t(val); return *this; }
        constexpr ValidFlags& operator|=(uint16_t val) { return *this = value | val; }
        constexpr ValidFlags& operator&=(uint16_t val) { return *this = value & val; }
        
        constexpr bool age() const { return (value & AGE) > 0; }
        constexpr void age(bool val) { value &= ~AGE; if(val) value |= AGE; }
        constexpr bool baseStationId() const { return (value & BASE_STATION_ID) > 0; }
        constexpr void baseStationId(bool val) { value &= ~BASE_STATION_ID; if(val) value |= BASE_STATION_ID; }
        constexpr bool baseStationStatus() const { return (value & BASE_STATION_STATUS) > 0; }
        constexpr void baseStationStatus(bool val) { value &= ~BASE_STATION_STATUS; if(val) value |= BASE_STATION_STATUS; }
        constexpr bool numChannels() const { return (value & NUM_CHANNELS) > 0; }
        constexpr void numChannels(bool val) { value &= ~NUM_CHANNELS; if(val) value |= NUM_CHANNELS; }
        constexpr uint16_t flags() const { return (value & FLAGS) >> 0; }
        constexpr void flags(uint16_t val) { value = (value & ~FLAGS) | (val << 0); }
        constexpr bool allSet() const { return value == ALL; }
        constexpr void setAll() { value |= ALL; }
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
///@defgroup gnss_dgps_channel_cpp  (0x81,0x0F) Dgps Channel
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
        
        constexpr ValidFlags() : value(NONE) {}
        constexpr ValidFlags(int val) : value((uint16_t)val) {}
        constexpr operator uint16_t() const { return value; }
        constexpr ValidFlags& operator=(uint16_t val) { value = val; return *this; }
        constexpr ValidFlags& operator=(int val) { value = uint16_t(val); return *this; }
        constexpr ValidFlags& operator|=(uint16_t val) { return *this = value | val; }
        constexpr ValidFlags& operator&=(uint16_t val) { return *this = value & val; }
        
        constexpr bool id() const { return (value & ID) > 0; }
        constexpr void id(bool val) { value &= ~ID; if(val) value |= ID; }
        constexpr bool age() const { return (value & AGE) > 0; }
        constexpr void age(bool val) { value &= ~AGE; if(val) value |= AGE; }
        constexpr bool rangeCorrection() const { return (value & RANGE_CORRECTION) > 0; }
        constexpr void rangeCorrection(bool val) { value &= ~RANGE_CORRECTION; if(val) value |= RANGE_CORRECTION; }
        constexpr bool rangeRateCorrection() const { return (value & RANGE_RATE_CORRECTION) > 0; }
        constexpr void rangeRateCorrection(bool val) { value &= ~RANGE_RATE_CORRECTION; if(val) value |= RANGE_RATE_CORRECTION; }
        constexpr uint16_t flags() const { return (value & FLAGS) >> 0; }
        constexpr void flags(uint16_t val) { value = (value & ~FLAGS) | (val << 0); }
        constexpr bool allSet() const { return value == ALL; }
        constexpr void setAll() { value |= ALL; }
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
///@defgroup gnss_clock_info_2_cpp  (0x81,0x10) Clock Info 2
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
        
        constexpr ValidFlags() : value(NONE) {}
        constexpr ValidFlags(int val) : value((uint16_t)val) {}
        constexpr operator uint16_t() const { return value; }
        constexpr ValidFlags& operator=(uint16_t val) { value = val; return *this; }
        constexpr ValidFlags& operator=(int val) { value = uint16_t(val); return *this; }
        constexpr ValidFlags& operator|=(uint16_t val) { return *this = value | val; }
        constexpr ValidFlags& operator&=(uint16_t val) { return *this = value & val; }
        
        constexpr bool bias() const { return (value & BIAS) > 0; }
        constexpr void bias(bool val) { value &= ~BIAS; if(val) value |= BIAS; }
        constexpr bool drift() const { return (value & DRIFT) > 0; }
        constexpr void drift(bool val) { value &= ~DRIFT; if(val) value |= DRIFT; }
        constexpr bool biasAccuracy() const { return (value & BIAS_ACCURACY) > 0; }
        constexpr void biasAccuracy(bool val) { value &= ~BIAS_ACCURACY; if(val) value |= BIAS_ACCURACY; }
        constexpr bool driftAccuracy() const { return (value & DRIFT_ACCURACY) > 0; }
        constexpr void driftAccuracy(bool val) { value &= ~DRIFT_ACCURACY; if(val) value |= DRIFT_ACCURACY; }
        constexpr uint16_t flags() const { return (value & FLAGS) >> 0; }
        constexpr void flags(uint16_t val) { value = (value & ~FLAGS) | (val << 0); }
        constexpr bool allSet() const { return value == ALL; }
        constexpr void setAll() { value |= ALL; }
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
///@defgroup gnss_gps_leap_seconds_cpp  (0x81,0x11) Gps Leap Seconds
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
        
        constexpr ValidFlags() : value(NONE) {}
        constexpr ValidFlags(int val) : value((uint16_t)val) {}
        constexpr operator uint16_t() const { return value; }
        constexpr ValidFlags& operator=(uint16_t val) { value = val; return *this; }
        constexpr ValidFlags& operator=(int val) { value = uint16_t(val); return *this; }
        constexpr ValidFlags& operator|=(uint16_t val) { return *this = value | val; }
        constexpr ValidFlags& operator&=(uint16_t val) { return *this = value & val; }
        
        constexpr bool leapSeconds() const { return (value & LEAP_SECONDS) > 0; }
        constexpr void leapSeconds(bool val) { value &= ~LEAP_SECONDS; if(val) value |= LEAP_SECONDS; }
        constexpr bool allSet() const { return value == ALL; }
        constexpr void setAll() { value |= ALL; }
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
///@defgroup gnss_sbas_info_cpp  (0x81,0x12) Sbas Info
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
        
        constexpr SbasStatus() : value(NONE) {}
        constexpr SbasStatus(int val) : value((uint8_t)val) {}
        constexpr operator uint8_t() const { return value; }
        constexpr SbasStatus& operator=(uint8_t val) { value = val; return *this; }
        constexpr SbasStatus& operator=(int val) { value = uint8_t(val); return *this; }
        constexpr SbasStatus& operator|=(uint8_t val) { return *this = value | val; }
        constexpr SbasStatus& operator&=(uint8_t val) { return *this = value & val; }
        
        constexpr bool rangeAvailable() const { return (value & RANGE_AVAILABLE) > 0; }
        constexpr void rangeAvailable(bool val) { value &= ~RANGE_AVAILABLE; if(val) value |= RANGE_AVAILABLE; }
        constexpr bool correctionsAvailable() const { return (value & CORRECTIONS_AVAILABLE) > 0; }
        constexpr void correctionsAvailable(bool val) { value &= ~CORRECTIONS_AVAILABLE; if(val) value |= CORRECTIONS_AVAILABLE; }
        constexpr bool integrityAvailable() const { return (value & INTEGRITY_AVAILABLE) > 0; }
        constexpr void integrityAvailable(bool val) { value &= ~INTEGRITY_AVAILABLE; if(val) value |= INTEGRITY_AVAILABLE; }
        constexpr bool testMode() const { return (value & TEST_MODE) > 0; }
        constexpr void testMode(bool val) { value &= ~TEST_MODE; if(val) value |= TEST_MODE; }
        constexpr bool allSet() const { return value == ALL; }
        constexpr void setAll() { value |= ALL; }
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
        
        constexpr ValidFlags() : value(NONE) {}
        constexpr ValidFlags(int val) : value((uint16_t)val) {}
        constexpr operator uint16_t() const { return value; }
        constexpr ValidFlags& operator=(uint16_t val) { value = val; return *this; }
        constexpr ValidFlags& operator=(int val) { value = uint16_t(val); return *this; }
        constexpr ValidFlags& operator|=(uint16_t val) { return *this = value | val; }
        constexpr ValidFlags& operator&=(uint16_t val) { return *this = value & val; }
        
        constexpr bool tow() const { return (value & TOW) > 0; }
        constexpr void tow(bool val) { value &= ~TOW; if(val) value |= TOW; }
        constexpr bool weekNumber() const { return (value & WEEK_NUMBER) > 0; }
        constexpr void weekNumber(bool val) { value &= ~WEEK_NUMBER; if(val) value |= WEEK_NUMBER; }
        constexpr bool sbasSystem() const { return (value & SBAS_SYSTEM) > 0; }
        constexpr void sbasSystem(bool val) { value &= ~SBAS_SYSTEM; if(val) value |= SBAS_SYSTEM; }
        constexpr bool sbasId() const { return (value & SBAS_ID) > 0; }
        constexpr void sbasId(bool val) { value &= ~SBAS_ID; if(val) value |= SBAS_ID; }
        constexpr bool count() const { return (value & COUNT) > 0; }
        constexpr void count(bool val) { value &= ~COUNT; if(val) value |= COUNT; }
        constexpr bool sbasStatus() const { return (value & SBAS_STATUS) > 0; }
        constexpr void sbasStatus(bool val) { value &= ~SBAS_STATUS; if(val) value |= SBAS_STATUS; }
        constexpr uint16_t flags() const { return (value & FLAGS) >> 0; }
        constexpr void flags(uint16_t val) { value = (value & ~FLAGS) | (val << 0); }
        constexpr bool allSet() const { return value == ALL; }
        constexpr void setAll() { value |= ALL; }
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
///@defgroup gnss_sbas_correction_cpp  (0x81,0x13) Sbas Correction
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
        
        constexpr ValidFlags() : value(NONE) {}
        constexpr ValidFlags(int val) : value((uint16_t)val) {}
        constexpr operator uint16_t() const { return value; }
        constexpr ValidFlags& operator=(uint16_t val) { value = val; return *this; }
        constexpr ValidFlags& operator=(int val) { value = uint16_t(val); return *this; }
        constexpr ValidFlags& operator|=(uint16_t val) { return *this = value | val; }
        constexpr ValidFlags& operator&=(uint16_t val) { return *this = value & val; }
        
        constexpr bool udrei() const { return (value & UDREI) > 0; }
        constexpr void udrei(bool val) { value &= ~UDREI; if(val) value |= UDREI; }
        constexpr bool pseudorangeCorrection() const { return (value & PSEUDORANGE_CORRECTION) > 0; }
        constexpr void pseudorangeCorrection(bool val) { value &= ~PSEUDORANGE_CORRECTION; if(val) value |= PSEUDORANGE_CORRECTION; }
        constexpr bool ionoCorrection() const { return (value & IONO_CORRECTION) > 0; }
        constexpr void ionoCorrection(bool val) { value &= ~IONO_CORRECTION; if(val) value |= IONO_CORRECTION; }
        constexpr uint16_t flags() const { return (value & FLAGS) >> 0; }
        constexpr void flags(uint16_t val) { value = (value & ~FLAGS) | (val << 0); }
        constexpr bool allSet() const { return value == ALL; }
        constexpr void setAll() { value |= ALL; }
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
///@defgroup gnss_rf_error_detection_cpp  (0x81,0x14) Rf Error Detection
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
        
        constexpr ValidFlags() : value(NONE) {}
        constexpr ValidFlags(int val) : value((uint16_t)val) {}
        constexpr operator uint16_t() const { return value; }
        constexpr ValidFlags& operator=(uint16_t val) { value = val; return *this; }
        constexpr ValidFlags& operator=(int val) { value = uint16_t(val); return *this; }
        constexpr ValidFlags& operator|=(uint16_t val) { return *this = value | val; }
        constexpr ValidFlags& operator&=(uint16_t val) { return *this = value & val; }
        
        constexpr bool rfBand() const { return (value & RF_BAND) > 0; }
        constexpr void rfBand(bool val) { value &= ~RF_BAND; if(val) value |= RF_BAND; }
        constexpr bool jammingState() const { return (value & JAMMING_STATE) > 0; }
        constexpr void jammingState(bool val) { value &= ~JAMMING_STATE; if(val) value |= JAMMING_STATE; }
        constexpr bool spoofingState() const { return (value & SPOOFING_STATE) > 0; }
        constexpr void spoofingState(bool val) { value &= ~SPOOFING_STATE; if(val) value |= SPOOFING_STATE; }
        constexpr uint16_t flags() const { return (value & FLAGS) >> 0; }
        constexpr void flags(uint16_t val) { value = (value & ~FLAGS) | (val << 0); }
        constexpr bool allSet() const { return value == ALL; }
        constexpr void setAll() { value |= ALL; }
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
///@defgroup gnss_base_station_info_cpp  (0x81,0x30) Base Station Info
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
        
        constexpr IndicatorFlags() : value(NONE) {}
        constexpr IndicatorFlags(int val) : value((uint16_t)val) {}
        constexpr operator uint16_t() const { return value; }
        constexpr IndicatorFlags& operator=(uint16_t val) { value = val; return *this; }
        constexpr IndicatorFlags& operator=(int val) { value = uint16_t(val); return *this; }
        constexpr IndicatorFlags& operator|=(uint16_t val) { return *this = value | val; }
        constexpr IndicatorFlags& operator&=(uint16_t val) { return *this = value & val; }
        
        constexpr bool gps() const { return (value & GPS) > 0; }
        constexpr void gps(bool val) { value &= ~GPS; if(val) value |= GPS; }
        constexpr bool glonass() const { return (value & GLONASS) > 0; }
        constexpr void glonass(bool val) { value &= ~GLONASS; if(val) value |= GLONASS; }
        constexpr bool galileo() const { return (value & GALILEO) > 0; }
        constexpr void galileo(bool val) { value &= ~GALILEO; if(val) value |= GALILEO; }
        constexpr bool beidou() const { return (value & BEIDOU) > 0; }
        constexpr void beidou(bool val) { value &= ~BEIDOU; if(val) value |= BEIDOU; }
        constexpr bool refStation() const { return (value & REF_STATION) > 0; }
        constexpr void refStation(bool val) { value &= ~REF_STATION; if(val) value |= REF_STATION; }
        constexpr bool singleReceiver() const { return (value & SINGLE_RECEIVER) > 0; }
        constexpr void singleReceiver(bool val) { value &= ~SINGLE_RECEIVER; if(val) value |= SINGLE_RECEIVER; }
        constexpr bool quarterCycleBit1() const { return (value & QUARTER_CYCLE_BIT1) > 0; }
        constexpr void quarterCycleBit1(bool val) { value &= ~QUARTER_CYCLE_BIT1; if(val) value |= QUARTER_CYCLE_BIT1; }
        constexpr bool quarterCycleBit2() const { return (value & QUARTER_CYCLE_BIT2) > 0; }
        constexpr void quarterCycleBit2(bool val) { value &= ~QUARTER_CYCLE_BIT2; if(val) value |= QUARTER_CYCLE_BIT2; }
        constexpr uint16_t quarterCycleBits() const { return (value & QUARTER_CYCLE_BITS) >> 6; }
        constexpr void quarterCycleBits(uint16_t val) { value = (value & ~QUARTER_CYCLE_BITS) | (val << 6); }
        constexpr bool allSet() const { return value == ALL; }
        constexpr void setAll() { value |= ALL; }
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
        
        constexpr ValidFlags() : value(NONE) {}
        constexpr ValidFlags(int val) : value((uint16_t)val) {}
        constexpr operator uint16_t() const { return value; }
        constexpr ValidFlags& operator=(uint16_t val) { value = val; return *this; }
        constexpr ValidFlags& operator=(int val) { value = uint16_t(val); return *this; }
        constexpr ValidFlags& operator|=(uint16_t val) { return *this = value | val; }
        constexpr ValidFlags& operator&=(uint16_t val) { return *this = value & val; }
        
        constexpr bool tow() const { return (value & TOW) > 0; }
        constexpr void tow(bool val) { value &= ~TOW; if(val) value |= TOW; }
        constexpr bool weekNumber() const { return (value & WEEK_NUMBER) > 0; }
        constexpr void weekNumber(bool val) { value &= ~WEEK_NUMBER; if(val) value |= WEEK_NUMBER; }
        constexpr bool ecefPosition() const { return (value & ECEF_POSITION) > 0; }
        constexpr void ecefPosition(bool val) { value &= ~ECEF_POSITION; if(val) value |= ECEF_POSITION; }
        constexpr bool height() const { return (value & HEIGHT) > 0; }
        constexpr void height(bool val) { value &= ~HEIGHT; if(val) value |= HEIGHT; }
        constexpr bool stationId() const { return (value & STATION_ID) > 0; }
        constexpr void stationId(bool val) { value &= ~STATION_ID; if(val) value |= STATION_ID; }
        constexpr bool indicators() const { return (value & INDICATORS) > 0; }
        constexpr void indicators(bool val) { value &= ~INDICATORS; if(val) value |= INDICATORS; }
        constexpr uint16_t flags() const { return (value & FLAGS) >> 0; }
        constexpr void flags(uint16_t val) { value = (value & ~FLAGS) | (val << 0); }
        constexpr bool allSet() const { return value == ALL; }
        constexpr void setAll() { value |= ALL; }
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
///@defgroup gnss_rtk_corrections_status_cpp  (0x81,0x31) Rtk Corrections Status
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
        
        constexpr ValidFlags() : value(NONE) {}
        constexpr ValidFlags(int val) : value((uint16_t)val) {}
        constexpr operator uint16_t() const { return value; }
        constexpr ValidFlags& operator=(uint16_t val) { value = val; return *this; }
        constexpr ValidFlags& operator=(int val) { value = uint16_t(val); return *this; }
        constexpr ValidFlags& operator|=(uint16_t val) { return *this = value | val; }
        constexpr ValidFlags& operator&=(uint16_t val) { return *this = value & val; }
        
        constexpr bool tow() const { return (value & TOW) > 0; }
        constexpr void tow(bool val) { value &= ~TOW; if(val) value |= TOW; }
        constexpr bool weekNumber() const { return (value & WEEK_NUMBER) > 0; }
        constexpr void weekNumber(bool val) { value &= ~WEEK_NUMBER; if(val) value |= WEEK_NUMBER; }
        constexpr bool epochStatus() const { return (value & EPOCH_STATUS) > 0; }
        constexpr void epochStatus(bool val) { value &= ~EPOCH_STATUS; if(val) value |= EPOCH_STATUS; }
        constexpr bool dongleStatus() const { return (value & DONGLE_STATUS) > 0; }
        constexpr void dongleStatus(bool val) { value &= ~DONGLE_STATUS; if(val) value |= DONGLE_STATUS; }
        constexpr bool gpsLatency() const { return (value & GPS_LATENCY) > 0; }
        constexpr void gpsLatency(bool val) { value &= ~GPS_LATENCY; if(val) value |= GPS_LATENCY; }
        constexpr bool glonassLatency() const { return (value & GLONASS_LATENCY) > 0; }
        constexpr void glonassLatency(bool val) { value &= ~GLONASS_LATENCY; if(val) value |= GLONASS_LATENCY; }
        constexpr bool galileoLatency() const { return (value & GALILEO_LATENCY) > 0; }
        constexpr void galileoLatency(bool val) { value &= ~GALILEO_LATENCY; if(val) value |= GALILEO_LATENCY; }
        constexpr bool beidouLatency() const { return (value & BEIDOU_LATENCY) > 0; }
        constexpr void beidouLatency(bool val) { value &= ~BEIDOU_LATENCY; if(val) value |= BEIDOU_LATENCY; }
        constexpr uint16_t flags() const { return (value & FLAGS) >> 0; }
        constexpr void flags(uint16_t val) { value = (value & ~FLAGS) | (val << 0); }
        constexpr bool allSet() const { return value == ALL; }
        constexpr void setAll() { value |= ALL; }
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
        
        constexpr EpochStatus() : value(NONE) {}
        constexpr EpochStatus(int val) : value((uint16_t)val) {}
        constexpr operator uint16_t() const { return value; }
        constexpr EpochStatus& operator=(uint16_t val) { value = val; return *this; }
        constexpr EpochStatus& operator=(int val) { value = uint16_t(val); return *this; }
        constexpr EpochStatus& operator|=(uint16_t val) { return *this = value | val; }
        constexpr EpochStatus& operator&=(uint16_t val) { return *this = value & val; }
        
        constexpr bool antennaLocationReceived() const { return (value & ANTENNA_LOCATION_RECEIVED) > 0; }
        constexpr void antennaLocationReceived(bool val) { value &= ~ANTENNA_LOCATION_RECEIVED; if(val) value |= ANTENNA_LOCATION_RECEIVED; }
        constexpr bool antennaDescriptionReceived() const { return (value & ANTENNA_DESCRIPTION_RECEIVED) > 0; }
        constexpr void antennaDescriptionReceived(bool val) { value &= ~ANTENNA_DESCRIPTION_RECEIVED; if(val) value |= ANTENNA_DESCRIPTION_RECEIVED; }
        constexpr bool gpsReceived() const { return (value & GPS_RECEIVED) > 0; }
        constexpr void gpsReceived(bool val) { value &= ~GPS_RECEIVED; if(val) value |= GPS_RECEIVED; }
        constexpr bool glonassReceived() const { return (value & GLONASS_RECEIVED) > 0; }
        constexpr void glonassReceived(bool val) { value &= ~GLONASS_RECEIVED; if(val) value |= GLONASS_RECEIVED; }
        constexpr bool galileoReceived() const { return (value & GALILEO_RECEIVED) > 0; }
        constexpr void galileoReceived(bool val) { value &= ~GALILEO_RECEIVED; if(val) value |= GALILEO_RECEIVED; }
        constexpr bool beidouReceived() const { return (value & BEIDOU_RECEIVED) > 0; }
        constexpr void beidouReceived(bool val) { value &= ~BEIDOU_RECEIVED; if(val) value |= BEIDOU_RECEIVED; }
        constexpr bool usingGpsMsmMessages() const { return (value & USING_GPS_MSM_MESSAGES) > 0; }
        constexpr void usingGpsMsmMessages(bool val) { value &= ~USING_GPS_MSM_MESSAGES; if(val) value |= USING_GPS_MSM_MESSAGES; }
        constexpr bool usingGlonassMsmMessages() const { return (value & USING_GLONASS_MSM_MESSAGES) > 0; }
        constexpr void usingGlonassMsmMessages(bool val) { value &= ~USING_GLONASS_MSM_MESSAGES; if(val) value |= USING_GLONASS_MSM_MESSAGES; }
        constexpr bool dongleStatusReadFailed() const { return (value & DONGLE_STATUS_READ_FAILED) > 0; }
        constexpr void dongleStatusReadFailed(bool val) { value &= ~DONGLE_STATUS_READ_FAILED; if(val) value |= DONGLE_STATUS_READ_FAILED; }
        constexpr bool allSet() const { return value == ALL; }
        constexpr void setAll() { value |= ALL; }
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
///@defgroup gnss_satellite_status_cpp  (0x81,0x20) Satellite Status
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
        
        constexpr ValidFlags() : value(NONE) {}
        constexpr ValidFlags(int val) : value((uint16_t)val) {}
        constexpr operator uint16_t() const { return value; }
        constexpr ValidFlags& operator=(uint16_t val) { value = val; return *this; }
        constexpr ValidFlags& operator=(int val) { value = uint16_t(val); return *this; }
        constexpr ValidFlags& operator|=(uint16_t val) { return *this = value | val; }
        constexpr ValidFlags& operator&=(uint16_t val) { return *this = value & val; }
        
        constexpr bool tow() const { return (value & TOW) > 0; }
        constexpr void tow(bool val) { value &= ~TOW; if(val) value |= TOW; }
        constexpr bool weekNumber() const { return (value & WEEK_NUMBER) > 0; }
        constexpr void weekNumber(bool val) { value &= ~WEEK_NUMBER; if(val) value |= WEEK_NUMBER; }
        constexpr bool gnssId() const { return (value & GNSS_ID) > 0; }
        constexpr void gnssId(bool val) { value &= ~GNSS_ID; if(val) value |= GNSS_ID; }
        constexpr bool satelliteId() const { return (value & SATELLITE_ID) > 0; }
        constexpr void satelliteId(bool val) { value &= ~SATELLITE_ID; if(val) value |= SATELLITE_ID; }
        constexpr bool elevation() const { return (value & ELEVATION) > 0; }
        constexpr void elevation(bool val) { value &= ~ELEVATION; if(val) value |= ELEVATION; }
        constexpr bool azimuth() const { return (value & AZIMUTH) > 0; }
        constexpr void azimuth(bool val) { value &= ~AZIMUTH; if(val) value |= AZIMUTH; }
        constexpr bool health() const { return (value & HEALTH) > 0; }
        constexpr void health(bool val) { value &= ~HEALTH; if(val) value |= HEALTH; }
        constexpr uint16_t flags() const { return (value & FLAGS) >> 0; }
        constexpr void flags(uint16_t val) { value = (value & ~FLAGS) | (val << 0); }
        constexpr bool allSet() const { return value == ALL; }
        constexpr void setAll() { value |= ALL; }
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
///@defgroup gnss_raw_cpp  (0x81,0x22) Raw
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
        
        constexpr ValidFlags() : value(NONE) {}
        constexpr ValidFlags(int val) : value((uint16_t)val) {}
        constexpr operator uint16_t() const { return value; }
        constexpr ValidFlags& operator=(uint16_t val) { value = val; return *this; }
        constexpr ValidFlags& operator=(int val) { value = uint16_t(val); return *this; }
        constexpr ValidFlags& operator|=(uint16_t val) { return *this = value | val; }
        constexpr ValidFlags& operator&=(uint16_t val) { return *this = value & val; }
        
        constexpr bool tow() const { return (value & TOW) > 0; }
        constexpr void tow(bool val) { value &= ~TOW; if(val) value |= TOW; }
        constexpr bool weekNumber() const { return (value & WEEK_NUMBER) > 0; }
        constexpr void weekNumber(bool val) { value &= ~WEEK_NUMBER; if(val) value |= WEEK_NUMBER; }
        constexpr bool receiverId() const { return (value & RECEIVER_ID) > 0; }
        constexpr void receiverId(bool val) { value &= ~RECEIVER_ID; if(val) value |= RECEIVER_ID; }
        constexpr bool trackingChannel() const { return (value & TRACKING_CHANNEL) > 0; }
        constexpr void trackingChannel(bool val) { value &= ~TRACKING_CHANNEL; if(val) value |= TRACKING_CHANNEL; }
        constexpr bool gnssId() const { return (value & GNSS_ID) > 0; }
        constexpr void gnssId(bool val) { value &= ~GNSS_ID; if(val) value |= GNSS_ID; }
        constexpr bool satelliteId() const { return (value & SATELLITE_ID) > 0; }
        constexpr void satelliteId(bool val) { value &= ~SATELLITE_ID; if(val) value |= SATELLITE_ID; }
        constexpr bool signalId() const { return (value & SIGNAL_ID) > 0; }
        constexpr void signalId(bool val) { value &= ~SIGNAL_ID; if(val) value |= SIGNAL_ID; }
        constexpr bool signalStrength() const { return (value & SIGNAL_STRENGTH) > 0; }
        constexpr void signalStrength(bool val) { value &= ~SIGNAL_STRENGTH; if(val) value |= SIGNAL_STRENGTH; }
        constexpr bool quality() const { return (value & QUALITY) > 0; }
        constexpr void quality(bool val) { value &= ~QUALITY; if(val) value |= QUALITY; }
        constexpr bool pseudorange() const { return (value & PSEUDORANGE) > 0; }
        constexpr void pseudorange(bool val) { value &= ~PSEUDORANGE; if(val) value |= PSEUDORANGE; }
        constexpr bool carrierPhase() const { return (value & CARRIER_PHASE) > 0; }
        constexpr void carrierPhase(bool val) { value &= ~CARRIER_PHASE; if(val) value |= CARRIER_PHASE; }
        constexpr bool doppler() const { return (value & DOPPLER) > 0; }
        constexpr void doppler(bool val) { value &= ~DOPPLER; if(val) value |= DOPPLER; }
        constexpr bool rangeUncertainty() const { return (value & RANGE_UNCERTAINTY) > 0; }
        constexpr void rangeUncertainty(bool val) { value &= ~RANGE_UNCERTAINTY; if(val) value |= RANGE_UNCERTAINTY; }
        constexpr bool carrierPhaseUncertainty() const { return (value & CARRIER_PHASE_UNCERTAINTY) > 0; }
        constexpr void carrierPhaseUncertainty(bool val) { value &= ~CARRIER_PHASE_UNCERTAINTY; if(val) value |= CARRIER_PHASE_UNCERTAINTY; }
        constexpr bool dopplerUncertainty() const { return (value & DOPPLER_UNCERTAINTY) > 0; }
        constexpr void dopplerUncertainty(bool val) { value &= ~DOPPLER_UNCERTAINTY; if(val) value |= DOPPLER_UNCERTAINTY; }
        constexpr bool lockTime() const { return (value & LOCK_TIME) > 0; }
        constexpr void lockTime(bool val) { value &= ~LOCK_TIME; if(val) value |= LOCK_TIME; }
        constexpr uint16_t flags() const { return (value & FLAGS) >> 0; }
        constexpr void flags(uint16_t val) { value = (value & ~FLAGS) | (val << 0); }
        constexpr bool allSet() const { return value == ALL; }
        constexpr void setAll() { value |= ALL; }
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
///@defgroup gnss_gps_ephemeris_cpp  (0x81,0x61) Gps Ephemeris
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
            ISC_L5      = 0x0004,  ///<  
            FLAGS       = 0x0007,  ///<  
            ALL         = 0x0007,
        };
        uint16_t value = NONE;
        
        constexpr ValidFlags() : value(NONE) {}
        constexpr ValidFlags(int val) : value((uint16_t)val) {}
        constexpr operator uint16_t() const { return value; }
        constexpr ValidFlags& operator=(uint16_t val) { value = val; return *this; }
        constexpr ValidFlags& operator=(int val) { value = uint16_t(val); return *this; }
        constexpr ValidFlags& operator|=(uint16_t val) { return *this = value | val; }
        constexpr ValidFlags& operator&=(uint16_t val) { return *this = value & val; }
        
        constexpr bool ephemeris() const { return (value & EPHEMERIS) > 0; }
        constexpr void ephemeris(bool val) { value &= ~EPHEMERIS; if(val) value |= EPHEMERIS; }
        constexpr bool modernData() const { return (value & MODERN_DATA) > 0; }
        constexpr void modernData(bool val) { value &= ~MODERN_DATA; if(val) value |= MODERN_DATA; }
        constexpr bool iscL5() const { return (value & ISC_L5) > 0; }
        constexpr void iscL5(bool val) { value &= ~ISC_L5; if(val) value |= ISC_L5; }
        constexpr uint16_t flags() const { return (value & FLAGS) >> 0; }
        constexpr void flags(uint16_t val) { value = (value & ~FLAGS) | (val << 0); }
        constexpr bool allSet() const { return value == ALL; }
        constexpr void setAll() { value |= ALL; }
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
    double ISC_L1CA = 0; ///< Inter-signal correction (L1).
    double ISC_L2C = 0; ///< Inter-signal correction (L2, or L5 if isc_l5 flag is set).
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
    static constexpr const char* DOC_NAME = "GPS Ephemeris";
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
///@defgroup gnss_galileo_ephemeris_cpp  (0x81,0x63) Galileo Ephemeris
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
            ISC_L5      = 0x0004,  ///<  
            FLAGS       = 0x0007,  ///<  
            ALL         = 0x0007,
        };
        uint16_t value = NONE;
        
        constexpr ValidFlags() : value(NONE) {}
        constexpr ValidFlags(int val) : value((uint16_t)val) {}
        constexpr operator uint16_t() const { return value; }
        constexpr ValidFlags& operator=(uint16_t val) { value = val; return *this; }
        constexpr ValidFlags& operator=(int val) { value = uint16_t(val); return *this; }
        constexpr ValidFlags& operator|=(uint16_t val) { return *this = value | val; }
        constexpr ValidFlags& operator&=(uint16_t val) { return *this = value & val; }
        
        constexpr bool ephemeris() const { return (value & EPHEMERIS) > 0; }
        constexpr void ephemeris(bool val) { value &= ~EPHEMERIS; if(val) value |= EPHEMERIS; }
        constexpr bool modernData() const { return (value & MODERN_DATA) > 0; }
        constexpr void modernData(bool val) { value &= ~MODERN_DATA; if(val) value |= MODERN_DATA; }
        constexpr bool iscL5() const { return (value & ISC_L5) > 0; }
        constexpr void iscL5(bool val) { value &= ~ISC_L5; if(val) value |= ISC_L5; }
        constexpr uint16_t flags() const { return (value & FLAGS) >> 0; }
        constexpr void flags(uint16_t val) { value = (value & ~FLAGS) | (val << 0); }
        constexpr bool allSet() const { return value == ALL; }
        constexpr void setAll() { value |= ALL; }
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
    double ISC_L1CA = 0; ///< Inter-signal correction (L1).
    double ISC_L2C = 0; ///< Inter-signal correction (L2, or L5 if isc_l5 flag is set).
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
    static constexpr const char* DOC_NAME = "Galileo Ephemeris";
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
///@defgroup gnss_glo_ephemeris_cpp  (0x81,0x62) Glo Ephemeris
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
        
        constexpr ValidFlags() : value(NONE) {}
        constexpr ValidFlags(int val) : value((uint16_t)val) {}
        constexpr operator uint16_t() const { return value; }
        constexpr ValidFlags& operator=(uint16_t val) { value = val; return *this; }
        constexpr ValidFlags& operator=(int val) { value = uint16_t(val); return *this; }
        constexpr ValidFlags& operator|=(uint16_t val) { return *this = value | val; }
        constexpr ValidFlags& operator&=(uint16_t val) { return *this = value & val; }
        
        constexpr bool ephemeris() const { return (value & EPHEMERIS) > 0; }
        constexpr void ephemeris(bool val) { value &= ~EPHEMERIS; if(val) value |= EPHEMERIS; }
        constexpr bool flags() const { return (value & FLAGS) > 0; }
        constexpr void flags(bool val) { value &= ~FLAGS; if(val) value |= FLAGS; }
        constexpr bool allSet() const { return value == ALL; }
        constexpr void setAll() { value |= ALL; }
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
///@defgroup gnss_beidou_ephemeris_cpp  (0x81,0x64) Beidou Ephemeris
/// BeiDou Ephemeris Data
///
///@{

struct BeidouEphemeris
{
    struct ValidFlags : Bitfield<ValidFlags>
    {
        typedef uint16_t Type;
        enum _enumType : uint16_t
        {
            NONE        = 0x0000,
            EPHEMERIS   = 0x0001,  ///<  
            MODERN_DATA = 0x0002,  ///<  
            ISC_L5      = 0x0004,  ///<  
            FLAGS       = 0x0007,  ///<  
            ALL         = 0x0007,
        };
        uint16_t value = NONE;
        
        constexpr ValidFlags() : value(NONE) {}
        constexpr ValidFlags(int val) : value((uint16_t)val) {}
        constexpr operator uint16_t() const { return value; }
        constexpr ValidFlags& operator=(uint16_t val) { value = val; return *this; }
        constexpr ValidFlags& operator=(int val) { value = uint16_t(val); return *this; }
        constexpr ValidFlags& operator|=(uint16_t val) { return *this = value | val; }
        constexpr ValidFlags& operator&=(uint16_t val) { return *this = value & val; }
        
        constexpr bool ephemeris() const { return (value & EPHEMERIS) > 0; }
        constexpr void ephemeris(bool val) { value &= ~EPHEMERIS; if(val) value |= EPHEMERIS; }
        constexpr bool modernData() const { return (value & MODERN_DATA) > 0; }
        constexpr void modernData(bool val) { value &= ~MODERN_DATA; if(val) value |= MODERN_DATA; }
        constexpr bool iscL5() const { return (value & ISC_L5) > 0; }
        constexpr void iscL5(bool val) { value &= ~ISC_L5; if(val) value |= ISC_L5; }
        constexpr uint16_t flags() const { return (value & FLAGS) >> 0; }
        constexpr void flags(uint16_t val) { value = (value & ~FLAGS) | (val << 0); }
        constexpr bool allSet() const { return value == ALL; }
        constexpr void setAll() { value |= ALL; }
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
    double ISC_L1CA = 0; ///< Inter-signal correction (L1).
    double ISC_L2C = 0; ///< Inter-signal correction (L2, or L5 if isc_l5 flag is set).
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
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_gnss::DATA_BEIDOU_EPHEMERIS;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "BeidouEphemeris";
    static constexpr const char* DOC_NAME = "BeiDou Ephemeris";
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
///@defgroup gnss_gps_iono_corr_cpp  (0x81,0x71) Gps Iono Corr
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
        
        constexpr ValidFlags() : value(NONE) {}
        constexpr ValidFlags(int val) : value((uint16_t)val) {}
        constexpr operator uint16_t() const { return value; }
        constexpr ValidFlags& operator=(uint16_t val) { value = val; return *this; }
        constexpr ValidFlags& operator=(int val) { value = uint16_t(val); return *this; }
        constexpr ValidFlags& operator|=(uint16_t val) { return *this = value | val; }
        constexpr ValidFlags& operator&=(uint16_t val) { return *this = value & val; }
        
        constexpr bool tow() const { return (value & TOW) > 0; }
        constexpr void tow(bool val) { value &= ~TOW; if(val) value |= TOW; }
        constexpr bool weekNumber() const { return (value & WEEK_NUMBER) > 0; }
        constexpr void weekNumber(bool val) { value &= ~WEEK_NUMBER; if(val) value |= WEEK_NUMBER; }
        constexpr bool alpha() const { return (value & ALPHA) > 0; }
        constexpr void alpha(bool val) { value &= ~ALPHA; if(val) value |= ALPHA; }
        constexpr bool beta() const { return (value & BETA) > 0; }
        constexpr void beta(bool val) { value &= ~BETA; if(val) value |= BETA; }
        constexpr uint16_t flags() const { return (value & FLAGS) >> 0; }
        constexpr void flags(uint16_t val) { value = (value & ~FLAGS) | (val << 0); }
        constexpr bool allSet() const { return value == ALL; }
        constexpr void setAll() { value |= ALL; }
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
///@defgroup gnss_galileo_iono_corr_cpp  (0x81,0x73) Galileo Iono Corr
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
        
        constexpr ValidFlags() : value(NONE) {}
        constexpr ValidFlags(int val) : value((uint16_t)val) {}
        constexpr operator uint16_t() const { return value; }
        constexpr ValidFlags& operator=(uint16_t val) { value = val; return *this; }
        constexpr ValidFlags& operator=(int val) { value = uint16_t(val); return *this; }
        constexpr ValidFlags& operator|=(uint16_t val) { return *this = value | val; }
        constexpr ValidFlags& operator&=(uint16_t val) { return *this = value & val; }
        
        constexpr bool tow() const { return (value & TOW) > 0; }
        constexpr void tow(bool val) { value &= ~TOW; if(val) value |= TOW; }
        constexpr bool weekNumber() const { return (value & WEEK_NUMBER) > 0; }
        constexpr void weekNumber(bool val) { value &= ~WEEK_NUMBER; if(val) value |= WEEK_NUMBER; }
        constexpr bool alpha() const { return (value & ALPHA) > 0; }
        constexpr void alpha(bool val) { value &= ~ALPHA; if(val) value |= ALPHA; }
        constexpr bool disturbanceFlags() const { return (value & DISTURBANCE_FLAGS) > 0; }
        constexpr void disturbanceFlags(bool val) { value &= ~DISTURBANCE_FLAGS; if(val) value |= DISTURBANCE_FLAGS; }
        constexpr uint16_t flags() const { return (value & FLAGS) >> 0; }
        constexpr void flags(uint16_t val) { value = (value & ~FLAGS) | (val << 0); }
        constexpr bool allSet() const { return value == ALL; }
        constexpr void setAll() { value |= ALL; }
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
////////////////////////////////////////////////////////////////////////////////
///@defgroup gnss_beidou_iono_corr_cpp  (0x81,0x74) Beidou Iono Corr
/// Ionospheric Correction Terms for BeiDou
///
///@{

struct BeidouIonoCorr
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
            ALPHA_CORR  = 0x0010,  ///<  
            FLAGS       = 0x001F,  ///<  
            ALL         = 0x001F,
        };
        uint16_t value = NONE;
        
        constexpr ValidFlags() : value(NONE) {}
        constexpr ValidFlags(int val) : value((uint16_t)val) {}
        constexpr operator uint16_t() const { return value; }
        constexpr ValidFlags& operator=(uint16_t val) { value = val; return *this; }
        constexpr ValidFlags& operator=(int val) { value = uint16_t(val); return *this; }
        constexpr ValidFlags& operator|=(uint16_t val) { return *this = value | val; }
        constexpr ValidFlags& operator&=(uint16_t val) { return *this = value & val; }
        
        constexpr bool tow() const { return (value & TOW) > 0; }
        constexpr void tow(bool val) { value &= ~TOW; if(val) value |= TOW; }
        constexpr bool weekNumber() const { return (value & WEEK_NUMBER) > 0; }
        constexpr void weekNumber(bool val) { value &= ~WEEK_NUMBER; if(val) value |= WEEK_NUMBER; }
        constexpr bool alpha() const { return (value & ALPHA) > 0; }
        constexpr void alpha(bool val) { value &= ~ALPHA; if(val) value |= ALPHA; }
        constexpr bool beta() const { return (value & BETA) > 0; }
        constexpr void beta(bool val) { value &= ~BETA; if(val) value |= BETA; }
        constexpr bool alphaCorr() const { return (value & ALPHA_CORR) > 0; }
        constexpr void alphaCorr(bool val) { value &= ~ALPHA_CORR; if(val) value |= ALPHA_CORR; }
        constexpr uint16_t flags() const { return (value & FLAGS) >> 0; }
        constexpr void flags(uint16_t val) { value = (value & ~FLAGS) | (val << 0); }
        constexpr bool allSet() const { return value == ALL; }
        constexpr void setAll() { value |= ALL; }
    };
    /// Parameters
    double time_of_week = 0; ///< GPS Time of week [seconds]
    uint16_t week_number = 0; ///< GPS Week since 1980 [weeks]
    double alpha[4] = {0}; ///< Ionospheric Delay Terms.
    double beta[4] = {0}; ///< Ionospheric Delay Terms.
    double alpha_corr[9] = {0}; ///< Ionospheric Delay Correction Terms.
    ValidFlags valid_flags;
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_gnss::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_gnss::DATA_BEIDOU_IONO_CORR;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "BeidouIonoCorr";
    static constexpr const char* DOC_NAME = "BeiDou Ionospheric Correction";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(time_of_week,week_number,alpha,beta,alpha_corr,valid_flags);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(time_of_week),std::ref(week_number),std::ref(alpha),std::ref(beta),std::ref(alpha_corr),std::ref(valid_flags));
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

