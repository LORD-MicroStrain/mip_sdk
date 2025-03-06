#pragma once

#include <mip/definitions/common.h>
#include <mip/mip_descriptors.h>
#include <mip/mip_result.h>
#include <mip/mip_interface.h>

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
namespace mip {
namespace C {
extern "C" {

#endif // __cplusplus

////////////////////////////////////////////////////////////////////////////////
///@addtogroup MipData_c
///@{
///@defgroup gnss_data_c  Gnss Data
///
///@{

////////////////////////////////////////////////////////////////////////////////
// Descriptors
////////////////////////////////////////////////////////////////////////////////

enum 
{
    MIP_GNSS_DATA_DESC_SET                     = 0x81,
    
    MIP_DATA_DESC_GNSS_POSITION_LLH            = 0x03,
    MIP_DATA_DESC_GNSS_POSITION_ECEF           = 0x04,
    MIP_DATA_DESC_GNSS_VELOCITY_NED            = 0x05,
    MIP_DATA_DESC_GNSS_VELOCITY_ECEF           = 0x06,
    MIP_DATA_DESC_GNSS_DOP                     = 0x07,
    MIP_DATA_DESC_GNSS_UTC_TIME                = 0x08,
    MIP_DATA_DESC_GNSS_GPS_TIME                = 0x09,
    MIP_DATA_DESC_GNSS_CLOCK_INFO              = 0x0A,
    MIP_DATA_DESC_GNSS_FIX_INFO                = 0x0B,
    MIP_DATA_DESC_GNSS_SV_INFO                 = 0x0C,
    MIP_DATA_DESC_GNSS_HW_STATUS               = 0x0D,
    MIP_DATA_DESC_GNSS_DGPS_INFO               = 0x0E,
    MIP_DATA_DESC_GNSS_DGPS_CHANNEL_STATUS     = 0x0F,
    MIP_DATA_DESC_GNSS_CLOCK_INFO_2            = 0x10,
    MIP_DATA_DESC_GNSS_GPS_LEAP_SECONDS        = 0x11,
    MIP_DATA_DESC_GNSS_SBAS_INFO               = 0x12,
    MIP_DATA_DESC_GNSS_SBAS_CORRECTION         = 0x13,
    MIP_DATA_DESC_GNSS_RF_ERROR_DETECTION      = 0x14,
    MIP_DATA_DESC_GNSS_SATELLITE_STATUS        = 0x20,
    MIP_DATA_DESC_GNSS_SATELLITE_SIGNAL_STATUS = 0x21,
    MIP_DATA_DESC_GNSS_RAW                     = 0x22,
    MIP_DATA_DESC_GNSS_BASE_STATION_INFO       = 0x30,
    MIP_DATA_DESC_GNSS_RTK_CORRECTIONS_STATUS  = 0x31,
    MIP_DATA_DESC_GNSS_GPS_EPHEMERIS           = 0x61,
    MIP_DATA_DESC_GNSS_GLONASS_EPHEMERIS       = 0x62,
    MIP_DATA_DESC_GNSS_GALILEO_EPHEMERIS       = 0x63,
    MIP_DATA_DESC_GNSS_BEIDOU_EPHEMERIS        = 0x64,
    MIP_DATA_DESC_GNSS_GPS_IONO_CORR           = 0x71,
    MIP_DATA_DESC_GNSS_GALILEO_IONO_CORR       = 0x73,
    MIP_DATA_DESC_GNSS_BEIDOU_IONO_CORR        = 0x74,
    
};

////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////

enum { MIP_GNSS1_DATA_DESC_SET = 0x91 };
enum { MIP_GNSS2_DATA_DESC_SET = 0x92 };
enum { MIP_GNSS3_DATA_DESC_SET = 0x93 };
enum { MIP_GNSS4_DATA_DESC_SET = 0x94 };
enum { MIP_GNSS5_DATA_DESC_SET = 0x95 };
enum mip_gnss_constellation_id
{
    MIP_GNSS_CONSTELLATION_ID_UNKNOWN = 0,  ///<  
    MIP_GNSS_CONSTELLATION_ID_GPS     = 1,  ///<  
    MIP_GNSS_CONSTELLATION_ID_GLONASS = 2,  ///<  
    MIP_GNSS_CONSTELLATION_ID_GALILEO = 3,  ///<  
    MIP_GNSS_CONSTELLATION_ID_BEIDOU  = 4,  ///<  
    MIP_GNSS_CONSTELLATION_ID_SBAS    = 5,  ///<  
};
typedef enum mip_gnss_constellation_id mip_gnss_constellation_id;

static inline void insert_mip_gnss_constellation_id(microstrain_serializer* serializer, const mip_gnss_constellation_id self)
{
    microstrain_insert_u8(serializer, (uint8_t)(self));
}
static inline void extract_mip_gnss_constellation_id(microstrain_serializer* serializer, mip_gnss_constellation_id* self)
{
    uint8_t tmp = 0;
    microstrain_extract_u8(serializer, &tmp);
    *self = (mip_gnss_constellation_id)tmp;
}

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
    MIP_GNSS_SIGNAL_ID_BEIDOU_B2A     = 169,  ///<  
};
typedef enum mip_gnss_signal_id mip_gnss_signal_id;

static inline void insert_mip_gnss_signal_id(microstrain_serializer* serializer, const mip_gnss_signal_id self)
{
    microstrain_insert_u8(serializer, (uint8_t)(self));
}
static inline void extract_mip_gnss_signal_id(microstrain_serializer* serializer, mip_gnss_signal_id* self)
{
    uint8_t tmp = 0;
    microstrain_extract_u8(serializer, &tmp);
    *self = (mip_gnss_signal_id)tmp;
}

enum mip_sbas_system
{
    MIP_SBAS_SYSTEM_UNKNOWN = 0,  ///<  
    MIP_SBAS_SYSTEM_WAAS    = 1,  ///<  
    MIP_SBAS_SYSTEM_EGNOS   = 2,  ///<  
    MIP_SBAS_SYSTEM_MSAS    = 3,  ///<  
    MIP_SBAS_SYSTEM_GAGAN   = 4,  ///<  
};
typedef enum mip_sbas_system mip_sbas_system;

static inline void insert_mip_sbas_system(microstrain_serializer* serializer, const mip_sbas_system self)
{
    microstrain_insert_u8(serializer, (uint8_t)(self));
}
static inline void extract_mip_sbas_system(microstrain_serializer* serializer, mip_sbas_system* self)
{
    uint8_t tmp = 0;
    microstrain_extract_u8(serializer, &tmp);
    *self = (mip_sbas_system)tmp;
}

enum { MIP_GNSS_DGPS_INFO_MAX_CHANNEL_NUMBER = 32 };
enum { MIP_GNSS_SV_INFO_MAX_SV_NUMBER = 32 };

////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
///@defgroup gnss_pos_llh_c  (0x81,0x03) Pos Llh
/// GNSS reported position in the WGS84 geodetic frame
///
///@{

typedef uint16_t mip_gnss_pos_llh_data_valid_flags;
static const mip_gnss_pos_llh_data_valid_flags MIP_GNSS_POS_LLH_DATA_VALID_FLAGS_NONE                = 0x0000;
static const mip_gnss_pos_llh_data_valid_flags MIP_GNSS_POS_LLH_DATA_VALID_FLAGS_LAT_LON             = 0x0001; ///<  
static const mip_gnss_pos_llh_data_valid_flags MIP_GNSS_POS_LLH_DATA_VALID_FLAGS_ELLIPSOID_HEIGHT    = 0x0002; ///<  
static const mip_gnss_pos_llh_data_valid_flags MIP_GNSS_POS_LLH_DATA_VALID_FLAGS_MSL_HEIGHT          = 0x0004; ///<  
static const mip_gnss_pos_llh_data_valid_flags MIP_GNSS_POS_LLH_DATA_VALID_FLAGS_HORIZONTAL_ACCURACY = 0x0008; ///<  
static const mip_gnss_pos_llh_data_valid_flags MIP_GNSS_POS_LLH_DATA_VALID_FLAGS_VERTICAL_ACCURACY   = 0x0010; ///<  
static const mip_gnss_pos_llh_data_valid_flags MIP_GNSS_POS_LLH_DATA_VALID_FLAGS_FLAGS               = 0x001F; ///<  
static const mip_gnss_pos_llh_data_valid_flags MIP_GNSS_POS_LLH_DATA_VALID_FLAGS_ALL                 = 0x001F;
static inline void insert_mip_gnss_pos_llh_data_valid_flags(microstrain_serializer* serializer, const mip_gnss_pos_llh_data_valid_flags self)
{
    microstrain_insert_u16(serializer, (uint16_t)(self));
}
static inline void extract_mip_gnss_pos_llh_data_valid_flags(microstrain_serializer* serializer, mip_gnss_pos_llh_data_valid_flags* self)
{
    uint16_t tmp = 0;
    microstrain_extract_u16(serializer, &tmp);
    *self = (mip_gnss_pos_llh_data_valid_flags)tmp;
}


struct mip_gnss_pos_llh_data
{
    double latitude; ///< [degrees]
    double longitude; ///< [degrees]
    double ellipsoid_height; ///< [meters]
    double msl_height; ///< [meters]
    float horizontal_accuracy; ///< [meters]
    float vertical_accuracy; ///< [meters]
    mip_gnss_pos_llh_data_valid_flags valid_flags;
};
typedef struct mip_gnss_pos_llh_data mip_gnss_pos_llh_data;

void insert_mip_gnss_pos_llh_data(microstrain_serializer* serializer, const mip_gnss_pos_llh_data* self);
void extract_mip_gnss_pos_llh_data(microstrain_serializer* serializer, mip_gnss_pos_llh_data* self);
bool extract_mip_gnss_pos_llh_data_from_field(const mip_field_view* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup gnss_pos_ecef_c  (0x81,0x04) Pos Ecef
/// GNSS reported position in the Earth-centered, Earth-Fixed (ECEF) frame
///
///@{

typedef uint16_t mip_gnss_pos_ecef_data_valid_flags;
static const mip_gnss_pos_ecef_data_valid_flags MIP_GNSS_POS_ECEF_DATA_VALID_FLAGS_NONE              = 0x0000;
static const mip_gnss_pos_ecef_data_valid_flags MIP_GNSS_POS_ECEF_DATA_VALID_FLAGS_POSITION          = 0x0001; ///<  
static const mip_gnss_pos_ecef_data_valid_flags MIP_GNSS_POS_ECEF_DATA_VALID_FLAGS_POSITION_ACCURACY = 0x0002; ///<  
static const mip_gnss_pos_ecef_data_valid_flags MIP_GNSS_POS_ECEF_DATA_VALID_FLAGS_FLAGS             = 0x0003; ///<  
static const mip_gnss_pos_ecef_data_valid_flags MIP_GNSS_POS_ECEF_DATA_VALID_FLAGS_ALL               = 0x0003;
static inline void insert_mip_gnss_pos_ecef_data_valid_flags(microstrain_serializer* serializer, const mip_gnss_pos_ecef_data_valid_flags self)
{
    microstrain_insert_u16(serializer, (uint16_t)(self));
}
static inline void extract_mip_gnss_pos_ecef_data_valid_flags(microstrain_serializer* serializer, mip_gnss_pos_ecef_data_valid_flags* self)
{
    uint16_t tmp = 0;
    microstrain_extract_u16(serializer, &tmp);
    *self = (mip_gnss_pos_ecef_data_valid_flags)tmp;
}


struct mip_gnss_pos_ecef_data
{
    mip_vector3d x; ///< [meters]
    float x_accuracy; ///< [meters]
    mip_gnss_pos_ecef_data_valid_flags valid_flags;
};
typedef struct mip_gnss_pos_ecef_data mip_gnss_pos_ecef_data;

void insert_mip_gnss_pos_ecef_data(microstrain_serializer* serializer, const mip_gnss_pos_ecef_data* self);
void extract_mip_gnss_pos_ecef_data(microstrain_serializer* serializer, mip_gnss_pos_ecef_data* self);
bool extract_mip_gnss_pos_ecef_data_from_field(const mip_field_view* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup gnss_vel_ned_c  (0x81,0x05) Vel Ned
/// GNSS reported velocity in the NED frame
///
///@{

typedef uint16_t mip_gnss_vel_ned_data_valid_flags;
static const mip_gnss_vel_ned_data_valid_flags MIP_GNSS_VEL_NED_DATA_VALID_FLAGS_NONE             = 0x0000;
static const mip_gnss_vel_ned_data_valid_flags MIP_GNSS_VEL_NED_DATA_VALID_FLAGS_VELOCITY         = 0x0001; ///<  
static const mip_gnss_vel_ned_data_valid_flags MIP_GNSS_VEL_NED_DATA_VALID_FLAGS_SPEED_3D         = 0x0002; ///<  
static const mip_gnss_vel_ned_data_valid_flags MIP_GNSS_VEL_NED_DATA_VALID_FLAGS_GROUND_SPEED     = 0x0004; ///<  
static const mip_gnss_vel_ned_data_valid_flags MIP_GNSS_VEL_NED_DATA_VALID_FLAGS_HEADING          = 0x0008; ///<  
static const mip_gnss_vel_ned_data_valid_flags MIP_GNSS_VEL_NED_DATA_VALID_FLAGS_SPEED_ACCURACY   = 0x0010; ///<  
static const mip_gnss_vel_ned_data_valid_flags MIP_GNSS_VEL_NED_DATA_VALID_FLAGS_HEADING_ACCURACY = 0x0020; ///<  
static const mip_gnss_vel_ned_data_valid_flags MIP_GNSS_VEL_NED_DATA_VALID_FLAGS_FLAGS            = 0x003F; ///<  
static const mip_gnss_vel_ned_data_valid_flags MIP_GNSS_VEL_NED_DATA_VALID_FLAGS_ALL              = 0x003F;
static inline void insert_mip_gnss_vel_ned_data_valid_flags(microstrain_serializer* serializer, const mip_gnss_vel_ned_data_valid_flags self)
{
    microstrain_insert_u16(serializer, (uint16_t)(self));
}
static inline void extract_mip_gnss_vel_ned_data_valid_flags(microstrain_serializer* serializer, mip_gnss_vel_ned_data_valid_flags* self)
{
    uint16_t tmp = 0;
    microstrain_extract_u16(serializer, &tmp);
    *self = (mip_gnss_vel_ned_data_valid_flags)tmp;
}


struct mip_gnss_vel_ned_data
{
    mip_vector3f v; ///< [meters/second]
    float speed; ///< [meters/second]
    float ground_speed; ///< [meters/second]
    float heading; ///< [degrees]
    float speed_accuracy; ///< [meters/second]
    float heading_accuracy; ///< [degrees]
    mip_gnss_vel_ned_data_valid_flags valid_flags;
};
typedef struct mip_gnss_vel_ned_data mip_gnss_vel_ned_data;

void insert_mip_gnss_vel_ned_data(microstrain_serializer* serializer, const mip_gnss_vel_ned_data* self);
void extract_mip_gnss_vel_ned_data(microstrain_serializer* serializer, mip_gnss_vel_ned_data* self);
bool extract_mip_gnss_vel_ned_data_from_field(const mip_field_view* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup gnss_vel_ecef_c  (0x81,0x06) Vel Ecef
/// GNSS reported velocity in the Earth-centered, Earth-Fixed (ECEF) frame
///
///@{

typedef uint16_t mip_gnss_vel_ecef_data_valid_flags;
static const mip_gnss_vel_ecef_data_valid_flags MIP_GNSS_VEL_ECEF_DATA_VALID_FLAGS_NONE              = 0x0000;
static const mip_gnss_vel_ecef_data_valid_flags MIP_GNSS_VEL_ECEF_DATA_VALID_FLAGS_VELOCITY          = 0x0001; ///<  
static const mip_gnss_vel_ecef_data_valid_flags MIP_GNSS_VEL_ECEF_DATA_VALID_FLAGS_VELOCITY_ACCURACY = 0x0002; ///<  
static const mip_gnss_vel_ecef_data_valid_flags MIP_GNSS_VEL_ECEF_DATA_VALID_FLAGS_FLAGS             = 0x0003; ///<  
static const mip_gnss_vel_ecef_data_valid_flags MIP_GNSS_VEL_ECEF_DATA_VALID_FLAGS_ALL               = 0x0003;
static inline void insert_mip_gnss_vel_ecef_data_valid_flags(microstrain_serializer* serializer, const mip_gnss_vel_ecef_data_valid_flags self)
{
    microstrain_insert_u16(serializer, (uint16_t)(self));
}
static inline void extract_mip_gnss_vel_ecef_data_valid_flags(microstrain_serializer* serializer, mip_gnss_vel_ecef_data_valid_flags* self)
{
    uint16_t tmp = 0;
    microstrain_extract_u16(serializer, &tmp);
    *self = (mip_gnss_vel_ecef_data_valid_flags)tmp;
}


struct mip_gnss_vel_ecef_data
{
    mip_vector3f v; ///< [meters/second]
    float v_accuracy; ///< [meters/second]
    mip_gnss_vel_ecef_data_valid_flags valid_flags;
};
typedef struct mip_gnss_vel_ecef_data mip_gnss_vel_ecef_data;

void insert_mip_gnss_vel_ecef_data(microstrain_serializer* serializer, const mip_gnss_vel_ecef_data* self);
void extract_mip_gnss_vel_ecef_data(microstrain_serializer* serializer, mip_gnss_vel_ecef_data* self);
bool extract_mip_gnss_vel_ecef_data_from_field(const mip_field_view* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup gnss_dop_c  (0x81,0x07) Dop
/// GNSS reported dilution of precision information.
///
///@{

typedef uint16_t mip_gnss_dop_data_valid_flags;
static const mip_gnss_dop_data_valid_flags MIP_GNSS_DOP_DATA_VALID_FLAGS_NONE  = 0x0000;
static const mip_gnss_dop_data_valid_flags MIP_GNSS_DOP_DATA_VALID_FLAGS_GDOP  = 0x0001; ///<  
static const mip_gnss_dop_data_valid_flags MIP_GNSS_DOP_DATA_VALID_FLAGS_PDOP  = 0x0002; ///<  
static const mip_gnss_dop_data_valid_flags MIP_GNSS_DOP_DATA_VALID_FLAGS_HDOP  = 0x0004; ///<  
static const mip_gnss_dop_data_valid_flags MIP_GNSS_DOP_DATA_VALID_FLAGS_VDOP  = 0x0008; ///<  
static const mip_gnss_dop_data_valid_flags MIP_GNSS_DOP_DATA_VALID_FLAGS_TDOP  = 0x0010; ///<  
static const mip_gnss_dop_data_valid_flags MIP_GNSS_DOP_DATA_VALID_FLAGS_NDOP  = 0x0020; ///<  
static const mip_gnss_dop_data_valid_flags MIP_GNSS_DOP_DATA_VALID_FLAGS_EDOP  = 0x0040; ///<  
static const mip_gnss_dop_data_valid_flags MIP_GNSS_DOP_DATA_VALID_FLAGS_FLAGS = 0x007F; ///<  
static const mip_gnss_dop_data_valid_flags MIP_GNSS_DOP_DATA_VALID_FLAGS_ALL   = 0x007F;
static inline void insert_mip_gnss_dop_data_valid_flags(microstrain_serializer* serializer, const mip_gnss_dop_data_valid_flags self)
{
    microstrain_insert_u16(serializer, (uint16_t)(self));
}
static inline void extract_mip_gnss_dop_data_valid_flags(microstrain_serializer* serializer, mip_gnss_dop_data_valid_flags* self)
{
    uint16_t tmp = 0;
    microstrain_extract_u16(serializer, &tmp);
    *self = (mip_gnss_dop_data_valid_flags)tmp;
}


struct mip_gnss_dop_data
{
    float gdop; ///< Geometric DOP
    float pdop; ///< Position DOP
    float hdop; ///< Horizontal DOP
    float vdop; ///< Vertical DOP
    float tdop; ///< Time DOP
    float ndop; ///< Northing DOP
    float edop; ///< Easting DOP
    mip_gnss_dop_data_valid_flags valid_flags;
};
typedef struct mip_gnss_dop_data mip_gnss_dop_data;

void insert_mip_gnss_dop_data(microstrain_serializer* serializer, const mip_gnss_dop_data* self);
void extract_mip_gnss_dop_data(microstrain_serializer* serializer, mip_gnss_dop_data* self);
bool extract_mip_gnss_dop_data_from_field(const mip_field_view* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup gnss_utc_time_c  (0x81,0x08) Utc Time
/// GNSS reported Coordinated Universal Time
///
///@{

typedef uint16_t mip_gnss_utc_time_data_valid_flags;
static const mip_gnss_utc_time_data_valid_flags MIP_GNSS_UTC_TIME_DATA_VALID_FLAGS_NONE               = 0x0000;
static const mip_gnss_utc_time_data_valid_flags MIP_GNSS_UTC_TIME_DATA_VALID_FLAGS_GNSS_DATE_TIME     = 0x0001; ///<  
static const mip_gnss_utc_time_data_valid_flags MIP_GNSS_UTC_TIME_DATA_VALID_FLAGS_LEAP_SECONDS_KNOWN = 0x0002; ///<  
static const mip_gnss_utc_time_data_valid_flags MIP_GNSS_UTC_TIME_DATA_VALID_FLAGS_FLAGS              = 0x0003; ///<  
static const mip_gnss_utc_time_data_valid_flags MIP_GNSS_UTC_TIME_DATA_VALID_FLAGS_ALL                = 0x0003;
static inline void insert_mip_gnss_utc_time_data_valid_flags(microstrain_serializer* serializer, const mip_gnss_utc_time_data_valid_flags self)
{
    microstrain_insert_u16(serializer, (uint16_t)(self));
}
static inline void extract_mip_gnss_utc_time_data_valid_flags(microstrain_serializer* serializer, mip_gnss_utc_time_data_valid_flags* self)
{
    uint16_t tmp = 0;
    microstrain_extract_u16(serializer, &tmp);
    *self = (mip_gnss_utc_time_data_valid_flags)tmp;
}


struct mip_gnss_utc_time_data
{
    uint16_t year;
    uint8_t month; ///< Month (1-12)
    uint8_t day; ///< Day (1-31)
    uint8_t hour; ///< Hour (0-23)
    uint8_t min; ///< Minute (0-59)
    uint8_t sec; ///< Second (0-59)
    uint32_t msec; ///< Millisecond(0-999)
    mip_gnss_utc_time_data_valid_flags valid_flags;
};
typedef struct mip_gnss_utc_time_data mip_gnss_utc_time_data;

void insert_mip_gnss_utc_time_data(microstrain_serializer* serializer, const mip_gnss_utc_time_data* self);
void extract_mip_gnss_utc_time_data(microstrain_serializer* serializer, mip_gnss_utc_time_data* self);
bool extract_mip_gnss_utc_time_data_from_field(const mip_field_view* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup gnss_gps_time_c  (0x81,0x09) Gps Time
/// GNSS reported GPS Time
///
///@{

typedef uint16_t mip_gnss_gps_time_data_valid_flags;
static const mip_gnss_gps_time_data_valid_flags MIP_GNSS_GPS_TIME_DATA_VALID_FLAGS_NONE        = 0x0000;
static const mip_gnss_gps_time_data_valid_flags MIP_GNSS_GPS_TIME_DATA_VALID_FLAGS_TOW         = 0x0001; ///<  
static const mip_gnss_gps_time_data_valid_flags MIP_GNSS_GPS_TIME_DATA_VALID_FLAGS_WEEK_NUMBER = 0x0002; ///<  
static const mip_gnss_gps_time_data_valid_flags MIP_GNSS_GPS_TIME_DATA_VALID_FLAGS_FLAGS       = 0x0003; ///<  
static const mip_gnss_gps_time_data_valid_flags MIP_GNSS_GPS_TIME_DATA_VALID_FLAGS_ALL         = 0x0003;
static inline void insert_mip_gnss_gps_time_data_valid_flags(microstrain_serializer* serializer, const mip_gnss_gps_time_data_valid_flags self)
{
    microstrain_insert_u16(serializer, (uint16_t)(self));
}
static inline void extract_mip_gnss_gps_time_data_valid_flags(microstrain_serializer* serializer, mip_gnss_gps_time_data_valid_flags* self)
{
    uint16_t tmp = 0;
    microstrain_extract_u16(serializer, &tmp);
    *self = (mip_gnss_gps_time_data_valid_flags)tmp;
}


struct mip_gnss_gps_time_data
{
    double tow; ///< GPS Time of week [seconds]
    uint16_t week_number; ///< GPS Week since 1980 [weeks]
    mip_gnss_gps_time_data_valid_flags valid_flags;
};
typedef struct mip_gnss_gps_time_data mip_gnss_gps_time_data;

void insert_mip_gnss_gps_time_data(microstrain_serializer* serializer, const mip_gnss_gps_time_data* self);
void extract_mip_gnss_gps_time_data(microstrain_serializer* serializer, mip_gnss_gps_time_data* self);
bool extract_mip_gnss_gps_time_data_from_field(const mip_field_view* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup gnss_clock_info_c  (0x81,0x0A) Clock Info
/// GNSS reported receiver clock parameters
///
///@{

typedef uint16_t mip_gnss_clock_info_data_valid_flags;
static const mip_gnss_clock_info_data_valid_flags MIP_GNSS_CLOCK_INFO_DATA_VALID_FLAGS_NONE              = 0x0000;
static const mip_gnss_clock_info_data_valid_flags MIP_GNSS_CLOCK_INFO_DATA_VALID_FLAGS_BIAS              = 0x0001; ///<  
static const mip_gnss_clock_info_data_valid_flags MIP_GNSS_CLOCK_INFO_DATA_VALID_FLAGS_DRIFT             = 0x0002; ///<  
static const mip_gnss_clock_info_data_valid_flags MIP_GNSS_CLOCK_INFO_DATA_VALID_FLAGS_ACCURACY_ESTIMATE = 0x0004; ///<  
static const mip_gnss_clock_info_data_valid_flags MIP_GNSS_CLOCK_INFO_DATA_VALID_FLAGS_FLAGS             = 0x0007; ///<  
static const mip_gnss_clock_info_data_valid_flags MIP_GNSS_CLOCK_INFO_DATA_VALID_FLAGS_ALL               = 0x0007;
static inline void insert_mip_gnss_clock_info_data_valid_flags(microstrain_serializer* serializer, const mip_gnss_clock_info_data_valid_flags self)
{
    microstrain_insert_u16(serializer, (uint16_t)(self));
}
static inline void extract_mip_gnss_clock_info_data_valid_flags(microstrain_serializer* serializer, mip_gnss_clock_info_data_valid_flags* self)
{
    uint16_t tmp = 0;
    microstrain_extract_u16(serializer, &tmp);
    *self = (mip_gnss_clock_info_data_valid_flags)tmp;
}


struct mip_gnss_clock_info_data
{
    double bias; ///< [seconds]
    double drift; ///< [seconds/second]
    double accuracy_estimate; ///< [seconds]
    mip_gnss_clock_info_data_valid_flags valid_flags;
};
typedef struct mip_gnss_clock_info_data mip_gnss_clock_info_data;

void insert_mip_gnss_clock_info_data(microstrain_serializer* serializer, const mip_gnss_clock_info_data* self);
void extract_mip_gnss_clock_info_data(microstrain_serializer* serializer, mip_gnss_clock_info_data* self);
bool extract_mip_gnss_clock_info_data_from_field(const mip_field_view* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup gnss_fix_info_c  (0x81,0x0B) Fix Info
/// GNSS reported position fix type
///
///@{

enum mip_gnss_fix_info_data_fix_type
{
    MIP_GNSS_FIX_INFO_DATA_FIX_TYPE_FIX_3D           = 0,  ///<  
    MIP_GNSS_FIX_INFO_DATA_FIX_TYPE_FIX_2D           = 1,  ///<  
    MIP_GNSS_FIX_INFO_DATA_FIX_TYPE_FIX_TIME_ONLY    = 2,  ///<  
    MIP_GNSS_FIX_INFO_DATA_FIX_TYPE_FIX_NONE         = 3,  ///<  
    MIP_GNSS_FIX_INFO_DATA_FIX_TYPE_FIX_INVALID      = 4,  ///<  
    MIP_GNSS_FIX_INFO_DATA_FIX_TYPE_FIX_RTK_FLOAT    = 5,  ///<  
    MIP_GNSS_FIX_INFO_DATA_FIX_TYPE_FIX_RTK_FIXED    = 6,  ///<  
    MIP_GNSS_FIX_INFO_DATA_FIX_TYPE_FIX_DIFFERENTIAL = 7,  ///<  
};
typedef enum mip_gnss_fix_info_data_fix_type mip_gnss_fix_info_data_fix_type;

static inline void insert_mip_gnss_fix_info_data_fix_type(microstrain_serializer* serializer, const mip_gnss_fix_info_data_fix_type self)
{
    microstrain_insert_u8(serializer, (uint8_t)(self));
}
static inline void extract_mip_gnss_fix_info_data_fix_type(microstrain_serializer* serializer, mip_gnss_fix_info_data_fix_type* self)
{
    uint8_t tmp = 0;
    microstrain_extract_u8(serializer, &tmp);
    *self = (mip_gnss_fix_info_data_fix_type)tmp;
}

typedef uint16_t mip_gnss_fix_info_data_fix_flags;
static const mip_gnss_fix_info_data_fix_flags MIP_GNSS_FIX_INFO_DATA_FIX_FLAGS_NONE       = 0x0000;
static const mip_gnss_fix_info_data_fix_flags MIP_GNSS_FIX_INFO_DATA_FIX_FLAGS_SBAS_USED  = 0x0001; ///<  
static const mip_gnss_fix_info_data_fix_flags MIP_GNSS_FIX_INFO_DATA_FIX_FLAGS_DGNSS_USED = 0x0002; ///<  
static const mip_gnss_fix_info_data_fix_flags MIP_GNSS_FIX_INFO_DATA_FIX_FLAGS_ALL        = 0x0003;
static inline void insert_mip_gnss_fix_info_data_fix_flags(microstrain_serializer* serializer, const mip_gnss_fix_info_data_fix_flags self)
{
    microstrain_insert_u16(serializer, (uint16_t)(self));
}
static inline void extract_mip_gnss_fix_info_data_fix_flags(microstrain_serializer* serializer, mip_gnss_fix_info_data_fix_flags* self)
{
    uint16_t tmp = 0;
    microstrain_extract_u16(serializer, &tmp);
    *self = (mip_gnss_fix_info_data_fix_flags)tmp;
}

typedef uint16_t mip_gnss_fix_info_data_valid_flags;
static const mip_gnss_fix_info_data_valid_flags MIP_GNSS_FIX_INFO_DATA_VALID_FLAGS_NONE      = 0x0000;
static const mip_gnss_fix_info_data_valid_flags MIP_GNSS_FIX_INFO_DATA_VALID_FLAGS_FIX_TYPE  = 0x0001; ///<  
static const mip_gnss_fix_info_data_valid_flags MIP_GNSS_FIX_INFO_DATA_VALID_FLAGS_NUM_SV    = 0x0002; ///<  
static const mip_gnss_fix_info_data_valid_flags MIP_GNSS_FIX_INFO_DATA_VALID_FLAGS_FIX_FLAGS = 0x0004; ///<  
static const mip_gnss_fix_info_data_valid_flags MIP_GNSS_FIX_INFO_DATA_VALID_FLAGS_FLAGS     = 0x0007; ///<  
static const mip_gnss_fix_info_data_valid_flags MIP_GNSS_FIX_INFO_DATA_VALID_FLAGS_ALL       = 0x0007;
static inline void insert_mip_gnss_fix_info_data_valid_flags(microstrain_serializer* serializer, const mip_gnss_fix_info_data_valid_flags self)
{
    microstrain_insert_u16(serializer, (uint16_t)(self));
}
static inline void extract_mip_gnss_fix_info_data_valid_flags(microstrain_serializer* serializer, mip_gnss_fix_info_data_valid_flags* self)
{
    uint16_t tmp = 0;
    microstrain_extract_u16(serializer, &tmp);
    *self = (mip_gnss_fix_info_data_valid_flags)tmp;
}


struct mip_gnss_fix_info_data
{
    mip_gnss_fix_info_data_fix_type fix_type;
    uint8_t num_sv;
    mip_gnss_fix_info_data_fix_flags fix_flags;
    mip_gnss_fix_info_data_valid_flags valid_flags;
};
typedef struct mip_gnss_fix_info_data mip_gnss_fix_info_data;

void insert_mip_gnss_fix_info_data(microstrain_serializer* serializer, const mip_gnss_fix_info_data* self);
void extract_mip_gnss_fix_info_data(microstrain_serializer* serializer, mip_gnss_fix_info_data* self);
bool extract_mip_gnss_fix_info_data_from_field(const mip_field_view* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup gnss_sv_info_c  (0x81,0x0C) Sv Info
/// GNSS reported space vehicle information
/// 
/// When enabled, these fields will arrive in separate MIP packets
///
///@{

typedef uint16_t mip_gnss_sv_info_data_svflags;
static const mip_gnss_sv_info_data_svflags MIP_GNSS_SV_INFO_DATA_SVFLAGS_NONE                = 0x0000;
static const mip_gnss_sv_info_data_svflags MIP_GNSS_SV_INFO_DATA_SVFLAGS_USED_FOR_NAVIGATION = 0x0001; ///<  
static const mip_gnss_sv_info_data_svflags MIP_GNSS_SV_INFO_DATA_SVFLAGS_HEALTHY             = 0x0002; ///<  
static const mip_gnss_sv_info_data_svflags MIP_GNSS_SV_INFO_DATA_SVFLAGS_ALL                 = 0x0003;
static inline void insert_mip_gnss_sv_info_data_svflags(microstrain_serializer* serializer, const mip_gnss_sv_info_data_svflags self)
{
    microstrain_insert_u16(serializer, (uint16_t)(self));
}
static inline void extract_mip_gnss_sv_info_data_svflags(microstrain_serializer* serializer, mip_gnss_sv_info_data_svflags* self)
{
    uint16_t tmp = 0;
    microstrain_extract_u16(serializer, &tmp);
    *self = (mip_gnss_sv_info_data_svflags)tmp;
}

typedef uint16_t mip_gnss_sv_info_data_valid_flags;
static const mip_gnss_sv_info_data_valid_flags MIP_GNSS_SV_INFO_DATA_VALID_FLAGS_NONE                = 0x0000;
static const mip_gnss_sv_info_data_valid_flags MIP_GNSS_SV_INFO_DATA_VALID_FLAGS_CHANNEL             = 0x0001; ///<  
static const mip_gnss_sv_info_data_valid_flags MIP_GNSS_SV_INFO_DATA_VALID_FLAGS_SV_ID               = 0x0002; ///<  
static const mip_gnss_sv_info_data_valid_flags MIP_GNSS_SV_INFO_DATA_VALID_FLAGS_CARRIER_NOISE_RATIO = 0x0004; ///<  
static const mip_gnss_sv_info_data_valid_flags MIP_GNSS_SV_INFO_DATA_VALID_FLAGS_AZIMUTH             = 0x0008; ///<  
static const mip_gnss_sv_info_data_valid_flags MIP_GNSS_SV_INFO_DATA_VALID_FLAGS_ELEVATION           = 0x0010; ///<  
static const mip_gnss_sv_info_data_valid_flags MIP_GNSS_SV_INFO_DATA_VALID_FLAGS_SV_FLAGS            = 0x0020; ///<  
static const mip_gnss_sv_info_data_valid_flags MIP_GNSS_SV_INFO_DATA_VALID_FLAGS_FLAGS               = 0x003F; ///<  
static const mip_gnss_sv_info_data_valid_flags MIP_GNSS_SV_INFO_DATA_VALID_FLAGS_ALL                 = 0x003F;
static inline void insert_mip_gnss_sv_info_data_valid_flags(microstrain_serializer* serializer, const mip_gnss_sv_info_data_valid_flags self)
{
    microstrain_insert_u16(serializer, (uint16_t)(self));
}
static inline void extract_mip_gnss_sv_info_data_valid_flags(microstrain_serializer* serializer, mip_gnss_sv_info_data_valid_flags* self)
{
    uint16_t tmp = 0;
    microstrain_extract_u16(serializer, &tmp);
    *self = (mip_gnss_sv_info_data_valid_flags)tmp;
}


struct mip_gnss_sv_info_data
{
    uint8_t channel; ///< Receiver channel number
    uint8_t sv_id; ///< GNSS Satellite ID
    uint16_t carrier_noise_ratio; ///< [dBHz]
    int16_t azimuth; ///< [deg]
    int16_t elevation; ///< [deg]
    mip_gnss_sv_info_data_svflags sv_flags;
    mip_gnss_sv_info_data_valid_flags valid_flags;
};
typedef struct mip_gnss_sv_info_data mip_gnss_sv_info_data;

void insert_mip_gnss_sv_info_data(microstrain_serializer* serializer, const mip_gnss_sv_info_data* self);
void extract_mip_gnss_sv_info_data(microstrain_serializer* serializer, mip_gnss_sv_info_data* self);
bool extract_mip_gnss_sv_info_data_from_field(const mip_field_view* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup gnss_hw_status_c  (0x81,0x0D) Hw Status
/// GNSS reported hardware status
///
///@{

enum mip_gnss_hw_status_data_receiver_state
{
    MIP_GNSS_HW_STATUS_DATA_RECEIVER_STATE_OFF     = 0,  ///<  
    MIP_GNSS_HW_STATUS_DATA_RECEIVER_STATE_ON      = 1,  ///<  
    MIP_GNSS_HW_STATUS_DATA_RECEIVER_STATE_UNKNOWN = 2,  ///<  
};
typedef enum mip_gnss_hw_status_data_receiver_state mip_gnss_hw_status_data_receiver_state;

static inline void insert_mip_gnss_hw_status_data_receiver_state(microstrain_serializer* serializer, const mip_gnss_hw_status_data_receiver_state self)
{
    microstrain_insert_u8(serializer, (uint8_t)(self));
}
static inline void extract_mip_gnss_hw_status_data_receiver_state(microstrain_serializer* serializer, mip_gnss_hw_status_data_receiver_state* self)
{
    uint8_t tmp = 0;
    microstrain_extract_u8(serializer, &tmp);
    *self = (mip_gnss_hw_status_data_receiver_state)tmp;
}

enum mip_gnss_hw_status_data_antenna_state
{
    MIP_GNSS_HW_STATUS_DATA_ANTENNA_STATE_INIT    = 1,  ///<  
    MIP_GNSS_HW_STATUS_DATA_ANTENNA_STATE_SHORT   = 2,  ///<  
    MIP_GNSS_HW_STATUS_DATA_ANTENNA_STATE_OPEN    = 3,  ///<  
    MIP_GNSS_HW_STATUS_DATA_ANTENNA_STATE_GOOD    = 4,  ///<  
    MIP_GNSS_HW_STATUS_DATA_ANTENNA_STATE_UNKNOWN = 5,  ///<  
};
typedef enum mip_gnss_hw_status_data_antenna_state mip_gnss_hw_status_data_antenna_state;

static inline void insert_mip_gnss_hw_status_data_antenna_state(microstrain_serializer* serializer, const mip_gnss_hw_status_data_antenna_state self)
{
    microstrain_insert_u8(serializer, (uint8_t)(self));
}
static inline void extract_mip_gnss_hw_status_data_antenna_state(microstrain_serializer* serializer, mip_gnss_hw_status_data_antenna_state* self)
{
    uint8_t tmp = 0;
    microstrain_extract_u8(serializer, &tmp);
    *self = (mip_gnss_hw_status_data_antenna_state)tmp;
}

enum mip_gnss_hw_status_data_antenna_power
{
    MIP_GNSS_HW_STATUS_DATA_ANTENNA_POWER_OFF     = 0,  ///<  
    MIP_GNSS_HW_STATUS_DATA_ANTENNA_POWER_ON      = 1,  ///<  
    MIP_GNSS_HW_STATUS_DATA_ANTENNA_POWER_UNKNOWN = 2,  ///<  
};
typedef enum mip_gnss_hw_status_data_antenna_power mip_gnss_hw_status_data_antenna_power;

static inline void insert_mip_gnss_hw_status_data_antenna_power(microstrain_serializer* serializer, const mip_gnss_hw_status_data_antenna_power self)
{
    microstrain_insert_u8(serializer, (uint8_t)(self));
}
static inline void extract_mip_gnss_hw_status_data_antenna_power(microstrain_serializer* serializer, mip_gnss_hw_status_data_antenna_power* self)
{
    uint8_t tmp = 0;
    microstrain_extract_u8(serializer, &tmp);
    *self = (mip_gnss_hw_status_data_antenna_power)tmp;
}

typedef uint16_t mip_gnss_hw_status_data_valid_flags;
static const mip_gnss_hw_status_data_valid_flags MIP_GNSS_HW_STATUS_DATA_VALID_FLAGS_NONE          = 0x0000;
static const mip_gnss_hw_status_data_valid_flags MIP_GNSS_HW_STATUS_DATA_VALID_FLAGS_SENSOR_STATE  = 0x0001; ///<  
static const mip_gnss_hw_status_data_valid_flags MIP_GNSS_HW_STATUS_DATA_VALID_FLAGS_ANTENNA_STATE = 0x0002; ///<  
static const mip_gnss_hw_status_data_valid_flags MIP_GNSS_HW_STATUS_DATA_VALID_FLAGS_ANTENNA_POWER = 0x0004; ///<  
static const mip_gnss_hw_status_data_valid_flags MIP_GNSS_HW_STATUS_DATA_VALID_FLAGS_FLAGS         = 0x0007; ///<  
static const mip_gnss_hw_status_data_valid_flags MIP_GNSS_HW_STATUS_DATA_VALID_FLAGS_ALL           = 0x0007;
static inline void insert_mip_gnss_hw_status_data_valid_flags(microstrain_serializer* serializer, const mip_gnss_hw_status_data_valid_flags self)
{
    microstrain_insert_u16(serializer, (uint16_t)(self));
}
static inline void extract_mip_gnss_hw_status_data_valid_flags(microstrain_serializer* serializer, mip_gnss_hw_status_data_valid_flags* self)
{
    uint16_t tmp = 0;
    microstrain_extract_u16(serializer, &tmp);
    *self = (mip_gnss_hw_status_data_valid_flags)tmp;
}


struct mip_gnss_hw_status_data
{
    mip_gnss_hw_status_data_receiver_state receiver_state;
    mip_gnss_hw_status_data_antenna_state antenna_state;
    mip_gnss_hw_status_data_antenna_power antenna_power;
    mip_gnss_hw_status_data_valid_flags valid_flags;
};
typedef struct mip_gnss_hw_status_data mip_gnss_hw_status_data;

void insert_mip_gnss_hw_status_data(microstrain_serializer* serializer, const mip_gnss_hw_status_data* self);
void extract_mip_gnss_hw_status_data(microstrain_serializer* serializer, mip_gnss_hw_status_data* self);
bool extract_mip_gnss_hw_status_data_from_field(const mip_field_view* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup gnss_dgps_info_c  (0x81,0x0E) Dgps Info
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

typedef uint16_t mip_gnss_dgps_info_data_valid_flags;
static const mip_gnss_dgps_info_data_valid_flags MIP_GNSS_DGPS_INFO_DATA_VALID_FLAGS_NONE                = 0x0000;
static const mip_gnss_dgps_info_data_valid_flags MIP_GNSS_DGPS_INFO_DATA_VALID_FLAGS_AGE                 = 0x0001; ///<  
static const mip_gnss_dgps_info_data_valid_flags MIP_GNSS_DGPS_INFO_DATA_VALID_FLAGS_BASE_STATION_ID     = 0x0002; ///<  
static const mip_gnss_dgps_info_data_valid_flags MIP_GNSS_DGPS_INFO_DATA_VALID_FLAGS_BASE_STATION_STATUS = 0x0004; ///<  
static const mip_gnss_dgps_info_data_valid_flags MIP_GNSS_DGPS_INFO_DATA_VALID_FLAGS_NUM_CHANNELS        = 0x0008; ///<  
static const mip_gnss_dgps_info_data_valid_flags MIP_GNSS_DGPS_INFO_DATA_VALID_FLAGS_FLAGS               = 0x000F; ///<  
static const mip_gnss_dgps_info_data_valid_flags MIP_GNSS_DGPS_INFO_DATA_VALID_FLAGS_ALL                 = 0x000F;
static inline void insert_mip_gnss_dgps_info_data_valid_flags(microstrain_serializer* serializer, const mip_gnss_dgps_info_data_valid_flags self)
{
    microstrain_insert_u16(serializer, (uint16_t)(self));
}
static inline void extract_mip_gnss_dgps_info_data_valid_flags(microstrain_serializer* serializer, mip_gnss_dgps_info_data_valid_flags* self)
{
    uint16_t tmp = 0;
    microstrain_extract_u16(serializer, &tmp);
    *self = (mip_gnss_dgps_info_data_valid_flags)tmp;
}


struct mip_gnss_dgps_info_data
{
    uint8_t sv_id;
    float age;
    float range_correction;
    float range_rate_correction;
    mip_gnss_dgps_info_data_valid_flags valid_flags;
};
typedef struct mip_gnss_dgps_info_data mip_gnss_dgps_info_data;

void insert_mip_gnss_dgps_info_data(microstrain_serializer* serializer, const mip_gnss_dgps_info_data* self);
void extract_mip_gnss_dgps_info_data(microstrain_serializer* serializer, mip_gnss_dgps_info_data* self);
bool extract_mip_gnss_dgps_info_data_from_field(const mip_field_view* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup gnss_dgps_channel_c  (0x81,0x0F) Dgps Channel
/// GNSS reported DGPS Channel Status status
/// 
/// When enabled, a separate field for each active space vehicle will be sent in the packet.
///
///@{

typedef uint16_t mip_gnss_dgps_channel_data_valid_flags;
static const mip_gnss_dgps_channel_data_valid_flags MIP_GNSS_DGPS_CHANNEL_DATA_VALID_FLAGS_NONE                  = 0x0000;
static const mip_gnss_dgps_channel_data_valid_flags MIP_GNSS_DGPS_CHANNEL_DATA_VALID_FLAGS_ID                    = 0x0001; ///<  
static const mip_gnss_dgps_channel_data_valid_flags MIP_GNSS_DGPS_CHANNEL_DATA_VALID_FLAGS_AGE                   = 0x0002; ///<  
static const mip_gnss_dgps_channel_data_valid_flags MIP_GNSS_DGPS_CHANNEL_DATA_VALID_FLAGS_RANGE_CORRECTION      = 0x0004; ///<  
static const mip_gnss_dgps_channel_data_valid_flags MIP_GNSS_DGPS_CHANNEL_DATA_VALID_FLAGS_RANGE_RATE_CORRECTION = 0x0008; ///<  
static const mip_gnss_dgps_channel_data_valid_flags MIP_GNSS_DGPS_CHANNEL_DATA_VALID_FLAGS_FLAGS                 = 0x000F; ///<  
static const mip_gnss_dgps_channel_data_valid_flags MIP_GNSS_DGPS_CHANNEL_DATA_VALID_FLAGS_ALL                   = 0x000F;
static inline void insert_mip_gnss_dgps_channel_data_valid_flags(microstrain_serializer* serializer, const mip_gnss_dgps_channel_data_valid_flags self)
{
    microstrain_insert_u16(serializer, (uint16_t)(self));
}
static inline void extract_mip_gnss_dgps_channel_data_valid_flags(microstrain_serializer* serializer, mip_gnss_dgps_channel_data_valid_flags* self)
{
    uint16_t tmp = 0;
    microstrain_extract_u16(serializer, &tmp);
    *self = (mip_gnss_dgps_channel_data_valid_flags)tmp;
}


struct mip_gnss_dgps_channel_data
{
    uint8_t sv_id;
    float age; ///< [s]
    float range_correction; ///< [m]
    float range_rate_correction; ///< [m/s]
    mip_gnss_dgps_channel_data_valid_flags valid_flags;
};
typedef struct mip_gnss_dgps_channel_data mip_gnss_dgps_channel_data;

void insert_mip_gnss_dgps_channel_data(microstrain_serializer* serializer, const mip_gnss_dgps_channel_data* self);
void extract_mip_gnss_dgps_channel_data(microstrain_serializer* serializer, mip_gnss_dgps_channel_data* self);
bool extract_mip_gnss_dgps_channel_data_from_field(const mip_field_view* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup gnss_clock_info_2_c  (0x81,0x10) Clock Info 2
/// GNSS reported receiver clock parameters
/// 
/// This supersedes MIP_DATA_DESC_GNSS_CLOCK_INFO with additional information.
///
///@{

typedef uint16_t mip_gnss_clock_info_2_data_valid_flags;
static const mip_gnss_clock_info_2_data_valid_flags MIP_GNSS_CLOCK_INFO_2_DATA_VALID_FLAGS_NONE           = 0x0000;
static const mip_gnss_clock_info_2_data_valid_flags MIP_GNSS_CLOCK_INFO_2_DATA_VALID_FLAGS_BIAS           = 0x0001; ///<  
static const mip_gnss_clock_info_2_data_valid_flags MIP_GNSS_CLOCK_INFO_2_DATA_VALID_FLAGS_DRIFT          = 0x0002; ///<  
static const mip_gnss_clock_info_2_data_valid_flags MIP_GNSS_CLOCK_INFO_2_DATA_VALID_FLAGS_BIAS_ACCURACY  = 0x0004; ///<  
static const mip_gnss_clock_info_2_data_valid_flags MIP_GNSS_CLOCK_INFO_2_DATA_VALID_FLAGS_DRIFT_ACCURACY = 0x0008; ///<  
static const mip_gnss_clock_info_2_data_valid_flags MIP_GNSS_CLOCK_INFO_2_DATA_VALID_FLAGS_FLAGS          = 0x000F; ///<  
static const mip_gnss_clock_info_2_data_valid_flags MIP_GNSS_CLOCK_INFO_2_DATA_VALID_FLAGS_ALL            = 0x000F;
static inline void insert_mip_gnss_clock_info_2_data_valid_flags(microstrain_serializer* serializer, const mip_gnss_clock_info_2_data_valid_flags self)
{
    microstrain_insert_u16(serializer, (uint16_t)(self));
}
static inline void extract_mip_gnss_clock_info_2_data_valid_flags(microstrain_serializer* serializer, mip_gnss_clock_info_2_data_valid_flags* self)
{
    uint16_t tmp = 0;
    microstrain_extract_u16(serializer, &tmp);
    *self = (mip_gnss_clock_info_2_data_valid_flags)tmp;
}


struct mip_gnss_clock_info_2_data
{
    double bias;
    double drift;
    double bias_accuracy_estimate;
    double drift_accuracy_estimate;
    mip_gnss_clock_info_2_data_valid_flags valid_flags;
};
typedef struct mip_gnss_clock_info_2_data mip_gnss_clock_info_2_data;

void insert_mip_gnss_clock_info_2_data(microstrain_serializer* serializer, const mip_gnss_clock_info_2_data* self);
void extract_mip_gnss_clock_info_2_data(microstrain_serializer* serializer, mip_gnss_clock_info_2_data* self);
bool extract_mip_gnss_clock_info_2_data_from_field(const mip_field_view* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup gnss_gps_leap_seconds_c  (0x81,0x11) Gps Leap Seconds
/// GNSS reported leap seconds (difference between GPS and UTC Time)
///
///@{

typedef uint16_t mip_gnss_gps_leap_seconds_data_valid_flags;
static const mip_gnss_gps_leap_seconds_data_valid_flags MIP_GNSS_GPS_LEAP_SECONDS_DATA_VALID_FLAGS_NONE         = 0x0000;
static const mip_gnss_gps_leap_seconds_data_valid_flags MIP_GNSS_GPS_LEAP_SECONDS_DATA_VALID_FLAGS_LEAP_SECONDS = 0x0002; ///<  
static const mip_gnss_gps_leap_seconds_data_valid_flags MIP_GNSS_GPS_LEAP_SECONDS_DATA_VALID_FLAGS_ALL          = 0x0002;
static inline void insert_mip_gnss_gps_leap_seconds_data_valid_flags(microstrain_serializer* serializer, const mip_gnss_gps_leap_seconds_data_valid_flags self)
{
    microstrain_insert_u16(serializer, (uint16_t)(self));
}
static inline void extract_mip_gnss_gps_leap_seconds_data_valid_flags(microstrain_serializer* serializer, mip_gnss_gps_leap_seconds_data_valid_flags* self)
{
    uint16_t tmp = 0;
    microstrain_extract_u16(serializer, &tmp);
    *self = (mip_gnss_gps_leap_seconds_data_valid_flags)tmp;
}


struct mip_gnss_gps_leap_seconds_data
{
    uint8_t leap_seconds; ///< [s]
    mip_gnss_gps_leap_seconds_data_valid_flags valid_flags;
};
typedef struct mip_gnss_gps_leap_seconds_data mip_gnss_gps_leap_seconds_data;

void insert_mip_gnss_gps_leap_seconds_data(microstrain_serializer* serializer, const mip_gnss_gps_leap_seconds_data* self);
void extract_mip_gnss_gps_leap_seconds_data(microstrain_serializer* serializer, mip_gnss_gps_leap_seconds_data* self);
bool extract_mip_gnss_gps_leap_seconds_data_from_field(const mip_field_view* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup gnss_sbas_info_c  (0x81,0x12) Sbas Info
/// GNSS SBAS status
///
///@{

typedef uint8_t mip_gnss_sbas_info_data_sbas_status;
static const mip_gnss_sbas_info_data_sbas_status MIP_GNSS_SBAS_INFO_DATA_SBAS_STATUS_NONE                  = 0x00;
static const mip_gnss_sbas_info_data_sbas_status MIP_GNSS_SBAS_INFO_DATA_SBAS_STATUS_RANGE_AVAILABLE       = 0x01; ///<  
static const mip_gnss_sbas_info_data_sbas_status MIP_GNSS_SBAS_INFO_DATA_SBAS_STATUS_CORRECTIONS_AVAILABLE = 0x02; ///<  
static const mip_gnss_sbas_info_data_sbas_status MIP_GNSS_SBAS_INFO_DATA_SBAS_STATUS_INTEGRITY_AVAILABLE   = 0x04; ///<  
static const mip_gnss_sbas_info_data_sbas_status MIP_GNSS_SBAS_INFO_DATA_SBAS_STATUS_TEST_MODE             = 0x08; ///<  
static const mip_gnss_sbas_info_data_sbas_status MIP_GNSS_SBAS_INFO_DATA_SBAS_STATUS_ALL                   = 0x0F;
static inline void insert_mip_gnss_sbas_info_data_sbas_status(microstrain_serializer* serializer, const mip_gnss_sbas_info_data_sbas_status self)
{
    microstrain_insert_u8(serializer, (uint8_t)(self));
}
static inline void extract_mip_gnss_sbas_info_data_sbas_status(microstrain_serializer* serializer, mip_gnss_sbas_info_data_sbas_status* self)
{
    uint8_t tmp = 0;
    microstrain_extract_u8(serializer, &tmp);
    *self = (mip_gnss_sbas_info_data_sbas_status)tmp;
}

typedef uint16_t mip_gnss_sbas_info_data_valid_flags;
static const mip_gnss_sbas_info_data_valid_flags MIP_GNSS_SBAS_INFO_DATA_VALID_FLAGS_NONE        = 0x0000;
static const mip_gnss_sbas_info_data_valid_flags MIP_GNSS_SBAS_INFO_DATA_VALID_FLAGS_TOW         = 0x0001; ///<  
static const mip_gnss_sbas_info_data_valid_flags MIP_GNSS_SBAS_INFO_DATA_VALID_FLAGS_WEEK_NUMBER = 0x0002; ///<  
static const mip_gnss_sbas_info_data_valid_flags MIP_GNSS_SBAS_INFO_DATA_VALID_FLAGS_SBAS_SYSTEM = 0x0004; ///<  
static const mip_gnss_sbas_info_data_valid_flags MIP_GNSS_SBAS_INFO_DATA_VALID_FLAGS_SBAS_ID     = 0x0008; ///<  
static const mip_gnss_sbas_info_data_valid_flags MIP_GNSS_SBAS_INFO_DATA_VALID_FLAGS_COUNT       = 0x0010; ///<  
static const mip_gnss_sbas_info_data_valid_flags MIP_GNSS_SBAS_INFO_DATA_VALID_FLAGS_SBAS_STATUS = 0x0020; ///<  
static const mip_gnss_sbas_info_data_valid_flags MIP_GNSS_SBAS_INFO_DATA_VALID_FLAGS_FLAGS       = 0x003F; ///<  
static const mip_gnss_sbas_info_data_valid_flags MIP_GNSS_SBAS_INFO_DATA_VALID_FLAGS_ALL         = 0x003F;
static inline void insert_mip_gnss_sbas_info_data_valid_flags(microstrain_serializer* serializer, const mip_gnss_sbas_info_data_valid_flags self)
{
    microstrain_insert_u16(serializer, (uint16_t)(self));
}
static inline void extract_mip_gnss_sbas_info_data_valid_flags(microstrain_serializer* serializer, mip_gnss_sbas_info_data_valid_flags* self)
{
    uint16_t tmp = 0;
    microstrain_extract_u16(serializer, &tmp);
    *self = (mip_gnss_sbas_info_data_valid_flags)tmp;
}


struct mip_gnss_sbas_info_data
{
    double time_of_week; ///< GPS Time of week [seconds]
    uint16_t week_number; ///< GPS Week since 1980 [weeks]
    mip_sbas_system sbas_system; ///< SBAS system id
    uint8_t sbas_id; ///< SBAS satellite id.
    uint8_t count; ///< Number of SBAS corrections
    mip_gnss_sbas_info_data_sbas_status sbas_status; ///< Status of the SBAS service
    mip_gnss_sbas_info_data_valid_flags valid_flags;
};
typedef struct mip_gnss_sbas_info_data mip_gnss_sbas_info_data;

void insert_mip_gnss_sbas_info_data(microstrain_serializer* serializer, const mip_gnss_sbas_info_data* self);
void extract_mip_gnss_sbas_info_data(microstrain_serializer* serializer, mip_gnss_sbas_info_data* self);
bool extract_mip_gnss_sbas_info_data_from_field(const mip_field_view* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup gnss_sbas_correction_c  (0x81,0x13) Sbas Correction
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

typedef uint16_t mip_gnss_sbas_correction_data_valid_flags;
static const mip_gnss_sbas_correction_data_valid_flags MIP_GNSS_SBAS_CORRECTION_DATA_VALID_FLAGS_NONE                   = 0x0000;
static const mip_gnss_sbas_correction_data_valid_flags MIP_GNSS_SBAS_CORRECTION_DATA_VALID_FLAGS_UDREI                  = 0x0001; ///<  
static const mip_gnss_sbas_correction_data_valid_flags MIP_GNSS_SBAS_CORRECTION_DATA_VALID_FLAGS_PSEUDORANGE_CORRECTION = 0x0002; ///<  
static const mip_gnss_sbas_correction_data_valid_flags MIP_GNSS_SBAS_CORRECTION_DATA_VALID_FLAGS_IONO_CORRECTION        = 0x0004; ///<  
static const mip_gnss_sbas_correction_data_valid_flags MIP_GNSS_SBAS_CORRECTION_DATA_VALID_FLAGS_FLAGS                  = 0x0007; ///<  
static const mip_gnss_sbas_correction_data_valid_flags MIP_GNSS_SBAS_CORRECTION_DATA_VALID_FLAGS_ALL                    = 0x0007;
static inline void insert_mip_gnss_sbas_correction_data_valid_flags(microstrain_serializer* serializer, const mip_gnss_sbas_correction_data_valid_flags self)
{
    microstrain_insert_u16(serializer, (uint16_t)(self));
}
static inline void extract_mip_gnss_sbas_correction_data_valid_flags(microstrain_serializer* serializer, mip_gnss_sbas_correction_data_valid_flags* self)
{
    uint16_t tmp = 0;
    microstrain_extract_u16(serializer, &tmp);
    *self = (mip_gnss_sbas_correction_data_valid_flags)tmp;
}


struct mip_gnss_sbas_correction_data
{
    uint8_t index; ///< Index of this field in this epoch.
    uint8_t count; ///< Total number of fields in this epoch.
    double time_of_week; ///< GPS Time of week the message was received [seconds]
    uint16_t week_number; ///< GPS Week since 1980 [weeks]
    mip_gnss_constellation_id gnss_id; ///< GNSS constellation id
    uint8_t sv_id; ///< GNSS satellite id within the constellation.
    uint8_t udrei; ///< [See above 0-13 usable, 14 not monitored, 15 - do not use]
    float pseudorange_correction; ///< Pseudo-range correction [meters].
    float iono_correction; ///< Ionospheric correction [meters].
    mip_gnss_sbas_correction_data_valid_flags valid_flags;
};
typedef struct mip_gnss_sbas_correction_data mip_gnss_sbas_correction_data;

void insert_mip_gnss_sbas_correction_data(microstrain_serializer* serializer, const mip_gnss_sbas_correction_data* self);
void extract_mip_gnss_sbas_correction_data(microstrain_serializer* serializer, mip_gnss_sbas_correction_data* self);
bool extract_mip_gnss_sbas_correction_data_from_field(const mip_field_view* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup gnss_rf_error_detection_c  (0x81,0x14) Rf Error Detection
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
typedef enum mip_gnss_rf_error_detection_data_rfband mip_gnss_rf_error_detection_data_rfband;

static inline void insert_mip_gnss_rf_error_detection_data_rfband(microstrain_serializer* serializer, const mip_gnss_rf_error_detection_data_rfband self)
{
    microstrain_insert_u8(serializer, (uint8_t)(self));
}
static inline void extract_mip_gnss_rf_error_detection_data_rfband(microstrain_serializer* serializer, mip_gnss_rf_error_detection_data_rfband* self)
{
    uint8_t tmp = 0;
    microstrain_extract_u8(serializer, &tmp);
    *self = (mip_gnss_rf_error_detection_data_rfband)tmp;
}

enum mip_gnss_rf_error_detection_data_jamming_state
{
    MIP_GNSS_RF_ERROR_DETECTION_DATA_JAMMING_STATE_UNKNOWN     = 0,  ///<  
    MIP_GNSS_RF_ERROR_DETECTION_DATA_JAMMING_STATE_NONE        = 1,  ///<  
    MIP_GNSS_RF_ERROR_DETECTION_DATA_JAMMING_STATE_PARTIAL     = 2,  ///<  
    MIP_GNSS_RF_ERROR_DETECTION_DATA_JAMMING_STATE_SIGNIFICANT = 3,  ///<  
};
typedef enum mip_gnss_rf_error_detection_data_jamming_state mip_gnss_rf_error_detection_data_jamming_state;

static inline void insert_mip_gnss_rf_error_detection_data_jamming_state(microstrain_serializer* serializer, const mip_gnss_rf_error_detection_data_jamming_state self)
{
    microstrain_insert_u8(serializer, (uint8_t)(self));
}
static inline void extract_mip_gnss_rf_error_detection_data_jamming_state(microstrain_serializer* serializer, mip_gnss_rf_error_detection_data_jamming_state* self)
{
    uint8_t tmp = 0;
    microstrain_extract_u8(serializer, &tmp);
    *self = (mip_gnss_rf_error_detection_data_jamming_state)tmp;
}

enum mip_gnss_rf_error_detection_data_spoofing_state
{
    MIP_GNSS_RF_ERROR_DETECTION_DATA_SPOOFING_STATE_UNKNOWN     = 0,  ///<  
    MIP_GNSS_RF_ERROR_DETECTION_DATA_SPOOFING_STATE_NONE        = 1,  ///<  
    MIP_GNSS_RF_ERROR_DETECTION_DATA_SPOOFING_STATE_PARTIAL     = 2,  ///<  
    MIP_GNSS_RF_ERROR_DETECTION_DATA_SPOOFING_STATE_SIGNIFICANT = 3,  ///<  
};
typedef enum mip_gnss_rf_error_detection_data_spoofing_state mip_gnss_rf_error_detection_data_spoofing_state;

static inline void insert_mip_gnss_rf_error_detection_data_spoofing_state(microstrain_serializer* serializer, const mip_gnss_rf_error_detection_data_spoofing_state self)
{
    microstrain_insert_u8(serializer, (uint8_t)(self));
}
static inline void extract_mip_gnss_rf_error_detection_data_spoofing_state(microstrain_serializer* serializer, mip_gnss_rf_error_detection_data_spoofing_state* self)
{
    uint8_t tmp = 0;
    microstrain_extract_u8(serializer, &tmp);
    *self = (mip_gnss_rf_error_detection_data_spoofing_state)tmp;
}

typedef uint16_t mip_gnss_rf_error_detection_data_valid_flags;
static const mip_gnss_rf_error_detection_data_valid_flags MIP_GNSS_RF_ERROR_DETECTION_DATA_VALID_FLAGS_NONE           = 0x0000;
static const mip_gnss_rf_error_detection_data_valid_flags MIP_GNSS_RF_ERROR_DETECTION_DATA_VALID_FLAGS_RF_BAND        = 0x0001; ///<  
static const mip_gnss_rf_error_detection_data_valid_flags MIP_GNSS_RF_ERROR_DETECTION_DATA_VALID_FLAGS_JAMMING_STATE  = 0x0002; ///<  
static const mip_gnss_rf_error_detection_data_valid_flags MIP_GNSS_RF_ERROR_DETECTION_DATA_VALID_FLAGS_SPOOFING_STATE = 0x0004; ///<  
static const mip_gnss_rf_error_detection_data_valid_flags MIP_GNSS_RF_ERROR_DETECTION_DATA_VALID_FLAGS_FLAGS          = 0x0007; ///<  
static const mip_gnss_rf_error_detection_data_valid_flags MIP_GNSS_RF_ERROR_DETECTION_DATA_VALID_FLAGS_ALL            = 0x0007;
static inline void insert_mip_gnss_rf_error_detection_data_valid_flags(microstrain_serializer* serializer, const mip_gnss_rf_error_detection_data_valid_flags self)
{
    microstrain_insert_u16(serializer, (uint16_t)(self));
}
static inline void extract_mip_gnss_rf_error_detection_data_valid_flags(microstrain_serializer* serializer, mip_gnss_rf_error_detection_data_valid_flags* self)
{
    uint16_t tmp = 0;
    microstrain_extract_u16(serializer, &tmp);
    *self = (mip_gnss_rf_error_detection_data_valid_flags)tmp;
}


struct mip_gnss_rf_error_detection_data
{
    mip_gnss_rf_error_detection_data_rfband rf_band; ///< RF Band of the reported information
    mip_gnss_rf_error_detection_data_jamming_state jamming_state; ///< GNSS Jamming State (as reported by the GNSS module)
    mip_gnss_rf_error_detection_data_spoofing_state spoofing_state; ///< GNSS Spoofing State (as reported by the GNSS module)
    uint8_t reserved[4]; ///< Reserved for future use
    mip_gnss_rf_error_detection_data_valid_flags valid_flags;
};
typedef struct mip_gnss_rf_error_detection_data mip_gnss_rf_error_detection_data;

void insert_mip_gnss_rf_error_detection_data(microstrain_serializer* serializer, const mip_gnss_rf_error_detection_data* self);
void extract_mip_gnss_rf_error_detection_data(microstrain_serializer* serializer, mip_gnss_rf_error_detection_data* self);
bool extract_mip_gnss_rf_error_detection_data_from_field(const mip_field_view* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup gnss_base_station_info_c  (0x81,0x30) Base Station Info
/// RTCM reported base station information (sourced from RTCM Message 1005 or 1006)
/// 
/// Valid Flag Mapping:
///
///@{

typedef uint16_t mip_gnss_base_station_info_data_indicator_flags;
static const mip_gnss_base_station_info_data_indicator_flags MIP_GNSS_BASE_STATION_INFO_DATA_INDICATOR_FLAGS_NONE               = 0x0000;
static const mip_gnss_base_station_info_data_indicator_flags MIP_GNSS_BASE_STATION_INFO_DATA_INDICATOR_FLAGS_GPS                = 0x0001; ///<  
static const mip_gnss_base_station_info_data_indicator_flags MIP_GNSS_BASE_STATION_INFO_DATA_INDICATOR_FLAGS_GLONASS            = 0x0002; ///<  
static const mip_gnss_base_station_info_data_indicator_flags MIP_GNSS_BASE_STATION_INFO_DATA_INDICATOR_FLAGS_GALILEO            = 0x0004; ///<  
static const mip_gnss_base_station_info_data_indicator_flags MIP_GNSS_BASE_STATION_INFO_DATA_INDICATOR_FLAGS_BEIDOU             = 0x0008; ///<  
static const mip_gnss_base_station_info_data_indicator_flags MIP_GNSS_BASE_STATION_INFO_DATA_INDICATOR_FLAGS_REF_STATION        = 0x0010; ///<  
static const mip_gnss_base_station_info_data_indicator_flags MIP_GNSS_BASE_STATION_INFO_DATA_INDICATOR_FLAGS_SINGLE_RECEIVER    = 0x0020; ///<  
static const mip_gnss_base_station_info_data_indicator_flags MIP_GNSS_BASE_STATION_INFO_DATA_INDICATOR_FLAGS_QUARTER_CYCLE_BIT1 = 0x0040; ///<  
static const mip_gnss_base_station_info_data_indicator_flags MIP_GNSS_BASE_STATION_INFO_DATA_INDICATOR_FLAGS_QUARTER_CYCLE_BIT2 = 0x0080; ///<  
static const mip_gnss_base_station_info_data_indicator_flags MIP_GNSS_BASE_STATION_INFO_DATA_INDICATOR_FLAGS_QUARTER_CYCLE_BITS = 0x00C0; ///<  
static const mip_gnss_base_station_info_data_indicator_flags MIP_GNSS_BASE_STATION_INFO_DATA_INDICATOR_FLAGS_ALL                = 0x00FF;
static inline void insert_mip_gnss_base_station_info_data_indicator_flags(microstrain_serializer* serializer, const mip_gnss_base_station_info_data_indicator_flags self)
{
    microstrain_insert_u16(serializer, (uint16_t)(self));
}
static inline void extract_mip_gnss_base_station_info_data_indicator_flags(microstrain_serializer* serializer, mip_gnss_base_station_info_data_indicator_flags* self)
{
    uint16_t tmp = 0;
    microstrain_extract_u16(serializer, &tmp);
    *self = (mip_gnss_base_station_info_data_indicator_flags)tmp;
}

typedef uint16_t mip_gnss_base_station_info_data_valid_flags;
static const mip_gnss_base_station_info_data_valid_flags MIP_GNSS_BASE_STATION_INFO_DATA_VALID_FLAGS_NONE          = 0x0000;
static const mip_gnss_base_station_info_data_valid_flags MIP_GNSS_BASE_STATION_INFO_DATA_VALID_FLAGS_TOW           = 0x0001; ///<  
static const mip_gnss_base_station_info_data_valid_flags MIP_GNSS_BASE_STATION_INFO_DATA_VALID_FLAGS_WEEK_NUMBER   = 0x0002; ///<  
static const mip_gnss_base_station_info_data_valid_flags MIP_GNSS_BASE_STATION_INFO_DATA_VALID_FLAGS_ECEF_POSITION = 0x0004; ///<  
static const mip_gnss_base_station_info_data_valid_flags MIP_GNSS_BASE_STATION_INFO_DATA_VALID_FLAGS_HEIGHT        = 0x0008; ///<  
static const mip_gnss_base_station_info_data_valid_flags MIP_GNSS_BASE_STATION_INFO_DATA_VALID_FLAGS_STATION_ID    = 0x0010; ///<  
static const mip_gnss_base_station_info_data_valid_flags MIP_GNSS_BASE_STATION_INFO_DATA_VALID_FLAGS_INDICATORS    = 0x0020; ///<  
static const mip_gnss_base_station_info_data_valid_flags MIP_GNSS_BASE_STATION_INFO_DATA_VALID_FLAGS_FLAGS         = 0x003F; ///<  
static const mip_gnss_base_station_info_data_valid_flags MIP_GNSS_BASE_STATION_INFO_DATA_VALID_FLAGS_ALL           = 0x003F;
static inline void insert_mip_gnss_base_station_info_data_valid_flags(microstrain_serializer* serializer, const mip_gnss_base_station_info_data_valid_flags self)
{
    microstrain_insert_u16(serializer, (uint16_t)(self));
}
static inline void extract_mip_gnss_base_station_info_data_valid_flags(microstrain_serializer* serializer, mip_gnss_base_station_info_data_valid_flags* self)
{
    uint16_t tmp = 0;
    microstrain_extract_u16(serializer, &tmp);
    *self = (mip_gnss_base_station_info_data_valid_flags)tmp;
}


struct mip_gnss_base_station_info_data
{
    double time_of_week; ///< GPS Time of week the message was received [seconds]
    uint16_t week_number; ///< GPS Week since 1980 [weeks]
    mip_vector3d ecef_pos; ///< Earth-centered, Earth-fixed [m]
    float height; ///< Antenna Height above the marker used in the survey [m]
    uint16_t station_id; ///< Range: 0-4095
    mip_gnss_base_station_info_data_indicator_flags indicators; ///< Bitfield
    mip_gnss_base_station_info_data_valid_flags valid_flags;
};
typedef struct mip_gnss_base_station_info_data mip_gnss_base_station_info_data;

void insert_mip_gnss_base_station_info_data(microstrain_serializer* serializer, const mip_gnss_base_station_info_data* self);
void extract_mip_gnss_base_station_info_data(microstrain_serializer* serializer, mip_gnss_base_station_info_data* self);
bool extract_mip_gnss_base_station_info_data_from_field(const mip_field_view* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup gnss_rtk_corrections_status_c  (0x81,0x31) Rtk Corrections Status
///
///@{

typedef uint16_t mip_gnss_rtk_corrections_status_data_valid_flags;
static const mip_gnss_rtk_corrections_status_data_valid_flags MIP_GNSS_RTK_CORRECTIONS_STATUS_DATA_VALID_FLAGS_NONE            = 0x0000;
static const mip_gnss_rtk_corrections_status_data_valid_flags MIP_GNSS_RTK_CORRECTIONS_STATUS_DATA_VALID_FLAGS_TOW             = 0x0001; ///<  
static const mip_gnss_rtk_corrections_status_data_valid_flags MIP_GNSS_RTK_CORRECTIONS_STATUS_DATA_VALID_FLAGS_WEEK_NUMBER     = 0x0002; ///<  
static const mip_gnss_rtk_corrections_status_data_valid_flags MIP_GNSS_RTK_CORRECTIONS_STATUS_DATA_VALID_FLAGS_EPOCH_STATUS    = 0x0004; ///<  
static const mip_gnss_rtk_corrections_status_data_valid_flags MIP_GNSS_RTK_CORRECTIONS_STATUS_DATA_VALID_FLAGS_DONGLE_STATUS   = 0x0008; ///<  
static const mip_gnss_rtk_corrections_status_data_valid_flags MIP_GNSS_RTK_CORRECTIONS_STATUS_DATA_VALID_FLAGS_GPS_LATENCY     = 0x0010; ///<  
static const mip_gnss_rtk_corrections_status_data_valid_flags MIP_GNSS_RTK_CORRECTIONS_STATUS_DATA_VALID_FLAGS_GLONASS_LATENCY = 0x0020; ///<  
static const mip_gnss_rtk_corrections_status_data_valid_flags MIP_GNSS_RTK_CORRECTIONS_STATUS_DATA_VALID_FLAGS_GALILEO_LATENCY = 0x0040; ///<  
static const mip_gnss_rtk_corrections_status_data_valid_flags MIP_GNSS_RTK_CORRECTIONS_STATUS_DATA_VALID_FLAGS_BEIDOU_LATENCY  = 0x0080; ///<  
static const mip_gnss_rtk_corrections_status_data_valid_flags MIP_GNSS_RTK_CORRECTIONS_STATUS_DATA_VALID_FLAGS_FLAGS           = 0x00FF; ///<  
static const mip_gnss_rtk_corrections_status_data_valid_flags MIP_GNSS_RTK_CORRECTIONS_STATUS_DATA_VALID_FLAGS_ALL             = 0x00FF;
static inline void insert_mip_gnss_rtk_corrections_status_data_valid_flags(microstrain_serializer* serializer, const mip_gnss_rtk_corrections_status_data_valid_flags self)
{
    microstrain_insert_u16(serializer, (uint16_t)(self));
}
static inline void extract_mip_gnss_rtk_corrections_status_data_valid_flags(microstrain_serializer* serializer, mip_gnss_rtk_corrections_status_data_valid_flags* self)
{
    uint16_t tmp = 0;
    microstrain_extract_u16(serializer, &tmp);
    *self = (mip_gnss_rtk_corrections_status_data_valid_flags)tmp;
}

typedef uint16_t mip_gnss_rtk_corrections_status_data_epoch_status;
static const mip_gnss_rtk_corrections_status_data_epoch_status MIP_GNSS_RTK_CORRECTIONS_STATUS_DATA_EPOCH_STATUS_NONE                         = 0x0000;
static const mip_gnss_rtk_corrections_status_data_epoch_status MIP_GNSS_RTK_CORRECTIONS_STATUS_DATA_EPOCH_STATUS_ANTENNA_LOCATION_RECEIVED    = 0x0001; ///<  
static const mip_gnss_rtk_corrections_status_data_epoch_status MIP_GNSS_RTK_CORRECTIONS_STATUS_DATA_EPOCH_STATUS_ANTENNA_DESCRIPTION_RECEIVED = 0x0002; ///<  
static const mip_gnss_rtk_corrections_status_data_epoch_status MIP_GNSS_RTK_CORRECTIONS_STATUS_DATA_EPOCH_STATUS_GPS_RECEIVED                 = 0x0004; ///<  
static const mip_gnss_rtk_corrections_status_data_epoch_status MIP_GNSS_RTK_CORRECTIONS_STATUS_DATA_EPOCH_STATUS_GLONASS_RECEIVED             = 0x0008; ///<  
static const mip_gnss_rtk_corrections_status_data_epoch_status MIP_GNSS_RTK_CORRECTIONS_STATUS_DATA_EPOCH_STATUS_GALILEO_RECEIVED             = 0x0010; ///<  
static const mip_gnss_rtk_corrections_status_data_epoch_status MIP_GNSS_RTK_CORRECTIONS_STATUS_DATA_EPOCH_STATUS_BEIDOU_RECEIVED              = 0x0020; ///<  
static const mip_gnss_rtk_corrections_status_data_epoch_status MIP_GNSS_RTK_CORRECTIONS_STATUS_DATA_EPOCH_STATUS_USING_GPS_MSM_MESSAGES       = 0x0040; ///<  Using MSM messages for GPS corrections instead of RTCM messages 1001-1004
static const mip_gnss_rtk_corrections_status_data_epoch_status MIP_GNSS_RTK_CORRECTIONS_STATUS_DATA_EPOCH_STATUS_USING_GLONASS_MSM_MESSAGES   = 0x0080; ///<  Using MSM messages for GLONASS corrections instead of RTCM messages 1009-1012
static const mip_gnss_rtk_corrections_status_data_epoch_status MIP_GNSS_RTK_CORRECTIONS_STATUS_DATA_EPOCH_STATUS_DONGLE_STATUS_READ_FAILED    = 0x0100; ///<  A read of the dongle status was attempted, but failed
static const mip_gnss_rtk_corrections_status_data_epoch_status MIP_GNSS_RTK_CORRECTIONS_STATUS_DATA_EPOCH_STATUS_ALL                          = 0x01FF;
static inline void insert_mip_gnss_rtk_corrections_status_data_epoch_status(microstrain_serializer* serializer, const mip_gnss_rtk_corrections_status_data_epoch_status self)
{
    microstrain_insert_u16(serializer, (uint16_t)(self));
}
static inline void extract_mip_gnss_rtk_corrections_status_data_epoch_status(microstrain_serializer* serializer, mip_gnss_rtk_corrections_status_data_epoch_status* self)
{
    uint16_t tmp = 0;
    microstrain_extract_u16(serializer, &tmp);
    *self = (mip_gnss_rtk_corrections_status_data_epoch_status)tmp;
}


struct mip_gnss_rtk_corrections_status_data
{
    double time_of_week; ///< GPS Time of week [seconds]
    uint16_t week_number; ///< GPS Week since 1980 [weeks]
    mip_gnss_rtk_corrections_status_data_epoch_status epoch_status; ///< Status of the corrections received during this epoch
    uint32_t dongle_status; ///< RTK Dongle Status Flags (valid only when using RTK dongle, see Get RTK Device Status Flags (0x0F,0x01) for details)
    float gps_correction_latency; ///< Latency of last GPS correction [seconds]
    float glonass_correction_latency; ///< Latency of last GLONASS correction [seconds]
    float galileo_correction_latency; ///< Latency of last Galileo correction [seconds]
    float beidou_correction_latency; ///< Latency of last Beidou correction [seconds]
    uint32_t reserved[4]; ///< Reserved for future use
    mip_gnss_rtk_corrections_status_data_valid_flags valid_flags;
};
typedef struct mip_gnss_rtk_corrections_status_data mip_gnss_rtk_corrections_status_data;

void insert_mip_gnss_rtk_corrections_status_data(microstrain_serializer* serializer, const mip_gnss_rtk_corrections_status_data* self);
void extract_mip_gnss_rtk_corrections_status_data(microstrain_serializer* serializer, mip_gnss_rtk_corrections_status_data* self);
bool extract_mip_gnss_rtk_corrections_status_data_from_field(const mip_field_view* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup gnss_satellite_status_c  (0x81,0x20) Satellite Status
/// Status information for a GNSS satellite.
///
///@{

typedef uint16_t mip_gnss_satellite_status_data_valid_flags;
static const mip_gnss_satellite_status_data_valid_flags MIP_GNSS_SATELLITE_STATUS_DATA_VALID_FLAGS_NONE         = 0x0000;
static const mip_gnss_satellite_status_data_valid_flags MIP_GNSS_SATELLITE_STATUS_DATA_VALID_FLAGS_TOW          = 0x0001; ///<  
static const mip_gnss_satellite_status_data_valid_flags MIP_GNSS_SATELLITE_STATUS_DATA_VALID_FLAGS_WEEK_NUMBER  = 0x0002; ///<  
static const mip_gnss_satellite_status_data_valid_flags MIP_GNSS_SATELLITE_STATUS_DATA_VALID_FLAGS_GNSS_ID      = 0x0004; ///<  
static const mip_gnss_satellite_status_data_valid_flags MIP_GNSS_SATELLITE_STATUS_DATA_VALID_FLAGS_SATELLITE_ID = 0x0008; ///<  
static const mip_gnss_satellite_status_data_valid_flags MIP_GNSS_SATELLITE_STATUS_DATA_VALID_FLAGS_ELEVATION    = 0x0010; ///<  
static const mip_gnss_satellite_status_data_valid_flags MIP_GNSS_SATELLITE_STATUS_DATA_VALID_FLAGS_AZIMUTH      = 0x0020; ///<  
static const mip_gnss_satellite_status_data_valid_flags MIP_GNSS_SATELLITE_STATUS_DATA_VALID_FLAGS_HEALTH       = 0x0040; ///<  
static const mip_gnss_satellite_status_data_valid_flags MIP_GNSS_SATELLITE_STATUS_DATA_VALID_FLAGS_FLAGS        = 0x007F; ///<  
static const mip_gnss_satellite_status_data_valid_flags MIP_GNSS_SATELLITE_STATUS_DATA_VALID_FLAGS_ALL          = 0x007F;
static inline void insert_mip_gnss_satellite_status_data_valid_flags(microstrain_serializer* serializer, const mip_gnss_satellite_status_data_valid_flags self)
{
    microstrain_insert_u16(serializer, (uint16_t)(self));
}
static inline void extract_mip_gnss_satellite_status_data_valid_flags(microstrain_serializer* serializer, mip_gnss_satellite_status_data_valid_flags* self)
{
    uint16_t tmp = 0;
    microstrain_extract_u16(serializer, &tmp);
    *self = (mip_gnss_satellite_status_data_valid_flags)tmp;
}


struct mip_gnss_satellite_status_data
{
    uint8_t index; ///< Index of this field in this epoch.
    uint8_t count; ///< Total number of fields in this epoch.
    double time_of_week; ///< GPS Time of week [seconds]
    uint16_t week_number; ///< GPS Week since 1980 [weeks]
    mip_gnss_constellation_id gnss_id;
    uint8_t satellite_id; ///< GNSS satellite id within the constellation
    float elevation; ///< Elevation of the satellite relative to the rover [degrees]
    float azimuth; ///< Azimuth of the satellite relative to the rover [degrees]
    bool health; ///< True if the satellite is healthy.
    mip_gnss_satellite_status_data_valid_flags valid_flags;
};
typedef struct mip_gnss_satellite_status_data mip_gnss_satellite_status_data;

void insert_mip_gnss_satellite_status_data(microstrain_serializer* serializer, const mip_gnss_satellite_status_data* self);
void extract_mip_gnss_satellite_status_data(microstrain_serializer* serializer, mip_gnss_satellite_status_data* self);
bool extract_mip_gnss_satellite_status_data_from_field(const mip_field_view* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup gnss_raw_c  (0x81,0x22) Raw
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
typedef enum mip_gnss_raw_data_gnss_signal_quality mip_gnss_raw_data_gnss_signal_quality;

static inline void insert_mip_gnss_raw_data_gnss_signal_quality(microstrain_serializer* serializer, const mip_gnss_raw_data_gnss_signal_quality self)
{
    microstrain_insert_u8(serializer, (uint8_t)(self));
}
static inline void extract_mip_gnss_raw_data_gnss_signal_quality(microstrain_serializer* serializer, mip_gnss_raw_data_gnss_signal_quality* self)
{
    uint8_t tmp = 0;
    microstrain_extract_u8(serializer, &tmp);
    *self = (mip_gnss_raw_data_gnss_signal_quality)tmp;
}

typedef uint16_t mip_gnss_raw_data_valid_flags;
static const mip_gnss_raw_data_valid_flags MIP_GNSS_RAW_DATA_VALID_FLAGS_NONE                      = 0x0000;
static const mip_gnss_raw_data_valid_flags MIP_GNSS_RAW_DATA_VALID_FLAGS_TOW                       = 0x0001; ///<  
static const mip_gnss_raw_data_valid_flags MIP_GNSS_RAW_DATA_VALID_FLAGS_WEEK_NUMBER               = 0x0002; ///<  
static const mip_gnss_raw_data_valid_flags MIP_GNSS_RAW_DATA_VALID_FLAGS_RECEIVER_ID               = 0x0004; ///<  
static const mip_gnss_raw_data_valid_flags MIP_GNSS_RAW_DATA_VALID_FLAGS_TRACKING_CHANNEL          = 0x0008; ///<  
static const mip_gnss_raw_data_valid_flags MIP_GNSS_RAW_DATA_VALID_FLAGS_GNSS_ID                   = 0x0010; ///<  
static const mip_gnss_raw_data_valid_flags MIP_GNSS_RAW_DATA_VALID_FLAGS_SATELLITE_ID              = 0x0020; ///<  
static const mip_gnss_raw_data_valid_flags MIP_GNSS_RAW_DATA_VALID_FLAGS_SIGNAL_ID                 = 0x0040; ///<  
static const mip_gnss_raw_data_valid_flags MIP_GNSS_RAW_DATA_VALID_FLAGS_SIGNAL_STRENGTH           = 0x0080; ///<  
static const mip_gnss_raw_data_valid_flags MIP_GNSS_RAW_DATA_VALID_FLAGS_QUALITY                   = 0x0100; ///<  
static const mip_gnss_raw_data_valid_flags MIP_GNSS_RAW_DATA_VALID_FLAGS_PSEUDORANGE               = 0x0200; ///<  
static const mip_gnss_raw_data_valid_flags MIP_GNSS_RAW_DATA_VALID_FLAGS_CARRIER_PHASE             = 0x0400; ///<  
static const mip_gnss_raw_data_valid_flags MIP_GNSS_RAW_DATA_VALID_FLAGS_DOPPLER                   = 0x0800; ///<  
static const mip_gnss_raw_data_valid_flags MIP_GNSS_RAW_DATA_VALID_FLAGS_RANGE_UNCERTAINTY         = 0x1000; ///<  
static const mip_gnss_raw_data_valid_flags MIP_GNSS_RAW_DATA_VALID_FLAGS_CARRIER_PHASE_UNCERTAINTY = 0x2000; ///<  
static const mip_gnss_raw_data_valid_flags MIP_GNSS_RAW_DATA_VALID_FLAGS_DOPPLER_UNCERTAINTY       = 0x4000; ///<  
static const mip_gnss_raw_data_valid_flags MIP_GNSS_RAW_DATA_VALID_FLAGS_LOCK_TIME                 = 0x8000; ///<  
static const mip_gnss_raw_data_valid_flags MIP_GNSS_RAW_DATA_VALID_FLAGS_FLAGS                     = 0xFFFF; ///<  
static const mip_gnss_raw_data_valid_flags MIP_GNSS_RAW_DATA_VALID_FLAGS_ALL                       = 0xFFFF;
static inline void insert_mip_gnss_raw_data_valid_flags(microstrain_serializer* serializer, const mip_gnss_raw_data_valid_flags self)
{
    microstrain_insert_u16(serializer, (uint16_t)(self));
}
static inline void extract_mip_gnss_raw_data_valid_flags(microstrain_serializer* serializer, mip_gnss_raw_data_valid_flags* self)
{
    uint16_t tmp = 0;
    microstrain_extract_u16(serializer, &tmp);
    *self = (mip_gnss_raw_data_valid_flags)tmp;
}


struct mip_gnss_raw_data
{
    uint8_t index; ///< Index of this field in this epoch.
    uint8_t count; ///< Total number of fields in this epoch.
    double time_of_week; ///< GPS Time of week [seconds]
    uint16_t week_number; ///< GPS Week since 1980 [weeks]
    uint16_t receiver_id; ///< When the measurement comes from RTCM, this will be the reference station ID; otherwise, it's the receiver number (1,2,...)
    uint8_t tracking_channel; ///< Channel the receiver is using to track this satellite.
    mip_gnss_constellation_id gnss_id;
    uint8_t satellite_id; ///< GNSS satellite id within the constellation.
    mip_gnss_signal_id signal_id; ///< Signal identifier for the satellite.
    float signal_strength; ///< Carrier to noise ratio [dBHz].
    mip_gnss_raw_data_gnss_signal_quality quality; ///< Indicator of signal quality.
    double pseudorange; ///< Pseudo-range measurement [meters].
    double carrier_phase; ///< Carrier phase measurement [Carrier periods].
    float doppler; ///< Measured doppler shift [Hz].
    float range_uncert; ///< Uncertainty of the pseudo-range measurement [m].
    float phase_uncert; ///< Uncertainty of the phase measurement [Carrier periods].
    float doppler_uncert; ///< Uncertainty of the measured doppler shift [Hz].
    float lock_time; ///< DOC Minimum carrier phase lock time [s].  Note: the maximum value is dependent on the receiver.
    mip_gnss_raw_data_valid_flags valid_flags;
};
typedef struct mip_gnss_raw_data mip_gnss_raw_data;

void insert_mip_gnss_raw_data(microstrain_serializer* serializer, const mip_gnss_raw_data* self);
void extract_mip_gnss_raw_data(microstrain_serializer* serializer, mip_gnss_raw_data* self);
bool extract_mip_gnss_raw_data_from_field(const mip_field_view* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup gnss_gps_ephemeris_c  (0x81,0x61) Gps Ephemeris
/// GPS Ephemeris Data
///
///@{

typedef uint16_t mip_gnss_gps_ephemeris_data_valid_flags;
static const mip_gnss_gps_ephemeris_data_valid_flags MIP_GNSS_GPS_EPHEMERIS_DATA_VALID_FLAGS_NONE        = 0x0000;
static const mip_gnss_gps_ephemeris_data_valid_flags MIP_GNSS_GPS_EPHEMERIS_DATA_VALID_FLAGS_EPHEMERIS   = 0x0001; ///<  
static const mip_gnss_gps_ephemeris_data_valid_flags MIP_GNSS_GPS_EPHEMERIS_DATA_VALID_FLAGS_MODERN_DATA = 0x0002; ///<  
static const mip_gnss_gps_ephemeris_data_valid_flags MIP_GNSS_GPS_EPHEMERIS_DATA_VALID_FLAGS_ISC_L5      = 0x0004; ///<  
static const mip_gnss_gps_ephemeris_data_valid_flags MIP_GNSS_GPS_EPHEMERIS_DATA_VALID_FLAGS_FLAGS       = 0x0007; ///<  
static const mip_gnss_gps_ephemeris_data_valid_flags MIP_GNSS_GPS_EPHEMERIS_DATA_VALID_FLAGS_ALL         = 0x0007;
static inline void insert_mip_gnss_gps_ephemeris_data_valid_flags(microstrain_serializer* serializer, const mip_gnss_gps_ephemeris_data_valid_flags self)
{
    microstrain_insert_u16(serializer, (uint16_t)(self));
}
static inline void extract_mip_gnss_gps_ephemeris_data_valid_flags(microstrain_serializer* serializer, mip_gnss_gps_ephemeris_data_valid_flags* self)
{
    uint16_t tmp = 0;
    microstrain_extract_u16(serializer, &tmp);
    *self = (mip_gnss_gps_ephemeris_data_valid_flags)tmp;
}


struct mip_gnss_gps_ephemeris_data
{
    uint8_t index; ///< Index of this field in this epoch.
    uint8_t count; ///< Total number of fields in this epoch.
    double time_of_week; ///< GPS Time of week [seconds]
    uint16_t week_number; ///< GPS Week since 1980 [weeks]
    uint8_t satellite_id; ///< GNSS satellite id within the constellation.
    uint8_t health; ///< Satellite and signal health
    uint8_t iodc; ///< Issue of Data Clock. This increments each time the data changes and rolls over at 4. It is used to make sure various raw data elements from different sources line up correctly.
    uint8_t iode; ///< Issue of Data Ephemeris.
    double t_oc; ///< Reference time for clock data.
    double af0; ///< Clock bias in [s].
    double af1; ///< Clock drift in [s/s].
    double af2; ///< Clock drift rate in [s/s^2].
    double t_gd; ///< T Group Delay [s].
    double ISC_L1CA; ///< Inter-signal correction (L1).
    double ISC_L2C; ///< Inter-signal correction (L2, or L5 if isc_l5 flag is set).
    double t_oe; ///< Reference time for ephemeris in [s].
    double a; ///< Semi-major axis [m].
    double a_dot; ///< Semi-major axis rate [m/s].
    double mean_anomaly; ///< [rad].
    double delta_mean_motion; ///< [rad].
    double delta_mean_motion_dot; ///< [rad/s].
    double eccentricity;
    double argument_of_perigee; ///< [rad].
    double omega; ///< Longitude of Ascending Node [rad].
    double omega_dot; ///< Rate of Right Ascension [rad/s].
    double inclination; ///< Inclination angle [rad].
    double inclination_dot; ///< Inclination angle rate of change [rad/s].
    double c_ic; ///< Harmonic Correction Term.
    double c_is; ///< Harmonic Correction Term.
    double c_uc; ///< Harmonic Correction Term.
    double c_us; ///< Harmonic Correction Term.
    double c_rc; ///< Harmonic Correction Term.
    double c_rs; ///< Harmonic Correction Term.
    mip_gnss_gps_ephemeris_data_valid_flags valid_flags;
};
typedef struct mip_gnss_gps_ephemeris_data mip_gnss_gps_ephemeris_data;

void insert_mip_gnss_gps_ephemeris_data(microstrain_serializer* serializer, const mip_gnss_gps_ephemeris_data* self);
void extract_mip_gnss_gps_ephemeris_data(microstrain_serializer* serializer, mip_gnss_gps_ephemeris_data* self);
bool extract_mip_gnss_gps_ephemeris_data_from_field(const mip_field_view* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup gnss_galileo_ephemeris_c  (0x81,0x63) Galileo Ephemeris
/// Galileo Ephemeris Data
///
///@{

typedef uint16_t mip_gnss_galileo_ephemeris_data_valid_flags;
static const mip_gnss_galileo_ephemeris_data_valid_flags MIP_GNSS_GALILEO_EPHEMERIS_DATA_VALID_FLAGS_NONE        = 0x0000;
static const mip_gnss_galileo_ephemeris_data_valid_flags MIP_GNSS_GALILEO_EPHEMERIS_DATA_VALID_FLAGS_EPHEMERIS   = 0x0001; ///<  
static const mip_gnss_galileo_ephemeris_data_valid_flags MIP_GNSS_GALILEO_EPHEMERIS_DATA_VALID_FLAGS_MODERN_DATA = 0x0002; ///<  
static const mip_gnss_galileo_ephemeris_data_valid_flags MIP_GNSS_GALILEO_EPHEMERIS_DATA_VALID_FLAGS_ISC_L5      = 0x0004; ///<  
static const mip_gnss_galileo_ephemeris_data_valid_flags MIP_GNSS_GALILEO_EPHEMERIS_DATA_VALID_FLAGS_FLAGS       = 0x0007; ///<  
static const mip_gnss_galileo_ephemeris_data_valid_flags MIP_GNSS_GALILEO_EPHEMERIS_DATA_VALID_FLAGS_ALL         = 0x0007;
static inline void insert_mip_gnss_galileo_ephemeris_data_valid_flags(microstrain_serializer* serializer, const mip_gnss_galileo_ephemeris_data_valid_flags self)
{
    microstrain_insert_u16(serializer, (uint16_t)(self));
}
static inline void extract_mip_gnss_galileo_ephemeris_data_valid_flags(microstrain_serializer* serializer, mip_gnss_galileo_ephemeris_data_valid_flags* self)
{
    uint16_t tmp = 0;
    microstrain_extract_u16(serializer, &tmp);
    *self = (mip_gnss_galileo_ephemeris_data_valid_flags)tmp;
}


struct mip_gnss_galileo_ephemeris_data
{
    uint8_t index; ///< Index of this field in this epoch.
    uint8_t count; ///< Total number of fields in this epoch.
    double time_of_week; ///< GPS Time of week [seconds]
    uint16_t week_number; ///< GPS Week since 1980 [weeks]
    uint8_t satellite_id; ///< GNSS satellite id within the constellation.
    uint8_t health; ///< Satellite and signal health
    uint8_t iodc; ///< Issue of Data Clock. This increments each time the data changes and rolls over at 4. It is used to make sure various raw data elements from different sources line up correctly.
    uint8_t iode; ///< Issue of Data Ephemeris.
    double t_oc; ///< Reference time for clock data.
    double af0; ///< Clock bias in [s].
    double af1; ///< Clock drift in [s/s].
    double af2; ///< Clock drift rate in [s/s^2].
    double t_gd; ///< T Group Delay [s].
    double ISC_L1CA; ///< Inter-signal correction (L1).
    double ISC_L2C; ///< Inter-signal correction (L2, or L5 if isc_l5 flag is set).
    double t_oe; ///< Reference time for ephemeris in [s].
    double a; ///< Semi-major axis [m].
    double a_dot; ///< Semi-major axis rate [m/s].
    double mean_anomaly; ///< [rad].
    double delta_mean_motion; ///< [rad].
    double delta_mean_motion_dot; ///< [rad/s].
    double eccentricity;
    double argument_of_perigee; ///< [rad].
    double omega; ///< Longitude of Ascending Node [rad].
    double omega_dot; ///< Rate of Right Ascension [rad/s].
    double inclination; ///< Inclination angle [rad].
    double inclination_dot; ///< Inclination angle rate of change [rad/s].
    double c_ic; ///< Harmonic Correction Term.
    double c_is; ///< Harmonic Correction Term.
    double c_uc; ///< Harmonic Correction Term.
    double c_us; ///< Harmonic Correction Term.
    double c_rc; ///< Harmonic Correction Term.
    double c_rs; ///< Harmonic Correction Term.
    mip_gnss_galileo_ephemeris_data_valid_flags valid_flags;
};
typedef struct mip_gnss_galileo_ephemeris_data mip_gnss_galileo_ephemeris_data;

void insert_mip_gnss_galileo_ephemeris_data(microstrain_serializer* serializer, const mip_gnss_galileo_ephemeris_data* self);
void extract_mip_gnss_galileo_ephemeris_data(microstrain_serializer* serializer, mip_gnss_galileo_ephemeris_data* self);
bool extract_mip_gnss_galileo_ephemeris_data_from_field(const mip_field_view* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup gnss_glo_ephemeris_c  (0x81,0x62) Glo Ephemeris
/// Glonass Ephemeris Data
///
///@{

typedef uint16_t mip_gnss_glo_ephemeris_data_valid_flags;
static const mip_gnss_glo_ephemeris_data_valid_flags MIP_GNSS_GLO_EPHEMERIS_DATA_VALID_FLAGS_NONE      = 0x0000;
static const mip_gnss_glo_ephemeris_data_valid_flags MIP_GNSS_GLO_EPHEMERIS_DATA_VALID_FLAGS_EPHEMERIS = 0x0001; ///<  
static const mip_gnss_glo_ephemeris_data_valid_flags MIP_GNSS_GLO_EPHEMERIS_DATA_VALID_FLAGS_FLAGS     = 0x0001; ///<  
static const mip_gnss_glo_ephemeris_data_valid_flags MIP_GNSS_GLO_EPHEMERIS_DATA_VALID_FLAGS_ALL       = 0x0001;
static inline void insert_mip_gnss_glo_ephemeris_data_valid_flags(microstrain_serializer* serializer, const mip_gnss_glo_ephemeris_data_valid_flags self)
{
    microstrain_insert_u16(serializer, (uint16_t)(self));
}
static inline void extract_mip_gnss_glo_ephemeris_data_valid_flags(microstrain_serializer* serializer, mip_gnss_glo_ephemeris_data_valid_flags* self)
{
    uint16_t tmp = 0;
    microstrain_extract_u16(serializer, &tmp);
    *self = (mip_gnss_glo_ephemeris_data_valid_flags)tmp;
}


struct mip_gnss_glo_ephemeris_data
{
    uint8_t index; ///< Index of this field in this epoch.
    uint8_t count; ///< Total number of fields in this epoch.
    double time_of_week; ///< GPS Time of week [seconds]
    uint16_t week_number; ///< GPS Week since 1980 [weeks]
    uint8_t satellite_id; ///< GNSS satellite id within the constellation.
    int8_t freq_number; ///< GLONASS frequency number (-7 to 24)
    uint32_t tk; ///< Frame start time within current day [seconds]
    uint32_t tb; ///< Ephemeris reference time [seconds]
    uint8_t sat_type; ///< Type of satellite (M) GLONASS = 0, GLONASS-M = 1
    double gamma; ///< Relative deviation of carrier frequency from nominal [dimensionless]
    double tau_n; ///< Time correction relative to GLONASS Time [seconds]
    mip_vector3d x; ///< Satellite PE-90 position [m]
    mip_vector3f v; ///< Satellite PE-90 velocity [m/s]
    mip_vector3f a; ///< Satellite PE-90 acceleration due to perturbations [m/s^2]
    uint8_t health; ///< Satellite Health (Bn), Non-zero indicates satellite malfunction
    uint8_t P; ///< Satellite operation mode (See GLONASS ICD)
    uint8_t NT; ///< Day number within a 4 year period.
    float delta_tau_n; ///< Time difference between L1 and L2[m/s]
    uint8_t Ft; ///< User Range Accuracy (See GLONASS ICD)
    uint8_t En; ///< Age of current information [days]
    uint8_t P1; ///< Time interval between adjacent values of tb [minutes]
    uint8_t P2; ///< Oddness "1" or evenness "0" of the value of tb.
    uint8_t P3; ///< Number of satellites in almanac for this frame
    uint8_t P4; ///< Flag indicating ephemeris parameters are present
    mip_gnss_glo_ephemeris_data_valid_flags valid_flags;
};
typedef struct mip_gnss_glo_ephemeris_data mip_gnss_glo_ephemeris_data;

void insert_mip_gnss_glo_ephemeris_data(microstrain_serializer* serializer, const mip_gnss_glo_ephemeris_data* self);
void extract_mip_gnss_glo_ephemeris_data(microstrain_serializer* serializer, mip_gnss_glo_ephemeris_data* self);
bool extract_mip_gnss_glo_ephemeris_data_from_field(const mip_field_view* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup gnss_beidou_ephemeris_c  (0x81,0x64) Beidou Ephemeris
/// BeiDou Ephemeris Data
///
///@{

typedef uint16_t mip_gnss_beidou_ephemeris_data_valid_flags;
static const mip_gnss_beidou_ephemeris_data_valid_flags MIP_GNSS_BEIDOU_EPHEMERIS_DATA_VALID_FLAGS_NONE        = 0x0000;
static const mip_gnss_beidou_ephemeris_data_valid_flags MIP_GNSS_BEIDOU_EPHEMERIS_DATA_VALID_FLAGS_EPHEMERIS   = 0x0001; ///<  
static const mip_gnss_beidou_ephemeris_data_valid_flags MIP_GNSS_BEIDOU_EPHEMERIS_DATA_VALID_FLAGS_MODERN_DATA = 0x0002; ///<  
static const mip_gnss_beidou_ephemeris_data_valid_flags MIP_GNSS_BEIDOU_EPHEMERIS_DATA_VALID_FLAGS_ISC_L5      = 0x0004; ///<  
static const mip_gnss_beidou_ephemeris_data_valid_flags MIP_GNSS_BEIDOU_EPHEMERIS_DATA_VALID_FLAGS_FLAGS       = 0x0007; ///<  
static const mip_gnss_beidou_ephemeris_data_valid_flags MIP_GNSS_BEIDOU_EPHEMERIS_DATA_VALID_FLAGS_ALL         = 0x0007;
static inline void insert_mip_gnss_beidou_ephemeris_data_valid_flags(microstrain_serializer* serializer, const mip_gnss_beidou_ephemeris_data_valid_flags self)
{
    microstrain_insert_u16(serializer, (uint16_t)(self));
}
static inline void extract_mip_gnss_beidou_ephemeris_data_valid_flags(microstrain_serializer* serializer, mip_gnss_beidou_ephemeris_data_valid_flags* self)
{
    uint16_t tmp = 0;
    microstrain_extract_u16(serializer, &tmp);
    *self = (mip_gnss_beidou_ephemeris_data_valid_flags)tmp;
}


struct mip_gnss_beidou_ephemeris_data
{
    uint8_t index; ///< Index of this field in this epoch.
    uint8_t count; ///< Total number of fields in this epoch.
    double time_of_week; ///< GPS Time of week [seconds]
    uint16_t week_number; ///< GPS Week since 1980 [weeks]
    uint8_t satellite_id; ///< GNSS satellite id within the constellation.
    uint8_t health; ///< Satellite and signal health
    uint8_t iodc; ///< Issue of Data Clock. This increments each time the data changes and rolls over at 4. It is used to make sure various raw data elements from different sources line up correctly.
    uint8_t iode; ///< Issue of Data Ephemeris.
    double t_oc; ///< Reference time for clock data.
    double af0; ///< Clock bias in [s].
    double af1; ///< Clock drift in [s/s].
    double af2; ///< Clock drift rate in [s/s^2].
    double t_gd; ///< T Group Delay [s].
    double ISC_L1CA; ///< Inter-signal correction (L1).
    double ISC_L2C; ///< Inter-signal correction (L2, or L5 if isc_l5 flag is set).
    double t_oe; ///< Reference time for ephemeris in [s].
    double a; ///< Semi-major axis [m].
    double a_dot; ///< Semi-major axis rate [m/s].
    double mean_anomaly; ///< [rad].
    double delta_mean_motion; ///< [rad].
    double delta_mean_motion_dot; ///< [rad/s].
    double eccentricity;
    double argument_of_perigee; ///< [rad].
    double omega; ///< Longitude of Ascending Node [rad].
    double omega_dot; ///< Rate of Right Ascension [rad/s].
    double inclination; ///< Inclination angle [rad].
    double inclination_dot; ///< Inclination angle rate of change [rad/s].
    double c_ic; ///< Harmonic Correction Term.
    double c_is; ///< Harmonic Correction Term.
    double c_uc; ///< Harmonic Correction Term.
    double c_us; ///< Harmonic Correction Term.
    double c_rc; ///< Harmonic Correction Term.
    double c_rs; ///< Harmonic Correction Term.
    mip_gnss_beidou_ephemeris_data_valid_flags valid_flags;
};
typedef struct mip_gnss_beidou_ephemeris_data mip_gnss_beidou_ephemeris_data;

void insert_mip_gnss_beidou_ephemeris_data(microstrain_serializer* serializer, const mip_gnss_beidou_ephemeris_data* self);
void extract_mip_gnss_beidou_ephemeris_data(microstrain_serializer* serializer, mip_gnss_beidou_ephemeris_data* self);
bool extract_mip_gnss_beidou_ephemeris_data_from_field(const mip_field_view* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup gnss_gps_iono_corr_c  (0x81,0x71) Gps Iono Corr
/// Ionospheric Correction Terms for GNSS
///
///@{

typedef uint16_t mip_gnss_gps_iono_corr_data_valid_flags;
static const mip_gnss_gps_iono_corr_data_valid_flags MIP_GNSS_GPS_IONO_CORR_DATA_VALID_FLAGS_NONE        = 0x0000;
static const mip_gnss_gps_iono_corr_data_valid_flags MIP_GNSS_GPS_IONO_CORR_DATA_VALID_FLAGS_TOW         = 0x0001; ///<  
static const mip_gnss_gps_iono_corr_data_valid_flags MIP_GNSS_GPS_IONO_CORR_DATA_VALID_FLAGS_WEEK_NUMBER = 0x0002; ///<  
static const mip_gnss_gps_iono_corr_data_valid_flags MIP_GNSS_GPS_IONO_CORR_DATA_VALID_FLAGS_ALPHA       = 0x0004; ///<  
static const mip_gnss_gps_iono_corr_data_valid_flags MIP_GNSS_GPS_IONO_CORR_DATA_VALID_FLAGS_BETA        = 0x0008; ///<  
static const mip_gnss_gps_iono_corr_data_valid_flags MIP_GNSS_GPS_IONO_CORR_DATA_VALID_FLAGS_FLAGS       = 0x000F; ///<  
static const mip_gnss_gps_iono_corr_data_valid_flags MIP_GNSS_GPS_IONO_CORR_DATA_VALID_FLAGS_ALL         = 0x000F;
static inline void insert_mip_gnss_gps_iono_corr_data_valid_flags(microstrain_serializer* serializer, const mip_gnss_gps_iono_corr_data_valid_flags self)
{
    microstrain_insert_u16(serializer, (uint16_t)(self));
}
static inline void extract_mip_gnss_gps_iono_corr_data_valid_flags(microstrain_serializer* serializer, mip_gnss_gps_iono_corr_data_valid_flags* self)
{
    uint16_t tmp = 0;
    microstrain_extract_u16(serializer, &tmp);
    *self = (mip_gnss_gps_iono_corr_data_valid_flags)tmp;
}


struct mip_gnss_gps_iono_corr_data
{
    double time_of_week; ///< GPS Time of week [seconds]
    uint16_t week_number; ///< GPS Week since 1980 [weeks]
    double alpha[4]; ///< Ionospheric Correction Terms.
    double beta[4]; ///< Ionospheric Correction Terms.
    mip_gnss_gps_iono_corr_data_valid_flags valid_flags;
};
typedef struct mip_gnss_gps_iono_corr_data mip_gnss_gps_iono_corr_data;

void insert_mip_gnss_gps_iono_corr_data(microstrain_serializer* serializer, const mip_gnss_gps_iono_corr_data* self);
void extract_mip_gnss_gps_iono_corr_data(microstrain_serializer* serializer, mip_gnss_gps_iono_corr_data* self);
bool extract_mip_gnss_gps_iono_corr_data_from_field(const mip_field_view* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup gnss_galileo_iono_corr_c  (0x81,0x73) Galileo Iono Corr
/// Ionospheric Correction Terms for Galileo
///
///@{

typedef uint16_t mip_gnss_galileo_iono_corr_data_valid_flags;
static const mip_gnss_galileo_iono_corr_data_valid_flags MIP_GNSS_GALILEO_IONO_CORR_DATA_VALID_FLAGS_NONE              = 0x0000;
static const mip_gnss_galileo_iono_corr_data_valid_flags MIP_GNSS_GALILEO_IONO_CORR_DATA_VALID_FLAGS_TOW               = 0x0001; ///<  
static const mip_gnss_galileo_iono_corr_data_valid_flags MIP_GNSS_GALILEO_IONO_CORR_DATA_VALID_FLAGS_WEEK_NUMBER       = 0x0002; ///<  
static const mip_gnss_galileo_iono_corr_data_valid_flags MIP_GNSS_GALILEO_IONO_CORR_DATA_VALID_FLAGS_ALPHA             = 0x0004; ///<  
static const mip_gnss_galileo_iono_corr_data_valid_flags MIP_GNSS_GALILEO_IONO_CORR_DATA_VALID_FLAGS_DISTURBANCE_FLAGS = 0x0008; ///<  
static const mip_gnss_galileo_iono_corr_data_valid_flags MIP_GNSS_GALILEO_IONO_CORR_DATA_VALID_FLAGS_FLAGS             = 0x000F; ///<  
static const mip_gnss_galileo_iono_corr_data_valid_flags MIP_GNSS_GALILEO_IONO_CORR_DATA_VALID_FLAGS_ALL               = 0x000F;
static inline void insert_mip_gnss_galileo_iono_corr_data_valid_flags(microstrain_serializer* serializer, const mip_gnss_galileo_iono_corr_data_valid_flags self)
{
    microstrain_insert_u16(serializer, (uint16_t)(self));
}
static inline void extract_mip_gnss_galileo_iono_corr_data_valid_flags(microstrain_serializer* serializer, mip_gnss_galileo_iono_corr_data_valid_flags* self)
{
    uint16_t tmp = 0;
    microstrain_extract_u16(serializer, &tmp);
    *self = (mip_gnss_galileo_iono_corr_data_valid_flags)tmp;
}


struct mip_gnss_galileo_iono_corr_data
{
    double time_of_week; ///< GPS Time of week [seconds]
    uint16_t week_number; ///< GPS Week since 1980 [weeks]
    mip_vector3d alpha; ///< Coefficients for the model.
    uint8_t disturbance_flags; ///< Region disturbance flags (bits 1-5).
    mip_gnss_galileo_iono_corr_data_valid_flags valid_flags;
};
typedef struct mip_gnss_galileo_iono_corr_data mip_gnss_galileo_iono_corr_data;

void insert_mip_gnss_galileo_iono_corr_data(microstrain_serializer* serializer, const mip_gnss_galileo_iono_corr_data* self);
void extract_mip_gnss_galileo_iono_corr_data(microstrain_serializer* serializer, mip_gnss_galileo_iono_corr_data* self);
bool extract_mip_gnss_galileo_iono_corr_data_from_field(const mip_field_view* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup gnss_beidou_iono_corr_c  (0x81,0x74) Beidou Iono Corr
/// Ionospheric Correction Terms for BeiDou
///
///@{

typedef uint16_t mip_gnss_beidou_iono_corr_data_valid_flags;
static const mip_gnss_beidou_iono_corr_data_valid_flags MIP_GNSS_BEIDOU_IONO_CORR_DATA_VALID_FLAGS_NONE        = 0x0000;
static const mip_gnss_beidou_iono_corr_data_valid_flags MIP_GNSS_BEIDOU_IONO_CORR_DATA_VALID_FLAGS_TOW         = 0x0001; ///<  
static const mip_gnss_beidou_iono_corr_data_valid_flags MIP_GNSS_BEIDOU_IONO_CORR_DATA_VALID_FLAGS_WEEK_NUMBER = 0x0002; ///<  
static const mip_gnss_beidou_iono_corr_data_valid_flags MIP_GNSS_BEIDOU_IONO_CORR_DATA_VALID_FLAGS_ALPHA       = 0x0004; ///<  
static const mip_gnss_beidou_iono_corr_data_valid_flags MIP_GNSS_BEIDOU_IONO_CORR_DATA_VALID_FLAGS_BETA        = 0x0008; ///<  
static const mip_gnss_beidou_iono_corr_data_valid_flags MIP_GNSS_BEIDOU_IONO_CORR_DATA_VALID_FLAGS_ALPHA_CORR  = 0x0010; ///<  
static const mip_gnss_beidou_iono_corr_data_valid_flags MIP_GNSS_BEIDOU_IONO_CORR_DATA_VALID_FLAGS_FLAGS       = 0x001F; ///<  
static const mip_gnss_beidou_iono_corr_data_valid_flags MIP_GNSS_BEIDOU_IONO_CORR_DATA_VALID_FLAGS_ALL         = 0x001F;
static inline void insert_mip_gnss_beidou_iono_corr_data_valid_flags(microstrain_serializer* serializer, const mip_gnss_beidou_iono_corr_data_valid_flags self)
{
    microstrain_insert_u16(serializer, (uint16_t)(self));
}
static inline void extract_mip_gnss_beidou_iono_corr_data_valid_flags(microstrain_serializer* serializer, mip_gnss_beidou_iono_corr_data_valid_flags* self)
{
    uint16_t tmp = 0;
    microstrain_extract_u16(serializer, &tmp);
    *self = (mip_gnss_beidou_iono_corr_data_valid_flags)tmp;
}


struct mip_gnss_beidou_iono_corr_data
{
    double time_of_week; ///< GPS Time of week [seconds]
    uint16_t week_number; ///< GPS Week since 1980 [weeks]
    double alpha[4]; ///< Ionospheric Delay Terms.
    double beta[4]; ///< Ionospheric Delay Terms.
    double alpha_corr[9]; ///< Ionospheric Delay Correction Terms.
    mip_gnss_beidou_iono_corr_data_valid_flags valid_flags;
};
typedef struct mip_gnss_beidou_iono_corr_data mip_gnss_beidou_iono_corr_data;

void insert_mip_gnss_beidou_iono_corr_data(microstrain_serializer* serializer, const mip_gnss_beidou_iono_corr_data* self);
void extract_mip_gnss_beidou_iono_corr_data(microstrain_serializer* serializer, mip_gnss_beidou_iono_corr_data* self);
bool extract_mip_gnss_beidou_iono_corr_data_from_field(const mip_field_view* field, void* ptr);


///@}
///

///@}
///@}
///
////////////////////////////////////////////////////////////////////////////////
#ifdef __cplusplus
} // extern "C"
} // namespace C
} // namespace mip
#endif // __cplusplus

