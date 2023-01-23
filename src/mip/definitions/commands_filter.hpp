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

namespace commands_filter {

////////////////////////////////////////////////////////////////////////////////
///@addtogroup MipCommands_cpp  MIP Commands [CPP]
///@{
///@defgroup filter_commands_cpp  Filter Commands [CPP]
///
///@{

////////////////////////////////////////////////////////////////////////////////
// Descriptors
////////////////////////////////////////////////////////////////////////////////

enum 
{
    DESCRIPTOR_SET                                            = 0x0D,
    
    CMD_RESET_FILTER                                          = 0x01,
    CMD_SET_INITIAL_ATTITUDE                                  = 0x02,
    CMD_SET_INITIAL_HEADING                                   = 0x03,
    CMD_SET_INITIAL_HEADING_FROM_MAG                          = 0x04,
    CMD_RUN                                                   = 0x05,
    CMD_SELECT_FILTER                                         = 0x0F,
    CMD_VEHICLE_DYNAMICS_MODE                                 = 0x10,
    CMD_SENSOR2VEHICLE_ROTATION_EULER                         = 0x11,
    CMD_SENSOR2VEHICLE_OFFSET                                 = 0x12,
    CMD_ANTENNA_OFFSET                                        = 0x13,
    CMD_ESTIMATION_CONTROL_FLAGS                              = 0x14,
    CMD_GNSS_SOURCE_CONTROL                                   = 0x15,
    CMD_EXTERNAL_GNSS_UPDATE                                  = 0x16,
    CMD_EXTERNAL_HEADING_UPDATE                               = 0x17,
    CMD_HEADING_UPDATE_CONTROL                                = 0x18,
    CMD_AUTOINIT_CONTROL                                      = 0x19,
    CMD_ACCEL_NOISE                                           = 0x1A,
    CMD_GYRO_NOISE                                            = 0x1B,
    CMD_ACCEL_BIAS_MODEL                                      = 0x1C,
    CMD_GYRO_BIAS_MODEL                                       = 0x1D,
    CMD_ZUPT_CONTROL                                          = 0x1E,
    CMD_EXTERNAL_HEADING_UPDATE_WITH_TIME                     = 0x1F,
    CMD_ANGULAR_ZUPT_CONTROL                                  = 0x20,
    CMD_TARE_ORIENTATION                                      = 0x21,
    CMD_COMMANDED_ZUPT                                        = 0x22,
    CMD_COMMANDED_ANGULAR_ZUPT                                = 0x23,
    CMD_AUTO_HEADING_UPDATE_CONTROL                           = 0x24,
    CMD_MAG_AUTO_CALIBRATION_CONTROL                          = 0x25,
    CMD_MAG_CAPTURE_AUTO_CALIBRATION                          = 0x27,
    CMD_GRAVITY_NOISE                                         = 0x28,
    CMD_PRESSURE_NOISE                                        = 0x29,
    CMD_GRAVITY_NOISE_MINIMUM                                 = 0x2A,
    CMD_HARD_IRON_OFFSET_NOISE                                = 0x2B,
    CMD_SOFT_IRON_MATRIX_NOISE                                = 0x2C,
    CMD_LOW_PASS_SENSOR_FILTER                                = 0x30,
    CMD_MAG_NOISE                                             = 0x42,
    CMD_DECLINATION_SOURCE                                    = 0x43,
    CMD_HOT_START_CONTROL                                     = 0x48,
    CMD_SECONDARY_VELOCITY_AIDING_CONTROL                     = 0x4A,
    CMD_INCLINATION_SOURCE                                    = 0x4C,
    CMD_MAGNETIC_MAGNITUDE_SOURCE                             = 0x4D,
    CMD_SENSOR2VEHICLE_ROTATION_DCM                           = 0x4E,
    CMD_SENSOR2VEHICLE_ROTATION_QUATERNION                    = 0x4F,
    CMD_REFERENCE_POSITION                                    = 0x26,
    CMD_ENABLE_MEASUREMENT                                    = 0x41,
    CMD_ACCEL_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL    = 0x44,
    CMD_MAG_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL      = 0x45,
    CMD_MAG_DIP_ANGLE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL      = 0x46,
    CMD_ALTITUDE_AIDING_CONTROL                               = 0x47,
    CMD_SECONDARY_PITCH_ROLL_AIDING_CONTROL                   = 0x4B,
    CMD_AIDING_MEASUREMENT_ENABLE                             = 0x50,
    CMD_KINEMATIC_CONSTRAINT                                  = 0x51,
    CMD_INITIALIZATION_CONFIGURATION                          = 0x52,
    CMD_ADAPTIVE_FILTER_OPTIONS                               = 0x53,
    CMD_MULTI_ANTENNA_OFFSET                                  = 0x54,
    CMD_REL_POS_CONFIGURATION                                 = 0x55,
    CMD_REF_POINT_LEVER_ARM                                   = 0x56,
    CMD_SPEED_MEASUREMENT                                     = 0x60,
    CMD_SPEED_LEVER_ARM                                       = 0x61,
    CMD_GYRO_CONSTRAINT_CONTROL                               = 0x62,
    CMD_VEHICLE_CONSTRAINT_CONTROL                            = 0x63,
    CMD_ANTENNA_CALIBRATION_CONTROL                           = 0x64,
    CMD_TO_VEHICLE_CALIBRATION_CONTROL                        = 0x65,
    
    REPLY_VEHICLE_DYNAMICS_MODE                               = 0x80,
    REPLY_SENSOR2VEHICLE_ROTATION_EULER                       = 0x81,
    REPLY_SENSOR2VEHICLE_OFFSET                               = 0x82,
    REPLY_ANTENNA_OFFSET                                      = 0x83,
    REPLY_ESTIMATION_CONTROL_FLAGS                            = 0x84,
    REPLY_GNSS_SOURCE_CONTROL                                 = 0x86,
    REPLY_HEADING_UPDATE_CONTROL                              = 0x87,
    REPLY_AUTOINIT_CONTROL                                    = 0x88,
    REPLY_ACCEL_NOISE                                         = 0x89,
    REPLY_GYRO_NOISE                                          = 0x8A,
    REPLY_MAG_NOISE                                           = 0xB1,
    REPLY_ACCEL_BIAS_MODEL                                    = 0x8B,
    REPLY_GYRO_BIAS_MODEL                                     = 0x8C,
    REPLY_ZUPT_CONTROL                                        = 0x8D,
    REPLY_ANGULAR_ZUPT_CONTROL                                = 0x8E,
    REPLY_SELECT_FILTER                                       = 0x8F,
    REPLY_GRAVITY_NOISE                                       = 0x93,
    REPLY_PRESSURE_NOISE                                      = 0x94,
    REPLY_GRAVITY_NOISE_MINIMUM                               = 0x95,
    REPLY_HARD_IRON_OFFSET_NOISE                              = 0x96,
    REPLY_SOFT_IRON_MATRIX_NOISE                              = 0x97,
    REPLY_LOW_PASS_SENSOR_FILTER                              = 0xA0,
    REPLY_SET_INITIAL_HEADING                                 = 0x98,
    REPLY_REFERENCE_POSITION                                  = 0x90,
    REPLY_AUTO_HEADING_UPDATE_CONTROL                         = 0x91,
    REPLY_MAG_AUTO_CALIBRATION_CONTROL                        = 0x92,
    REPLY_ENABLE_MEASUREMENT                                  = 0xB0,
    REPLY_DECLINATION_SOURCE                                  = 0xB2,
    REPLY_ACCEL_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL  = 0xB3,
    REPLY_MAG_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL    = 0xB4,
    REPLY_MAG_DIP_ANGLE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL    = 0xB5,
    REPLY_MAG_ANGULAR_RATE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL = 0xB6,
    REPLY_ALTITUDE_AIDING_CONTROL                             = 0xB7,
    REPLY_HOT_START_CONTROL                                   = 0xB8,
    REPLY_SECONDARY_VELOCITY_AIDING_CONTROL                   = 0xBA,
    REPLY_SECONDARY_PITCH_ROLL_AIDING_CONTROL                 = 0xBB,
    REPLY_INCLINATION_SOURCE                                  = 0xBC,
    REPLY_MAGNETIC_MAGNITUDE_SOURCE                           = 0xBD,
    REPLY_SENSOR2VEHICLE_ROTATION_DCM                         = 0xBE,
    REPLY_SENSOR2VEHICLE_ROTATION_QUATERNION                  = 0xBF,
    REPLY_AIDING_MEASUREMENT_ENABLE                           = 0xD0,
    REPLY_KINEMATIC_CONSTRAINT                                = 0xD1,
    REPLY_INITIALIZATION_CONFIGURATION                        = 0xD2,
    REPLY_ADAPTIVE_FILTER_OPTIONS                             = 0xD3,
    REPLY_MULTI_ANTENNA_OFFSET                                = 0xD4,
    REPLY_REL_POS_CONFIGURATION                               = 0xD5,
    REPLY_SPEED_MEASUREMENT                                   = 0xE0,
    REPLY_GYRO_CONSTRAINT_CONTROL                             = 0xE2,
    REPLY_VEHICLE_CONSTRAINT_CONTROL                          = 0xE3,
    REPLY_ANTENNA_CALIBRATION_CONTROL                         = 0xE4,
    REPLY_TARE_ORIENTATION                                    = 0xA1,
    REPLY_REF_POINT_LEVER_ARM                                 = 0xD6,
    REPLY_SPEED_LEVER_ARM                                     = 0xE1,
};

////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////

enum class FilterReferenceFrame : uint8_t
{
    ECEF = 1,  ///<  WGS84 Earth-fixed, earth centered coordinates
    LLH  = 2,  ///<  WGS84 Latitude, longitude, and height above ellipsoid
};

enum class FilterMagDeclinationSource : uint8_t
{
    NONE   = 1,  ///<  Magnetic field is assumed to have an declination angle equal to zero.
    WMM    = 2,  ///<  Magnetic field is assumed to conform to the World Magnetic Model, calculated using current location estimate as an input to the model.
    MANUAL = 3,  ///<  Magnetic field is assumed to have the declination angle specified by the user.
};

enum class FilterAdaptiveMeasurement : uint8_t
{
    DISABLED = 0,  ///<  No adaptive measurement
    FIXED    = 1,  ///<  Enable fixed adaptive measurement (use specified limits)
    AUTO     = 2,  ///<  Enable auto adaptive measurement
};


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_reset  (0x0D,0x01) Reset [CPP]
/// Resets the filter to the initialization state.
/// 
/// If the auto-initialization feature is disabled, the initial attitude or heading must be set in
/// order to enter the run state after a reset.
///
///@{

struct Reset
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_RESET_FILTER;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    
};
void insert(Serializer& serializer, const Reset& self);
void extract(Serializer& serializer, Reset& self);

CmdResult reset(C::mip_interface& device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_set_initial_attitude  (0x0D,0x02) Set Initial Attitude [CPP]
/// Set the sensor initial attitude.
/// 
/// This command can only be issued in the "Init" state and should be used with a good
/// estimate of the vehicle attitude.  The Euler angles are the sensor body frame with respect
/// to the NED frame.
/// 
/// The valid input ranges are as follows:
/// 
/// Roll:    [-pi, pi]
/// Pitch:   [-pi/2, pi/2]
/// Heading: [-pi, pi]
/// 
///
///@{

struct SetInitialAttitude
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_SET_INITIAL_ATTITUDE;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    float roll = 0; ///< [radians]
    float pitch = 0; ///< [radians]
    float heading = 0; ///< [radians]
    
};
void insert(Serializer& serializer, const SetInitialAttitude& self);
void extract(Serializer& serializer, SetInitialAttitude& self);

CmdResult setInitialAttitude(C::mip_interface& device, float roll, float pitch, float heading);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_estimation_control  (0x0D,0x14) Estimation Control [CPP]
/// Estimation Control Flags
/// 
/// Controls which parameters are estimated by the Kalman Filter.
/// 
/// Desired settings should be logically ORed together.
/// 
/// Examples:
/// 
/// 0x0001 - Enable Gyro Bias Estimation Only
/// 0x0063 - Enable Gyro Bias, Accel Bias, and Mag Auto Hard and Soft Iron Cal States Only
/// 
///
///@{

struct EstimationControl
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_ESTIMATION_CONTROL_FLAGS;
    
    static const bool HAS_WRITE_FUNCTION = true;
    static const bool HAS_READ_FUNCTION = true;
    static const bool HAS_SAVE_FUNCTION = true;
    static const bool HAS_LOAD_FUNCTION = true;
    static const bool HAS_RESET_FUNCTION = true;
    
    struct EnableFlags : Bitfield<EnableFlags>
    {
        enum _enumType : uint16_t
        {
            NONE               = 0x0000,
            GYRO_BIAS          = 0x0001,  ///<  
            ACCEL_BIAS         = 0x0002,  ///<  
            GYRO_SCALE_FACTOR  = 0x0004,  ///<  
            ACCEL_SCALE_FACTOR = 0x0008,  ///<  
            ANTENNA_OFFSET     = 0x0010,  ///<  
            AUTO_MAG_HARD_IRON = 0x0020,  ///<  
            AUTO_MAG_SOFT_IRON = 0x0040,  ///<  
            ALL                = 0x007F,
        };
        uint16_t value = NONE;
        
        EnableFlags() : value(NONE) {}
        EnableFlags(int val) : value((uint16_t)val) {}
        operator uint16_t() const { return value; }
        EnableFlags& operator=(uint16_t val) { value = val; return *this; }
        EnableFlags& operator=(int val) { value = val; return *this; }
        EnableFlags& operator|=(uint16_t val) { return *this = value | val; }
        EnableFlags& operator&=(uint16_t val) { return *this = value & val; }
        
        bool gyroBias() const { return (value & GYRO_BIAS) > 0; }
        void gyroBias(bool val) { if(val) value |= GYRO_BIAS; else value &= ~GYRO_BIAS; }
        bool accelBias() const { return (value & ACCEL_BIAS) > 0; }
        void accelBias(bool val) { if(val) value |= ACCEL_BIAS; else value &= ~ACCEL_BIAS; }
        bool gyroScaleFactor() const { return (value & GYRO_SCALE_FACTOR) > 0; }
        void gyroScaleFactor(bool val) { if(val) value |= GYRO_SCALE_FACTOR; else value &= ~GYRO_SCALE_FACTOR; }
        bool accelScaleFactor() const { return (value & ACCEL_SCALE_FACTOR) > 0; }
        void accelScaleFactor(bool val) { if(val) value |= ACCEL_SCALE_FACTOR; else value &= ~ACCEL_SCALE_FACTOR; }
        bool antennaOffset() const { return (value & ANTENNA_OFFSET) > 0; }
        void antennaOffset(bool val) { if(val) value |= ANTENNA_OFFSET; else value &= ~ANTENNA_OFFSET; }
        bool autoMagHardIron() const { return (value & AUTO_MAG_HARD_IRON) > 0; }
        void autoMagHardIron(bool val) { if(val) value |= AUTO_MAG_HARD_IRON; else value &= ~AUTO_MAG_HARD_IRON; }
        bool autoMagSoftIron() const { return (value & AUTO_MAG_SOFT_IRON) > 0; }
        void autoMagSoftIron(bool val) { if(val) value |= AUTO_MAG_SOFT_IRON; else value &= ~AUTO_MAG_SOFT_IRON; }
        
        bool allSet() const { return value == ALL; }
        void setAll() { value |= ALL; }
    };
    
    FunctionSelector function = static_cast<FunctionSelector>(0);
    EnableFlags enable; ///< See above
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_ESTIMATION_CONTROL_FLAGS;
        
        EnableFlags enable; ///< See above
        
    };
};
void insert(Serializer& serializer, const EstimationControl& self);
void extract(Serializer& serializer, EstimationControl& self);

void insert(Serializer& serializer, const EstimationControl::Response& self);
void extract(Serializer& serializer, EstimationControl::Response& self);

CmdResult writeEstimationControl(C::mip_interface& device, EstimationControl::EnableFlags enable);
CmdResult readEstimationControl(C::mip_interface& device, EstimationControl::EnableFlags* enableOut);
CmdResult saveEstimationControl(C::mip_interface& device);
CmdResult loadEstimationControl(C::mip_interface& device);
CmdResult defaultEstimationControl(C::mip_interface& device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_external_gnss_update  (0x0D,0x16) External Gnss Update [CPP]
/// Provide a filter measurement from an external GNSS
/// 
/// The GNSS source control must be set to "external" for this command to succeed, otherwise it will be NACK'd.
/// Please refer to your device user manual for information on the maximum rate of this message.
/// 
///
///@{

struct ExternalGnssUpdate
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_EXTERNAL_GNSS_UPDATE;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    double gps_time = 0; ///< [seconds]
    uint16_t gps_week = 0; ///< [GPS week number, not modulus 1024]
    double latitude = 0; ///< [degrees]
    double longitude = 0; ///< [degrees]
    double height = 0; ///< Above WGS84 ellipsoid [meters]
    float velocity[3] = {0}; ///< NED Frame [meters/second]
    float pos_uncertainty[3] = {0}; ///< NED Frame, 1-sigma [meters]
    float vel_uncertainty[3] = {0}; ///< NED Frame, 1-sigma [meters/second]
    
};
void insert(Serializer& serializer, const ExternalGnssUpdate& self);
void extract(Serializer& serializer, ExternalGnssUpdate& self);

CmdResult externalGnssUpdate(C::mip_interface& device, double gpsTime, uint16_t gpsWeek, double latitude, double longitude, double height, const float* velocity, const float* posUncertainty, const float* velUncertainty);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_external_heading_update  (0x0D,0x17) External Heading Update [CPP]
/// Provide a filter measurement from an external heading source
/// 
/// The heading must be the sensor frame with respect to the NED frame.
/// 
/// The heading update control must be set to external for this command to update the filter; otherwise it is NACK'd.
/// Heading angle uncertainties of &lt;= 0.0 will be NACK'd
/// 
/// Please refer to your device user manual for information on the maximum rate of this message.
/// 
/// On -25 models, if the declination source (0x0D, 0x43) is not valid, true heading updates will be NACK'd.
/// On -45 models, if the declination source is invalid, magnetic heading updates will be NACK'd.
/// 
/// 
///
///@{

struct ExternalHeadingUpdate
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_EXTERNAL_HEADING_UPDATE;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    float heading = 0; ///< Bounded by +-PI [radians]
    float heading_uncertainty = 0; ///< 1-sigma [radians]
    uint8_t type = 0; ///< 1 - True, 2 - Magnetic
    
};
void insert(Serializer& serializer, const ExternalHeadingUpdate& self);
void extract(Serializer& serializer, ExternalHeadingUpdate& self);

CmdResult externalHeadingUpdate(C::mip_interface& device, float heading, float headingUncertainty, uint8_t type);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_external_heading_update_with_time  (0x0D,0x1F) External Heading Update With Time [CPP]
/// Provide a filter measurement from an external heading source at a specific GPS time
/// 
/// This is more accurate than the External Heading Update (0x0D, 0x17) and should be used in applications
/// where the rate of heading change will cause significant measurement error due to the sampling, transmission,
/// and processing time required.  Accurate time stamping of the heading information is important.
/// 
/// The heading must be the sensor frame with respect to the NED frame.
/// 
/// The heading update control must be set to external for this command to update the filter; otherwise it is NACK'd.
/// Heading angle uncertainties of &lt;= 0.0 will be NACK'd
/// 
/// Please refer to your device user manual for information on the maximum rate of this message.
/// 
/// On -25 models, if the declination source (0x0D, 0x43) is not valid, true heading updates will be NACK'd.
/// On -45 models, if the declination source is invalid, magnetic heading updates will be NACK'd.
/// 
/// 
///
///@{

struct ExternalHeadingUpdateWithTime
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_EXTERNAL_HEADING_UPDATE_WITH_TIME;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    double gps_time = 0; ///< [seconds]
    uint16_t gps_week = 0; ///< [GPS week number, not modulus 1024]
    float heading = 0; ///< Relative to true north, bounded by +-PI [radians]
    float heading_uncertainty = 0; ///< 1-sigma [radians]
    uint8_t type = 0; ///< 1 - True, 2 - Magnetic
    
};
void insert(Serializer& serializer, const ExternalHeadingUpdateWithTime& self);
void extract(Serializer& serializer, ExternalHeadingUpdateWithTime& self);

CmdResult externalHeadingUpdateWithTime(C::mip_interface& device, double gpsTime, uint16_t gpsWeek, float heading, float headingUncertainty, uint8_t type);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_tare_orientation  (0x0D,0x21) Tare Orientation [CPP]
/// Tare the device orientation.
/// 
/// This function uses the current device orientation relative to the NED frame as the current sensor to vehicle transformation.
/// This command is provided as a convenient way to set the sensor to vehicle frame transformation.
/// The filter must be initialized and have a valid attitude output. If the attitude is not valid, an error will be returned.
///
///@{

struct TareOrientation
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_TARE_ORIENTATION;
    
    static const bool HAS_WRITE_FUNCTION = true;
    static const bool HAS_READ_FUNCTION = true;
    static const bool HAS_SAVE_FUNCTION = true;
    static const bool HAS_LOAD_FUNCTION = true;
    static const bool HAS_RESET_FUNCTION = true;
    
    struct MipTareAxes : Bitfield<MipTareAxes>
    {
        enum _enumType : uint8_t
        {
            NONE  = 0x0,
            ROLL  = 0x1,  ///<  
            PITCH = 0x2,  ///<  
            YAW   = 0x4,  ///<  
            ALL   = 0x7,
        };
        uint8_t value = NONE;
        
        MipTareAxes() : value(NONE) {}
        MipTareAxes(int val) : value((uint8_t)val) {}
        operator uint8_t() const { return value; }
        MipTareAxes& operator=(uint8_t val) { value = val; return *this; }
        MipTareAxes& operator=(int val) { value = val; return *this; }
        MipTareAxes& operator|=(uint8_t val) { return *this = value | val; }
        MipTareAxes& operator&=(uint8_t val) { return *this = value & val; }
        
        bool roll() const { return (value & ROLL) > 0; }
        void roll(bool val) { if(val) value |= ROLL; else value &= ~ROLL; }
        bool pitch() const { return (value & PITCH) > 0; }
        void pitch(bool val) { if(val) value |= PITCH; else value &= ~PITCH; }
        bool yaw() const { return (value & YAW) > 0; }
        void yaw(bool val) { if(val) value |= YAW; else value &= ~YAW; }
        
        bool allSet() const { return value == ALL; }
        void setAll() { value |= ALL; }
    };
    
    FunctionSelector function = static_cast<FunctionSelector>(0);
    MipTareAxes axes; ///< Axes to tare
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_TARE_ORIENTATION;
        
        MipTareAxes axes; ///< Axes to tare
        
    };
};
void insert(Serializer& serializer, const TareOrientation& self);
void extract(Serializer& serializer, TareOrientation& self);

void insert(Serializer& serializer, const TareOrientation::Response& self);
void extract(Serializer& serializer, TareOrientation::Response& self);

CmdResult writeTareOrientation(C::mip_interface& device, TareOrientation::MipTareAxes axes);
CmdResult readTareOrientation(C::mip_interface& device, TareOrientation::MipTareAxes* axesOut);
CmdResult saveTareOrientation(C::mip_interface& device);
CmdResult loadTareOrientation(C::mip_interface& device);
CmdResult defaultTareOrientation(C::mip_interface& device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_vehicle_dynamics_mode  (0x0D,0x10) Vehicle Dynamics Mode [CPP]
/// Controls the vehicle dynamics mode.
///
///@{

struct VehicleDynamicsMode
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_VEHICLE_DYNAMICS_MODE;
    
    static const bool HAS_WRITE_FUNCTION = true;
    static const bool HAS_READ_FUNCTION = true;
    static const bool HAS_SAVE_FUNCTION = true;
    static const bool HAS_LOAD_FUNCTION = true;
    static const bool HAS_RESET_FUNCTION = true;
    
    enum class DynamicsMode : uint8_t
    {
        PORTABLE        = 1,  ///<  
        AUTOMOTIVE      = 2,  ///<  
        AIRBORNE        = 3,  ///<  
        AIRBORNE_HIGH_G = 4,  ///<  
    };
    
    FunctionSelector function = static_cast<FunctionSelector>(0);
    DynamicsMode mode = static_cast<DynamicsMode>(0);
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_VEHICLE_DYNAMICS_MODE;
        
        DynamicsMode mode = static_cast<DynamicsMode>(0);
        
    };
};
void insert(Serializer& serializer, const VehicleDynamicsMode& self);
void extract(Serializer& serializer, VehicleDynamicsMode& self);

void insert(Serializer& serializer, const VehicleDynamicsMode::Response& self);
void extract(Serializer& serializer, VehicleDynamicsMode::Response& self);

CmdResult writeVehicleDynamicsMode(C::mip_interface& device, VehicleDynamicsMode::DynamicsMode mode);
CmdResult readVehicleDynamicsMode(C::mip_interface& device, VehicleDynamicsMode::DynamicsMode* modeOut);
CmdResult saveVehicleDynamicsMode(C::mip_interface& device);
CmdResult loadVehicleDynamicsMode(C::mip_interface& device);
CmdResult defaultVehicleDynamicsMode(C::mip_interface& device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_sensor_to_vehicle_rotation_euler  (0x0D,0x11) Sensor To Vehicle Rotation Euler [CPP]
/// Set the sensor to vehicle frame rotation using Yaw, Pitch, Roll Euler angles.
/// 
/// Note: This is the rotation, the inverse of the transformation.
/// These angles define the rotation from the sensor body frame to the fixed vehicle frame.<br/>
/// Please reference the device Theory of Operation for more information.<br/>
/// The rotation is stored in the device as a quaternion.  When Euler angles are read back from the device, they may not
/// be equivalent in value to the Euler angles used to set the rotation, but they are functionally equivalent.<br/>
/// <br/><br/>
/// This rotation affects the following output quantities:<br/><br/>
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
///
///@{

struct SensorToVehicleRotationEuler
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_SENSOR2VEHICLE_ROTATION_EULER;
    
    static const bool HAS_WRITE_FUNCTION = true;
    static const bool HAS_READ_FUNCTION = true;
    static const bool HAS_SAVE_FUNCTION = true;
    static const bool HAS_LOAD_FUNCTION = true;
    static const bool HAS_RESET_FUNCTION = true;
    
    FunctionSelector function = static_cast<FunctionSelector>(0);
    float roll = 0; ///< [radians]
    float pitch = 0; ///< [radians]
    float yaw = 0; ///< [radians]
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_SENSOR2VEHICLE_ROTATION_EULER;
        
        float roll = 0; ///< [radians]
        float pitch = 0; ///< [radians]
        float yaw = 0; ///< [radians]
        
    };
};
void insert(Serializer& serializer, const SensorToVehicleRotationEuler& self);
void extract(Serializer& serializer, SensorToVehicleRotationEuler& self);

void insert(Serializer& serializer, const SensorToVehicleRotationEuler::Response& self);
void extract(Serializer& serializer, SensorToVehicleRotationEuler::Response& self);

CmdResult writeSensorToVehicleRotationEuler(C::mip_interface& device, float roll, float pitch, float yaw);
CmdResult readSensorToVehicleRotationEuler(C::mip_interface& device, float* rollOut, float* pitchOut, float* yawOut);
CmdResult saveSensorToVehicleRotationEuler(C::mip_interface& device);
CmdResult loadSensorToVehicleRotationEuler(C::mip_interface& device);
CmdResult defaultSensorToVehicleRotationEuler(C::mip_interface& device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_sensor_to_vehicle_rotation_dcm  (0x0D,0x4E) Sensor To Vehicle Rotation Dcm [CPP]
/// Set the sensor to vehicle frame rotation using a row-major direction cosine matrix.
/// 
/// Note: This is the rotation, the inverse of the transformation.
/// This matrix defines the rotation from the sensor body frame to the fixed vehicle frame.<br/>
/// Please reference the device Theory of Operation for more information.<br/>
/// The matrix must be orthonormal (tolerance 1e-3) or the device will NACK the command.
/// The rotation is stored in the device as a quaternion.  When the DCM is read back from the device, the components may not
/// be exactly equivalent in value to the DCM used to set the rotation, but they are functionally equivalent.<br/>
/// <br/>
/// Matrix element order:<br/><br/>
/// 
/// EQSTART T_{SEN}^{VEH} = \begin{bmatrix} 0 &amp; 1 &amp; 2\\  3 &amp; 4 &amp; 5\\ 6 &amp; 7 &amp; 8 \end{bmatrix} EQEND
/// 
/// <br/><br/>
/// This rotation affects the following output quantities:<br/><br/>
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
///
///@{

struct SensorToVehicleRotationDcm
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_SENSOR2VEHICLE_ROTATION_DCM;
    
    static const bool HAS_WRITE_FUNCTION = true;
    static const bool HAS_READ_FUNCTION = true;
    static const bool HAS_SAVE_FUNCTION = true;
    static const bool HAS_LOAD_FUNCTION = true;
    static const bool HAS_RESET_FUNCTION = true;
    
    FunctionSelector function = static_cast<FunctionSelector>(0);
    float dcm[9] = {0};
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_SENSOR2VEHICLE_ROTATION_DCM;
        
        float dcm[9] = {0};
        
    };
};
void insert(Serializer& serializer, const SensorToVehicleRotationDcm& self);
void extract(Serializer& serializer, SensorToVehicleRotationDcm& self);

void insert(Serializer& serializer, const SensorToVehicleRotationDcm::Response& self);
void extract(Serializer& serializer, SensorToVehicleRotationDcm::Response& self);

CmdResult writeSensorToVehicleRotationDcm(C::mip_interface& device, const float* dcm);
CmdResult readSensorToVehicleRotationDcm(C::mip_interface& device, float* dcmOut);
CmdResult saveSensorToVehicleRotationDcm(C::mip_interface& device);
CmdResult loadSensorToVehicleRotationDcm(C::mip_interface& device);
CmdResult defaultSensorToVehicleRotationDcm(C::mip_interface& device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_sensor_to_vehicle_rotation_quaternion  (0x0D,0x4F) Sensor To Vehicle Rotation Quaternion [CPP]
/// Set the sensor to vehicle frame rotation using a quaternion.
/// 
/// Note: This is the rotation, the inverse of the transformation.
/// This quaternion defines the rotation from the sensor body frame to the fixed vehicle frame.<br/>
/// Please reference the device Theory of Operation for more information.<br/>
/// The quaternion must be unit length (tolerance 1e-3) or the device will NACK the command.
/// The rotation is stored in the device as a unit quaternion.  When the quaternion elements are read back from the device, they may not
/// be equivalent in value to the quaternion used to set the rotation, due to normalization.<br/>
/// <br/>
/// Quaternion element definition:<br/><br/>
/// <br/>
/// EQSTART Q_{SEN}^{VEH} = \begin{bmatrix} q_{0} &amp; q_{1}*i  &amp; q_{2}*j  &amp; q_{3}*k \end{bmatrix} EQEND
/// <br/><br/>
/// This rotation affects the following output quantities:<br/><br/>
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
///
///@{

struct SensorToVehicleRotationQuaternion
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_SENSOR2VEHICLE_ROTATION_QUATERNION;
    
    static const bool HAS_WRITE_FUNCTION = true;
    static const bool HAS_READ_FUNCTION = true;
    static const bool HAS_SAVE_FUNCTION = true;
    static const bool HAS_LOAD_FUNCTION = true;
    static const bool HAS_RESET_FUNCTION = true;
    
    FunctionSelector function = static_cast<FunctionSelector>(0);
    float quat[4] = {0};
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_SENSOR2VEHICLE_ROTATION_QUATERNION;
        
        float quat[4] = {0};
        
    };
};
void insert(Serializer& serializer, const SensorToVehicleRotationQuaternion& self);
void extract(Serializer& serializer, SensorToVehicleRotationQuaternion& self);

void insert(Serializer& serializer, const SensorToVehicleRotationQuaternion::Response& self);
void extract(Serializer& serializer, SensorToVehicleRotationQuaternion::Response& self);

CmdResult writeSensorToVehicleRotationQuaternion(C::mip_interface& device, const float* quat);
CmdResult readSensorToVehicleRotationQuaternion(C::mip_interface& device, float* quatOut);
CmdResult saveSensorToVehicleRotationQuaternion(C::mip_interface& device);
CmdResult loadSensorToVehicleRotationQuaternion(C::mip_interface& device);
CmdResult defaultSensorToVehicleRotationQuaternion(C::mip_interface& device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_sensor_to_vehicle_offset  (0x0D,0x12) Sensor To Vehicle Offset [CPP]
/// Set the sensor to vehicle frame offset, expressed in the sensor frame.
/// 
/// This is a simple offset, not a lever arm.  It does not compensate for inertial effects experienced from being offset from the center of gravity/rotation of the vehicle.
/// It simply adds the offset to the position output to express it in the origin of the user's vehicle frame.
/// 
/// This offset affects the following output quantities:
/// Estimated LLH Position
/// 
/// The magnitude of the offset vector is limited to 10 meters
///
///@{

struct SensorToVehicleOffset
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_SENSOR2VEHICLE_OFFSET;
    
    static const bool HAS_WRITE_FUNCTION = true;
    static const bool HAS_READ_FUNCTION = true;
    static const bool HAS_SAVE_FUNCTION = true;
    static const bool HAS_LOAD_FUNCTION = true;
    static const bool HAS_RESET_FUNCTION = true;
    
    FunctionSelector function = static_cast<FunctionSelector>(0);
    float offset[3] = {0}; ///< [meters]
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_SENSOR2VEHICLE_OFFSET;
        
        float offset[3] = {0}; ///< [meters]
        
    };
};
void insert(Serializer& serializer, const SensorToVehicleOffset& self);
void extract(Serializer& serializer, SensorToVehicleOffset& self);

void insert(Serializer& serializer, const SensorToVehicleOffset::Response& self);
void extract(Serializer& serializer, SensorToVehicleOffset::Response& self);

CmdResult writeSensorToVehicleOffset(C::mip_interface& device, const float* offset);
CmdResult readSensorToVehicleOffset(C::mip_interface& device, float* offsetOut);
CmdResult saveSensorToVehicleOffset(C::mip_interface& device);
CmdResult loadSensorToVehicleOffset(C::mip_interface& device);
CmdResult defaultSensorToVehicleOffset(C::mip_interface& device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_antenna_offset  (0x0D,0x13) Antenna Offset [CPP]
/// Set the sensor to GNSS antenna offset.
/// 
/// This is expressed in the sensor frame, from the sensor origin to the GNSS antenna RF center.
/// 
/// The magnitude of the offset vector is limited to 10 meters
/// 
///
///@{

struct AntennaOffset
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_ANTENNA_OFFSET;
    
    static const bool HAS_WRITE_FUNCTION = true;
    static const bool HAS_READ_FUNCTION = true;
    static const bool HAS_SAVE_FUNCTION = true;
    static const bool HAS_LOAD_FUNCTION = true;
    static const bool HAS_RESET_FUNCTION = true;
    
    FunctionSelector function = static_cast<FunctionSelector>(0);
    float offset[3] = {0}; ///< [meters]
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_ANTENNA_OFFSET;
        
        float offset[3] = {0}; ///< [meters]
        
    };
};
void insert(Serializer& serializer, const AntennaOffset& self);
void extract(Serializer& serializer, AntennaOffset& self);

void insert(Serializer& serializer, const AntennaOffset::Response& self);
void extract(Serializer& serializer, AntennaOffset::Response& self);

CmdResult writeAntennaOffset(C::mip_interface& device, const float* offset);
CmdResult readAntennaOffset(C::mip_interface& device, float* offsetOut);
CmdResult saveAntennaOffset(C::mip_interface& device);
CmdResult loadAntennaOffset(C::mip_interface& device);
CmdResult defaultAntennaOffset(C::mip_interface& device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_gnss_source  (0x0D,0x15) Gnss Source [CPP]
/// Control the source of GNSS information used to update the Kalman Filter.
/// 
/// Changing the GNSS source while the sensor is in the "running" state may temporarily place
/// it back in the "init" state until the new source of GNSS data is received.
/// 
///
///@{

struct GnssSource
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_GNSS_SOURCE_CONTROL;
    
    static const bool HAS_WRITE_FUNCTION = true;
    static const bool HAS_READ_FUNCTION = true;
    static const bool HAS_SAVE_FUNCTION = true;
    static const bool HAS_LOAD_FUNCTION = true;
    static const bool HAS_RESET_FUNCTION = true;
    
    enum class Source : uint8_t
    {
        ALL_INT = 1,  ///<  All internal receivers
        EXT     = 2,  ///<  External GNSS messages provided by user
        INT_1   = 3,  ///<  Internal GNSS Receiver 1 only
        INT_2   = 4,  ///<  Internal GNSS Receiver 2 only
    };
    
    FunctionSelector function = static_cast<FunctionSelector>(0);
    Source source = static_cast<Source>(0);
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_GNSS_SOURCE_CONTROL;
        
        Source source = static_cast<Source>(0);
        
    };
};
void insert(Serializer& serializer, const GnssSource& self);
void extract(Serializer& serializer, GnssSource& self);

void insert(Serializer& serializer, const GnssSource::Response& self);
void extract(Serializer& serializer, GnssSource::Response& self);

CmdResult writeGnssSource(C::mip_interface& device, GnssSource::Source source);
CmdResult readGnssSource(C::mip_interface& device, GnssSource::Source* sourceOut);
CmdResult saveGnssSource(C::mip_interface& device);
CmdResult loadGnssSource(C::mip_interface& device);
CmdResult defaultGnssSource(C::mip_interface& device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_heading_source  (0x0D,0x18) Heading Source [CPP]
/// Control the source of heading information used to update the Kalman Filter.
/// 
/// 1. To use internal GNSS velocity vector for heading updates, the target application
/// must have minimal (preferably no) side-slip.  This option is good for wheeled vehicles.
/// 
/// 2. On some devices, when using GNSS velocity vector for heading updates, the X-axis of the device
/// must align with the direction of travel.  Please reference the user guide for your particular device to
/// determine if this limitation is applicable.
/// 
/// 3. When none is selected, the heading estimate can still converge if GNSS is available and sufficient dynamic motion
/// (change in direction of travel and acceleration) is experienced.  The heading may drift when: stationary, traveling
/// at a constant speed, or during a constant course over ground.
///
///@{

struct HeadingSource
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_HEADING_UPDATE_CONTROL;
    
    static const bool HAS_WRITE_FUNCTION = true;
    static const bool HAS_READ_FUNCTION = true;
    static const bool HAS_SAVE_FUNCTION = true;
    static const bool HAS_LOAD_FUNCTION = true;
    static const bool HAS_RESET_FUNCTION = true;
    
    enum class Source : uint8_t
    {
        NONE                          = 0,  ///<  See note 3
        MAG                           = 1,  ///<  
        GNSS_VEL                      = 2,  ///<  See notes 1,2
        EXTERNAL                      = 3,  ///<  
        GNSS_VEL_AND_MAG              = 4,  ///<  
        GNSS_VEL_AND_EXTERNAL         = 5,  ///<  
        MAG_AND_EXTERNAL              = 6,  ///<  
        GNSS_VEL_AND_MAG_AND_EXTERNAL = 7,  ///<  
    };
    
    FunctionSelector function = static_cast<FunctionSelector>(0);
    Source source = static_cast<Source>(0);
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_HEADING_UPDATE_CONTROL;
        
        Source source = static_cast<Source>(0);
        
    };
};
void insert(Serializer& serializer, const HeadingSource& self);
void extract(Serializer& serializer, HeadingSource& self);

void insert(Serializer& serializer, const HeadingSource::Response& self);
void extract(Serializer& serializer, HeadingSource::Response& self);

CmdResult writeHeadingSource(C::mip_interface& device, HeadingSource::Source source);
CmdResult readHeadingSource(C::mip_interface& device, HeadingSource::Source* sourceOut);
CmdResult saveHeadingSource(C::mip_interface& device);
CmdResult loadHeadingSource(C::mip_interface& device);
CmdResult defaultHeadingSource(C::mip_interface& device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_auto_init_control  (0x0D,0x19) Auto Init Control [CPP]
/// Filter Auto-initialization Control
/// 
/// Enable/Disable automatic initialization upon device startup.
/// 
/// Possible enable values:
/// 
/// 0x00 - Disable auto-initialization
/// 0x01 - Enable auto-initialization
/// 
///
///@{

struct AutoInitControl
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_AUTOINIT_CONTROL;
    
    static const bool HAS_WRITE_FUNCTION = true;
    static const bool HAS_READ_FUNCTION = true;
    static const bool HAS_SAVE_FUNCTION = true;
    static const bool HAS_LOAD_FUNCTION = true;
    static const bool HAS_RESET_FUNCTION = true;
    
    FunctionSelector function = static_cast<FunctionSelector>(0);
    uint8_t enable = 0; ///< See above
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_AUTOINIT_CONTROL;
        
        uint8_t enable = 0; ///< See above
        
    };
};
void insert(Serializer& serializer, const AutoInitControl& self);
void extract(Serializer& serializer, AutoInitControl& self);

void insert(Serializer& serializer, const AutoInitControl::Response& self);
void extract(Serializer& serializer, AutoInitControl::Response& self);

CmdResult writeAutoInitControl(C::mip_interface& device, uint8_t enable);
CmdResult readAutoInitControl(C::mip_interface& device, uint8_t* enableOut);
CmdResult saveAutoInitControl(C::mip_interface& device);
CmdResult loadAutoInitControl(C::mip_interface& device);
CmdResult defaultAutoInitControl(C::mip_interface& device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_accel_noise  (0x0D,0x1A) Accel Noise [CPP]
/// Accelerometer Noise Standard Deviation
/// 
/// Each of the noise values must be greater than 0.0.
/// 
/// The noise value represents process noise in the 3DM-GX5-45 NAV Estimation Filter.
/// Changing this value modifies how the filter responds to dynamic input and can be used to tune the performance of the filter.
/// Default values provide good performance for most laboratory conditions.
/// 
///
///@{

struct AccelNoise
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_ACCEL_NOISE;
    
    static const bool HAS_WRITE_FUNCTION = true;
    static const bool HAS_READ_FUNCTION = true;
    static const bool HAS_SAVE_FUNCTION = true;
    static const bool HAS_LOAD_FUNCTION = true;
    static const bool HAS_RESET_FUNCTION = true;
    
    FunctionSelector function = static_cast<FunctionSelector>(0);
    float x = 0; ///< Accel Noise 1-sigma [meters/second^2]
    float y = 0; ///< Accel Noise 1-sigma [meters/second^2]
    float z = 0; ///< Accel Noise 1-sigma [meters/second^2]
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_ACCEL_NOISE;
        
        float x = 0; ///< Accel Noise 1-sigma [meters/second^2]
        float y = 0; ///< Accel Noise 1-sigma [meters/second^2]
        float z = 0; ///< Accel Noise 1-sigma [meters/second^2]
        
    };
};
void insert(Serializer& serializer, const AccelNoise& self);
void extract(Serializer& serializer, AccelNoise& self);

void insert(Serializer& serializer, const AccelNoise::Response& self);
void extract(Serializer& serializer, AccelNoise::Response& self);

CmdResult writeAccelNoise(C::mip_interface& device, float x, float y, float z);
CmdResult readAccelNoise(C::mip_interface& device, float* xOut, float* yOut, float* zOut);
CmdResult saveAccelNoise(C::mip_interface& device);
CmdResult loadAccelNoise(C::mip_interface& device);
CmdResult defaultAccelNoise(C::mip_interface& device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_gyro_noise  (0x0D,0x1B) Gyro Noise [CPP]
/// Gyroscope Noise Standard Deviation
/// 
/// Each of the noise values must be greater than 0.0
/// 
/// The noise value represents process noise in the 3DM-GX5-45 NAV Estimation Filter.
/// Changing this value modifies how the filter responds to dynamic input and can be used to tune the performance of the filter.
/// Default values provide good performance for most laboratory conditions.
/// 
///
///@{

struct GyroNoise
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_GYRO_NOISE;
    
    static const bool HAS_WRITE_FUNCTION = true;
    static const bool HAS_READ_FUNCTION = true;
    static const bool HAS_SAVE_FUNCTION = true;
    static const bool HAS_LOAD_FUNCTION = true;
    static const bool HAS_RESET_FUNCTION = true;
    
    FunctionSelector function = static_cast<FunctionSelector>(0);
    float x = 0; ///< Gyro Noise 1-sigma [meters/second^2]
    float y = 0; ///< Gyro Noise 1-sigma [meters/second^2]
    float z = 0; ///< Gyro Noise 1-sigma [meters/second^2]
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_GYRO_NOISE;
        
        float x = 0; ///< Gyro Noise 1-sigma [meters/second^2]
        float y = 0; ///< Gyro Noise 1-sigma [meters/second^2]
        float z = 0; ///< Gyro Noise 1-sigma [meters/second^2]
        
    };
};
void insert(Serializer& serializer, const GyroNoise& self);
void extract(Serializer& serializer, GyroNoise& self);

void insert(Serializer& serializer, const GyroNoise::Response& self);
void extract(Serializer& serializer, GyroNoise::Response& self);

CmdResult writeGyroNoise(C::mip_interface& device, float x, float y, float z);
CmdResult readGyroNoise(C::mip_interface& device, float* xOut, float* yOut, float* zOut);
CmdResult saveGyroNoise(C::mip_interface& device);
CmdResult loadGyroNoise(C::mip_interface& device);
CmdResult defaultGyroNoise(C::mip_interface& device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_accel_bias_model  (0x0D,0x1C) Accel Bias Model [CPP]
/// Accelerometer Bias Model Parameters
/// 
/// Each of the noise values must be greater than 0.0
/// 
///
///@{

struct AccelBiasModel
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_ACCEL_BIAS_MODEL;
    
    static const bool HAS_WRITE_FUNCTION = true;
    static const bool HAS_READ_FUNCTION = true;
    static const bool HAS_SAVE_FUNCTION = true;
    static const bool HAS_LOAD_FUNCTION = true;
    static const bool HAS_RESET_FUNCTION = true;
    
    FunctionSelector function = static_cast<FunctionSelector>(0);
    float x_beta = 0; ///< Accel Bias Beta [1/second]
    float y_beta = 0; ///< Accel Bias Beta [1/second]
    float z_beta = 0; ///< Accel Bias Beta [1/second]
    float x = 0; ///< Accel Noise 1-sigma [meters/second^2]
    float y = 0; ///< Accel Noise 1-sigma [meters/second^2]
    float z = 0; ///< Accel Noise 1-sigma [meters/second^2]
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_ACCEL_BIAS_MODEL;
        
        float x_beta = 0; ///< Accel Bias Beta [1/second]
        float y_beta = 0; ///< Accel Bias Beta [1/second]
        float z_beta = 0; ///< Accel Bias Beta [1/second]
        float x = 0; ///< Accel Noise 1-sigma [meters/second^2]
        float y = 0; ///< Accel Noise 1-sigma [meters/second^2]
        float z = 0; ///< Accel Noise 1-sigma [meters/second^2]
        
    };
};
void insert(Serializer& serializer, const AccelBiasModel& self);
void extract(Serializer& serializer, AccelBiasModel& self);

void insert(Serializer& serializer, const AccelBiasModel::Response& self);
void extract(Serializer& serializer, AccelBiasModel::Response& self);

CmdResult writeAccelBiasModel(C::mip_interface& device, float xBeta, float yBeta, float zBeta, float x, float y, float z);
CmdResult readAccelBiasModel(C::mip_interface& device, float* xBetaOut, float* yBetaOut, float* zBetaOut, float* xOut, float* yOut, float* zOut);
CmdResult saveAccelBiasModel(C::mip_interface& device);
CmdResult loadAccelBiasModel(C::mip_interface& device);
CmdResult defaultAccelBiasModel(C::mip_interface& device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_gyro_bias_model  (0x0D,0x1D) Gyro Bias Model [CPP]
/// Gyroscope Bias Model Parameters
/// 
/// Each of the noise values must be greater than 0.0
/// 
///
///@{

struct GyroBiasModel
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_GYRO_BIAS_MODEL;
    
    static const bool HAS_WRITE_FUNCTION = true;
    static const bool HAS_READ_FUNCTION = true;
    static const bool HAS_SAVE_FUNCTION = true;
    static const bool HAS_LOAD_FUNCTION = true;
    static const bool HAS_RESET_FUNCTION = true;
    
    FunctionSelector function = static_cast<FunctionSelector>(0);
    float x_beta = 0; ///< Gyro Bias Beta [1/second]
    float y_beta = 0; ///< Gyro Bias Beta [1/second]
    float z_beta = 0; ///< Gyro Bias Beta [1/second]
    float x = 0; ///< Gyro Noise 1-sigma [meters/second^2]
    float y = 0; ///< Gyro Noise 1-sigma [meters/second^2]
    float z = 0; ///< Gyro Noise 1-sigma [meters/second^2]
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_GYRO_BIAS_MODEL;
        
        float x_beta = 0; ///< Gyro Bias Beta [1/second]
        float y_beta = 0; ///< Gyro Bias Beta [1/second]
        float z_beta = 0; ///< Gyro Bias Beta [1/second]
        float x = 0; ///< Gyro Noise 1-sigma [meters/second^2]
        float y = 0; ///< Gyro Noise 1-sigma [meters/second^2]
        float z = 0; ///< Gyro Noise 1-sigma [meters/second^2]
        
    };
};
void insert(Serializer& serializer, const GyroBiasModel& self);
void extract(Serializer& serializer, GyroBiasModel& self);

void insert(Serializer& serializer, const GyroBiasModel::Response& self);
void extract(Serializer& serializer, GyroBiasModel::Response& self);

CmdResult writeGyroBiasModel(C::mip_interface& device, float xBeta, float yBeta, float zBeta, float x, float y, float z);
CmdResult readGyroBiasModel(C::mip_interface& device, float* xBetaOut, float* yBetaOut, float* zBetaOut, float* xOut, float* yOut, float* zOut);
CmdResult saveGyroBiasModel(C::mip_interface& device);
CmdResult loadGyroBiasModel(C::mip_interface& device);
CmdResult defaultGyroBiasModel(C::mip_interface& device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_altitude_aiding  (0x0D,0x47) Altitude Aiding [CPP]
/// Altitude Aiding Control
/// 
/// Select altitude input for absolute altitude and/or vertical velocity. The primary altitude reading is always GNSS.
/// Aiding inputs are used to improve GNSS altitude readings when GNSS is available and to backup GNSS during GNSS outages.
/// 
/// Possible altitude aiding selector values:
/// 
/// 0x00 - No altitude aiding (disable)
/// 0x01 - Enable pressure sensor aiding(1)
/// 
/// 1. Pressure altitude is based on "instant sea level pressure" which is dependent on location and weather conditions and can vary by more than 40 meters.
/// 
///
///@{

struct AltitudeAiding
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_ALTITUDE_AIDING_CONTROL;
    
    static const bool HAS_WRITE_FUNCTION = true;
    static const bool HAS_READ_FUNCTION = true;
    static const bool HAS_SAVE_FUNCTION = true;
    static const bool HAS_LOAD_FUNCTION = true;
    static const bool HAS_RESET_FUNCTION = true;
    
    FunctionSelector function = static_cast<FunctionSelector>(0);
    uint8_t aiding_selector = 0; ///< See above
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_ALTITUDE_AIDING_CONTROL;
        
        uint8_t aiding_selector = 0; ///< See above
        
    };
};
void insert(Serializer& serializer, const AltitudeAiding& self);
void extract(Serializer& serializer, AltitudeAiding& self);

void insert(Serializer& serializer, const AltitudeAiding::Response& self);
void extract(Serializer& serializer, AltitudeAiding::Response& self);

CmdResult writeAltitudeAiding(C::mip_interface& device, uint8_t aidingSelector);
CmdResult readAltitudeAiding(C::mip_interface& device, uint8_t* aidingSelectorOut);
CmdResult saveAltitudeAiding(C::mip_interface& device);
CmdResult loadAltitudeAiding(C::mip_interface& device);
CmdResult defaultAltitudeAiding(C::mip_interface& device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_auto_zupt  (0x0D,0x1E) Auto Zupt [CPP]
/// Zero Velocity Update
/// The ZUPT is triggered when the scalar magnitude of the GNSS reported velocity vector is equal-to or less than the threshold value.
/// The device will NACK threshold values that are less than zero (i.e.negative.)
///
///@{

struct AutoZupt
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_ZUPT_CONTROL;
    
    static const bool HAS_WRITE_FUNCTION = true;
    static const bool HAS_READ_FUNCTION = true;
    static const bool HAS_SAVE_FUNCTION = true;
    static const bool HAS_LOAD_FUNCTION = true;
    static const bool HAS_RESET_FUNCTION = true;
    
    FunctionSelector function = static_cast<FunctionSelector>(0);
    uint8_t enable = 0; ///< 0 - Disable, 1 - Enable
    float threshold = 0; ///< [meters/second]
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_ZUPT_CONTROL;
        
        uint8_t enable = 0; ///< 0 - Disable, 1 - Enable
        float threshold = 0; ///< [meters/second]
        
    };
};
void insert(Serializer& serializer, const AutoZupt& self);
void extract(Serializer& serializer, AutoZupt& self);

void insert(Serializer& serializer, const AutoZupt::Response& self);
void extract(Serializer& serializer, AutoZupt::Response& self);

CmdResult writeAutoZupt(C::mip_interface& device, uint8_t enable, float threshold);
CmdResult readAutoZupt(C::mip_interface& device, uint8_t* enableOut, float* thresholdOut);
CmdResult saveAutoZupt(C::mip_interface& device);
CmdResult loadAutoZupt(C::mip_interface& device);
CmdResult defaultAutoZupt(C::mip_interface& device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_auto_angular_zupt  (0x0D,0x20) Auto Angular Zupt [CPP]
/// Zero Angular Rate Update
/// The ZUPT is triggered when the scalar magnitude of the angular rate vector is equal-to or less than the threshold value.
/// The device will NACK threshold values that are less than zero (i.e.negative.)
///
///@{

struct AutoAngularZupt
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_ANGULAR_ZUPT_CONTROL;
    
    static const bool HAS_WRITE_FUNCTION = true;
    static const bool HAS_READ_FUNCTION = true;
    static const bool HAS_SAVE_FUNCTION = true;
    static const bool HAS_LOAD_FUNCTION = true;
    static const bool HAS_RESET_FUNCTION = true;
    
    FunctionSelector function = static_cast<FunctionSelector>(0);
    uint8_t enable = 0; ///< 0 - Disable, 1 - Enable
    float threshold = 0; ///< [radians/second]
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_ANGULAR_ZUPT_CONTROL;
        
        uint8_t enable = 0; ///< 0 - Disable, 1 - Enable
        float threshold = 0; ///< [radians/second]
        
    };
};
void insert(Serializer& serializer, const AutoAngularZupt& self);
void extract(Serializer& serializer, AutoAngularZupt& self);

void insert(Serializer& serializer, const AutoAngularZupt::Response& self);
void extract(Serializer& serializer, AutoAngularZupt::Response& self);

CmdResult writeAutoAngularZupt(C::mip_interface& device, uint8_t enable, float threshold);
CmdResult readAutoAngularZupt(C::mip_interface& device, uint8_t* enableOut, float* thresholdOut);
CmdResult saveAutoAngularZupt(C::mip_interface& device);
CmdResult loadAutoAngularZupt(C::mip_interface& device);
CmdResult defaultAutoAngularZupt(C::mip_interface& device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_commanded_zupt  (0x0D,0x22) Commanded Zupt [CPP]
/// Commanded Zero Velocity Update
/// Please see the device user manual for the maximum rate of this message.
///
///@{

struct CommandedZupt
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_COMMANDED_ZUPT;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    
};
void insert(Serializer& serializer, const CommandedZupt& self);
void extract(Serializer& serializer, CommandedZupt& self);

CmdResult commandedZupt(C::mip_interface& device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_commanded_angular_zupt  (0x0D,0x23) Commanded Angular Zupt [CPP]
/// Commanded Zero Angular Rate Update
/// Please see the device user manual for the maximum rate of this message.
///
///@{

struct CommandedAngularZupt
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_COMMANDED_ANGULAR_ZUPT;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    
};
void insert(Serializer& serializer, const CommandedAngularZupt& self);
void extract(Serializer& serializer, CommandedAngularZupt& self);

CmdResult commandedAngularZupt(C::mip_interface& device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_reference_position  (0x0D,0x26) Reference Position [CPP]
/// Set the Lat/Long/Alt reference position for the sensor.
/// 
/// This position is used by the sensor to calculate the WGS84 gravity and WMM2015 magnetic field parameters.
/// 
///
///@{

struct ReferencePosition
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_REFERENCE_POSITION;
    
    static const bool HAS_WRITE_FUNCTION = true;
    static const bool HAS_READ_FUNCTION = true;
    static const bool HAS_SAVE_FUNCTION = true;
    static const bool HAS_LOAD_FUNCTION = true;
    static const bool HAS_RESET_FUNCTION = true;
    
    FunctionSelector function = static_cast<FunctionSelector>(0);
    bool enable = 0; ///< enable/disable
    double latitude = 0; ///< [degrees]
    double longitude = 0; ///< [degrees]
    double altitude = 0; ///< [meters]
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_REFERENCE_POSITION;
        
        bool enable = 0; ///< enable/disable
        double latitude = 0; ///< [degrees]
        double longitude = 0; ///< [degrees]
        double altitude = 0; ///< [meters]
        
    };
};
void insert(Serializer& serializer, const ReferencePosition& self);
void extract(Serializer& serializer, ReferencePosition& self);

void insert(Serializer& serializer, const ReferencePosition::Response& self);
void extract(Serializer& serializer, ReferencePosition::Response& self);

CmdResult writeReferencePosition(C::mip_interface& device, bool enable, double latitude, double longitude, double altitude);
CmdResult readReferencePosition(C::mip_interface& device, bool* enableOut, double* latitudeOut, double* longitudeOut, double* altitudeOut);
CmdResult saveReferencePosition(C::mip_interface& device);
CmdResult loadReferencePosition(C::mip_interface& device);
CmdResult defaultReferencePosition(C::mip_interface& device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_accel_magnitude_error_adaptive_measurement  (0x0D,0x44) Accel Magnitude Error Adaptive Measurement [CPP]
/// Enable or disable the gravity magnitude error adaptive measurement.
/// This function can be used to tune the filter performance in the target application
/// 
/// Pick values that give you the least occurrence of invalid EF attitude output.
/// The default values are good for standard low dynamics applications.
/// Increase values for higher dynamic conditions, lower values for lower dynamic.
/// Too low a value will result in excessive heading errors.
/// Higher values increase heading errors when undergoing magnetic field anomalies caused by DC currents, magnets, steel structures,etc.
/// 
/// Adaptive measurements can be enabled/disabled without the need for providing the additional parameters.
/// In this case, only the function selector and enable value are required; all other parameters will remain at their previous values.
/// When “auto-adaptive” is selected, the filter and limit parameters are ignored.
/// Instead, aiding measurements which rely on the gravity vector will be automatically reweighted by the Kalman filter according to the perceived measurement quality.
/// 
///
///@{

struct AccelMagnitudeErrorAdaptiveMeasurement
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_ACCEL_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL;
    
    static const bool HAS_WRITE_FUNCTION = true;
    static const bool HAS_READ_FUNCTION = true;
    static const bool HAS_SAVE_FUNCTION = true;
    static const bool HAS_LOAD_FUNCTION = true;
    static const bool HAS_RESET_FUNCTION = true;
    
    FunctionSelector function = static_cast<FunctionSelector>(0);
    FilterAdaptiveMeasurement adaptive_measurement = static_cast<FilterAdaptiveMeasurement>(0); ///< Adaptive measurement selector
    float frequency = 0; ///< Low-pass filter curoff frequency [hertz]
    float low_limit = 0; ///< [meters/second^2]
    float high_limit = 0; ///< [meters/second^2]
    float low_limit_uncertainty = 0; ///< 1-Sigma [meters/second^2]
    float high_limit_uncertainty = 0; ///< 1-Sigma [meters/second^2]
    float minimum_uncertainty = 0; ///< 1-Sigma [meters/second^2]
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_ACCEL_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL;
        
        FilterAdaptiveMeasurement adaptive_measurement = static_cast<FilterAdaptiveMeasurement>(0); ///< Adaptive measurement selector
        float frequency = 0; ///< Low-pass filter curoff frequency [hertz]
        float low_limit = 0; ///< [meters/second^2]
        float high_limit = 0; ///< [meters/second^2]
        float low_limit_uncertainty = 0; ///< 1-Sigma [meters/second^2]
        float high_limit_uncertainty = 0; ///< 1-Sigma [meters/second^2]
        float minimum_uncertainty = 0; ///< 1-Sigma [meters/second^2]
        
    };
};
void insert(Serializer& serializer, const AccelMagnitudeErrorAdaptiveMeasurement& self);
void extract(Serializer& serializer, AccelMagnitudeErrorAdaptiveMeasurement& self);

void insert(Serializer& serializer, const AccelMagnitudeErrorAdaptiveMeasurement::Response& self);
void extract(Serializer& serializer, AccelMagnitudeErrorAdaptiveMeasurement::Response& self);

CmdResult writeAccelMagnitudeErrorAdaptiveMeasurement(C::mip_interface& device, FilterAdaptiveMeasurement adaptiveMeasurement, float frequency, float lowLimit, float highLimit, float lowLimitUncertainty, float highLimitUncertainty, float minimumUncertainty);
CmdResult readAccelMagnitudeErrorAdaptiveMeasurement(C::mip_interface& device, FilterAdaptiveMeasurement* adaptiveMeasurementOut, float* frequencyOut, float* lowLimitOut, float* highLimitOut, float* lowLimitUncertaintyOut, float* highLimitUncertaintyOut, float* minimumUncertaintyOut);
CmdResult saveAccelMagnitudeErrorAdaptiveMeasurement(C::mip_interface& device);
CmdResult loadAccelMagnitudeErrorAdaptiveMeasurement(C::mip_interface& device);
CmdResult defaultAccelMagnitudeErrorAdaptiveMeasurement(C::mip_interface& device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_mag_magnitude_error_adaptive_measurement  (0x0D,0x45) Mag Magnitude Error Adaptive Measurement [CPP]
/// Enable or disable the magnetometer magnitude error adaptive measurement.
/// This feature will reject magnetometer readings that are out of range of the thresholds specified (fixed adaptive) or calculated internally (auto-adaptive).
/// 
/// Pick values that give you the least occurrence of invalid EF attitude output.
/// The default values are good for standard low dynamics applications.
/// Increase values for higher dynamic conditions, lower values for lower dynamic.
/// Too low a value will result in excessive heading errors.
/// Higher values increase heading errors when undergoing magnetic field anomalies caused by DC currents, magnets, steel structures,etc.
/// 
///
///@{

struct MagMagnitudeErrorAdaptiveMeasurement
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_MAG_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL;
    
    static const bool HAS_WRITE_FUNCTION = true;
    static const bool HAS_READ_FUNCTION = true;
    static const bool HAS_SAVE_FUNCTION = true;
    static const bool HAS_LOAD_FUNCTION = true;
    static const bool HAS_RESET_FUNCTION = true;
    
    FunctionSelector function = static_cast<FunctionSelector>(0);
    FilterAdaptiveMeasurement adaptive_measurement = static_cast<FilterAdaptiveMeasurement>(0); ///< Adaptive measurement selector
    float frequency = 0; ///< Low-pass filter curoff frequency [hertz]
    float low_limit = 0; ///< [meters/second^2]
    float high_limit = 0; ///< [meters/second^2]
    float low_limit_uncertainty = 0; ///< 1-Sigma [meters/second^2]
    float high_limit_uncertainty = 0; ///< 1-Sigma [meters/second^2]
    float minimum_uncertainty = 0; ///< 1-Sigma [meters/second^2]
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_MAG_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL;
        
        FilterAdaptiveMeasurement adaptive_measurement = static_cast<FilterAdaptiveMeasurement>(0); ///< Adaptive measurement selector
        float frequency = 0; ///< Low-pass filter curoff frequency [hertz]
        float low_limit = 0; ///< [meters/second^2]
        float high_limit = 0; ///< [meters/second^2]
        float low_limit_uncertainty = 0; ///< 1-Sigma [meters/second^2]
        float high_limit_uncertainty = 0; ///< 1-Sigma [meters/second^2]
        float minimum_uncertainty = 0; ///< 1-Sigma [meters/second^2]
        
    };
};
void insert(Serializer& serializer, const MagMagnitudeErrorAdaptiveMeasurement& self);
void extract(Serializer& serializer, MagMagnitudeErrorAdaptiveMeasurement& self);

void insert(Serializer& serializer, const MagMagnitudeErrorAdaptiveMeasurement::Response& self);
void extract(Serializer& serializer, MagMagnitudeErrorAdaptiveMeasurement::Response& self);

CmdResult writeMagMagnitudeErrorAdaptiveMeasurement(C::mip_interface& device, FilterAdaptiveMeasurement adaptiveMeasurement, float frequency, float lowLimit, float highLimit, float lowLimitUncertainty, float highLimitUncertainty, float minimumUncertainty);
CmdResult readMagMagnitudeErrorAdaptiveMeasurement(C::mip_interface& device, FilterAdaptiveMeasurement* adaptiveMeasurementOut, float* frequencyOut, float* lowLimitOut, float* highLimitOut, float* lowLimitUncertaintyOut, float* highLimitUncertaintyOut, float* minimumUncertaintyOut);
CmdResult saveMagMagnitudeErrorAdaptiveMeasurement(C::mip_interface& device);
CmdResult loadMagMagnitudeErrorAdaptiveMeasurement(C::mip_interface& device);
CmdResult defaultMagMagnitudeErrorAdaptiveMeasurement(C::mip_interface& device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_mag_dip_angle_error_adaptive_measurement  (0x0D,0x46) Mag Dip Angle Error Adaptive Measurement [CPP]
/// Enable or disable the magnetometer dip angle error adaptive measurement.
/// This function can be used to tune the filter performance in the target application
/// 
/// Pick values that give you the least occurrence of invalid EF attitude output.
/// The default values are good for standard low dynamics applications.
/// Increase values for higher dynamic conditions, lower values for lower dynamic.
/// Too low a value will result in excessive heading errors.
/// Higher values increase heading errors when undergoing magnetic field anomalies caused by DC currents, magnets, steel structures,etc.
/// 
/// The magnetometer dip angle adaptive measurement is ignored if the auto-adaptive magnetometer magnitude or auto-adaptive accel magnitude options are selected.
/// 
///
///@{

struct MagDipAngleErrorAdaptiveMeasurement
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_MAG_DIP_ANGLE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL;
    
    static const bool HAS_WRITE_FUNCTION = true;
    static const bool HAS_READ_FUNCTION = true;
    static const bool HAS_SAVE_FUNCTION = true;
    static const bool HAS_LOAD_FUNCTION = true;
    static const bool HAS_RESET_FUNCTION = true;
    
    FunctionSelector function = static_cast<FunctionSelector>(0);
    bool enable = 0; ///< Enable/Disable
    float frequency = 0; ///< Low-pass filter curoff frequency [hertz]
    float high_limit = 0; ///< [meters/second^2]
    float high_limit_uncertainty = 0; ///< 1-Sigma [meters/second^2]
    float minimum_uncertainty = 0; ///< 1-Sigma [meters/second^2]
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_MAG_DIP_ANGLE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL;
        
        bool enable = 0; ///< Enable/Disable
        float frequency = 0; ///< Low-pass filter curoff frequency [hertz]
        float high_limit = 0; ///< [meters/second^2]
        float high_limit_uncertainty = 0; ///< 1-Sigma [meters/second^2]
        float minimum_uncertainty = 0; ///< 1-Sigma [meters/second^2]
        
    };
};
void insert(Serializer& serializer, const MagDipAngleErrorAdaptiveMeasurement& self);
void extract(Serializer& serializer, MagDipAngleErrorAdaptiveMeasurement& self);

void insert(Serializer& serializer, const MagDipAngleErrorAdaptiveMeasurement::Response& self);
void extract(Serializer& serializer, MagDipAngleErrorAdaptiveMeasurement::Response& self);

CmdResult writeMagDipAngleErrorAdaptiveMeasurement(C::mip_interface& device, bool enable, float frequency, float highLimit, float highLimitUncertainty, float minimumUncertainty);
CmdResult readMagDipAngleErrorAdaptiveMeasurement(C::mip_interface& device, bool* enableOut, float* frequencyOut, float* highLimitOut, float* highLimitUncertaintyOut, float* minimumUncertaintyOut);
CmdResult saveMagDipAngleErrorAdaptiveMeasurement(C::mip_interface& device);
CmdResult loadMagDipAngleErrorAdaptiveMeasurement(C::mip_interface& device);
CmdResult defaultMagDipAngleErrorAdaptiveMeasurement(C::mip_interface& device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_aiding_measurement_enable  (0x0D,0x50) Aiding Measurement Enable [CPP]
/// Enables / disables the specified aiding measurement source.
/// 
/// 
///
///@{

struct AidingMeasurementEnable
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_AIDING_MEASUREMENT_ENABLE;
    
    static const bool HAS_WRITE_FUNCTION = true;
    static const bool HAS_READ_FUNCTION = true;
    static const bool HAS_SAVE_FUNCTION = true;
    static const bool HAS_LOAD_FUNCTION = true;
    static const bool HAS_RESET_FUNCTION = true;
    
    enum class AidingSource : uint16_t
    {
        GNSS_POS_VEL     = 0,  ///<  GNSS Position and Velocity
        GNSS_HEADING     = 1,  ///<  GNSS Heading (dual antenna)
        ALTIMETER        = 2,  ///<  Altimeter
        SPEED            = 3,  ///<  Speed sensor / Odometer
        MAGNETOMETER     = 4,  ///<  Magnetometer
        EXTERNAL_HEADING = 5,  ///<  External heading input
        ALL              = 65535,  ///<  Save/load/reset all options
    };
    
    FunctionSelector function = static_cast<FunctionSelector>(0);
    AidingSource aiding_source = static_cast<AidingSource>(0); ///< Aiding measurement source
    bool enable = 0; ///< Controls the aiding source
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_AIDING_MEASUREMENT_ENABLE;
        
        AidingSource aiding_source = static_cast<AidingSource>(0); ///< Aiding measurement source
        bool enable = 0; ///< Controls the aiding source
        
    };
};
void insert(Serializer& serializer, const AidingMeasurementEnable& self);
void extract(Serializer& serializer, AidingMeasurementEnable& self);

void insert(Serializer& serializer, const AidingMeasurementEnable::Response& self);
void extract(Serializer& serializer, AidingMeasurementEnable::Response& self);

CmdResult writeAidingMeasurementEnable(C::mip_interface& device, AidingMeasurementEnable::AidingSource aidingSource, bool enable);
CmdResult readAidingMeasurementEnable(C::mip_interface& device, AidingMeasurementEnable::AidingSource aidingSource, bool* enableOut);
CmdResult saveAidingMeasurementEnable(C::mip_interface& device, AidingMeasurementEnable::AidingSource aidingSource);
CmdResult loadAidingMeasurementEnable(C::mip_interface& device, AidingMeasurementEnable::AidingSource aidingSource);
CmdResult defaultAidingMeasurementEnable(C::mip_interface& device, AidingMeasurementEnable::AidingSource aidingSource);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_run  (0x0D,0x05) Run [CPP]
/// Manual run command.
/// 
/// If the initialization configuration has the "wait_for_run_command" option enabled, the filter will wait until it receives this command before commencing integration and enabling the Kalman filter. Prior to the receipt of this command, the filter will remain in the filter initialization mode.
///
///@{

struct Run
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_RUN;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    
};
void insert(Serializer& serializer, const Run& self);
void extract(Serializer& serializer, Run& self);

CmdResult run(C::mip_interface& device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_kinematic_constraint  (0x0D,0x51) Kinematic Constraint [CPP]
/// Controls kinematic constraint model selection for the navigation filter.
/// 
/// See manual for explanation of how the kinematic constraints are applied.
///
///@{

struct KinematicConstraint
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_KINEMATIC_CONSTRAINT;
    
    static const bool HAS_WRITE_FUNCTION = true;
    static const bool HAS_READ_FUNCTION = true;
    static const bool HAS_SAVE_FUNCTION = true;
    static const bool HAS_LOAD_FUNCTION = true;
    static const bool HAS_RESET_FUNCTION = true;
    
    FunctionSelector function = static_cast<FunctionSelector>(0);
    uint8_t acceleration_constraint_selection = 0; ///< Acceleration constraint: <br/> 0=None (default), <br/> 1=Zero-acceleration.
    uint8_t velocity_constraint_selection = 0; ///< 0=None (default), <br/> 1=Zero-velocity, <br/> 2=Wheeled-vehicle. <br/>
    uint8_t angular_constraint_selection = 0; ///< 0=None (default), 1=Zero-angular rate (ZUPT).
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_KINEMATIC_CONSTRAINT;
        
        uint8_t acceleration_constraint_selection = 0; ///< Acceleration constraint: <br/> 0=None (default), <br/> 1=Zero-acceleration.
        uint8_t velocity_constraint_selection = 0; ///< 0=None (default), <br/> 1=Zero-velocity, <br/> 2=Wheeled-vehicle. <br/>
        uint8_t angular_constraint_selection = 0; ///< 0=None (default), 1=Zero-angular rate (ZUPT).
        
    };
};
void insert(Serializer& serializer, const KinematicConstraint& self);
void extract(Serializer& serializer, KinematicConstraint& self);

void insert(Serializer& serializer, const KinematicConstraint::Response& self);
void extract(Serializer& serializer, KinematicConstraint::Response& self);

CmdResult writeKinematicConstraint(C::mip_interface& device, uint8_t accelerationConstraintSelection, uint8_t velocityConstraintSelection, uint8_t angularConstraintSelection);
CmdResult readKinematicConstraint(C::mip_interface& device, uint8_t* accelerationConstraintSelectionOut, uint8_t* velocityConstraintSelectionOut, uint8_t* angularConstraintSelectionOut);
CmdResult saveKinematicConstraint(C::mip_interface& device);
CmdResult loadKinematicConstraint(C::mip_interface& device);
CmdResult defaultKinematicConstraint(C::mip_interface& device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_initialization_configuration  (0x0D,0x52) Initialization Configuration [CPP]
/// Controls the source and values used for initial conditions of the navigation solution.
/// 
/// Notes: Initial conditions are the position, velocity, and attitude of the platform used when the filter starts running or is reset.
/// For the user specified position array, the units are meters if the ECEF frame is selected, and degrees latitude, degrees longitude, and meters above ellipsoid if the latitude/longitude/height frame is selected.
/// For the user specified velocity array, the units are meters per second, but the reference frame depends on the reference frame selector (ECEF or NED).
///
///@{

struct InitializationConfiguration
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_INITIALIZATION_CONFIGURATION;
    
    static const bool HAS_WRITE_FUNCTION = true;
    static const bool HAS_READ_FUNCTION = true;
    static const bool HAS_SAVE_FUNCTION = true;
    static const bool HAS_LOAD_FUNCTION = true;
    static const bool HAS_RESET_FUNCTION = true;
    
    struct AlignmentSelector : Bitfield<AlignmentSelector>
    {
        enum _enumType : uint8_t
        {
            NONE         = 0x00,
            DUAL_ANTENNA = 0x01,  ///<  Dual-antenna GNSS alignment
            KINEMATIC    = 0x02,  ///<  GNSS kinematic alignment (GNSS velocity determines initial heading)
            MAGNETOMETER = 0x04,  ///<  Magnetometer heading alignment
            ALL          = 0x07,
        };
        uint8_t value = NONE;
        
        AlignmentSelector() : value(NONE) {}
        AlignmentSelector(int val) : value((uint8_t)val) {}
        operator uint8_t() const { return value; }
        AlignmentSelector& operator=(uint8_t val) { value = val; return *this; }
        AlignmentSelector& operator=(int val) { value = val; return *this; }
        AlignmentSelector& operator|=(uint8_t val) { return *this = value | val; }
        AlignmentSelector& operator&=(uint8_t val) { return *this = value & val; }
        
        bool dualAntenna() const { return (value & DUAL_ANTENNA) > 0; }
        void dualAntenna(bool val) { if(val) value |= DUAL_ANTENNA; else value &= ~DUAL_ANTENNA; }
        bool kinematic() const { return (value & KINEMATIC) > 0; }
        void kinematic(bool val) { if(val) value |= KINEMATIC; else value &= ~KINEMATIC; }
        bool magnetometer() const { return (value & MAGNETOMETER) > 0; }
        void magnetometer(bool val) { if(val) value |= MAGNETOMETER; else value &= ~MAGNETOMETER; }
        
        bool allSet() const { return value == ALL; }
        void setAll() { value |= ALL; }
    };
    
    enum class InitialConditionSource : uint8_t
    {
        AUTO_POS_VEL_ATT        = 0,  ///<  Automatic position, velocity and attitude
        AUTO_POS_VEL_PITCH_ROLL = 1,  ///<  Automatic position and velocity, automatic pitch and roll, and user-specified heading
        AUTO_POS_VEL            = 2,  ///<  Automatic position and velocity, with fully user-specified attitude
        MANUAL                  = 3,  ///<  User-specified position, velocity, and attitude.
    };
    
    FunctionSelector function = static_cast<FunctionSelector>(0);
    uint8_t wait_for_run_command = 0; ///< Initialize filter only after receiving "run" command
    InitialConditionSource initial_cond_src = static_cast<InitialConditionSource>(0); ///< Initial condition source:
    AlignmentSelector auto_heading_alignment_selector; ///< Bitfield specifying the allowed automatic heading alignment methods for automatic initial conditions. Bits are set to 1 to enable, and the correspond to the following: <br/>
    float initial_heading = 0; ///< User-specified initial platform heading (degrees).
    float initial_pitch = 0; ///< User-specified initial platform pitch (degrees)
    float initial_roll = 0; ///< User-specified initial platform roll (degrees)
    float initial_position[3] = {0}; ///< User-specified initial platform position (units determined by reference frame selector, see note.)
    float initial_velocity[3] = {0}; ///< User-specified initial platform velocity (units determined by reference frame selector, see note.)
    FilterReferenceFrame reference_frame_selector = static_cast<FilterReferenceFrame>(0); ///< User-specified initial position/velocity reference frames
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_INITIALIZATION_CONFIGURATION;
        
        uint8_t wait_for_run_command = 0; ///< Initialize filter only after receiving "run" command
        InitialConditionSource initial_cond_src = static_cast<InitialConditionSource>(0); ///< Initial condition source:
        AlignmentSelector auto_heading_alignment_selector; ///< Bitfield specifying the allowed automatic heading alignment methods for automatic initial conditions. Bits are set to 1 to enable, and the correspond to the following: <br/>
        float initial_heading = 0; ///< User-specified initial platform heading (degrees).
        float initial_pitch = 0; ///< User-specified initial platform pitch (degrees)
        float initial_roll = 0; ///< User-specified initial platform roll (degrees)
        float initial_position[3] = {0}; ///< User-specified initial platform position (units determined by reference frame selector, see note.)
        float initial_velocity[3] = {0}; ///< User-specified initial platform velocity (units determined by reference frame selector, see note.)
        FilterReferenceFrame reference_frame_selector = static_cast<FilterReferenceFrame>(0); ///< User-specified initial position/velocity reference frames
        
    };
};
void insert(Serializer& serializer, const InitializationConfiguration& self);
void extract(Serializer& serializer, InitializationConfiguration& self);

void insert(Serializer& serializer, const InitializationConfiguration::Response& self);
void extract(Serializer& serializer, InitializationConfiguration::Response& self);

CmdResult writeInitializationConfiguration(C::mip_interface& device, uint8_t waitForRunCommand, InitializationConfiguration::InitialConditionSource initialCondSrc, InitializationConfiguration::AlignmentSelector autoHeadingAlignmentSelector, float initialHeading, float initialPitch, float initialRoll, const float* initialPosition, const float* initialVelocity, FilterReferenceFrame referenceFrameSelector);
CmdResult readInitializationConfiguration(C::mip_interface& device, uint8_t* waitForRunCommandOut, InitializationConfiguration::InitialConditionSource* initialCondSrcOut, InitializationConfiguration::AlignmentSelector* autoHeadingAlignmentSelectorOut, float* initialHeadingOut, float* initialPitchOut, float* initialRollOut, float* initialPositionOut, float* initialVelocityOut, FilterReferenceFrame* referenceFrameSelectorOut);
CmdResult saveInitializationConfiguration(C::mip_interface& device);
CmdResult loadInitializationConfiguration(C::mip_interface& device);
CmdResult defaultInitializationConfiguration(C::mip_interface& device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_adaptive_filter_options  (0x0D,0x53) Adaptive Filter Options [CPP]
/// Configures the basic setup for auto-adaptive filtering. See product manual for a detailed description of this feature.
///
///@{

struct AdaptiveFilterOptions
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_ADAPTIVE_FILTER_OPTIONS;
    
    static const bool HAS_WRITE_FUNCTION = true;
    static const bool HAS_READ_FUNCTION = true;
    static const bool HAS_SAVE_FUNCTION = true;
    static const bool HAS_LOAD_FUNCTION = true;
    static const bool HAS_RESET_FUNCTION = true;
    
    FunctionSelector function = static_cast<FunctionSelector>(0);
    uint8_t level = 0; ///< Auto-adaptive operating level: <br/> 0=Off, <br/> 1=Conservative, <br/> 2=Moderate (default), <br/> 3=Aggressive.
    uint16_t time_limit = 0; ///< Maximum duration of measurement rejection before entering recovery mode    (ms)
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_ADAPTIVE_FILTER_OPTIONS;
        
        uint8_t level = 0; ///< Auto-adaptive operating level: <br/> 0=Off, <br/> 1=Conservative, <br/> 2=Moderate (default), <br/> 3=Aggressive.
        uint16_t time_limit = 0; ///< Maximum duration of measurement rejection before entering recovery mode    (ms)
        
    };
};
void insert(Serializer& serializer, const AdaptiveFilterOptions& self);
void extract(Serializer& serializer, AdaptiveFilterOptions& self);

void insert(Serializer& serializer, const AdaptiveFilterOptions::Response& self);
void extract(Serializer& serializer, AdaptiveFilterOptions::Response& self);

CmdResult writeAdaptiveFilterOptions(C::mip_interface& device, uint8_t level, uint16_t timeLimit);
CmdResult readAdaptiveFilterOptions(C::mip_interface& device, uint8_t* levelOut, uint16_t* timeLimitOut);
CmdResult saveAdaptiveFilterOptions(C::mip_interface& device);
CmdResult loadAdaptiveFilterOptions(C::mip_interface& device);
CmdResult defaultAdaptiveFilterOptions(C::mip_interface& device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_multi_antenna_offset  (0x0D,0x54) Multi Antenna Offset [CPP]
/// Set the antenna lever arm.
/// 
/// This command works with devices that utilize multiple antennas.
/// <br/><br/><b>Offset Limit</b>: 10 m magnitude (default)
///
///@{

struct MultiAntennaOffset
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_MULTI_ANTENNA_OFFSET;
    
    static const bool HAS_WRITE_FUNCTION = true;
    static const bool HAS_READ_FUNCTION = true;
    static const bool HAS_SAVE_FUNCTION = true;
    static const bool HAS_LOAD_FUNCTION = true;
    static const bool HAS_RESET_FUNCTION = true;
    
    FunctionSelector function = static_cast<FunctionSelector>(0);
    uint8_t receiver_id = 0; ///< Receiver: 1, 2, etc...
    float antenna_offset[3] = {0}; ///< Antenna lever arm offset vector in the vehicle frame (m)
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_MULTI_ANTENNA_OFFSET;
        
        uint8_t receiver_id = 0;
        float antenna_offset[3] = {0};
        
    };
};
void insert(Serializer& serializer, const MultiAntennaOffset& self);
void extract(Serializer& serializer, MultiAntennaOffset& self);

void insert(Serializer& serializer, const MultiAntennaOffset::Response& self);
void extract(Serializer& serializer, MultiAntennaOffset::Response& self);

CmdResult writeMultiAntennaOffset(C::mip_interface& device, uint8_t receiverId, const float* antennaOffset);
CmdResult readMultiAntennaOffset(C::mip_interface& device, uint8_t receiverId, float* antennaOffsetOut);
CmdResult saveMultiAntennaOffset(C::mip_interface& device, uint8_t receiverId);
CmdResult loadMultiAntennaOffset(C::mip_interface& device, uint8_t receiverId);
CmdResult defaultMultiAntennaOffset(C::mip_interface& device, uint8_t receiverId);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_rel_pos_configuration  (0x0D,0x55) Rel Pos Configuration [CPP]
/// Configure the reference location for filter relative positioning outputs
///
///@{

struct RelPosConfiguration
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_REL_POS_CONFIGURATION;
    
    static const bool HAS_WRITE_FUNCTION = true;
    static const bool HAS_READ_FUNCTION = true;
    static const bool HAS_SAVE_FUNCTION = true;
    static const bool HAS_LOAD_FUNCTION = true;
    static const bool HAS_RESET_FUNCTION = true;
    
    FunctionSelector function = static_cast<FunctionSelector>(0);
    uint8_t source = 0; ///< 0 - auto (RTK base station), 1 - manual
    FilterReferenceFrame reference_frame_selector = static_cast<FilterReferenceFrame>(0); ///< ECEF or LLH
    double reference_coordinates[3] = {0}; ///< reference coordinates, units determined by source selection
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_REL_POS_CONFIGURATION;
        
        uint8_t source = 0; ///< 0 - auto (RTK base station), 1 - manual
        FilterReferenceFrame reference_frame_selector = static_cast<FilterReferenceFrame>(0); ///< ECEF or LLH
        double reference_coordinates[3] = {0}; ///< reference coordinates, units determined by source selection
        
    };
};
void insert(Serializer& serializer, const RelPosConfiguration& self);
void extract(Serializer& serializer, RelPosConfiguration& self);

void insert(Serializer& serializer, const RelPosConfiguration::Response& self);
void extract(Serializer& serializer, RelPosConfiguration::Response& self);

CmdResult writeRelPosConfiguration(C::mip_interface& device, uint8_t source, FilterReferenceFrame referenceFrameSelector, const double* referenceCoordinates);
CmdResult readRelPosConfiguration(C::mip_interface& device, uint8_t* sourceOut, FilterReferenceFrame* referenceFrameSelectorOut, double* referenceCoordinatesOut);
CmdResult saveRelPosConfiguration(C::mip_interface& device);
CmdResult loadRelPosConfiguration(C::mip_interface& device);
CmdResult defaultRelPosConfiguration(C::mip_interface& device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_ref_point_lever_arm  (0x0D,0x56) Ref Point Lever Arm [CPP]
/// Lever arm offset with respect to the sensor for the indicated point of reference.
/// This is used to change the location of the indicated point of reference, and will affect filter position and velocity outputs.
/// Changing this setting from default will result in a global position offset that depends on vehicle attitude,
/// and a velocity offset that depends on vehicle attitude and angular rate.
/// <br/>The lever arm is defined by a 3-element vector that points from the sensor to the desired reference point, with (x,y,z) components given in the vehicle's reference frame.
/// <br/><br/>Note, if the reference point selector is set to VEH (1), this setting will affect the following data fields: (0x82, 0x01), (0x82, 0x02), (0x82, 0x40), (0x82, 0x41), and (0x82, 42)
/// <br/><br/><b>Offset Limits</b>
/// <br/>Reference Point VEH (1): 10 m magnitude (default)
///
///@{

struct RefPointLeverArm
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_REF_POINT_LEVER_ARM;
    
    static const bool HAS_WRITE_FUNCTION = true;
    static const bool HAS_READ_FUNCTION = true;
    static const bool HAS_SAVE_FUNCTION = true;
    static const bool HAS_LOAD_FUNCTION = true;
    static const bool HAS_RESET_FUNCTION = true;
    
    enum class ReferencePointSelector : uint8_t
    {
        VEH = 1,  ///<  Defines the origin of the vehicle
    };
    
    FunctionSelector function = static_cast<FunctionSelector>(0);
    ReferencePointSelector ref_point_sel = static_cast<ReferencePointSelector>(0); ///< Reserved, must be 1
    float lever_arm_offset[3] = {0}; ///< [m] Lever arm offset vector in the vehicle's reference frame.
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_REF_POINT_LEVER_ARM;
        
        ReferencePointSelector ref_point_sel = static_cast<ReferencePointSelector>(0); ///< Reserved, must be 1
        float lever_arm_offset[3] = {0}; ///< [m] Lever arm offset vector in the vehicle's reference frame.
        
    };
};
void insert(Serializer& serializer, const RefPointLeverArm& self);
void extract(Serializer& serializer, RefPointLeverArm& self);

void insert(Serializer& serializer, const RefPointLeverArm::Response& self);
void extract(Serializer& serializer, RefPointLeverArm::Response& self);

CmdResult writeRefPointLeverArm(C::mip_interface& device, RefPointLeverArm::ReferencePointSelector refPointSel, const float* leverArmOffset);
CmdResult readRefPointLeverArm(C::mip_interface& device, RefPointLeverArm::ReferencePointSelector* refPointSelOut, float* leverArmOffsetOut);
CmdResult saveRefPointLeverArm(C::mip_interface& device);
CmdResult loadRefPointLeverArm(C::mip_interface& device);
CmdResult defaultRefPointLeverArm(C::mip_interface& device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_speed_measurement  (0x0D,0x60) Speed Measurement [CPP]
/// Speed aiding measurement, where speed is defined as rate of motion along the vehicle's x-axis direction.
/// Can be used by an external odometer/speedometer, for example.
/// This command cannot be used if the internal odometer is configured.
///
///@{

struct SpeedMeasurement
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_SPEED_MEASUREMENT;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    uint8_t source = 0; ///< Reserved, must be 1.
    float time_of_week = 0; ///< GPS time of week when speed was sampled
    float speed = 0; ///< Estimated speed along vehicle's x-axis (may be positive or negative) [meters/second]
    float speed_uncertainty = 0; ///< Estimated uncertainty in the speed measurement (1-sigma value) [meters/second]
    
};
void insert(Serializer& serializer, const SpeedMeasurement& self);
void extract(Serializer& serializer, SpeedMeasurement& self);

CmdResult speedMeasurement(C::mip_interface& device, uint8_t source, float timeOfWeek, float speed, float speedUncertainty);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_speed_lever_arm  (0x0D,0x61) Speed Lever Arm [CPP]
/// Lever arm offset for speed measurements.
/// This is used to compensate for an off-center measurement point
/// having a different speed due to rotation of the vehicle.
/// The typical use case for this would be an odometer attached to a wheel
/// on a standard 4-wheeled vehicle. If the odometer is on the left wheel,
/// it will report higher speed on right turns and lower speed on left turns.
/// This is because the outside edge of the curve is longer than the inside edge.
///
///@{

struct SpeedLeverArm
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_SPEED_LEVER_ARM;
    
    static const bool HAS_WRITE_FUNCTION = true;
    static const bool HAS_READ_FUNCTION = true;
    static const bool HAS_SAVE_FUNCTION = true;
    static const bool HAS_LOAD_FUNCTION = true;
    static const bool HAS_RESET_FUNCTION = true;
    
    FunctionSelector function = static_cast<FunctionSelector>(0);
    uint8_t source = 0; ///< Reserved, must be 1.
    float lever_arm_offset[3] = {0}; ///< [m] Lever arm offset vector in the vehicle's reference frame.
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_SPEED_LEVER_ARM;
        
        uint8_t source = 0; ///< Reserved, must be 1.
        float lever_arm_offset[3] = {0}; ///< [m] Lever arm offset vector in the vehicle's reference frame.
        
    };
};
void insert(Serializer& serializer, const SpeedLeverArm& self);
void extract(Serializer& serializer, SpeedLeverArm& self);

void insert(Serializer& serializer, const SpeedLeverArm::Response& self);
void extract(Serializer& serializer, SpeedLeverArm::Response& self);

CmdResult writeSpeedLeverArm(C::mip_interface& device, uint8_t source, const float* leverArmOffset);
CmdResult readSpeedLeverArm(C::mip_interface& device, uint8_t source, float* leverArmOffsetOut);
CmdResult saveSpeedLeverArm(C::mip_interface& device, uint8_t source);
CmdResult loadSpeedLeverArm(C::mip_interface& device, uint8_t source);
CmdResult defaultSpeedLeverArm(C::mip_interface& device, uint8_t source);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_wheeled_vehicle_constraint_control  (0x0D,0x63) Wheeled Vehicle Constraint Control [CPP]
/// Configure the wheeled vehicle kinematic constraint.
/// 
/// When enabled, the filter uses the assumption that velocity is constrained to the primary vehicle axis.
/// By convention, the primary vehicle axis is the vehicle X-axis (note: the sensor may be physically installed in
/// any orientation on the vehicle if the appropriate mounting transformation has been specified).
/// This constraint will typically improve heading estimates for vehicles where the assumption is valid, such
/// as an automobile, particularly when GNSS coverage is intermittent.
///
///@{

struct WheeledVehicleConstraintControl
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_VEHICLE_CONSTRAINT_CONTROL;
    
    static const bool HAS_WRITE_FUNCTION = true;
    static const bool HAS_READ_FUNCTION = true;
    static const bool HAS_SAVE_FUNCTION = true;
    static const bool HAS_LOAD_FUNCTION = true;
    static const bool HAS_RESET_FUNCTION = true;
    
    FunctionSelector function = static_cast<FunctionSelector>(0);
    uint8_t enable = 0; ///< 0 - Disable, 1 - Enable
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_VEHICLE_CONSTRAINT_CONTROL;
        
        uint8_t enable = 0; ///< 0 - Disable, 1 - Enable
        
    };
};
void insert(Serializer& serializer, const WheeledVehicleConstraintControl& self);
void extract(Serializer& serializer, WheeledVehicleConstraintControl& self);

void insert(Serializer& serializer, const WheeledVehicleConstraintControl::Response& self);
void extract(Serializer& serializer, WheeledVehicleConstraintControl::Response& self);

CmdResult writeWheeledVehicleConstraintControl(C::mip_interface& device, uint8_t enable);
CmdResult readWheeledVehicleConstraintControl(C::mip_interface& device, uint8_t* enableOut);
CmdResult saveWheeledVehicleConstraintControl(C::mip_interface& device);
CmdResult loadWheeledVehicleConstraintControl(C::mip_interface& device);
CmdResult defaultWheeledVehicleConstraintControl(C::mip_interface& device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_vertical_gyro_constraint_control  (0x0D,0x62) Vertical Gyro Constraint Control [CPP]
/// Configure the vertical gyro kinematic constraint.
/// 
/// When enabled and no valid GNSS measurements are available, the filter uses the accelerometers to track pitch
/// and roll under the assumption that the sensor platform is not undergoing linear acceleration.
/// This constraint is useful to maintain accurate pitch and roll during GNSS signal outages.
///
///@{

struct VerticalGyroConstraintControl
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_GYRO_CONSTRAINT_CONTROL;
    
    static const bool HAS_WRITE_FUNCTION = true;
    static const bool HAS_READ_FUNCTION = true;
    static const bool HAS_SAVE_FUNCTION = true;
    static const bool HAS_LOAD_FUNCTION = true;
    static const bool HAS_RESET_FUNCTION = true;
    
    FunctionSelector function = static_cast<FunctionSelector>(0);
    uint8_t enable = 0; ///< 0 - Disable, 1 - Enable
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_GYRO_CONSTRAINT_CONTROL;
        
        uint8_t enable = 0; ///< 0 - Disable, 1 - Enable
        
    };
};
void insert(Serializer& serializer, const VerticalGyroConstraintControl& self);
void extract(Serializer& serializer, VerticalGyroConstraintControl& self);

void insert(Serializer& serializer, const VerticalGyroConstraintControl::Response& self);
void extract(Serializer& serializer, VerticalGyroConstraintControl::Response& self);

CmdResult writeVerticalGyroConstraintControl(C::mip_interface& device, uint8_t enable);
CmdResult readVerticalGyroConstraintControl(C::mip_interface& device, uint8_t* enableOut);
CmdResult saveVerticalGyroConstraintControl(C::mip_interface& device);
CmdResult loadVerticalGyroConstraintControl(C::mip_interface& device);
CmdResult defaultVerticalGyroConstraintControl(C::mip_interface& device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_gnss_antenna_cal_control  (0x0D,0x64) Gnss Antenna Cal Control [CPP]
/// Configure the GNSS antenna lever arm calibration.
/// 
/// When enabled, the filter will enable lever arm error tracking, up to the maximum offset specified.
///
///@{

struct GnssAntennaCalControl
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_ANTENNA_CALIBRATION_CONTROL;
    
    static const bool HAS_WRITE_FUNCTION = true;
    static const bool HAS_READ_FUNCTION = true;
    static const bool HAS_SAVE_FUNCTION = true;
    static const bool HAS_LOAD_FUNCTION = true;
    static const bool HAS_RESET_FUNCTION = true;
    
    FunctionSelector function = static_cast<FunctionSelector>(0);
    uint8_t enable = 0; ///< 0 - Disable, 1 - Enable
    float max_offset = 0; ///< Maximum absolute value of lever arm offset error in the vehicle frame [meters]. See device user manual for the valid range of this parameter.
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_ANTENNA_CALIBRATION_CONTROL;
        
        uint8_t enable = 0; ///< 0 - Disable, 1 - Enable
        float max_offset = 0; ///< Maximum absolute value of lever arm offset error in the vehicle frame [meters]. See device user manual for the valid range of this parameter.
        
    };
};
void insert(Serializer& serializer, const GnssAntennaCalControl& self);
void extract(Serializer& serializer, GnssAntennaCalControl& self);

void insert(Serializer& serializer, const GnssAntennaCalControl::Response& self);
void extract(Serializer& serializer, GnssAntennaCalControl::Response& self);

CmdResult writeGnssAntennaCalControl(C::mip_interface& device, uint8_t enable, float maxOffset);
CmdResult readGnssAntennaCalControl(C::mip_interface& device, uint8_t* enableOut, float* maxOffsetOut);
CmdResult saveGnssAntennaCalControl(C::mip_interface& device);
CmdResult loadGnssAntennaCalControl(C::mip_interface& device);
CmdResult defaultGnssAntennaCalControl(C::mip_interface& device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_hard_iron_offset_noise  (0x0D,0x2B) Hard Iron Offset Noise [CPP]
/// Set the expected hard iron offset noise 1-sigma values
/// 
/// This function can be used to tune the filter performance in the target application.
/// 
/// Each of the noise values must be greater than 0.0
/// 
/// The noise value represents process noise in the 3DM-GX5-45 NAV Estimation Filter.
/// Changing this value modifies how the filter responds to dynamic input and can be used to tune the performance of the filter.
/// Default values provide good performance for most laboratory conditions.
/// 
///
///@{

struct HardIronOffsetNoise
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_HARD_IRON_OFFSET_NOISE;
    
    static const bool HAS_WRITE_FUNCTION = true;
    static const bool HAS_READ_FUNCTION = true;
    static const bool HAS_SAVE_FUNCTION = true;
    static const bool HAS_LOAD_FUNCTION = true;
    static const bool HAS_RESET_FUNCTION = true;
    
    FunctionSelector function = static_cast<FunctionSelector>(0);
    float x = 0; ///< HI Offset Noise 1-sima [gauss]
    float y = 0; ///< HI Offset Noise 1-sima [gauss]
    float z = 0; ///< HI Offset Noise 1-sima [gauss]
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_HARD_IRON_OFFSET_NOISE;
        
        float x = 0; ///< HI Offset Noise 1-sima [gauss]
        float y = 0; ///< HI Offset Noise 1-sima [gauss]
        float z = 0; ///< HI Offset Noise 1-sima [gauss]
        
    };
};
void insert(Serializer& serializer, const HardIronOffsetNoise& self);
void extract(Serializer& serializer, HardIronOffsetNoise& self);

void insert(Serializer& serializer, const HardIronOffsetNoise::Response& self);
void extract(Serializer& serializer, HardIronOffsetNoise::Response& self);

CmdResult writeHardIronOffsetNoise(C::mip_interface& device, float x, float y, float z);
CmdResult readHardIronOffsetNoise(C::mip_interface& device, float* xOut, float* yOut, float* zOut);
CmdResult saveHardIronOffsetNoise(C::mip_interface& device);
CmdResult loadHardIronOffsetNoise(C::mip_interface& device);
CmdResult defaultHardIronOffsetNoise(C::mip_interface& device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_magnetic_declination_source  (0x0D,0x43) Magnetic Declination Source [CPP]
/// Source for magnetic declination angle, and user supplied value for manual selection.
///
///@{

struct MagneticDeclinationSource
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_DECLINATION_SOURCE;
    
    static const bool HAS_WRITE_FUNCTION = true;
    static const bool HAS_READ_FUNCTION = true;
    static const bool HAS_SAVE_FUNCTION = true;
    static const bool HAS_LOAD_FUNCTION = true;
    static const bool HAS_RESET_FUNCTION = true;
    
    FunctionSelector function = static_cast<FunctionSelector>(0);
    FilterMagDeclinationSource source = static_cast<FilterMagDeclinationSource>(0); ///< Magnetic field declination angle source
    float declination = 0; ///< Declination angle used when 'source' is set to 'MANUAL' (radians)
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_DECLINATION_SOURCE;
        
        FilterMagDeclinationSource source = static_cast<FilterMagDeclinationSource>(0); ///< Magnetic field declination angle source
        float declination = 0; ///< Declination angle used when 'source' is set to 'MANUAL' (radians)
        
    };
};
void insert(Serializer& serializer, const MagneticDeclinationSource& self);
void extract(Serializer& serializer, MagneticDeclinationSource& self);

void insert(Serializer& serializer, const MagneticDeclinationSource::Response& self);
void extract(Serializer& serializer, MagneticDeclinationSource::Response& self);

CmdResult writeMagneticDeclinationSource(C::mip_interface& device, FilterMagDeclinationSource source, float declination);
CmdResult readMagneticDeclinationSource(C::mip_interface& device, FilterMagDeclinationSource* sourceOut, float* declinationOut);
CmdResult saveMagneticDeclinationSource(C::mip_interface& device);
CmdResult loadMagneticDeclinationSource(C::mip_interface& device);
CmdResult defaultMagneticDeclinationSource(C::mip_interface& device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_set_initial_heading  (0x0D,0x03) Set Initial Heading [CPP]
/// Set the initial heading angle.
/// 
/// The estimation filter will reset the heading estimate to provided value. If the product supports magnetometer aiding and this feature has been enabled, the heading
/// argument will be ignored and the filter will initialize using the inferred magnetic heading.
///
///@{

struct SetInitialHeading
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_SET_INITIAL_HEADING;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    float heading = 0; ///< Initial heading in radians [-pi, pi]
    
};
void insert(Serializer& serializer, const SetInitialHeading& self);
void extract(Serializer& serializer, SetInitialHeading& self);

CmdResult setInitialHeading(C::mip_interface& device, float heading);
///@}
///

///@}
///@}
///
////////////////////////////////////////////////////////////////////////////////
} // namespace commands_filter
} // namespace mip

