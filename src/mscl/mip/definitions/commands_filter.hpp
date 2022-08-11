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
///@addtogroup MipCommands
///@{
///@defgroup filter_commands_cpp  FILTERCommands
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


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_reset  Reset Navigation Filter
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
///@defgroup cpp_set_initial_attitude  Set Initial Attitude
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
    
    float roll;
    float pitch;
    float heading;
    
};
void insert(Serializer& serializer, const SetInitialAttitude& self);
void extract(Serializer& serializer, SetInitialAttitude& self);

CmdResult setInitialAttitude(C::mip_interface& device, float roll, float pitch, float heading);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_estimation_control  Estimation Control Flags
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
            GYRO_BIAS          = 0x0001,
            ACCEL_BIAS         = 0x0002,
            GYRO_SCALE_FACTOR  = 0x0004,
            ACCEL_SCALE_FACTOR = 0x0008,
            ANTENNA_OFFSET     = 0x0010,
            AUTO_MAG_HARD_IRON = 0x0020,
            AUTO_MAG_SOFT_IRON = 0x0040,
        };
        uint16_t value = NONE;
        
        operator uint16_t() const { return value; }
        EnableFlags& operator=(uint16_t val) { value = val; return *this; }
        EnableFlags& operator|=(uint16_t val) { return *this = value | val; }
        EnableFlags& operator&=(uint16_t val) { return *this = value & val; }
    };
    
    FunctionSelector function;
    EnableFlags enable;
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_ESTIMATION_CONTROL_FLAGS;
        
        EnableFlags enable;
        
    };
};
void insert(Serializer& serializer, const EstimationControl& self);
void extract(Serializer& serializer, EstimationControl& self);

void insert(Serializer& serializer, const EstimationControl::Response& self);
void extract(Serializer& serializer, EstimationControl::Response& self);

CmdResult writeEstimationControl(C::mip_interface& device, EstimationControl::EnableFlags enable);
CmdResult readEstimationControl(C::mip_interface& device, EstimationControl::EnableFlags& enable);
CmdResult saveEstimationControl(C::mip_interface& device);
CmdResult loadEstimationControl(C::mip_interface& device);
CmdResult defaultEstimationControl(C::mip_interface& device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_external_gnss_update  External GNSS Update
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
    
    double gps_time;
    uint16_t gps_week;
    double latitude;
    double longitude;
    double height;
    float velocity[3];
    float pos_uncertainty[3];
    float vel_uncertainty[3];
    
};
void insert(Serializer& serializer, const ExternalGnssUpdate& self);
void extract(Serializer& serializer, ExternalGnssUpdate& self);

CmdResult externalGnssUpdate(C::mip_interface& device, double gps_time, uint16_t gps_week, double latitude, double longitude, double height, const float* velocity, const float* pos_uncertainty, const float* vel_uncertainty);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_external_heading_update  External Heading Update
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
    
    float heading;
    float heading_uncertainty;
    uint8_t type;
    
};
void insert(Serializer& serializer, const ExternalHeadingUpdate& self);
void extract(Serializer& serializer, ExternalHeadingUpdate& self);

CmdResult externalHeadingUpdate(C::mip_interface& device, float heading, float heading_uncertainty, uint8_t type);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_external_heading_update_with_time  External Heading Update With Time
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
    
    double gps_time;
    uint16_t gps_week;
    float heading;
    float heading_uncertainty;
    uint8_t type;
    
};
void insert(Serializer& serializer, const ExternalHeadingUpdateWithTime& self);
void extract(Serializer& serializer, ExternalHeadingUpdateWithTime& self);

CmdResult externalHeadingUpdateWithTime(C::mip_interface& device, double gps_time, uint16_t gps_week, float heading, float heading_uncertainty, uint8_t type);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_tare_orientation  Tare Sensor Orientation
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
            ROLL  = 0x1,
            PITCH = 0x2,
            YAW   = 0x4,
        };
        uint8_t value = NONE;
        
        operator uint8_t() const { return value; }
        MipTareAxes& operator=(uint8_t val) { value = val; return *this; }
        MipTareAxes& operator|=(uint8_t val) { return *this = value | val; }
        MipTareAxes& operator&=(uint8_t val) { return *this = value & val; }
    };
    
    FunctionSelector function;
    MipTareAxes axes;
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_TARE_ORIENTATION;
        
        MipTareAxes axes;
        
    };
};
void insert(Serializer& serializer, const TareOrientation& self);
void extract(Serializer& serializer, TareOrientation& self);

void insert(Serializer& serializer, const TareOrientation::Response& self);
void extract(Serializer& serializer, TareOrientation::Response& self);

CmdResult writeTareOrientation(C::mip_interface& device, TareOrientation::MipTareAxes axes);
CmdResult readTareOrientation(C::mip_interface& device, TareOrientation::MipTareAxes& axes);
CmdResult saveTareOrientation(C::mip_interface& device);
CmdResult loadTareOrientation(C::mip_interface& device);
CmdResult defaultTareOrientation(C::mip_interface& device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_sensor_to_vehicle_rotation_euler  Sensor to Vehicle Frame Rotation Euler
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
    
    FunctionSelector function;
    float roll;
    float pitch;
    float yaw;
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_SENSOR2VEHICLE_ROTATION_EULER;
        
        float roll;
        float pitch;
        float yaw;
        
    };
};
void insert(Serializer& serializer, const SensorToVehicleRotationEuler& self);
void extract(Serializer& serializer, SensorToVehicleRotationEuler& self);

void insert(Serializer& serializer, const SensorToVehicleRotationEuler::Response& self);
void extract(Serializer& serializer, SensorToVehicleRotationEuler::Response& self);

CmdResult writeSensorToVehicleRotationEuler(C::mip_interface& device, float roll, float pitch, float yaw);
CmdResult readSensorToVehicleRotationEuler(C::mip_interface& device, float& roll, float& pitch, float& yaw);
CmdResult saveSensorToVehicleRotationEuler(C::mip_interface& device);
CmdResult loadSensorToVehicleRotationEuler(C::mip_interface& device);
CmdResult defaultSensorToVehicleRotationEuler(C::mip_interface& device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_sensor_to_vehicle_rotation_dcm  Sensor to Vehicle Frame Rotation DCM
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
    
    FunctionSelector function;
    float dcm[9];
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_SENSOR2VEHICLE_ROTATION_DCM;
        
        float dcm[9];
        
    };
};
void insert(Serializer& serializer, const SensorToVehicleRotationDcm& self);
void extract(Serializer& serializer, SensorToVehicleRotationDcm& self);

void insert(Serializer& serializer, const SensorToVehicleRotationDcm::Response& self);
void extract(Serializer& serializer, SensorToVehicleRotationDcm::Response& self);

CmdResult writeSensorToVehicleRotationDcm(C::mip_interface& device, const float* dcm);
CmdResult readSensorToVehicleRotationDcm(C::mip_interface& device, float* dcm);
CmdResult saveSensorToVehicleRotationDcm(C::mip_interface& device);
CmdResult loadSensorToVehicleRotationDcm(C::mip_interface& device);
CmdResult defaultSensorToVehicleRotationDcm(C::mip_interface& device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_sensor_to_vehicle_rotation_quaternion  Sensor to Vehicle Frame Rotation Quaternion
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
    
    FunctionSelector function;
    float quat[4];
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_SENSOR2VEHICLE_ROTATION_QUATERNION;
        
        float quat[4];
        
    };
};
void insert(Serializer& serializer, const SensorToVehicleRotationQuaternion& self);
void extract(Serializer& serializer, SensorToVehicleRotationQuaternion& self);

void insert(Serializer& serializer, const SensorToVehicleRotationQuaternion::Response& self);
void extract(Serializer& serializer, SensorToVehicleRotationQuaternion::Response& self);

CmdResult writeSensorToVehicleRotationQuaternion(C::mip_interface& device, const float* quat);
CmdResult readSensorToVehicleRotationQuaternion(C::mip_interface& device, float* quat);
CmdResult saveSensorToVehicleRotationQuaternion(C::mip_interface& device);
CmdResult loadSensorToVehicleRotationQuaternion(C::mip_interface& device);
CmdResult defaultSensorToVehicleRotationQuaternion(C::mip_interface& device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_sensor_to_vehicle_offset  Sensor to Vehicle Frame Offset
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
    
    FunctionSelector function;
    float offset[3];
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_SENSOR2VEHICLE_OFFSET;
        
        float offset[3];
        
    };
};
void insert(Serializer& serializer, const SensorToVehicleOffset& self);
void extract(Serializer& serializer, SensorToVehicleOffset& self);

void insert(Serializer& serializer, const SensorToVehicleOffset::Response& self);
void extract(Serializer& serializer, SensorToVehicleOffset::Response& self);

CmdResult writeSensorToVehicleOffset(C::mip_interface& device, const float* offset);
CmdResult readSensorToVehicleOffset(C::mip_interface& device, float* offset);
CmdResult saveSensorToVehicleOffset(C::mip_interface& device);
CmdResult loadSensorToVehicleOffset(C::mip_interface& device);
CmdResult defaultSensorToVehicleOffset(C::mip_interface& device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_antenna_offset  GNSS Antenna Offset Control
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
    
    FunctionSelector function;
    float offset[3];
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_ANTENNA_OFFSET;
        
        float offset[3];
        
    };
};
void insert(Serializer& serializer, const AntennaOffset& self);
void extract(Serializer& serializer, AntennaOffset& self);

void insert(Serializer& serializer, const AntennaOffset::Response& self);
void extract(Serializer& serializer, AntennaOffset::Response& self);

CmdResult writeAntennaOffset(C::mip_interface& device, const float* offset);
CmdResult readAntennaOffset(C::mip_interface& device, float* offset);
CmdResult saveAntennaOffset(C::mip_interface& device);
CmdResult loadAntennaOffset(C::mip_interface& device);
CmdResult defaultAntennaOffset(C::mip_interface& device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_gnss_source  GNSS Aiding Source Control
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
    
    FunctionSelector function;
    Source source;
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_GNSS_SOURCE_CONTROL;
        
        Source source;
        
    };
};
void insert(Serializer& serializer, const GnssSource& self);
void extract(Serializer& serializer, GnssSource& self);

void insert(Serializer& serializer, const GnssSource::Response& self);
void extract(Serializer& serializer, GnssSource::Response& self);

CmdResult writeGnssSource(C::mip_interface& device, GnssSource::Source source);
CmdResult readGnssSource(C::mip_interface& device, GnssSource::Source& source);
CmdResult saveGnssSource(C::mip_interface& device);
CmdResult loadGnssSource(C::mip_interface& device);
CmdResult defaultGnssSource(C::mip_interface& device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_heading_source  Heading Aiding Source Control
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
        GNSS_VEL                      = 2,  ///<  Seen notes 1,2
        EXTERNAL                      = 3,  ///<  
        GNSS_VEL_AND_MAG              = 4,  ///<  
        GNSS_VEL_AND_EXTERNAL         = 5,  ///<  
        MAG_AND_EXTERNAL              = 6,  ///<  
        GNSS_VEL_AND_MAG_AND_EXTERNAL = 7,  ///<  
    };
    
    FunctionSelector function;
    Source source;
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_HEADING_UPDATE_CONTROL;
        
        Source source;
        
    };
};
void insert(Serializer& serializer, const HeadingSource& self);
void extract(Serializer& serializer, HeadingSource& self);

void insert(Serializer& serializer, const HeadingSource::Response& self);
void extract(Serializer& serializer, HeadingSource::Response& self);

CmdResult writeHeadingSource(C::mip_interface& device, HeadingSource::Source source);
CmdResult readHeadingSource(C::mip_interface& device, HeadingSource::Source& source);
CmdResult saveHeadingSource(C::mip_interface& device);
CmdResult loadHeadingSource(C::mip_interface& device);
CmdResult defaultHeadingSource(C::mip_interface& device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_altitude_aiding  Altitude Aiding Control
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
    
    FunctionSelector function;
    uint8_t aiding_selector;
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_ALTITUDE_AIDING_CONTROL;
        
        uint8_t aiding_selector;
        
    };
};
void insert(Serializer& serializer, const AltitudeAiding& self);
void extract(Serializer& serializer, AltitudeAiding& self);

void insert(Serializer& serializer, const AltitudeAiding::Response& self);
void extract(Serializer& serializer, AltitudeAiding::Response& self);

CmdResult writeAltitudeAiding(C::mip_interface& device, uint8_t aiding_selector);
CmdResult readAltitudeAiding(C::mip_interface& device, uint8_t& aiding_selector);
CmdResult saveAltitudeAiding(C::mip_interface& device);
CmdResult loadAltitudeAiding(C::mip_interface& device);
CmdResult defaultAltitudeAiding(C::mip_interface& device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_auto_zupt  Zero Velocity Update Control
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
    
    FunctionSelector function;
    uint8_t enable;
    float threshold;
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_ZUPT_CONTROL;
        
        uint8_t enable;
        float threshold;
        
    };
};
void insert(Serializer& serializer, const AutoZupt& self);
void extract(Serializer& serializer, AutoZupt& self);

void insert(Serializer& serializer, const AutoZupt::Response& self);
void extract(Serializer& serializer, AutoZupt::Response& self);

CmdResult writeAutoZupt(C::mip_interface& device, uint8_t enable, float threshold);
CmdResult readAutoZupt(C::mip_interface& device, uint8_t& enable, float& threshold);
CmdResult saveAutoZupt(C::mip_interface& device);
CmdResult loadAutoZupt(C::mip_interface& device);
CmdResult defaultAutoZupt(C::mip_interface& device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_auto_angular_zupt  Zero Angular Rate Update Control
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
    
    FunctionSelector function;
    uint8_t enable;
    float threshold;
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_ANGULAR_ZUPT_CONTROL;
        
        uint8_t enable;
        float threshold;
        
    };
};
void insert(Serializer& serializer, const AutoAngularZupt& self);
void extract(Serializer& serializer, AutoAngularZupt& self);

void insert(Serializer& serializer, const AutoAngularZupt::Response& self);
void extract(Serializer& serializer, AutoAngularZupt::Response& self);

CmdResult writeAutoAngularZupt(C::mip_interface& device, uint8_t enable, float threshold);
CmdResult readAutoAngularZupt(C::mip_interface& device, uint8_t& enable, float& threshold);
CmdResult saveAutoAngularZupt(C::mip_interface& device);
CmdResult loadAutoAngularZupt(C::mip_interface& device);
CmdResult defaultAutoAngularZupt(C::mip_interface& device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_commanded_zupt  Commanded Zero Veloicty Update
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
///@defgroup cpp_commanded_angular_zupt  Commanded Zero Angular Rate Update
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
///@defgroup cpp_aiding_measurement_enable  Aiding Measurement Control
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
    
    FunctionSelector function;
    AidingSource aiding_source;
    bool enable;
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_AIDING_MEASUREMENT_ENABLE;
        
        AidingSource aiding_source;
        bool enable;
        
    };
};
void insert(Serializer& serializer, const AidingMeasurementEnable& self);
void extract(Serializer& serializer, AidingMeasurementEnable& self);

void insert(Serializer& serializer, const AidingMeasurementEnable::Response& self);
void extract(Serializer& serializer, AidingMeasurementEnable::Response& self);

CmdResult writeAidingMeasurementEnable(C::mip_interface& device, AidingMeasurementEnable::AidingSource aiding_source, bool enable);
CmdResult readAidingMeasurementEnable(C::mip_interface& device, AidingMeasurementEnable::AidingSource aiding_source, bool& enable);
CmdResult saveAidingMeasurementEnable(C::mip_interface& device, AidingMeasurementEnable::AidingSource aiding_source);
CmdResult loadAidingMeasurementEnable(C::mip_interface& device, AidingMeasurementEnable::AidingSource aiding_source);
CmdResult defaultAidingMeasurementEnable(C::mip_interface& device, AidingMeasurementEnable::AidingSource aiding_source);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_run  Run Navigation Filter
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
///@defgroup cpp_kinematic_constraint  Kinematic Constraint Control
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
    
    FunctionSelector function;
    uint8_t acceleration_constraint_selection;
    uint8_t velocity_constraint_selection;
    uint8_t angular_constraint_selection;
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_KINEMATIC_CONSTRAINT;
        
        uint8_t acceleration_constraint_selection;
        uint8_t velocity_constraint_selection;
        uint8_t angular_constraint_selection;
        
    };
};
void insert(Serializer& serializer, const KinematicConstraint& self);
void extract(Serializer& serializer, KinematicConstraint& self);

void insert(Serializer& serializer, const KinematicConstraint::Response& self);
void extract(Serializer& serializer, KinematicConstraint::Response& self);

CmdResult writeKinematicConstraint(C::mip_interface& device, uint8_t acceleration_constraint_selection, uint8_t velocity_constraint_selection, uint8_t angular_constraint_selection);
CmdResult readKinematicConstraint(C::mip_interface& device, uint8_t& acceleration_constraint_selection, uint8_t& velocity_constraint_selection, uint8_t& angular_constraint_selection);
CmdResult saveKinematicConstraint(C::mip_interface& device);
CmdResult loadKinematicConstraint(C::mip_interface& device);
CmdResult defaultKinematicConstraint(C::mip_interface& device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_initialization_configuration  Navigation Filter Initialization
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
            DUAL_ANTENNA = 0x01,
            KINEMATIC    = 0x02,
            MAGNETOMETER = 0x04,
        };
        uint8_t value = NONE;
        
        operator uint8_t() const { return value; }
        AlignmentSelector& operator=(uint8_t val) { value = val; return *this; }
        AlignmentSelector& operator|=(uint8_t val) { return *this = value | val; }
        AlignmentSelector& operator&=(uint8_t val) { return *this = value & val; }
    };
    
    enum class InitialConditionSource : uint8_t
    {
        AUTO_POS_VEL_ATT        = 0,  ///<  Automatic position, velocity and attitude
        AUTO_POS_VEL_PITCH_ROLL = 1,  ///<  Automatic position and velocity, automatic pitch and roll, and user-specified heading
        AUTO_POS_VEL            = 2,  ///<  Automatic position and velocity, with fully user-specified attitude
        MANUAL                  = 3,  ///<  User-specified position, velocity, and attitude.
    };
    
    FunctionSelector function;
    uint8_t wait_for_run_command;
    InitialConditionSource initial_cond_src;
    AlignmentSelector auto_heading_alignment_selector;
    float initial_heading;
    float initial_pitch;
    float initial_roll;
    float initial_position[3];
    float initial_velocity[3];
    FilterReferenceFrame reference_frame_selector;
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_INITIALIZATION_CONFIGURATION;
        
        uint8_t wait_for_run_command;
        InitialConditionSource initial_cond_src;
        AlignmentSelector auto_heading_alignment_selector;
        float initial_heading;
        float initial_pitch;
        float initial_roll;
        float initial_position[3];
        float initial_velocity[3];
        FilterReferenceFrame reference_frame_selector;
        
    };
};
void insert(Serializer& serializer, const InitializationConfiguration& self);
void extract(Serializer& serializer, InitializationConfiguration& self);

void insert(Serializer& serializer, const InitializationConfiguration::Response& self);
void extract(Serializer& serializer, InitializationConfiguration::Response& self);

CmdResult writeInitializationConfiguration(C::mip_interface& device, uint8_t wait_for_run_command, InitializationConfiguration::InitialConditionSource initial_cond_src, InitializationConfiguration::AlignmentSelector auto_heading_alignment_selector, float initial_heading, float initial_pitch, float initial_roll, const float* initial_position, const float* initial_velocity, FilterReferenceFrame reference_frame_selector);
CmdResult readInitializationConfiguration(C::mip_interface& device, uint8_t& wait_for_run_command, InitializationConfiguration::InitialConditionSource& initial_cond_src, InitializationConfiguration::AlignmentSelector& auto_heading_alignment_selector, float& initial_heading, float& initial_pitch, float& initial_roll, float* initial_position, float* initial_velocity, FilterReferenceFrame& reference_frame_selector);
CmdResult saveInitializationConfiguration(C::mip_interface& device);
CmdResult loadInitializationConfiguration(C::mip_interface& device);
CmdResult defaultInitializationConfiguration(C::mip_interface& device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_adaptive_filter_options  Adaptive Filter Control
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
    
    FunctionSelector function;
    uint8_t level;
    uint16_t time_limit;
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_ADAPTIVE_FILTER_OPTIONS;
        
        uint8_t level;
        uint16_t time_limit;
        
    };
};
void insert(Serializer& serializer, const AdaptiveFilterOptions& self);
void extract(Serializer& serializer, AdaptiveFilterOptions& self);

void insert(Serializer& serializer, const AdaptiveFilterOptions::Response& self);
void extract(Serializer& serializer, AdaptiveFilterOptions::Response& self);

CmdResult writeAdaptiveFilterOptions(C::mip_interface& device, uint8_t level, uint16_t time_limit);
CmdResult readAdaptiveFilterOptions(C::mip_interface& device, uint8_t& level, uint16_t& time_limit);
CmdResult saveAdaptiveFilterOptions(C::mip_interface& device);
CmdResult loadAdaptiveFilterOptions(C::mip_interface& device);
CmdResult defaultAdaptiveFilterOptions(C::mip_interface& device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_multi_antenna_offset  GNSS Multi-Antenna Offset Control
/// Set the antenna lever arm.
/// 
/// This command works with devices that utilize multiple antennas.
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
    
    FunctionSelector function;
    uint8_t receiver_id;
    float antenna_offset[3];
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_MULTI_ANTENNA_OFFSET;
        
        uint8_t receiver_id;
        float antenna_offset[3];
        
    };
};
void insert(Serializer& serializer, const MultiAntennaOffset& self);
void extract(Serializer& serializer, MultiAntennaOffset& self);

void insert(Serializer& serializer, const MultiAntennaOffset::Response& self);
void extract(Serializer& serializer, MultiAntennaOffset::Response& self);

CmdResult writeMultiAntennaOffset(C::mip_interface& device, uint8_t receiver_id, const float* antenna_offset);
CmdResult readMultiAntennaOffset(C::mip_interface& device, uint8_t receiver_id, float* antenna_offset);
CmdResult saveMultiAntennaOffset(C::mip_interface& device, uint8_t receiver_id);
CmdResult loadMultiAntennaOffset(C::mip_interface& device, uint8_t receiver_id);
CmdResult defaultMultiAntennaOffset(C::mip_interface& device, uint8_t receiver_id);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_rel_pos_configuration  Relative Position Configuration
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
    
    FunctionSelector function;
    uint8_t source;
    FilterReferenceFrame reference_frame_selector;
    double reference_coordinates[3];
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_REL_POS_CONFIGURATION;
        
        uint8_t source;
        FilterReferenceFrame reference_frame_selector;
        double reference_coordinates[3];
        
    };
};
void insert(Serializer& serializer, const RelPosConfiguration& self);
void extract(Serializer& serializer, RelPosConfiguration& self);

void insert(Serializer& serializer, const RelPosConfiguration::Response& self);
void extract(Serializer& serializer, RelPosConfiguration::Response& self);

CmdResult writeRelPosConfiguration(C::mip_interface& device, uint8_t source, FilterReferenceFrame reference_frame_selector, const double* reference_coordinates);
CmdResult readRelPosConfiguration(C::mip_interface& device, uint8_t& source, FilterReferenceFrame& reference_frame_selector, double* reference_coordinates);
CmdResult saveRelPosConfiguration(C::mip_interface& device);
CmdResult loadRelPosConfiguration(C::mip_interface& device);
CmdResult defaultRelPosConfiguration(C::mip_interface& device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_ref_point_lever_arm  Reference point lever arm
/// Lever arm offset with respect to the sensor for the indicated point of reference.
/// This is used to change the location of the indicated point of reference, and will affect filter position and velocity outputs.
/// Changing this setting from default will result in a global position offset that depends on vehicle attitude,
/// and a velocity offset that depends on vehicle attitude and angular rate.
/// The lever arm is defined by a 3-element vector that points from the sensor to the desired reference point, with (x,y,z) components given in the vehicle's reference frame.
/// Note, if the reference point selector is set to VEH (1), this setting will affect the following data fields: (0x82, 0x01), (0x82, 0x02), (0x82, 0x40), (0x82, 0x41), and (0x82, 42)
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
    
    FunctionSelector function;
    ReferencePointSelector ref_point_sel;
    float lever_arm_offset[3];
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_REF_POINT_LEVER_ARM;
        
        ReferencePointSelector ref_point_sel;
        float lever_arm_offset[3];
        
    };
};
void insert(Serializer& serializer, const RefPointLeverArm& self);
void extract(Serializer& serializer, RefPointLeverArm& self);

void insert(Serializer& serializer, const RefPointLeverArm::Response& self);
void extract(Serializer& serializer, RefPointLeverArm::Response& self);

CmdResult writeRefPointLeverArm(C::mip_interface& device, RefPointLeverArm::ReferencePointSelector ref_point_sel, const float* lever_arm_offset);
CmdResult readRefPointLeverArm(C::mip_interface& device, RefPointLeverArm::ReferencePointSelector& ref_point_sel, float* lever_arm_offset);
CmdResult saveRefPointLeverArm(C::mip_interface& device);
CmdResult loadRefPointLeverArm(C::mip_interface& device);
CmdResult defaultRefPointLeverArm(C::mip_interface& device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_speed_measurement  Input speed measurement
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
    
    uint8_t source;
    float time_of_week;
    float speed;
    float speed_uncertainty;
    
};
void insert(Serializer& serializer, const SpeedMeasurement& self);
void extract(Serializer& serializer, SpeedMeasurement& self);

CmdResult speedMeasurement(C::mip_interface& device, uint8_t source, float time_of_week, float speed, float speed_uncertainty);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_speed_lever_arm  Measurement speed lever arm
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
    
    FunctionSelector function;
    uint8_t source;
    float lever_arm_offset[3];
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_SPEED_LEVER_ARM;
        
        uint8_t source;
        float lever_arm_offset[3];
        
    };
};
void insert(Serializer& serializer, const SpeedLeverArm& self);
void extract(Serializer& serializer, SpeedLeverArm& self);

void insert(Serializer& serializer, const SpeedLeverArm::Response& self);
void extract(Serializer& serializer, SpeedLeverArm::Response& self);

CmdResult writeSpeedLeverArm(C::mip_interface& device, uint8_t source, const float* lever_arm_offset);
CmdResult readSpeedLeverArm(C::mip_interface& device, uint8_t source, float* lever_arm_offset);
CmdResult saveSpeedLeverArm(C::mip_interface& device, uint8_t source);
CmdResult loadSpeedLeverArm(C::mip_interface& device, uint8_t source);
CmdResult defaultSpeedLeverArm(C::mip_interface& device, uint8_t source);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_wheeled_vehicle_constraint_control  Wheeled Vehicle Constraint Control
/// Configure the wheeled vehicle kinematic constraint.
/// 
/// When enabled, the filter uses the assumption that velocity is constrained to the primary vehicle axis.
/// By convention, the primary vehicle axis is the vehicle X-axis (note: the sensor may be physically installed in
/// any orientation on the vehicle if the appropriate mounting transformation has been specified).
/// This constraint will typically improve heading estimates for vehicles where the assumption is valid, such
/// as an automobile, particulary when GNSS coverage is intermittent.
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
    
    FunctionSelector function;
    uint8_t enable;
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_VEHICLE_CONSTRAINT_CONTROL;
        
        uint8_t enable;
        
    };
};
void insert(Serializer& serializer, const WheeledVehicleConstraintControl& self);
void extract(Serializer& serializer, WheeledVehicleConstraintControl& self);

void insert(Serializer& serializer, const WheeledVehicleConstraintControl::Response& self);
void extract(Serializer& serializer, WheeledVehicleConstraintControl::Response& self);

CmdResult writeWheeledVehicleConstraintControl(C::mip_interface& device, uint8_t enable);
CmdResult readWheeledVehicleConstraintControl(C::mip_interface& device, uint8_t& enable);
CmdResult saveWheeledVehicleConstraintControl(C::mip_interface& device);
CmdResult loadWheeledVehicleConstraintControl(C::mip_interface& device);
CmdResult defaultWheeledVehicleConstraintControl(C::mip_interface& device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_vertical_gyro_constraint_control  Vertical Gyro Constraint Control
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
    
    FunctionSelector function;
    uint8_t enable;
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_GYRO_CONSTRAINT_CONTROL;
        
        uint8_t enable;
        
    };
};
void insert(Serializer& serializer, const VerticalGyroConstraintControl& self);
void extract(Serializer& serializer, VerticalGyroConstraintControl& self);

void insert(Serializer& serializer, const VerticalGyroConstraintControl::Response& self);
void extract(Serializer& serializer, VerticalGyroConstraintControl::Response& self);

CmdResult writeVerticalGyroConstraintControl(C::mip_interface& device, uint8_t enable);
CmdResult readVerticalGyroConstraintControl(C::mip_interface& device, uint8_t& enable);
CmdResult saveVerticalGyroConstraintControl(C::mip_interface& device);
CmdResult loadVerticalGyroConstraintControl(C::mip_interface& device);
CmdResult defaultVerticalGyroConstraintControl(C::mip_interface& device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_gnss_antenna_cal_control  GNSS Antenna Offset Calibration Control
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
    
    FunctionSelector function;
    uint8_t enable;
    float max_offset;
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_ANTENNA_CALIBRATION_CONTROL;
        
        uint8_t enable;
        float max_offset;
        
    };
};
void insert(Serializer& serializer, const GnssAntennaCalControl& self);
void extract(Serializer& serializer, GnssAntennaCalControl& self);

void insert(Serializer& serializer, const GnssAntennaCalControl::Response& self);
void extract(Serializer& serializer, GnssAntennaCalControl::Response& self);

CmdResult writeGnssAntennaCalControl(C::mip_interface& device, uint8_t enable, float max_offset);
CmdResult readGnssAntennaCalControl(C::mip_interface& device, uint8_t& enable, float& max_offset);
CmdResult saveGnssAntennaCalControl(C::mip_interface& device);
CmdResult loadGnssAntennaCalControl(C::mip_interface& device);
CmdResult defaultGnssAntennaCalControl(C::mip_interface& device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_magnetic_declination_source  Magnetic Field Declination Source Control
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
    
    FunctionSelector function;
    FilterMagDeclinationSource source;
    float declination;
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_DECLINATION_SOURCE;
        
        FilterMagDeclinationSource source;
        float declination;
        
    };
};
void insert(Serializer& serializer, const MagneticDeclinationSource& self);
void extract(Serializer& serializer, MagneticDeclinationSource& self);

void insert(Serializer& serializer, const MagneticDeclinationSource::Response& self);
void extract(Serializer& serializer, MagneticDeclinationSource::Response& self);

CmdResult writeMagneticDeclinationSource(C::mip_interface& device, FilterMagDeclinationSource source, float declination);
CmdResult readMagneticDeclinationSource(C::mip_interface& device, FilterMagDeclinationSource& source, float& declination);
CmdResult saveMagneticDeclinationSource(C::mip_interface& device);
CmdResult loadMagneticDeclinationSource(C::mip_interface& device);
CmdResult defaultMagneticDeclinationSource(C::mip_interface& device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_set_initial_heading  Set Initial Heading Control
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
    
    float heading;
    
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

