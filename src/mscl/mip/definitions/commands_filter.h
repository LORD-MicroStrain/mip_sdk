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
///@defgroup FILTERCommands  FILTER
///
///@{

////////////////////////////////////////////////////////////////////////////////
// Descriptors
////////////////////////////////////////////////////////////////////////////////

enum mip_filter_commands_descriptors
{
    MIP_FILTER_CMD_DESC_SET                                  = 0x0D,
    
    MIP_CMD_DESC_FILTER_RESET_FILTER                         = 0x01,
    MIP_CMD_DESC_FILTER_SET_INITIAL_ATTITUDE                 = 0x02,
    MIP_CMD_DESC_FILTER_ESTIMATION_CONTROL_FLAGS             = 0x14,
    MIP_CMD_DESC_FILTER_EXTERNAL_GNSS_UPDATE                 = 0x16,
    MIP_CMD_DESC_FILTER_EXTERNAL_HEADING_UPDATE              = 0x17,
    MIP_CMD_DESC_FILTER_EXTERNAL_HEADING_UPDATE_WITH_TIME    = 0x1F,
    MIP_CMD_DESC_FILTER_TARE_ORIENTATION                     = 0x21,
    MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_ROTATION_EULER        = 0x11,
    MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_ROTATION_DCM          = 0x4E,
    MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_ROTATION_QUATERNION   = 0x4F,
    MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_OFFSET                = 0x12,
    MIP_CMD_DESC_FILTER_ANTENNA_OFFSET                       = 0x13,
    MIP_CMD_DESC_FILTER_GNSS_SOURCE_CONTROL                  = 0x15,
    MIP_CMD_DESC_FILTER_HEADING_UPDATE_CONTROL               = 0x18,
    MIP_CMD_DESC_FILTER_ALTITUDE_AIDING_CONTROL              = 0x47,
    MIP_CMD_DESC_FILTER_ZUPT_CONTROL                         = 0x1E,
    MIP_CMD_DESC_FILTER_ANGULAR_ZUPT_CONTROL                 = 0x20,
    MIP_CMD_DESC_FILTER_COMMANDED_ZUPT                       = 0x22,
    MIP_CMD_DESC_FILTER_COMMANDED_ANGULAR_ZUPT               = 0x23,
    MIP_CMD_DESC_FILTER_AIDING_MEASUREMENT_ENABLE            = 0x50,
    MIP_CMD_DESC_FILTER_RUN                                  = 0x05,
    MIP_CMD_DESC_FILTER_KINEMATIC_CONSTRAINT                 = 0x51,
    MIP_CMD_DESC_FILTER_INITIALIZATION_CONFIGURATION         = 0x52,
    MIP_CMD_DESC_FILTER_ADAPTIVE_FILTER_OPTIONS              = 0x53,
    MIP_CMD_DESC_FILTER_MULTI_ANTENNA_OFFSET                 = 0x54,
    MIP_CMD_DESC_FILTER_REL_POS_CONFIGURATION                = 0x55,
    MIP_CMD_DESC_FILTER_REF_POINT_LEVER_ARM                  = 0x56,
    MIP_CMD_DESC_FILTER_SPEED_MEASUREMENT                    = 0x60,
    MIP_CMD_DESC_FILTER_SPEED_LEVER_ARM                      = 0x61,
    MIP_CMD_DESC_WHEELED_VEHICLE_CONSTRAINT_CONTROL          = 0x63,
    MIP_CMD_DESC_VERTICAL_GYRO_CONSTRAINT_CONTROL            = 0x62,
    MIP_CMD_DESC_GNSS_ANTENNA_CALIBRATION_CONTROL            = 0x64,
    MIP_CMD_DESC_FILTER_DECLINATION_SOURCE                   = 0x43,
    MIP_CMD_DESC_FILTER_SET_INITIAL_HEADING                  = 0x03,
    
    MIP_REPLY_DESC_FILTER_ESTIMATION_CONTROL_FLAGS           = 0x84,
    MIP_REPLY_DESC_FILTER_TARE_ORIENTATION                   = 0xA1,
    MIP_REPLY_DESC_FILTER_SENSOR2VEHICLE_ROTATION_EULER      = 0x81,
    MIP_REPLY_DESC_FILTER_SENSOR2VEHICLE_ROTATION_DCM        = 0xBE,
    MIP_REPLY_DESC_FILTER_SENSOR2VEHICLE_ROTATION_QUATERNION = 0xBF,
    MIP_REPLY_DESC_FILTER_SENSOR2VEHICLE_OFFSET              = 0x82,
    MIP_REPLY_DESC_FILTER_ANTENNA_OFFSET                     = 0x83,
    MIP_REPLY_DESC_FILTER_GNSS_SOURCE_CONTROL                = 0x86,
    MIP_REPLY_DESC_FILTER_HEADING_UPDATE_CONTROL             = 0x87,
    MIP_REPLY_DESC_FILTER_ALTITUDE_AIDING_CONTROL            = 0xB7,
    MIP_REPLY_DESC_FILTER_ZUPT_CONTROL                       = 0x8D,
    MIP_REPLY_DESC_FILTER_ANGULAR_ZUPT_CONTROL               = 0x8E,
    MIP_REPLY_DESC_FILTER_AIDING_MEASUREMENT_ENABLE          = 0xD0,
    MIP_REPLY_DESC_FILTER_KINEMATIC_CONSTRAINT               = 0xD1,
    MIP_REPLY_DESC_FILTER_INITIALIZATION_CONFIGURATION       = 0xD2,
    MIP_REPLY_DESC_FILTER_ADAPTIVE_FILTER_OPTIONS            = 0xD3,
    MIP_REPLY_DESC_FILTER_MULTI_ANTENNA_OFFSET               = 0xD4,
    MIP_REPLY_DESC_FILTER_REL_POS_CONFIGURATION              = 0xD5,
    MIP_REPLY_DESC_FILTER_REF_POINT_LEVER_ARM                = 0xD6,
    MIP_REPLY_DESC_FILTER_SPEED_LEVER_ARM                    = 0xE1,
    MIP_REPLY_DESC_WHEELED_VEHICLE_CONSTRAINT_CONTROL        = 0xE3,
    MIP_REPLY_DESC_VERTICAL_GYRO_CONSTRAINT_CONTROL          = 0xE2,
    MIP_REPLY_DESC_GNSS_ANTENNA_CALIBRATION_CONTROL          = 0xE4,
    MIP_REPLY_DESC_FILTER_DECLINATION_SOURCE                 = 0xB2,
};
#ifdef __cplusplus
namespace C {
extern "C" {
#endif // __cplusplus


////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////

enum mip_filter_reference_frame
{
    MIP_FILTER_REFERENCE_FRAME_ECEF = 1,  ///<  WGS84 Earth-fixed, earth centered coordinates
    MIP_FILTER_REFERENCE_FRAME_LLH  = 2,  ///<  WGS84 Latitude, longitude, and height above ellipsoid
};
size_t insert_mip_filter_reference_frame(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_filter_reference_frame self);
size_t extract_mip_filter_reference_frame(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_filter_reference_frame* self);

enum mip_filter_mag_declination_source
{
    MIP_FILTER_MAG_DECLINATION_SOURCE_NONE   = 1,  ///<  Magnetic field is assumed to have an declination angle equal to zero.
    MIP_FILTER_MAG_DECLINATION_SOURCE_WMM    = 2,  ///<  Magnetic field is assumed to conform to the World Magnetic Model, calculated using current location estimate as an input to the model.
    MIP_FILTER_MAG_DECLINATION_SOURCE_MANUAL = 3,  ///<  Magnetic field is assumed to have the declination angle specified by the user.
};
size_t insert_mip_filter_mag_declination_source(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_filter_mag_declination_source self);
size_t extract_mip_filter_mag_declination_source(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_filter_mag_declination_source* self);


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
///@defgroup reset  Reset Navigation Filter
/// Resets the filter to the initialization state.
/// 
/// If the auto-initialization feature is disabled, the initial attitude or heading must be set in
/// order to enter the run state after a reset.
///
///@{

struct mip_filter_reset_command
{
};
size_t insert_mip_filter_reset_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_reset_command* self);
size_t extract_mip_filter_reset_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_reset_command* self);

mip_cmd_result mip_filter_reset(struct mip_interface* device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup set_initial_attitude  Set Initial Attitude
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

struct mip_filter_set_initial_attitude_command
{
    float                                             roll;
    float                                             pitch;
    float                                             heading;
};
size_t insert_mip_filter_set_initial_attitude_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_set_initial_attitude_command* self);
size_t extract_mip_filter_set_initial_attitude_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_set_initial_attitude_command* self);

mip_cmd_result mip_filter_set_initial_attitude(struct mip_interface* device, float roll, float pitch, float heading);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup estimation_control  Estimation Control Flags
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

enum mip_filter_estimation_control_command_enable_flags
{
    MIP_FILTER_ESTIMATION_CONTROL_COMMAND_ENABLE_FLAGS_GYRO_BIAS          = 0x01,
    MIP_FILTER_ESTIMATION_CONTROL_COMMAND_ENABLE_FLAGS_ACCEL_BIAS         = 0x02,
    MIP_FILTER_ESTIMATION_CONTROL_COMMAND_ENABLE_FLAGS_GYRO_SCALE_FACTOR  = 0x04,
    MIP_FILTER_ESTIMATION_CONTROL_COMMAND_ENABLE_FLAGS_ACCEL_SCALE_FACTOR = 0x08,
    MIP_FILTER_ESTIMATION_CONTROL_COMMAND_ENABLE_FLAGS_ANTENNA_OFFSET     = 0x10,
    MIP_FILTER_ESTIMATION_CONTROL_COMMAND_ENABLE_FLAGS_AUTO_MAG_HARD_IRON = 0x20,
    MIP_FILTER_ESTIMATION_CONTROL_COMMAND_ENABLE_FLAGS_AUTO_MAG_SOFT_IRON = 0x40,
};
size_t insert_mip_filter_estimation_control_command_enable_flags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_filter_estimation_control_command_enable_flags self);
size_t extract_mip_filter_estimation_control_command_enable_flags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_filter_estimation_control_command_enable_flags* self);

struct mip_filter_estimation_control_command
{
    enum mip_function_selector                        function;
    enum mip_filter_estimation_control_command_enable_flags enable;
};
size_t insert_mip_filter_estimation_control_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_estimation_control_command* self);
size_t extract_mip_filter_estimation_control_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_estimation_control_command* self);

struct mip_filter_estimation_control_response
{
    enum mip_filter_estimation_control_command_enable_flags enable;
};
size_t insert_mip_filter_estimation_control_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_estimation_control_response* self);
size_t extract_mip_filter_estimation_control_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_estimation_control_response* self);

mip_cmd_result write_mip_filter_estimation_control(struct mip_interface* device, enum mip_filter_estimation_control_command_enable_flags enable);
mip_cmd_result read_mip_filter_estimation_control(struct mip_interface* device, enum mip_filter_estimation_control_command_enable_flags* enable);
mip_cmd_result save_mip_filter_estimation_control(struct mip_interface* device);
mip_cmd_result load_mip_filter_estimation_control(struct mip_interface* device);
mip_cmd_result default_mip_filter_estimation_control(struct mip_interface* device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup external_gnss_update  External GNSS Update
/// Provide a filter measurement from an external GNSS
/// 
/// The GNSS source control must be set to "external" for this command to succeed, otherwise it will be NACK'd.
/// Please refer to your device user manual for information on the maximum rate of this message.
/// 
///
///@{

struct mip_filter_external_gnss_update_command
{
    double                                            gps_time;
    uint16_t                                          gps_week;
    double                                            latitude;
    double                                            longitude;
    double                                            height;
    float                                             velocity[3];
    float                                             pos_uncertainty[3];
    float                                             vel_uncertainty[3];
};
size_t insert_mip_filter_external_gnss_update_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_external_gnss_update_command* self);
size_t extract_mip_filter_external_gnss_update_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_external_gnss_update_command* self);

mip_cmd_result mip_filter_external_gnss_update(struct mip_interface* device, double gps_time, uint16_t gps_week, double latitude, double longitude, double height, const float* velocity, const float* pos_uncertainty, const float* vel_uncertainty);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup external_heading_update  External Heading Update
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

struct mip_filter_external_heading_update_command
{
    float                                             heading;
    float                                             heading_uncertainty;
    uint8_t                                           type;
};
size_t insert_mip_filter_external_heading_update_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_external_heading_update_command* self);
size_t extract_mip_filter_external_heading_update_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_external_heading_update_command* self);

mip_cmd_result mip_filter_external_heading_update(struct mip_interface* device, float heading, float heading_uncertainty, uint8_t type);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup external_heading_update_with_time  External Heading Update With Time
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

struct mip_filter_external_heading_update_with_time_command
{
    double                                            gps_time;
    uint16_t                                          gps_week;
    float                                             heading;
    float                                             heading_uncertainty;
    uint8_t                                           type;
};
size_t insert_mip_filter_external_heading_update_with_time_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_external_heading_update_with_time_command* self);
size_t extract_mip_filter_external_heading_update_with_time_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_external_heading_update_with_time_command* self);

mip_cmd_result mip_filter_external_heading_update_with_time(struct mip_interface* device, double gps_time, uint16_t gps_week, float heading, float heading_uncertainty, uint8_t type);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup tare_orientation  Tare Sensor Orientation
/// Tare the device orientation.
/// 
/// This function uses the current device orientation relative to the NED frame as the current sensor to vehicle transformation.
/// This command is provided as a convenient way to set the sensor to vehicle frame transformation.
/// The filter must be initialized and have a valid attitude output. If the attitude is not valid, an error will be returned.
///
///@{

enum mip_filter_tare_orientation_command_mip_tare_axes
{
    MIP_FILTER_TARE_ORIENTATION_COMMAND_MIP_TARE_AXES_ROLL  = 0x01,
    MIP_FILTER_TARE_ORIENTATION_COMMAND_MIP_TARE_AXES_PITCH = 0x02,
    MIP_FILTER_TARE_ORIENTATION_COMMAND_MIP_TARE_AXES_YAW   = 0x04,
};
size_t insert_mip_filter_tare_orientation_command_mip_tare_axes(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_filter_tare_orientation_command_mip_tare_axes self);
size_t extract_mip_filter_tare_orientation_command_mip_tare_axes(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_filter_tare_orientation_command_mip_tare_axes* self);

struct mip_filter_tare_orientation_command
{
    enum mip_function_selector                        function;
    enum mip_filter_tare_orientation_command_mip_tare_axes axes;
};
size_t insert_mip_filter_tare_orientation_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_tare_orientation_command* self);
size_t extract_mip_filter_tare_orientation_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_tare_orientation_command* self);

struct mip_filter_tare_orientation_response
{
    enum mip_filter_tare_orientation_command_mip_tare_axes axes;
};
size_t insert_mip_filter_tare_orientation_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_tare_orientation_response* self);
size_t extract_mip_filter_tare_orientation_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_tare_orientation_response* self);

mip_cmd_result write_mip_filter_tare_orientation(struct mip_interface* device, enum mip_filter_tare_orientation_command_mip_tare_axes axes);
mip_cmd_result read_mip_filter_tare_orientation(struct mip_interface* device, enum mip_filter_tare_orientation_command_mip_tare_axes* axes);
mip_cmd_result save_mip_filter_tare_orientation(struct mip_interface* device);
mip_cmd_result load_mip_filter_tare_orientation(struct mip_interface* device);
mip_cmd_result default_mip_filter_tare_orientation(struct mip_interface* device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup sensor_to_vehicle_rotation_euler  Sensor to Vehicle Frame Rotation Euler
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

struct mip_filter_sensor_to_vehicle_rotation_euler_command
{
    enum mip_function_selector                        function;
    float                                             roll;
    float                                             pitch;
    float                                             yaw;
};
size_t insert_mip_filter_sensor_to_vehicle_rotation_euler_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_sensor_to_vehicle_rotation_euler_command* self);
size_t extract_mip_filter_sensor_to_vehicle_rotation_euler_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_sensor_to_vehicle_rotation_euler_command* self);

struct mip_filter_sensor_to_vehicle_rotation_euler_response
{
    float                                             roll;
    float                                             pitch;
    float                                             yaw;
};
size_t insert_mip_filter_sensor_to_vehicle_rotation_euler_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_sensor_to_vehicle_rotation_euler_response* self);
size_t extract_mip_filter_sensor_to_vehicle_rotation_euler_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_sensor_to_vehicle_rotation_euler_response* self);

mip_cmd_result write_mip_filter_sensor_to_vehicle_rotation_euler(struct mip_interface* device, float roll, float pitch, float yaw);
mip_cmd_result read_mip_filter_sensor_to_vehicle_rotation_euler(struct mip_interface* device, float* roll, float* pitch, float* yaw);
mip_cmd_result save_mip_filter_sensor_to_vehicle_rotation_euler(struct mip_interface* device);
mip_cmd_result load_mip_filter_sensor_to_vehicle_rotation_euler(struct mip_interface* device);
mip_cmd_result default_mip_filter_sensor_to_vehicle_rotation_euler(struct mip_interface* device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup sensor_to_vehicle_rotation_dcm  Sensor to Vehicle Frame Rotation DCM
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

struct mip_filter_sensor_to_vehicle_rotation_dcm_command
{
    enum mip_function_selector                        function;
    float                                             dcm[9];
};
size_t insert_mip_filter_sensor_to_vehicle_rotation_dcm_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_sensor_to_vehicle_rotation_dcm_command* self);
size_t extract_mip_filter_sensor_to_vehicle_rotation_dcm_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_sensor_to_vehicle_rotation_dcm_command* self);

struct mip_filter_sensor_to_vehicle_rotation_dcm_response
{
    float                                             dcm[9];
};
size_t insert_mip_filter_sensor_to_vehicle_rotation_dcm_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_sensor_to_vehicle_rotation_dcm_response* self);
size_t extract_mip_filter_sensor_to_vehicle_rotation_dcm_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_sensor_to_vehicle_rotation_dcm_response* self);

mip_cmd_result write_mip_filter_sensor_to_vehicle_rotation_dcm(struct mip_interface* device, const float* dcm);
mip_cmd_result read_mip_filter_sensor_to_vehicle_rotation_dcm(struct mip_interface* device, float* dcm);
mip_cmd_result save_mip_filter_sensor_to_vehicle_rotation_dcm(struct mip_interface* device);
mip_cmd_result load_mip_filter_sensor_to_vehicle_rotation_dcm(struct mip_interface* device);
mip_cmd_result default_mip_filter_sensor_to_vehicle_rotation_dcm(struct mip_interface* device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup sensor_to_vehicle_rotation_quaternion  Sensor to Vehicle Frame Rotation Quaternion
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

struct mip_filter_sensor_to_vehicle_rotation_quaternion_command
{
    enum mip_function_selector                        function;
    float                                             quat[4];
};
size_t insert_mip_filter_sensor_to_vehicle_rotation_quaternion_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_sensor_to_vehicle_rotation_quaternion_command* self);
size_t extract_mip_filter_sensor_to_vehicle_rotation_quaternion_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_sensor_to_vehicle_rotation_quaternion_command* self);

struct mip_filter_sensor_to_vehicle_rotation_quaternion_response
{
    float                                             quat[4];
};
size_t insert_mip_filter_sensor_to_vehicle_rotation_quaternion_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_sensor_to_vehicle_rotation_quaternion_response* self);
size_t extract_mip_filter_sensor_to_vehicle_rotation_quaternion_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_sensor_to_vehicle_rotation_quaternion_response* self);

mip_cmd_result write_mip_filter_sensor_to_vehicle_rotation_quaternion(struct mip_interface* device, const float* quat);
mip_cmd_result read_mip_filter_sensor_to_vehicle_rotation_quaternion(struct mip_interface* device, float* quat);
mip_cmd_result save_mip_filter_sensor_to_vehicle_rotation_quaternion(struct mip_interface* device);
mip_cmd_result load_mip_filter_sensor_to_vehicle_rotation_quaternion(struct mip_interface* device);
mip_cmd_result default_mip_filter_sensor_to_vehicle_rotation_quaternion(struct mip_interface* device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup sensor_to_vehicle_offset  Sensor to Vehicle Frame Offset
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

struct mip_filter_sensor_to_vehicle_offset_command
{
    enum mip_function_selector                        function;
    float                                             offset[3];
};
size_t insert_mip_filter_sensor_to_vehicle_offset_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_sensor_to_vehicle_offset_command* self);
size_t extract_mip_filter_sensor_to_vehicle_offset_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_sensor_to_vehicle_offset_command* self);

struct mip_filter_sensor_to_vehicle_offset_response
{
    float                                             offset[3];
};
size_t insert_mip_filter_sensor_to_vehicle_offset_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_sensor_to_vehicle_offset_response* self);
size_t extract_mip_filter_sensor_to_vehicle_offset_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_sensor_to_vehicle_offset_response* self);

mip_cmd_result write_mip_filter_sensor_to_vehicle_offset(struct mip_interface* device, const float* offset);
mip_cmd_result read_mip_filter_sensor_to_vehicle_offset(struct mip_interface* device, float* offset);
mip_cmd_result save_mip_filter_sensor_to_vehicle_offset(struct mip_interface* device);
mip_cmd_result load_mip_filter_sensor_to_vehicle_offset(struct mip_interface* device);
mip_cmd_result default_mip_filter_sensor_to_vehicle_offset(struct mip_interface* device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup antenna_offset  GNSS Antenna Offset Control
/// Set the sensor to GNSS antenna offset.
/// 
/// This is expressed in the sensor frame, from the sensor origin to the GNSS antenna RF center.
/// 
/// The magnitude of the offset vector is limited to 10 meters
/// 
///
///@{

struct mip_filter_antenna_offset_command
{
    enum mip_function_selector                        function;
    float                                             offset[3];
};
size_t insert_mip_filter_antenna_offset_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_antenna_offset_command* self);
size_t extract_mip_filter_antenna_offset_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_antenna_offset_command* self);

struct mip_filter_antenna_offset_response
{
    float                                             offset[3];
};
size_t insert_mip_filter_antenna_offset_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_antenna_offset_response* self);
size_t extract_mip_filter_antenna_offset_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_antenna_offset_response* self);

mip_cmd_result write_mip_filter_antenna_offset(struct mip_interface* device, const float* offset);
mip_cmd_result read_mip_filter_antenna_offset(struct mip_interface* device, float* offset);
mip_cmd_result save_mip_filter_antenna_offset(struct mip_interface* device);
mip_cmd_result load_mip_filter_antenna_offset(struct mip_interface* device);
mip_cmd_result default_mip_filter_antenna_offset(struct mip_interface* device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup gnss_source  GNSS Aiding Source Control
/// Control the source of GNSS information used to update the Kalman Filter.
/// 
/// Changing the GNSS source while the sensor is in the "running" state may temporarily place
/// it back in the "init" state until the new source of GNSS data is received.
/// 
///
///@{

enum mip_filter_gnss_source_command_gnss_source
{
    MIP_FILTER_GNSS_SOURCE_COMMAND_GNSS_SOURCE_ALL_INT = 1,  ///<  All internal receivers
    MIP_FILTER_GNSS_SOURCE_COMMAND_GNSS_SOURCE_EXT     = 2,  ///<  External GNSS messages provided by user
    MIP_FILTER_GNSS_SOURCE_COMMAND_GNSS_SOURCE_INT_1   = 3,  ///<  Internal GNSS Receiver 1 only
    MIP_FILTER_GNSS_SOURCE_COMMAND_GNSS_SOURCE_INT_2   = 4,  ///<  Internal GNSS Receiver 2 only
};
size_t insert_mip_filter_gnss_source_command_gnss_source(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_filter_gnss_source_command_gnss_source self);
size_t extract_mip_filter_gnss_source_command_gnss_source(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_filter_gnss_source_command_gnss_source* self);

struct mip_filter_gnss_source_command
{
    enum mip_function_selector                        function;
    enum mip_filter_gnss_source_command_gnss_source   source;
};
size_t insert_mip_filter_gnss_source_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_gnss_source_command* self);
size_t extract_mip_filter_gnss_source_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_gnss_source_command* self);

struct mip_filter_gnss_source_response
{
    enum mip_filter_gnss_source_command_gnss_source   source;
};
size_t insert_mip_filter_gnss_source_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_gnss_source_response* self);
size_t extract_mip_filter_gnss_source_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_gnss_source_response* self);

mip_cmd_result write_mip_filter_gnss_source(struct mip_interface* device, enum mip_filter_gnss_source_command_gnss_source source);
mip_cmd_result read_mip_filter_gnss_source(struct mip_interface* device, enum mip_filter_gnss_source_command_gnss_source* source);
mip_cmd_result save_mip_filter_gnss_source(struct mip_interface* device);
mip_cmd_result load_mip_filter_gnss_source(struct mip_interface* device);
mip_cmd_result default_mip_filter_gnss_source(struct mip_interface* device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup heading_source  Heading Aiding Source Control
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

enum mip_filter_heading_source_command_heading_source
{
    MIP_FILTER_HEADING_SOURCE_COMMAND_HEADING_SOURCE_NONE                          = 0,  ///<  See note 3
    MIP_FILTER_HEADING_SOURCE_COMMAND_HEADING_SOURCE_MAG                           = 1,  ///<  
    MIP_FILTER_HEADING_SOURCE_COMMAND_HEADING_SOURCE_GNSS_VEL                      = 2,  ///<  Seen notes 1,2
    MIP_FILTER_HEADING_SOURCE_COMMAND_HEADING_SOURCE_EXTERNAL                      = 3,  ///<  
    MIP_FILTER_HEADING_SOURCE_COMMAND_HEADING_SOURCE_GNSS_VEL_AND_MAG              = 4,  ///<  
    MIP_FILTER_HEADING_SOURCE_COMMAND_HEADING_SOURCE_GNSS_VEL_AND_EXTERNAL         = 5,  ///<  
    MIP_FILTER_HEADING_SOURCE_COMMAND_HEADING_SOURCE_MAG_AND_EXTERNAL              = 6,  ///<  
    MIP_FILTER_HEADING_SOURCE_COMMAND_HEADING_SOURCE_GNSS_VEL_AND_MAG_AND_EXTERNAL = 7,  ///<  
};
size_t insert_mip_filter_heading_source_command_heading_source(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_filter_heading_source_command_heading_source self);
size_t extract_mip_filter_heading_source_command_heading_source(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_filter_heading_source_command_heading_source* self);

struct mip_filter_heading_source_command
{
    enum mip_function_selector                        function;
    enum mip_filter_heading_source_command_heading_source source;
};
size_t insert_mip_filter_heading_source_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_heading_source_command* self);
size_t extract_mip_filter_heading_source_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_heading_source_command* self);

struct mip_filter_heading_source_response
{
    enum mip_filter_heading_source_command_heading_source source;
};
size_t insert_mip_filter_heading_source_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_heading_source_response* self);
size_t extract_mip_filter_heading_source_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_heading_source_response* self);

mip_cmd_result write_mip_filter_heading_source(struct mip_interface* device, enum mip_filter_heading_source_command_heading_source source);
mip_cmd_result read_mip_filter_heading_source(struct mip_interface* device, enum mip_filter_heading_source_command_heading_source* source);
mip_cmd_result save_mip_filter_heading_source(struct mip_interface* device);
mip_cmd_result load_mip_filter_heading_source(struct mip_interface* device);
mip_cmd_result default_mip_filter_heading_source(struct mip_interface* device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup altitude_aiding  Altitude Aiding Control
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

struct mip_filter_altitude_aiding_command
{
    enum mip_function_selector                        function;
    uint8_t                                           aiding_selector;
};
size_t insert_mip_filter_altitude_aiding_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_altitude_aiding_command* self);
size_t extract_mip_filter_altitude_aiding_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_altitude_aiding_command* self);

struct mip_filter_altitude_aiding_response
{
    uint8_t                                           aiding_selector;
};
size_t insert_mip_filter_altitude_aiding_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_altitude_aiding_response* self);
size_t extract_mip_filter_altitude_aiding_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_altitude_aiding_response* self);

mip_cmd_result write_mip_filter_altitude_aiding(struct mip_interface* device, uint8_t aiding_selector);
mip_cmd_result read_mip_filter_altitude_aiding(struct mip_interface* device, uint8_t* aiding_selector);
mip_cmd_result save_mip_filter_altitude_aiding(struct mip_interface* device);
mip_cmd_result load_mip_filter_altitude_aiding(struct mip_interface* device);
mip_cmd_result default_mip_filter_altitude_aiding(struct mip_interface* device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup auto_zupt  Zero Velocity Update Control
/// Zero Velocity Update
/// The ZUPT is triggered when the scalar magnitude of the GNSS reported velocity vector is equal-to or less than the threshold value.
/// The device will NACK threshold values that are less than zero (i.e.negative.)
///
///@{

struct mip_filter_auto_zupt_command
{
    enum mip_function_selector                        function;
    uint8_t                                           enable;
    float                                             threshold;
};
size_t insert_mip_filter_auto_zupt_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_auto_zupt_command* self);
size_t extract_mip_filter_auto_zupt_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_auto_zupt_command* self);

struct mip_filter_auto_zupt_response
{
    uint8_t                                           enable;
    float                                             threshold;
};
size_t insert_mip_filter_auto_zupt_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_auto_zupt_response* self);
size_t extract_mip_filter_auto_zupt_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_auto_zupt_response* self);

mip_cmd_result write_mip_filter_auto_zupt(struct mip_interface* device, uint8_t enable, float threshold);
mip_cmd_result read_mip_filter_auto_zupt(struct mip_interface* device, uint8_t* enable, float* threshold);
mip_cmd_result save_mip_filter_auto_zupt(struct mip_interface* device);
mip_cmd_result load_mip_filter_auto_zupt(struct mip_interface* device);
mip_cmd_result default_mip_filter_auto_zupt(struct mip_interface* device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup auto_angular_zupt  Zero Angular Rate Update Control
/// Zero Angular Rate Update
/// The ZUPT is triggered when the scalar magnitude of the angular rate vector is equal-to or less than the threshold value.
/// The device will NACK threshold values that are less than zero (i.e.negative.)
///
///@{

struct mip_filter_auto_angular_zupt_command
{
    enum mip_function_selector                        function;
    uint8_t                                           enable;
    float                                             threshold;
};
size_t insert_mip_filter_auto_angular_zupt_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_auto_angular_zupt_command* self);
size_t extract_mip_filter_auto_angular_zupt_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_auto_angular_zupt_command* self);

struct mip_filter_auto_angular_zupt_response
{
    uint8_t                                           enable;
    float                                             threshold;
};
size_t insert_mip_filter_auto_angular_zupt_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_auto_angular_zupt_response* self);
size_t extract_mip_filter_auto_angular_zupt_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_auto_angular_zupt_response* self);

mip_cmd_result write_mip_filter_auto_angular_zupt(struct mip_interface* device, uint8_t enable, float threshold);
mip_cmd_result read_mip_filter_auto_angular_zupt(struct mip_interface* device, uint8_t* enable, float* threshold);
mip_cmd_result save_mip_filter_auto_angular_zupt(struct mip_interface* device);
mip_cmd_result load_mip_filter_auto_angular_zupt(struct mip_interface* device);
mip_cmd_result default_mip_filter_auto_angular_zupt(struct mip_interface* device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup commanded_zupt  Commanded Zero Veloicty Update
/// Commanded Zero Velocity Update
/// Please see the device user manual for the maximum rate of this message.
///
///@{

struct mip_filter_commanded_zupt_command
{
};
size_t insert_mip_filter_commanded_zupt_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_commanded_zupt_command* self);
size_t extract_mip_filter_commanded_zupt_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_commanded_zupt_command* self);

mip_cmd_result mip_filter_commanded_zupt(struct mip_interface* device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup commanded_angular_zupt  Commanded Zero Angular Rate Update
/// Commanded Zero Angular Rate Update
/// Please see the device user manual for the maximum rate of this message.
///
///@{

struct mip_filter_commanded_angular_zupt_command
{
};
size_t insert_mip_filter_commanded_angular_zupt_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_commanded_angular_zupt_command* self);
size_t extract_mip_filter_commanded_angular_zupt_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_commanded_angular_zupt_command* self);

mip_cmd_result mip_filter_commanded_angular_zupt(struct mip_interface* device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup aiding_measurement_enable  Aiding Measurement Control
/// Enables / disables the specified aiding measurement source.
/// 
/// 
///
///@{

enum mip_filter_aiding_measurement_enable_command_aiding_source
{
    MIP_FILTER_AIDING_MEASUREMENT_ENABLE_COMMAND_AIDING_SOURCE_GNSS_POS_VEL     = 0,  ///<  GNSS Position and Velocity
    MIP_FILTER_AIDING_MEASUREMENT_ENABLE_COMMAND_AIDING_SOURCE_GNSS_HEADING     = 1,  ///<  GNSS Heading (dual antenna)
    MIP_FILTER_AIDING_MEASUREMENT_ENABLE_COMMAND_AIDING_SOURCE_ALTIMETER        = 2,  ///<  Altimeter
    MIP_FILTER_AIDING_MEASUREMENT_ENABLE_COMMAND_AIDING_SOURCE_SPEED            = 3,  ///<  Speed sensor / Odometer
    MIP_FILTER_AIDING_MEASUREMENT_ENABLE_COMMAND_AIDING_SOURCE_MAGNETOMETER     = 4,  ///<  Magnetometer
    MIP_FILTER_AIDING_MEASUREMENT_ENABLE_COMMAND_AIDING_SOURCE_EXTERNAL_HEADING = 5,  ///<  External heading input
    MIP_FILTER_AIDING_MEASUREMENT_ENABLE_COMMAND_AIDING_SOURCE_ALL              = 65535,  ///<  Save/load/reset all options
};
size_t insert_mip_filter_aiding_measurement_enable_command_aiding_source(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_filter_aiding_measurement_enable_command_aiding_source self);
size_t extract_mip_filter_aiding_measurement_enable_command_aiding_source(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_filter_aiding_measurement_enable_command_aiding_source* self);

struct mip_filter_aiding_measurement_enable_command
{
    enum mip_function_selector                        function;
    enum mip_filter_aiding_measurement_enable_command_aiding_source aiding_source;
    bool                                              enable;
};
size_t insert_mip_filter_aiding_measurement_enable_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_aiding_measurement_enable_command* self);
size_t extract_mip_filter_aiding_measurement_enable_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_aiding_measurement_enable_command* self);

struct mip_filter_aiding_measurement_enable_response
{
    enum mip_filter_aiding_measurement_enable_command_aiding_source aiding_source;
    bool                                              enable;
};
size_t insert_mip_filter_aiding_measurement_enable_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_aiding_measurement_enable_response* self);
size_t extract_mip_filter_aiding_measurement_enable_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_aiding_measurement_enable_response* self);

mip_cmd_result write_mip_filter_aiding_measurement_enable(struct mip_interface* device, enum mip_filter_aiding_measurement_enable_command_aiding_source aiding_source, bool enable);
mip_cmd_result read_mip_filter_aiding_measurement_enable(struct mip_interface* device, enum mip_filter_aiding_measurement_enable_command_aiding_source aiding_source, bool* enable);
mip_cmd_result save_mip_filter_aiding_measurement_enable(struct mip_interface* device, enum mip_filter_aiding_measurement_enable_command_aiding_source aiding_source);
mip_cmd_result load_mip_filter_aiding_measurement_enable(struct mip_interface* device, enum mip_filter_aiding_measurement_enable_command_aiding_source aiding_source);
mip_cmd_result default_mip_filter_aiding_measurement_enable(struct mip_interface* device, enum mip_filter_aiding_measurement_enable_command_aiding_source aiding_source);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup run  Run Navigation Filter
/// Manual run command.
/// 
/// If the initialization configuration has the "wait_for_run_command" option enabled, the filter will wait until it receives this command before commencing integration and enabling the Kalman filter. Prior to the receipt of this command, the filter will remain in the filter initialization mode.
///
///@{

struct mip_filter_run_command
{
};
size_t insert_mip_filter_run_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_run_command* self);
size_t extract_mip_filter_run_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_run_command* self);

mip_cmd_result mip_filter_run(struct mip_interface* device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup kinematic_constraint  Kinematic Constraint Control
/// Controls kinematic constraint model selection for the navigation filter.
/// 
/// See manual for explanation of how the kinematic constraints are applied.
///
///@{

struct mip_filter_kinematic_constraint_command
{
    enum mip_function_selector                        function;
    uint8_t                                           acceleration_constraint_selection;
    uint8_t                                           velocity_constraint_selection;
    uint8_t                                           angular_constraint_selection;
};
size_t insert_mip_filter_kinematic_constraint_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_kinematic_constraint_command* self);
size_t extract_mip_filter_kinematic_constraint_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_kinematic_constraint_command* self);

struct mip_filter_kinematic_constraint_response
{
    uint8_t                                           acceleration_constraint_selection;
    uint8_t                                           velocity_constraint_selection;
    uint8_t                                           angular_constraint_selection;
};
size_t insert_mip_filter_kinematic_constraint_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_kinematic_constraint_response* self);
size_t extract_mip_filter_kinematic_constraint_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_kinematic_constraint_response* self);

mip_cmd_result write_mip_filter_kinematic_constraint(struct mip_interface* device, uint8_t acceleration_constraint_selection, uint8_t velocity_constraint_selection, uint8_t angular_constraint_selection);
mip_cmd_result read_mip_filter_kinematic_constraint(struct mip_interface* device, uint8_t* acceleration_constraint_selection, uint8_t* velocity_constraint_selection, uint8_t* angular_constraint_selection);
mip_cmd_result save_mip_filter_kinematic_constraint(struct mip_interface* device);
mip_cmd_result load_mip_filter_kinematic_constraint(struct mip_interface* device);
mip_cmd_result default_mip_filter_kinematic_constraint(struct mip_interface* device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup initialization_configuration  Navigation Filter Initialization
/// Controls the source and values used for initial conditions of the navigation solution.
/// 
/// Notes: Initial conditions are the position, velocity, and attitude of the platform used when the filter starts running or is reset.
/// For the user specified position array, the units are meters if the ECEF frame is selected, and degrees latitude, degrees longitude, and meters above ellipsoid if the latitude/longitude/height frame is selected.
/// For the user specified velocity array, the units are meters per second, but the reference frame depends on the reference frame selector (ECEF or NED).
///
///@{

enum mip_filter_initialization_configuration_command_initial_condition_source
{
    MIP_FILTER_INITIALIZATION_CONFIGURATION_COMMAND_INITIAL_CONDITION_SOURCE_AUTO_POS_VEL_ATT        = 0,  ///<  Automatic position, velocity and attitude
    MIP_FILTER_INITIALIZATION_CONFIGURATION_COMMAND_INITIAL_CONDITION_SOURCE_AUTO_POS_VEL_PITCH_ROLL = 1,  ///<  Automatic position and velocity, automatic pitch and roll, and user-specified heading
    MIP_FILTER_INITIALIZATION_CONFIGURATION_COMMAND_INITIAL_CONDITION_SOURCE_AUTO_POS_VEL            = 2,  ///<  Automatic position and velocity, with fully user-specified attitude
    MIP_FILTER_INITIALIZATION_CONFIGURATION_COMMAND_INITIAL_CONDITION_SOURCE_MANUAL                  = 3,  ///<  User-specified position, velocity, and attitude.
};
size_t insert_mip_filter_initialization_configuration_command_initial_condition_source(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_filter_initialization_configuration_command_initial_condition_source self);
size_t extract_mip_filter_initialization_configuration_command_initial_condition_source(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_filter_initialization_configuration_command_initial_condition_source* self);

enum mip_filter_initialization_configuration_command_alignment_selector
{
    MIP_FILTER_INITIALIZATION_CONFIGURATION_COMMAND_ALIGNMENT_SELECTOR_DUAL_ANTENNA = 0x01,
    MIP_FILTER_INITIALIZATION_CONFIGURATION_COMMAND_ALIGNMENT_SELECTOR_KINEMATIC    = 0x02,
    MIP_FILTER_INITIALIZATION_CONFIGURATION_COMMAND_ALIGNMENT_SELECTOR_MAGNETOMETER = 0x04,
};
size_t insert_mip_filter_initialization_configuration_command_alignment_selector(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_filter_initialization_configuration_command_alignment_selector self);
size_t extract_mip_filter_initialization_configuration_command_alignment_selector(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_filter_initialization_configuration_command_alignment_selector* self);

struct mip_filter_initialization_configuration_command
{
    enum mip_function_selector                        function;
    uint8_t                                           wait_for_run_command;
    enum mip_filter_initialization_configuration_command_initial_condition_source initial_cond_src;
    enum mip_filter_initialization_configuration_command_alignment_selector auto_heading_alignment_selector;
    float                                             initial_heading;
    float                                             initial_pitch;
    float                                             initial_roll;
    float                                             initial_position[3];
    float                                             initial_velocity[3];
    enum mip_filter_reference_frame                   reference_frame_selector;
};
size_t insert_mip_filter_initialization_configuration_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_initialization_configuration_command* self);
size_t extract_mip_filter_initialization_configuration_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_initialization_configuration_command* self);

struct mip_filter_initialization_configuration_response
{
    uint8_t                                           wait_for_run_command;
    enum mip_filter_initialization_configuration_command_initial_condition_source initial_cond_src;
    enum mip_filter_initialization_configuration_command_alignment_selector auto_heading_alignment_selector;
    float                                             initial_heading;
    float                                             initial_pitch;
    float                                             initial_roll;
    float                                             initial_position[3];
    float                                             initial_velocity[3];
    enum mip_filter_reference_frame                   reference_frame_selector;
};
size_t insert_mip_filter_initialization_configuration_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_initialization_configuration_response* self);
size_t extract_mip_filter_initialization_configuration_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_initialization_configuration_response* self);

mip_cmd_result write_mip_filter_initialization_configuration(struct mip_interface* device, uint8_t wait_for_run_command, enum mip_filter_initialization_configuration_command_initial_condition_source initial_cond_src, enum mip_filter_initialization_configuration_command_alignment_selector auto_heading_alignment_selector, float initial_heading, float initial_pitch, float initial_roll, const float* initial_position, const float* initial_velocity, enum mip_filter_reference_frame reference_frame_selector);
mip_cmd_result read_mip_filter_initialization_configuration(struct mip_interface* device, uint8_t* wait_for_run_command, enum mip_filter_initialization_configuration_command_initial_condition_source* initial_cond_src, enum mip_filter_initialization_configuration_command_alignment_selector* auto_heading_alignment_selector, float* initial_heading, float* initial_pitch, float* initial_roll, float* initial_position, float* initial_velocity, enum mip_filter_reference_frame* reference_frame_selector);
mip_cmd_result save_mip_filter_initialization_configuration(struct mip_interface* device);
mip_cmd_result load_mip_filter_initialization_configuration(struct mip_interface* device);
mip_cmd_result default_mip_filter_initialization_configuration(struct mip_interface* device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup adaptive_filter_options  Adaptive Filter Control
/// Configures the basic setup for auto-adaptive filtering. See product manual for a detailed description of this feature.
///
///@{

struct mip_filter_adaptive_filter_options_command
{
    enum mip_function_selector                        function;
    uint8_t                                           level;
    uint16_t                                          time_limit;
};
size_t insert_mip_filter_adaptive_filter_options_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_adaptive_filter_options_command* self);
size_t extract_mip_filter_adaptive_filter_options_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_adaptive_filter_options_command* self);

struct mip_filter_adaptive_filter_options_response
{
    uint8_t                                           level;
    uint16_t                                          time_limit;
};
size_t insert_mip_filter_adaptive_filter_options_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_adaptive_filter_options_response* self);
size_t extract_mip_filter_adaptive_filter_options_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_adaptive_filter_options_response* self);

mip_cmd_result write_mip_filter_adaptive_filter_options(struct mip_interface* device, uint8_t level, uint16_t time_limit);
mip_cmd_result read_mip_filter_adaptive_filter_options(struct mip_interface* device, uint8_t* level, uint16_t* time_limit);
mip_cmd_result save_mip_filter_adaptive_filter_options(struct mip_interface* device);
mip_cmd_result load_mip_filter_adaptive_filter_options(struct mip_interface* device);
mip_cmd_result default_mip_filter_adaptive_filter_options(struct mip_interface* device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup multi_antenna_offset  GNSS Multi-Antenna Offset Control
/// Set the antenna lever arm.
/// 
/// This command works with devices that utilize multiple antennas.
///
///@{

struct mip_filter_multi_antenna_offset_command
{
    enum mip_function_selector                        function;
    uint8_t                                           receiver_id;
    float                                             antenna_offset[3];
};
size_t insert_mip_filter_multi_antenna_offset_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_multi_antenna_offset_command* self);
size_t extract_mip_filter_multi_antenna_offset_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_multi_antenna_offset_command* self);

struct mip_filter_multi_antenna_offset_response
{
    uint8_t                                           receiver_id;
    float                                             antenna_offset[3];
};
size_t insert_mip_filter_multi_antenna_offset_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_multi_antenna_offset_response* self);
size_t extract_mip_filter_multi_antenna_offset_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_multi_antenna_offset_response* self);

mip_cmd_result write_mip_filter_multi_antenna_offset(struct mip_interface* device, uint8_t receiver_id, const float* antenna_offset);
mip_cmd_result read_mip_filter_multi_antenna_offset(struct mip_interface* device, uint8_t receiver_id, float* antenna_offset);
mip_cmd_result save_mip_filter_multi_antenna_offset(struct mip_interface* device, uint8_t receiver_id);
mip_cmd_result load_mip_filter_multi_antenna_offset(struct mip_interface* device, uint8_t receiver_id);
mip_cmd_result default_mip_filter_multi_antenna_offset(struct mip_interface* device, uint8_t receiver_id);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup rel_pos_configuration  Relative Position Configuration
/// Configure the reference location for filter relative positioning outputs
///
///@{

struct mip_filter_rel_pos_configuration_command
{
    enum mip_function_selector                        function;
    uint8_t                                           source;
    enum mip_filter_reference_frame                   reference_frame_selector;
    double                                            reference_coordinates[3];
};
size_t insert_mip_filter_rel_pos_configuration_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_rel_pos_configuration_command* self);
size_t extract_mip_filter_rel_pos_configuration_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_rel_pos_configuration_command* self);

struct mip_filter_rel_pos_configuration_response
{
    uint8_t                                           source;
    enum mip_filter_reference_frame                   reference_frame_selector;
    double                                            reference_coordinates[3];
};
size_t insert_mip_filter_rel_pos_configuration_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_rel_pos_configuration_response* self);
size_t extract_mip_filter_rel_pos_configuration_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_rel_pos_configuration_response* self);

mip_cmd_result write_mip_filter_rel_pos_configuration(struct mip_interface* device, uint8_t source, enum mip_filter_reference_frame reference_frame_selector, const double* reference_coordinates);
mip_cmd_result read_mip_filter_rel_pos_configuration(struct mip_interface* device, uint8_t* source, enum mip_filter_reference_frame* reference_frame_selector, double* reference_coordinates);
mip_cmd_result save_mip_filter_rel_pos_configuration(struct mip_interface* device);
mip_cmd_result load_mip_filter_rel_pos_configuration(struct mip_interface* device);
mip_cmd_result default_mip_filter_rel_pos_configuration(struct mip_interface* device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup ref_point_lever_arm  Reference point lever arm
/// Lever arm offset with respect to the sensor for the indicated point of reference.
/// This is used to change the location of the indicated point of reference, and will affect filter position and velocity outputs.
/// Changing this setting from default will result in a global position offset that depends on vehicle attitude,
/// and a velocity offset that depends on vehicle attitude and angular rate.
/// The lever arm is defined by a 3-element vector that points from the sensor to the desired reference point, with (x,y,z) components given in the vehicle's reference frame.
/// Note, if the reference point selector is set to VEH (1), this setting will affect the following data fields: (0x82, 0x01), (0x82, 0x02), (0x82, 0x40), (0x82, 0x41), and (0x82, 42)
///
///@{

enum mip_filter_ref_point_lever_arm_command_reference_point_selector
{
    MIP_FILTER_REF_POINT_LEVER_ARM_COMMAND_REFERENCE_POINT_SELECTOR_VEH = 1,  ///<  Defines the origin of the vehicle
};
size_t insert_mip_filter_ref_point_lever_arm_command_reference_point_selector(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_filter_ref_point_lever_arm_command_reference_point_selector self);
size_t extract_mip_filter_ref_point_lever_arm_command_reference_point_selector(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_filter_ref_point_lever_arm_command_reference_point_selector* self);

struct mip_filter_ref_point_lever_arm_command
{
    enum mip_function_selector                        function;
    enum mip_filter_ref_point_lever_arm_command_reference_point_selector ref_point_sel;
    float                                             lever_arm_offset[3];
};
size_t insert_mip_filter_ref_point_lever_arm_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_ref_point_lever_arm_command* self);
size_t extract_mip_filter_ref_point_lever_arm_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_ref_point_lever_arm_command* self);

struct mip_filter_ref_point_lever_arm_response
{
    enum mip_filter_ref_point_lever_arm_command_reference_point_selector ref_point_sel;
    float                                             lever_arm_offset[3];
};
size_t insert_mip_filter_ref_point_lever_arm_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_ref_point_lever_arm_response* self);
size_t extract_mip_filter_ref_point_lever_arm_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_ref_point_lever_arm_response* self);

mip_cmd_result write_mip_filter_ref_point_lever_arm(struct mip_interface* device, enum mip_filter_ref_point_lever_arm_command_reference_point_selector ref_point_sel, const float* lever_arm_offset);
mip_cmd_result read_mip_filter_ref_point_lever_arm(struct mip_interface* device, enum mip_filter_ref_point_lever_arm_command_reference_point_selector* ref_point_sel, float* lever_arm_offset);
mip_cmd_result save_mip_filter_ref_point_lever_arm(struct mip_interface* device);
mip_cmd_result load_mip_filter_ref_point_lever_arm(struct mip_interface* device);
mip_cmd_result default_mip_filter_ref_point_lever_arm(struct mip_interface* device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup speed_measurement  Input speed measurement
/// Speed aiding measurement, where speed is defined as rate of motion along the vehicle's x-axis direction.
/// Can be used by an external odometer/speedometer, for example.
/// This command cannot be used if the internal odometer is configured.
///
///@{

struct mip_filter_speed_measurement_command
{
    uint8_t                                           source;
    float                                             time_of_week;
    float                                             speed;
    float                                             speed_uncertainty;
};
size_t insert_mip_filter_speed_measurement_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_speed_measurement_command* self);
size_t extract_mip_filter_speed_measurement_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_speed_measurement_command* self);

mip_cmd_result mip_filter_speed_measurement(struct mip_interface* device, uint8_t source, float time_of_week, float speed, float speed_uncertainty);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup speed_lever_arm  Measurement speed lever arm
/// Lever arm offset for speed measurements.
/// This is used to compensate for an off-center measurement point
/// having a different speed due to rotation of the vehicle.
/// The typical use case for this would be an odometer attached to a wheel
/// on a standard 4-wheeled vehicle. If the odometer is on the left wheel,
/// it will report higher speed on right turns and lower speed on left turns.
/// This is because the outside edge of the curve is longer than the inside edge.
///
///@{

struct mip_filter_speed_lever_arm_command
{
    enum mip_function_selector                        function;
    uint8_t                                           source;
    float                                             lever_arm_offset[3];
};
size_t insert_mip_filter_speed_lever_arm_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_speed_lever_arm_command* self);
size_t extract_mip_filter_speed_lever_arm_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_speed_lever_arm_command* self);

struct mip_filter_speed_lever_arm_response
{
    uint8_t                                           source;
    float                                             lever_arm_offset[3];
};
size_t insert_mip_filter_speed_lever_arm_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_speed_lever_arm_response* self);
size_t extract_mip_filter_speed_lever_arm_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_speed_lever_arm_response* self);

mip_cmd_result write_mip_filter_speed_lever_arm(struct mip_interface* device, uint8_t source, const float* lever_arm_offset);
mip_cmd_result read_mip_filter_speed_lever_arm(struct mip_interface* device, uint8_t source, float* lever_arm_offset);
mip_cmd_result save_mip_filter_speed_lever_arm(struct mip_interface* device, uint8_t source);
mip_cmd_result load_mip_filter_speed_lever_arm(struct mip_interface* device, uint8_t source);
mip_cmd_result default_mip_filter_speed_lever_arm(struct mip_interface* device, uint8_t source);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup wheeled_vehicle_constraint_control  Wheeled Vehicle Constraint Control
/// Configure the wheeled vehicle kinematic constraint.
/// 
/// When enabled, the filter uses the assumption that velocity is constrained to the primary vehicle axis.
/// By convention, the primary vehicle axis is the vehicle X-axis (note: the sensor may be physically installed in
/// any orientation on the vehicle if the appropriate mounting transformation has been specified).
/// This constraint will typically improve heading estimates for vehicles where the assumption is valid, such
/// as an automobile, particulary when GNSS coverage is intermittent.
///
///@{

struct mip_filter_wheeled_vehicle_constraint_control_command
{
    enum mip_function_selector                        function;
    uint8_t                                           enable;
};
size_t insert_mip_filter_wheeled_vehicle_constraint_control_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_wheeled_vehicle_constraint_control_command* self);
size_t extract_mip_filter_wheeled_vehicle_constraint_control_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_wheeled_vehicle_constraint_control_command* self);

struct mip_filter_wheeled_vehicle_constraint_control_response
{
    uint8_t                                           enable;
};
size_t insert_mip_filter_wheeled_vehicle_constraint_control_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_wheeled_vehicle_constraint_control_response* self);
size_t extract_mip_filter_wheeled_vehicle_constraint_control_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_wheeled_vehicle_constraint_control_response* self);

mip_cmd_result write_mip_filter_wheeled_vehicle_constraint_control(struct mip_interface* device, uint8_t enable);
mip_cmd_result read_mip_filter_wheeled_vehicle_constraint_control(struct mip_interface* device, uint8_t* enable);
mip_cmd_result save_mip_filter_wheeled_vehicle_constraint_control(struct mip_interface* device);
mip_cmd_result load_mip_filter_wheeled_vehicle_constraint_control(struct mip_interface* device);
mip_cmd_result default_mip_filter_wheeled_vehicle_constraint_control(struct mip_interface* device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup vertical_gyro_constraint_control  Vertical Gyro Constraint Control
/// Configure the vertical gyro kinematic constraint.
/// 
/// When enabled and no valid GNSS measurements are available, the filter uses the accelerometers to track pitch
/// and roll under the assumption that the sensor platform is not undergoing linear acceleration.
/// This constraint is useful to maintain accurate pitch and roll during GNSS signal outages.
///
///@{

struct mip_filter_vertical_gyro_constraint_control_command
{
    enum mip_function_selector                        function;
    uint8_t                                           enable;
};
size_t insert_mip_filter_vertical_gyro_constraint_control_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_vertical_gyro_constraint_control_command* self);
size_t extract_mip_filter_vertical_gyro_constraint_control_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_vertical_gyro_constraint_control_command* self);

struct mip_filter_vertical_gyro_constraint_control_response
{
    uint8_t                                           enable;
};
size_t insert_mip_filter_vertical_gyro_constraint_control_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_vertical_gyro_constraint_control_response* self);
size_t extract_mip_filter_vertical_gyro_constraint_control_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_vertical_gyro_constraint_control_response* self);

mip_cmd_result write_mip_filter_vertical_gyro_constraint_control(struct mip_interface* device, uint8_t enable);
mip_cmd_result read_mip_filter_vertical_gyro_constraint_control(struct mip_interface* device, uint8_t* enable);
mip_cmd_result save_mip_filter_vertical_gyro_constraint_control(struct mip_interface* device);
mip_cmd_result load_mip_filter_vertical_gyro_constraint_control(struct mip_interface* device);
mip_cmd_result default_mip_filter_vertical_gyro_constraint_control(struct mip_interface* device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup gnss_antenna_cal_control  GNSS Antenna Offset Calibration Control
/// Configure the GNSS antenna lever arm calibration.
/// 
/// When enabled, the filter will enable lever arm error tracking, up to the maximum offset specified.
///
///@{

struct mip_filter_gnss_antenna_cal_control_command
{
    enum mip_function_selector                        function;
    uint8_t                                           enable;
    float                                             max_offset;
};
size_t insert_mip_filter_gnss_antenna_cal_control_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_gnss_antenna_cal_control_command* self);
size_t extract_mip_filter_gnss_antenna_cal_control_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_gnss_antenna_cal_control_command* self);

struct mip_filter_gnss_antenna_cal_control_response
{
    uint8_t                                           enable;
    float                                             max_offset;
};
size_t insert_mip_filter_gnss_antenna_cal_control_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_gnss_antenna_cal_control_response* self);
size_t extract_mip_filter_gnss_antenna_cal_control_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_gnss_antenna_cal_control_response* self);

mip_cmd_result write_mip_filter_gnss_antenna_cal_control(struct mip_interface* device, uint8_t enable, float max_offset);
mip_cmd_result read_mip_filter_gnss_antenna_cal_control(struct mip_interface* device, uint8_t* enable, float* max_offset);
mip_cmd_result save_mip_filter_gnss_antenna_cal_control(struct mip_interface* device);
mip_cmd_result load_mip_filter_gnss_antenna_cal_control(struct mip_interface* device);
mip_cmd_result default_mip_filter_gnss_antenna_cal_control(struct mip_interface* device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup magnetic_declination_source  Magnetic Field Declination Source Control
/// Source for magnetic declination angle, and user supplied value for manual selection.
///
///@{

struct mip_filter_magnetic_declination_source_command
{
    enum mip_function_selector                        function;
    enum mip_filter_mag_declination_source            source;
    float                                             declination;
};
size_t insert_mip_filter_magnetic_declination_source_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_magnetic_declination_source_command* self);
size_t extract_mip_filter_magnetic_declination_source_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_magnetic_declination_source_command* self);

struct mip_filter_magnetic_declination_source_response
{
    enum mip_filter_mag_declination_source            source;
    float                                             declination;
};
size_t insert_mip_filter_magnetic_declination_source_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_magnetic_declination_source_response* self);
size_t extract_mip_filter_magnetic_declination_source_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_magnetic_declination_source_response* self);

mip_cmd_result write_mip_filter_magnetic_declination_source(struct mip_interface* device, enum mip_filter_mag_declination_source source, float declination);
mip_cmd_result read_mip_filter_magnetic_declination_source(struct mip_interface* device, enum mip_filter_mag_declination_source* source, float* declination);
mip_cmd_result save_mip_filter_magnetic_declination_source(struct mip_interface* device);
mip_cmd_result load_mip_filter_magnetic_declination_source(struct mip_interface* device);
mip_cmd_result default_mip_filter_magnetic_declination_source(struct mip_interface* device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup set_initial_heading  Set Initial Heading Control
/// Set the initial heading angle.
/// 
/// The estimation filter will reset the heading estimate to provided value. If the product supports magnetometer aiding and this feature has been enabled, the heading
/// argument will be ignored and the filter will initialize using the inferred magnetic heading.
///
///@{

struct mip_filter_set_initial_heading_command
{
    float                                             heading;
};
size_t insert_mip_filter_set_initial_heading_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_set_initial_heading_command* self);
size_t extract_mip_filter_set_initial_heading_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_set_initial_heading_command* self);

mip_cmd_result mip_filter_set_initial_heading(struct mip_interface* device, float heading);
///@}
///

///@}
///@}
///
////////////////////////////////////////////////////////////////////////////////

#ifdef __cplusplus
} // extern "C"
} // namespace C
namespace FilterCommands {


struct reset : C::mip_filter_reset_command
{
    static const uint8_t descriptorSet = MIP_FILTER_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_FILTER_RESET_FILTER;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::insert_mip_filter_reset_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_filter_reset_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = false;
    
};
MipCmdResult reset(C::mip_interface& device)
{
    return C::mip_filter_reset(&device);
}



struct setInitialAttitude : C::mip_filter_set_initial_attitude_command
{
    static const uint8_t descriptorSet = MIP_FILTER_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_FILTER_SET_INITIAL_ATTITUDE;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::insert_mip_filter_set_initial_attitude_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_filter_set_initial_attitude_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = false;
    
};
MipCmdResult setInitialAttitude(C::mip_interface& device, float roll, float pitch, float heading)
{
    return C::mip_filter_set_initial_attitude(&device, roll, pitch, heading);
}



struct estimationControl : C::mip_filter_estimation_control_command
{
    static const uint8_t descriptorSet = MIP_FILTER_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_FILTER_ESTIMATION_CONTROL_FLAGS;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::insert_mip_filter_estimation_control_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_filter_estimation_control_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = true;
    static const bool canWrite = true;
    static const bool canRead = true;
    static const bool canSave = true;
    static const bool canLoad = true;
    static const bool canReset = true;
    
    struct Response : C::mip_filter_estimation_control_response
    {
        static const uint8_t descriptorSet = MIP_FILTER_CMD_DESC_SET;
        static const uint8_t fieldDescriptor = MIP_REPLY_DESC_FILTER_ESTIMATION_CONTROL_FLAGS;
        
        size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::insert_mip_filter_estimation_control_response(buffer, bufferSize, offset, this);
        }
        size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::extract_mip_filter_estimation_control_response(buffer, bufferSize, offset, this);
        }
    };
    
};
MipCmdResult writeEstimationcontrol(C::mip_interface& device, C::mip_filter_estimation_control_command_enable_flags enable)
{
    return C::write_mip_filter_estimation_control(&device, enable);
}
MipCmdResult readEstimationcontrol(C::mip_interface& device, enum C::mip_filter_estimation_control_command_enable_flags& enable)
{
    return C::read_mip_filter_estimation_control(&device, &enable);
}
MipCmdResult saveEstimationcontrol(C::mip_interface& device)
{
    return C::save_mip_filter_estimation_control(&device);
}
MipCmdResult loadEstimationcontrol(C::mip_interface& device)
{
    return C::load_mip_filter_estimation_control(&device);
}
MipCmdResult defaultEstimationcontrol(C::mip_interface& device)
{
    return C::default_mip_filter_estimation_control(&device);
}



struct externalGnssUpdate : C::mip_filter_external_gnss_update_command
{
    static const uint8_t descriptorSet = MIP_FILTER_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_FILTER_EXTERNAL_GNSS_UPDATE;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::insert_mip_filter_external_gnss_update_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_filter_external_gnss_update_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = false;
    
};
MipCmdResult externalGnssUpdate(C::mip_interface& device, double gps_time, uint16_t gps_week, double latitude, double longitude, double height, const float* velocity, const float* pos_uncertainty, const float* vel_uncertainty)
{
    return C::mip_filter_external_gnss_update(&device, gps_time, gps_week, latitude, longitude, height, velocity, pos_uncertainty, vel_uncertainty);
}



struct externalHeadingUpdate : C::mip_filter_external_heading_update_command
{
    static const uint8_t descriptorSet = MIP_FILTER_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_FILTER_EXTERNAL_HEADING_UPDATE;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::insert_mip_filter_external_heading_update_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_filter_external_heading_update_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = false;
    
};
MipCmdResult externalHeadingUpdate(C::mip_interface& device, float heading, float heading_uncertainty, uint8_t type)
{
    return C::mip_filter_external_heading_update(&device, heading, heading_uncertainty, type);
}



struct externalHeadingUpdateWithTime : C::mip_filter_external_heading_update_with_time_command
{
    static const uint8_t descriptorSet = MIP_FILTER_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_FILTER_EXTERNAL_HEADING_UPDATE_WITH_TIME;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::insert_mip_filter_external_heading_update_with_time_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_filter_external_heading_update_with_time_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = false;
    
};
MipCmdResult externalHeadingUpdateWithTime(C::mip_interface& device, double gps_time, uint16_t gps_week, float heading, float heading_uncertainty, uint8_t type)
{
    return C::mip_filter_external_heading_update_with_time(&device, gps_time, gps_week, heading, heading_uncertainty, type);
}



struct tareOrientation : C::mip_filter_tare_orientation_command
{
    static const uint8_t descriptorSet = MIP_FILTER_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_FILTER_TARE_ORIENTATION;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::insert_mip_filter_tare_orientation_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_filter_tare_orientation_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = true;
    static const bool canWrite = true;
    static const bool canRead = true;
    static const bool canSave = true;
    static const bool canLoad = true;
    static const bool canReset = true;
    
    struct Response : C::mip_filter_tare_orientation_response
    {
        static const uint8_t descriptorSet = MIP_FILTER_CMD_DESC_SET;
        static const uint8_t fieldDescriptor = MIP_REPLY_DESC_FILTER_TARE_ORIENTATION;
        
        size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::insert_mip_filter_tare_orientation_response(buffer, bufferSize, offset, this);
        }
        size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::extract_mip_filter_tare_orientation_response(buffer, bufferSize, offset, this);
        }
    };
    
};
MipCmdResult writeTareorientation(C::mip_interface& device, C::mip_filter_tare_orientation_command_mip_tare_axes axes)
{
    return C::write_mip_filter_tare_orientation(&device, axes);
}
MipCmdResult readTareorientation(C::mip_interface& device, enum C::mip_filter_tare_orientation_command_mip_tare_axes& axes)
{
    return C::read_mip_filter_tare_orientation(&device, &axes);
}
MipCmdResult saveTareorientation(C::mip_interface& device)
{
    return C::save_mip_filter_tare_orientation(&device);
}
MipCmdResult loadTareorientation(C::mip_interface& device)
{
    return C::load_mip_filter_tare_orientation(&device);
}
MipCmdResult defaultTareorientation(C::mip_interface& device)
{
    return C::default_mip_filter_tare_orientation(&device);
}



struct sensorToVehicleRotationEuler : C::mip_filter_sensor_to_vehicle_rotation_euler_command
{
    static const uint8_t descriptorSet = MIP_FILTER_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_ROTATION_EULER;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::insert_mip_filter_sensor_to_vehicle_rotation_euler_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_filter_sensor_to_vehicle_rotation_euler_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = true;
    static const bool canWrite = true;
    static const bool canRead = true;
    static const bool canSave = true;
    static const bool canLoad = true;
    static const bool canReset = true;
    
    struct Response : C::mip_filter_sensor_to_vehicle_rotation_euler_response
    {
        static const uint8_t descriptorSet = MIP_FILTER_CMD_DESC_SET;
        static const uint8_t fieldDescriptor = MIP_REPLY_DESC_FILTER_SENSOR2VEHICLE_ROTATION_EULER;
        
        size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::insert_mip_filter_sensor_to_vehicle_rotation_euler_response(buffer, bufferSize, offset, this);
        }
        size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::extract_mip_filter_sensor_to_vehicle_rotation_euler_response(buffer, bufferSize, offset, this);
        }
    };
    
};
MipCmdResult writeSensortovehiclerotationeuler(C::mip_interface& device, float roll, float pitch, float yaw)
{
    return C::write_mip_filter_sensor_to_vehicle_rotation_euler(&device, roll, pitch, yaw);
}
MipCmdResult readSensortovehiclerotationeuler(C::mip_interface& device, float& roll, float& pitch, float& yaw)
{
    return C::read_mip_filter_sensor_to_vehicle_rotation_euler(&device, &roll, &pitch, &yaw);
}
MipCmdResult saveSensortovehiclerotationeuler(C::mip_interface& device)
{
    return C::save_mip_filter_sensor_to_vehicle_rotation_euler(&device);
}
MipCmdResult loadSensortovehiclerotationeuler(C::mip_interface& device)
{
    return C::load_mip_filter_sensor_to_vehicle_rotation_euler(&device);
}
MipCmdResult defaultSensortovehiclerotationeuler(C::mip_interface& device)
{
    return C::default_mip_filter_sensor_to_vehicle_rotation_euler(&device);
}



struct sensorToVehicleRotationDcm : C::mip_filter_sensor_to_vehicle_rotation_dcm_command
{
    static const uint8_t descriptorSet = MIP_FILTER_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_ROTATION_DCM;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::insert_mip_filter_sensor_to_vehicle_rotation_dcm_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_filter_sensor_to_vehicle_rotation_dcm_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = true;
    static const bool canWrite = true;
    static const bool canRead = true;
    static const bool canSave = true;
    static const bool canLoad = true;
    static const bool canReset = true;
    
    struct Response : C::mip_filter_sensor_to_vehicle_rotation_dcm_response
    {
        static const uint8_t descriptorSet = MIP_FILTER_CMD_DESC_SET;
        static const uint8_t fieldDescriptor = MIP_REPLY_DESC_FILTER_SENSOR2VEHICLE_ROTATION_DCM;
        
        size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::insert_mip_filter_sensor_to_vehicle_rotation_dcm_response(buffer, bufferSize, offset, this);
        }
        size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::extract_mip_filter_sensor_to_vehicle_rotation_dcm_response(buffer, bufferSize, offset, this);
        }
    };
    
};
MipCmdResult writeSensortovehiclerotationdcm(C::mip_interface& device, const float* dcm)
{
    return C::write_mip_filter_sensor_to_vehicle_rotation_dcm(&device, dcm);
}
MipCmdResult readSensortovehiclerotationdcm(C::mip_interface& device, float* dcm)
{
    return C::read_mip_filter_sensor_to_vehicle_rotation_dcm(&device, dcm);
}
MipCmdResult saveSensortovehiclerotationdcm(C::mip_interface& device)
{
    return C::save_mip_filter_sensor_to_vehicle_rotation_dcm(&device);
}
MipCmdResult loadSensortovehiclerotationdcm(C::mip_interface& device)
{
    return C::load_mip_filter_sensor_to_vehicle_rotation_dcm(&device);
}
MipCmdResult defaultSensortovehiclerotationdcm(C::mip_interface& device)
{
    return C::default_mip_filter_sensor_to_vehicle_rotation_dcm(&device);
}



struct sensorToVehicleRotationQuaternion : C::mip_filter_sensor_to_vehicle_rotation_quaternion_command
{
    static const uint8_t descriptorSet = MIP_FILTER_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_ROTATION_QUATERNION;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::insert_mip_filter_sensor_to_vehicle_rotation_quaternion_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_filter_sensor_to_vehicle_rotation_quaternion_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = true;
    static const bool canWrite = true;
    static const bool canRead = true;
    static const bool canSave = true;
    static const bool canLoad = true;
    static const bool canReset = true;
    
    struct Response : C::mip_filter_sensor_to_vehicle_rotation_quaternion_response
    {
        static const uint8_t descriptorSet = MIP_FILTER_CMD_DESC_SET;
        static const uint8_t fieldDescriptor = MIP_REPLY_DESC_FILTER_SENSOR2VEHICLE_ROTATION_QUATERNION;
        
        size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::insert_mip_filter_sensor_to_vehicle_rotation_quaternion_response(buffer, bufferSize, offset, this);
        }
        size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::extract_mip_filter_sensor_to_vehicle_rotation_quaternion_response(buffer, bufferSize, offset, this);
        }
    };
    
};
MipCmdResult writeSensortovehiclerotationquaternion(C::mip_interface& device, const float* quat)
{
    return C::write_mip_filter_sensor_to_vehicle_rotation_quaternion(&device, quat);
}
MipCmdResult readSensortovehiclerotationquaternion(C::mip_interface& device, float* quat)
{
    return C::read_mip_filter_sensor_to_vehicle_rotation_quaternion(&device, quat);
}
MipCmdResult saveSensortovehiclerotationquaternion(C::mip_interface& device)
{
    return C::save_mip_filter_sensor_to_vehicle_rotation_quaternion(&device);
}
MipCmdResult loadSensortovehiclerotationquaternion(C::mip_interface& device)
{
    return C::load_mip_filter_sensor_to_vehicle_rotation_quaternion(&device);
}
MipCmdResult defaultSensortovehiclerotationquaternion(C::mip_interface& device)
{
    return C::default_mip_filter_sensor_to_vehicle_rotation_quaternion(&device);
}



struct sensorToVehicleOffset : C::mip_filter_sensor_to_vehicle_offset_command
{
    static const uint8_t descriptorSet = MIP_FILTER_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_OFFSET;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::insert_mip_filter_sensor_to_vehicle_offset_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_filter_sensor_to_vehicle_offset_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = true;
    static const bool canWrite = true;
    static const bool canRead = true;
    static const bool canSave = true;
    static const bool canLoad = true;
    static const bool canReset = true;
    
    struct Response : C::mip_filter_sensor_to_vehicle_offset_response
    {
        static const uint8_t descriptorSet = MIP_FILTER_CMD_DESC_SET;
        static const uint8_t fieldDescriptor = MIP_REPLY_DESC_FILTER_SENSOR2VEHICLE_OFFSET;
        
        size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::insert_mip_filter_sensor_to_vehicle_offset_response(buffer, bufferSize, offset, this);
        }
        size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::extract_mip_filter_sensor_to_vehicle_offset_response(buffer, bufferSize, offset, this);
        }
    };
    
};
MipCmdResult writeSensortovehicleoffset(C::mip_interface& device, const float* offset)
{
    return C::write_mip_filter_sensor_to_vehicle_offset(&device, offset);
}
MipCmdResult readSensortovehicleoffset(C::mip_interface& device, float* offset)
{
    return C::read_mip_filter_sensor_to_vehicle_offset(&device, offset);
}
MipCmdResult saveSensortovehicleoffset(C::mip_interface& device)
{
    return C::save_mip_filter_sensor_to_vehicle_offset(&device);
}
MipCmdResult loadSensortovehicleoffset(C::mip_interface& device)
{
    return C::load_mip_filter_sensor_to_vehicle_offset(&device);
}
MipCmdResult defaultSensortovehicleoffset(C::mip_interface& device)
{
    return C::default_mip_filter_sensor_to_vehicle_offset(&device);
}



struct antennaOffset : C::mip_filter_antenna_offset_command
{
    static const uint8_t descriptorSet = MIP_FILTER_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_FILTER_ANTENNA_OFFSET;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::insert_mip_filter_antenna_offset_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_filter_antenna_offset_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = true;
    static const bool canWrite = true;
    static const bool canRead = true;
    static const bool canSave = true;
    static const bool canLoad = true;
    static const bool canReset = true;
    
    struct Response : C::mip_filter_antenna_offset_response
    {
        static const uint8_t descriptorSet = MIP_FILTER_CMD_DESC_SET;
        static const uint8_t fieldDescriptor = MIP_REPLY_DESC_FILTER_ANTENNA_OFFSET;
        
        size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::insert_mip_filter_antenna_offset_response(buffer, bufferSize, offset, this);
        }
        size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::extract_mip_filter_antenna_offset_response(buffer, bufferSize, offset, this);
        }
    };
    
};
MipCmdResult writeAntennaoffset(C::mip_interface& device, const float* offset)
{
    return C::write_mip_filter_antenna_offset(&device, offset);
}
MipCmdResult readAntennaoffset(C::mip_interface& device, float* offset)
{
    return C::read_mip_filter_antenna_offset(&device, offset);
}
MipCmdResult saveAntennaoffset(C::mip_interface& device)
{
    return C::save_mip_filter_antenna_offset(&device);
}
MipCmdResult loadAntennaoffset(C::mip_interface& device)
{
    return C::load_mip_filter_antenna_offset(&device);
}
MipCmdResult defaultAntennaoffset(C::mip_interface& device)
{
    return C::default_mip_filter_antenna_offset(&device);
}



struct gnssSource : C::mip_filter_gnss_source_command
{
    static const uint8_t descriptorSet = MIP_FILTER_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_FILTER_GNSS_SOURCE_CONTROL;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::insert_mip_filter_gnss_source_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_filter_gnss_source_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = true;
    static const bool canWrite = true;
    static const bool canRead = true;
    static const bool canSave = true;
    static const bool canLoad = true;
    static const bool canReset = true;
    
    struct Response : C::mip_filter_gnss_source_response
    {
        static const uint8_t descriptorSet = MIP_FILTER_CMD_DESC_SET;
        static const uint8_t fieldDescriptor = MIP_REPLY_DESC_FILTER_GNSS_SOURCE_CONTROL;
        
        size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::insert_mip_filter_gnss_source_response(buffer, bufferSize, offset, this);
        }
        size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::extract_mip_filter_gnss_source_response(buffer, bufferSize, offset, this);
        }
    };
    
};
MipCmdResult writeGnsssource(C::mip_interface& device, C::mip_filter_gnss_source_command_gnss_source source)
{
    return C::write_mip_filter_gnss_source(&device, source);
}
MipCmdResult readGnsssource(C::mip_interface& device, enum C::mip_filter_gnss_source_command_gnss_source& source)
{
    return C::read_mip_filter_gnss_source(&device, &source);
}
MipCmdResult saveGnsssource(C::mip_interface& device)
{
    return C::save_mip_filter_gnss_source(&device);
}
MipCmdResult loadGnsssource(C::mip_interface& device)
{
    return C::load_mip_filter_gnss_source(&device);
}
MipCmdResult defaultGnsssource(C::mip_interface& device)
{
    return C::default_mip_filter_gnss_source(&device);
}



struct headingSource : C::mip_filter_heading_source_command
{
    static const uint8_t descriptorSet = MIP_FILTER_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_FILTER_HEADING_UPDATE_CONTROL;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::insert_mip_filter_heading_source_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_filter_heading_source_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = true;
    static const bool canWrite = true;
    static const bool canRead = true;
    static const bool canSave = true;
    static const bool canLoad = true;
    static const bool canReset = true;
    
    struct Response : C::mip_filter_heading_source_response
    {
        static const uint8_t descriptorSet = MIP_FILTER_CMD_DESC_SET;
        static const uint8_t fieldDescriptor = MIP_REPLY_DESC_FILTER_HEADING_UPDATE_CONTROL;
        
        size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::insert_mip_filter_heading_source_response(buffer, bufferSize, offset, this);
        }
        size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::extract_mip_filter_heading_source_response(buffer, bufferSize, offset, this);
        }
    };
    
};
MipCmdResult writeHeadingsource(C::mip_interface& device, C::mip_filter_heading_source_command_heading_source source)
{
    return C::write_mip_filter_heading_source(&device, source);
}
MipCmdResult readHeadingsource(C::mip_interface& device, enum C::mip_filter_heading_source_command_heading_source& source)
{
    return C::read_mip_filter_heading_source(&device, &source);
}
MipCmdResult saveHeadingsource(C::mip_interface& device)
{
    return C::save_mip_filter_heading_source(&device);
}
MipCmdResult loadHeadingsource(C::mip_interface& device)
{
    return C::load_mip_filter_heading_source(&device);
}
MipCmdResult defaultHeadingsource(C::mip_interface& device)
{
    return C::default_mip_filter_heading_source(&device);
}



struct altitudeAiding : C::mip_filter_altitude_aiding_command
{
    static const uint8_t descriptorSet = MIP_FILTER_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_FILTER_ALTITUDE_AIDING_CONTROL;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::insert_mip_filter_altitude_aiding_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_filter_altitude_aiding_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = true;
    static const bool canWrite = true;
    static const bool canRead = true;
    static const bool canSave = true;
    static const bool canLoad = true;
    static const bool canReset = true;
    
    struct Response : C::mip_filter_altitude_aiding_response
    {
        static const uint8_t descriptorSet = MIP_FILTER_CMD_DESC_SET;
        static const uint8_t fieldDescriptor = MIP_REPLY_DESC_FILTER_ALTITUDE_AIDING_CONTROL;
        
        size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::insert_mip_filter_altitude_aiding_response(buffer, bufferSize, offset, this);
        }
        size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::extract_mip_filter_altitude_aiding_response(buffer, bufferSize, offset, this);
        }
    };
    
};
MipCmdResult writeAltitudeaiding(C::mip_interface& device, uint8_t aiding_selector)
{
    return C::write_mip_filter_altitude_aiding(&device, aiding_selector);
}
MipCmdResult readAltitudeaiding(C::mip_interface& device, uint8_t& aiding_selector)
{
    return C::read_mip_filter_altitude_aiding(&device, &aiding_selector);
}
MipCmdResult saveAltitudeaiding(C::mip_interface& device)
{
    return C::save_mip_filter_altitude_aiding(&device);
}
MipCmdResult loadAltitudeaiding(C::mip_interface& device)
{
    return C::load_mip_filter_altitude_aiding(&device);
}
MipCmdResult defaultAltitudeaiding(C::mip_interface& device)
{
    return C::default_mip_filter_altitude_aiding(&device);
}



struct autoZupt : C::mip_filter_auto_zupt_command
{
    static const uint8_t descriptorSet = MIP_FILTER_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_FILTER_ZUPT_CONTROL;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::insert_mip_filter_auto_zupt_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_filter_auto_zupt_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = true;
    static const bool canWrite = true;
    static const bool canRead = true;
    static const bool canSave = true;
    static const bool canLoad = true;
    static const bool canReset = true;
    
    struct Response : C::mip_filter_auto_zupt_response
    {
        static const uint8_t descriptorSet = MIP_FILTER_CMD_DESC_SET;
        static const uint8_t fieldDescriptor = MIP_REPLY_DESC_FILTER_ZUPT_CONTROL;
        
        size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::insert_mip_filter_auto_zupt_response(buffer, bufferSize, offset, this);
        }
        size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::extract_mip_filter_auto_zupt_response(buffer, bufferSize, offset, this);
        }
    };
    
};
MipCmdResult writeAutozupt(C::mip_interface& device, uint8_t enable, float threshold)
{
    return C::write_mip_filter_auto_zupt(&device, enable, threshold);
}
MipCmdResult readAutozupt(C::mip_interface& device, uint8_t& enable, float& threshold)
{
    return C::read_mip_filter_auto_zupt(&device, &enable, &threshold);
}
MipCmdResult saveAutozupt(C::mip_interface& device)
{
    return C::save_mip_filter_auto_zupt(&device);
}
MipCmdResult loadAutozupt(C::mip_interface& device)
{
    return C::load_mip_filter_auto_zupt(&device);
}
MipCmdResult defaultAutozupt(C::mip_interface& device)
{
    return C::default_mip_filter_auto_zupt(&device);
}



struct autoAngularZupt : C::mip_filter_auto_angular_zupt_command
{
    static const uint8_t descriptorSet = MIP_FILTER_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_FILTER_ANGULAR_ZUPT_CONTROL;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::insert_mip_filter_auto_angular_zupt_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_filter_auto_angular_zupt_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = true;
    static const bool canWrite = true;
    static const bool canRead = true;
    static const bool canSave = true;
    static const bool canLoad = true;
    static const bool canReset = true;
    
    struct Response : C::mip_filter_auto_angular_zupt_response
    {
        static const uint8_t descriptorSet = MIP_FILTER_CMD_DESC_SET;
        static const uint8_t fieldDescriptor = MIP_REPLY_DESC_FILTER_ANGULAR_ZUPT_CONTROL;
        
        size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::insert_mip_filter_auto_angular_zupt_response(buffer, bufferSize, offset, this);
        }
        size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::extract_mip_filter_auto_angular_zupt_response(buffer, bufferSize, offset, this);
        }
    };
    
};
MipCmdResult writeAutoangularzupt(C::mip_interface& device, uint8_t enable, float threshold)
{
    return C::write_mip_filter_auto_angular_zupt(&device, enable, threshold);
}
MipCmdResult readAutoangularzupt(C::mip_interface& device, uint8_t& enable, float& threshold)
{
    return C::read_mip_filter_auto_angular_zupt(&device, &enable, &threshold);
}
MipCmdResult saveAutoangularzupt(C::mip_interface& device)
{
    return C::save_mip_filter_auto_angular_zupt(&device);
}
MipCmdResult loadAutoangularzupt(C::mip_interface& device)
{
    return C::load_mip_filter_auto_angular_zupt(&device);
}
MipCmdResult defaultAutoangularzupt(C::mip_interface& device)
{
    return C::default_mip_filter_auto_angular_zupt(&device);
}



struct commandedZupt : C::mip_filter_commanded_zupt_command
{
    static const uint8_t descriptorSet = MIP_FILTER_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_FILTER_COMMANDED_ZUPT;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::insert_mip_filter_commanded_zupt_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_filter_commanded_zupt_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = false;
    
};
MipCmdResult commandedZupt(C::mip_interface& device)
{
    return C::mip_filter_commanded_zupt(&device);
}



struct commandedAngularZupt : C::mip_filter_commanded_angular_zupt_command
{
    static const uint8_t descriptorSet = MIP_FILTER_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_FILTER_COMMANDED_ANGULAR_ZUPT;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::insert_mip_filter_commanded_angular_zupt_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_filter_commanded_angular_zupt_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = false;
    
};
MipCmdResult commandedAngularZupt(C::mip_interface& device)
{
    return C::mip_filter_commanded_angular_zupt(&device);
}



struct aidingMeasurementEnable : C::mip_filter_aiding_measurement_enable_command
{
    static const uint8_t descriptorSet = MIP_FILTER_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_FILTER_AIDING_MEASUREMENT_ENABLE;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::insert_mip_filter_aiding_measurement_enable_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_filter_aiding_measurement_enable_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = true;
    static const bool canWrite = true;
    static const bool canRead = true;
    static const bool canSave = true;
    static const bool canLoad = true;
    static const bool canReset = true;
    
    struct Response : C::mip_filter_aiding_measurement_enable_response
    {
        static const uint8_t descriptorSet = MIP_FILTER_CMD_DESC_SET;
        static const uint8_t fieldDescriptor = MIP_REPLY_DESC_FILTER_AIDING_MEASUREMENT_ENABLE;
        
        size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::insert_mip_filter_aiding_measurement_enable_response(buffer, bufferSize, offset, this);
        }
        size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::extract_mip_filter_aiding_measurement_enable_response(buffer, bufferSize, offset, this);
        }
    };
    
};
MipCmdResult writeAidingmeasurementenable(C::mip_interface& device, C::mip_filter_aiding_measurement_enable_command_aiding_source aiding_source, bool enable)
{
    return C::write_mip_filter_aiding_measurement_enable(&device, aiding_source, enable);
}
MipCmdResult readAidingmeasurementenable(C::mip_interface& device, C::mip_filter_aiding_measurement_enable_command_aiding_source aiding_source, bool& enable)
{
    return C::read_mip_filter_aiding_measurement_enable(&device, aiding_source, &enable);
}
MipCmdResult saveAidingmeasurementenable(C::mip_interface& device, C::mip_filter_aiding_measurement_enable_command_aiding_source aiding_source)
{
    return C::save_mip_filter_aiding_measurement_enable(&device, aiding_source);
}
MipCmdResult loadAidingmeasurementenable(C::mip_interface& device, C::mip_filter_aiding_measurement_enable_command_aiding_source aiding_source)
{
    return C::load_mip_filter_aiding_measurement_enable(&device, aiding_source);
}
MipCmdResult defaultAidingmeasurementenable(C::mip_interface& device, C::mip_filter_aiding_measurement_enable_command_aiding_source aiding_source)
{
    return C::default_mip_filter_aiding_measurement_enable(&device, aiding_source);
}



struct run : C::mip_filter_run_command
{
    static const uint8_t descriptorSet = MIP_FILTER_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_FILTER_RUN;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::insert_mip_filter_run_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_filter_run_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = false;
    
};
MipCmdResult run(C::mip_interface& device)
{
    return C::mip_filter_run(&device);
}



struct kinematicConstraint : C::mip_filter_kinematic_constraint_command
{
    static const uint8_t descriptorSet = MIP_FILTER_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_FILTER_KINEMATIC_CONSTRAINT;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::insert_mip_filter_kinematic_constraint_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_filter_kinematic_constraint_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = true;
    static const bool canWrite = true;
    static const bool canRead = true;
    static const bool canSave = true;
    static const bool canLoad = true;
    static const bool canReset = true;
    
    struct Response : C::mip_filter_kinematic_constraint_response
    {
        static const uint8_t descriptorSet = MIP_FILTER_CMD_DESC_SET;
        static const uint8_t fieldDescriptor = MIP_REPLY_DESC_FILTER_KINEMATIC_CONSTRAINT;
        
        size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::insert_mip_filter_kinematic_constraint_response(buffer, bufferSize, offset, this);
        }
        size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::extract_mip_filter_kinematic_constraint_response(buffer, bufferSize, offset, this);
        }
    };
    
};
MipCmdResult writeKinematicconstraint(C::mip_interface& device, uint8_t acceleration_constraint_selection, uint8_t velocity_constraint_selection, uint8_t angular_constraint_selection)
{
    return C::write_mip_filter_kinematic_constraint(&device, acceleration_constraint_selection, velocity_constraint_selection, angular_constraint_selection);
}
MipCmdResult readKinematicconstraint(C::mip_interface& device, uint8_t& acceleration_constraint_selection, uint8_t& velocity_constraint_selection, uint8_t& angular_constraint_selection)
{
    return C::read_mip_filter_kinematic_constraint(&device, &acceleration_constraint_selection, &velocity_constraint_selection, &angular_constraint_selection);
}
MipCmdResult saveKinematicconstraint(C::mip_interface& device)
{
    return C::save_mip_filter_kinematic_constraint(&device);
}
MipCmdResult loadKinematicconstraint(C::mip_interface& device)
{
    return C::load_mip_filter_kinematic_constraint(&device);
}
MipCmdResult defaultKinematicconstraint(C::mip_interface& device)
{
    return C::default_mip_filter_kinematic_constraint(&device);
}



struct initializationConfiguration : C::mip_filter_initialization_configuration_command
{
    static const uint8_t descriptorSet = MIP_FILTER_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_FILTER_INITIALIZATION_CONFIGURATION;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::insert_mip_filter_initialization_configuration_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_filter_initialization_configuration_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = true;
    static const bool canWrite = true;
    static const bool canRead = true;
    static const bool canSave = true;
    static const bool canLoad = true;
    static const bool canReset = true;
    
    struct Response : C::mip_filter_initialization_configuration_response
    {
        static const uint8_t descriptorSet = MIP_FILTER_CMD_DESC_SET;
        static const uint8_t fieldDescriptor = MIP_REPLY_DESC_FILTER_INITIALIZATION_CONFIGURATION;
        
        size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::insert_mip_filter_initialization_configuration_response(buffer, bufferSize, offset, this);
        }
        size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::extract_mip_filter_initialization_configuration_response(buffer, bufferSize, offset, this);
        }
    };
    
};
MipCmdResult writeInitializationconfiguration(C::mip_interface& device, uint8_t wait_for_run_command, C::mip_filter_initialization_configuration_command_initial_condition_source initial_cond_src, C::mip_filter_initialization_configuration_command_alignment_selector auto_heading_alignment_selector, float initial_heading, float initial_pitch, float initial_roll, const float* initial_position, const float* initial_velocity, C::mip_filter_reference_frame reference_frame_selector)
{
    return C::write_mip_filter_initialization_configuration(&device, wait_for_run_command, initial_cond_src, auto_heading_alignment_selector, initial_heading, initial_pitch, initial_roll, initial_position, initial_velocity, reference_frame_selector);
}
MipCmdResult readInitializationconfiguration(C::mip_interface& device, uint8_t& wait_for_run_command, enum C::mip_filter_initialization_configuration_command_initial_condition_source& initial_cond_src, enum C::mip_filter_initialization_configuration_command_alignment_selector& auto_heading_alignment_selector, float& initial_heading, float& initial_pitch, float& initial_roll, float* initial_position, float* initial_velocity, enum C::mip_filter_reference_frame& reference_frame_selector)
{
    return C::read_mip_filter_initialization_configuration(&device, &wait_for_run_command, &initial_cond_src, &auto_heading_alignment_selector, &initial_heading, &initial_pitch, &initial_roll, initial_position, initial_velocity, &reference_frame_selector);
}
MipCmdResult saveInitializationconfiguration(C::mip_interface& device)
{
    return C::save_mip_filter_initialization_configuration(&device);
}
MipCmdResult loadInitializationconfiguration(C::mip_interface& device)
{
    return C::load_mip_filter_initialization_configuration(&device);
}
MipCmdResult defaultInitializationconfiguration(C::mip_interface& device)
{
    return C::default_mip_filter_initialization_configuration(&device);
}



struct adaptiveFilterOptions : C::mip_filter_adaptive_filter_options_command
{
    static const uint8_t descriptorSet = MIP_FILTER_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_FILTER_ADAPTIVE_FILTER_OPTIONS;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::insert_mip_filter_adaptive_filter_options_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_filter_adaptive_filter_options_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = true;
    static const bool canWrite = true;
    static const bool canRead = true;
    static const bool canSave = true;
    static const bool canLoad = true;
    static const bool canReset = true;
    
    struct Response : C::mip_filter_adaptive_filter_options_response
    {
        static const uint8_t descriptorSet = MIP_FILTER_CMD_DESC_SET;
        static const uint8_t fieldDescriptor = MIP_REPLY_DESC_FILTER_ADAPTIVE_FILTER_OPTIONS;
        
        size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::insert_mip_filter_adaptive_filter_options_response(buffer, bufferSize, offset, this);
        }
        size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::extract_mip_filter_adaptive_filter_options_response(buffer, bufferSize, offset, this);
        }
    };
    
};
MipCmdResult writeAdaptivefilteroptions(C::mip_interface& device, uint8_t level, uint16_t time_limit)
{
    return C::write_mip_filter_adaptive_filter_options(&device, level, time_limit);
}
MipCmdResult readAdaptivefilteroptions(C::mip_interface& device, uint8_t& level, uint16_t& time_limit)
{
    return C::read_mip_filter_adaptive_filter_options(&device, &level, &time_limit);
}
MipCmdResult saveAdaptivefilteroptions(C::mip_interface& device)
{
    return C::save_mip_filter_adaptive_filter_options(&device);
}
MipCmdResult loadAdaptivefilteroptions(C::mip_interface& device)
{
    return C::load_mip_filter_adaptive_filter_options(&device);
}
MipCmdResult defaultAdaptivefilteroptions(C::mip_interface& device)
{
    return C::default_mip_filter_adaptive_filter_options(&device);
}



struct multiAntennaOffset : C::mip_filter_multi_antenna_offset_command
{
    static const uint8_t descriptorSet = MIP_FILTER_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_FILTER_MULTI_ANTENNA_OFFSET;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::insert_mip_filter_multi_antenna_offset_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_filter_multi_antenna_offset_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = true;
    static const bool canWrite = true;
    static const bool canRead = true;
    static const bool canSave = true;
    static const bool canLoad = true;
    static const bool canReset = true;
    
    struct Response : C::mip_filter_multi_antenna_offset_response
    {
        static const uint8_t descriptorSet = MIP_FILTER_CMD_DESC_SET;
        static const uint8_t fieldDescriptor = MIP_REPLY_DESC_FILTER_MULTI_ANTENNA_OFFSET;
        
        size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::insert_mip_filter_multi_antenna_offset_response(buffer, bufferSize, offset, this);
        }
        size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::extract_mip_filter_multi_antenna_offset_response(buffer, bufferSize, offset, this);
        }
    };
    
};
MipCmdResult writeMultiantennaoffset(C::mip_interface& device, uint8_t receiver_id, const float* antenna_offset)
{
    return C::write_mip_filter_multi_antenna_offset(&device, receiver_id, antenna_offset);
}
MipCmdResult readMultiantennaoffset(C::mip_interface& device, uint8_t receiver_id, float* antenna_offset)
{
    return C::read_mip_filter_multi_antenna_offset(&device, receiver_id, antenna_offset);
}
MipCmdResult saveMultiantennaoffset(C::mip_interface& device, uint8_t receiver_id)
{
    return C::save_mip_filter_multi_antenna_offset(&device, receiver_id);
}
MipCmdResult loadMultiantennaoffset(C::mip_interface& device, uint8_t receiver_id)
{
    return C::load_mip_filter_multi_antenna_offset(&device, receiver_id);
}
MipCmdResult defaultMultiantennaoffset(C::mip_interface& device, uint8_t receiver_id)
{
    return C::default_mip_filter_multi_antenna_offset(&device, receiver_id);
}



struct relPosConfiguration : C::mip_filter_rel_pos_configuration_command
{
    static const uint8_t descriptorSet = MIP_FILTER_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_FILTER_REL_POS_CONFIGURATION;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::insert_mip_filter_rel_pos_configuration_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_filter_rel_pos_configuration_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = true;
    static const bool canWrite = true;
    static const bool canRead = true;
    static const bool canSave = true;
    static const bool canLoad = true;
    static const bool canReset = true;
    
    struct Response : C::mip_filter_rel_pos_configuration_response
    {
        static const uint8_t descriptorSet = MIP_FILTER_CMD_DESC_SET;
        static const uint8_t fieldDescriptor = MIP_REPLY_DESC_FILTER_REL_POS_CONFIGURATION;
        
        size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::insert_mip_filter_rel_pos_configuration_response(buffer, bufferSize, offset, this);
        }
        size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::extract_mip_filter_rel_pos_configuration_response(buffer, bufferSize, offset, this);
        }
    };
    
};
MipCmdResult writeRelposconfiguration(C::mip_interface& device, uint8_t source, C::mip_filter_reference_frame reference_frame_selector, const double* reference_coordinates)
{
    return C::write_mip_filter_rel_pos_configuration(&device, source, reference_frame_selector, reference_coordinates);
}
MipCmdResult readRelposconfiguration(C::mip_interface& device, uint8_t& source, enum C::mip_filter_reference_frame& reference_frame_selector, double* reference_coordinates)
{
    return C::read_mip_filter_rel_pos_configuration(&device, &source, &reference_frame_selector, reference_coordinates);
}
MipCmdResult saveRelposconfiguration(C::mip_interface& device)
{
    return C::save_mip_filter_rel_pos_configuration(&device);
}
MipCmdResult loadRelposconfiguration(C::mip_interface& device)
{
    return C::load_mip_filter_rel_pos_configuration(&device);
}
MipCmdResult defaultRelposconfiguration(C::mip_interface& device)
{
    return C::default_mip_filter_rel_pos_configuration(&device);
}



struct refPointLeverArm : C::mip_filter_ref_point_lever_arm_command
{
    static const uint8_t descriptorSet = MIP_FILTER_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_FILTER_REF_POINT_LEVER_ARM;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::insert_mip_filter_ref_point_lever_arm_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_filter_ref_point_lever_arm_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = true;
    static const bool canWrite = true;
    static const bool canRead = true;
    static const bool canSave = true;
    static const bool canLoad = true;
    static const bool canReset = true;
    
    struct Response : C::mip_filter_ref_point_lever_arm_response
    {
        static const uint8_t descriptorSet = MIP_FILTER_CMD_DESC_SET;
        static const uint8_t fieldDescriptor = MIP_REPLY_DESC_FILTER_REF_POINT_LEVER_ARM;
        
        size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::insert_mip_filter_ref_point_lever_arm_response(buffer, bufferSize, offset, this);
        }
        size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::extract_mip_filter_ref_point_lever_arm_response(buffer, bufferSize, offset, this);
        }
    };
    
};
MipCmdResult writeRefpointleverarm(C::mip_interface& device, C::mip_filter_ref_point_lever_arm_command_reference_point_selector ref_point_sel, const float* lever_arm_offset)
{
    return C::write_mip_filter_ref_point_lever_arm(&device, ref_point_sel, lever_arm_offset);
}
MipCmdResult readRefpointleverarm(C::mip_interface& device, enum C::mip_filter_ref_point_lever_arm_command_reference_point_selector& ref_point_sel, float* lever_arm_offset)
{
    return C::read_mip_filter_ref_point_lever_arm(&device, &ref_point_sel, lever_arm_offset);
}
MipCmdResult saveRefpointleverarm(C::mip_interface& device)
{
    return C::save_mip_filter_ref_point_lever_arm(&device);
}
MipCmdResult loadRefpointleverarm(C::mip_interface& device)
{
    return C::load_mip_filter_ref_point_lever_arm(&device);
}
MipCmdResult defaultRefpointleverarm(C::mip_interface& device)
{
    return C::default_mip_filter_ref_point_lever_arm(&device);
}



struct speedMeasurement : C::mip_filter_speed_measurement_command
{
    static const uint8_t descriptorSet = MIP_FILTER_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_FILTER_SPEED_MEASUREMENT;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::insert_mip_filter_speed_measurement_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_filter_speed_measurement_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = false;
    
};
MipCmdResult speedMeasurement(C::mip_interface& device, uint8_t source, float time_of_week, float speed, float speed_uncertainty)
{
    return C::mip_filter_speed_measurement(&device, source, time_of_week, speed, speed_uncertainty);
}



struct speedLeverArm : C::mip_filter_speed_lever_arm_command
{
    static const uint8_t descriptorSet = MIP_FILTER_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_FILTER_SPEED_LEVER_ARM;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::insert_mip_filter_speed_lever_arm_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_filter_speed_lever_arm_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = true;
    static const bool canWrite = true;
    static const bool canRead = true;
    static const bool canSave = true;
    static const bool canLoad = true;
    static const bool canReset = true;
    
    struct Response : C::mip_filter_speed_lever_arm_response
    {
        static const uint8_t descriptorSet = MIP_FILTER_CMD_DESC_SET;
        static const uint8_t fieldDescriptor = MIP_REPLY_DESC_FILTER_SPEED_LEVER_ARM;
        
        size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::insert_mip_filter_speed_lever_arm_response(buffer, bufferSize, offset, this);
        }
        size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::extract_mip_filter_speed_lever_arm_response(buffer, bufferSize, offset, this);
        }
    };
    
};
MipCmdResult writeSpeedleverarm(C::mip_interface& device, uint8_t source, const float* lever_arm_offset)
{
    return C::write_mip_filter_speed_lever_arm(&device, source, lever_arm_offset);
}
MipCmdResult readSpeedleverarm(C::mip_interface& device, uint8_t source, float* lever_arm_offset)
{
    return C::read_mip_filter_speed_lever_arm(&device, source, lever_arm_offset);
}
MipCmdResult saveSpeedleverarm(C::mip_interface& device, uint8_t source)
{
    return C::save_mip_filter_speed_lever_arm(&device, source);
}
MipCmdResult loadSpeedleverarm(C::mip_interface& device, uint8_t source)
{
    return C::load_mip_filter_speed_lever_arm(&device, source);
}
MipCmdResult defaultSpeedleverarm(C::mip_interface& device, uint8_t source)
{
    return C::default_mip_filter_speed_lever_arm(&device, source);
}



struct wheeledVehicleConstraintControl : C::mip_filter_wheeled_vehicle_constraint_control_command
{
    static const uint8_t descriptorSet = MIP_FILTER_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_WHEELED_VEHICLE_CONSTRAINT_CONTROL;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::insert_mip_filter_wheeled_vehicle_constraint_control_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_filter_wheeled_vehicle_constraint_control_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = true;
    static const bool canWrite = true;
    static const bool canRead = true;
    static const bool canSave = true;
    static const bool canLoad = true;
    static const bool canReset = true;
    
    struct Response : C::mip_filter_wheeled_vehicle_constraint_control_response
    {
        static const uint8_t descriptorSet = MIP_FILTER_CMD_DESC_SET;
        static const uint8_t fieldDescriptor = MIP_REPLY_DESC_WHEELED_VEHICLE_CONSTRAINT_CONTROL;
        
        size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::insert_mip_filter_wheeled_vehicle_constraint_control_response(buffer, bufferSize, offset, this);
        }
        size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::extract_mip_filter_wheeled_vehicle_constraint_control_response(buffer, bufferSize, offset, this);
        }
    };
    
};
MipCmdResult writeWheeledvehicleconstraintcontrol(C::mip_interface& device, uint8_t enable)
{
    return C::write_mip_filter_wheeled_vehicle_constraint_control(&device, enable);
}
MipCmdResult readWheeledvehicleconstraintcontrol(C::mip_interface& device, uint8_t& enable)
{
    return C::read_mip_filter_wheeled_vehicle_constraint_control(&device, &enable);
}
MipCmdResult saveWheeledvehicleconstraintcontrol(C::mip_interface& device)
{
    return C::save_mip_filter_wheeled_vehicle_constraint_control(&device);
}
MipCmdResult loadWheeledvehicleconstraintcontrol(C::mip_interface& device)
{
    return C::load_mip_filter_wheeled_vehicle_constraint_control(&device);
}
MipCmdResult defaultWheeledvehicleconstraintcontrol(C::mip_interface& device)
{
    return C::default_mip_filter_wheeled_vehicle_constraint_control(&device);
}



struct verticalGyroConstraintControl : C::mip_filter_vertical_gyro_constraint_control_command
{
    static const uint8_t descriptorSet = MIP_FILTER_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_VERTICAL_GYRO_CONSTRAINT_CONTROL;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::insert_mip_filter_vertical_gyro_constraint_control_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_filter_vertical_gyro_constraint_control_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = true;
    static const bool canWrite = true;
    static const bool canRead = true;
    static const bool canSave = true;
    static const bool canLoad = true;
    static const bool canReset = true;
    
    struct Response : C::mip_filter_vertical_gyro_constraint_control_response
    {
        static const uint8_t descriptorSet = MIP_FILTER_CMD_DESC_SET;
        static const uint8_t fieldDescriptor = MIP_REPLY_DESC_VERTICAL_GYRO_CONSTRAINT_CONTROL;
        
        size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::insert_mip_filter_vertical_gyro_constraint_control_response(buffer, bufferSize, offset, this);
        }
        size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::extract_mip_filter_vertical_gyro_constraint_control_response(buffer, bufferSize, offset, this);
        }
    };
    
};
MipCmdResult writeVerticalgyroconstraintcontrol(C::mip_interface& device, uint8_t enable)
{
    return C::write_mip_filter_vertical_gyro_constraint_control(&device, enable);
}
MipCmdResult readVerticalgyroconstraintcontrol(C::mip_interface& device, uint8_t& enable)
{
    return C::read_mip_filter_vertical_gyro_constraint_control(&device, &enable);
}
MipCmdResult saveVerticalgyroconstraintcontrol(C::mip_interface& device)
{
    return C::save_mip_filter_vertical_gyro_constraint_control(&device);
}
MipCmdResult loadVerticalgyroconstraintcontrol(C::mip_interface& device)
{
    return C::load_mip_filter_vertical_gyro_constraint_control(&device);
}
MipCmdResult defaultVerticalgyroconstraintcontrol(C::mip_interface& device)
{
    return C::default_mip_filter_vertical_gyro_constraint_control(&device);
}



struct gnssAntennaCalControl : C::mip_filter_gnss_antenna_cal_control_command
{
    static const uint8_t descriptorSet = MIP_FILTER_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_GNSS_ANTENNA_CALIBRATION_CONTROL;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::insert_mip_filter_gnss_antenna_cal_control_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_filter_gnss_antenna_cal_control_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = true;
    static const bool canWrite = true;
    static const bool canRead = true;
    static const bool canSave = true;
    static const bool canLoad = true;
    static const bool canReset = true;
    
    struct Response : C::mip_filter_gnss_antenna_cal_control_response
    {
        static const uint8_t descriptorSet = MIP_FILTER_CMD_DESC_SET;
        static const uint8_t fieldDescriptor = MIP_REPLY_DESC_GNSS_ANTENNA_CALIBRATION_CONTROL;
        
        size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::insert_mip_filter_gnss_antenna_cal_control_response(buffer, bufferSize, offset, this);
        }
        size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::extract_mip_filter_gnss_antenna_cal_control_response(buffer, bufferSize, offset, this);
        }
    };
    
};
MipCmdResult writeGnssantennacalcontrol(C::mip_interface& device, uint8_t enable, float max_offset)
{
    return C::write_mip_filter_gnss_antenna_cal_control(&device, enable, max_offset);
}
MipCmdResult readGnssantennacalcontrol(C::mip_interface& device, uint8_t& enable, float& max_offset)
{
    return C::read_mip_filter_gnss_antenna_cal_control(&device, &enable, &max_offset);
}
MipCmdResult saveGnssantennacalcontrol(C::mip_interface& device)
{
    return C::save_mip_filter_gnss_antenna_cal_control(&device);
}
MipCmdResult loadGnssantennacalcontrol(C::mip_interface& device)
{
    return C::load_mip_filter_gnss_antenna_cal_control(&device);
}
MipCmdResult defaultGnssantennacalcontrol(C::mip_interface& device)
{
    return C::default_mip_filter_gnss_antenna_cal_control(&device);
}



struct magneticDeclinationSource : C::mip_filter_magnetic_declination_source_command
{
    static const uint8_t descriptorSet = MIP_FILTER_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_FILTER_DECLINATION_SOURCE;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::insert_mip_filter_magnetic_declination_source_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_filter_magnetic_declination_source_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = true;
    static const bool canWrite = true;
    static const bool canRead = true;
    static const bool canSave = true;
    static const bool canLoad = true;
    static const bool canReset = true;
    
    struct Response : C::mip_filter_magnetic_declination_source_response
    {
        static const uint8_t descriptorSet = MIP_FILTER_CMD_DESC_SET;
        static const uint8_t fieldDescriptor = MIP_REPLY_DESC_FILTER_DECLINATION_SOURCE;
        
        size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::insert_mip_filter_magnetic_declination_source_response(buffer, bufferSize, offset, this);
        }
        size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::extract_mip_filter_magnetic_declination_source_response(buffer, bufferSize, offset, this);
        }
    };
    
};
MipCmdResult writeMagneticdeclinationsource(C::mip_interface& device, C::mip_filter_mag_declination_source source, float declination)
{
    return C::write_mip_filter_magnetic_declination_source(&device, source, declination);
}
MipCmdResult readMagneticdeclinationsource(C::mip_interface& device, enum C::mip_filter_mag_declination_source& source, float& declination)
{
    return C::read_mip_filter_magnetic_declination_source(&device, &source, &declination);
}
MipCmdResult saveMagneticdeclinationsource(C::mip_interface& device)
{
    return C::save_mip_filter_magnetic_declination_source(&device);
}
MipCmdResult loadMagneticdeclinationsource(C::mip_interface& device)
{
    return C::load_mip_filter_magnetic_declination_source(&device);
}
MipCmdResult defaultMagneticdeclinationsource(C::mip_interface& device)
{
    return C::default_mip_filter_magnetic_declination_source(&device);
}



struct setInitialHeading : C::mip_filter_set_initial_heading_command
{
    static const uint8_t descriptorSet = MIP_FILTER_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_FILTER_SET_INITIAL_HEADING;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::insert_mip_filter_set_initial_heading_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_filter_set_initial_heading_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = false;
    
};
MipCmdResult setInitialHeading(C::mip_interface& device, float heading)
{
    return C::mip_filter_set_initial_heading(&device, heading);
}



} // namespace FilterCommands
} // namespace mscl
#endif // __cplusplus
