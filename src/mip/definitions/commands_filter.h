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
///@addtogroup MipCommands
///@{
///@defgroup FILTER_COMMAND  FILTER COMMAND
///
///@{

////////////////////////////////////////////////////////////////////////////////
// Descriptors
////////////////////////////////////////////////////////////////////////////////

enum MipFilterCommand_Descriptors
{
    MIP_FILTER_COMMAND_DESC_SET                              = 0x0D,
    
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
    MIP_REPLY_DESC_FILTER_SPEED_LEVER_ARM                    = 0xE1,
    MIP_REPLY_DESC_WHEELED_VEHICLE_CONSTRAINT_CONTROL        = 0xE3,
    MIP_REPLY_DESC_VERTICAL_GYRO_CONSTRAINT_CONTROL          = 0xE2,
    MIP_REPLY_DESC_GNSS_ANTENNA_CALIBRATION_CONTROL          = 0xE4,
    MIP_REPLY_DESC_FILTER_DECLINATION_SOURCE                 = 0xB2,
};

////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////

enum MipFilterReferenceFrame
{
    MIPFILTERREFERENCEFRAME_ECEF = 1,  ///<  WGS84 Earth-fixed, earth centered coordinates
    MIPFILTERREFERENCEFRAME_LLH  = 2,  ///<  WGS84 Latitude, longitude, and height above ellipsoid
};
size_t insert_MipFilterReferenceFrame(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipFilterReferenceFrame self);
size_t extract_MipFilterReferenceFrame(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipFilterReferenceFrame* self);

enum MipFilterMagDeclinationSource
{
    MIPFILTERMAGDECLINATIONSOURCE_NONE   = 1,  ///<  Magnetic field is assumed to have an declination angle equal to zero.
    MIPFILTERMAGDECLINATIONSOURCE_WMM    = 2,  ///<  Magnetic field is assumed to conform to the World Magnetic Model, calculated using current location estimate as an input to the model.
    MIPFILTERMAGDECLINATIONSOURCE_MANUAL = 3,  ///<  Magnetic field is assumed to have the declination angle specified by the user.
};
size_t insert_MipFilterMagDeclinationSource(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipFilterMagDeclinationSource self);
size_t extract_MipFilterMagDeclinationSource(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipFilterMagDeclinationSource* self);


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_reset_filter  Reset Filter
/// Resets the filter to the initialization state.
/// 
/// If the auto-initialization feature is disabled, the initial attitude or heading must be set in
/// order to enter the run state after a reset.
///
///@{

struct MipCmd_Filter_ResetFilter
{
};
size_t insert_MipCmd_Filter_ResetFilter(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_ResetFilter* self);
size_t extract_MipCmd_Filter_ResetFilter(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_ResetFilter* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_set_initial_attitude  Set Initial Attitude
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

struct MipCmd_Filter_SetInitialAttitude
{
    float                                             roll;
    float                                             pitch;
    float                                             heading;
};
size_t insert_MipCmd_Filter_SetInitialAttitude(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_SetInitialAttitude* self);
size_t extract_MipCmd_Filter_SetInitialAttitude(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_SetInitialAttitude* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_estimation_control  Estimation Control
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

enum MipCmd_Filter_EstimationControl_Enableflags
{
    MIPCMD_FILTER_ESTIMATIONCONTROL_ENABLEFLAGS_GYRO_BIAS          = 0x01,
    MIPCMD_FILTER_ESTIMATIONCONTROL_ENABLEFLAGS_ACCEL_BIAS         = 0x02,
    MIPCMD_FILTER_ESTIMATIONCONTROL_ENABLEFLAGS_GYRO_SCALE_FACTOR  = 0x04,
    MIPCMD_FILTER_ESTIMATIONCONTROL_ENABLEFLAGS_ACCEL_SCALE_FACTOR = 0x08,
    MIPCMD_FILTER_ESTIMATIONCONTROL_ENABLEFLAGS_ANTENNA_OFFSET     = 0x10,
    MIPCMD_FILTER_ESTIMATIONCONTROL_ENABLEFLAGS_AUTO_MAG_HARD_IRON = 0x20,
    MIPCMD_FILTER_ESTIMATIONCONTROL_ENABLEFLAGS_AUTO_MAG_SOFT_IRON = 0x40,
};
size_t insert_MipCmd_Filter_EstimationControl_Enableflags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipCmd_Filter_EstimationControl_Enableflags self);
size_t extract_MipCmd_Filter_EstimationControl_Enableflags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipCmd_Filter_EstimationControl_Enableflags* self);

struct MipCmd_Filter_EstimationControl
{
    enum MipFunctionSelector                          function;
    enum MipCmd_Filter_EstimationControl_Enableflags  enable;
};
size_t insert_MipCmd_Filter_EstimationControl(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_EstimationControl* self);
size_t extract_MipCmd_Filter_EstimationControl(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_EstimationControl* self);

struct MipCmd_Filter_EstimationControl_Response
{
    enum MipCmd_Filter_EstimationControl_Enableflags  enable;
};
size_t insert_MipCmd_Filter_EstimationControl_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_EstimationControl_Response* self);
size_t extract_MipCmd_Filter_EstimationControl_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_EstimationControl_Response* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_external_gnss_update  External Gnss Update
/// Provide a filter measurement from an external GNSS
/// 
/// The GNSS source control must be set to "external" for this command to succeed, otherwise it will be NACK'd.
/// Please refer to your device user manual for information on the maximum rate of this message.
/// 
///
///@{

struct MipCmd_Filter_ExternalGnssUpdate
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
size_t insert_MipCmd_Filter_ExternalGnssUpdate(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_ExternalGnssUpdate* self);
size_t extract_MipCmd_Filter_ExternalGnssUpdate(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_ExternalGnssUpdate* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_external_heading_update  External Heading Update
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

struct MipCmd_Filter_ExternalHeadingUpdate
{
    float                                             heading;
    float                                             heading_uncertainty;
    uint8_t                                           type;
};
size_t insert_MipCmd_Filter_ExternalHeadingUpdate(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_ExternalHeadingUpdate* self);
size_t extract_MipCmd_Filter_ExternalHeadingUpdate(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_ExternalHeadingUpdate* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_external_heading_update_with_time  External Heading Update With Time
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

struct MipCmd_Filter_ExternalHeadingUpdateWithTime
{
    double                                            gps_time;
    uint16_t                                          gps_week;
    float                                             heading;
    float                                             heading_uncertainty;
    uint8_t                                           type;
};
size_t insert_MipCmd_Filter_ExternalHeadingUpdateWithTime(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_ExternalHeadingUpdateWithTime* self);
size_t extract_MipCmd_Filter_ExternalHeadingUpdateWithTime(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_ExternalHeadingUpdateWithTime* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_tare_orientation  Tare Orientation
/// Tare the device orientation.
/// 
/// This function uses the current device orientation relative to the NED frame as the current sensor to vehicle transformation.
/// This command is provided as a convenient way to set the sensor to vehicle frame transformation.
/// The filter must be initialized and have a valid attitude output. If the attitude is not valid, an error will be returned.
///
///@{

enum MipCmd_Filter_TareOrientation_Miptareaxes
{
    MIPCMD_FILTER_TAREORIENTATION_MIPTAREAXES_ROLL  = 0x01,
    MIPCMD_FILTER_TAREORIENTATION_MIPTAREAXES_PITCH = 0x02,
    MIPCMD_FILTER_TAREORIENTATION_MIPTAREAXES_YAW   = 0x04,
};
size_t insert_MipCmd_Filter_TareOrientation_Miptareaxes(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipCmd_Filter_TareOrientation_Miptareaxes self);
size_t extract_MipCmd_Filter_TareOrientation_Miptareaxes(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipCmd_Filter_TareOrientation_Miptareaxes* self);

struct MipCmd_Filter_TareOrientation
{
    enum MipFunctionSelector                          function;
    enum MipCmd_Filter_TareOrientation_Miptareaxes    axes;
};
size_t insert_MipCmd_Filter_TareOrientation(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_TareOrientation* self);
size_t extract_MipCmd_Filter_TareOrientation(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_TareOrientation* self);

struct MipCmd_Filter_TareOrientation_Response
{
    enum MipCmd_Filter_TareOrientation_Miptareaxes    axes;
};
size_t insert_MipCmd_Filter_TareOrientation_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_TareOrientation_Response* self);
size_t extract_MipCmd_Filter_TareOrientation_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_TareOrientation_Response* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_sensor_2_vehicle_rotation_euler  Sensor 2 Vehicle Rotation Euler
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

struct MipCmd_Filter_Sensor2VehicleRotationEuler
{
    enum MipFunctionSelector                          function;
    float                                             roll;
    float                                             pitch;
    float                                             yaw;
};
size_t insert_MipCmd_Filter_Sensor2VehicleRotationEuler(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_Sensor2VehicleRotationEuler* self);
size_t extract_MipCmd_Filter_Sensor2VehicleRotationEuler(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_Sensor2VehicleRotationEuler* self);

struct MipCmd_Filter_Sensor2VehicleRotationEuler_Response
{
    float                                             roll;
    float                                             pitch;
    float                                             yaw;
};
size_t insert_MipCmd_Filter_Sensor2VehicleRotationEuler_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_Sensor2VehicleRotationEuler_Response* self);
size_t extract_MipCmd_Filter_Sensor2VehicleRotationEuler_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_Sensor2VehicleRotationEuler_Response* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_sensor_2_vehicle_rotation_dcm  Sensor 2 Vehicle Rotation Dcm
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

struct MipCmd_Filter_Sensor2VehicleRotationDcm
{
    enum MipFunctionSelector                          function;
    float                                             dcm[9];
};
size_t insert_MipCmd_Filter_Sensor2VehicleRotationDcm(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_Sensor2VehicleRotationDcm* self);
size_t extract_MipCmd_Filter_Sensor2VehicleRotationDcm(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_Sensor2VehicleRotationDcm* self);

struct MipCmd_Filter_Sensor2VehicleRotationDcm_Response
{
    float                                             dcm[9];
};
size_t insert_MipCmd_Filter_Sensor2VehicleRotationDcm_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_Sensor2VehicleRotationDcm_Response* self);
size_t extract_MipCmd_Filter_Sensor2VehicleRotationDcm_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_Sensor2VehicleRotationDcm_Response* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_sensor_2_vehicle_rotation_quaternion  Sensor 2 Vehicle Rotation Quaternion
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

struct MipCmd_Filter_Sensor2VehicleRotationQuaternion
{
    enum MipFunctionSelector                          function;
    float                                             quat[4];
};
size_t insert_MipCmd_Filter_Sensor2VehicleRotationQuaternion(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_Sensor2VehicleRotationQuaternion* self);
size_t extract_MipCmd_Filter_Sensor2VehicleRotationQuaternion(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_Sensor2VehicleRotationQuaternion* self);

struct MipCmd_Filter_Sensor2VehicleRotationQuaternion_Response
{
    float                                             quat[4];
};
size_t insert_MipCmd_Filter_Sensor2VehicleRotationQuaternion_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_Sensor2VehicleRotationQuaternion_Response* self);
size_t extract_MipCmd_Filter_Sensor2VehicleRotationQuaternion_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_Sensor2VehicleRotationQuaternion_Response* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_sensor_2_vehicle_offset  Sensor 2 Vehicle Offset
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

struct MipCmd_Filter_Sensor2VehicleOffset
{
    enum MipFunctionSelector                          function;
    float                                             offset[3];
};
size_t insert_MipCmd_Filter_Sensor2VehicleOffset(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_Sensor2VehicleOffset* self);
size_t extract_MipCmd_Filter_Sensor2VehicleOffset(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_Sensor2VehicleOffset* self);

struct MipCmd_Filter_Sensor2VehicleOffset_Response
{
    float                                             offset[3];
};
size_t insert_MipCmd_Filter_Sensor2VehicleOffset_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_Sensor2VehicleOffset_Response* self);
size_t extract_MipCmd_Filter_Sensor2VehicleOffset_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_Sensor2VehicleOffset_Response* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_antenna_offset  Antenna Offset
/// Set the sensor to GNSS antenna offset.
/// 
/// This is expressed in the sensor frame, from the sensor origin to the GNSS antenna RF center.
/// 
/// The magnitude of the offset vector is limited to 10 meters
/// 
///
///@{

struct MipCmd_Filter_AntennaOffset
{
    enum MipFunctionSelector                          function;
    float                                             offset[3];
};
size_t insert_MipCmd_Filter_AntennaOffset(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_AntennaOffset* self);
size_t extract_MipCmd_Filter_AntennaOffset(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_AntennaOffset* self);

struct MipCmd_Filter_AntennaOffset_Response
{
    float                                             offset[3];
};
size_t insert_MipCmd_Filter_AntennaOffset_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_AntennaOffset_Response* self);
size_t extract_MipCmd_Filter_AntennaOffset_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_AntennaOffset_Response* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_gnss_source  Gnss Source
/// Control the source of GNSS information used to update the Kalman Filter.
/// 
/// Changing the GNSS source while the sensor is in the "running" state will temporarily place
/// it back in the "init" state until the new source of GNSS data is received.
/// 
///
///@{

struct MipCmd_Filter_GnssSource
{
    enum MipFunctionSelector                          function;
    uint8_t                                           source;
};
size_t insert_MipCmd_Filter_GnssSource(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_GnssSource* self);
size_t extract_MipCmd_Filter_GnssSource(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_GnssSource* self);

struct MipCmd_Filter_GnssSource_Response
{
    uint8_t                                           source;
};
size_t insert_MipCmd_Filter_GnssSource_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_GnssSource_Response* self);
size_t extract_MipCmd_Filter_GnssSource_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_GnssSource_Response* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_heading_source  Heading Source
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

enum MipCmd_Filter_HeadingSource_Headingsource
{
    MIPCMD_FILTER_HEADINGSOURCE_HEADINGSOURCE_NONE                          = 0,  ///<  See note 3
    MIPCMD_FILTER_HEADINGSOURCE_HEADINGSOURCE_MAG                           = 1,  ///<  
    MIPCMD_FILTER_HEADINGSOURCE_HEADINGSOURCE_GNSS_VEL                      = 2,  ///<  Seen notes 1,2
    MIPCMD_FILTER_HEADINGSOURCE_HEADINGSOURCE_EXTERNAL                      = 3,  ///<  
    MIPCMD_FILTER_HEADINGSOURCE_HEADINGSOURCE_GNSS_VEL_AND_MAG              = 4,  ///<  
    MIPCMD_FILTER_HEADINGSOURCE_HEADINGSOURCE_GNSS_VEL_AND_EXTERNAL         = 5,  ///<  
    MIPCMD_FILTER_HEADINGSOURCE_HEADINGSOURCE_MAG_AND_EXTERNAL              = 6,  ///<  
    MIPCMD_FILTER_HEADINGSOURCE_HEADINGSOURCE_GNSS_VEL_AND_MAG_AND_EXTERNAL = 7,  ///<  
};
size_t insert_MipCmd_Filter_HeadingSource_Headingsource(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipCmd_Filter_HeadingSource_Headingsource self);
size_t extract_MipCmd_Filter_HeadingSource_Headingsource(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipCmd_Filter_HeadingSource_Headingsource* self);

struct MipCmd_Filter_HeadingSource
{
    enum MipFunctionSelector                          function;
    enum MipCmd_Filter_HeadingSource_Headingsource    source;
};
size_t insert_MipCmd_Filter_HeadingSource(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_HeadingSource* self);
size_t extract_MipCmd_Filter_HeadingSource(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_HeadingSource* self);

struct MipCmd_Filter_HeadingSource_Response
{
    enum MipCmd_Filter_HeadingSource_Headingsource    source;
};
size_t insert_MipCmd_Filter_HeadingSource_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_HeadingSource_Response* self);
size_t extract_MipCmd_Filter_HeadingSource_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_HeadingSource_Response* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_altitude_aiding  Altitude Aiding
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

struct MipCmd_Filter_AltitudeAiding
{
    enum MipFunctionSelector                          function;
    uint8_t                                           aiding_selector;
};
size_t insert_MipCmd_Filter_AltitudeAiding(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_AltitudeAiding* self);
size_t extract_MipCmd_Filter_AltitudeAiding(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_AltitudeAiding* self);

struct MipCmd_Filter_AltitudeAiding_Response
{
    uint8_t                                           aiding_selector;
};
size_t insert_MipCmd_Filter_AltitudeAiding_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_AltitudeAiding_Response* self);
size_t extract_MipCmd_Filter_AltitudeAiding_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_AltitudeAiding_Response* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_auto_zupt  Auto Zupt
/// Zero Velocity Update
/// The ZUPT is triggered when the scalar magnitude of the GNSS reported velocity vector is equal-to or less than the threshold value.
/// The device will NACK threshold values that are less than zero (i.e.negative.)
///
///@{

struct MipCmd_Filter_AutoZupt
{
    enum MipFunctionSelector                          function;
    uint8_t                                           enable;
    float                                             threshold;
};
size_t insert_MipCmd_Filter_AutoZupt(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_AutoZupt* self);
size_t extract_MipCmd_Filter_AutoZupt(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_AutoZupt* self);

struct MipCmd_Filter_AutoZupt_Response
{
    uint8_t                                           enable;
    float                                             threshold;
};
size_t insert_MipCmd_Filter_AutoZupt_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_AutoZupt_Response* self);
size_t extract_MipCmd_Filter_AutoZupt_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_AutoZupt_Response* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_auto_angular_zupt  Auto Angular Zupt
/// Zero Angular Rate Update
/// The ZUPT is triggered when the scalar magnitude of the angular rate vector is equal-to or less than the threshold value.
/// The device will NACK threshold values that are less than zero (i.e.negative.)
///
///@{

struct MipCmd_Filter_AutoAngularZupt
{
    enum MipFunctionSelector                          function;
    uint8_t                                           enable;
    float                                             threshold;
};
size_t insert_MipCmd_Filter_AutoAngularZupt(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_AutoAngularZupt* self);
size_t extract_MipCmd_Filter_AutoAngularZupt(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_AutoAngularZupt* self);

struct MipCmd_Filter_AutoAngularZupt_Response
{
    uint8_t                                           enable;
    float                                             threshold;
};
size_t insert_MipCmd_Filter_AutoAngularZupt_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_AutoAngularZupt_Response* self);
size_t extract_MipCmd_Filter_AutoAngularZupt_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_AutoAngularZupt_Response* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_commanded_zupt  Commanded Zupt
/// Commanded Zero Velocity Update
/// Please see the device user manual for the maximum rate of this message.
///
///@{

struct MipCmd_Filter_CommandedZupt
{
};
size_t insert_MipCmd_Filter_CommandedZupt(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_CommandedZupt* self);
size_t extract_MipCmd_Filter_CommandedZupt(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_CommandedZupt* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_commanded_angular_zupt  Commanded Angular Zupt
/// Commanded Zero Angular Rate Update
/// Please see the device user manual for the maximum rate of this message.
///
///@{

struct MipCmd_Filter_CommandedAngularZupt
{
};
size_t insert_MipCmd_Filter_CommandedAngularZupt(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_CommandedAngularZupt* self);
size_t extract_MipCmd_Filter_CommandedAngularZupt(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_CommandedAngularZupt* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_aiding_measurement_enable  Aiding Measurement Enable
/// Enables / disables the specified aiding measurement source.
/// 
/// 
///
///@{

enum MipCmd_Filter_AidingMeasurementEnable_Aidingsource
{
    MIPCMD_FILTER_AIDINGMEASUREMENTENABLE_AIDINGSOURCE_GNSS_POS_VEL     = 0,  ///<  GNSS Position and Velocity
    MIPCMD_FILTER_AIDINGMEASUREMENTENABLE_AIDINGSOURCE_GNSS_HEADING     = 1,  ///<  GNSS Heading (dual antenna)
    MIPCMD_FILTER_AIDINGMEASUREMENTENABLE_AIDINGSOURCE_ALTIMETER        = 2,  ///<  Altimeter
    MIPCMD_FILTER_AIDINGMEASUREMENTENABLE_AIDINGSOURCE_SPEED            = 3,  ///<  Speed sensor / Odometer
    MIPCMD_FILTER_AIDINGMEASUREMENTENABLE_AIDINGSOURCE_MAGNETOMETER     = 4,  ///<  Magnetometer
    MIPCMD_FILTER_AIDINGMEASUREMENTENABLE_AIDINGSOURCE_EXTERNAL_HEADING = 5,  ///<  External heading input
    MIPCMD_FILTER_AIDINGMEASUREMENTENABLE_AIDINGSOURCE_ALL              = 65535,  ///<  Save/load/reset all options
};
size_t insert_MipCmd_Filter_AidingMeasurementEnable_Aidingsource(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipCmd_Filter_AidingMeasurementEnable_Aidingsource self);
size_t extract_MipCmd_Filter_AidingMeasurementEnable_Aidingsource(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipCmd_Filter_AidingMeasurementEnable_Aidingsource* self);

struct MipCmd_Filter_AidingMeasurementEnable
{
    enum MipFunctionSelector                          function;
    enum MipCmd_Filter_AidingMeasurementEnable_Aidingsource aiding_source;
    bool                                              enable;
};
size_t insert_MipCmd_Filter_AidingMeasurementEnable(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_AidingMeasurementEnable* self);
size_t extract_MipCmd_Filter_AidingMeasurementEnable(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_AidingMeasurementEnable* self);

struct MipCmd_Filter_AidingMeasurementEnable_Response
{
    enum MipCmd_Filter_AidingMeasurementEnable_Aidingsource aiding_source;
    bool                                              enable;
};
size_t insert_MipCmd_Filter_AidingMeasurementEnable_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_AidingMeasurementEnable_Response* self);
size_t extract_MipCmd_Filter_AidingMeasurementEnable_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_AidingMeasurementEnable_Response* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_run  Run
/// Manual run command.
/// 
/// If the initialization configuration has the "wait_for_run_command" option enabled, the filter will wait until it receives this command before commencing integration and enabling the Kalman filter. Prior to the receipt of this command, the filter will remain in the filter initialization mode.
///
///@{

struct MipCmd_Filter_Run
{
};
size_t insert_MipCmd_Filter_Run(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_Run* self);
size_t extract_MipCmd_Filter_Run(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_Run* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_kinematic_constraint  Kinematic Constraint
/// Controls kinematic constraint model selection for the navigation filter.
/// 
/// See manual for explanation of how the kinematic constraints are applied.
///
///@{

struct MipCmd_Filter_KinematicConstraint
{
    enum MipFunctionSelector                          function;
    uint8_t                                           acceleration_constraint_selection;
    uint8_t                                           velocity_constraint_selection;
    uint8_t                                           angular_constraint_selection;
};
size_t insert_MipCmd_Filter_KinematicConstraint(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_KinematicConstraint* self);
size_t extract_MipCmd_Filter_KinematicConstraint(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_KinematicConstraint* self);

struct MipCmd_Filter_KinematicConstraint_Response
{
    uint8_t                                           acceleration_constraint_selection;
    uint8_t                                           velocity_constraint_selection;
    uint8_t                                           angular_constraint_selection;
};
size_t insert_MipCmd_Filter_KinematicConstraint_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_KinematicConstraint_Response* self);
size_t extract_MipCmd_Filter_KinematicConstraint_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_KinematicConstraint_Response* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_initialization_configuration  Initialization Configuration
/// Controls the source and values used for initial conditions of the navigation solution.
/// 
/// Notes: Initial conditions are the position, velocity, and attitude of the platform used when the filter starts running or is reset.
/// For the user specified position array, the units are meters if the ECEF frame is selected, and degrees latitude, degrees longitude, and meters above ellipsoid if the latitude/longitude/height frame is selected.
/// For the user specified velocity array, the units are meters per second, but the reference frame depends on the reference frame selector (ECEF or NED).
///
///@{

enum MipCmd_Filter_InitializationConfiguration_Initialconditionsource
{
    MIPCMD_FILTER_INITIALIZATIONCONFIGURATION_INITIALCONDITIONSOURCE_AUTO_POS_VEL_ATT        = 0,  ///<  Automatic position, velocity and attitude
    MIPCMD_FILTER_INITIALIZATIONCONFIGURATION_INITIALCONDITIONSOURCE_AUTO_POS_VEL_PITCH_ROLL = 1,  ///<  Automatic position and velocity, automatic pitch and roll, and user-specified heading
    MIPCMD_FILTER_INITIALIZATIONCONFIGURATION_INITIALCONDITIONSOURCE_AUTO_POS_VEL            = 2,  ///<  Automatic position and velocity, with fully user-specified attitude
    MIPCMD_FILTER_INITIALIZATIONCONFIGURATION_INITIALCONDITIONSOURCE_MANUAL                  = 3,  ///<  User-specified position, velocity, and attitude.
};
size_t insert_MipCmd_Filter_InitializationConfiguration_Initialconditionsource(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipCmd_Filter_InitializationConfiguration_Initialconditionsource self);
size_t extract_MipCmd_Filter_InitializationConfiguration_Initialconditionsource(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipCmd_Filter_InitializationConfiguration_Initialconditionsource* self);

enum MipCmd_Filter_InitializationConfiguration_Alignmentselector
{
    MIPCMD_FILTER_INITIALIZATIONCONFIGURATION_ALIGNMENTSELECTOR_DUAL_ANTENNA = 0x01,
    MIPCMD_FILTER_INITIALIZATIONCONFIGURATION_ALIGNMENTSELECTOR_KINEMATIC    = 0x02,
    MIPCMD_FILTER_INITIALIZATIONCONFIGURATION_ALIGNMENTSELECTOR_MAGNETOMETER = 0x04,
};
size_t insert_MipCmd_Filter_InitializationConfiguration_Alignmentselector(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipCmd_Filter_InitializationConfiguration_Alignmentselector self);
size_t extract_MipCmd_Filter_InitializationConfiguration_Alignmentselector(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipCmd_Filter_InitializationConfiguration_Alignmentselector* self);

struct MipCmd_Filter_InitializationConfiguration
{
    enum MipFunctionSelector                          function;
    uint8_t                                           wait_for_run_command;
    enum MipCmd_Filter_InitializationConfiguration_Initialconditionsource initial_cond_src;
    enum MipCmd_Filter_InitializationConfiguration_Alignmentselector auto_heading_alignment_selector;
    float                                             initial_heading;
    float                                             initial_pitch;
    float                                             initial_roll;
    float                                             initial_position[3];
    float                                             initial_velocity[3];
    enum MipFilterReferenceFrame                      reference_frame_selector;
};
size_t insert_MipCmd_Filter_InitializationConfiguration(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_InitializationConfiguration* self);
size_t extract_MipCmd_Filter_InitializationConfiguration(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_InitializationConfiguration* self);

struct MipCmd_Filter_InitializationConfiguration_Response
{
    uint8_t                                           wait_for_run_command;
    enum MipCmd_Filter_InitializationConfiguration_Initialconditionsource initial_cond_src;
    enum MipCmd_Filter_InitializationConfiguration_Alignmentselector auto_heading_alignment_selector;
    float                                             initial_heading;
    float                                             initial_pitch;
    float                                             initial_roll;
    float                                             initial_position[3];
    float                                             initial_velocity[3];
    enum MipFilterReferenceFrame                      reference_frame_selector;
};
size_t insert_MipCmd_Filter_InitializationConfiguration_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_InitializationConfiguration_Response* self);
size_t extract_MipCmd_Filter_InitializationConfiguration_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_InitializationConfiguration_Response* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_adaptive_filter_options  Adaptive Filter Options
/// Configures the basic setup for auto-adaptive filtering. See product manual for a detailed description of this feature.
///
///@{

struct MipCmd_Filter_AdaptiveFilterOptions
{
    enum MipFunctionSelector                          function;
    uint8_t                                           level;
    uint16_t                                          time_limit;
};
size_t insert_MipCmd_Filter_AdaptiveFilterOptions(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_AdaptiveFilterOptions* self);
size_t extract_MipCmd_Filter_AdaptiveFilterOptions(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_AdaptiveFilterOptions* self);

struct MipCmd_Filter_AdaptiveFilterOptions_Response
{
    uint8_t                                           level;
    uint16_t                                          time_limit;
};
size_t insert_MipCmd_Filter_AdaptiveFilterOptions_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_AdaptiveFilterOptions_Response* self);
size_t extract_MipCmd_Filter_AdaptiveFilterOptions_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_AdaptiveFilterOptions_Response* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_multi_antenna_offset  Multi Antenna Offset
/// Set the antenna lever arm.
/// 
/// This command works with devices that utilize multiple antennas.
///
///@{

struct MipCmd_Filter_MultiAntennaOffset
{
    enum MipFunctionSelector                          function;
    uint8_t                                           receiver_id;
    float                                             antenna_offset[3];
};
size_t insert_MipCmd_Filter_MultiAntennaOffset(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_MultiAntennaOffset* self);
size_t extract_MipCmd_Filter_MultiAntennaOffset(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_MultiAntennaOffset* self);

struct MipCmd_Filter_MultiAntennaOffset_Response
{
    uint8_t                                           receiver_id;
    float                                             antenna_offset[3];
};
size_t insert_MipCmd_Filter_MultiAntennaOffset_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_MultiAntennaOffset_Response* self);
size_t extract_MipCmd_Filter_MultiAntennaOffset_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_MultiAntennaOffset_Response* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_rel_pos_configuration  Rel Pos Configuration
/// Configure the reference location for filter relative positioning outputs
///
///@{

struct MipCmd_Filter_RelPosConfiguration
{
    enum MipFunctionSelector                          function;
    uint8_t                                           source;
    enum MipFilterReferenceFrame                      reference_frame_selector;
    double                                            reference_coordinates[3];
};
size_t insert_MipCmd_Filter_RelPosConfiguration(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_RelPosConfiguration* self);
size_t extract_MipCmd_Filter_RelPosConfiguration(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_RelPosConfiguration* self);

struct MipCmd_Filter_RelPosConfiguration_Response
{
    uint8_t                                           source;
    enum MipFilterReferenceFrame                      reference_frame_selector;
    double                                            reference_coordinates[3];
};
size_t insert_MipCmd_Filter_RelPosConfiguration_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_RelPosConfiguration_Response* self);
size_t extract_MipCmd_Filter_RelPosConfiguration_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_RelPosConfiguration_Response* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_speed_measurement  Speed Measurement
/// Speed aiding measurement, where speed is defined as rate of motion along the vehicle's x-axis direction.
/// Can be used by an external odometer/speedometer, for example.
/// This command cannot be used if the internal odometer is configured.
///
///@{

struct MipCmd_Filter_SpeedMeasurement
{
    uint8_t                                           source;
    float                                             time_of_week;
    float                                             speed;
    float                                             speed_uncertainty;
};
size_t insert_MipCmd_Filter_SpeedMeasurement(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_SpeedMeasurement* self);
size_t extract_MipCmd_Filter_SpeedMeasurement(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_SpeedMeasurement* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_speed_lever_arm  Speed Lever Arm
/// Lever arm offset for speed measurements.
/// This is used to compensate for an off-center measurement point
/// having a different speed due to rotation of the vehicle.
/// The typical use case for this would be an odometer attached to a wheel
/// on a standard 4-wheeled vehicle. If the odometer is on the left wheel,
/// it will report higher speed on right turns and lower speed on left turns.
/// This is because the outside edge of the curve is longer than the inside edge.
///
///@{

struct MipCmd_Filter_SpeedLeverArm
{
    enum MipFunctionSelector                          function;
    uint8_t                                           source;
    float                                             lever_arm_offset[3];
};
size_t insert_MipCmd_Filter_SpeedLeverArm(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_SpeedLeverArm* self);
size_t extract_MipCmd_Filter_SpeedLeverArm(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_SpeedLeverArm* self);

struct MipCmd_Filter_SpeedLeverArm_Response
{
    uint8_t                                           source;
    float                                             lever_arm_offset[3];
};
size_t insert_MipCmd_Filter_SpeedLeverArm_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_SpeedLeverArm_Response* self);
size_t extract_MipCmd_Filter_SpeedLeverArm_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_SpeedLeverArm_Response* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_wheeled_vehicle_constraint_control  Wheeled Vehicle Constraint Control
/// Configure the wheeled vehicle kinematic constraint.
/// 
/// When enabled, the filter uses the assumption that velocity is constrained to the primary vehicle axis.
/// By convention, the primary vehicle axis is the vehicle X-axis (note: the sensor may be physically installed in
/// any orientation on the vehicle if the appropriate mounting transformation has been specified).
/// This constraint will typically improve heading estimates for vehicles where the assumption is valid, such
/// as an automobile, particulary when GNSS coverage is intermittent.
///
///@{

struct MipCmd_Filter_WheeledVehicleConstraintControl
{
    enum MipFunctionSelector                          function;
    uint8_t                                           enable;
};
size_t insert_MipCmd_Filter_WheeledVehicleConstraintControl(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_WheeledVehicleConstraintControl* self);
size_t extract_MipCmd_Filter_WheeledVehicleConstraintControl(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_WheeledVehicleConstraintControl* self);

struct MipCmd_Filter_WheeledVehicleConstraintControl_Response
{
    uint8_t                                           enable;
};
size_t insert_MipCmd_Filter_WheeledVehicleConstraintControl_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_WheeledVehicleConstraintControl_Response* self);
size_t extract_MipCmd_Filter_WheeledVehicleConstraintControl_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_WheeledVehicleConstraintControl_Response* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_vertical_gyro_constraint_control  Vertical Gyro Constraint Control
/// Configure the vertical gyro kinematic constraint.
/// 
/// When enabled and no valid GNSS measurements are available, the filter uses the accelerometers to track pitch
/// and roll under the assumption that the sensor platform is not undergoing linear acceleration.
/// This constraint is useful to maintain accurate pitch and roll during GNSS signal outages.
///
///@{

struct MipCmd_Filter_VerticalGyroConstraintControl
{
    enum MipFunctionSelector                          function;
    uint8_t                                           enable;
};
size_t insert_MipCmd_Filter_VerticalGyroConstraintControl(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_VerticalGyroConstraintControl* self);
size_t extract_MipCmd_Filter_VerticalGyroConstraintControl(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_VerticalGyroConstraintControl* self);

struct MipCmd_Filter_VerticalGyroConstraintControl_Response
{
    uint8_t                                           enable;
};
size_t insert_MipCmd_Filter_VerticalGyroConstraintControl_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_VerticalGyroConstraintControl_Response* self);
size_t extract_MipCmd_Filter_VerticalGyroConstraintControl_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_VerticalGyroConstraintControl_Response* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_gnss_antenna_cal_control  Gnss Antenna Cal Control
/// Configure the GNSS antenna lever arm calibration.
/// 
/// When enabled, the filter will enable lever arm error tracking, up to the maximum offset specified.
///
///@{

struct MipCmd_Filter_GnssAntennaCalControl
{
    enum MipFunctionSelector                          function;
    uint8_t                                           enable;
    float                                             max_offset;
};
size_t insert_MipCmd_Filter_GnssAntennaCalControl(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_GnssAntennaCalControl* self);
size_t extract_MipCmd_Filter_GnssAntennaCalControl(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_GnssAntennaCalControl* self);

struct MipCmd_Filter_GnssAntennaCalControl_Response
{
    uint8_t                                           enable;
    float                                             max_offset;
};
size_t insert_MipCmd_Filter_GnssAntennaCalControl_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_GnssAntennaCalControl_Response* self);
size_t extract_MipCmd_Filter_GnssAntennaCalControl_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_GnssAntennaCalControl_Response* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_magnetic_declination_source  Magnetic Declination Source
/// Source for magnetic declination angle, and user supplied value for manual selection.
///
///@{

struct MipCmd_Filter_MagneticDeclinationSource
{
    enum MipFunctionSelector                          function;
    enum MipFilterMagDeclinationSource                source;
    float                                             declination;
};
size_t insert_MipCmd_Filter_MagneticDeclinationSource(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_MagneticDeclinationSource* self);
size_t extract_MipCmd_Filter_MagneticDeclinationSource(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_MagneticDeclinationSource* self);

struct MipCmd_Filter_MagneticDeclinationSource_Response
{
    enum MipFilterMagDeclinationSource                source;
    float                                             declination;
};
size_t insert_MipCmd_Filter_MagneticDeclinationSource_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_MagneticDeclinationSource_Response* self);
size_t extract_MipCmd_Filter_MagneticDeclinationSource_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_MagneticDeclinationSource_Response* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_set_initial_heading  Set Initial Heading
/// Set the initial heading angle.
/// 
/// The estimation filter will reset the heading estimate to provided value. If the product supports magnetometer aiding and this feature has been enabled, the heading
/// argument will be ignored and the filter will initialize using the inferred magnetic heading.
///
///@{

struct MipCmd_Filter_SetInitialHeading
{
    float                                             heading;
};
size_t insert_MipCmd_Filter_SetInitialHeading(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_SetInitialHeading* self);
size_t extract_MipCmd_Filter_SetInitialHeading(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_SetInitialHeading* self);

///@}
///

///@}
///@}
///
////////////////////////////////////////////////////////////////////////////////

#ifdef __cplusplus
} // extern "C"
} // namespace mscl
#endif // __cplusplus
