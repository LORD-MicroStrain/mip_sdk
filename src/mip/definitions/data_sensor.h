#pragma once

#include "descriptors.h"

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

////////////////////////////////////////////////////////////////////////////////
///@addtogroup MipData
///@{
///@defgroup SENSOR_DATA  SENSOR DATA
///
///@{

////////////////////////////////////////////////////////////////////////////////
// Descriptors
////////////////////////////////////////////////////////////////////////////////

enum MipSensorData_Descriptors
{
    MIP_SENSOR_DATA_DESC_SET                            = 0x80,
    
    MIP_DATA_DESC_SENSOR_ACCEL_RAW                      = 0x01,
    MIP_DATA_DESC_SENSOR_GYRO_RAW                       = 0x02,
    MIP_DATA_DESC_SENSOR_MAG_RAW                        = 0x03,
    MIP_DATA_DESC_SENSOR_PRESSURE_RAW                   = 0x16,
    MIP_DATA_DESC_SENSOR_ACCEL_SCALED                   = 0x04,
    MIP_DATA_DESC_SENSOR_GYRO_SCALED                    = 0x05,
    MIP_DATA_DESC_SENSOR_MAG_SCALED                     = 0x06,
    MIP_DATA_DESC_SENSOR_PRESSURE_SCALED                = 0x17,
    MIP_DATA_DESC_SENSOR_DELTA_THETA                    = 0x07,
    MIP_DATA_DESC_SENSOR_DELTA_VELOCITY                 = 0x08,
    MIP_DATA_DESC_SENSOR_COMP_ORIENTATION_MATRIX        = 0x09,
    MIP_DATA_DESC_SENSOR_COMP_QUATERNION                = 0x0A,
    MIP_DATA_DESC_SENSOR_COMP_EULER_ANGLES              = 0x0C,
    MIP_DATA_DESC_SENSOR_COMP_ORIENTATION_UPDATE_MATRIX = 0x0B,
    MIP_DATA_DESC_SENSOR_TEMPERATURE_RAW                = 0x0D,
    MIP_DATA_DESC_SENSOR_TIME_STAMP_INTERNAL            = 0x0E,
    MIP_DATA_DESC_SENSOR_TIME_STAMP_PPS                 = 0x0F,
    MIP_DATA_DESC_SENSOR_TIME_STAMP_GPS                 = 0x12,
    MIP_DATA_DESC_SENSOR_TEMPERATURE_ABS                = 0x14,
    MIP_DATA_DESC_SENSOR_STAB_ACCEL                     = 0x11,
    MIP_DATA_DESC_SENSOR_STAB_MAG                       = 0x10,
    MIP_DATA_DESC_SENSOR_OVERRANGE_STATUS               = 0x18,
    MIP_DATA_DESC_SENSOR_ODOMETER                       = 0x40,
    
};

////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_sensor_raw_accel  Sensor Raw Accel
/// Three element vector representing the sensed acceleration.
/// This quantity is temperature compensated and expressed in the sensor body frame.
///
///@{

struct MipData_Sensor_RawAccel
{
    float                                             raw_accel[3];
};
size_t insert_MipData_Sensor_RawAccel(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Sensor_RawAccel* self);
size_t extract_MipData_Sensor_RawAccel(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Sensor_RawAccel* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_sensor_raw_gyro  Sensor Raw Gyro
/// Three element vector representing the sensed angular rate.
/// This quantity is temperature compensated and expressed in the sensor body frame.
///
///@{

struct MipData_Sensor_RawGyro
{
    float                                             raw_gyro[3];
};
size_t insert_MipData_Sensor_RawGyro(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Sensor_RawGyro* self);
size_t extract_MipData_Sensor_RawGyro(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Sensor_RawGyro* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_sensor_raw_mag  Sensor Raw Mag
/// Three element vector representing the sensed magnetic field.
/// This quantity is temperature compensated and expressed in the vehicle frame.
///
///@{

struct MipData_Sensor_RawMag
{
    float                                             raw_mag[3];
};
size_t insert_MipData_Sensor_RawMag(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Sensor_RawMag* self);
size_t extract_MipData_Sensor_RawMag(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Sensor_RawMag* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_sensor_raw_pressure  Sensor Raw Pressure
/// Scalar value representing the sensed ambient pressure.
/// This quantity is temperature compensated.
///
///@{

struct MipData_Sensor_RawPressure
{
    float                                             raw_pressure;
};
size_t insert_MipData_Sensor_RawPressure(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Sensor_RawPressure* self);
size_t extract_MipData_Sensor_RawPressure(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Sensor_RawPressure* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_sensor_scaled_accel  Sensor Scaled Accel
/// 3-element vector representing the sensed acceleration.
/// This quantity is temperature compensated and expressed in the vehicle frame.
///
///@{

struct MipData_Sensor_ScaledAccel
{
    float                                             scaled_accel[3];
};
size_t insert_MipData_Sensor_ScaledAccel(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Sensor_ScaledAccel* self);
size_t extract_MipData_Sensor_ScaledAccel(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Sensor_ScaledAccel* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_sensor_scaled_gyro  Sensor Scaled Gyro
/// 3-element vector representing the sensed angular rate.
/// This quantity is temperature compensated and expressed in the vehicle frame.
///
///@{

struct MipData_Sensor_ScaledGyro
{
    float                                             scaled_gyro[3];
};
size_t insert_MipData_Sensor_ScaledGyro(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Sensor_ScaledGyro* self);
size_t extract_MipData_Sensor_ScaledGyro(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Sensor_ScaledGyro* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_sensor_scaled_mag  Sensor Scaled Mag
/// 3-element vector representing the sensed magnetic field.
/// This quantity is temperature compensated and expressed in the vehicle frame.
///
///@{

struct MipData_Sensor_ScaledMag
{
    float                                             scaled_mag[3];
};
size_t insert_MipData_Sensor_ScaledMag(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Sensor_ScaledMag* self);
size_t extract_MipData_Sensor_ScaledMag(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Sensor_ScaledMag* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_sensor_scaled_pressure  Sensor Scaled Pressure
/// Scalar value representing the sensed ambient pressure.
///
///@{

struct MipData_Sensor_ScaledPressure
{
    float                                             scaled_pressure;
};
size_t insert_MipData_Sensor_ScaledPressure(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Sensor_ScaledPressure* self);
size_t extract_MipData_Sensor_ScaledPressure(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Sensor_ScaledPressure* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_sensor_delta_theta  Sensor Delta Theta
/// 3-element vector representing the time integral of angular rate.
/// This quantity is the integral of sensed angular rate over the period set by the IMU message format.  It is expressed in the vehicle frame.
///
///@{

struct MipData_Sensor_DeltaTheta
{
    float                                             delta_theta[3];
};
size_t insert_MipData_Sensor_DeltaTheta(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Sensor_DeltaTheta* self);
size_t extract_MipData_Sensor_DeltaTheta(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Sensor_DeltaTheta* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_sensor_delta_velocity  Sensor Delta Velocity
/// 3-element vector representing the time integral of acceleration.
/// This quantity is the integral of sensed acceleration over the period set by the IMU message format.  It is expressed in the vehicle frame.
///
///@{

struct MipData_Sensor_DeltaVelocity
{
    float                                             delta_velocity[3];
};
size_t insert_MipData_Sensor_DeltaVelocity(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Sensor_DeltaVelocity* self);
size_t extract_MipData_Sensor_DeltaVelocity(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Sensor_DeltaVelocity* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_sensor_comp_orientation_matrix  Sensor Comp Orientation Matrix
/// 3x3 Direction Cosine Matrix EQSTART M_{ned}^{veh} EQEND describing the orientation of the device with respect to the NED local-level frame.
/// This matrix satisfies the following relationship:
/// 
/// EQSTART v^{veh} = M_{ned}^{veh} v^{ned} EQEND<br/>
/// 
/// Where:<br/>
/// 
/// EQSTART v^{ned} EQEND is a 3-element vector expressed in the NED frame. <br/>
/// EQSTART v^{veh} EQEND is the same 3-element vector expressed in the vehicle frame.  <br/>
/// <br/>
/// The matrix elements are stored is row-major order: EQSTART M = \begin{bmatrix} M_{11}, M_{12}, M_{13}, M_{21}, M_{22}, M_{23}, M_{31}, M_{32}, M_{33} \end{bmatrix} EQEND
///
///@{

struct MipData_Sensor_CompOrientationMatrix
{
    float                                             m[9];
};
size_t insert_MipData_Sensor_CompOrientationMatrix(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Sensor_CompOrientationMatrix* self);
size_t extract_MipData_Sensor_CompOrientationMatrix(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Sensor_CompOrientationMatrix* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_sensor_comp_quaternion  Sensor Comp Quaternion
/// 4x1 vector representation of the quaternion describing the orientation of the device with respect to the NED local-level frame.
/// This quaternion satisfies the following relationship:
/// 
/// EQSTART p^{veh} = q^{-1} p^{ned} q EQEND<br/>
/// 
/// Where:<br/>
/// EQSTART q = (q_w, q_x, q_y, q_z) ENDEQ is the quaternion desrcribing the rotation. <br/>
/// EQSTART p^ned = (0, v^{ned}_x, v^{ned}_y, v^{ned}_z) ENDEQ and EQSTART v^{ned} EQEND is a 3-element vector expressed in the NED frame.<br/>
/// EQSTART p^veh = (0, v^{veh}_x, v^{veh}_y, v^{veh}_z) ENDEQ and EQSTART v^{veh} EQEND is a 3-element vector expressed in the vehicle frame.<br/>
///
///@{

struct MipData_Sensor_CompQuaternion
{
    float                                             q[4];
};
size_t insert_MipData_Sensor_CompQuaternion(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Sensor_CompQuaternion* self);
size_t extract_MipData_Sensor_CompQuaternion(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Sensor_CompQuaternion* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_sensor_comp_euler_angles  Sensor Comp Euler Angles
/// Euler angles describing the orientation of the device with respect to the NED local-level frame.
/// The Euler angles are reported in 3-2-1 (Yaw-Pitch-Roll, AKA Aircraft) order.
///
///@{

struct MipData_Sensor_CompEulerAngles
{
    float                                             roll;
    float                                             pitch;
    float                                             yaw;
};
size_t insert_MipData_Sensor_CompEulerAngles(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Sensor_CompEulerAngles* self);
size_t extract_MipData_Sensor_CompEulerAngles(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Sensor_CompEulerAngles* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_sensor_comp_orientation_update_matrix  Sensor Comp Orientation Update Matrix
/// DEPRECATED!
///
///@{

struct MipData_Sensor_CompOrientationUpdateMatrix
{
    float                                             m[9];
};
size_t insert_MipData_Sensor_CompOrientationUpdateMatrix(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Sensor_CompOrientationUpdateMatrix* self);
size_t extract_MipData_Sensor_CompOrientationUpdateMatrix(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Sensor_CompOrientationUpdateMatrix* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_sensor_orientation_raw_temp  Sensor Orientation Raw Temp
/// DEPRECATED!
///
///@{

struct MipData_Sensor_OrientationRawTemp
{
    uint16_t                                          raw_temp[4];
};
size_t insert_MipData_Sensor_OrientationRawTemp(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Sensor_OrientationRawTemp* self);
size_t extract_MipData_Sensor_OrientationRawTemp(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Sensor_OrientationRawTemp* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_sensor_internal_timestamp  Sensor Internal Timestamp
/// DEPRECATED!
///
///@{

struct MipData_Sensor_InternalTimestamp
{
    uint32_t                                          counts;
};
size_t insert_MipData_Sensor_InternalTimestamp(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Sensor_InternalTimestamp* self);
size_t extract_MipData_Sensor_InternalTimestamp(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Sensor_InternalTimestamp* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_sensor_1pps_timestamp  Sensor 1Pps Timestamp
/// DEPRECATED!
///
///@{

struct MipData_Sensor_1ppsTimestamp
{
    uint32_t                                          seconds;
    uint32_t                                          useconds;
};
size_t insert_MipData_Sensor_1ppsTimestamp(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Sensor_1ppsTimestamp* self);
size_t extract_MipData_Sensor_1ppsTimestamp(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Sensor_1ppsTimestamp* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_sensor_gps_timestamp  Sensor Gps Timestamp
/// GPS timestamp of the SENSOR data
/// 
/// Should the PPS become unavailable, the device will revert to its internal clock, which will cause the reported time to drift from true GPS time.
/// Upon recovering from a PPS outage, the user should expect a jump in the reported GPS time due to the accumulation of internal clock error.
/// If synchronization to an external clock or onboard GNSS receiver (for products that have one) is disabled, this time is equivalent to internal system time.
/// 
/// Note: this data field may be deprecrated in the future. The more flexible shared data field (0x80, 0xD3) should be used instead.
///
///@{

enum MipData_Sensor_GpsTimestamp_Validflags
{
    MIPDATA_SENSOR_GPSTIMESTAMP_VALIDFLAGS_PPS_VALID         = 0x01,
    MIPDATA_SENSOR_GPSTIMESTAMP_VALIDFLAGS_TIME_REFRESH      = 0x02,
    MIPDATA_SENSOR_GPSTIMESTAMP_VALIDFLAGS_TIME_INITIALIZED  = 0x04,
    MIPDATA_SENSOR_GPSTIMESTAMP_VALIDFLAGS_TOW_VALID         = 0x08,
    MIPDATA_SENSOR_GPSTIMESTAMP_VALIDFLAGS_WEEK_NUMBER_VALID = 0x10,
};
size_t insert_MipData_Sensor_GpsTimestamp_Validflags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Sensor_GpsTimestamp_Validflags self);
size_t extract_MipData_Sensor_GpsTimestamp_Validflags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Sensor_GpsTimestamp_Validflags* self);

struct MipData_Sensor_GpsTimestamp
{
    double                                            tow;
    uint16_t                                          week_number;
    enum MipData_Sensor_GpsTimestamp_Validflags       valid_flags;
};
size_t insert_MipData_Sensor_GpsTimestamp(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Sensor_GpsTimestamp* self);
size_t extract_MipData_Sensor_GpsTimestamp(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Sensor_GpsTimestamp* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_sensor_temperature_abs  Sensor Temperature Abs
/// SENSOR reported temperature statistics
/// 
/// Temperature may originate from the MEMS sensors, or be calculated in combination with board temperature sensors.
/// All quantities are calculated with respect to the last power on or reset, whichever is later.
/// 
///
///@{

struct MipData_Sensor_TemperatureAbs
{
    float                                             min_temp;
    float                                             max_temp;
    float                                             mean_temp;
};
size_t insert_MipData_Sensor_TemperatureAbs(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Sensor_TemperatureAbs* self);
size_t extract_MipData_Sensor_TemperatureAbs(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Sensor_TemperatureAbs* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_sensor_up_vector  Sensor Up Vector
/// Gyro-stabilized 3-element vector representing the complementary filter's estimated vertical direction.
/// This quantity is expressed in the vehicle frame.
/// 
/// This quantity is sensitive to non-gravitational accelerations, which may cause notable deviations from the true vertical direction.
/// 
/// For legacy reasons, this vector is the inverse of the gravity vector.
/// 
///
///@{

struct MipData_Sensor_UpVector
{
    float                                             up[3];
};
size_t insert_MipData_Sensor_UpVector(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Sensor_UpVector* self);
size_t extract_MipData_Sensor_UpVector(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Sensor_UpVector* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_sensor_north_vector  Sensor North Vector
/// Gyro-stabilized 3-element vector representing the complementary filter's estimate of magnetic north.
/// This quantity is expressed in the vehicle frame.
/// 
/// This quantity is sensitive to local magnetic field perturbations, which may cause notable deviations from true magnetic north.
///
///@{

struct MipData_Sensor_NorthVector
{
    float                                             north[3];
};
size_t insert_MipData_Sensor_NorthVector(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Sensor_NorthVector* self);
size_t extract_MipData_Sensor_NorthVector(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Sensor_NorthVector* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_sensor_overrange_status  Sensor Overrange Status
///
///@{

enum MipData_Sensor_OverrangeStatus_Status
{
    MIPDATA_SENSOR_OVERRANGESTATUS_STATUS_ACCEL_X = 0x01,
    MIPDATA_SENSOR_OVERRANGESTATUS_STATUS_ACCEL_Y = 0x02,
    MIPDATA_SENSOR_OVERRANGESTATUS_STATUS_ACCEL_Z = 0x04,
    MIPDATA_SENSOR_OVERRANGESTATUS_STATUS_GYRO_X  = 0x10,
    MIPDATA_SENSOR_OVERRANGESTATUS_STATUS_GYRO_Y  = 0x20,
    MIPDATA_SENSOR_OVERRANGESTATUS_STATUS_GYRO_Z  = 0x40,
    MIPDATA_SENSOR_OVERRANGESTATUS_STATUS_MAG_X   = 0x100,
    MIPDATA_SENSOR_OVERRANGESTATUS_STATUS_MAG_Y   = 0x200,
    MIPDATA_SENSOR_OVERRANGESTATUS_STATUS_MAG_Z   = 0x400,
    MIPDATA_SENSOR_OVERRANGESTATUS_STATUS_PRESS   = 0x1000,
};
size_t insert_MipData_Sensor_OverrangeStatus_Status(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Sensor_OverrangeStatus_Status self);
size_t extract_MipData_Sensor_OverrangeStatus_Status(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Sensor_OverrangeStatus_Status* self);

struct MipData_Sensor_OverrangeStatus
{
    enum MipData_Sensor_OverrangeStatus_Status        status;
};
size_t insert_MipData_Sensor_OverrangeStatus(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Sensor_OverrangeStatus* self);
size_t extract_MipData_Sensor_OverrangeStatus(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Sensor_OverrangeStatus* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_sensor_odometer_data  Sensor Odometer Data
///
///@{

struct MipData_Sensor_OdometerData
{
    float                                             speed;
    float                                             uncertainty;
    uint16_t                                          valid_flags;
};
size_t insert_MipData_Sensor_OdometerData(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Sensor_OdometerData* self);
size_t extract_MipData_Sensor_OdometerData(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Sensor_OdometerData* self);

///@}
///

///@}
///@}
///
////////////////////////////////////////////////////////////////////////////////
