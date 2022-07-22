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
///@addtogroup MipData
///@{
///@defgroup SENSOR_DATA  SENSOR DATA
///
///@{

////////////////////////////////////////////////////////////////////////////////
// Descriptors
////////////////////////////////////////////////////////////////////////////////

enum mip_sensor_data_descriptors
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
#ifdef __cplusplus
namespace C {
extern "C" {
#endif // __cplusplus


////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_sensor_raw_accel  None
/// Three element vector representing the sensed acceleration.
/// This quantity is temperature compensated and expressed in the sensor body frame.
///
///@{

struct mip_sensor_raw_accel_data
{
    float                                             raw_accel[3];
};
size_t insert_mip_sensor_raw_accel_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_sensor_raw_accel_data* self);
size_t extract_mip_sensor_raw_accel_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_sensor_raw_accel_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_sensor_raw_gyro  None
/// Three element vector representing the sensed angular rate.
/// This quantity is temperature compensated and expressed in the sensor body frame.
///
///@{

struct mip_sensor_raw_gyro_data
{
    float                                             raw_gyro[3];
};
size_t insert_mip_sensor_raw_gyro_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_sensor_raw_gyro_data* self);
size_t extract_mip_sensor_raw_gyro_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_sensor_raw_gyro_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_sensor_raw_mag  None
/// Three element vector representing the sensed magnetic field.
/// This quantity is temperature compensated and expressed in the vehicle frame.
///
///@{

struct mip_sensor_raw_mag_data
{
    float                                             raw_mag[3];
};
size_t insert_mip_sensor_raw_mag_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_sensor_raw_mag_data* self);
size_t extract_mip_sensor_raw_mag_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_sensor_raw_mag_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_sensor_raw_pressure  None
/// Scalar value representing the sensed ambient pressure.
/// This quantity is temperature compensated.
///
///@{

struct mip_sensor_raw_pressure_data
{
    float                                             raw_pressure;
};
size_t insert_mip_sensor_raw_pressure_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_sensor_raw_pressure_data* self);
size_t extract_mip_sensor_raw_pressure_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_sensor_raw_pressure_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_sensor_scaled_accel  None
/// 3-element vector representing the sensed acceleration.
/// This quantity is temperature compensated and expressed in the vehicle frame.
///
///@{

struct mip_sensor_scaled_accel_data
{
    float                                             scaled_accel[3];
};
size_t insert_mip_sensor_scaled_accel_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_sensor_scaled_accel_data* self);
size_t extract_mip_sensor_scaled_accel_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_sensor_scaled_accel_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_sensor_scaled_gyro  None
/// 3-element vector representing the sensed angular rate.
/// This quantity is temperature compensated and expressed in the vehicle frame.
///
///@{

struct mip_sensor_scaled_gyro_data
{
    float                                             scaled_gyro[3];
};
size_t insert_mip_sensor_scaled_gyro_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_sensor_scaled_gyro_data* self);
size_t extract_mip_sensor_scaled_gyro_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_sensor_scaled_gyro_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_sensor_scaled_mag  None
/// 3-element vector representing the sensed magnetic field.
/// This quantity is temperature compensated and expressed in the vehicle frame.
///
///@{

struct mip_sensor_scaled_mag_data
{
    float                                             scaled_mag[3];
};
size_t insert_mip_sensor_scaled_mag_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_sensor_scaled_mag_data* self);
size_t extract_mip_sensor_scaled_mag_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_sensor_scaled_mag_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_sensor_scaled_pressure  None
/// Scalar value representing the sensed ambient pressure.
///
///@{

struct mip_sensor_scaled_pressure_data
{
    float                                             scaled_pressure;
};
size_t insert_mip_sensor_scaled_pressure_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_sensor_scaled_pressure_data* self);
size_t extract_mip_sensor_scaled_pressure_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_sensor_scaled_pressure_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_sensor_delta_theta  None
/// 3-element vector representing the time integral of angular rate.
/// This quantity is the integral of sensed angular rate over the period set by the IMU message format.  It is expressed in the vehicle frame.
///
///@{

struct mip_sensor_delta_theta_data
{
    float                                             delta_theta[3];
};
size_t insert_mip_sensor_delta_theta_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_sensor_delta_theta_data* self);
size_t extract_mip_sensor_delta_theta_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_sensor_delta_theta_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_sensor_delta_velocity  None
/// 3-element vector representing the time integral of acceleration.
/// This quantity is the integral of sensed acceleration over the period set by the IMU message format.  It is expressed in the vehicle frame.
///
///@{

struct mip_sensor_delta_velocity_data
{
    float                                             delta_velocity[3];
};
size_t insert_mip_sensor_delta_velocity_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_sensor_delta_velocity_data* self);
size_t extract_mip_sensor_delta_velocity_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_sensor_delta_velocity_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_sensor_comp_orientation_matrix  Complementary Filter Orientation Matrix
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

struct mip_sensor_comp_orientation_matrix_data
{
    float                                             m[9];
};
size_t insert_mip_sensor_comp_orientation_matrix_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_sensor_comp_orientation_matrix_data* self);
size_t extract_mip_sensor_comp_orientation_matrix_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_sensor_comp_orientation_matrix_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_sensor_comp_quaternion  Complementary Filter Quaternion
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

struct mip_sensor_comp_quaternion_data
{
    float                                             q[4];
};
size_t insert_mip_sensor_comp_quaternion_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_sensor_comp_quaternion_data* self);
size_t extract_mip_sensor_comp_quaternion_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_sensor_comp_quaternion_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_sensor_comp_euler_angles  Complementary Filter Euler Angles
/// Euler angles describing the orientation of the device with respect to the NED local-level frame.
/// The Euler angles are reported in 3-2-1 (Yaw-Pitch-Roll, AKA Aircraft) order.
///
///@{

struct mip_sensor_comp_euler_angles_data
{
    float                                             roll;
    float                                             pitch;
    float                                             yaw;
};
size_t insert_mip_sensor_comp_euler_angles_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_sensor_comp_euler_angles_data* self);
size_t extract_mip_sensor_comp_euler_angles_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_sensor_comp_euler_angles_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_sensor_comp_orientation_update_matrix  Complementary Filter Orientation Update Matrix
/// DEPRECATED!
///
///@{

struct mip_sensor_comp_orientation_update_matrix_data
{
    float                                             m[9];
};
size_t insert_mip_sensor_comp_orientation_update_matrix_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_sensor_comp_orientation_update_matrix_data* self);
size_t extract_mip_sensor_comp_orientation_update_matrix_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_sensor_comp_orientation_update_matrix_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_sensor_orientation_raw_temp  None
/// DEPRECATED!
///
///@{

struct mip_sensor_orientation_raw_temp_data
{
    uint16_t                                          raw_temp[4];
};
size_t insert_mip_sensor_orientation_raw_temp_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_sensor_orientation_raw_temp_data* self);
size_t extract_mip_sensor_orientation_raw_temp_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_sensor_orientation_raw_temp_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_sensor_internal_timestamp  None
/// DEPRECATED!
///
///@{

struct mip_sensor_internal_timestamp_data
{
    uint32_t                                          counts;
};
size_t insert_mip_sensor_internal_timestamp_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_sensor_internal_timestamp_data* self);
size_t extract_mip_sensor_internal_timestamp_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_sensor_internal_timestamp_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_sensor_1pps_timestamp  PPS Timestamp
/// DEPRECATED!
///
///@{

struct mip_sensor_1pps_timestamp_data
{
    uint32_t                                          seconds;
    uint32_t                                          useconds;
};
size_t insert_mip_sensor_1pps_timestamp_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_sensor_1pps_timestamp_data* self);
size_t extract_mip_sensor_1pps_timestamp_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_sensor_1pps_timestamp_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_sensor_gps_timestamp  None
/// GPS timestamp of the SENSOR data
/// 
/// Should the PPS become unavailable, the device will revert to its internal clock, which will cause the reported time to drift from true GPS time.
/// Upon recovering from a PPS outage, the user should expect a jump in the reported GPS time due to the accumulation of internal clock error.
/// If synchronization to an external clock or onboard GNSS receiver (for products that have one) is disabled, this time is equivalent to internal system time.
/// 
/// Note: this data field may be deprecrated in the future. The more flexible shared data field (0x80, 0xD3) should be used instead.
///
///@{

enum mip_sensor_gps_timestamp_data_valid_flags
{
    MIP_SENSOR_GPS_TIMESTAMP_DATA_VALID_FLAGS_PPS_VALID         = 0x01,
    MIP_SENSOR_GPS_TIMESTAMP_DATA_VALID_FLAGS_TIME_REFRESH      = 0x02,
    MIP_SENSOR_GPS_TIMESTAMP_DATA_VALID_FLAGS_TIME_INITIALIZED  = 0x04,
    MIP_SENSOR_GPS_TIMESTAMP_DATA_VALID_FLAGS_TOW_VALID         = 0x08,
    MIP_SENSOR_GPS_TIMESTAMP_DATA_VALID_FLAGS_WEEK_NUMBER_VALID = 0x10,
};
size_t insert_mip_sensor_gps_timestamp_data_valid_flags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_sensor_gps_timestamp_data_valid_flags self);
size_t extract_mip_sensor_gps_timestamp_data_valid_flags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_sensor_gps_timestamp_data_valid_flags* self);

struct mip_sensor_gps_timestamp_data
{
    double                                            tow;
    uint16_t                                          week_number;
    enum mip_sensor_gps_timestamp_data_valid_flags    valid_flags;
};
size_t insert_mip_sensor_gps_timestamp_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_sensor_gps_timestamp_data* self);
size_t extract_mip_sensor_gps_timestamp_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_sensor_gps_timestamp_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_sensor_temperature_abs  Temperature Statistics
/// SENSOR reported temperature statistics
/// 
/// Temperature may originate from the MEMS sensors, or be calculated in combination with board temperature sensors.
/// All quantities are calculated with respect to the last power on or reset, whichever is later.
/// 
///
///@{

struct mip_sensor_temperature_abs_data
{
    float                                             min_temp;
    float                                             max_temp;
    float                                             mean_temp;
};
size_t insert_mip_sensor_temperature_abs_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_sensor_temperature_abs_data* self);
size_t extract_mip_sensor_temperature_abs_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_sensor_temperature_abs_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_sensor_up_vector  None
/// Gyro-stabilized 3-element vector representing the complementary filter's estimated vertical direction.
/// This quantity is expressed in the vehicle frame.
/// 
/// This quantity is sensitive to non-gravitational accelerations, which may cause notable deviations from the true vertical direction.
/// 
/// For legacy reasons, this vector is the inverse of the gravity vector.
/// 
///
///@{

struct mip_sensor_up_vector_data
{
    float                                             up[3];
};
size_t insert_mip_sensor_up_vector_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_sensor_up_vector_data* self);
size_t extract_mip_sensor_up_vector_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_sensor_up_vector_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_sensor_north_vector  None
/// Gyro-stabilized 3-element vector representing the complementary filter's estimate of magnetic north.
/// This quantity is expressed in the vehicle frame.
/// 
/// This quantity is sensitive to local magnetic field perturbations, which may cause notable deviations from true magnetic north.
///
///@{

struct mip_sensor_north_vector_data
{
    float                                             north[3];
};
size_t insert_mip_sensor_north_vector_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_sensor_north_vector_data* self);
size_t extract_mip_sensor_north_vector_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_sensor_north_vector_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_sensor_overrange_status  None
///
///@{

enum mip_sensor_overrange_status_data_status
{
    MIP_SENSOR_OVERRANGE_STATUS_DATA_STATUS_ACCEL_X = 0x01,
    MIP_SENSOR_OVERRANGE_STATUS_DATA_STATUS_ACCEL_Y = 0x02,
    MIP_SENSOR_OVERRANGE_STATUS_DATA_STATUS_ACCEL_Z = 0x04,
    MIP_SENSOR_OVERRANGE_STATUS_DATA_STATUS_GYRO_X  = 0x10,
    MIP_SENSOR_OVERRANGE_STATUS_DATA_STATUS_GYRO_Y  = 0x20,
    MIP_SENSOR_OVERRANGE_STATUS_DATA_STATUS_GYRO_Z  = 0x40,
    MIP_SENSOR_OVERRANGE_STATUS_DATA_STATUS_MAG_X   = 0x100,
    MIP_SENSOR_OVERRANGE_STATUS_DATA_STATUS_MAG_Y   = 0x200,
    MIP_SENSOR_OVERRANGE_STATUS_DATA_STATUS_MAG_Z   = 0x400,
    MIP_SENSOR_OVERRANGE_STATUS_DATA_STATUS_PRESS   = 0x1000,
};
size_t insert_mip_sensor_overrange_status_data_status(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_sensor_overrange_status_data_status self);
size_t extract_mip_sensor_overrange_status_data_status(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_sensor_overrange_status_data_status* self);

struct mip_sensor_overrange_status_data
{
    enum mip_sensor_overrange_status_data_status      status;
};
size_t insert_mip_sensor_overrange_status_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_sensor_overrange_status_data* self);
size_t extract_mip_sensor_overrange_status_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_sensor_overrange_status_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_sensor_odometer_data  None
///
///@{

struct mip_sensor_odometer_data_data
{
    float                                             speed;
    float                                             uncertainty;
    uint16_t                                          valid_flags;
};
size_t insert_mip_sensor_odometer_data_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_sensor_odometer_data_data* self);
size_t extract_mip_sensor_odometer_data_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_sensor_odometer_data_data* self);

///@}
///

///@}
///@}
///
////////////////////////////////////////////////////////////////////////////////

#ifdef __cplusplus
} // extern "C"
} // namespace C


template<>
struct MipFieldInfo<C::mip_sensor_raw_accel_data>
{
    static const uint8_t descriptorSet = MIP_SENSOR_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_SENSOR_ACCEL_RAW;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_sensor_raw_accel_data& self)
    {
        return C::insert_mip_sensor_raw_accel_data(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_sensor_raw_accel_data& self)
    {
        return C::extract_mip_sensor_raw_accel_data(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_sensor_raw_gyro_data>
{
    static const uint8_t descriptorSet = MIP_SENSOR_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_SENSOR_GYRO_RAW;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_sensor_raw_gyro_data& self)
    {
        return C::insert_mip_sensor_raw_gyro_data(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_sensor_raw_gyro_data& self)
    {
        return C::extract_mip_sensor_raw_gyro_data(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_sensor_raw_mag_data>
{
    static const uint8_t descriptorSet = MIP_SENSOR_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_SENSOR_MAG_RAW;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_sensor_raw_mag_data& self)
    {
        return C::insert_mip_sensor_raw_mag_data(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_sensor_raw_mag_data& self)
    {
        return C::extract_mip_sensor_raw_mag_data(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_sensor_raw_pressure_data>
{
    static const uint8_t descriptorSet = MIP_SENSOR_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_SENSOR_PRESSURE_RAW;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_sensor_raw_pressure_data& self)
    {
        return C::insert_mip_sensor_raw_pressure_data(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_sensor_raw_pressure_data& self)
    {
        return C::extract_mip_sensor_raw_pressure_data(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_sensor_scaled_accel_data>
{
    static const uint8_t descriptorSet = MIP_SENSOR_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_SENSOR_ACCEL_SCALED;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_sensor_scaled_accel_data& self)
    {
        return C::insert_mip_sensor_scaled_accel_data(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_sensor_scaled_accel_data& self)
    {
        return C::extract_mip_sensor_scaled_accel_data(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_sensor_scaled_gyro_data>
{
    static const uint8_t descriptorSet = MIP_SENSOR_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_SENSOR_GYRO_SCALED;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_sensor_scaled_gyro_data& self)
    {
        return C::insert_mip_sensor_scaled_gyro_data(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_sensor_scaled_gyro_data& self)
    {
        return C::extract_mip_sensor_scaled_gyro_data(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_sensor_scaled_mag_data>
{
    static const uint8_t descriptorSet = MIP_SENSOR_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_SENSOR_MAG_SCALED;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_sensor_scaled_mag_data& self)
    {
        return C::insert_mip_sensor_scaled_mag_data(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_sensor_scaled_mag_data& self)
    {
        return C::extract_mip_sensor_scaled_mag_data(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_sensor_scaled_pressure_data>
{
    static const uint8_t descriptorSet = MIP_SENSOR_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_SENSOR_PRESSURE_SCALED;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_sensor_scaled_pressure_data& self)
    {
        return C::insert_mip_sensor_scaled_pressure_data(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_sensor_scaled_pressure_data& self)
    {
        return C::extract_mip_sensor_scaled_pressure_data(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_sensor_delta_theta_data>
{
    static const uint8_t descriptorSet = MIP_SENSOR_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_SENSOR_DELTA_THETA;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_sensor_delta_theta_data& self)
    {
        return C::insert_mip_sensor_delta_theta_data(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_sensor_delta_theta_data& self)
    {
        return C::extract_mip_sensor_delta_theta_data(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_sensor_delta_velocity_data>
{
    static const uint8_t descriptorSet = MIP_SENSOR_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_SENSOR_DELTA_VELOCITY;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_sensor_delta_velocity_data& self)
    {
        return C::insert_mip_sensor_delta_velocity_data(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_sensor_delta_velocity_data& self)
    {
        return C::extract_mip_sensor_delta_velocity_data(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_sensor_comp_orientation_matrix_data>
{
    static const uint8_t descriptorSet = MIP_SENSOR_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_SENSOR_COMP_ORIENTATION_MATRIX;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_sensor_comp_orientation_matrix_data& self)
    {
        return C::insert_mip_sensor_comp_orientation_matrix_data(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_sensor_comp_orientation_matrix_data& self)
    {
        return C::extract_mip_sensor_comp_orientation_matrix_data(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_sensor_comp_quaternion_data>
{
    static const uint8_t descriptorSet = MIP_SENSOR_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_SENSOR_COMP_QUATERNION;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_sensor_comp_quaternion_data& self)
    {
        return C::insert_mip_sensor_comp_quaternion_data(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_sensor_comp_quaternion_data& self)
    {
        return C::extract_mip_sensor_comp_quaternion_data(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_sensor_comp_euler_angles_data>
{
    static const uint8_t descriptorSet = MIP_SENSOR_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_SENSOR_COMP_EULER_ANGLES;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_sensor_comp_euler_angles_data& self)
    {
        return C::insert_mip_sensor_comp_euler_angles_data(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_sensor_comp_euler_angles_data& self)
    {
        return C::extract_mip_sensor_comp_euler_angles_data(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_sensor_comp_orientation_update_matrix_data>
{
    static const uint8_t descriptorSet = MIP_SENSOR_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_SENSOR_COMP_ORIENTATION_UPDATE_MATRIX;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_sensor_comp_orientation_update_matrix_data& self)
    {
        return C::insert_mip_sensor_comp_orientation_update_matrix_data(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_sensor_comp_orientation_update_matrix_data& self)
    {
        return C::extract_mip_sensor_comp_orientation_update_matrix_data(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_sensor_orientation_raw_temp_data>
{
    static const uint8_t descriptorSet = MIP_SENSOR_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_SENSOR_TEMPERATURE_RAW;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_sensor_orientation_raw_temp_data& self)
    {
        return C::insert_mip_sensor_orientation_raw_temp_data(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_sensor_orientation_raw_temp_data& self)
    {
        return C::extract_mip_sensor_orientation_raw_temp_data(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_sensor_internal_timestamp_data>
{
    static const uint8_t descriptorSet = MIP_SENSOR_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_SENSOR_TIME_STAMP_INTERNAL;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_sensor_internal_timestamp_data& self)
    {
        return C::insert_mip_sensor_internal_timestamp_data(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_sensor_internal_timestamp_data& self)
    {
        return C::extract_mip_sensor_internal_timestamp_data(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_sensor_1pps_timestamp_data>
{
    static const uint8_t descriptorSet = MIP_SENSOR_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_SENSOR_TIME_STAMP_PPS;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_sensor_1pps_timestamp_data& self)
    {
        return C::insert_mip_sensor_1pps_timestamp_data(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_sensor_1pps_timestamp_data& self)
    {
        return C::extract_mip_sensor_1pps_timestamp_data(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_sensor_gps_timestamp_data>
{
    static const uint8_t descriptorSet = MIP_SENSOR_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_SENSOR_TIME_STAMP_GPS;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_sensor_gps_timestamp_data& self)
    {
        return C::insert_mip_sensor_gps_timestamp_data(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_sensor_gps_timestamp_data& self)
    {
        return C::extract_mip_sensor_gps_timestamp_data(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_sensor_temperature_abs_data>
{
    static const uint8_t descriptorSet = MIP_SENSOR_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_SENSOR_TEMPERATURE_ABS;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_sensor_temperature_abs_data& self)
    {
        return C::insert_mip_sensor_temperature_abs_data(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_sensor_temperature_abs_data& self)
    {
        return C::extract_mip_sensor_temperature_abs_data(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_sensor_up_vector_data>
{
    static const uint8_t descriptorSet = MIP_SENSOR_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_SENSOR_STAB_ACCEL;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_sensor_up_vector_data& self)
    {
        return C::insert_mip_sensor_up_vector_data(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_sensor_up_vector_data& self)
    {
        return C::extract_mip_sensor_up_vector_data(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_sensor_north_vector_data>
{
    static const uint8_t descriptorSet = MIP_SENSOR_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_SENSOR_STAB_MAG;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_sensor_north_vector_data& self)
    {
        return C::insert_mip_sensor_north_vector_data(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_sensor_north_vector_data& self)
    {
        return C::extract_mip_sensor_north_vector_data(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_sensor_overrange_status_data>
{
    static const uint8_t descriptorSet = MIP_SENSOR_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_SENSOR_OVERRANGE_STATUS;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_sensor_overrange_status_data& self)
    {
        return C::insert_mip_sensor_overrange_status_data(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_sensor_overrange_status_data& self)
    {
        return C::extract_mip_sensor_overrange_status_data(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_sensor_odometer_data_data>
{
    static const uint8_t descriptorSet = MIP_SENSOR_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_SENSOR_ODOMETER;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_sensor_odometer_data_data& self)
    {
        return C::insert_mip_sensor_odometer_data_data(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_sensor_odometer_data_data& self)
    {
        return C::extract_mip_sensor_odometer_data_data(buffer, bufferSize, offset, &self);
    }
};



} // namespace mscl
#endif // __cplusplus
