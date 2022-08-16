#pragma once

#include "descriptors.h"
#include "../mip_result.h"

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
namespace mip {
namespace C {
extern "C" {

#endif // __cplusplus
struct mip_interface;
struct mip_serializer;
struct mip_field;

////////////////////////////////////////////////////////////////////////////////
///@addtogroup MipData
///@{
///@defgroup sensor_data_c  SENSORData
///
///@{

////////////////////////////////////////////////////////////////////////////////
// Descriptors
////////////////////////////////////////////////////////////////////////////////

enum 
{
    MIP_SENSOR_DATA_DESC_SET                            = 0x80,
    
    MIP_DATA_DESC_SENSOR_ACCEL_RAW                      = 0x01,
    MIP_DATA_DESC_SENSOR_GYRO_RAW                       = 0x02,
    MIP_DATA_DESC_SENSOR_MAG_RAW                        = 0x03,
    MIP_DATA_DESC_SENSOR_ACCEL_SCALED                   = 0x04,
    MIP_DATA_DESC_SENSOR_GYRO_SCALED                    = 0x05,
    MIP_DATA_DESC_SENSOR_MAG_SCALED                     = 0x06,
    MIP_DATA_DESC_SENSOR_DELTA_THETA                    = 0x07,
    MIP_DATA_DESC_SENSOR_DELTA_VELOCITY                 = 0x08,
    MIP_DATA_DESC_SENSOR_COMP_ORIENTATION_MATRIX        = 0x09,
    MIP_DATA_DESC_SENSOR_COMP_QUATERNION                = 0x0A,
    MIP_DATA_DESC_SENSOR_COMP_ORIENTATION_UPDATE_MATRIX = 0x0B,
    MIP_DATA_DESC_SENSOR_COMP_EULER_ANGLES              = 0x0C,
    MIP_DATA_DESC_SENSOR_TEMPERATURE_RAW                = 0x0D,
    MIP_DATA_DESC_SENSOR_TIME_STAMP_INTERNAL            = 0x0E,
    MIP_DATA_DESC_SENSOR_TIME_STAMP_PPS                 = 0x0F,
    MIP_DATA_DESC_SENSOR_STAB_MAG                       = 0x10,
    MIP_DATA_DESC_SENSOR_STAB_ACCEL                     = 0x11,
    MIP_DATA_DESC_SENSOR_TIME_STAMP_GPS                 = 0x12,
    MIP_DATA_DESC_SENSOR_TEMPERATURE_ABS                = 0x14,
    MIP_DATA_DESC_SENSOR_RAW_CLIP_DATA                  = 0x15,
    MIP_DATA_DESC_SENSOR_PRESSURE_RAW                   = 0x16,
    MIP_DATA_DESC_SENSOR_PRESSURE_SCALED                = 0x17,
    MIP_DATA_DESC_SENSOR_OVERRANGE_STATUS               = 0x18,
    MIP_DATA_DESC_SENSOR_ODOMETER                       = 0x40,
    
    MIP_DATA_DESC_ASPP                                  = 0x81,
    MIP_DATA_DESC_GXSB                                  = 0x82,
};

////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
///@defgroup c_raw_accel  None
/// Three element vector representing the sensed acceleration.
/// This quantity is temperature compensated and expressed in the sensor body frame.
///
///@{

struct mip_sensor_raw_accel_data
{
    float raw_accel[3];
    
};
void insert_mip_sensor_raw_accel_data(struct mip_serializer* serializer, const struct mip_sensor_raw_accel_data* self);
void extract_mip_sensor_raw_accel_data(struct mip_serializer* serializer, struct mip_sensor_raw_accel_data* self);
void extract_mip_sensor_raw_accel_data_from_field(const struct mip_field* field, void* ptr);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_raw_gyro  None
/// Three element vector representing the sensed angular rate.
/// This quantity is temperature compensated and expressed in the sensor body frame.
///
///@{

struct mip_sensor_raw_gyro_data
{
    float raw_gyro[3];
    
};
void insert_mip_sensor_raw_gyro_data(struct mip_serializer* serializer, const struct mip_sensor_raw_gyro_data* self);
void extract_mip_sensor_raw_gyro_data(struct mip_serializer* serializer, struct mip_sensor_raw_gyro_data* self);
void extract_mip_sensor_raw_gyro_data_from_field(const struct mip_field* field, void* ptr);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_raw_mag  None
/// Three element vector representing the sensed magnetic field.
/// This quantity is temperature compensated and expressed in the vehicle frame.
///
///@{

struct mip_sensor_raw_mag_data
{
    float raw_mag[3];
    
};
void insert_mip_sensor_raw_mag_data(struct mip_serializer* serializer, const struct mip_sensor_raw_mag_data* self);
void extract_mip_sensor_raw_mag_data(struct mip_serializer* serializer, struct mip_sensor_raw_mag_data* self);
void extract_mip_sensor_raw_mag_data_from_field(const struct mip_field* field, void* ptr);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_raw_pressure  None
/// Scalar value representing the sensed ambient pressure.
/// This quantity is temperature compensated.
///
///@{

struct mip_sensor_raw_pressure_data
{
    float raw_pressure;
    
};
void insert_mip_sensor_raw_pressure_data(struct mip_serializer* serializer, const struct mip_sensor_raw_pressure_data* self);
void extract_mip_sensor_raw_pressure_data(struct mip_serializer* serializer, struct mip_sensor_raw_pressure_data* self);
void extract_mip_sensor_raw_pressure_data_from_field(const struct mip_field* field, void* ptr);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_scaled_accel  None
/// 3-element vector representing the sensed acceleration.
/// This quantity is temperature compensated and expressed in the vehicle frame.
///
///@{

struct mip_sensor_scaled_accel_data
{
    float scaled_accel[3];
    
};
void insert_mip_sensor_scaled_accel_data(struct mip_serializer* serializer, const struct mip_sensor_scaled_accel_data* self);
void extract_mip_sensor_scaled_accel_data(struct mip_serializer* serializer, struct mip_sensor_scaled_accel_data* self);
void extract_mip_sensor_scaled_accel_data_from_field(const struct mip_field* field, void* ptr);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_scaled_gyro  None
/// 3-element vector representing the sensed angular rate.
/// This quantity is temperature compensated and expressed in the vehicle frame.
///
///@{

struct mip_sensor_scaled_gyro_data
{
    float scaled_gyro[3];
    
};
void insert_mip_sensor_scaled_gyro_data(struct mip_serializer* serializer, const struct mip_sensor_scaled_gyro_data* self);
void extract_mip_sensor_scaled_gyro_data(struct mip_serializer* serializer, struct mip_sensor_scaled_gyro_data* self);
void extract_mip_sensor_scaled_gyro_data_from_field(const struct mip_field* field, void* ptr);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_scaled_mag  None
/// 3-element vector representing the sensed magnetic field.
/// This quantity is temperature compensated and expressed in the vehicle frame.
///
///@{

struct mip_sensor_scaled_mag_data
{
    float scaled_mag[3];
    
};
void insert_mip_sensor_scaled_mag_data(struct mip_serializer* serializer, const struct mip_sensor_scaled_mag_data* self);
void extract_mip_sensor_scaled_mag_data(struct mip_serializer* serializer, struct mip_sensor_scaled_mag_data* self);
void extract_mip_sensor_scaled_mag_data_from_field(const struct mip_field* field, void* ptr);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_scaled_pressure  None
/// Scalar value representing the sensed ambient pressure.
///
///@{

struct mip_sensor_scaled_pressure_data
{
    float scaled_pressure;
    
};
void insert_mip_sensor_scaled_pressure_data(struct mip_serializer* serializer, const struct mip_sensor_scaled_pressure_data* self);
void extract_mip_sensor_scaled_pressure_data(struct mip_serializer* serializer, struct mip_sensor_scaled_pressure_data* self);
void extract_mip_sensor_scaled_pressure_data_from_field(const struct mip_field* field, void* ptr);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_delta_theta  None
/// 3-element vector representing the time integral of angular rate.
/// This quantity is the integral of sensed angular rate over the period set by the IMU message format.  It is expressed in the vehicle frame.
///
///@{

struct mip_sensor_delta_theta_data
{
    float delta_theta[3];
    
};
void insert_mip_sensor_delta_theta_data(struct mip_serializer* serializer, const struct mip_sensor_delta_theta_data* self);
void extract_mip_sensor_delta_theta_data(struct mip_serializer* serializer, struct mip_sensor_delta_theta_data* self);
void extract_mip_sensor_delta_theta_data_from_field(const struct mip_field* field, void* ptr);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_delta_velocity  None
/// 3-element vector representing the time integral of acceleration.
/// This quantity is the integral of sensed acceleration over the period set by the IMU message format.  It is expressed in the vehicle frame.
///
///@{

struct mip_sensor_delta_velocity_data
{
    float delta_velocity[3];
    
};
void insert_mip_sensor_delta_velocity_data(struct mip_serializer* serializer, const struct mip_sensor_delta_velocity_data* self);
void extract_mip_sensor_delta_velocity_data(struct mip_serializer* serializer, struct mip_sensor_delta_velocity_data* self);
void extract_mip_sensor_delta_velocity_data_from_field(const struct mip_field* field, void* ptr);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_comp_orientation_matrix  Complementary Filter Orientation Matrix
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
    float m[9];
    
};
void insert_mip_sensor_comp_orientation_matrix_data(struct mip_serializer* serializer, const struct mip_sensor_comp_orientation_matrix_data* self);
void extract_mip_sensor_comp_orientation_matrix_data(struct mip_serializer* serializer, struct mip_sensor_comp_orientation_matrix_data* self);
void extract_mip_sensor_comp_orientation_matrix_data_from_field(const struct mip_field* field, void* ptr);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_comp_quaternion  Complementary Filter Quaternion
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
    float q[4];
    
};
void insert_mip_sensor_comp_quaternion_data(struct mip_serializer* serializer, const struct mip_sensor_comp_quaternion_data* self);
void extract_mip_sensor_comp_quaternion_data(struct mip_serializer* serializer, struct mip_sensor_comp_quaternion_data* self);
void extract_mip_sensor_comp_quaternion_data_from_field(const struct mip_field* field, void* ptr);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_comp_euler_angles  Complementary Filter Euler Angles
/// Euler angles describing the orientation of the device with respect to the NED local-level frame.
/// The Euler angles are reported in 3-2-1 (Yaw-Pitch-Roll, AKA Aircraft) order.
///
///@{

struct mip_sensor_comp_euler_angles_data
{
    float roll;
    float pitch;
    float yaw;
    
};
void insert_mip_sensor_comp_euler_angles_data(struct mip_serializer* serializer, const struct mip_sensor_comp_euler_angles_data* self);
void extract_mip_sensor_comp_euler_angles_data(struct mip_serializer* serializer, struct mip_sensor_comp_euler_angles_data* self);
void extract_mip_sensor_comp_euler_angles_data_from_field(const struct mip_field* field, void* ptr);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_comp_orientation_update_matrix  Complementary Filter Orientation Update Matrix
/// DEPRECATED!
///
///@{

struct mip_sensor_comp_orientation_update_matrix_data
{
    float m[9];
    
};
void insert_mip_sensor_comp_orientation_update_matrix_data(struct mip_serializer* serializer, const struct mip_sensor_comp_orientation_update_matrix_data* self);
void extract_mip_sensor_comp_orientation_update_matrix_data(struct mip_serializer* serializer, struct mip_sensor_comp_orientation_update_matrix_data* self);
void extract_mip_sensor_comp_orientation_update_matrix_data_from_field(const struct mip_field* field, void* ptr);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_orientation_raw_temp  None
/// DEPRECATED!
///
///@{

struct mip_sensor_orientation_raw_temp_data
{
    uint16_t raw_temp[4];
    
};
void insert_mip_sensor_orientation_raw_temp_data(struct mip_serializer* serializer, const struct mip_sensor_orientation_raw_temp_data* self);
void extract_mip_sensor_orientation_raw_temp_data(struct mip_serializer* serializer, struct mip_sensor_orientation_raw_temp_data* self);
void extract_mip_sensor_orientation_raw_temp_data_from_field(const struct mip_field* field, void* ptr);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_internal_timestamp  None
/// DEPRECATED!
///
///@{

struct mip_sensor_internal_timestamp_data
{
    uint32_t counts;
    
};
void insert_mip_sensor_internal_timestamp_data(struct mip_serializer* serializer, const struct mip_sensor_internal_timestamp_data* self);
void extract_mip_sensor_internal_timestamp_data(struct mip_serializer* serializer, struct mip_sensor_internal_timestamp_data* self);
void extract_mip_sensor_internal_timestamp_data_from_field(const struct mip_field* field, void* ptr);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_pps_timestamp  PPS Timestamp
/// DEPRECATED!
///
///@{

struct mip_sensor_pps_timestamp_data
{
    uint32_t seconds;
    uint32_t useconds;
    
};
void insert_mip_sensor_pps_timestamp_data(struct mip_serializer* serializer, const struct mip_sensor_pps_timestamp_data* self);
void extract_mip_sensor_pps_timestamp_data(struct mip_serializer* serializer, struct mip_sensor_pps_timestamp_data* self);
void extract_mip_sensor_pps_timestamp_data_from_field(const struct mip_field* field, void* ptr);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_gps_timestamp  None
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
    MIP_SENSOR_GPS_TIMESTAMP_DATA_VALID_FLAGS_NONE              = 0x0000,
    MIP_SENSOR_GPS_TIMESTAMP_DATA_VALID_FLAGS_PPS_VALID         = 0x0001,
    MIP_SENSOR_GPS_TIMESTAMP_DATA_VALID_FLAGS_TIME_REFRESH      = 0x0002,
    MIP_SENSOR_GPS_TIMESTAMP_DATA_VALID_FLAGS_TIME_INITIALIZED  = 0x0004,
    MIP_SENSOR_GPS_TIMESTAMP_DATA_VALID_FLAGS_TOW_VALID         = 0x0008,
    MIP_SENSOR_GPS_TIMESTAMP_DATA_VALID_FLAGS_WEEK_NUMBER_VALID = 0x0010,
};

struct mip_sensor_gps_timestamp_data
{
    double tow;
    uint16_t week_number;
    enum mip_sensor_gps_timestamp_data_valid_flags valid_flags;
    
};
void insert_mip_sensor_gps_timestamp_data(struct mip_serializer* serializer, const struct mip_sensor_gps_timestamp_data* self);
void extract_mip_sensor_gps_timestamp_data(struct mip_serializer* serializer, struct mip_sensor_gps_timestamp_data* self);
void extract_mip_sensor_gps_timestamp_data_from_field(const struct mip_field* field, void* ptr);

void insert_mip_sensor_gps_timestamp_data_valid_flags(struct mip_serializer* serializer, const enum mip_sensor_gps_timestamp_data_valid_flags self);
void extract_mip_sensor_gps_timestamp_data_valid_flags(struct mip_serializer* serializer, enum mip_sensor_gps_timestamp_data_valid_flags* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_temperature_abs  Temperature Statistics
/// SENSOR reported temperature statistics
/// 
/// Temperature may originate from the MEMS sensors, or be calculated in combination with board temperature sensors.
/// All quantities are calculated with respect to the last power on or reset, whichever is later.
/// 
///
///@{

struct mip_sensor_temperature_abs_data
{
    float min_temp;
    float max_temp;
    float mean_temp;
    
};
void insert_mip_sensor_temperature_abs_data(struct mip_serializer* serializer, const struct mip_sensor_temperature_abs_data* self);
void extract_mip_sensor_temperature_abs_data(struct mip_serializer* serializer, struct mip_sensor_temperature_abs_data* self);
void extract_mip_sensor_temperature_abs_data_from_field(const struct mip_field* field, void* ptr);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_up_vector  None
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
    float up[3];
    
};
void insert_mip_sensor_up_vector_data(struct mip_serializer* serializer, const struct mip_sensor_up_vector_data* self);
void extract_mip_sensor_up_vector_data(struct mip_serializer* serializer, struct mip_sensor_up_vector_data* self);
void extract_mip_sensor_up_vector_data_from_field(const struct mip_field* field, void* ptr);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_north_vector  None
/// Gyro-stabilized 3-element vector representing the complementary filter's estimate of magnetic north.
/// This quantity is expressed in the vehicle frame.
/// 
/// This quantity is sensitive to local magnetic field perturbations, which may cause notable deviations from true magnetic north.
///
///@{

struct mip_sensor_north_vector_data
{
    float north[3];
    
};
void insert_mip_sensor_north_vector_data(struct mip_serializer* serializer, const struct mip_sensor_north_vector_data* self);
void extract_mip_sensor_north_vector_data(struct mip_serializer* serializer, struct mip_sensor_north_vector_data* self);
void extract_mip_sensor_north_vector_data_from_field(const struct mip_field* field, void* ptr);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_overrange_status  None
///
///@{

enum mip_sensor_overrange_status_data_status
{
    MIP_SENSOR_OVERRANGE_STATUS_DATA_STATUS_NONE    = 0x0000,
    MIP_SENSOR_OVERRANGE_STATUS_DATA_STATUS_ACCEL_X = 0x0001,
    MIP_SENSOR_OVERRANGE_STATUS_DATA_STATUS_ACCEL_Y = 0x0002,
    MIP_SENSOR_OVERRANGE_STATUS_DATA_STATUS_ACCEL_Z = 0x0004,
    MIP_SENSOR_OVERRANGE_STATUS_DATA_STATUS_GYRO_X  = 0x0010,
    MIP_SENSOR_OVERRANGE_STATUS_DATA_STATUS_GYRO_Y  = 0x0020,
    MIP_SENSOR_OVERRANGE_STATUS_DATA_STATUS_GYRO_Z  = 0x0040,
    MIP_SENSOR_OVERRANGE_STATUS_DATA_STATUS_MAG_X   = 0x0100,
    MIP_SENSOR_OVERRANGE_STATUS_DATA_STATUS_MAG_Y   = 0x0200,
    MIP_SENSOR_OVERRANGE_STATUS_DATA_STATUS_MAG_Z   = 0x0400,
    MIP_SENSOR_OVERRANGE_STATUS_DATA_STATUS_PRESS   = 0x1000,
};

struct mip_sensor_overrange_status_data
{
    enum mip_sensor_overrange_status_data_status status;
    
};
void insert_mip_sensor_overrange_status_data(struct mip_serializer* serializer, const struct mip_sensor_overrange_status_data* self);
void extract_mip_sensor_overrange_status_data(struct mip_serializer* serializer, struct mip_sensor_overrange_status_data* self);
void extract_mip_sensor_overrange_status_data_from_field(const struct mip_field* field, void* ptr);

void insert_mip_sensor_overrange_status_data_status(struct mip_serializer* serializer, const enum mip_sensor_overrange_status_data_status self);
void extract_mip_sensor_overrange_status_data_status(struct mip_serializer* serializer, enum mip_sensor_overrange_status_data_status* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_odometer_data  None
///
///@{

struct mip_sensor_odometer_data_data
{
    float speed;
    float uncertainty;
    uint16_t valid_flags;
    
};
void insert_mip_sensor_odometer_data_data(struct mip_serializer* serializer, const struct mip_sensor_odometer_data_data* self);
void extract_mip_sensor_odometer_data_data(struct mip_serializer* serializer, struct mip_sensor_odometer_data_data* self);
void extract_mip_sensor_odometer_data_data_from_field(const struct mip_field* field, void* ptr);

///@}
///

///@}
///@}
///
////////////////////////////////////////////////////////////////////////////////
#ifdef __cplusplus
} // namespace C
} // namespace mip
} // extern "C"
#endif // __cplusplus

