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
///@defgroup FILTER_DATA  FILTER DATA
///
///@{

////////////////////////////////////////////////////////////////////////////////
// Descriptors
////////////////////////////////////////////////////////////////////////////////

enum mip_filter_data_descriptors
{
    MIP_FILTER_DATA_DESC_SET                                         = 0x82,
    
    MIP_DATA_DESC_FILTER_LLH_POS                                     = 0x01,
    MIP_DATA_DESC_FILTER_NED_VEL                                     = 0x02,
    MIP_DATA_DESC_FILTER_ATT_QUATERNION                              = 0x03,
    MIP_DATA_DESC_FILTER_ATT_MATRIX                                  = 0x04,
    MIP_DATA_DESC_FILTER_ATT_EULER_ANGLES                            = 0x05,
    MIP_DATA_DESC_FILTER_GYRO_BIAS                                   = 0x06,
    MIP_DATA_DESC_FILTER_ACCEL_BIAS                                  = 0x07,
    MIP_DATA_DESC_FILTER_POS_UNCERTAINTY                             = 0x08,
    MIP_DATA_DESC_FILTER_VEL_UNCERTAINTY                             = 0x09,
    MIP_DATA_DESC_FILTER_ATT_UNCERTAINTY_EULER                       = 0x0A,
    MIP_DATA_DESC_FILTER_GYRO_BIAS_UNCERTAINTY                       = 0x0B,
    MIP_DATA_DESC_FILTER_ACCEL_BIAS_UNCERTAINTY                      = 0x0C,
    MIP_DATA_DESC_FILTER_FILTER_TIMESTAMP                            = 0x11,
    MIP_DATA_DESC_FILTER_FILTER_STATUS                               = 0x10,
    MIP_DATA_DESC_FILTER_LINEAR_ACCELERATION                         = 0x0D,
    MIP_DATA_DESC_FILTER_GRAVITY_VECTOR                              = 0x13,
    MIP_DATA_DESC_FILTER_COMPENSATED_ACCELERATION                    = 0x1C,
    MIP_DATA_DESC_FILTER_COMPENSATED_ANGULAR_RATE                    = 0x0E,
    MIP_DATA_DESC_FILTER_ATT_UNCERTAINTY_QUATERNION                  = 0x12,
    MIP_DATA_DESC_FILTER_WGS84_GRAVITY                               = 0x0F,
    MIP_DATA_DESC_FILTER_HEADING_UPDATE_STATE                        = 0x14,
    MIP_DATA_DESC_FILTER_MAGNETIC_MODEL                              = 0x15,
    MIP_DATA_DESC_FILTER_ACCEL_SCALE_FACTOR                          = 0x17,
    MIP_DATA_DESC_FILTER_ACCEL_SCALE_FACTOR_UNCERTAINTY              = 0x19,
    MIP_DATA_DESC_FILTER_GYRO_SCALE_FACTOR                           = 0x16,
    MIP_DATA_DESC_FILTER_GYRO_SCALE_FACTOR_UNCERTAINTY               = 0x18,
    MIP_DATA_DESC_FILTER_MAG_BIAS                                    = 0x1A,
    MIP_DATA_DESC_FILTER_MAG_BIAS_UNCERTAINTY                        = 0x1B,
    MIP_DATA_DESC_FILTER_STANDARD_ATMOSPHERE_DATA                    = 0x20,
    MIP_DATA_DESC_FILTER_PRESSURE_ALTITUDE_DATA                      = 0x21,
    MIP_DATA_DESC_FILTER_DENSITY_ALTITUDE_DATA                       = 0x22,
    MIP_DATA_DESC_FILTER_ANTENNA_OFFSET_CORRECTION                   = 0x30,
    MIP_DATA_DESC_FILTER_ANTENNA_OFFSET_CORRECTION_UNCERTAINTY       = 0x31,
    MIP_DATA_DESC_FILTER_MULTI_ANTENNA_OFFSET_CORRECTION             = 0x34,
    MIP_DATA_DESC_FILTER_MULTI_ANTENNA_OFFSET_CORRECTION_UNCERTAINTY = 0x35,
    MIP_DATA_DESC_FILTER_MAG_COMPENSATION_OFFSET                     = 0x25,
    MIP_DATA_DESC_FILTER_MAG_COMPENSATION_MATRIX                     = 0x26,
    MIP_DATA_DESC_FILTER_MAG_COMPENSATION_OFFSET_UNCERTAINTY         = 0x28,
    MIP_DATA_DESC_FILTER_MAG_COMPENSATION_MATRIX_UNCERTAINTY         = 0x29,
    MIP_DATA_DESC_FILTER_MAG_COVARIANCE                              = 0x2A,
    MIP_DATA_DESC_FILTER_MAG_RESIDUAL                                = 0x2C,
    MIP_DATA_DESC_FILTER_CLOCK_CORRECTION                            = 0x32,
    MIP_DATA_DESC_FILTER_CLOCK_CORRECTION_UNCERTAINTY                = 0x33,
    MIP_DATA_DESC_FILTER_GNSS_POS_AID_STATUS                         = 0x43,
    MIP_DATA_DESC_FILTER_GNSS_ATT_AID_STATUS                         = 0x44,
    MIP_DATA_DESC_FILTER_HEAD_AID_STATUS                             = 0x45,
    MIP_DATA_DESC_FILTER_REL_POS_NED                                 = 0x42,
    MIP_DATA_DESC_FILTER_ECEF_POS                                    = 0x40,
    MIP_DATA_DESC_FILTER_ECEF_VEL                                    = 0x41,
    MIP_DATA_DESC_FILTER_ECEF_POS_UNCERTAINTY                        = 0x36,
    MIP_DATA_DESC_FILTER_ECEF_VEL_UNCERTAINTY                        = 0x37,
    MIP_DATA_DESC_FILTER_AID_MEAS_SUMMARY                            = 0x46,
    MIP_DATA_DESC_FILTER_ODOMETER_SCALE_FACTOR_ERROR                 = 0x47,
    MIP_DATA_DESC_FILTER_ODOMETER_SCALE_FACTOR_ERROR_UNCERTAINTY     = 0x48,
    MIP_DATA_DESC_FILTER_GNSS_DUAL_ANTENNA_STATUS                    = 0x49,
    
};
#ifdef __cplusplus
namespace C {
extern "C" {
#endif // __cplusplus


////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////

enum mip_filter_mode
{
    MIP_FILTER_MODE_GX5_STARTUP            = 0,  ///<  
    MIP_FILTER_MODE_GX5_INIT               = 1,  ///<  
    MIP_FILTER_MODE_GX5_RUN_SOLUTION_VALID = 2,  ///<  
    MIP_FILTER_MODE_GX5_RUN_SOLUTION_ERROR = 3,  ///<  
    MIP_FILTER_MODE_GQ7_INIT               = 1,  ///<  
    MIP_FILTER_MODE_GQ7_VERT_GYRO          = 2,  ///<  
    MIP_FILTER_MODE_GQ7_AHRS               = 3,  ///<  
    MIP_FILTER_MODE_GQ7_FULL_NAV           = 4,  ///<  
};
size_t insert_mip_filter_mode(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_filter_mode self);
size_t extract_mip_filter_mode(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_filter_mode* self);

enum mip_filter_dynamics_mode
{
    MIP_FILTER_DYNAMICS_MODE_GX5_PORTABLE   = 1,  ///<  
    MIP_FILTER_DYNAMICS_MODE_GX5_AUTOMOTIVE = 2,  ///<  
    MIP_FILTER_DYNAMICS_MODE_GX5_AIRBORNE   = 3,  ///<  
    MIP_FILTER_DYNAMICS_MODE_GQ7_DEFAULT    = 1,  ///<  
};
size_t insert_mip_filter_dynamics_mode(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_filter_dynamics_mode self);
size_t extract_mip_filter_dynamics_mode(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_filter_dynamics_mode* self);

enum mip_filter_status_flags
{
    MIP_FILTER_STATUS_FLAGS_GX5_INIT_NO_ATTITUDE                           = 0x1000,
    MIP_FILTER_STATUS_FLAGS_GX5_INIT_NO_POSITION_VELOCITY                  = 0x2000,
    MIP_FILTER_STATUS_FLAGS_GX5_RUN_IMU_UNAVAILABLE                        = 0x01,
    MIP_FILTER_STATUS_FLAGS_GX5_RUN_GPS_UNAVAILABLE                        = 0x02,
    MIP_FILTER_STATUS_FLAGS_GX5_RUN_MATRIX_SINGULARITY                     = 0x08,
    MIP_FILTER_STATUS_FLAGS_GX5_RUN_POSITION_COVARIANCE_WARNING            = 0x10,
    MIP_FILTER_STATUS_FLAGS_GX5_RUN_VELOCITY_COVARIANCE_WARNING            = 0x20,
    MIP_FILTER_STATUS_FLAGS_GX5_RUN_ATTITUDE_COVARIANCE_WARNING            = 0x40,
    MIP_FILTER_STATUS_FLAGS_GX5_RUN_NAN_IN_SOLUTION_WARNING                = 0x80,
    MIP_FILTER_STATUS_FLAGS_GX5_RUN_GYRO_BIAS_EST_HIGH_WARNING             = 0x100,
    MIP_FILTER_STATUS_FLAGS_GX5_RUN_ACCEL_BIAS_EST_HIGH_WARNING            = 0x200,
    MIP_FILTER_STATUS_FLAGS_GX5_RUN_GYRO_SCALE_FACTOR_EST_HIGH_WARNING     = 0x400,
    MIP_FILTER_STATUS_FLAGS_GX5_RUN_ACCEL_SCALE_FACTOR_EST_HIGH_WARNING    = 0x800,
    MIP_FILTER_STATUS_FLAGS_GX5_RUN_MAG_BIAS_EST_HIGH_WARNING              = 0x1000,
    MIP_FILTER_STATUS_FLAGS_GX5_RUN_ANT_OFFSET_CORRECTION_EST_HIGH_WARNING = 0x2000,
    MIP_FILTER_STATUS_FLAGS_GX5_RUN_MAG_HARD_IRON_EST_HIGH_WARNING         = 0x4000,
    MIP_FILTER_STATUS_FLAGS_GX5_RUN_MAG_SOFT_IRON_EST_HIGH_WARNING         = 0x8000,
    MIP_FILTER_STATUS_FLAGS_GQ7_FILTER_CONDITION                           = 0x03,
    MIP_FILTER_STATUS_FLAGS_GQ7_ROLL_PITCH_WARNING                         = 0x04,
    MIP_FILTER_STATUS_FLAGS_GQ7_HEADING_WARNING                            = 0x08,
    MIP_FILTER_STATUS_FLAGS_GQ7_POSITION_WARNING                           = 0x10,
    MIP_FILTER_STATUS_FLAGS_GQ7_VELOCITY_WARNING                           = 0x20,
    MIP_FILTER_STATUS_FLAGS_GQ7_IMU_BIAS_WARNING                           = 0x40,
    MIP_FILTER_STATUS_FLAGS_GQ7_GNSS_CLK_WARNING                           = 0x80,
    MIP_FILTER_STATUS_FLAGS_GQ7_ANTENNA_LEVER_ARM_WARNING                  = 0x100,
    MIP_FILTER_STATUS_FLAGS_GQ7_MOUNTING_TRANSFORM_WARNING                 = 0x200,
    MIP_FILTER_STATUS_FLAGS_GQ7_TIME_SYNC_WARNING                          = 0x400,
    MIP_FILTER_STATUS_FLAGS_GQ7_SOLUTION_ERROR                             = 0xF000,
};
size_t insert_mip_filter_status_flags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_filter_status_flags self);
size_t extract_mip_filter_status_flags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_filter_status_flags* self);

enum mip_filter_aiding_measurement_type
{
    MIP_FILTER_AIDING_MEASUREMENT_TYPE_GNSS         = 1,  ///<  
    MIP_FILTER_AIDING_MEASUREMENT_TYPE_DUAL_ANTENNA = 2,  ///<  
    MIP_FILTER_AIDING_MEASUREMENT_TYPE_HEADING      = 3,  ///<  
    MIP_FILTER_AIDING_MEASUREMENT_TYPE_PRESSURE     = 4,  ///<  
    MIP_FILTER_AIDING_MEASUREMENT_TYPE_MAGNETOMETER = 5,  ///<  
    MIP_FILTER_AIDING_MEASUREMENT_TYPE_SPEED        = 6,  ///<  
};
size_t insert_mip_filter_aiding_measurement_type(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_filter_aiding_measurement_type self);
size_t extract_mip_filter_aiding_measurement_type(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_filter_aiding_measurement_type* self);

enum mip_filter_measurement_indicator
{
    MIP_FILTER_MEASUREMENT_INDICATOR_ENABLED               = 0x01,
    MIP_FILTER_MEASUREMENT_INDICATOR_USED                  = 0x02,
    MIP_FILTER_MEASUREMENT_INDICATOR_RESIDUAL_HIGH_WARNING = 0x04,
    MIP_FILTER_MEASUREMENT_INDICATOR_SAMPLE_TIME_WARNING   = 0x08,
    MIP_FILTER_MEASUREMENT_INDICATOR_CONFIGURATION_ERROR   = 0x10,
    MIP_FILTER_MEASUREMENT_INDICATOR_MAX_NUM_MEAS_EXCEEDED = 0x20,
};
size_t insert_mip_filter_measurement_indicator(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_filter_measurement_indicator self);
size_t extract_mip_filter_measurement_indicator(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_filter_measurement_indicator* self);

enum mip_gnss_aid_status_flags
{
    MIP_GNSS_AID_STATUS_FLAGS_TIGHT_COUPLING = 0x01,
    MIP_GNSS_AID_STATUS_FLAGS_DIFFERENTIAL   = 0x02,
    MIP_GNSS_AID_STATUS_FLAGS_INTEGER_FIX    = 0x04,
    MIP_GNSS_AID_STATUS_FLAGS_GPS_L1         = 0x08,
    MIP_GNSS_AID_STATUS_FLAGS_GPS_L2         = 0x10,
    MIP_GNSS_AID_STATUS_FLAGS_GPS_L5         = 0x20,
    MIP_GNSS_AID_STATUS_FLAGS_GLO_L1         = 0x40,
    MIP_GNSS_AID_STATUS_FLAGS_GLO_L2         = 0x80,
    MIP_GNSS_AID_STATUS_FLAGS_GAL_E1         = 0x100,
    MIP_GNSS_AID_STATUS_FLAGS_GAL_E5         = 0x200,
    MIP_GNSS_AID_STATUS_FLAGS_GAL_E6         = 0x400,
    MIP_GNSS_AID_STATUS_FLAGS_BEI_B1         = 0x800,
    MIP_GNSS_AID_STATUS_FLAGS_BEI_B2         = 0x1000,
    MIP_GNSS_AID_STATUS_FLAGS_BEI_B3         = 0x2000,
    MIP_GNSS_AID_STATUS_FLAGS_NO_FIX         = 0x4000,
    MIP_GNSS_AID_STATUS_FLAGS_CONFIG_ERROR   = 0x8000,
};
size_t insert_mip_gnss_aid_status_flags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_gnss_aid_status_flags self);
size_t extract_mip_gnss_aid_status_flags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_gnss_aid_status_flags* self);


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_llh_pos  LLH Position
/// Filter reported position in the WGS84 geodetic frame.
///
///@{

struct mip_filter_llh_pos_data
{
    double                                            latitude;
    double                                            longitude;
    double                                            ellipsoid_height;
    uint16_t                                          valid_flags;
};
size_t insert_mip_filter_llh_pos_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_llh_pos_data* self);
size_t extract_mip_filter_llh_pos_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_llh_pos_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_ned_velocity  None
/// Filter reported velocity in the NED local-level frame.
///
///@{

struct mip_filter_ned_velocity_data
{
    float                                             north;
    float                                             east;
    float                                             down;
    uint16_t                                          valid_flags;
};
size_t insert_mip_filter_ned_velocity_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_ned_velocity_data* self);
size_t extract_mip_filter_ned_velocity_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_ned_velocity_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_attitude_quaternion  None
/// 4x1 vector representation of the quaternion describing the orientation of the device with respect to the NED local-level frame.
/// This quaternion satisfies the following relationship:
/// 
/// EQSTART p^{veh} = q^{-1} p^{ned} q EQEND<br/>
/// 
/// Where:<br/>
/// EQSTART q = (q_w, q_x, q_y, q_z) EQEND is the quaternion desrcribing the rotation. <br/>
/// EQSTART p^ned = (0, v^{ned}_x, v^{ned}_y, v^{ned}_z) EQEND and EQSTART v^{ned} EQEND is a 3-element vector expressed in the NED frame.<br/>
/// EQSTART p^veh = (0, v^{veh}_x, v^{veh}_y, v^{veh}_z) EQEND and EQSTART v^{veh} EQEND is a 3-element vector expressed in the vehicle frame.<br/>
///
///@{

struct mip_filter_attitude_quaternion_data
{
    float                                             q[4];
    uint16_t                                          valid_flags;
};
size_t insert_mip_filter_attitude_quaternion_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_attitude_quaternion_data* self);
size_t extract_mip_filter_attitude_quaternion_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_attitude_quaternion_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_attitude_dcm  None
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
/// The matrix elements are stored is row-major order: EQSTART M_{ned}^{veh} = \begin{bmatrix} M_{11}, M_{12}, M_{13}, M_{21}, M_{22}, M_{23}, M_{31}, M_{32}, M_{33} \end{bmatrix} EQEND
///
///@{

struct mip_filter_attitude_dcm_data
{
    float                                             dcm[9];
    uint16_t                                          valid_flags;
};
size_t insert_mip_filter_attitude_dcm_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_attitude_dcm_data* self);
size_t extract_mip_filter_attitude_dcm_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_attitude_dcm_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_euler_angles  None
/// Filter reported Euler angles describing the orientation of the device with respect to the NED local-level frame.
/// The Euler angles are reported in 3-2-1 (Yaw-Pitch-Roll, AKA Aircraft) order.
///
///@{

struct mip_filter_euler_angles_data
{
    float                                             roll;
    float                                             pitch;
    float                                             yaw;
    uint16_t                                          valid_flags;
};
size_t insert_mip_filter_euler_angles_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_euler_angles_data* self);
size_t extract_mip_filter_euler_angles_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_euler_angles_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_gyro_bias  None
/// Filter reported gyro bias expressed in the sensor frame.
///
///@{

struct mip_filter_gyro_bias_data
{
    float                                             bias[3];
    uint16_t                                          valid_flags;
};
size_t insert_mip_filter_gyro_bias_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_gyro_bias_data* self);
size_t extract_mip_filter_gyro_bias_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_gyro_bias_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_accel_bias  None
/// Filter reported accelerometer bias expressed in the sensor frame.
///
///@{

struct mip_filter_accel_bias_data
{
    float                                             bias[3];
    uint16_t                                          valid_flags;
};
size_t insert_mip_filter_accel_bias_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_accel_bias_data* self);
size_t extract_mip_filter_accel_bias_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_accel_bias_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_llh_pos_uncertainty  LLH Position Uncertainty
/// Filter reported 1-sigma position uncertainty in the NED local-level frame.
///
///@{

struct mip_filter_llh_pos_uncertainty_data
{
    float                                             north;
    float                                             east;
    float                                             down;
    uint16_t                                          valid_flags;
};
size_t insert_mip_filter_llh_pos_uncertainty_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_llh_pos_uncertainty_data* self);
size_t extract_mip_filter_llh_pos_uncertainty_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_llh_pos_uncertainty_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_ned_vel_uncertainty  NED Velocity Uncertainty
/// Filter reported 1-sigma velocity uncertainties in the NED local-level frame.
///
///@{

struct mip_filter_ned_vel_uncertainty_data
{
    float                                             north;
    float                                             east;
    float                                             down;
    uint16_t                                          valid_flags;
};
size_t insert_mip_filter_ned_vel_uncertainty_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_ned_vel_uncertainty_data* self);
size_t extract_mip_filter_ned_vel_uncertainty_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_ned_vel_uncertainty_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_euler_angles_uncertainty  None
/// Filter reported 1-sigma Euler angle uncertainties.
/// The uncertainties are reported in 3-2-1 (Yaw-Pitch-Roll, AKA Aircraft) order.
///
///@{

struct mip_filter_euler_angles_uncertainty_data
{
    float                                             roll;
    float                                             pitch;
    float                                             yaw;
    uint16_t                                          valid_flags;
};
size_t insert_mip_filter_euler_angles_uncertainty_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_euler_angles_uncertainty_data* self);
size_t extract_mip_filter_euler_angles_uncertainty_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_euler_angles_uncertainty_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_gyro_bias_uncertainty  None
/// Filter reported 1-sigma gyro bias uncertainties expressed in the sensor frame.
///
///@{

struct mip_filter_gyro_bias_uncertainty_data
{
    float                                             bias_uncert[3];
    uint16_t                                          valid_flags;
};
size_t insert_mip_filter_gyro_bias_uncertainty_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_gyro_bias_uncertainty_data* self);
size_t extract_mip_filter_gyro_bias_uncertainty_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_gyro_bias_uncertainty_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_accel_bias_uncertainty  None
/// Filter reported 1-sigma accelerometer bias uncertainties expressed in the sensor frame.
///
///@{

struct mip_filter_accel_bias_uncertainty_data
{
    float                                             bias_uncert[3];
    uint16_t                                          valid_flags;
};
size_t insert_mip_filter_accel_bias_uncertainty_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_accel_bias_uncertainty_data* self);
size_t extract_mip_filter_accel_bias_uncertainty_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_accel_bias_uncertainty_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_timestamp  None
/// GPS timestamp of the Filter data
/// 
/// Should the PPS become unavailable, the device will revert to its internal clock, which will cause the reported time to drift from true GPS time.
/// Upon recovering from a PPS outage, the user should expect a jump in the reported GPS time due to the accumulation of internal clock error.
/// If synchronization to an external clock or onboard GNSS receiver (for products that have one) is disabled, this time is equivalent to internal system time.
/// 
/// Note: this data field may be deprecrated in the future. The more flexible shared data field (0x82, 0xD3) should be used instead.
///
///@{

struct mip_filter_timestamp_data
{
    double                                            tow;
    uint16_t                                          week_number;
    uint16_t                                          valid_flags;
};
size_t insert_mip_filter_timestamp_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_timestamp_data* self);
size_t extract_mip_filter_timestamp_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_timestamp_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_status  None
/// Device-specific filter status indicators.
///
///@{

struct mip_filter_status_data
{
    enum mip_filter_mode                              filter_state;
    enum mip_filter_dynamics_mode                     dynamics_mode;
    enum mip_filter_status_flags                      status_flags;
};
size_t insert_mip_filter_status_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_status_data* self);
size_t extract_mip_filter_status_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_status_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_linear_accel  None
/// Filter-compensated linear acceleration expressed in the vehicle frame.
/// Note: The estimated gravity has been removed from this data leaving only linear acceleration.
///
///@{

struct mip_filter_linear_accel_data
{
    float                                             accel[3];
    uint16_t                                          valid_flags;
};
size_t insert_mip_filter_linear_accel_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_linear_accel_data* self);
size_t extract_mip_filter_linear_accel_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_linear_accel_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_gravity_vector  None
/// Filter reported gravity vector expressed in the vehicle frame.
///
///@{

struct mip_filter_gravity_vector_data
{
    float                                             gravity[3];
    uint16_t                                          valid_flags;
};
size_t insert_mip_filter_gravity_vector_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_gravity_vector_data* self);
size_t extract_mip_filter_gravity_vector_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_gravity_vector_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_comp_accel  Compensated Acceleration
/// Filter-compensated acceleration expressed in the vehicle frame.
///
///@{

struct mip_filter_comp_accel_data
{
    float                                             accel[3];
    uint16_t                                          valid_flags;
};
size_t insert_mip_filter_comp_accel_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_comp_accel_data* self);
size_t extract_mip_filter_comp_accel_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_comp_accel_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_comp_angular_rate  None
/// Filter-compensated angular rate expressed in the vehicle frame.
///
///@{

struct mip_filter_comp_angular_rate_data
{
    float                                             gyro[3];
    uint16_t                                          valid_flags;
};
size_t insert_mip_filter_comp_angular_rate_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_comp_angular_rate_data* self);
size_t extract_mip_filter_comp_angular_rate_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_comp_angular_rate_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_quaternion_attitude_uncertainty  None
/// Filter reported quaternion uncertainties.
///
///@{

struct mip_filter_quaternion_attitude_uncertainty_data
{
    float                                             q[4];
    uint16_t                                          valid_flags;
};
size_t insert_mip_filter_quaternion_attitude_uncertainty_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_quaternion_attitude_uncertainty_data* self);
size_t extract_mip_filter_quaternion_attitude_uncertainty_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_quaternion_attitude_uncertainty_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_wgs84_gravity_mag  None
/// Filter reported WGS84 gravity magnitude.
///
///@{

struct mip_filter_wgs84_gravity_mag_data
{
    float                                             magnitude;
    uint16_t                                          valid_flags;
};
size_t insert_mip_filter_wgs84_gravity_mag_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_wgs84_gravity_mag_data* self);
size_t extract_mip_filter_wgs84_gravity_mag_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_wgs84_gravity_mag_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_heading_update_state  None
/// Filter reported heading update state.
/// 
/// Heading updates can be applied from the sources listed below.  Note, some of these sources may be combined.
/// The heading value is always relative to true north.
///
///@{

enum mip_filter_heading_update_state_data_heading_source
{
    MIP_FILTER_HEADING_UPDATE_STATE_DATA_HEADING_SOURCE_NONE                 = 0,  ///<  
    MIP_FILTER_HEADING_UPDATE_STATE_DATA_HEADING_SOURCE_MAGNETOMETER         = 1,  ///<  
    MIP_FILTER_HEADING_UPDATE_STATE_DATA_HEADING_SOURCE_GNSS_VELOCITY_VECTOR = 2,  ///<  
    MIP_FILTER_HEADING_UPDATE_STATE_DATA_HEADING_SOURCE_EXTERNAL             = 4,  ///<  
    MIP_FILTER_HEADING_UPDATE_STATE_DATA_HEADING_SOURCE_DUAL_ANTENNA         = 8,  ///<  
};
size_t insert_mip_filter_heading_update_state_data_heading_source(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_filter_heading_update_state_data_heading_source self);
size_t extract_mip_filter_heading_update_state_data_heading_source(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_filter_heading_update_state_data_heading_source* self);

struct mip_filter_heading_update_state_data
{
    float                                             heading;
    float                                             heading_1sigma;
    enum mip_filter_heading_update_state_data_heading_source source;
    uint16_t                                          valid_flags;
};
size_t insert_mip_filter_heading_update_state_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_heading_update_state_data* self);
size_t extract_mip_filter_heading_update_state_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_heading_update_state_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_magnetic_model  None
/// The World Magnetic Model is used for this data. Please refer to the device user manual for the current version of the model.
/// A valid GNSS location is required for the model to be valid.
///
///@{

struct mip_filter_magnetic_model_data
{
    float                                             intensity_north;
    float                                             intensity_east;
    float                                             intensity_down;
    float                                             inclination;
    float                                             declination;
    uint16_t                                          valid_flags;
};
size_t insert_mip_filter_magnetic_model_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_magnetic_model_data* self);
size_t extract_mip_filter_magnetic_model_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_magnetic_model_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_accel_scale_factor  None
/// Filter reported accelerometer scale factor expressed in the sensor frame.
///
///@{

struct mip_filter_accel_scale_factor_data
{
    float                                             scale_factor[3];
    uint16_t                                          valid_flags;
};
size_t insert_mip_filter_accel_scale_factor_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_accel_scale_factor_data* self);
size_t extract_mip_filter_accel_scale_factor_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_accel_scale_factor_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_accel_scale_factor_uncertainty  None
/// Filter reported 1-sigma accelerometer scale factor uncertainty expressed in the sensor frame.
///
///@{

struct mip_filter_accel_scale_factor_uncertainty_data
{
    float                                             scale_factor_uncert[3];
    uint16_t                                          valid_flags;
};
size_t insert_mip_filter_accel_scale_factor_uncertainty_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_accel_scale_factor_uncertainty_data* self);
size_t extract_mip_filter_accel_scale_factor_uncertainty_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_accel_scale_factor_uncertainty_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_gyro_scale_factor  None
/// Filter reported gyro scale factor expressed in the sensor frame.
///
///@{

struct mip_filter_gyro_scale_factor_data
{
    float                                             scale_factor[3];
    uint16_t                                          valid_flags;
};
size_t insert_mip_filter_gyro_scale_factor_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_gyro_scale_factor_data* self);
size_t extract_mip_filter_gyro_scale_factor_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_gyro_scale_factor_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_gyro_scale_factor_uncertainty  None
/// Filter reported 1-sigma gyro scale factor uncertainty expressed in the sensor frame.
///
///@{

struct mip_filter_gyro_scale_factor_uncertainty_data
{
    float                                             scale_factor_uncert[3];
    uint16_t                                          valid_flags;
};
size_t insert_mip_filter_gyro_scale_factor_uncertainty_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_gyro_scale_factor_uncertainty_data* self);
size_t extract_mip_filter_gyro_scale_factor_uncertainty_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_gyro_scale_factor_uncertainty_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_mag_bias  None
/// Filter reported magnetometer bias expressed in the sensor frame.
///
///@{

struct mip_filter_mag_bias_data
{
    float                                             bias[3];
    uint16_t                                          valid_flags;
};
size_t insert_mip_filter_mag_bias_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_mag_bias_data* self);
size_t extract_mip_filter_mag_bias_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_mag_bias_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_mag_bias_uncertainty  None
/// Filter reported 1-sigma magnetometer bias uncertainty expressed in the sensor frame.
///
///@{

struct mip_filter_mag_bias_uncertainty_data
{
    float                                             bias_uncert[3];
    uint16_t                                          valid_flags;
};
size_t insert_mip_filter_mag_bias_uncertainty_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_mag_bias_uncertainty_data* self);
size_t extract_mip_filter_mag_bias_uncertainty_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_mag_bias_uncertainty_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_standard_atmosphere  None
/// Filter reported standard atmosphere parameters.
/// 
/// The US 1976 Standard Atmosphere Model is used. A valid GNSS location is required for the model to be valid.
///
///@{

struct mip_filter_standard_atmosphere_data
{
    float                                             geometric_altitude;
    float                                             geopotential_altitude;
    float                                             standard_temperature;
    float                                             standard_pressure;
    float                                             standard_density;
    uint16_t                                          valid_flags;
};
size_t insert_mip_filter_standard_atmosphere_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_standard_atmosphere_data* self);
size_t extract_mip_filter_standard_atmosphere_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_standard_atmosphere_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_pressure_altitude  None
/// Filter reported pressure altitude.
/// 
/// The US 1976 Standard Atmosphere Model is used to calculate the pressure altitude in meters.
/// A valid pressure sensor reading is required for the pressure altitude to be valid.
/// The minimum pressure reading supported by the model is 0.0037 mBar, corresponding to an altitude of 84,852 meters.
///
///@{

struct mip_filter_pressure_altitude_data
{
    float                                             pressure_altitude;
    uint16_t                                          valid_flags;
};
size_t insert_mip_filter_pressure_altitude_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_pressure_altitude_data* self);
size_t extract_mip_filter_pressure_altitude_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_pressure_altitude_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_density_altitude  None
///
///@{

struct mip_filter_density_altitude_data
{
    float                                             density_altitude;
    uint16_t                                          valid_flags;
};
size_t insert_mip_filter_density_altitude_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_density_altitude_data* self);
size_t extract_mip_filter_density_altitude_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_density_altitude_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_antenna_offset_correction  None
/// Filter reported GNSS antenna offset in vehicle frame.
/// 
/// This offset added to any previously stored offset vector to compensate for errors in definition.
///
///@{

struct mip_filter_antenna_offset_correction_data
{
    float                                             offset[3];
    uint16_t                                          valid_flags;
};
size_t insert_mip_filter_antenna_offset_correction_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_antenna_offset_correction_data* self);
size_t extract_mip_filter_antenna_offset_correction_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_antenna_offset_correction_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_antenna_offset_correction_uncertainty  None
/// Filter reported 1-sigma GNSS antenna offset uncertainties in vehicle frame.
///
///@{

struct mip_filter_antenna_offset_correction_uncertainty_data
{
    float                                             offset_uncert[3];
    uint16_t                                          valid_flags;
};
size_t insert_mip_filter_antenna_offset_correction_uncertainty_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_antenna_offset_correction_uncertainty_data* self);
size_t extract_mip_filter_antenna_offset_correction_uncertainty_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_antenna_offset_correction_uncertainty_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_multi_antenna_offset_correction  None
/// Filter reported GNSS antenna offset in vehicle frame.
/// 
/// This offset added to any previously stored offset vector to compensate for errors in definition.
///
///@{

struct mip_filter_multi_antenna_offset_correction_data
{
    uint8_t                                           receiver_id;
    float                                             offset[3];
    uint16_t                                          valid_flags;
};
size_t insert_mip_filter_multi_antenna_offset_correction_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_multi_antenna_offset_correction_data* self);
size_t extract_mip_filter_multi_antenna_offset_correction_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_multi_antenna_offset_correction_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_multi_antenna_offset_correction_uncertainty  None
/// Filter reported 1-sigma GNSS antenna offset uncertainties in vehicle frame.
///
///@{

struct mip_filter_multi_antenna_offset_correction_uncertainty_data
{
    uint8_t                                           receiver_id;
    float                                             offset_uncert[3];
    uint16_t                                          valid_flags;
};
size_t insert_mip_filter_multi_antenna_offset_correction_uncertainty_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_multi_antenna_offset_correction_uncertainty_data* self);
size_t extract_mip_filter_multi_antenna_offset_correction_uncertainty_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_multi_antenna_offset_correction_uncertainty_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_magnetometer_offset  None
/// Filter reported magnetometer hard iron offset in sensor frame.
/// 
/// This offset added to any previously stored hard iron offset vector to compensate for magnetometer in-run bias errors.
///
///@{

struct mip_filter_magnetometer_offset_data
{
    float                                             hard_iron[3];
    uint16_t                                          valid_flags;
};
size_t insert_mip_filter_magnetometer_offset_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_magnetometer_offset_data* self);
size_t extract_mip_filter_magnetometer_offset_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_magnetometer_offset_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_magnetometer_matrix  None
/// Filter reported magnetometer soft iron matrix in sensor frame.
/// 
/// This matrix is post multiplied to any previously stored soft iron matrix to compensate for magnetometer in-run errors.
///
///@{

struct mip_filter_magnetometer_matrix_data
{
    float                                             soft_iron[9];
    uint16_t                                          valid_flags;
};
size_t insert_mip_filter_magnetometer_matrix_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_magnetometer_matrix_data* self);
size_t extract_mip_filter_magnetometer_matrix_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_magnetometer_matrix_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_magnetometer_offset_uncertainty  None
/// Filter reported 1-sigma magnetometer hard iron offset uncertainties in sensor frame.
///
///@{

struct mip_filter_magnetometer_offset_uncertainty_data
{
    float                                             hard_iron_uncertainty[3];
    uint16_t                                          valid_flags;
};
size_t insert_mip_filter_magnetometer_offset_uncertainty_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_magnetometer_offset_uncertainty_data* self);
size_t extract_mip_filter_magnetometer_offset_uncertainty_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_magnetometer_offset_uncertainty_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_magnetometer_matrix_uncertainty  None
/// Filter reported 1-sigma magnetometer soft iron matrix uncertainties in sensor frame.
///
///@{

struct mip_filter_magnetometer_matrix_uncertainty_data
{
    float                                             soft_iron_uncertainty[9];
    uint16_t                                          valid_flags;
};
size_t insert_mip_filter_magnetometer_matrix_uncertainty_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_magnetometer_matrix_uncertainty_data* self);
size_t extract_mip_filter_magnetometer_matrix_uncertainty_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_magnetometer_matrix_uncertainty_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_magnetometer_covariance_matrix  None
///
///@{

struct mip_filter_magnetometer_covariance_matrix_data
{
    float                                             covariance[9];
    uint16_t                                          valid_flags;
};
size_t insert_mip_filter_magnetometer_covariance_matrix_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_magnetometer_covariance_matrix_data* self);
size_t extract_mip_filter_magnetometer_covariance_matrix_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_magnetometer_covariance_matrix_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_magnetometer_residual_vector  None
/// Filter reported magnetometer measurement residuals in vehicle frame.
///
///@{

struct mip_filter_magnetometer_residual_vector_data
{
    float                                             residual[3];
    uint16_t                                          valid_flags;
};
size_t insert_mip_filter_magnetometer_residual_vector_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_magnetometer_residual_vector_data* self);
size_t extract_mip_filter_magnetometer_residual_vector_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_magnetometer_residual_vector_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_clock_correction  None
/// Filter reported GNSS receiver clock error parameters.
///
///@{

struct mip_filter_clock_correction_data
{
    uint8_t                                           receiver_id;
    float                                             bias;
    float                                             bias_drift;
    uint16_t                                          valid_flags;
};
size_t insert_mip_filter_clock_correction_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_clock_correction_data* self);
size_t extract_mip_filter_clock_correction_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_clock_correction_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_clock_correction_uncertainty  None
/// Filter reported 1-sigma GNSS receiver clock error parameters.
///
///@{

struct mip_filter_clock_correction_uncertainty_data
{
    uint8_t                                           receiver_id;
    float                                             bias_uncertainty;
    float                                             bias_drift_uncertainty;
    uint16_t                                          valid_flags;
};
size_t insert_mip_filter_clock_correction_uncertainty_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_clock_correction_uncertainty_data* self);
size_t extract_mip_filter_clock_correction_uncertainty_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_clock_correction_uncertainty_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_gnss_pos_aid_status  GNSS Position Aiding Status
/// Filter reported GNSS position aiding status
///
///@{

struct mip_filter_gnss_pos_aid_status_data
{
    uint8_t                                           receiver_id;
    float                                             time_of_week;
    enum mip_gnss_aid_status_flags                    status;
    uint8_t                                           reserved[8];
};
size_t insert_mip_filter_gnss_pos_aid_status_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_gnss_pos_aid_status_data* self);
size_t extract_mip_filter_gnss_pos_aid_status_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_gnss_pos_aid_status_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_gnss_att_aid_status  GNSS Attitude Aiding Status
/// Filter reported dual antenna GNSS attitude aiding status
///
///@{

struct mip_filter_gnss_att_aid_status_data
{
    float                                             time_of_week;
    enum mip_gnss_aid_status_flags                    status;
    uint8_t                                           reserved[8];
};
size_t insert_mip_filter_gnss_att_aid_status_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_gnss_att_aid_status_data* self);
size_t extract_mip_filter_gnss_att_aid_status_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_gnss_att_aid_status_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_head_aid_status  None
/// Filter reported GNSS heading aiding status
///
///@{

enum mip_filter_head_aid_status_data_heading_aid_type
{
    MIP_FILTER_HEAD_AID_STATUS_DATA_HEADING_AID_TYPE_DUAL_ANTENNA     = 1,  ///<  
    MIP_FILTER_HEAD_AID_STATUS_DATA_HEADING_AID_TYPE_EXTERNAL_MESSAGE = 2,  ///<  
};
size_t insert_mip_filter_head_aid_status_data_heading_aid_type(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_filter_head_aid_status_data_heading_aid_type self);
size_t extract_mip_filter_head_aid_status_data_heading_aid_type(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_filter_head_aid_status_data_heading_aid_type* self);

struct mip_filter_head_aid_status_data
{
    float                                             time_of_week;
    enum mip_filter_head_aid_status_data_heading_aid_type type;
    float                                             reserved[2];
};
size_t insert_mip_filter_head_aid_status_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_head_aid_status_data* self);
size_t extract_mip_filter_head_aid_status_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_head_aid_status_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_rel_pos_ned  NED Relative Position
/// Filter reported relative position, with respect to configured reference position
///
///@{

struct mip_filter_rel_pos_ned_data
{
    double                                            relative_position[3];
    uint16_t                                          valid_flags;
};
size_t insert_mip_filter_rel_pos_ned_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_rel_pos_ned_data* self);
size_t extract_mip_filter_rel_pos_ned_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_rel_pos_ned_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_ecef_pos  ECEF Position
/// Filter reported ECEF position
///
///@{

struct mip_filter_ecef_pos_data
{
    double                                            position_ecef[3];
    uint16_t                                          valid_flags;
};
size_t insert_mip_filter_ecef_pos_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_ecef_pos_data* self);
size_t extract_mip_filter_ecef_pos_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_ecef_pos_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_ecef_vel  ECEF Velocity
/// Filter reported ECEF velocity
///
///@{

struct mip_filter_ecef_vel_data
{
    float                                             velocity_ecef[3];
    uint16_t                                          valid_flags;
};
size_t insert_mip_filter_ecef_vel_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_ecef_vel_data* self);
size_t extract_mip_filter_ecef_vel_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_ecef_vel_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_ecef_pos_uncertainty  ECEF Position Uncertainty
/// Filter reported 1-sigma position uncertainty in the ECEF frame.
///
///@{

struct mip_filter_ecef_pos_uncertainty_data
{
    float                                             pos_uncertainty[3];
    uint16_t                                          valid_flags;
};
size_t insert_mip_filter_ecef_pos_uncertainty_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_ecef_pos_uncertainty_data* self);
size_t extract_mip_filter_ecef_pos_uncertainty_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_ecef_pos_uncertainty_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_ecef_vel_uncertainty  ECEF Velocity Uncertainty
/// Filter reported 1-sigma velocity uncertainties in the ECEF frame.
///
///@{

struct mip_filter_ecef_vel_uncertainty_data
{
    float                                             vel_uncertainty[3];
    uint16_t                                          valid_flags;
};
size_t insert_mip_filter_ecef_vel_uncertainty_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_ecef_vel_uncertainty_data* self);
size_t extract_mip_filter_ecef_vel_uncertainty_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_ecef_vel_uncertainty_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_aiding_measurement_summary  None
/// Filter reported aiding measurement summary. This message contains a summary of the specified aiding measurement over the previous measurement interval ending at the specified time.
///
///@{

struct mip_filter_aiding_measurement_summary_data
{
    float                                             time_of_week;
    uint8_t                                           source;
    enum mip_filter_aiding_measurement_type           type;
    enum mip_filter_measurement_indicator             indicator;
};
size_t insert_mip_filter_aiding_measurement_summary_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_aiding_measurement_summary_data* self);
size_t extract_mip_filter_aiding_measurement_summary_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_aiding_measurement_summary_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_odometer_scale_factor_error  Odometer Scale Factor Error
/// Filter reported odometer scale factor error. The total scale factor estimate is the user indicated scale factor, plus the user indicated scale factor times the scale factor error.
///
///@{

struct mip_filter_odometer_scale_factor_error_data
{
    float                                             scale_factor_error;
    uint16_t                                          valid_flags;
};
size_t insert_mip_filter_odometer_scale_factor_error_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_odometer_scale_factor_error_data* self);
size_t extract_mip_filter_odometer_scale_factor_error_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_odometer_scale_factor_error_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_odometer_scale_factor_error_uncertainty  Odometer Scale Factor Error Uncertainty
/// Filter reported odometer scale factor error uncertainty.
///
///@{

struct mip_filter_odometer_scale_factor_error_uncertainty_data
{
    float                                             scale_factor_error_uncertainty;
    uint16_t                                          valid_flags;
};
size_t insert_mip_filter_odometer_scale_factor_error_uncertainty_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_odometer_scale_factor_error_uncertainty_data* self);
size_t extract_mip_filter_odometer_scale_factor_error_uncertainty_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_odometer_scale_factor_error_uncertainty_data* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_filter_gnss_dual_antenna_status  GNSS Dual Antenna Status
/// Summary information for status of GNSS dual antenna heading estimate.
///
///@{

enum mip_filter_gnss_dual_antenna_status_data_fix_type
{
    MIP_FILTER_GNSS_DUAL_ANTENNA_STATUS_DATA_FIX_TYPE_FIX_NONE     = 0,  ///<  
    MIP_FILTER_GNSS_DUAL_ANTENNA_STATUS_DATA_FIX_TYPE_FIX_DA_FLOAT = 1,  ///<  
    MIP_FILTER_GNSS_DUAL_ANTENNA_STATUS_DATA_FIX_TYPE_FIX_DA_FIXED = 2,  ///<  
};
size_t insert_mip_filter_gnss_dual_antenna_status_data_fix_type(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_filter_gnss_dual_antenna_status_data_fix_type self);
size_t extract_mip_filter_gnss_dual_antenna_status_data_fix_type(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_filter_gnss_dual_antenna_status_data_fix_type* self);

enum mip_filter_gnss_dual_antenna_status_data_dual_antenna_status_flags
{
    MIP_FILTER_GNSS_DUAL_ANTENNA_STATUS_DATA_DUAL_ANTENNA_STATUS_FLAGS_RCV_1_DATA_VALID      = 0x01,
    MIP_FILTER_GNSS_DUAL_ANTENNA_STATUS_DATA_DUAL_ANTENNA_STATUS_FLAGS_RCV_2_DATA_VALID      = 0x02,
    MIP_FILTER_GNSS_DUAL_ANTENNA_STATUS_DATA_DUAL_ANTENNA_STATUS_FLAGS_ANTENNA_OFFSETS_VALID = 0x04,
};
size_t insert_mip_filter_gnss_dual_antenna_status_data_dual_antenna_status_flags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_filter_gnss_dual_antenna_status_data_dual_antenna_status_flags self);
size_t extract_mip_filter_gnss_dual_antenna_status_data_dual_antenna_status_flags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_filter_gnss_dual_antenna_status_data_dual_antenna_status_flags* self);

struct mip_filter_gnss_dual_antenna_status_data
{
    float                                             time_of_week;
    float                                             heading;
    float                                             heading_unc;
    enum mip_filter_gnss_dual_antenna_status_data_fix_type fix_type;
    enum mip_filter_gnss_dual_antenna_status_data_dual_antenna_status_flags status_flags;
    uint16_t                                          valid_flags;
};
size_t insert_mip_filter_gnss_dual_antenna_status_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_gnss_dual_antenna_status_data* self);
size_t extract_mip_filter_gnss_dual_antenna_status_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_gnss_dual_antenna_status_data* self);

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
struct MipFieldInfo<C::mip_filter_llh_pos_data>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_LLH_POS;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_filter_llh_pos_data& self)
    {
        return C::insert_mip_filter_llh_pos_data(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_filter_llh_pos_data& self)
    {
        return C::extract_mip_filter_llh_pos_data(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_filter_ned_velocity_data>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_NED_VEL;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_filter_ned_velocity_data& self)
    {
        return C::insert_mip_filter_ned_velocity_data(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_filter_ned_velocity_data& self)
    {
        return C::extract_mip_filter_ned_velocity_data(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_filter_attitude_quaternion_data>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_ATT_QUATERNION;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_filter_attitude_quaternion_data& self)
    {
        return C::insert_mip_filter_attitude_quaternion_data(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_filter_attitude_quaternion_data& self)
    {
        return C::extract_mip_filter_attitude_quaternion_data(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_filter_attitude_dcm_data>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_ATT_MATRIX;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_filter_attitude_dcm_data& self)
    {
        return C::insert_mip_filter_attitude_dcm_data(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_filter_attitude_dcm_data& self)
    {
        return C::extract_mip_filter_attitude_dcm_data(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_filter_euler_angles_data>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_ATT_EULER_ANGLES;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_filter_euler_angles_data& self)
    {
        return C::insert_mip_filter_euler_angles_data(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_filter_euler_angles_data& self)
    {
        return C::extract_mip_filter_euler_angles_data(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_filter_gyro_bias_data>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_GYRO_BIAS;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_filter_gyro_bias_data& self)
    {
        return C::insert_mip_filter_gyro_bias_data(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_filter_gyro_bias_data& self)
    {
        return C::extract_mip_filter_gyro_bias_data(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_filter_accel_bias_data>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_ACCEL_BIAS;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_filter_accel_bias_data& self)
    {
        return C::insert_mip_filter_accel_bias_data(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_filter_accel_bias_data& self)
    {
        return C::extract_mip_filter_accel_bias_data(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_filter_llh_pos_uncertainty_data>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_POS_UNCERTAINTY;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_filter_llh_pos_uncertainty_data& self)
    {
        return C::insert_mip_filter_llh_pos_uncertainty_data(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_filter_llh_pos_uncertainty_data& self)
    {
        return C::extract_mip_filter_llh_pos_uncertainty_data(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_filter_ned_vel_uncertainty_data>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_VEL_UNCERTAINTY;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_filter_ned_vel_uncertainty_data& self)
    {
        return C::insert_mip_filter_ned_vel_uncertainty_data(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_filter_ned_vel_uncertainty_data& self)
    {
        return C::extract_mip_filter_ned_vel_uncertainty_data(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_filter_euler_angles_uncertainty_data>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_ATT_UNCERTAINTY_EULER;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_filter_euler_angles_uncertainty_data& self)
    {
        return C::insert_mip_filter_euler_angles_uncertainty_data(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_filter_euler_angles_uncertainty_data& self)
    {
        return C::extract_mip_filter_euler_angles_uncertainty_data(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_filter_gyro_bias_uncertainty_data>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_GYRO_BIAS_UNCERTAINTY;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_filter_gyro_bias_uncertainty_data& self)
    {
        return C::insert_mip_filter_gyro_bias_uncertainty_data(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_filter_gyro_bias_uncertainty_data& self)
    {
        return C::extract_mip_filter_gyro_bias_uncertainty_data(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_filter_accel_bias_uncertainty_data>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_ACCEL_BIAS_UNCERTAINTY;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_filter_accel_bias_uncertainty_data& self)
    {
        return C::insert_mip_filter_accel_bias_uncertainty_data(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_filter_accel_bias_uncertainty_data& self)
    {
        return C::extract_mip_filter_accel_bias_uncertainty_data(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_filter_timestamp_data>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_FILTER_TIMESTAMP;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_filter_timestamp_data& self)
    {
        return C::insert_mip_filter_timestamp_data(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_filter_timestamp_data& self)
    {
        return C::extract_mip_filter_timestamp_data(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_filter_status_data>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_FILTER_STATUS;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_filter_status_data& self)
    {
        return C::insert_mip_filter_status_data(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_filter_status_data& self)
    {
        return C::extract_mip_filter_status_data(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_filter_linear_accel_data>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_LINEAR_ACCELERATION;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_filter_linear_accel_data& self)
    {
        return C::insert_mip_filter_linear_accel_data(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_filter_linear_accel_data& self)
    {
        return C::extract_mip_filter_linear_accel_data(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_filter_gravity_vector_data>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_GRAVITY_VECTOR;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_filter_gravity_vector_data& self)
    {
        return C::insert_mip_filter_gravity_vector_data(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_filter_gravity_vector_data& self)
    {
        return C::extract_mip_filter_gravity_vector_data(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_filter_comp_accel_data>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_COMPENSATED_ACCELERATION;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_filter_comp_accel_data& self)
    {
        return C::insert_mip_filter_comp_accel_data(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_filter_comp_accel_data& self)
    {
        return C::extract_mip_filter_comp_accel_data(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_filter_comp_angular_rate_data>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_COMPENSATED_ANGULAR_RATE;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_filter_comp_angular_rate_data& self)
    {
        return C::insert_mip_filter_comp_angular_rate_data(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_filter_comp_angular_rate_data& self)
    {
        return C::extract_mip_filter_comp_angular_rate_data(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_filter_quaternion_attitude_uncertainty_data>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_ATT_UNCERTAINTY_QUATERNION;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_filter_quaternion_attitude_uncertainty_data& self)
    {
        return C::insert_mip_filter_quaternion_attitude_uncertainty_data(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_filter_quaternion_attitude_uncertainty_data& self)
    {
        return C::extract_mip_filter_quaternion_attitude_uncertainty_data(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_filter_wgs84_gravity_mag_data>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_WGS84_GRAVITY;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_filter_wgs84_gravity_mag_data& self)
    {
        return C::insert_mip_filter_wgs84_gravity_mag_data(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_filter_wgs84_gravity_mag_data& self)
    {
        return C::extract_mip_filter_wgs84_gravity_mag_data(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_filter_heading_update_state_data>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_HEADING_UPDATE_STATE;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_filter_heading_update_state_data& self)
    {
        return C::insert_mip_filter_heading_update_state_data(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_filter_heading_update_state_data& self)
    {
        return C::extract_mip_filter_heading_update_state_data(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_filter_magnetic_model_data>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_MAGNETIC_MODEL;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_filter_magnetic_model_data& self)
    {
        return C::insert_mip_filter_magnetic_model_data(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_filter_magnetic_model_data& self)
    {
        return C::extract_mip_filter_magnetic_model_data(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_filter_accel_scale_factor_data>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_ACCEL_SCALE_FACTOR;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_filter_accel_scale_factor_data& self)
    {
        return C::insert_mip_filter_accel_scale_factor_data(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_filter_accel_scale_factor_data& self)
    {
        return C::extract_mip_filter_accel_scale_factor_data(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_filter_accel_scale_factor_uncertainty_data>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_ACCEL_SCALE_FACTOR_UNCERTAINTY;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_filter_accel_scale_factor_uncertainty_data& self)
    {
        return C::insert_mip_filter_accel_scale_factor_uncertainty_data(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_filter_accel_scale_factor_uncertainty_data& self)
    {
        return C::extract_mip_filter_accel_scale_factor_uncertainty_data(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_filter_gyro_scale_factor_data>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_GYRO_SCALE_FACTOR;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_filter_gyro_scale_factor_data& self)
    {
        return C::insert_mip_filter_gyro_scale_factor_data(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_filter_gyro_scale_factor_data& self)
    {
        return C::extract_mip_filter_gyro_scale_factor_data(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_filter_gyro_scale_factor_uncertainty_data>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_GYRO_SCALE_FACTOR_UNCERTAINTY;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_filter_gyro_scale_factor_uncertainty_data& self)
    {
        return C::insert_mip_filter_gyro_scale_factor_uncertainty_data(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_filter_gyro_scale_factor_uncertainty_data& self)
    {
        return C::extract_mip_filter_gyro_scale_factor_uncertainty_data(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_filter_mag_bias_data>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_MAG_BIAS;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_filter_mag_bias_data& self)
    {
        return C::insert_mip_filter_mag_bias_data(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_filter_mag_bias_data& self)
    {
        return C::extract_mip_filter_mag_bias_data(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_filter_mag_bias_uncertainty_data>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_MAG_BIAS_UNCERTAINTY;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_filter_mag_bias_uncertainty_data& self)
    {
        return C::insert_mip_filter_mag_bias_uncertainty_data(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_filter_mag_bias_uncertainty_data& self)
    {
        return C::extract_mip_filter_mag_bias_uncertainty_data(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_filter_standard_atmosphere_data>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_STANDARD_ATMOSPHERE_DATA;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_filter_standard_atmosphere_data& self)
    {
        return C::insert_mip_filter_standard_atmosphere_data(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_filter_standard_atmosphere_data& self)
    {
        return C::extract_mip_filter_standard_atmosphere_data(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_filter_pressure_altitude_data>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_PRESSURE_ALTITUDE_DATA;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_filter_pressure_altitude_data& self)
    {
        return C::insert_mip_filter_pressure_altitude_data(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_filter_pressure_altitude_data& self)
    {
        return C::extract_mip_filter_pressure_altitude_data(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_filter_density_altitude_data>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_DENSITY_ALTITUDE_DATA;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_filter_density_altitude_data& self)
    {
        return C::insert_mip_filter_density_altitude_data(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_filter_density_altitude_data& self)
    {
        return C::extract_mip_filter_density_altitude_data(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_filter_antenna_offset_correction_data>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_ANTENNA_OFFSET_CORRECTION;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_filter_antenna_offset_correction_data& self)
    {
        return C::insert_mip_filter_antenna_offset_correction_data(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_filter_antenna_offset_correction_data& self)
    {
        return C::extract_mip_filter_antenna_offset_correction_data(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_filter_antenna_offset_correction_uncertainty_data>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_ANTENNA_OFFSET_CORRECTION_UNCERTAINTY;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_filter_antenna_offset_correction_uncertainty_data& self)
    {
        return C::insert_mip_filter_antenna_offset_correction_uncertainty_data(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_filter_antenna_offset_correction_uncertainty_data& self)
    {
        return C::extract_mip_filter_antenna_offset_correction_uncertainty_data(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_filter_multi_antenna_offset_correction_data>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_MULTI_ANTENNA_OFFSET_CORRECTION;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_filter_multi_antenna_offset_correction_data& self)
    {
        return C::insert_mip_filter_multi_antenna_offset_correction_data(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_filter_multi_antenna_offset_correction_data& self)
    {
        return C::extract_mip_filter_multi_antenna_offset_correction_data(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_filter_multi_antenna_offset_correction_uncertainty_data>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_MULTI_ANTENNA_OFFSET_CORRECTION_UNCERTAINTY;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_filter_multi_antenna_offset_correction_uncertainty_data& self)
    {
        return C::insert_mip_filter_multi_antenna_offset_correction_uncertainty_data(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_filter_multi_antenna_offset_correction_uncertainty_data& self)
    {
        return C::extract_mip_filter_multi_antenna_offset_correction_uncertainty_data(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_filter_magnetometer_offset_data>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_MAG_COMPENSATION_OFFSET;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_filter_magnetometer_offset_data& self)
    {
        return C::insert_mip_filter_magnetometer_offset_data(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_filter_magnetometer_offset_data& self)
    {
        return C::extract_mip_filter_magnetometer_offset_data(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_filter_magnetometer_matrix_data>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_MAG_COMPENSATION_MATRIX;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_filter_magnetometer_matrix_data& self)
    {
        return C::insert_mip_filter_magnetometer_matrix_data(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_filter_magnetometer_matrix_data& self)
    {
        return C::extract_mip_filter_magnetometer_matrix_data(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_filter_magnetometer_offset_uncertainty_data>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_MAG_COMPENSATION_OFFSET_UNCERTAINTY;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_filter_magnetometer_offset_uncertainty_data& self)
    {
        return C::insert_mip_filter_magnetometer_offset_uncertainty_data(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_filter_magnetometer_offset_uncertainty_data& self)
    {
        return C::extract_mip_filter_magnetometer_offset_uncertainty_data(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_filter_magnetometer_matrix_uncertainty_data>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_MAG_COMPENSATION_MATRIX_UNCERTAINTY;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_filter_magnetometer_matrix_uncertainty_data& self)
    {
        return C::insert_mip_filter_magnetometer_matrix_uncertainty_data(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_filter_magnetometer_matrix_uncertainty_data& self)
    {
        return C::extract_mip_filter_magnetometer_matrix_uncertainty_data(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_filter_magnetometer_covariance_matrix_data>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_MAG_COVARIANCE;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_filter_magnetometer_covariance_matrix_data& self)
    {
        return C::insert_mip_filter_magnetometer_covariance_matrix_data(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_filter_magnetometer_covariance_matrix_data& self)
    {
        return C::extract_mip_filter_magnetometer_covariance_matrix_data(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_filter_magnetometer_residual_vector_data>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_MAG_RESIDUAL;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_filter_magnetometer_residual_vector_data& self)
    {
        return C::insert_mip_filter_magnetometer_residual_vector_data(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_filter_magnetometer_residual_vector_data& self)
    {
        return C::extract_mip_filter_magnetometer_residual_vector_data(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_filter_clock_correction_data>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_CLOCK_CORRECTION;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_filter_clock_correction_data& self)
    {
        return C::insert_mip_filter_clock_correction_data(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_filter_clock_correction_data& self)
    {
        return C::extract_mip_filter_clock_correction_data(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_filter_clock_correction_uncertainty_data>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_CLOCK_CORRECTION_UNCERTAINTY;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_filter_clock_correction_uncertainty_data& self)
    {
        return C::insert_mip_filter_clock_correction_uncertainty_data(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_filter_clock_correction_uncertainty_data& self)
    {
        return C::extract_mip_filter_clock_correction_uncertainty_data(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_filter_gnss_pos_aid_status_data>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_GNSS_POS_AID_STATUS;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_filter_gnss_pos_aid_status_data& self)
    {
        return C::insert_mip_filter_gnss_pos_aid_status_data(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_filter_gnss_pos_aid_status_data& self)
    {
        return C::extract_mip_filter_gnss_pos_aid_status_data(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_filter_gnss_att_aid_status_data>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_GNSS_ATT_AID_STATUS;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_filter_gnss_att_aid_status_data& self)
    {
        return C::insert_mip_filter_gnss_att_aid_status_data(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_filter_gnss_att_aid_status_data& self)
    {
        return C::extract_mip_filter_gnss_att_aid_status_data(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_filter_head_aid_status_data>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_HEAD_AID_STATUS;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_filter_head_aid_status_data& self)
    {
        return C::insert_mip_filter_head_aid_status_data(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_filter_head_aid_status_data& self)
    {
        return C::extract_mip_filter_head_aid_status_data(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_filter_rel_pos_ned_data>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_REL_POS_NED;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_filter_rel_pos_ned_data& self)
    {
        return C::insert_mip_filter_rel_pos_ned_data(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_filter_rel_pos_ned_data& self)
    {
        return C::extract_mip_filter_rel_pos_ned_data(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_filter_ecef_pos_data>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_ECEF_POS;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_filter_ecef_pos_data& self)
    {
        return C::insert_mip_filter_ecef_pos_data(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_filter_ecef_pos_data& self)
    {
        return C::extract_mip_filter_ecef_pos_data(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_filter_ecef_vel_data>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_ECEF_VEL;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_filter_ecef_vel_data& self)
    {
        return C::insert_mip_filter_ecef_vel_data(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_filter_ecef_vel_data& self)
    {
        return C::extract_mip_filter_ecef_vel_data(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_filter_ecef_pos_uncertainty_data>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_ECEF_POS_UNCERTAINTY;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_filter_ecef_pos_uncertainty_data& self)
    {
        return C::insert_mip_filter_ecef_pos_uncertainty_data(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_filter_ecef_pos_uncertainty_data& self)
    {
        return C::extract_mip_filter_ecef_pos_uncertainty_data(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_filter_ecef_vel_uncertainty_data>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_ECEF_VEL_UNCERTAINTY;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_filter_ecef_vel_uncertainty_data& self)
    {
        return C::insert_mip_filter_ecef_vel_uncertainty_data(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_filter_ecef_vel_uncertainty_data& self)
    {
        return C::extract_mip_filter_ecef_vel_uncertainty_data(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_filter_aiding_measurement_summary_data>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_AID_MEAS_SUMMARY;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_filter_aiding_measurement_summary_data& self)
    {
        return C::insert_mip_filter_aiding_measurement_summary_data(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_filter_aiding_measurement_summary_data& self)
    {
        return C::extract_mip_filter_aiding_measurement_summary_data(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_filter_odometer_scale_factor_error_data>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_ODOMETER_SCALE_FACTOR_ERROR;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_filter_odometer_scale_factor_error_data& self)
    {
        return C::insert_mip_filter_odometer_scale_factor_error_data(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_filter_odometer_scale_factor_error_data& self)
    {
        return C::extract_mip_filter_odometer_scale_factor_error_data(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_filter_odometer_scale_factor_error_uncertainty_data>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_ODOMETER_SCALE_FACTOR_ERROR_UNCERTAINTY;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_filter_odometer_scale_factor_error_uncertainty_data& self)
    {
        return C::insert_mip_filter_odometer_scale_factor_error_uncertainty_data(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_filter_odometer_scale_factor_error_uncertainty_data& self)
    {
        return C::extract_mip_filter_odometer_scale_factor_error_uncertainty_data(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_filter_gnss_dual_antenna_status_data>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_GNSS_DUAL_ANTENNA_STATUS;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_filter_gnss_dual_antenna_status_data& self)
    {
        return C::insert_mip_filter_gnss_dual_antenna_status_data(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_filter_gnss_dual_antenna_status_data& self)
    {
        return C::extract_mip_filter_gnss_dual_antenna_status_data(buffer, bufferSize, offset, &self);
    }
};



} // namespace mscl
#endif // __cplusplus
