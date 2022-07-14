#pragma once

#include "descriptors.h"
#include "../mip_result.h"

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
namespace mscl {
extern "C" {
#endif // __cplusplus

struct MipInterfaceState;

////////////////////////////////////////////////////////////////////////////////
///@addtogroup MipData
///@{
///@defgroup FILTER_DATA  FILTER DATA
///
///@{

////////////////////////////////////////////////////////////////////////////////
// Descriptors
////////////////////////////////////////////////////////////////////////////////

enum MipFilterDataDescriptors
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

////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////

enum MipFilterMode
{
    MIPFILTERMODE_GX5_STARTUP            = 0,  ///<  
    MIPFILTERMODE_GX5_INIT               = 1,  ///<  
    MIPFILTERMODE_GX5_RUN_SOLUTION_VALID = 2,  ///<  
    MIPFILTERMODE_GX5_RUN_SOLUTION_ERROR = 3,  ///<  
    MIPFILTERMODE_GQ7_INIT               = 1,  ///<  
    MIPFILTERMODE_GQ7_VERT_GYRO          = 2,  ///<  
    MIPFILTERMODE_GQ7_AHRS               = 3,  ///<  
    MIPFILTERMODE_GQ7_FULL_NAV           = 4,  ///<  
};
size_t insert_MipFilterMode(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipFilterMode self);
size_t extract_MipFilterMode(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipFilterMode* self);

enum MipFilterDynamicsMode
{
    MIPFILTERDYNAMICSMODE_GX5_PORTABLE   = 1,  ///<  
    MIPFILTERDYNAMICSMODE_GX5_AUTOMOTIVE = 2,  ///<  
    MIPFILTERDYNAMICSMODE_GX5_AIRBORNE   = 3,  ///<  
    MIPFILTERDYNAMICSMODE_GQ7_DEFAULT    = 1,  ///<  
};
size_t insert_MipFilterDynamicsMode(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipFilterDynamicsMode self);
size_t extract_MipFilterDynamicsMode(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipFilterDynamicsMode* self);

enum MipFilterStatusFlags
{
    MIPFILTERSTATUSFLAGS_GX5_INIT_NO_ATTITUDE                           = 0x1000,
    MIPFILTERSTATUSFLAGS_GX5_INIT_NO_POSITION_VELOCITY                  = 0x2000,
    MIPFILTERSTATUSFLAGS_GX5_RUN_IMU_UNAVAILABLE                        = 0x01,
    MIPFILTERSTATUSFLAGS_GX5_RUN_GPS_UNAVAILABLE                        = 0x02,
    MIPFILTERSTATUSFLAGS_GX5_RUN_MATRIX_SINGULARITY                     = 0x08,
    MIPFILTERSTATUSFLAGS_GX5_RUN_POSITION_COVARIANCE_WARNING            = 0x10,
    MIPFILTERSTATUSFLAGS_GX5_RUN_VELOCITY_COVARIANCE_WARNING            = 0x20,
    MIPFILTERSTATUSFLAGS_GX5_RUN_ATTITUDE_COVARIANCE_WARNING            = 0x40,
    MIPFILTERSTATUSFLAGS_GX5_RUN_NAN_IN_SOLUTION_WARNING                = 0x80,
    MIPFILTERSTATUSFLAGS_GX5_RUN_GYRO_BIAS_EST_HIGH_WARNING             = 0x100,
    MIPFILTERSTATUSFLAGS_GX5_RUN_ACCEL_BIAS_EST_HIGH_WARNING            = 0x200,
    MIPFILTERSTATUSFLAGS_GX5_RUN_GYRO_SCALE_FACTOR_EST_HIGH_WARNING     = 0x400,
    MIPFILTERSTATUSFLAGS_GX5_RUN_ACCEL_SCALE_FACTOR_EST_HIGH_WARNING    = 0x800,
    MIPFILTERSTATUSFLAGS_GX5_RUN_MAG_BIAS_EST_HIGH_WARNING              = 0x1000,
    MIPFILTERSTATUSFLAGS_GX5_RUN_ANT_OFFSET_CORRECTION_EST_HIGH_WARNING = 0x2000,
    MIPFILTERSTATUSFLAGS_GX5_RUN_MAG_HARD_IRON_EST_HIGH_WARNING         = 0x4000,
    MIPFILTERSTATUSFLAGS_GX5_RUN_MAG_SOFT_IRON_EST_HIGH_WARNING         = 0x8000,
    MIPFILTERSTATUSFLAGS_GQ7_FILTER_CONDITION                           = 0x03,
    MIPFILTERSTATUSFLAGS_GQ7_ROLL_PITCH_WARNING                         = 0x04,
    MIPFILTERSTATUSFLAGS_GQ7_HEADING_WARNING                            = 0x08,
    MIPFILTERSTATUSFLAGS_GQ7_POSITION_WARNING                           = 0x10,
    MIPFILTERSTATUSFLAGS_GQ7_VELOCITY_WARNING                           = 0x20,
    MIPFILTERSTATUSFLAGS_GQ7_IMU_BIAS_WARNING                           = 0x40,
    MIPFILTERSTATUSFLAGS_GQ7_GNSS_CLK_WARNING                           = 0x80,
    MIPFILTERSTATUSFLAGS_GQ7_ANTENNA_LEVER_ARM_WARNING                  = 0x100,
    MIPFILTERSTATUSFLAGS_GQ7_MOUNTING_TRANSFORM_WARNING                 = 0x200,
    MIPFILTERSTATUSFLAGS_GQ7_TIME_SYNC_WARNING                          = 0x400,
    MIPFILTERSTATUSFLAGS_GQ7_SOLUTION_ERROR                             = 0xF000,
};
size_t insert_MipFilterStatusFlags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipFilterStatusFlags self);
size_t extract_MipFilterStatusFlags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipFilterStatusFlags* self);

enum MipFilterAidingMeasurementType
{
    MIPFILTERAIDINGMEASUREMENTTYPE_GNSS         = 1,  ///<  
    MIPFILTERAIDINGMEASUREMENTTYPE_DUAL_ANTENNA = 2,  ///<  
    MIPFILTERAIDINGMEASUREMENTTYPE_HEADING      = 3,  ///<  
    MIPFILTERAIDINGMEASUREMENTTYPE_PRESSURE     = 4,  ///<  
    MIPFILTERAIDINGMEASUREMENTTYPE_MAGNETOMETER = 5,  ///<  
    MIPFILTERAIDINGMEASUREMENTTYPE_SPEED        = 6,  ///<  
};
size_t insert_MipFilterAidingMeasurementType(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipFilterAidingMeasurementType self);
size_t extract_MipFilterAidingMeasurementType(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipFilterAidingMeasurementType* self);

enum MipFilterMeasurementIndicator
{
    MIPFILTERMEASUREMENTINDICATOR_ENABLED               = 0x01,
    MIPFILTERMEASUREMENTINDICATOR_USED                  = 0x02,
    MIPFILTERMEASUREMENTINDICATOR_RESIDUAL_HIGH_WARNING = 0x04,
    MIPFILTERMEASUREMENTINDICATOR_SAMPLE_TIME_WARNING   = 0x08,
    MIPFILTERMEASUREMENTINDICATOR_CONFIGURATION_ERROR   = 0x10,
    MIPFILTERMEASUREMENTINDICATOR_MAX_NUM_MEAS_EXCEEDED = 0x20,
};
size_t insert_MipFilterMeasurementIndicator(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipFilterMeasurementIndicator self);
size_t extract_MipFilterMeasurementIndicator(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipFilterMeasurementIndicator* self);

enum MipGnssAidStatusFlags
{
    MIPGNSSAIDSTATUSFLAGS_TIGHT_COUPLING = 0x01,
    MIPGNSSAIDSTATUSFLAGS_DIFFERENTIAL   = 0x02,
    MIPGNSSAIDSTATUSFLAGS_INTEGER_FIX    = 0x04,
    MIPGNSSAIDSTATUSFLAGS_GPS_L1         = 0x08,
    MIPGNSSAIDSTATUSFLAGS_GPS_L2         = 0x10,
    MIPGNSSAIDSTATUSFLAGS_GPS_L5         = 0x20,
    MIPGNSSAIDSTATUSFLAGS_GLO_L1         = 0x40,
    MIPGNSSAIDSTATUSFLAGS_GLO_L2         = 0x80,
    MIPGNSSAIDSTATUSFLAGS_GAL_E1         = 0x100,
    MIPGNSSAIDSTATUSFLAGS_GAL_E5         = 0x200,
    MIPGNSSAIDSTATUSFLAGS_GAL_E6         = 0x400,
    MIPGNSSAIDSTATUSFLAGS_BEI_B1         = 0x800,
    MIPGNSSAIDSTATUSFLAGS_BEI_B2         = 0x1000,
    MIPGNSSAIDSTATUSFLAGS_BEI_B3         = 0x2000,
    MIPGNSSAIDSTATUSFLAGS_NO_FIX         = 0x4000,
    MIPGNSSAIDSTATUSFLAGS_CONFIG_ERROR   = 0x8000,
};
size_t insert_MipGnssAidStatusFlags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipGnssAidStatusFlags self);
size_t extract_MipGnssAidStatusFlags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipGnssAidStatusFlags* self);


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_filter_llh_pos  Filter Llh Pos
/// Filter reported position in the WGS84 geodetic frame.
///
///@{

struct MipData_Filter_LlhPos
{
    double                                            latitude;
    double                                            longitude;
    double                                            ellipsoid_height;
    uint16_t                                          valid_flags;
};
size_t insert_MipData_Filter_LlhPos(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_LlhPos* self);
size_t extract_MipData_Filter_LlhPos(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_LlhPos* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_filter_ned_velocity  Filter Ned Velocity
/// Filter reported velocity in the NED local-level frame.
///
///@{

struct MipData_Filter_NedVelocity
{
    float                                             north;
    float                                             east;
    float                                             down;
    uint16_t                                          valid_flags;
};
size_t insert_MipData_Filter_NedVelocity(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_NedVelocity* self);
size_t extract_MipData_Filter_NedVelocity(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_NedVelocity* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_filter_attitude_quaternion  Filter Attitude Quaternion
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

struct MipData_Filter_AttitudeQuaternion
{
    float                                             q[4];
    uint16_t                                          valid_flags;
};
size_t insert_MipData_Filter_AttitudeQuaternion(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_AttitudeQuaternion* self);
size_t extract_MipData_Filter_AttitudeQuaternion(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_AttitudeQuaternion* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_filter_attitude_dcm  Filter Attitude Dcm
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

struct MipData_Filter_AttitudeDcm
{
    float                                             dcm[9];
    uint16_t                                          valid_flags;
};
size_t insert_MipData_Filter_AttitudeDcm(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_AttitudeDcm* self);
size_t extract_MipData_Filter_AttitudeDcm(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_AttitudeDcm* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_filter_euler_angles  Filter Euler Angles
/// Filter reported Euler angles describing the orientation of the device with respect to the NED local-level frame.
/// The Euler angles are reported in 3-2-1 (Yaw-Pitch-Roll, AKA Aircraft) order.
///
///@{

struct MipData_Filter_EulerAngles
{
    float                                             roll;
    float                                             pitch;
    float                                             yaw;
    uint16_t                                          valid_flags;
};
size_t insert_MipData_Filter_EulerAngles(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_EulerAngles* self);
size_t extract_MipData_Filter_EulerAngles(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_EulerAngles* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_filter_gyro_bias  Filter Gyro Bias
/// Filter reported gyro bias expressed in the sensor frame.
///
///@{

struct MipData_Filter_GyroBias
{
    float                                             bias[3];
    uint16_t                                          valid_flags;
};
size_t insert_MipData_Filter_GyroBias(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_GyroBias* self);
size_t extract_MipData_Filter_GyroBias(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_GyroBias* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_filter_accel_bias  Filter Accel Bias
/// Filter reported accelerometer bias expressed in the sensor frame.
///
///@{

struct MipData_Filter_AccelBias
{
    float                                             bias[3];
    uint16_t                                          valid_flags;
};
size_t insert_MipData_Filter_AccelBias(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_AccelBias* self);
size_t extract_MipData_Filter_AccelBias(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_AccelBias* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_filter_llh_pos_uncertainty  Filter Llh Pos Uncertainty
/// Filter reported 1-sigma position uncertainty in the NED local-level frame.
///
///@{

struct MipData_Filter_LlhPosUncertainty
{
    float                                             north;
    float                                             east;
    float                                             down;
    uint16_t                                          valid_flags;
};
size_t insert_MipData_Filter_LlhPosUncertainty(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_LlhPosUncertainty* self);
size_t extract_MipData_Filter_LlhPosUncertainty(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_LlhPosUncertainty* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_filter_ned_vel_uncertainty  Filter Ned Vel Uncertainty
/// Filter reported 1-sigma velocity uncertainties in the NED local-level frame.
///
///@{

struct MipData_Filter_NedVelUncertainty
{
    float                                             north;
    float                                             east;
    float                                             down;
    uint16_t                                          valid_flags;
};
size_t insert_MipData_Filter_NedVelUncertainty(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_NedVelUncertainty* self);
size_t extract_MipData_Filter_NedVelUncertainty(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_NedVelUncertainty* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_filter_euler_angles_uncertainty  Filter Euler Angles Uncertainty
/// Filter reported 1-sigma Euler angle uncertainties.
/// The uncertainties are reported in 3-2-1 (Yaw-Pitch-Roll, AKA Aircraft) order.
///
///@{

struct MipData_Filter_EulerAnglesUncertainty
{
    float                                             roll;
    float                                             pitch;
    float                                             yaw;
    uint16_t                                          valid_flags;
};
size_t insert_MipData_Filter_EulerAnglesUncertainty(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_EulerAnglesUncertainty* self);
size_t extract_MipData_Filter_EulerAnglesUncertainty(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_EulerAnglesUncertainty* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_filter_gyro_bias_uncertainty  Filter Gyro Bias Uncertainty
/// Filter reported 1-sigma gyro bias uncertainties expressed in the sensor frame.
///
///@{

struct MipData_Filter_GyroBiasUncertainty
{
    float                                             bias_uncert[3];
    uint16_t                                          valid_flags;
};
size_t insert_MipData_Filter_GyroBiasUncertainty(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_GyroBiasUncertainty* self);
size_t extract_MipData_Filter_GyroBiasUncertainty(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_GyroBiasUncertainty* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_filter_accel_bias_uncertainty  Filter Accel Bias Uncertainty
/// Filter reported 1-sigma accelerometer bias uncertainties expressed in the sensor frame.
///
///@{

struct MipData_Filter_AccelBiasUncertainty
{
    float                                             bias_uncert[3];
    uint16_t                                          valid_flags;
};
size_t insert_MipData_Filter_AccelBiasUncertainty(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_AccelBiasUncertainty* self);
size_t extract_MipData_Filter_AccelBiasUncertainty(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_AccelBiasUncertainty* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_filter_timestamp  Filter Timestamp
/// GPS timestamp of the Filter data
/// 
/// Should the PPS become unavailable, the device will revert to its internal clock, which will cause the reported time to drift from true GPS time.
/// Upon recovering from a PPS outage, the user should expect a jump in the reported GPS time due to the accumulation of internal clock error.
/// If synchronization to an external clock or onboard GNSS receiver (for products that have one) is disabled, this time is equivalent to internal system time.
/// 
/// Note: this data field may be deprecrated in the future. The more flexible shared data field (0x82, 0xD3) should be used instead.
///
///@{

struct MipData_Filter_Timestamp
{
    double                                            tow;
    uint16_t                                          week_number;
    uint16_t                                          valid_flags;
};
size_t insert_MipData_Filter_Timestamp(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_Timestamp* self);
size_t extract_MipData_Filter_Timestamp(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_Timestamp* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_filter_status  Filter Status
/// Device-specific filter status indicators.
///
///@{

struct MipData_Filter_Status
{
    enum MipFilterMode                                filter_state;
    enum MipFilterDynamicsMode                        dynamics_mode;
    enum MipFilterStatusFlags                         status_flags;
};
size_t insert_MipData_Filter_Status(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_Status* self);
size_t extract_MipData_Filter_Status(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_Status* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_filter_linear_accel  Filter Linear Accel
/// Filter-compensated linear acceleration expressed in the vehicle frame.
/// Note: The estimated gravity has been removed from this data leaving only linear acceleration.
///
///@{

struct MipData_Filter_LinearAccel
{
    float                                             accel[3];
    uint16_t                                          valid_flags;
};
size_t insert_MipData_Filter_LinearAccel(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_LinearAccel* self);
size_t extract_MipData_Filter_LinearAccel(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_LinearAccel* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_filter_gravity_vector  Filter Gravity Vector
/// Filter reported gravity vector expressed in the vehicle frame.
///
///@{

struct MipData_Filter_GravityVector
{
    float                                             gravity[3];
    uint16_t                                          valid_flags;
};
size_t insert_MipData_Filter_GravityVector(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_GravityVector* self);
size_t extract_MipData_Filter_GravityVector(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_GravityVector* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_filter_comp_accel  Filter Comp Accel
/// Filter-compensated acceleration expressed in the vehicle frame.
///
///@{

struct MipData_Filter_CompAccel
{
    float                                             accel[3];
    uint16_t                                          valid_flags;
};
size_t insert_MipData_Filter_CompAccel(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_CompAccel* self);
size_t extract_MipData_Filter_CompAccel(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_CompAccel* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_filter_comp_angular_rate  Filter Comp Angular Rate
/// Filter-compensated angular rate expressed in the vehicle frame.
///
///@{

struct MipData_Filter_CompAngularRate
{
    float                                             gyro[3];
    uint16_t                                          valid_flags;
};
size_t insert_MipData_Filter_CompAngularRate(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_CompAngularRate* self);
size_t extract_MipData_Filter_CompAngularRate(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_CompAngularRate* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_filter_quaternion_attitude_uncertainty  Filter Quaternion Attitude Uncertainty
/// Filter reported quaternion uncertainties.
///
///@{

struct MipData_Filter_QuaternionAttitudeUncertainty
{
    float                                             q[4];
    uint16_t                                          valid_flags;
};
size_t insert_MipData_Filter_QuaternionAttitudeUncertainty(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_QuaternionAttitudeUncertainty* self);
size_t extract_MipData_Filter_QuaternionAttitudeUncertainty(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_QuaternionAttitudeUncertainty* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_filter_wgs84_gravity_mag  Filter Wgs84 Gravity Mag
/// Filter reported WGS84 gravity magnitude.
///
///@{

struct MipData_Filter_Wgs84GravityMag
{
    float                                             magnitude;
    uint16_t                                          valid_flags;
};
size_t insert_MipData_Filter_Wgs84GravityMag(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_Wgs84GravityMag* self);
size_t extract_MipData_Filter_Wgs84GravityMag(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_Wgs84GravityMag* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_filter_heading_update_state  Filter Heading Update State
/// Filter reported heading update state.
/// 
/// Heading updates can be applied from the sources listed below.  Note, some of these sources may be combined.
/// The heading value is always relative to true north.
///
///@{

enum MipData_Filter_HeadingUpdateState_Headingsource
{
    MIPDATA_FILTER_HEADINGUPDATESTATE_HEADINGSOURCE_NONE                 = 0,  ///<  
    MIPDATA_FILTER_HEADINGUPDATESTATE_HEADINGSOURCE_MAGNETOMETER         = 1,  ///<  
    MIPDATA_FILTER_HEADINGUPDATESTATE_HEADINGSOURCE_GNSS_VELOCITY_VECTOR = 2,  ///<  
    MIPDATA_FILTER_HEADINGUPDATESTATE_HEADINGSOURCE_EXTERNAL             = 4,  ///<  
    MIPDATA_FILTER_HEADINGUPDATESTATE_HEADINGSOURCE_DUAL_ANTENNA         = 8,  ///<  
};
size_t insert_MipData_Filter_HeadingUpdateState_Headingsource(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Filter_HeadingUpdateState_Headingsource self);
size_t extract_MipData_Filter_HeadingUpdateState_Headingsource(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Filter_HeadingUpdateState_Headingsource* self);

struct MipData_Filter_HeadingUpdateState
{
    float                                             heading;
    float                                             heading_1sigma;
    enum MipData_Filter_HeadingUpdateState_Headingsource source;
    uint16_t                                          valid_flags;
};
size_t insert_MipData_Filter_HeadingUpdateState(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_HeadingUpdateState* self);
size_t extract_MipData_Filter_HeadingUpdateState(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_HeadingUpdateState* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_filter_magnetic_model  Filter Magnetic Model
/// The World Magnetic Model is used for this data. Please refer to the device user manual for the current version of the model.
/// A valid GNSS location is required for the model to be valid.
///
///@{

struct MipData_Filter_MagneticModel
{
    float                                             intensity_north;
    float                                             intensity_east;
    float                                             intensity_down;
    float                                             inclination;
    float                                             declination;
    uint16_t                                          valid_flags;
};
size_t insert_MipData_Filter_MagneticModel(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_MagneticModel* self);
size_t extract_MipData_Filter_MagneticModel(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_MagneticModel* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_filter_accel_scale_factor  Filter Accel Scale Factor
/// Filter reported accelerometer scale factor expressed in the sensor frame.
///
///@{

struct MipData_Filter_AccelScaleFactor
{
    float                                             scale_factor[3];
    uint16_t                                          valid_flags;
};
size_t insert_MipData_Filter_AccelScaleFactor(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_AccelScaleFactor* self);
size_t extract_MipData_Filter_AccelScaleFactor(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_AccelScaleFactor* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_filter_accel_scale_factor_uncertainty  Filter Accel Scale Factor Uncertainty
/// Filter reported 1-sigma accelerometer scale factor uncertainty expressed in the sensor frame.
///
///@{

struct MipData_Filter_AccelScaleFactorUncertainty
{
    float                                             scale_factor_uncert[3];
    uint16_t                                          valid_flags;
};
size_t insert_MipData_Filter_AccelScaleFactorUncertainty(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_AccelScaleFactorUncertainty* self);
size_t extract_MipData_Filter_AccelScaleFactorUncertainty(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_AccelScaleFactorUncertainty* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_filter_gyro_scale_factor  Filter Gyro Scale Factor
/// Filter reported gyro scale factor expressed in the sensor frame.
///
///@{

struct MipData_Filter_GyroScaleFactor
{
    float                                             scale_factor[3];
    uint16_t                                          valid_flags;
};
size_t insert_MipData_Filter_GyroScaleFactor(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_GyroScaleFactor* self);
size_t extract_MipData_Filter_GyroScaleFactor(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_GyroScaleFactor* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_filter_gyro_scale_factor_uncertainty  Filter Gyro Scale Factor Uncertainty
/// Filter reported 1-sigma gyro scale factor uncertainty expressed in the sensor frame.
///
///@{

struct MipData_Filter_GyroScaleFactorUncertainty
{
    float                                             scale_factor_uncert[3];
    uint16_t                                          valid_flags;
};
size_t insert_MipData_Filter_GyroScaleFactorUncertainty(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_GyroScaleFactorUncertainty* self);
size_t extract_MipData_Filter_GyroScaleFactorUncertainty(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_GyroScaleFactorUncertainty* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_filter_mag_bias  Filter Mag Bias
/// Filter reported magnetometer bias expressed in the sensor frame.
///
///@{

struct MipData_Filter_MagBias
{
    float                                             bias[3];
    uint16_t                                          valid_flags;
};
size_t insert_MipData_Filter_MagBias(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_MagBias* self);
size_t extract_MipData_Filter_MagBias(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_MagBias* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_filter_mag_bias_uncertainty  Filter Mag Bias Uncertainty
/// Filter reported 1-sigma magnetometer bias uncertainty expressed in the sensor frame.
///
///@{

struct MipData_Filter_MagBiasUncertainty
{
    float                                             bias_uncert[3];
    uint16_t                                          valid_flags;
};
size_t insert_MipData_Filter_MagBiasUncertainty(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_MagBiasUncertainty* self);
size_t extract_MipData_Filter_MagBiasUncertainty(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_MagBiasUncertainty* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_filter_standard_atmosphere  Filter Standard Atmosphere
/// Filter reported standard atmosphere parameters.
/// 
/// The US 1976 Standard Atmosphere Model is used. A valid GNSS location is required for the model to be valid.
///
///@{

struct MipData_Filter_StandardAtmosphere
{
    float                                             geometric_altitude;
    float                                             geopotential_altitude;
    float                                             standard_temperature;
    float                                             standard_pressure;
    float                                             standard_density;
    uint16_t                                          valid_flags;
};
size_t insert_MipData_Filter_StandardAtmosphere(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_StandardAtmosphere* self);
size_t extract_MipData_Filter_StandardAtmosphere(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_StandardAtmosphere* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_filter_pressure_altitude  Filter Pressure Altitude
/// Filter reported pressure altitude.
/// 
/// The US 1976 Standard Atmosphere Model is used to calculate the pressure altitude in meters.
/// A valid pressure sensor reading is required for the pressure altitude to be valid.
/// The minimum pressure reading supported by the model is 0.0037 mBar, corresponding to an altitude of 84,852 meters.
///
///@{

struct MipData_Filter_PressureAltitude
{
    float                                             pressure_altitude;
    uint16_t                                          valid_flags;
};
size_t insert_MipData_Filter_PressureAltitude(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_PressureAltitude* self);
size_t extract_MipData_Filter_PressureAltitude(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_PressureAltitude* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_filter_density_altitude  Filter Density Altitude
///
///@{

struct MipData_Filter_DensityAltitude
{
    float                                             density_altitude;
    uint16_t                                          valid_flags;
};
size_t insert_MipData_Filter_DensityAltitude(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_DensityAltitude* self);
size_t extract_MipData_Filter_DensityAltitude(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_DensityAltitude* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_filter_antenna_offset_correction  Filter Antenna Offset Correction
/// Filter reported GNSS antenna offset in vehicle frame.
/// 
/// This offset added to any previously stored offset vector to compensate for errors in definition.
///
///@{

struct MipData_Filter_AntennaOffsetCorrection
{
    float                                             offset[3];
    uint16_t                                          valid_flags;
};
size_t insert_MipData_Filter_AntennaOffsetCorrection(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_AntennaOffsetCorrection* self);
size_t extract_MipData_Filter_AntennaOffsetCorrection(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_AntennaOffsetCorrection* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_filter_antenna_offset_correction_uncertainty  Filter Antenna Offset Correction Uncertainty
/// Filter reported 1-sigma GNSS antenna offset uncertainties in vehicle frame.
///
///@{

struct MipData_Filter_AntennaOffsetCorrectionUncertainty
{
    float                                             offset_uncert[3];
    uint16_t                                          valid_flags;
};
size_t insert_MipData_Filter_AntennaOffsetCorrectionUncertainty(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_AntennaOffsetCorrectionUncertainty* self);
size_t extract_MipData_Filter_AntennaOffsetCorrectionUncertainty(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_AntennaOffsetCorrectionUncertainty* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_filter_multi_antenna_offset_correction  Filter Multi Antenna Offset Correction
/// Filter reported GNSS antenna offset in vehicle frame.
/// 
/// This offset added to any previously stored offset vector to compensate for errors in definition.
///
///@{

struct MipData_Filter_MultiAntennaOffsetCorrection
{
    uint8_t                                           receiver_id;
    float                                             offset[3];
    uint16_t                                          valid_flags;
};
size_t insert_MipData_Filter_MultiAntennaOffsetCorrection(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_MultiAntennaOffsetCorrection* self);
size_t extract_MipData_Filter_MultiAntennaOffsetCorrection(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_MultiAntennaOffsetCorrection* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_filter_multi_antenna_offset_correction_uncertainty  Filter Multi Antenna Offset Correction Uncertainty
/// Filter reported 1-sigma GNSS antenna offset uncertainties in vehicle frame.
///
///@{

struct MipData_Filter_MultiAntennaOffsetCorrectionUncertainty
{
    uint8_t                                           receiver_id;
    float                                             offset_uncert[3];
    uint16_t                                          valid_flags;
};
size_t insert_MipData_Filter_MultiAntennaOffsetCorrectionUncertainty(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_MultiAntennaOffsetCorrectionUncertainty* self);
size_t extract_MipData_Filter_MultiAntennaOffsetCorrectionUncertainty(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_MultiAntennaOffsetCorrectionUncertainty* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_filter_magnetometer_offset  Filter Magnetometer Offset
/// Filter reported magnetometer hard iron offset in sensor frame.
/// 
/// This offset added to any previously stored hard iron offset vector to compensate for magnetometer in-run bias errors.
///
///@{

struct MipData_Filter_MagnetometerOffset
{
    float                                             hard_iron[3];
    uint16_t                                          valid_flags;
};
size_t insert_MipData_Filter_MagnetometerOffset(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_MagnetometerOffset* self);
size_t extract_MipData_Filter_MagnetometerOffset(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_MagnetometerOffset* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_filter_magnetometer_matrix  Filter Magnetometer Matrix
/// Filter reported magnetometer soft iron matrix in sensor frame.
/// 
/// This matrix is post multiplied to any previously stored soft iron matrix to compensate for magnetometer in-run errors.
///
///@{

struct MipData_Filter_MagnetometerMatrix
{
    float                                             soft_iron[9];
    uint16_t                                          valid_flags;
};
size_t insert_MipData_Filter_MagnetometerMatrix(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_MagnetometerMatrix* self);
size_t extract_MipData_Filter_MagnetometerMatrix(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_MagnetometerMatrix* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_filter_magnetometer_offset_uncertainty  Filter Magnetometer Offset Uncertainty
/// Filter reported 1-sigma magnetometer hard iron offset uncertainties in sensor frame.
///
///@{

struct MipData_Filter_MagnetometerOffsetUncertainty
{
    float                                             hard_iron_uncertainty[3];
    uint16_t                                          valid_flags;
};
size_t insert_MipData_Filter_MagnetometerOffsetUncertainty(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_MagnetometerOffsetUncertainty* self);
size_t extract_MipData_Filter_MagnetometerOffsetUncertainty(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_MagnetometerOffsetUncertainty* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_filter_magnetometer_matrix_uncertainty  Filter Magnetometer Matrix Uncertainty
/// Filter reported 1-sigma magnetometer soft iron matrix uncertainties in sensor frame.
///
///@{

struct MipData_Filter_MagnetometerMatrixUncertainty
{
    float                                             soft_iron_uncertainty[9];
    uint16_t                                          valid_flags;
};
size_t insert_MipData_Filter_MagnetometerMatrixUncertainty(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_MagnetometerMatrixUncertainty* self);
size_t extract_MipData_Filter_MagnetometerMatrixUncertainty(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_MagnetometerMatrixUncertainty* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_filter_magnetometer_covariance_matrix  Filter Magnetometer Covariance Matrix
///
///@{

struct MipData_Filter_MagnetometerCovarianceMatrix
{
    float                                             covariance[9];
    uint16_t                                          valid_flags;
};
size_t insert_MipData_Filter_MagnetometerCovarianceMatrix(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_MagnetometerCovarianceMatrix* self);
size_t extract_MipData_Filter_MagnetometerCovarianceMatrix(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_MagnetometerCovarianceMatrix* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_filter_magnetometer_residual_vector  Filter Magnetometer Residual Vector
/// Filter reported magnetometer measurement residuals in vehicle frame.
///
///@{

struct MipData_Filter_MagnetometerResidualVector
{
    float                                             residual[3];
    uint16_t                                          valid_flags;
};
size_t insert_MipData_Filter_MagnetometerResidualVector(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_MagnetometerResidualVector* self);
size_t extract_MipData_Filter_MagnetometerResidualVector(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_MagnetometerResidualVector* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_filter_clock_correction  Filter Clock Correction
/// Filter reported GNSS receiver clock error parameters.
///
///@{

struct MipData_Filter_ClockCorrection
{
    uint8_t                                           receiver_id;
    float                                             bias;
    float                                             bias_drift;
    uint16_t                                          valid_flags;
};
size_t insert_MipData_Filter_ClockCorrection(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_ClockCorrection* self);
size_t extract_MipData_Filter_ClockCorrection(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_ClockCorrection* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_filter_clock_correction_uncertainty  Filter Clock Correction Uncertainty
/// Filter reported 1-sigma GNSS receiver clock error parameters.
///
///@{

struct MipData_Filter_ClockCorrectionUncertainty
{
    uint8_t                                           receiver_id;
    float                                             bias_uncertainty;
    float                                             bias_drift_uncertainty;
    uint16_t                                          valid_flags;
};
size_t insert_MipData_Filter_ClockCorrectionUncertainty(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_ClockCorrectionUncertainty* self);
size_t extract_MipData_Filter_ClockCorrectionUncertainty(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_ClockCorrectionUncertainty* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_filter_gnss_pos_aid_status  Filter Gnss Pos Aid Status
/// Filter reported GNSS position aiding status
///
///@{

struct MipData_Filter_GnssPosAidStatus
{
    uint8_t                                           receiver_id;
    float                                             time_of_week;
    enum MipGnssAidStatusFlags                        status;
    uint8_t                                           reserved[8];
};
size_t insert_MipData_Filter_GnssPosAidStatus(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_GnssPosAidStatus* self);
size_t extract_MipData_Filter_GnssPosAidStatus(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_GnssPosAidStatus* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_filter_gnss_att_aid_status  Filter Gnss Att Aid Status
/// Filter reported dual antenna GNSS attitude aiding status
///
///@{

struct MipData_Filter_GnssAttAidStatus
{
    float                                             time_of_week;
    enum MipGnssAidStatusFlags                        status;
    uint8_t                                           reserved[8];
};
size_t insert_MipData_Filter_GnssAttAidStatus(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_GnssAttAidStatus* self);
size_t extract_MipData_Filter_GnssAttAidStatus(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_GnssAttAidStatus* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_filter_head_aid_status  Filter Head Aid Status
/// Filter reported GNSS heading aiding status
///
///@{

enum MipData_Filter_HeadAidStatus_Headingaidtype
{
    MIPDATA_FILTER_HEADAIDSTATUS_HEADINGAIDTYPE_DUAL_ANTENNA     = 1,  ///<  
    MIPDATA_FILTER_HEADAIDSTATUS_HEADINGAIDTYPE_EXTERNAL_MESSAGE = 2,  ///<  
};
size_t insert_MipData_Filter_HeadAidStatus_Headingaidtype(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Filter_HeadAidStatus_Headingaidtype self);
size_t extract_MipData_Filter_HeadAidStatus_Headingaidtype(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Filter_HeadAidStatus_Headingaidtype* self);

struct MipData_Filter_HeadAidStatus
{
    float                                             time_of_week;
    enum MipData_Filter_HeadAidStatus_Headingaidtype  type;
    float                                             reserved[2];
};
size_t insert_MipData_Filter_HeadAidStatus(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_HeadAidStatus* self);
size_t extract_MipData_Filter_HeadAidStatus(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_HeadAidStatus* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_filter_rel_pos_ned  Filter Rel Pos Ned
/// Filter reported relative position, with respect to configured reference position
///
///@{

struct MipData_Filter_RelPosNed
{
    double                                            relative_position[3];
    uint16_t                                          valid_flags;
};
size_t insert_MipData_Filter_RelPosNed(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_RelPosNed* self);
size_t extract_MipData_Filter_RelPosNed(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_RelPosNed* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_filter_ecef_pos  Filter Ecef Pos
/// Filter reported ECEF position
///
///@{

struct MipData_Filter_EcefPos
{
    double                                            position_ecef[3];
    uint16_t                                          valid_flags;
};
size_t insert_MipData_Filter_EcefPos(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_EcefPos* self);
size_t extract_MipData_Filter_EcefPos(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_EcefPos* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_filter_ecef_vel  Filter Ecef Vel
/// Filter reported ECEF velocity
///
///@{

struct MipData_Filter_EcefVel
{
    float                                             velocity_ecef[3];
    uint16_t                                          valid_flags;
};
size_t insert_MipData_Filter_EcefVel(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_EcefVel* self);
size_t extract_MipData_Filter_EcefVel(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_EcefVel* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_filter_ecef_pos_uncertainty  Filter Ecef Pos Uncertainty
/// Filter reported 1-sigma position uncertainty in the ECEF frame.
///
///@{

struct MipData_Filter_EcefPosUncertainty
{
    float                                             pos_uncertainty[3];
    uint16_t                                          valid_flags;
};
size_t insert_MipData_Filter_EcefPosUncertainty(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_EcefPosUncertainty* self);
size_t extract_MipData_Filter_EcefPosUncertainty(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_EcefPosUncertainty* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_filter_ecef_vel_uncertainty  Filter Ecef Vel Uncertainty
/// Filter reported 1-sigma velocity uncertainties in the ECEF frame.
///
///@{

struct MipData_Filter_EcefVelUncertainty
{
    float                                             vel_uncertainty[3];
    uint16_t                                          valid_flags;
};
size_t insert_MipData_Filter_EcefVelUncertainty(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_EcefVelUncertainty* self);
size_t extract_MipData_Filter_EcefVelUncertainty(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_EcefVelUncertainty* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_filter_aiding_measurement_summary  Filter Aiding Measurement Summary
/// Filter reported aiding measurement summary. This message contains a summary of the specified aiding measurement over the previous measurement interval ending at the specified time.
///
///@{

struct MipData_Filter_AidingMeasurementSummary
{
    float                                             time_of_week;
    uint8_t                                           source;
    enum MipFilterAidingMeasurementType               type;
    enum MipFilterMeasurementIndicator                indicator;
};
size_t insert_MipData_Filter_AidingMeasurementSummary(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_AidingMeasurementSummary* self);
size_t extract_MipData_Filter_AidingMeasurementSummary(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_AidingMeasurementSummary* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_filter_odometer_scale_factor_error  Filter Odometer Scale Factor Error
/// Filter reported odometer scale factor error. The total scale factor estimate is the user indicated scale factor, plus the user indicated scale factor times the scale factor error.
///
///@{

struct MipData_Filter_OdometerScaleFactorError
{
    float                                             scale_factor_error;
    uint16_t                                          valid_flags;
};
size_t insert_MipData_Filter_OdometerScaleFactorError(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_OdometerScaleFactorError* self);
size_t extract_MipData_Filter_OdometerScaleFactorError(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_OdometerScaleFactorError* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_filter_odometer_scale_factor_error_uncertainty  Filter Odometer Scale Factor Error Uncertainty
/// Filter reported odometer scale factor error uncertainty.
///
///@{

struct MipData_Filter_OdometerScaleFactorErrorUncertainty
{
    float                                             scale_factor_error_uncertainty;
    uint16_t                                          valid_flags;
};
size_t insert_MipData_Filter_OdometerScaleFactorErrorUncertainty(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_OdometerScaleFactorErrorUncertainty* self);
size_t extract_MipData_Filter_OdometerScaleFactorErrorUncertainty(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_OdometerScaleFactorErrorUncertainty* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_filter_gnss_dual_antenna_status  Filter Gnss Dual Antenna Status
/// Summary information for status of GNSS dual antenna heading estimate.
///
///@{

enum MipData_Filter_GnssDualAntennaStatus_Fixtype
{
    MIPDATA_FILTER_GNSSDUALANTENNASTATUS_FIXTYPE_FIX_NONE     = 0,  ///<  
    MIPDATA_FILTER_GNSSDUALANTENNASTATUS_FIXTYPE_FIX_DA_FLOAT = 1,  ///<  
    MIPDATA_FILTER_GNSSDUALANTENNASTATUS_FIXTYPE_FIX_DA_FIXED = 2,  ///<  
};
size_t insert_MipData_Filter_GnssDualAntennaStatus_Fixtype(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Filter_GnssDualAntennaStatus_Fixtype self);
size_t extract_MipData_Filter_GnssDualAntennaStatus_Fixtype(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Filter_GnssDualAntennaStatus_Fixtype* self);

enum MipData_Filter_GnssDualAntennaStatus_Dualantennastatusflags
{
    MIPDATA_FILTER_GNSSDUALANTENNASTATUS_DUALANTENNASTATUSFLAGS_RCV_1_DATA_VALID      = 0x01,
    MIPDATA_FILTER_GNSSDUALANTENNASTATUS_DUALANTENNASTATUSFLAGS_RCV_2_DATA_VALID      = 0x02,
    MIPDATA_FILTER_GNSSDUALANTENNASTATUS_DUALANTENNASTATUSFLAGS_ANTENNA_OFFSETS_VALID = 0x04,
};
size_t insert_MipData_Filter_GnssDualAntennaStatus_Dualantennastatusflags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Filter_GnssDualAntennaStatus_Dualantennastatusflags self);
size_t extract_MipData_Filter_GnssDualAntennaStatus_Dualantennastatusflags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Filter_GnssDualAntennaStatus_Dualantennastatusflags* self);

struct MipData_Filter_GnssDualAntennaStatus
{
    float                                             time_of_week;
    float                                             heading;
    float                                             heading_unc;
    enum MipData_Filter_GnssDualAntennaStatus_Fixtype fix_type;
    enum MipData_Filter_GnssDualAntennaStatus_Dualantennastatusflags status_flags;
    uint16_t                                          valid_flags;
};
size_t insert_MipData_Filter_GnssDualAntennaStatus(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_GnssDualAntennaStatus* self);
size_t extract_MipData_Filter_GnssDualAntennaStatus(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_GnssDualAntennaStatus* self);

///@}
///

///@}
///@}
///
////////////////////////////////////////////////////////////////////////////////

#ifdef __cplusplus
} // extern "C"


template<>
struct MipFieldInfo<MipData_Filter_LlhPos>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_LLH_POS;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Filter_LlhPos& self)
    {
        return insert_MipData_Filter_LlhPos(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Filter_LlhPos& self)
    {
        return extract_MipData_Filter_LlhPos(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Filter_NedVelocity>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_NED_VEL;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Filter_NedVelocity& self)
    {
        return insert_MipData_Filter_NedVelocity(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Filter_NedVelocity& self)
    {
        return extract_MipData_Filter_NedVelocity(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Filter_AttitudeQuaternion>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_ATT_QUATERNION;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Filter_AttitudeQuaternion& self)
    {
        return insert_MipData_Filter_AttitudeQuaternion(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Filter_AttitudeQuaternion& self)
    {
        return extract_MipData_Filter_AttitudeQuaternion(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Filter_AttitudeDcm>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_ATT_MATRIX;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Filter_AttitudeDcm& self)
    {
        return insert_MipData_Filter_AttitudeDcm(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Filter_AttitudeDcm& self)
    {
        return extract_MipData_Filter_AttitudeDcm(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Filter_EulerAngles>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_ATT_EULER_ANGLES;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Filter_EulerAngles& self)
    {
        return insert_MipData_Filter_EulerAngles(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Filter_EulerAngles& self)
    {
        return extract_MipData_Filter_EulerAngles(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Filter_GyroBias>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_GYRO_BIAS;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Filter_GyroBias& self)
    {
        return insert_MipData_Filter_GyroBias(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Filter_GyroBias& self)
    {
        return extract_MipData_Filter_GyroBias(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Filter_AccelBias>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_ACCEL_BIAS;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Filter_AccelBias& self)
    {
        return insert_MipData_Filter_AccelBias(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Filter_AccelBias& self)
    {
        return extract_MipData_Filter_AccelBias(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Filter_LlhPosUncertainty>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_POS_UNCERTAINTY;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Filter_LlhPosUncertainty& self)
    {
        return insert_MipData_Filter_LlhPosUncertainty(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Filter_LlhPosUncertainty& self)
    {
        return extract_MipData_Filter_LlhPosUncertainty(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Filter_NedVelUncertainty>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_VEL_UNCERTAINTY;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Filter_NedVelUncertainty& self)
    {
        return insert_MipData_Filter_NedVelUncertainty(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Filter_NedVelUncertainty& self)
    {
        return extract_MipData_Filter_NedVelUncertainty(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Filter_EulerAnglesUncertainty>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_ATT_UNCERTAINTY_EULER;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Filter_EulerAnglesUncertainty& self)
    {
        return insert_MipData_Filter_EulerAnglesUncertainty(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Filter_EulerAnglesUncertainty& self)
    {
        return extract_MipData_Filter_EulerAnglesUncertainty(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Filter_GyroBiasUncertainty>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_GYRO_BIAS_UNCERTAINTY;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Filter_GyroBiasUncertainty& self)
    {
        return insert_MipData_Filter_GyroBiasUncertainty(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Filter_GyroBiasUncertainty& self)
    {
        return extract_MipData_Filter_GyroBiasUncertainty(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Filter_AccelBiasUncertainty>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_ACCEL_BIAS_UNCERTAINTY;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Filter_AccelBiasUncertainty& self)
    {
        return insert_MipData_Filter_AccelBiasUncertainty(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Filter_AccelBiasUncertainty& self)
    {
        return extract_MipData_Filter_AccelBiasUncertainty(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Filter_Timestamp>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_FILTER_TIMESTAMP;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Filter_Timestamp& self)
    {
        return insert_MipData_Filter_Timestamp(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Filter_Timestamp& self)
    {
        return extract_MipData_Filter_Timestamp(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Filter_Status>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_FILTER_STATUS;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Filter_Status& self)
    {
        return insert_MipData_Filter_Status(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Filter_Status& self)
    {
        return extract_MipData_Filter_Status(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Filter_LinearAccel>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_LINEAR_ACCELERATION;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Filter_LinearAccel& self)
    {
        return insert_MipData_Filter_LinearAccel(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Filter_LinearAccel& self)
    {
        return extract_MipData_Filter_LinearAccel(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Filter_GravityVector>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_GRAVITY_VECTOR;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Filter_GravityVector& self)
    {
        return insert_MipData_Filter_GravityVector(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Filter_GravityVector& self)
    {
        return extract_MipData_Filter_GravityVector(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Filter_CompAccel>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_COMPENSATED_ACCELERATION;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Filter_CompAccel& self)
    {
        return insert_MipData_Filter_CompAccel(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Filter_CompAccel& self)
    {
        return extract_MipData_Filter_CompAccel(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Filter_CompAngularRate>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_COMPENSATED_ANGULAR_RATE;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Filter_CompAngularRate& self)
    {
        return insert_MipData_Filter_CompAngularRate(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Filter_CompAngularRate& self)
    {
        return extract_MipData_Filter_CompAngularRate(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Filter_QuaternionAttitudeUncertainty>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_ATT_UNCERTAINTY_QUATERNION;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Filter_QuaternionAttitudeUncertainty& self)
    {
        return insert_MipData_Filter_QuaternionAttitudeUncertainty(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Filter_QuaternionAttitudeUncertainty& self)
    {
        return extract_MipData_Filter_QuaternionAttitudeUncertainty(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Filter_Wgs84GravityMag>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_WGS84_GRAVITY;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Filter_Wgs84GravityMag& self)
    {
        return insert_MipData_Filter_Wgs84GravityMag(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Filter_Wgs84GravityMag& self)
    {
        return extract_MipData_Filter_Wgs84GravityMag(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Filter_HeadingUpdateState>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_HEADING_UPDATE_STATE;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Filter_HeadingUpdateState& self)
    {
        return insert_MipData_Filter_HeadingUpdateState(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Filter_HeadingUpdateState& self)
    {
        return extract_MipData_Filter_HeadingUpdateState(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Filter_MagneticModel>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_MAGNETIC_MODEL;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Filter_MagneticModel& self)
    {
        return insert_MipData_Filter_MagneticModel(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Filter_MagneticModel& self)
    {
        return extract_MipData_Filter_MagneticModel(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Filter_AccelScaleFactor>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_ACCEL_SCALE_FACTOR;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Filter_AccelScaleFactor& self)
    {
        return insert_MipData_Filter_AccelScaleFactor(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Filter_AccelScaleFactor& self)
    {
        return extract_MipData_Filter_AccelScaleFactor(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Filter_AccelScaleFactorUncertainty>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_ACCEL_SCALE_FACTOR_UNCERTAINTY;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Filter_AccelScaleFactorUncertainty& self)
    {
        return insert_MipData_Filter_AccelScaleFactorUncertainty(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Filter_AccelScaleFactorUncertainty& self)
    {
        return extract_MipData_Filter_AccelScaleFactorUncertainty(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Filter_GyroScaleFactor>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_GYRO_SCALE_FACTOR;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Filter_GyroScaleFactor& self)
    {
        return insert_MipData_Filter_GyroScaleFactor(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Filter_GyroScaleFactor& self)
    {
        return extract_MipData_Filter_GyroScaleFactor(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Filter_GyroScaleFactorUncertainty>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_GYRO_SCALE_FACTOR_UNCERTAINTY;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Filter_GyroScaleFactorUncertainty& self)
    {
        return insert_MipData_Filter_GyroScaleFactorUncertainty(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Filter_GyroScaleFactorUncertainty& self)
    {
        return extract_MipData_Filter_GyroScaleFactorUncertainty(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Filter_MagBias>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_MAG_BIAS;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Filter_MagBias& self)
    {
        return insert_MipData_Filter_MagBias(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Filter_MagBias& self)
    {
        return extract_MipData_Filter_MagBias(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Filter_MagBiasUncertainty>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_MAG_BIAS_UNCERTAINTY;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Filter_MagBiasUncertainty& self)
    {
        return insert_MipData_Filter_MagBiasUncertainty(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Filter_MagBiasUncertainty& self)
    {
        return extract_MipData_Filter_MagBiasUncertainty(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Filter_StandardAtmosphere>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_STANDARD_ATMOSPHERE_DATA;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Filter_StandardAtmosphere& self)
    {
        return insert_MipData_Filter_StandardAtmosphere(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Filter_StandardAtmosphere& self)
    {
        return extract_MipData_Filter_StandardAtmosphere(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Filter_PressureAltitude>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_PRESSURE_ALTITUDE_DATA;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Filter_PressureAltitude& self)
    {
        return insert_MipData_Filter_PressureAltitude(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Filter_PressureAltitude& self)
    {
        return extract_MipData_Filter_PressureAltitude(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Filter_DensityAltitude>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_DENSITY_ALTITUDE_DATA;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Filter_DensityAltitude& self)
    {
        return insert_MipData_Filter_DensityAltitude(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Filter_DensityAltitude& self)
    {
        return extract_MipData_Filter_DensityAltitude(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Filter_AntennaOffsetCorrection>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_ANTENNA_OFFSET_CORRECTION;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Filter_AntennaOffsetCorrection& self)
    {
        return insert_MipData_Filter_AntennaOffsetCorrection(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Filter_AntennaOffsetCorrection& self)
    {
        return extract_MipData_Filter_AntennaOffsetCorrection(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Filter_AntennaOffsetCorrectionUncertainty>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_ANTENNA_OFFSET_CORRECTION_UNCERTAINTY;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Filter_AntennaOffsetCorrectionUncertainty& self)
    {
        return insert_MipData_Filter_AntennaOffsetCorrectionUncertainty(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Filter_AntennaOffsetCorrectionUncertainty& self)
    {
        return extract_MipData_Filter_AntennaOffsetCorrectionUncertainty(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Filter_MultiAntennaOffsetCorrection>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_MULTI_ANTENNA_OFFSET_CORRECTION;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Filter_MultiAntennaOffsetCorrection& self)
    {
        return insert_MipData_Filter_MultiAntennaOffsetCorrection(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Filter_MultiAntennaOffsetCorrection& self)
    {
        return extract_MipData_Filter_MultiAntennaOffsetCorrection(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Filter_MultiAntennaOffsetCorrectionUncertainty>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_MULTI_ANTENNA_OFFSET_CORRECTION_UNCERTAINTY;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Filter_MultiAntennaOffsetCorrectionUncertainty& self)
    {
        return insert_MipData_Filter_MultiAntennaOffsetCorrectionUncertainty(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Filter_MultiAntennaOffsetCorrectionUncertainty& self)
    {
        return extract_MipData_Filter_MultiAntennaOffsetCorrectionUncertainty(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Filter_MagnetometerOffset>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_MAG_COMPENSATION_OFFSET;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Filter_MagnetometerOffset& self)
    {
        return insert_MipData_Filter_MagnetometerOffset(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Filter_MagnetometerOffset& self)
    {
        return extract_MipData_Filter_MagnetometerOffset(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Filter_MagnetometerMatrix>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_MAG_COMPENSATION_MATRIX;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Filter_MagnetometerMatrix& self)
    {
        return insert_MipData_Filter_MagnetometerMatrix(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Filter_MagnetometerMatrix& self)
    {
        return extract_MipData_Filter_MagnetometerMatrix(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Filter_MagnetometerOffsetUncertainty>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_MAG_COMPENSATION_OFFSET_UNCERTAINTY;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Filter_MagnetometerOffsetUncertainty& self)
    {
        return insert_MipData_Filter_MagnetometerOffsetUncertainty(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Filter_MagnetometerOffsetUncertainty& self)
    {
        return extract_MipData_Filter_MagnetometerOffsetUncertainty(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Filter_MagnetometerMatrixUncertainty>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_MAG_COMPENSATION_MATRIX_UNCERTAINTY;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Filter_MagnetometerMatrixUncertainty& self)
    {
        return insert_MipData_Filter_MagnetometerMatrixUncertainty(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Filter_MagnetometerMatrixUncertainty& self)
    {
        return extract_MipData_Filter_MagnetometerMatrixUncertainty(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Filter_MagnetometerCovarianceMatrix>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_MAG_COVARIANCE;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Filter_MagnetometerCovarianceMatrix& self)
    {
        return insert_MipData_Filter_MagnetometerCovarianceMatrix(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Filter_MagnetometerCovarianceMatrix& self)
    {
        return extract_MipData_Filter_MagnetometerCovarianceMatrix(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Filter_MagnetometerResidualVector>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_MAG_RESIDUAL;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Filter_MagnetometerResidualVector& self)
    {
        return insert_MipData_Filter_MagnetometerResidualVector(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Filter_MagnetometerResidualVector& self)
    {
        return extract_MipData_Filter_MagnetometerResidualVector(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Filter_ClockCorrection>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_CLOCK_CORRECTION;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Filter_ClockCorrection& self)
    {
        return insert_MipData_Filter_ClockCorrection(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Filter_ClockCorrection& self)
    {
        return extract_MipData_Filter_ClockCorrection(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Filter_ClockCorrectionUncertainty>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_CLOCK_CORRECTION_UNCERTAINTY;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Filter_ClockCorrectionUncertainty& self)
    {
        return insert_MipData_Filter_ClockCorrectionUncertainty(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Filter_ClockCorrectionUncertainty& self)
    {
        return extract_MipData_Filter_ClockCorrectionUncertainty(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Filter_GnssPosAidStatus>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_GNSS_POS_AID_STATUS;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Filter_GnssPosAidStatus& self)
    {
        return insert_MipData_Filter_GnssPosAidStatus(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Filter_GnssPosAidStatus& self)
    {
        return extract_MipData_Filter_GnssPosAidStatus(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Filter_GnssAttAidStatus>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_GNSS_ATT_AID_STATUS;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Filter_GnssAttAidStatus& self)
    {
        return insert_MipData_Filter_GnssAttAidStatus(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Filter_GnssAttAidStatus& self)
    {
        return extract_MipData_Filter_GnssAttAidStatus(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Filter_HeadAidStatus>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_HEAD_AID_STATUS;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Filter_HeadAidStatus& self)
    {
        return insert_MipData_Filter_HeadAidStatus(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Filter_HeadAidStatus& self)
    {
        return extract_MipData_Filter_HeadAidStatus(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Filter_RelPosNed>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_REL_POS_NED;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Filter_RelPosNed& self)
    {
        return insert_MipData_Filter_RelPosNed(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Filter_RelPosNed& self)
    {
        return extract_MipData_Filter_RelPosNed(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Filter_EcefPos>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_ECEF_POS;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Filter_EcefPos& self)
    {
        return insert_MipData_Filter_EcefPos(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Filter_EcefPos& self)
    {
        return extract_MipData_Filter_EcefPos(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Filter_EcefVel>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_ECEF_VEL;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Filter_EcefVel& self)
    {
        return insert_MipData_Filter_EcefVel(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Filter_EcefVel& self)
    {
        return extract_MipData_Filter_EcefVel(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Filter_EcefPosUncertainty>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_ECEF_POS_UNCERTAINTY;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Filter_EcefPosUncertainty& self)
    {
        return insert_MipData_Filter_EcefPosUncertainty(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Filter_EcefPosUncertainty& self)
    {
        return extract_MipData_Filter_EcefPosUncertainty(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Filter_EcefVelUncertainty>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_ECEF_VEL_UNCERTAINTY;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Filter_EcefVelUncertainty& self)
    {
        return insert_MipData_Filter_EcefVelUncertainty(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Filter_EcefVelUncertainty& self)
    {
        return extract_MipData_Filter_EcefVelUncertainty(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Filter_AidingMeasurementSummary>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_AID_MEAS_SUMMARY;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Filter_AidingMeasurementSummary& self)
    {
        return insert_MipData_Filter_AidingMeasurementSummary(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Filter_AidingMeasurementSummary& self)
    {
        return extract_MipData_Filter_AidingMeasurementSummary(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Filter_OdometerScaleFactorError>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_ODOMETER_SCALE_FACTOR_ERROR;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Filter_OdometerScaleFactorError& self)
    {
        return insert_MipData_Filter_OdometerScaleFactorError(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Filter_OdometerScaleFactorError& self)
    {
        return extract_MipData_Filter_OdometerScaleFactorError(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Filter_OdometerScaleFactorErrorUncertainty>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_ODOMETER_SCALE_FACTOR_ERROR_UNCERTAINTY;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Filter_OdometerScaleFactorErrorUncertainty& self)
    {
        return insert_MipData_Filter_OdometerScaleFactorErrorUncertainty(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Filter_OdometerScaleFactorErrorUncertainty& self)
    {
        return extract_MipData_Filter_OdometerScaleFactorErrorUncertainty(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipData_Filter_GnssDualAntennaStatus>
{
    static const uint8_t descriptorSet = MIP_FILTER_DATA_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_DATA_DESC_FILTER_GNSS_DUAL_ANTENNA_STATUS;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipData_Filter_GnssDualAntennaStatus& self)
    {
        return insert_MipData_Filter_GnssDualAntennaStatus(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipData_Filter_GnssDualAntennaStatus& self)
    {
        return extract_MipData_Filter_GnssDualAntennaStatus(buffer, bufferSize, offset, &self);
    }
};



} // namespace mscl
#endif // __cplusplus
