#pragma once

#include "descriptors.h"
#include "../mip_result.h"

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

namespace mscl {
class MipSerializer;

namespace C {
struct mip_interface;
} // namespace C

namespace data_filter {

////////////////////////////////////////////////////////////////////////////////
///@addtogroup MipData
///@{
///@defgroup filter_data_cpp  FILTERData
///
///@{

////////////////////////////////////////////////////////////////////////////////
// Descriptors
////////////////////////////////////////////////////////////////////////////////

enum 
{
    DESCRIPTOR_SET                                   = 0x82,
    
    DATA_POS_LLH                                     = 0x01,
    DATA_VEL_NED                                     = 0x02,
    DATA_ATT_QUATERNION                              = 0x03,
    DATA_ATT_MATRIX                                  = 0x04,
    DATA_ATT_EULER_ANGLES                            = 0x05,
    DATA_GYRO_BIAS                                   = 0x06,
    DATA_ACCEL_BIAS                                  = 0x07,
    DATA_POS_UNCERTAINTY                             = 0x08,
    DATA_VEL_UNCERTAINTY                             = 0x09,
    DATA_ATT_UNCERTAINTY_EULER                       = 0x0A,
    DATA_GYRO_BIAS_UNCERTAINTY                       = 0x0B,
    DATA_ACCEL_BIAS_UNCERTAINTY                      = 0x0C,
    DATA_LINEAR_ACCELERATION                         = 0x0D,
    DATA_COMPENSATED_ANGULAR_RATE                    = 0x0E,
    DATA_WGS84_GRAVITY                               = 0x0F,
    DATA_FILTER_STATUS                               = 0x10,
    DATA_FILTER_TIMESTAMP                            = 0x11,
    DATA_ATT_UNCERTAINTY_QUATERNION                  = 0x12,
    DATA_GRAVITY_VECTOR                              = 0x13,
    DATA_HEADING_UPDATE_STATE                        = 0x14,
    DATA_MAGNETIC_MODEL                              = 0x15,
    DATA_GYRO_SCALE_FACTOR                           = 0x16,
    DATA_ACCEL_SCALE_FACTOR                          = 0x17,
    DATA_GYRO_SCALE_FACTOR_UNCERTAINTY               = 0x18,
    DATA_ACCEL_SCALE_FACTOR_UNCERTAINTY              = 0x19,
    DATA_MAG_BIAS                                    = 0x1A,
    DATA_MAG_BIAS_UNCERTAINTY                        = 0x1B,
    DATA_COMPENSATED_ACCELERATION                    = 0x1C,
    DATA_STANDARD_ATMOSPHERE_DATA                    = 0x20,
    DATA_PRESSURE_ALTITUDE_DATA                      = 0x21,
    DATA_DENSITY_ALTITUDE_DATA                       = 0x22,
    DATA_MAG_SCALE_FACTOR                            = 0x23,
    DATA_MAG_SCALE_FACTOR_UNCERTAINTY                = 0x24,
    DATA_MAG_COMPENSATION_OFFSET                     = 0x25,
    DATA_MAG_COMPENSATION_MATRIX                     = 0x26,
    DATA_COMPENSATED_MAGNETOMETER                    = 0x27,
    DATA_MAG_COMPENSATION_OFFSET_UNCERTAINTY         = 0x28,
    DATA_MAG_COMPENSATION_MATRIX_UNCERTAINTY         = 0x29,
    DATA_MAG_COVARIANCE                              = 0x2A,
    DATA_GRAVITY_COVARIANCE                          = 0x2B,
    DATA_MAG_RESIDUAL                                = 0x2C,
    DATA_MAG_FILTERED_RESIDUAL                       = 0x2D,
    DATA_ANTENNA_OFFSET_CORRECTION                   = 0x30,
    DATA_ANTENNA_OFFSET_CORRECTION_UNCERTAINTY       = 0x31,
    DATA_CLOCK_CORRECTION                            = 0x32,
    DATA_CLOCK_CORRECTION_UNCERTAINTY                = 0x33,
    DATA_MULTI_ANTENNA_OFFSET_CORRECTION             = 0x34,
    DATA_MULTI_ANTENNA_OFFSET_CORRECTION_UNCERTAINTY = 0x35,
    DATA_ECEF_POS_UNCERTAINTY                        = 0x36,
    DATA_ECEF_VEL_UNCERTAINTY                        = 0x37,
    DATA_ECEF_POS                                    = 0x40,
    DATA_ECEF_VEL                                    = 0x41,
    DATA_REL_POS_NED                                 = 0x42,
    DATA_GNSS_POS_AID_STATUS                         = 0x43,
    DATA_GNSS_ATT_AID_STATUS                         = 0x44,
    DATA_HEAD_AID_STATUS                             = 0x45,
    DATA_AID_MEAS_SUMMARY                            = 0x46,
    DATA_ODOMETER_SCALE_FACTOR_ERROR                 = 0x47,
    DATA_ODOMETER_SCALE_FACTOR_ERROR_UNCERTAINTY     = 0x48,
    DATA_GNSS_DUAL_ANTENNA_STATUS                    = 0x49,
    
};

////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////

enum class FilterMode : uint16_t
{
    GX5_STARTUP            = 0,  ///<  
    GX5_INIT               = 1,  ///<  
    GX5_RUN_SOLUTION_VALID = 2,  ///<  
    GX5_RUN_SOLUTION_ERROR = 3,  ///<  
    GQ7_INIT               = 1,  ///<  
    GQ7_VERT_GYRO          = 2,  ///<  
    GQ7_AHRS               = 3,  ///<  
    GQ7_FULL_NAV           = 4,  ///<  
};

enum class FilterDynamicsMode : uint16_t
{
    GX5_PORTABLE   = 1,  ///<  
    GX5_AUTOMOTIVE = 2,  ///<  
    GX5_AIRBORNE   = 3,  ///<  
    GQ7_DEFAULT    = 1,  ///<  
};

enum class FilterStatusFlags : uint16_t
{
    GX5_INIT_NO_ATTITUDE                           = 0x1000,
    GX5_INIT_NO_POSITION_VELOCITY                  = 0x2000,
    GX5_RUN_IMU_UNAVAILABLE                        = 0x01,
    GX5_RUN_GPS_UNAVAILABLE                        = 0x02,
    GX5_RUN_MATRIX_SINGULARITY                     = 0x08,
    GX5_RUN_POSITION_COVARIANCE_WARNING            = 0x10,
    GX5_RUN_VELOCITY_COVARIANCE_WARNING            = 0x20,
    GX5_RUN_ATTITUDE_COVARIANCE_WARNING            = 0x40,
    GX5_RUN_NAN_IN_SOLUTION_WARNING                = 0x80,
    GX5_RUN_GYRO_BIAS_EST_HIGH_WARNING             = 0x100,
    GX5_RUN_ACCEL_BIAS_EST_HIGH_WARNING            = 0x200,
    GX5_RUN_GYRO_SCALE_FACTOR_EST_HIGH_WARNING     = 0x400,
    GX5_RUN_ACCEL_SCALE_FACTOR_EST_HIGH_WARNING    = 0x800,
    GX5_RUN_MAG_BIAS_EST_HIGH_WARNING              = 0x1000,
    GX5_RUN_ANT_OFFSET_CORRECTION_EST_HIGH_WARNING = 0x2000,
    GX5_RUN_MAG_HARD_IRON_EST_HIGH_WARNING         = 0x4000,
    GX5_RUN_MAG_SOFT_IRON_EST_HIGH_WARNING         = 0x8000,
    GQ7_FILTER_CONDITION                           = 0x03,
    GQ7_ROLL_PITCH_WARNING                         = 0x04,
    GQ7_HEADING_WARNING                            = 0x08,
    GQ7_POSITION_WARNING                           = 0x10,
    GQ7_VELOCITY_WARNING                           = 0x20,
    GQ7_IMU_BIAS_WARNING                           = 0x40,
    GQ7_GNSS_CLK_WARNING                           = 0x80,
    GQ7_ANTENNA_LEVER_ARM_WARNING                  = 0x100,
    GQ7_MOUNTING_TRANSFORM_WARNING                 = 0x200,
    GQ7_TIME_SYNC_WARNING                          = 0x400,
    GQ7_SOLUTION_ERROR                             = 0xF000,
};

enum class FilterAidingMeasurementType : uint8_t
{
    GNSS         = 1,  ///<  
    DUAL_ANTENNA = 2,  ///<  
    HEADING      = 3,  ///<  
    PRESSURE     = 4,  ///<  
    MAGNETOMETER = 5,  ///<  
    SPEED        = 6,  ///<  
};

enum class FilterMeasurementIndicator : uint8_t
{
    ENABLED               = 0x01,
    USED                  = 0x02,
    RESIDUAL_HIGH_WARNING = 0x04,
    SAMPLE_TIME_WARNING   = 0x08,
    CONFIGURATION_ERROR   = 0x10,
    MAX_NUM_MEAS_EXCEEDED = 0x20,
};

enum class GnssAidStatusFlags : uint16_t
{
    TIGHT_COUPLING = 0x01,
    DIFFERENTIAL   = 0x02,
    INTEGER_FIX    = 0x04,
    GPS_L1         = 0x08,
    GPS_L2         = 0x10,
    GPS_L5         = 0x20,
    GLO_L1         = 0x40,
    GLO_L2         = 0x80,
    GAL_E1         = 0x100,
    GAL_E5         = 0x200,
    GAL_E6         = 0x400,
    BEI_B1         = 0x800,
    BEI_B2         = 0x1000,
    BEI_B3         = 0x2000,
    NO_FIX         = 0x4000,
    CONFIG_ERROR   = 0x8000,
};


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_position_llh  LLH Position
/// Filter reported position in the WGS84 geodetic frame.
///
///@{

struct PositionLlh
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::data_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::data_filter::DATA_POS_LLH;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    double latitude;
    double longitude;
    double ellipsoid_height;
    uint16_t valid_flags;
    
};
void insert(MipSerializer& serializer, const PositionLlh& self);
void extract(MipSerializer& serializer, PositionLlh& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_velocity_ned  None
/// Filter reported velocity in the NED local-level frame.
///
///@{

struct VelocityNed
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::data_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::data_filter::DATA_VEL_NED;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    float north;
    float east;
    float down;
    uint16_t valid_flags;
    
};
void insert(MipSerializer& serializer, const VelocityNed& self);
void extract(MipSerializer& serializer, VelocityNed& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_attitude_quaternion  None
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

struct AttitudeQuaternion
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::data_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::data_filter::DATA_ATT_QUATERNION;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    float q[4];
    uint16_t valid_flags;
    
};
void insert(MipSerializer& serializer, const AttitudeQuaternion& self);
void extract(MipSerializer& serializer, AttitudeQuaternion& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_attitude_dcm  None
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

struct AttitudeDcm
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::data_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::data_filter::DATA_ATT_MATRIX;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    float dcm[9];
    uint16_t valid_flags;
    
};
void insert(MipSerializer& serializer, const AttitudeDcm& self);
void extract(MipSerializer& serializer, AttitudeDcm& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_euler_angles  None
/// Filter reported Euler angles describing the orientation of the device with respect to the NED local-level frame.
/// The Euler angles are reported in 3-2-1 (Yaw-Pitch-Roll, AKA Aircraft) order.
///
///@{

struct EulerAngles
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::data_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::data_filter::DATA_ATT_EULER_ANGLES;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    float roll;
    float pitch;
    float yaw;
    uint16_t valid_flags;
    
};
void insert(MipSerializer& serializer, const EulerAngles& self);
void extract(MipSerializer& serializer, EulerAngles& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_gyro_bias  None
/// Filter reported gyro bias expressed in the sensor frame.
///
///@{

struct GyroBias
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::data_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::data_filter::DATA_GYRO_BIAS;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    float bias[3];
    uint16_t valid_flags;
    
};
void insert(MipSerializer& serializer, const GyroBias& self);
void extract(MipSerializer& serializer, GyroBias& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_accel_bias  None
/// Filter reported accelerometer bias expressed in the sensor frame.
///
///@{

struct AccelBias
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::data_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::data_filter::DATA_ACCEL_BIAS;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    float bias[3];
    uint16_t valid_flags;
    
};
void insert(MipSerializer& serializer, const AccelBias& self);
void extract(MipSerializer& serializer, AccelBias& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_position_llh_uncertainty  LLH Position Uncertainty
/// Filter reported 1-sigma position uncertainty in the NED local-level frame.
///
///@{

struct PositionLlhUncertainty
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::data_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::data_filter::DATA_POS_UNCERTAINTY;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    float north;
    float east;
    float down;
    uint16_t valid_flags;
    
};
void insert(MipSerializer& serializer, const PositionLlhUncertainty& self);
void extract(MipSerializer& serializer, PositionLlhUncertainty& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_velocity_ned_uncertainty  NED Velocity Uncertainty
/// Filter reported 1-sigma velocity uncertainties in the NED local-level frame.
///
///@{

struct VelocityNedUncertainty
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::data_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::data_filter::DATA_VEL_UNCERTAINTY;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    float north;
    float east;
    float down;
    uint16_t valid_flags;
    
};
void insert(MipSerializer& serializer, const VelocityNedUncertainty& self);
void extract(MipSerializer& serializer, VelocityNedUncertainty& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_euler_angles_uncertainty  None
/// Filter reported 1-sigma Euler angle uncertainties.
/// The uncertainties are reported in 3-2-1 (Yaw-Pitch-Roll, AKA Aircraft) order.
///
///@{

struct EulerAnglesUncertainty
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::data_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::data_filter::DATA_ATT_UNCERTAINTY_EULER;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    float roll;
    float pitch;
    float yaw;
    uint16_t valid_flags;
    
};
void insert(MipSerializer& serializer, const EulerAnglesUncertainty& self);
void extract(MipSerializer& serializer, EulerAnglesUncertainty& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_gyro_bias_uncertainty  None
/// Filter reported 1-sigma gyro bias uncertainties expressed in the sensor frame.
///
///@{

struct GyroBiasUncertainty
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::data_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::data_filter::DATA_GYRO_BIAS_UNCERTAINTY;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    float bias_uncert[3];
    uint16_t valid_flags;
    
};
void insert(MipSerializer& serializer, const GyroBiasUncertainty& self);
void extract(MipSerializer& serializer, GyroBiasUncertainty& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_accel_bias_uncertainty  None
/// Filter reported 1-sigma accelerometer bias uncertainties expressed in the sensor frame.
///
///@{

struct AccelBiasUncertainty
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::data_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::data_filter::DATA_ACCEL_BIAS_UNCERTAINTY;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    float bias_uncert[3];
    uint16_t valid_flags;
    
};
void insert(MipSerializer& serializer, const AccelBiasUncertainty& self);
void extract(MipSerializer& serializer, AccelBiasUncertainty& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_timestamp  None
/// GPS timestamp of the Filter data
/// 
/// Should the PPS become unavailable, the device will revert to its internal clock, which will cause the reported time to drift from true GPS time.
/// Upon recovering from a PPS outage, the user should expect a jump in the reported GPS time due to the accumulation of internal clock error.
/// If synchronization to an external clock or onboard GNSS receiver (for products that have one) is disabled, this time is equivalent to internal system time.
/// 
/// Note: this data field may be deprecrated in the future. The more flexible shared data field (0x82, 0xD3) should be used instead.
///
///@{

struct Timestamp
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::data_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::data_filter::DATA_FILTER_TIMESTAMP;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    double tow;
    uint16_t week_number;
    uint16_t valid_flags;
    
};
void insert(MipSerializer& serializer, const Timestamp& self);
void extract(MipSerializer& serializer, Timestamp& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_status  None
/// Device-specific filter status indicators.
///
///@{

struct Status
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::data_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::data_filter::DATA_FILTER_STATUS;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    FilterMode filter_state;
    FilterDynamicsMode dynamics_mode;
    FilterStatusFlags status_flags;
    
};
void insert(MipSerializer& serializer, const Status& self);
void extract(MipSerializer& serializer, Status& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_linear_accel  None
/// Filter-compensated linear acceleration expressed in the vehicle frame.
/// Note: The estimated gravity has been removed from this data leaving only linear acceleration.
///
///@{

struct LinearAccel
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::data_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::data_filter::DATA_LINEAR_ACCELERATION;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    float accel[3];
    uint16_t valid_flags;
    
};
void insert(MipSerializer& serializer, const LinearAccel& self);
void extract(MipSerializer& serializer, LinearAccel& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_gravity_vector  None
/// Filter reported gravity vector expressed in the vehicle frame.
///
///@{

struct GravityVector
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::data_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::data_filter::DATA_GRAVITY_VECTOR;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    float gravity[3];
    uint16_t valid_flags;
    
};
void insert(MipSerializer& serializer, const GravityVector& self);
void extract(MipSerializer& serializer, GravityVector& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_comp_accel  Compensated Acceleration
/// Filter-compensated acceleration expressed in the vehicle frame.
///
///@{

struct CompAccel
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::data_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::data_filter::DATA_COMPENSATED_ACCELERATION;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    float accel[3];
    uint16_t valid_flags;
    
};
void insert(MipSerializer& serializer, const CompAccel& self);
void extract(MipSerializer& serializer, CompAccel& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_comp_angular_rate  None
/// Filter-compensated angular rate expressed in the vehicle frame.
///
///@{

struct CompAngularRate
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::data_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::data_filter::DATA_COMPENSATED_ANGULAR_RATE;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    float gyro[3];
    uint16_t valid_flags;
    
};
void insert(MipSerializer& serializer, const CompAngularRate& self);
void extract(MipSerializer& serializer, CompAngularRate& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_quaternion_attitude_uncertainty  None
/// Filter reported quaternion uncertainties.
///
///@{

struct QuaternionAttitudeUncertainty
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::data_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::data_filter::DATA_ATT_UNCERTAINTY_QUATERNION;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    float q[4];
    uint16_t valid_flags;
    
};
void insert(MipSerializer& serializer, const QuaternionAttitudeUncertainty& self);
void extract(MipSerializer& serializer, QuaternionAttitudeUncertainty& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_wgs84_gravity_mag  None
/// Filter reported WGS84 gravity magnitude.
///
///@{

struct Wgs84GravityMag
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::data_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::data_filter::DATA_WGS84_GRAVITY;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    float magnitude;
    uint16_t valid_flags;
    
};
void insert(MipSerializer& serializer, const Wgs84GravityMag& self);
void extract(MipSerializer& serializer, Wgs84GravityMag& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_heading_update_state  None
/// Filter reported heading update state.
/// 
/// Heading updates can be applied from the sources listed below.  Note, some of these sources may be combined.
/// The heading value is always relative to true north.
///
///@{

struct HeadingUpdateState
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::data_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::data_filter::DATA_HEADING_UPDATE_STATE;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    enum class HeadingSource : uint16_t
    {
        NONE                 = 0,  ///<  
        MAGNETOMETER         = 1,  ///<  
        GNSS_VELOCITY_VECTOR = 2,  ///<  
        EXTERNAL             = 4,  ///<  
        DUAL_ANTENNA         = 8,  ///<  
    };
    
    float heading;
    float heading_1sigma;
    HeadingSource source;
    uint16_t valid_flags;
    
};
void insert(MipSerializer& serializer, const HeadingUpdateState& self);
void extract(MipSerializer& serializer, HeadingUpdateState& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_magnetic_model  None
/// The World Magnetic Model is used for this data. Please refer to the device user manual for the current version of the model.
/// A valid GNSS location is required for the model to be valid.
///
///@{

struct MagneticModel
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::data_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::data_filter::DATA_MAGNETIC_MODEL;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    float intensity_north;
    float intensity_east;
    float intensity_down;
    float inclination;
    float declination;
    uint16_t valid_flags;
    
};
void insert(MipSerializer& serializer, const MagneticModel& self);
void extract(MipSerializer& serializer, MagneticModel& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_accel_scale_factor  None
/// Filter reported accelerometer scale factor expressed in the sensor frame.
///
///@{

struct AccelScaleFactor
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::data_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::data_filter::DATA_ACCEL_SCALE_FACTOR;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    float scale_factor[3];
    uint16_t valid_flags;
    
};
void insert(MipSerializer& serializer, const AccelScaleFactor& self);
void extract(MipSerializer& serializer, AccelScaleFactor& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_accel_scale_factor_uncertainty  None
/// Filter reported 1-sigma accelerometer scale factor uncertainty expressed in the sensor frame.
///
///@{

struct AccelScaleFactorUncertainty
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::data_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::data_filter::DATA_ACCEL_SCALE_FACTOR_UNCERTAINTY;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    float scale_factor_uncert[3];
    uint16_t valid_flags;
    
};
void insert(MipSerializer& serializer, const AccelScaleFactorUncertainty& self);
void extract(MipSerializer& serializer, AccelScaleFactorUncertainty& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_gyro_scale_factor  None
/// Filter reported gyro scale factor expressed in the sensor frame.
///
///@{

struct GyroScaleFactor
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::data_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::data_filter::DATA_GYRO_SCALE_FACTOR;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    float scale_factor[3];
    uint16_t valid_flags;
    
};
void insert(MipSerializer& serializer, const GyroScaleFactor& self);
void extract(MipSerializer& serializer, GyroScaleFactor& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_gyro_scale_factor_uncertainty  None
/// Filter reported 1-sigma gyro scale factor uncertainty expressed in the sensor frame.
///
///@{

struct GyroScaleFactorUncertainty
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::data_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::data_filter::DATA_GYRO_SCALE_FACTOR_UNCERTAINTY;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    float scale_factor_uncert[3];
    uint16_t valid_flags;
    
};
void insert(MipSerializer& serializer, const GyroScaleFactorUncertainty& self);
void extract(MipSerializer& serializer, GyroScaleFactorUncertainty& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_mag_bias  None
/// Filter reported magnetometer bias expressed in the sensor frame.
///
///@{

struct MagBias
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::data_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::data_filter::DATA_MAG_BIAS;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    float bias[3];
    uint16_t valid_flags;
    
};
void insert(MipSerializer& serializer, const MagBias& self);
void extract(MipSerializer& serializer, MagBias& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_mag_bias_uncertainty  None
/// Filter reported 1-sigma magnetometer bias uncertainty expressed in the sensor frame.
///
///@{

struct MagBiasUncertainty
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::data_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::data_filter::DATA_MAG_BIAS_UNCERTAINTY;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    float bias_uncert[3];
    uint16_t valid_flags;
    
};
void insert(MipSerializer& serializer, const MagBiasUncertainty& self);
void extract(MipSerializer& serializer, MagBiasUncertainty& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_standard_atmosphere  None
/// Filter reported standard atmosphere parameters.
/// 
/// The US 1976 Standard Atmosphere Model is used. A valid GNSS location is required for the model to be valid.
///
///@{

struct StandardAtmosphere
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::data_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::data_filter::DATA_STANDARD_ATMOSPHERE_DATA;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    float geometric_altitude;
    float geopotential_altitude;
    float standard_temperature;
    float standard_pressure;
    float standard_density;
    uint16_t valid_flags;
    
};
void insert(MipSerializer& serializer, const StandardAtmosphere& self);
void extract(MipSerializer& serializer, StandardAtmosphere& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_pressure_altitude  None
/// Filter reported pressure altitude.
/// 
/// The US 1976 Standard Atmosphere Model is used to calculate the pressure altitude in meters.
/// A valid pressure sensor reading is required for the pressure altitude to be valid.
/// The minimum pressure reading supported by the model is 0.0037 mBar, corresponding to an altitude of 84,852 meters.
///
///@{

struct PressureAltitude
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::data_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::data_filter::DATA_PRESSURE_ALTITUDE_DATA;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    float pressure_altitude;
    uint16_t valid_flags;
    
};
void insert(MipSerializer& serializer, const PressureAltitude& self);
void extract(MipSerializer& serializer, PressureAltitude& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_density_altitude  None
///
///@{

struct DensityAltitude
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::data_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::data_filter::DATA_DENSITY_ALTITUDE_DATA;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    float density_altitude;
    uint16_t valid_flags;
    
};
void insert(MipSerializer& serializer, const DensityAltitude& self);
void extract(MipSerializer& serializer, DensityAltitude& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_antenna_offset_correction  None
/// Filter reported GNSS antenna offset in vehicle frame.
/// 
/// This offset added to any previously stored offset vector to compensate for errors in definition.
///
///@{

struct AntennaOffsetCorrection
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::data_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::data_filter::DATA_ANTENNA_OFFSET_CORRECTION;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    float offset[3];
    uint16_t valid_flags;
    
};
void insert(MipSerializer& serializer, const AntennaOffsetCorrection& self);
void extract(MipSerializer& serializer, AntennaOffsetCorrection& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_antenna_offset_correction_uncertainty  None
/// Filter reported 1-sigma GNSS antenna offset uncertainties in vehicle frame.
///
///@{

struct AntennaOffsetCorrectionUncertainty
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::data_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::data_filter::DATA_ANTENNA_OFFSET_CORRECTION_UNCERTAINTY;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    float offset_uncert[3];
    uint16_t valid_flags;
    
};
void insert(MipSerializer& serializer, const AntennaOffsetCorrectionUncertainty& self);
void extract(MipSerializer& serializer, AntennaOffsetCorrectionUncertainty& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_multi_antenna_offset_correction  None
/// Filter reported GNSS antenna offset in vehicle frame.
/// 
/// This offset added to any previously stored offset vector to compensate for errors in definition.
///
///@{

struct MultiAntennaOffsetCorrection
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::data_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::data_filter::DATA_MULTI_ANTENNA_OFFSET_CORRECTION;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    uint8_t receiver_id;
    float offset[3];
    uint16_t valid_flags;
    
};
void insert(MipSerializer& serializer, const MultiAntennaOffsetCorrection& self);
void extract(MipSerializer& serializer, MultiAntennaOffsetCorrection& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_multi_antenna_offset_correction_uncertainty  None
/// Filter reported 1-sigma GNSS antenna offset uncertainties in vehicle frame.
///
///@{

struct MultiAntennaOffsetCorrectionUncertainty
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::data_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::data_filter::DATA_MULTI_ANTENNA_OFFSET_CORRECTION_UNCERTAINTY;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    uint8_t receiver_id;
    float offset_uncert[3];
    uint16_t valid_flags;
    
};
void insert(MipSerializer& serializer, const MultiAntennaOffsetCorrectionUncertainty& self);
void extract(MipSerializer& serializer, MultiAntennaOffsetCorrectionUncertainty& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_magnetometer_offset  None
/// Filter reported magnetometer hard iron offset in sensor frame.
/// 
/// This offset added to any previously stored hard iron offset vector to compensate for magnetometer in-run bias errors.
///
///@{

struct MagnetometerOffset
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::data_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::data_filter::DATA_MAG_COMPENSATION_OFFSET;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    float hard_iron[3];
    uint16_t valid_flags;
    
};
void insert(MipSerializer& serializer, const MagnetometerOffset& self);
void extract(MipSerializer& serializer, MagnetometerOffset& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_magnetometer_matrix  None
/// Filter reported magnetometer soft iron matrix in sensor frame.
/// 
/// This matrix is post multiplied to any previously stored soft iron matrix to compensate for magnetometer in-run errors.
///
///@{

struct MagnetometerMatrix
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::data_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::data_filter::DATA_MAG_COMPENSATION_MATRIX;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    float soft_iron[9];
    uint16_t valid_flags;
    
};
void insert(MipSerializer& serializer, const MagnetometerMatrix& self);
void extract(MipSerializer& serializer, MagnetometerMatrix& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_magnetometer_offset_uncertainty  None
/// Filter reported 1-sigma magnetometer hard iron offset uncertainties in sensor frame.
///
///@{

struct MagnetometerOffsetUncertainty
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::data_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::data_filter::DATA_MAG_COMPENSATION_OFFSET_UNCERTAINTY;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    float hard_iron_uncertainty[3];
    uint16_t valid_flags;
    
};
void insert(MipSerializer& serializer, const MagnetometerOffsetUncertainty& self);
void extract(MipSerializer& serializer, MagnetometerOffsetUncertainty& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_magnetometer_matrix_uncertainty  None
/// Filter reported 1-sigma magnetometer soft iron matrix uncertainties in sensor frame.
///
///@{

struct MagnetometerMatrixUncertainty
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::data_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::data_filter::DATA_MAG_COMPENSATION_MATRIX_UNCERTAINTY;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    float soft_iron_uncertainty[9];
    uint16_t valid_flags;
    
};
void insert(MipSerializer& serializer, const MagnetometerMatrixUncertainty& self);
void extract(MipSerializer& serializer, MagnetometerMatrixUncertainty& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_magnetometer_covariance_matrix  None
///
///@{

struct MagnetometerCovarianceMatrix
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::data_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::data_filter::DATA_MAG_COVARIANCE;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    float covariance[9];
    uint16_t valid_flags;
    
};
void insert(MipSerializer& serializer, const MagnetometerCovarianceMatrix& self);
void extract(MipSerializer& serializer, MagnetometerCovarianceMatrix& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_magnetometer_residual_vector  None
/// Filter reported magnetometer measurement residuals in vehicle frame.
///
///@{

struct MagnetometerResidualVector
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::data_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::data_filter::DATA_MAG_RESIDUAL;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    float residual[3];
    uint16_t valid_flags;
    
};
void insert(MipSerializer& serializer, const MagnetometerResidualVector& self);
void extract(MipSerializer& serializer, MagnetometerResidualVector& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_clock_correction  None
/// Filter reported GNSS receiver clock error parameters.
///
///@{

struct ClockCorrection
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::data_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::data_filter::DATA_CLOCK_CORRECTION;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    uint8_t receiver_id;
    float bias;
    float bias_drift;
    uint16_t valid_flags;
    
};
void insert(MipSerializer& serializer, const ClockCorrection& self);
void extract(MipSerializer& serializer, ClockCorrection& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_clock_correction_uncertainty  None
/// Filter reported 1-sigma GNSS receiver clock error parameters.
///
///@{

struct ClockCorrectionUncertainty
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::data_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::data_filter::DATA_CLOCK_CORRECTION_UNCERTAINTY;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    uint8_t receiver_id;
    float bias_uncertainty;
    float bias_drift_uncertainty;
    uint16_t valid_flags;
    
};
void insert(MipSerializer& serializer, const ClockCorrectionUncertainty& self);
void extract(MipSerializer& serializer, ClockCorrectionUncertainty& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_gnss_pos_aid_status  GNSS Position Aiding Status
/// Filter reported GNSS position aiding status
///
///@{

struct GnssPosAidStatus
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::data_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::data_filter::DATA_GNSS_POS_AID_STATUS;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    uint8_t receiver_id;
    float time_of_week;
    GnssAidStatusFlags status;
    uint8_t reserved[8];
    
};
void insert(MipSerializer& serializer, const GnssPosAidStatus& self);
void extract(MipSerializer& serializer, GnssPosAidStatus& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_gnss_att_aid_status  GNSS Attitude Aiding Status
/// Filter reported dual antenna GNSS attitude aiding status
///
///@{

struct GnssAttAidStatus
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::data_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::data_filter::DATA_GNSS_ATT_AID_STATUS;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    float time_of_week;
    GnssAidStatusFlags status;
    uint8_t reserved[8];
    
};
void insert(MipSerializer& serializer, const GnssAttAidStatus& self);
void extract(MipSerializer& serializer, GnssAttAidStatus& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_head_aid_status  None
/// Filter reported GNSS heading aiding status
///
///@{

struct HeadAidStatus
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::data_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::data_filter::DATA_HEAD_AID_STATUS;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    enum class HeadingAidType : uint8_t
    {
        DUAL_ANTENNA     = 1,  ///<  
        EXTERNAL_MESSAGE = 2,  ///<  
    };
    
    float time_of_week;
    HeadingAidType type;
    float reserved[2];
    
};
void insert(MipSerializer& serializer, const HeadAidStatus& self);
void extract(MipSerializer& serializer, HeadAidStatus& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_rel_pos_ned  NED Relative Position
/// Filter reported relative position, with respect to configured reference position
///
///@{

struct RelPosNed
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::data_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::data_filter::DATA_REL_POS_NED;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    double relative_position[3];
    uint16_t valid_flags;
    
};
void insert(MipSerializer& serializer, const RelPosNed& self);
void extract(MipSerializer& serializer, RelPosNed& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_ecef_pos  ECEF Position
/// Filter reported ECEF position
///
///@{

struct EcefPos
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::data_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::data_filter::DATA_ECEF_POS;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    double position_ecef[3];
    uint16_t valid_flags;
    
};
void insert(MipSerializer& serializer, const EcefPos& self);
void extract(MipSerializer& serializer, EcefPos& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_ecef_vel  ECEF Velocity
/// Filter reported ECEF velocity
///
///@{

struct EcefVel
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::data_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::data_filter::DATA_ECEF_VEL;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    float velocity_ecef[3];
    uint16_t valid_flags;
    
};
void insert(MipSerializer& serializer, const EcefVel& self);
void extract(MipSerializer& serializer, EcefVel& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_ecef_pos_uncertainty  ECEF Position Uncertainty
/// Filter reported 1-sigma position uncertainty in the ECEF frame.
///
///@{

struct EcefPosUncertainty
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::data_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::data_filter::DATA_ECEF_POS_UNCERTAINTY;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    float pos_uncertainty[3];
    uint16_t valid_flags;
    
};
void insert(MipSerializer& serializer, const EcefPosUncertainty& self);
void extract(MipSerializer& serializer, EcefPosUncertainty& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_ecef_vel_uncertainty  ECEF Velocity Uncertainty
/// Filter reported 1-sigma velocity uncertainties in the ECEF frame.
///
///@{

struct EcefVelUncertainty
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::data_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::data_filter::DATA_ECEF_VEL_UNCERTAINTY;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    float vel_uncertainty[3];
    uint16_t valid_flags;
    
};
void insert(MipSerializer& serializer, const EcefVelUncertainty& self);
void extract(MipSerializer& serializer, EcefVelUncertainty& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_aiding_measurement_summary  None
/// Filter reported aiding measurement summary. This message contains a summary of the specified aiding measurement over the previous measurement interval ending at the specified time.
///
///@{

struct AidingMeasurementSummary
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::data_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::data_filter::DATA_AID_MEAS_SUMMARY;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    float time_of_week;
    uint8_t source;
    FilterAidingMeasurementType type;
    FilterMeasurementIndicator indicator;
    
};
void insert(MipSerializer& serializer, const AidingMeasurementSummary& self);
void extract(MipSerializer& serializer, AidingMeasurementSummary& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_odometer_scale_factor_error  Odometer Scale Factor Error
/// Filter reported odometer scale factor error. The total scale factor estimate is the user indicated scale factor, plus the user indicated scale factor times the scale factor error.
///
///@{

struct OdometerScaleFactorError
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::data_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::data_filter::DATA_ODOMETER_SCALE_FACTOR_ERROR;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    float scale_factor_error;
    uint16_t valid_flags;
    
};
void insert(MipSerializer& serializer, const OdometerScaleFactorError& self);
void extract(MipSerializer& serializer, OdometerScaleFactorError& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_odometer_scale_factor_error_uncertainty  Odometer Scale Factor Error Uncertainty
/// Filter reported odometer scale factor error uncertainty.
///
///@{

struct OdometerScaleFactorErrorUncertainty
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::data_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::data_filter::DATA_ODOMETER_SCALE_FACTOR_ERROR_UNCERTAINTY;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    float scale_factor_error_uncertainty;
    uint16_t valid_flags;
    
};
void insert(MipSerializer& serializer, const OdometerScaleFactorErrorUncertainty& self);
void extract(MipSerializer& serializer, OdometerScaleFactorErrorUncertainty& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_gnss_dual_antenna_status  GNSS Dual Antenna Status
/// Summary information for status of GNSS dual antenna heading estimate.
///
///@{

struct GnssDualAntennaStatus
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::data_filter::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::data_filter::DATA_GNSS_DUAL_ANTENNA_STATUS;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    enum class FixType : uint8_t
    {
        FIX_NONE     = 0,  ///<  
        FIX_DA_FLOAT = 1,  ///<  
        FIX_DA_FIXED = 2,  ///<  
    };
    
    enum class DualAntennaStatusFlags : uint16_t
    {
        RCV_1_DATA_VALID      = 0x01,
        RCV_2_DATA_VALID      = 0x02,
        ANTENNA_OFFSETS_VALID = 0x04,
    };
    
    float time_of_week;
    float heading;
    float heading_unc;
    FixType fix_type;
    DualAntennaStatusFlags status_flags;
    uint16_t valid_flags;
    
};
void insert(MipSerializer& serializer, const GnssDualAntennaStatus& self);
void extract(MipSerializer& serializer, GnssDualAntennaStatus& self);

///@}
///

///@}
///@}
///
////////////////////////////////////////////////////////////////////////////////
} // namespace data_filter
} // namespace mscl

