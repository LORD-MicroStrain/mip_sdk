#pragma once

#include <mip/definitions/common.hpp>
#include <mip/mip_descriptors.hpp>
#include <mip/mip_result.hpp>
#include <mip/mip_interface.hpp>

#include <stdint.h>
#include <stddef.h>

namespace mip {
namespace C {
struct mip_interface;
} // namespace C

namespace data_filter {

////////////////////////////////////////////////////////////////////////////////
///@addtogroup MipData_cpp
///@{
///@defgroup filter_data_cpp  Filter Data
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
    DATA_FRAME_CONFIG_ERROR                          = 0x50,
    DATA_FRAME_CONFIG_ERROR_UNCERTAINTY              = 0x51,
    
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
    INIT                   = 1,  ///<  
    VERT_GYRO              = 2,  ///<  
    AHRS                   = 3,  ///<  
    FULL_NAV               = 4,  ///<  
};

enum class FilterDynamicsMode : uint16_t
{
    GX5_PORTABLE   = 1,  ///<  
    GX5_AUTOMOTIVE = 2,  ///<  
    GX5_AIRBORNE   = 3,  ///<  
    GQ7_DEFAULT    = 1,  ///<  
};

struct FilterStatusFlags : Bitfield<FilterStatusFlags>
{
    typedef uint16_t Type;
    enum _enumType : uint16_t
    {
        NONE                                           = 0x0000,
        GX5_INIT_NO_ATTITUDE                           = 0x1000,  ///<  
        GX5_INIT_NO_POSITION_VELOCITY                  = 0x2000,  ///<  
        GX5_RUN_IMU_UNAVAILABLE                        = 0x0001,  ///<  
        GX5_RUN_GPS_UNAVAILABLE                        = 0x0002,  ///<  
        GX5_RUN_MATRIX_SINGULARITY                     = 0x0008,  ///<  
        GX5_RUN_POSITION_COVARIANCE_WARNING            = 0x0010,  ///<  
        GX5_RUN_VELOCITY_COVARIANCE_WARNING            = 0x0020,  ///<  
        GX5_RUN_ATTITUDE_COVARIANCE_WARNING            = 0x0040,  ///<  
        GX5_RUN_NAN_IN_SOLUTION_WARNING                = 0x0080,  ///<  
        GX5_RUN_GYRO_BIAS_EST_HIGH_WARNING             = 0x0100,  ///<  
        GX5_RUN_ACCEL_BIAS_EST_HIGH_WARNING            = 0x0200,  ///<  
        GX5_RUN_GYRO_SCALE_FACTOR_EST_HIGH_WARNING     = 0x0400,  ///<  
        GX5_RUN_ACCEL_SCALE_FACTOR_EST_HIGH_WARNING    = 0x0800,  ///<  
        GX5_RUN_MAG_BIAS_EST_HIGH_WARNING              = 0x1000,  ///<  
        GX5_RUN_ANT_OFFSET_CORRECTION_EST_HIGH_WARNING = 0x2000,  ///<  
        GX5_RUN_MAG_HARD_IRON_EST_HIGH_WARNING         = 0x4000,  ///<  
        GX5_RUN_MAG_SOFT_IRON_EST_HIGH_WARNING         = 0x8000,  ///<  
        GQ7_FILTER_CONDITION                           = 0x0003,  ///<  
        GQ7_ROLL_PITCH_WARNING                         = 0x0004,  ///<  
        GQ7_HEADING_WARNING                            = 0x0008,  ///<  
        GQ7_POSITION_WARNING                           = 0x0010,  ///<  
        GQ7_VELOCITY_WARNING                           = 0x0020,  ///<  
        GQ7_IMU_BIAS_WARNING                           = 0x0040,  ///<  
        GQ7_GNSS_CLK_WARNING                           = 0x0080,  ///<  
        GQ7_ANTENNA_LEVER_ARM_WARNING                  = 0x0100,  ///<  
        GQ7_MOUNTING_TRANSFORM_WARNING                 = 0x0200,  ///<  
        GQ7_TIME_SYNC_WARNING                          = 0x0400,  ///<  No time synchronization pulse detected
        GQ7_SOLUTION_ERROR                             = 0xF000,  ///<  Filter computation warning flags. If any bits 12-15 are set, and all filter outputs will be invalid.
        ALL                                            = 0xFFFF,
    };
    uint16_t value = NONE;
    
    constexpr FilterStatusFlags() : value(NONE) {}
    constexpr FilterStatusFlags(int val) : value((uint16_t)val) {}
    constexpr operator uint16_t() const { return value; }
    constexpr FilterStatusFlags& operator=(uint16_t val) { value = val; return *this; }
    constexpr FilterStatusFlags& operator=(int val) { value = uint16_t(val); return *this; }
    constexpr FilterStatusFlags& operator|=(uint16_t val) { return *this = value | val; }
    constexpr FilterStatusFlags& operator&=(uint16_t val) { return *this = value & val; }
    
    constexpr bool gx5InitNoAttitude() const { return (value & GX5_INIT_NO_ATTITUDE) > 0; }
    constexpr void gx5InitNoAttitude(bool val) { value &= ~GX5_INIT_NO_ATTITUDE; if(val) value |= GX5_INIT_NO_ATTITUDE; }
    constexpr bool gx5InitNoPositionVelocity() const { return (value & GX5_INIT_NO_POSITION_VELOCITY) > 0; }
    constexpr void gx5InitNoPositionVelocity(bool val) { value &= ~GX5_INIT_NO_POSITION_VELOCITY; if(val) value |= GX5_INIT_NO_POSITION_VELOCITY; }
    constexpr bool gx5RunImuUnavailable() const { return (value & GX5_RUN_IMU_UNAVAILABLE) > 0; }
    constexpr void gx5RunImuUnavailable(bool val) { value &= ~GX5_RUN_IMU_UNAVAILABLE; if(val) value |= GX5_RUN_IMU_UNAVAILABLE; }
    constexpr bool gx5RunGpsUnavailable() const { return (value & GX5_RUN_GPS_UNAVAILABLE) > 0; }
    constexpr void gx5RunGpsUnavailable(bool val) { value &= ~GX5_RUN_GPS_UNAVAILABLE; if(val) value |= GX5_RUN_GPS_UNAVAILABLE; }
    constexpr bool gx5RunMatrixSingularity() const { return (value & GX5_RUN_MATRIX_SINGULARITY) > 0; }
    constexpr void gx5RunMatrixSingularity(bool val) { value &= ~GX5_RUN_MATRIX_SINGULARITY; if(val) value |= GX5_RUN_MATRIX_SINGULARITY; }
    constexpr bool gx5RunPositionCovarianceWarning() const { return (value & GX5_RUN_POSITION_COVARIANCE_WARNING) > 0; }
    constexpr void gx5RunPositionCovarianceWarning(bool val) { value &= ~GX5_RUN_POSITION_COVARIANCE_WARNING; if(val) value |= GX5_RUN_POSITION_COVARIANCE_WARNING; }
    constexpr bool gx5RunVelocityCovarianceWarning() const { return (value & GX5_RUN_VELOCITY_COVARIANCE_WARNING) > 0; }
    constexpr void gx5RunVelocityCovarianceWarning(bool val) { value &= ~GX5_RUN_VELOCITY_COVARIANCE_WARNING; if(val) value |= GX5_RUN_VELOCITY_COVARIANCE_WARNING; }
    constexpr bool gx5RunAttitudeCovarianceWarning() const { return (value & GX5_RUN_ATTITUDE_COVARIANCE_WARNING) > 0; }
    constexpr void gx5RunAttitudeCovarianceWarning(bool val) { value &= ~GX5_RUN_ATTITUDE_COVARIANCE_WARNING; if(val) value |= GX5_RUN_ATTITUDE_COVARIANCE_WARNING; }
    constexpr bool gx5RunNanInSolutionWarning() const { return (value & GX5_RUN_NAN_IN_SOLUTION_WARNING) > 0; }
    constexpr void gx5RunNanInSolutionWarning(bool val) { value &= ~GX5_RUN_NAN_IN_SOLUTION_WARNING; if(val) value |= GX5_RUN_NAN_IN_SOLUTION_WARNING; }
    constexpr bool gx5RunGyroBiasEstHighWarning() const { return (value & GX5_RUN_GYRO_BIAS_EST_HIGH_WARNING) > 0; }
    constexpr void gx5RunGyroBiasEstHighWarning(bool val) { value &= ~GX5_RUN_GYRO_BIAS_EST_HIGH_WARNING; if(val) value |= GX5_RUN_GYRO_BIAS_EST_HIGH_WARNING; }
    constexpr bool gx5RunAccelBiasEstHighWarning() const { return (value & GX5_RUN_ACCEL_BIAS_EST_HIGH_WARNING) > 0; }
    constexpr void gx5RunAccelBiasEstHighWarning(bool val) { value &= ~GX5_RUN_ACCEL_BIAS_EST_HIGH_WARNING; if(val) value |= GX5_RUN_ACCEL_BIAS_EST_HIGH_WARNING; }
    constexpr bool gx5RunGyroScaleFactorEstHighWarning() const { return (value & GX5_RUN_GYRO_SCALE_FACTOR_EST_HIGH_WARNING) > 0; }
    constexpr void gx5RunGyroScaleFactorEstHighWarning(bool val) { value &= ~GX5_RUN_GYRO_SCALE_FACTOR_EST_HIGH_WARNING; if(val) value |= GX5_RUN_GYRO_SCALE_FACTOR_EST_HIGH_WARNING; }
    constexpr bool gx5RunAccelScaleFactorEstHighWarning() const { return (value & GX5_RUN_ACCEL_SCALE_FACTOR_EST_HIGH_WARNING) > 0; }
    constexpr void gx5RunAccelScaleFactorEstHighWarning(bool val) { value &= ~GX5_RUN_ACCEL_SCALE_FACTOR_EST_HIGH_WARNING; if(val) value |= GX5_RUN_ACCEL_SCALE_FACTOR_EST_HIGH_WARNING; }
    constexpr bool gx5RunMagBiasEstHighWarning() const { return (value & GX5_RUN_MAG_BIAS_EST_HIGH_WARNING) > 0; }
    constexpr void gx5RunMagBiasEstHighWarning(bool val) { value &= ~GX5_RUN_MAG_BIAS_EST_HIGH_WARNING; if(val) value |= GX5_RUN_MAG_BIAS_EST_HIGH_WARNING; }
    constexpr bool gx5RunAntOffsetCorrectionEstHighWarning() const { return (value & GX5_RUN_ANT_OFFSET_CORRECTION_EST_HIGH_WARNING) > 0; }
    constexpr void gx5RunAntOffsetCorrectionEstHighWarning(bool val) { value &= ~GX5_RUN_ANT_OFFSET_CORRECTION_EST_HIGH_WARNING; if(val) value |= GX5_RUN_ANT_OFFSET_CORRECTION_EST_HIGH_WARNING; }
    constexpr bool gx5RunMagHardIronEstHighWarning() const { return (value & GX5_RUN_MAG_HARD_IRON_EST_HIGH_WARNING) > 0; }
    constexpr void gx5RunMagHardIronEstHighWarning(bool val) { value &= ~GX5_RUN_MAG_HARD_IRON_EST_HIGH_WARNING; if(val) value |= GX5_RUN_MAG_HARD_IRON_EST_HIGH_WARNING; }
    constexpr bool gx5RunMagSoftIronEstHighWarning() const { return (value & GX5_RUN_MAG_SOFT_IRON_EST_HIGH_WARNING) > 0; }
    constexpr void gx5RunMagSoftIronEstHighWarning(bool val) { value &= ~GX5_RUN_MAG_SOFT_IRON_EST_HIGH_WARNING; if(val) value |= GX5_RUN_MAG_SOFT_IRON_EST_HIGH_WARNING; }
    constexpr uint16_t gq7FilterCondition() const { return (value & GQ7_FILTER_CONDITION) >> 0; }
    constexpr void gq7FilterCondition(uint16_t val) { value = (value & ~GQ7_FILTER_CONDITION) | (val << 0); }
    constexpr bool gq7RollPitchWarning() const { return (value & GQ7_ROLL_PITCH_WARNING) > 0; }
    constexpr void gq7RollPitchWarning(bool val) { value &= ~GQ7_ROLL_PITCH_WARNING; if(val) value |= GQ7_ROLL_PITCH_WARNING; }
    constexpr bool gq7HeadingWarning() const { return (value & GQ7_HEADING_WARNING) > 0; }
    constexpr void gq7HeadingWarning(bool val) { value &= ~GQ7_HEADING_WARNING; if(val) value |= GQ7_HEADING_WARNING; }
    constexpr bool gq7PositionWarning() const { return (value & GQ7_POSITION_WARNING) > 0; }
    constexpr void gq7PositionWarning(bool val) { value &= ~GQ7_POSITION_WARNING; if(val) value |= GQ7_POSITION_WARNING; }
    constexpr bool gq7VelocityWarning() const { return (value & GQ7_VELOCITY_WARNING) > 0; }
    constexpr void gq7VelocityWarning(bool val) { value &= ~GQ7_VELOCITY_WARNING; if(val) value |= GQ7_VELOCITY_WARNING; }
    constexpr bool gq7ImuBiasWarning() const { return (value & GQ7_IMU_BIAS_WARNING) > 0; }
    constexpr void gq7ImuBiasWarning(bool val) { value &= ~GQ7_IMU_BIAS_WARNING; if(val) value |= GQ7_IMU_BIAS_WARNING; }
    constexpr bool gq7GnssClkWarning() const { return (value & GQ7_GNSS_CLK_WARNING) > 0; }
    constexpr void gq7GnssClkWarning(bool val) { value &= ~GQ7_GNSS_CLK_WARNING; if(val) value |= GQ7_GNSS_CLK_WARNING; }
    constexpr bool gq7AntennaLeverArmWarning() const { return (value & GQ7_ANTENNA_LEVER_ARM_WARNING) > 0; }
    constexpr void gq7AntennaLeverArmWarning(bool val) { value &= ~GQ7_ANTENNA_LEVER_ARM_WARNING; if(val) value |= GQ7_ANTENNA_LEVER_ARM_WARNING; }
    constexpr bool gq7MountingTransformWarning() const { return (value & GQ7_MOUNTING_TRANSFORM_WARNING) > 0; }
    constexpr void gq7MountingTransformWarning(bool val) { value &= ~GQ7_MOUNTING_TRANSFORM_WARNING; if(val) value |= GQ7_MOUNTING_TRANSFORM_WARNING; }
    constexpr bool gq7TimeSyncWarning() const { return (value & GQ7_TIME_SYNC_WARNING) > 0; }
    constexpr void gq7TimeSyncWarning(bool val) { value &= ~GQ7_TIME_SYNC_WARNING; if(val) value |= GQ7_TIME_SYNC_WARNING; }
    constexpr uint16_t gq7SolutionError() const { return (value & GQ7_SOLUTION_ERROR) >> 12; }
    constexpr void gq7SolutionError(uint16_t val) { value = (value & ~GQ7_SOLUTION_ERROR) | (val << 12); }
    constexpr bool allSet() const { return value == ALL; }
    constexpr void setAll() { value |= ALL; }
};
enum class FilterAidingMeasurementType : uint8_t
{
    GNSS                          = 1,  ///<  
    DUAL_ANTENNA                  = 2,  ///<  
    HEADING                       = 3,  ///<  
    PRESSURE                      = 4,  ///<  
    MAGNETOMETER                  = 5,  ///<  
    SPEED                         = 6,  ///<  
    AIDING_POS_ECEF               = 33,  ///<  
    AIDING_POS_LLH                = 34,  ///<  
    AIDING_HEIGHT_ABOVE_ELLIPSOID = 35,  ///<  
    AIDING_VEL_ECEF               = 40,  ///<  
    AIDING_VEL_NED                = 41,  ///<  
    AIDING_VEL_BODY_FRAME         = 42,  ///<  
    AIDING_HEADING_TRUE           = 49,  ///<  
    AIDING_MAGNETIC_FIELD         = 50,  ///<  
    AIDING_PRESSURE               = 51,  ///<  
};

struct FilterMeasurementIndicator : Bitfield<FilterMeasurementIndicator>
{
    typedef uint8_t Type;
    enum _enumType : uint8_t
    {
        NONE                  = 0x00,
        ENABLED               = 0x01,  ///<  
        USED                  = 0x02,  ///<  
        RESIDUAL_HIGH_WARNING = 0x04,  ///<  
        SAMPLE_TIME_WARNING   = 0x08,  ///<  
        CONFIGURATION_ERROR   = 0x10,  ///<  
        MAX_NUM_MEAS_EXCEEDED = 0x20,  ///<  
        ALL                   = 0x3F,
    };
    uint8_t value = NONE;
    
    constexpr FilterMeasurementIndicator() : value(NONE) {}
    constexpr FilterMeasurementIndicator(int val) : value((uint8_t)val) {}
    constexpr operator uint8_t() const { return value; }
    constexpr FilterMeasurementIndicator& operator=(uint8_t val) { value = val; return *this; }
    constexpr FilterMeasurementIndicator& operator=(int val) { value = uint8_t(val); return *this; }
    constexpr FilterMeasurementIndicator& operator|=(uint8_t val) { return *this = value | val; }
    constexpr FilterMeasurementIndicator& operator&=(uint8_t val) { return *this = value & val; }
    
    constexpr bool enabled() const { return (value & ENABLED) > 0; }
    constexpr void enabled(bool val) { value &= ~ENABLED; if(val) value |= ENABLED; }
    constexpr bool used() const { return (value & USED) > 0; }
    constexpr void used(bool val) { value &= ~USED; if(val) value |= USED; }
    constexpr bool residualHighWarning() const { return (value & RESIDUAL_HIGH_WARNING) > 0; }
    constexpr void residualHighWarning(bool val) { value &= ~RESIDUAL_HIGH_WARNING; if(val) value |= RESIDUAL_HIGH_WARNING; }
    constexpr bool sampleTimeWarning() const { return (value & SAMPLE_TIME_WARNING) > 0; }
    constexpr void sampleTimeWarning(bool val) { value &= ~SAMPLE_TIME_WARNING; if(val) value |= SAMPLE_TIME_WARNING; }
    constexpr bool configurationError() const { return (value & CONFIGURATION_ERROR) > 0; }
    constexpr void configurationError(bool val) { value &= ~CONFIGURATION_ERROR; if(val) value |= CONFIGURATION_ERROR; }
    constexpr bool maxNumMeasExceeded() const { return (value & MAX_NUM_MEAS_EXCEEDED) > 0; }
    constexpr void maxNumMeasExceeded(bool val) { value &= ~MAX_NUM_MEAS_EXCEEDED; if(val) value |= MAX_NUM_MEAS_EXCEEDED; }
    constexpr bool allSet() const { return value == ALL; }
    constexpr void setAll() { value |= ALL; }
};
struct GnssAidStatusFlags : Bitfield<GnssAidStatusFlags>
{
    typedef uint16_t Type;
    enum _enumType : uint16_t
    {
        NONE           = 0x0000,
        TIGHT_COUPLING = 0x0001,  ///<  If 1, the Kalman filter is processing raw range information from this GNSS module
        DIFFERENTIAL   = 0x0002,  ///<  If 1, the Kalman filter is processing RTK corrections from this GNSS module
        INTEGER_FIX    = 0x0004,  ///<  If 1, the Kalman filter has an RTK integer fix from this GNSS module, indicating the best position performance possible
        GPS_L1         = 0x0008,  ///<  If 1, the Kalman filter is using GPS L1 measurements
        GPS_L2         = 0x0010,  ///<  If 1, the Kalman filter is using GPS L2 measurements
        GPS_L5         = 0x0020,  ///<  If 1, the Kalman filter is using GPS L5 measurements (not available on the GQ7)
        GLO_L1         = 0x0040,  ///<  If 1, the Kalman filter is using GLONASS L1 measurements
        GLO_L2         = 0x0080,  ///<  If 1, the Kalman filter is using GLONASS L2 measurements
        GAL_E1         = 0x0100,  ///<  If 1, the Kalman filter is using Galileo E1 measurements
        GAL_E5         = 0x0200,  ///<  If 1, the Kalman filter is using Galileo E5 measurements
        GAL_E6         = 0x0400,  ///<  If 1, the Kalman filter is using Galileo E6 measurements
        BEI_B1         = 0x0800,  ///<  If 1, the Kalman filter is using Beidou B1 measurements (not enabled on GQ7 currently)
        BEI_B2         = 0x1000,  ///<  If 1, the Kalman filter is using Beidou B2 measurements (not enabled on GQ7 currently)
        BEI_B3         = 0x2000,  ///<  If 1, the Kalman filter is using Beidou B3 measurements (not available on the GQ7)
        NO_FIX         = 0x4000,  ///<  If 1, this GNSS module is reporting no position fix
        CONFIG_ERROR   = 0x8000,  ///<  If 1, there is likely an issue with the antenna offset for this GNSS module
        ALL            = 0xFFFF,
    };
    uint16_t value = NONE;
    
    constexpr GnssAidStatusFlags() : value(NONE) {}
    constexpr GnssAidStatusFlags(int val) : value((uint16_t)val) {}
    constexpr operator uint16_t() const { return value; }
    constexpr GnssAidStatusFlags& operator=(uint16_t val) { value = val; return *this; }
    constexpr GnssAidStatusFlags& operator=(int val) { value = uint16_t(val); return *this; }
    constexpr GnssAidStatusFlags& operator|=(uint16_t val) { return *this = value | val; }
    constexpr GnssAidStatusFlags& operator&=(uint16_t val) { return *this = value & val; }
    
    constexpr bool tightCoupling() const { return (value & TIGHT_COUPLING) > 0; }
    constexpr void tightCoupling(bool val) { value &= ~TIGHT_COUPLING; if(val) value |= TIGHT_COUPLING; }
    constexpr bool differential() const { return (value & DIFFERENTIAL) > 0; }
    constexpr void differential(bool val) { value &= ~DIFFERENTIAL; if(val) value |= DIFFERENTIAL; }
    constexpr bool integerFix() const { return (value & INTEGER_FIX) > 0; }
    constexpr void integerFix(bool val) { value &= ~INTEGER_FIX; if(val) value |= INTEGER_FIX; }
    constexpr bool gpsL1() const { return (value & GPS_L1) > 0; }
    constexpr void gpsL1(bool val) { value &= ~GPS_L1; if(val) value |= GPS_L1; }
    constexpr bool gpsL2() const { return (value & GPS_L2) > 0; }
    constexpr void gpsL2(bool val) { value &= ~GPS_L2; if(val) value |= GPS_L2; }
    constexpr bool gpsL5() const { return (value & GPS_L5) > 0; }
    constexpr void gpsL5(bool val) { value &= ~GPS_L5; if(val) value |= GPS_L5; }
    constexpr bool gloL1() const { return (value & GLO_L1) > 0; }
    constexpr void gloL1(bool val) { value &= ~GLO_L1; if(val) value |= GLO_L1; }
    constexpr bool gloL2() const { return (value & GLO_L2) > 0; }
    constexpr void gloL2(bool val) { value &= ~GLO_L2; if(val) value |= GLO_L2; }
    constexpr bool galE1() const { return (value & GAL_E1) > 0; }
    constexpr void galE1(bool val) { value &= ~GAL_E1; if(val) value |= GAL_E1; }
    constexpr bool galE5() const { return (value & GAL_E5) > 0; }
    constexpr void galE5(bool val) { value &= ~GAL_E5; if(val) value |= GAL_E5; }
    constexpr bool galE6() const { return (value & GAL_E6) > 0; }
    constexpr void galE6(bool val) { value &= ~GAL_E6; if(val) value |= GAL_E6; }
    constexpr bool beiB1() const { return (value & BEI_B1) > 0; }
    constexpr void beiB1(bool val) { value &= ~BEI_B1; if(val) value |= BEI_B1; }
    constexpr bool beiB2() const { return (value & BEI_B2) > 0; }
    constexpr void beiB2(bool val) { value &= ~BEI_B2; if(val) value |= BEI_B2; }
    constexpr bool beiB3() const { return (value & BEI_B3) > 0; }
    constexpr void beiB3(bool val) { value &= ~BEI_B3; if(val) value |= BEI_B3; }
    constexpr bool noFix() const { return (value & NO_FIX) > 0; }
    constexpr void noFix(bool val) { value &= ~NO_FIX; if(val) value |= NO_FIX; }
    constexpr bool configError() const { return (value & CONFIG_ERROR) > 0; }
    constexpr void configError(bool val) { value &= ~CONFIG_ERROR; if(val) value |= CONFIG_ERROR; }
    constexpr bool allSet() const { return value == ALL; }
    constexpr void setAll() { value |= ALL; }
};

////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
///@defgroup filter_position_llh_cpp  (0x82,0x01) Position Llh
/// Filter reported position in the WGS84 geodetic frame.
///
///@{

struct PositionLlh
{
    /// Parameters
    double latitude = 0; ///< [degrees]
    double longitude = 0; ///< [degrees]
    double ellipsoid_height = 0; ///< [meters]
    uint16_t valid_flags = 0; ///< 0 - Invalid, 1 - valid
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_POS_LLH;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "PositionLlh";
    static constexpr const char* DOC_NAME = "LLH Position";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(latitude,longitude,ellipsoid_height,valid_flags);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(latitude),std::ref(longitude),std::ref(ellipsoid_height),std::ref(valid_flags));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup filter_velocity_ned_cpp  (0x82,0x02) Velocity Ned
/// Filter reported velocity in the NED local-level frame.
///
///@{

struct VelocityNed
{
    /// Parameters
    float north = 0; ///< [meters/second]
    float east = 0; ///< [meters/second]
    float down = 0; ///< [meters/second]
    uint16_t valid_flags = 0; ///< 0 - Invalid, 1 - valid
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_VEL_NED;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "VelocityNed";
    static constexpr const char* DOC_NAME = "VelocityNed";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(north,east,down,valid_flags);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(north),std::ref(east),std::ref(down),std::ref(valid_flags));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup filter_attitude_quaternion_cpp  (0x82,0x03) Attitude Quaternion
/// 4x1 vector representation of the quaternion describing the orientation of the device with respect to the NED local-level frame.
/// This quaternion satisfies the following relationship:
/// 
/// EQSTART p^{veh} = q^{-1} p^{ned} q EQEND<br/>
/// 
/// Where:<br/>
/// EQSTART q = (q_w, q_x, q_y, q_z) EQEND is the quaternion describing the rotation. <br/>
/// EQSTART p^ned = (0, v^{ned}_x, v^{ned}_y, v^{ned}_z) EQEND and EQSTART v^{ned} EQEND is a 3-element vector expressed in the NED frame.<br/>
/// EQSTART p^veh = (0, v^{veh}_x, v^{veh}_y, v^{veh}_z) EQEND and EQSTART v^{veh} EQEND is a 3-element vector expressed in the vehicle frame.<br/>
///
///@{

struct AttitudeQuaternion
{
    /// Parameters
    Quatf q; ///< Quaternion elements EQSTART q = (q_w, q_x, q_y, q_z) EQEND
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_ATT_QUATERNION;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "AttitudeQuaternion";
    static constexpr const char* DOC_NAME = "AttitudeQuaternion";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(q[0],q[1],q[2],q[3],valid_flags);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(q[0]),std::ref(q[1]),std::ref(q[2]),std::ref(q[3]),std::ref(valid_flags));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup filter_attitude_dcm_cpp  (0x82,0x04) Attitude Dcm
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
/// The matrix elements are stored is row-major order: EQSTART M_{ned}^{veh} = \\begin{bmatrix} M_{11}, M_{12}, M_{13}, M_{21}, M_{22}, M_{23}, M_{31}, M_{32}, M_{33} \\end{bmatrix} EQEND
///
///@{

struct AttitudeDcm
{
    /// Parameters
    Matrix3f dcm; ///< Matrix elements in row-major order.
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_ATT_MATRIX;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "AttitudeDcm";
    static constexpr const char* DOC_NAME = "AttitudeDcm";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(dcm[0],dcm[1],dcm[2],dcm[3],dcm[4],dcm[5],dcm[6],dcm[7],dcm[8],valid_flags);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(dcm[0]),std::ref(dcm[1]),std::ref(dcm[2]),std::ref(dcm[3]),std::ref(dcm[4]),std::ref(dcm[5]),std::ref(dcm[6]),std::ref(dcm[7]),std::ref(dcm[8]),std::ref(valid_flags));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup filter_euler_angles_cpp  (0x82,0x05) Euler Angles
/// Filter reported Euler angles describing the orientation of the device with respect to the NED local-level frame.
/// The Euler angles are reported in 3-2-1 (Yaw-Pitch-Roll, AKA Aircraft) order.
///
///@{

struct EulerAngles
{
    /// Parameters
    float roll = 0; ///< [radians]
    float pitch = 0; ///< [radians]
    float yaw = 0; ///< [radians]
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_ATT_EULER_ANGLES;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "EulerAngles";
    static constexpr const char* DOC_NAME = "EulerAngles";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(roll,pitch,yaw,valid_flags);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(roll),std::ref(pitch),std::ref(yaw),std::ref(valid_flags));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup filter_gyro_bias_cpp  (0x82,0x06) Gyro Bias
/// Filter reported gyro bias expressed in the sensor frame.
///
///@{

struct GyroBias
{
    /// Parameters
    Vector3f bias; ///< (x, y, z) [radians/second]
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_GYRO_BIAS;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "GyroBias";
    static constexpr const char* DOC_NAME = "GyroBias";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(bias[0],bias[1],bias[2],valid_flags);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(bias[0]),std::ref(bias[1]),std::ref(bias[2]),std::ref(valid_flags));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup filter_accel_bias_cpp  (0x82,0x07) Accel Bias
/// Filter reported accelerometer bias expressed in the sensor frame.
///
///@{

struct AccelBias
{
    /// Parameters
    Vector3f bias; ///< (x, y, z) [meters/second^2]
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_ACCEL_BIAS;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "AccelBias";
    static constexpr const char* DOC_NAME = "AccelBias";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(bias[0],bias[1],bias[2],valid_flags);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(bias[0]),std::ref(bias[1]),std::ref(bias[2]),std::ref(valid_flags));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup filter_position_llh_uncertainty_cpp  (0x82,0x08) Position Llh Uncertainty
/// Filter reported 1-sigma position uncertainty in the NED local-level frame.
///
///@{

struct PositionLlhUncertainty
{
    /// Parameters
    float north = 0; ///< [meters]
    float east = 0; ///< [meters]
    float down = 0; ///< [meters]
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_POS_UNCERTAINTY;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "PositionLlhUncertainty";
    static constexpr const char* DOC_NAME = "LLH Position Uncertainty";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(north,east,down,valid_flags);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(north),std::ref(east),std::ref(down),std::ref(valid_flags));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup filter_velocity_ned_uncertainty_cpp  (0x82,0x09) Velocity Ned Uncertainty
/// Filter reported 1-sigma velocity uncertainties in the NED local-level frame.
///
///@{

struct VelocityNedUncertainty
{
    /// Parameters
    float north = 0; ///< [meters/second]
    float east = 0; ///< [meters/second]
    float down = 0; ///< [meters/second]
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_VEL_UNCERTAINTY;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "VelocityNedUncertainty";
    static constexpr const char* DOC_NAME = "NED Velocity Uncertainty";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(north,east,down,valid_flags);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(north),std::ref(east),std::ref(down),std::ref(valid_flags));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup filter_euler_angles_uncertainty_cpp  (0x82,0x0A) Euler Angles Uncertainty
/// Filter reported 1-sigma Euler angle uncertainties.
/// The uncertainties are reported in 3-2-1 (Yaw-Pitch-Roll, AKA Aircraft) order.
///
///@{

struct EulerAnglesUncertainty
{
    /// Parameters
    float roll = 0; ///< [radians]
    float pitch = 0; ///< [radians]
    float yaw = 0; ///< [radians]
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_ATT_UNCERTAINTY_EULER;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "EulerAnglesUncertainty";
    static constexpr const char* DOC_NAME = "EulerAnglesUncertainty";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(roll,pitch,yaw,valid_flags);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(roll),std::ref(pitch),std::ref(yaw),std::ref(valid_flags));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup filter_gyro_bias_uncertainty_cpp  (0x82,0x0B) Gyro Bias Uncertainty
/// Filter reported 1-sigma gyro bias uncertainties expressed in the sensor frame.
///
///@{

struct GyroBiasUncertainty
{
    /// Parameters
    Vector3f bias_uncert; ///< (x,y,z) [radians/sec]
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_GYRO_BIAS_UNCERTAINTY;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "GyroBiasUncertainty";
    static constexpr const char* DOC_NAME = "GyroBiasUncertainty";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(bias_uncert[0],bias_uncert[1],bias_uncert[2],valid_flags);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(bias_uncert[0]),std::ref(bias_uncert[1]),std::ref(bias_uncert[2]),std::ref(valid_flags));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup filter_accel_bias_uncertainty_cpp  (0x82,0x0C) Accel Bias Uncertainty
/// Filter reported 1-sigma accelerometer bias uncertainties expressed in the sensor frame.
///
///@{

struct AccelBiasUncertainty
{
    /// Parameters
    Vector3f bias_uncert; ///< (x,y,z) [meters/second^2]
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_ACCEL_BIAS_UNCERTAINTY;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "AccelBiasUncertainty";
    static constexpr const char* DOC_NAME = "AccelBiasUncertainty";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(bias_uncert[0],bias_uncert[1],bias_uncert[2],valid_flags);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(bias_uncert[0]),std::ref(bias_uncert[1]),std::ref(bias_uncert[2]),std::ref(valid_flags));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup filter_timestamp_cpp  (0x82,0x11) Timestamp
/// GPS timestamp of the Filter data
/// 
/// Should the PPS become unavailable, the device will revert to its internal clock, which will cause the reported time to drift from true GPS time.
/// Upon recovering from a PPS outage, the user should expect a jump in the reported GPS time due to the accumulation of internal clock error.
/// If synchronization to an external clock or onboard GNSS receiver (for products that have one) is disabled, this time is equivalent to internal system time.
/// 
/// Note: this data field may be deprecated in the future. The more flexible shared data field (0x82, 0xD3) should be used instead.
///
///@{

struct Timestamp
{
    /// Parameters
    double tow = 0; ///< GPS Time of Week [seconds]
    uint16_t week_number = 0; ///< GPS Week Number since 1980 [weeks]
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_FILTER_TIMESTAMP;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "Timestamp";
    static constexpr const char* DOC_NAME = "Timestamp";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(tow,week_number,valid_flags);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(tow),std::ref(week_number),std::ref(valid_flags));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup filter_status_cpp  (0x82,0x10) Status
/// Device-specific filter status indicators.
///
///@{

struct Status
{
    /// Parameters
    FilterMode filter_state = static_cast<FilterMode>(0); ///< Device-specific filter state.  Please consult the user manual for definition.
    FilterDynamicsMode dynamics_mode = static_cast<FilterDynamicsMode>(0); ///< Device-specific dynamics mode. Please consult the user manual for definition.
    FilterStatusFlags status_flags; ///< Device-specific status flags.  Please consult the user manual for definition.
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_FILTER_STATUS;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "Status";
    static constexpr const char* DOC_NAME = "Status";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(filter_state,dynamics_mode,status_flags);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(filter_state),std::ref(dynamics_mode),std::ref(status_flags));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup filter_linear_accel_cpp  (0x82,0x0D) Linear Accel
/// Filter-compensated linear acceleration expressed in the vehicle frame.
/// Note: The estimated gravity has been removed from this data leaving only linear acceleration.
///
///@{

struct LinearAccel
{
    /// Parameters
    Vector3f accel; ///< (x,y,z) [meters/second^2]
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_LINEAR_ACCELERATION;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "LinearAccel";
    static constexpr const char* DOC_NAME = "LinearAccel";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(accel[0],accel[1],accel[2],valid_flags);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(accel[0]),std::ref(accel[1]),std::ref(accel[2]),std::ref(valid_flags));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup filter_gravity_vector_cpp  (0x82,0x13) Gravity Vector
/// Filter reported gravity vector expressed in the vehicle frame.
///
///@{

struct GravityVector
{
    /// Parameters
    Vector3f gravity; ///< (x, y, z) [meters/second^2]
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_GRAVITY_VECTOR;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "GravityVector";
    static constexpr const char* DOC_NAME = "GravityVector";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(gravity[0],gravity[1],gravity[2],valid_flags);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(gravity[0]),std::ref(gravity[1]),std::ref(gravity[2]),std::ref(valid_flags));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup filter_comp_accel_cpp  (0x82,0x1C) Comp Accel
/// Filter-compensated acceleration expressed in the vehicle frame.
///
///@{

struct CompAccel
{
    /// Parameters
    Vector3f accel; ///< (x,y,z) [meters/second^2]
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_COMPENSATED_ACCELERATION;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "CompAccel";
    static constexpr const char* DOC_NAME = "Compensated Acceleration";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(accel[0],accel[1],accel[2],valid_flags);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(accel[0]),std::ref(accel[1]),std::ref(accel[2]),std::ref(valid_flags));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup filter_comp_angular_rate_cpp  (0x82,0x0E) Comp Angular Rate
/// Filter-compensated angular rate expressed in the vehicle frame.
///
///@{

struct CompAngularRate
{
    /// Parameters
    Vector3f gyro; ///< (x, y, z) [radians/second]
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_COMPENSATED_ANGULAR_RATE;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "CompAngularRate";
    static constexpr const char* DOC_NAME = "CompAngularRate";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(gyro[0],gyro[1],gyro[2],valid_flags);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(gyro[0]),std::ref(gyro[1]),std::ref(gyro[2]),std::ref(valid_flags));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup filter_quaternion_attitude_uncertainty_cpp  (0x82,0x12) Quaternion Attitude Uncertainty
/// Filter reported quaternion uncertainties.
///
///@{

struct QuaternionAttitudeUncertainty
{
    /// Parameters
    Quatf q; ///< [dimensionless]
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_ATT_UNCERTAINTY_QUATERNION;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "QuaternionAttitudeUncertainty";
    static constexpr const char* DOC_NAME = "QuaternionAttitudeUncertainty";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(q[0],q[1],q[2],q[3],valid_flags);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(q[0]),std::ref(q[1]),std::ref(q[2]),std::ref(q[3]),std::ref(valid_flags));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup filter_wgs84_gravity_mag_cpp  (0x82,0x0F) Wgs84 Gravity Mag
/// Filter reported WGS84 gravity magnitude.
///
///@{

struct Wgs84GravityMag
{
    /// Parameters
    float magnitude = 0; ///< [meters/second^2]
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_WGS84_GRAVITY;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "Wgs84GravityMag";
    static constexpr const char* DOC_NAME = "Wgs84GravityMag";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(magnitude,valid_flags);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(magnitude),std::ref(valid_flags));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup filter_heading_update_state_cpp  (0x82,0x14) Heading Update State
/// Filter reported heading update state.
/// 
/// Heading updates can be applied from the sources listed below.  Note, some of these sources may be combined.
/// The heading value is always relative to true north.
///
///@{

struct HeadingUpdateState
{
    enum class HeadingSource : uint16_t
    {
        NONE                 = 0,  ///<  
        MAGNETOMETER         = 1,  ///<  
        GNSS_VELOCITY_VECTOR = 2,  ///<  
        EXTERNAL             = 4,  ///<  
        DUAL_ANTENNA         = 8,  ///<  
    };
    
    /// Parameters
    float heading = 0; ///< [radians]
    float heading_1sigma = 0; ///< [radians]
    HeadingSource source = static_cast<HeadingSource>(0);
    uint16_t valid_flags = 0; ///< 1 if a valid heading update was received in 2 seconds, 0 otherwise.
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_HEADING_UPDATE_STATE;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "HeadingUpdateState";
    static constexpr const char* DOC_NAME = "HeadingUpdateState";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(heading,heading_1sigma,source,valid_flags);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(heading),std::ref(heading_1sigma),std::ref(source),std::ref(valid_flags));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup filter_magnetic_model_cpp  (0x82,0x15) Magnetic Model
/// The World Magnetic Model is used for this data. Please refer to the device user manual for the current version of the model.
/// A valid GNSS location is required for the model to be valid.
///
///@{

struct MagneticModel
{
    /// Parameters
    float intensity_north = 0; ///< [Gauss]
    float intensity_east = 0; ///< [Gauss]
    float intensity_down = 0; ///< [Gauss]
    float inclination = 0; ///< [radians]
    float declination = 0; ///< [radians]
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_MAGNETIC_MODEL;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "MagneticModel";
    static constexpr const char* DOC_NAME = "MagneticModel";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(intensity_north,intensity_east,intensity_down,inclination,declination,valid_flags);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(intensity_north),std::ref(intensity_east),std::ref(intensity_down),std::ref(inclination),std::ref(declination),std::ref(valid_flags));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup filter_accel_scale_factor_cpp  (0x82,0x17) Accel Scale Factor
/// Filter reported accelerometer scale factor expressed in the sensor frame.
///
///@{

struct AccelScaleFactor
{
    /// Parameters
    Vector3f scale_factor; ///< (x,y,z) [dimensionless]
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_ACCEL_SCALE_FACTOR;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "AccelScaleFactor";
    static constexpr const char* DOC_NAME = "AccelScaleFactor";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(scale_factor[0],scale_factor[1],scale_factor[2],valid_flags);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(scale_factor[0]),std::ref(scale_factor[1]),std::ref(scale_factor[2]),std::ref(valid_flags));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup filter_accel_scale_factor_uncertainty_cpp  (0x82,0x19) Accel Scale Factor Uncertainty
/// Filter reported 1-sigma accelerometer scale factor uncertainty expressed in the sensor frame.
///
///@{

struct AccelScaleFactorUncertainty
{
    /// Parameters
    Vector3f scale_factor_uncert; ///< (x,y,z) [dimensionless]
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_ACCEL_SCALE_FACTOR_UNCERTAINTY;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "AccelScaleFactorUncertainty";
    static constexpr const char* DOC_NAME = "AccelScaleFactorUncertainty";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(scale_factor_uncert[0],scale_factor_uncert[1],scale_factor_uncert[2],valid_flags);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(scale_factor_uncert[0]),std::ref(scale_factor_uncert[1]),std::ref(scale_factor_uncert[2]),std::ref(valid_flags));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup filter_gyro_scale_factor_cpp  (0x82,0x16) Gyro Scale Factor
/// Filter reported gyro scale factor expressed in the sensor frame.
///
///@{

struct GyroScaleFactor
{
    /// Parameters
    Vector3f scale_factor; ///< (x,y,z) [dimensionless]
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_GYRO_SCALE_FACTOR;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "GyroScaleFactor";
    static constexpr const char* DOC_NAME = "GyroScaleFactor";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(scale_factor[0],scale_factor[1],scale_factor[2],valid_flags);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(scale_factor[0]),std::ref(scale_factor[1]),std::ref(scale_factor[2]),std::ref(valid_flags));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup filter_gyro_scale_factor_uncertainty_cpp  (0x82,0x18) Gyro Scale Factor Uncertainty
/// Filter reported 1-sigma gyro scale factor uncertainty expressed in the sensor frame.
///
///@{

struct GyroScaleFactorUncertainty
{
    /// Parameters
    Vector3f scale_factor_uncert; ///< (x,y,z) [dimensionless]
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_GYRO_SCALE_FACTOR_UNCERTAINTY;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "GyroScaleFactorUncertainty";
    static constexpr const char* DOC_NAME = "GyroScaleFactorUncertainty";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(scale_factor_uncert[0],scale_factor_uncert[1],scale_factor_uncert[2],valid_flags);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(scale_factor_uncert[0]),std::ref(scale_factor_uncert[1]),std::ref(scale_factor_uncert[2]),std::ref(valid_flags));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup filter_mag_bias_cpp  (0x82,0x1A) Mag Bias
/// Filter reported magnetometer bias expressed in the sensor frame.
///
///@{

struct MagBias
{
    /// Parameters
    Vector3f bias; ///< (x,y,z) [Gauss]
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_MAG_BIAS;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "MagBias";
    static constexpr const char* DOC_NAME = "MagBias";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(bias[0],bias[1],bias[2],valid_flags);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(bias[0]),std::ref(bias[1]),std::ref(bias[2]),std::ref(valid_flags));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup filter_mag_bias_uncertainty_cpp  (0x82,0x1B) Mag Bias Uncertainty
/// Filter reported 1-sigma magnetometer bias uncertainty expressed in the sensor frame.
///
///@{

struct MagBiasUncertainty
{
    /// Parameters
    Vector3f bias_uncert; ///< (x,y,z) [Gauss]
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_MAG_BIAS_UNCERTAINTY;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "MagBiasUncertainty";
    static constexpr const char* DOC_NAME = "MagBiasUncertainty";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(bias_uncert[0],bias_uncert[1],bias_uncert[2],valid_flags);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(bias_uncert[0]),std::ref(bias_uncert[1]),std::ref(bias_uncert[2]),std::ref(valid_flags));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup filter_standard_atmosphere_cpp  (0x82,0x20) Standard Atmosphere
/// Filter reported standard atmosphere parameters.
/// 
/// The US 1976 Standard Atmosphere Model is used. A valid GNSS location is required for the model to be valid.
///
///@{

struct StandardAtmosphere
{
    /// Parameters
    float geometric_altitude = 0; ///< Input into calculation [meters]
    float geopotential_altitude = 0; ///< [meters]
    float standard_temperature = 0; ///< [degC]
    float standard_pressure = 0; ///< [milliBar]
    float standard_density = 0; ///< [kilogram/meter^3]
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_STANDARD_ATMOSPHERE_DATA;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "StandardAtmosphere";
    static constexpr const char* DOC_NAME = "StandardAtmosphere";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(geometric_altitude,geopotential_altitude,standard_temperature,standard_pressure,standard_density,valid_flags);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(geometric_altitude),std::ref(geopotential_altitude),std::ref(standard_temperature),std::ref(standard_pressure),std::ref(standard_density),std::ref(valid_flags));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup filter_pressure_altitude_cpp  (0x82,0x21) Pressure Altitude
/// Filter reported pressure altitude.
/// 
/// The US 1976 Standard Atmosphere Model is used to calculate the pressure altitude in meters.
/// A valid pressure sensor reading is required for the pressure altitude to be valid.
/// The minimum pressure reading supported by the model is 0.0037 mBar, corresponding to an altitude of 84,852 meters.
///
///@{

struct PressureAltitude
{
    /// Parameters
    float pressure_altitude = 0; ///< [meters]
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_PRESSURE_ALTITUDE_DATA;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "PressureAltitude";
    static constexpr const char* DOC_NAME = "PressureAltitude";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(pressure_altitude,valid_flags);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(pressure_altitude),std::ref(valid_flags));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup filter_density_altitude_cpp  (0x82,0x22) Density Altitude
///
///@{

struct DensityAltitude
{
    /// Parameters
    float density_altitude = 0; ///< m
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_DENSITY_ALTITUDE_DATA;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "DensityAltitude";
    static constexpr const char* DOC_NAME = "DensityAltitude";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(density_altitude,valid_flags);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(density_altitude),std::ref(valid_flags));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup filter_antenna_offset_correction_cpp  (0x82,0x30) Antenna Offset Correction
/// Filter reported GNSS antenna offset in vehicle frame.
/// 
/// This offset added to any previously stored offset vector to compensate for errors in definition.
///
///@{

struct AntennaOffsetCorrection
{
    /// Parameters
    Vector3f offset; ///< (x,y,z) [meters]
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_ANTENNA_OFFSET_CORRECTION;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "AntennaOffsetCorrection";
    static constexpr const char* DOC_NAME = "AntennaOffsetCorrection";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(offset[0],offset[1],offset[2],valid_flags);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(offset[0]),std::ref(offset[1]),std::ref(offset[2]),std::ref(valid_flags));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup filter_antenna_offset_correction_uncertainty_cpp  (0x82,0x31) Antenna Offset Correction Uncertainty
/// Filter reported 1-sigma GNSS antenna offset uncertainties in vehicle frame.
///
///@{

struct AntennaOffsetCorrectionUncertainty
{
    /// Parameters
    Vector3f offset_uncert; ///< (x,y,z) [meters]
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_ANTENNA_OFFSET_CORRECTION_UNCERTAINTY;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "AntennaOffsetCorrectionUncertainty";
    static constexpr const char* DOC_NAME = "AntennaOffsetCorrectionUncertainty";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(offset_uncert[0],offset_uncert[1],offset_uncert[2],valid_flags);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(offset_uncert[0]),std::ref(offset_uncert[1]),std::ref(offset_uncert[2]),std::ref(valid_flags));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup filter_multi_antenna_offset_correction_cpp  (0x82,0x34) Multi Antenna Offset Correction
/// Filter reported GNSS antenna offset in vehicle frame.
/// 
/// This offset added to any previously stored offset vector to compensate for errors in definition.
///
///@{

struct MultiAntennaOffsetCorrection
{
    /// Parameters
    uint8_t receiver_id = 0; ///< Receiver ID for the receiver to which the antenna is attached
    Vector3f offset; ///< (x,y,z) [meters]
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_MULTI_ANTENNA_OFFSET_CORRECTION;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "MultiAntennaOffsetCorrection";
    static constexpr const char* DOC_NAME = "MultiAntennaOffsetCorrection";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(receiver_id,offset[0],offset[1],offset[2],valid_flags);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(receiver_id),std::ref(offset[0]),std::ref(offset[1]),std::ref(offset[2]),std::ref(valid_flags));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup filter_multi_antenna_offset_correction_uncertainty_cpp  (0x82,0x35) Multi Antenna Offset Correction Uncertainty
/// Filter reported 1-sigma GNSS antenna offset uncertainties in vehicle frame.
///
///@{

struct MultiAntennaOffsetCorrectionUncertainty
{
    /// Parameters
    uint8_t receiver_id = 0; ///< Receiver ID for the receiver to which the antenna is attached
    Vector3f offset_uncert; ///< (x,y,z) [meters]
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_MULTI_ANTENNA_OFFSET_CORRECTION_UNCERTAINTY;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "MultiAntennaOffsetCorrectionUncertainty";
    static constexpr const char* DOC_NAME = "MultiAntennaOffsetCorrectionUncertainty";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(receiver_id,offset_uncert[0],offset_uncert[1],offset_uncert[2],valid_flags);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(receiver_id),std::ref(offset_uncert[0]),std::ref(offset_uncert[1]),std::ref(offset_uncert[2]),std::ref(valid_flags));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup filter_magnetometer_offset_cpp  (0x82,0x25) Magnetometer Offset
/// Filter reported magnetometer hard iron offset in sensor frame.
/// 
/// This offset added to any previously stored hard iron offset vector to compensate for magnetometer in-run bias errors.
///
///@{

struct MagnetometerOffset
{
    /// Parameters
    Vector3f hard_iron; ///< (x,y,z) [Gauss]
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_MAG_COMPENSATION_OFFSET;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "MagnetometerOffset";
    static constexpr const char* DOC_NAME = "MagnetometerOffset";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(hard_iron[0],hard_iron[1],hard_iron[2],valid_flags);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(hard_iron[0]),std::ref(hard_iron[1]),std::ref(hard_iron[2]),std::ref(valid_flags));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup filter_magnetometer_matrix_cpp  (0x82,0x26) Magnetometer Matrix
/// Filter reported magnetometer soft iron matrix in sensor frame.
/// 
/// This matrix is post multiplied to any previously stored soft iron matrix to compensate for magnetometer in-run errors.
///
///@{

struct MagnetometerMatrix
{
    /// Parameters
    Matrix3f soft_iron; ///< Row-major [dimensionless]
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_MAG_COMPENSATION_MATRIX;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "MagnetometerMatrix";
    static constexpr const char* DOC_NAME = "MagnetometerMatrix";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(soft_iron[0],soft_iron[1],soft_iron[2],soft_iron[3],soft_iron[4],soft_iron[5],soft_iron[6],soft_iron[7],soft_iron[8],valid_flags);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(soft_iron[0]),std::ref(soft_iron[1]),std::ref(soft_iron[2]),std::ref(soft_iron[3]),std::ref(soft_iron[4]),std::ref(soft_iron[5]),std::ref(soft_iron[6]),std::ref(soft_iron[7]),std::ref(soft_iron[8]),std::ref(valid_flags));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup filter_magnetometer_offset_uncertainty_cpp  (0x82,0x28) Magnetometer Offset Uncertainty
/// Filter reported 1-sigma magnetometer hard iron offset uncertainties in sensor frame.
///
///@{

struct MagnetometerOffsetUncertainty
{
    /// Parameters
    Vector3f hard_iron_uncertainty; ///< (x,y,z) [Gauss]
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_MAG_COMPENSATION_OFFSET_UNCERTAINTY;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "MagnetometerOffsetUncertainty";
    static constexpr const char* DOC_NAME = "MagnetometerOffsetUncertainty";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(hard_iron_uncertainty[0],hard_iron_uncertainty[1],hard_iron_uncertainty[2],valid_flags);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(hard_iron_uncertainty[0]),std::ref(hard_iron_uncertainty[1]),std::ref(hard_iron_uncertainty[2]),std::ref(valid_flags));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup filter_magnetometer_matrix_uncertainty_cpp  (0x82,0x29) Magnetometer Matrix Uncertainty
/// Filter reported 1-sigma magnetometer soft iron matrix uncertainties in sensor frame.
///
///@{

struct MagnetometerMatrixUncertainty
{
    /// Parameters
    Matrix3f soft_iron_uncertainty; ///< Row-major [dimensionless]
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_MAG_COMPENSATION_MATRIX_UNCERTAINTY;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "MagnetometerMatrixUncertainty";
    static constexpr const char* DOC_NAME = "MagnetometerMatrixUncertainty";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(soft_iron_uncertainty[0],soft_iron_uncertainty[1],soft_iron_uncertainty[2],soft_iron_uncertainty[3],soft_iron_uncertainty[4],soft_iron_uncertainty[5],soft_iron_uncertainty[6],soft_iron_uncertainty[7],soft_iron_uncertainty[8],valid_flags);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(soft_iron_uncertainty[0]),std::ref(soft_iron_uncertainty[1]),std::ref(soft_iron_uncertainty[2]),std::ref(soft_iron_uncertainty[3]),std::ref(soft_iron_uncertainty[4]),std::ref(soft_iron_uncertainty[5]),std::ref(soft_iron_uncertainty[6]),std::ref(soft_iron_uncertainty[7]),std::ref(soft_iron_uncertainty[8]),std::ref(valid_flags));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup filter_magnetometer_covariance_matrix_cpp  (0x82,0x2A) Magnetometer Covariance Matrix
///
///@{

struct MagnetometerCovarianceMatrix
{
    /// Parameters
    Matrix3f covariance;
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_MAG_COVARIANCE;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "MagnetometerCovarianceMatrix";
    static constexpr const char* DOC_NAME = "MagnetometerCovarianceMatrix";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(covariance[0],covariance[1],covariance[2],covariance[3],covariance[4],covariance[5],covariance[6],covariance[7],covariance[8],valid_flags);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(covariance[0]),std::ref(covariance[1]),std::ref(covariance[2]),std::ref(covariance[3]),std::ref(covariance[4]),std::ref(covariance[5]),std::ref(covariance[6]),std::ref(covariance[7]),std::ref(covariance[8]),std::ref(valid_flags));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup filter_magnetometer_residual_vector_cpp  (0x82,0x2C) Magnetometer Residual Vector
/// Filter reported magnetometer measurement residuals in vehicle frame.
///
///@{

struct MagnetometerResidualVector
{
    /// Parameters
    Vector3f residual; ///< (x,y,z) [Gauss]
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_MAG_RESIDUAL;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "MagnetometerResidualVector";
    static constexpr const char* DOC_NAME = "MagnetometerResidualVector";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(residual[0],residual[1],residual[2],valid_flags);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(residual[0]),std::ref(residual[1]),std::ref(residual[2]),std::ref(valid_flags));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup filter_clock_correction_cpp  (0x82,0x32) Clock Correction
/// Filter reported GNSS receiver clock error parameters.
///
///@{

struct ClockCorrection
{
    /// Parameters
    uint8_t receiver_id = 0; ///< 1, 2, etc.
    float bias = 0; ///< [seconds]
    float bias_drift = 0; ///< [seconds/second]
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_CLOCK_CORRECTION;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "ClockCorrection";
    static constexpr const char* DOC_NAME = "ClockCorrection";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(receiver_id,bias,bias_drift,valid_flags);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(receiver_id),std::ref(bias),std::ref(bias_drift),std::ref(valid_flags));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup filter_clock_correction_uncertainty_cpp  (0x82,0x33) Clock Correction Uncertainty
/// Filter reported 1-sigma GNSS receiver clock error parameters.
///
///@{

struct ClockCorrectionUncertainty
{
    /// Parameters
    uint8_t receiver_id = 0; ///< 1, 2, etc.
    float bias_uncertainty = 0; ///< [seconds]
    float bias_drift_uncertainty = 0; ///< [seconds/second]
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_CLOCK_CORRECTION_UNCERTAINTY;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "ClockCorrectionUncertainty";
    static constexpr const char* DOC_NAME = "ClockCorrectionUncertainty";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(receiver_id,bias_uncertainty,bias_drift_uncertainty,valid_flags);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(receiver_id),std::ref(bias_uncertainty),std::ref(bias_drift_uncertainty),std::ref(valid_flags));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup filter_gnss_pos_aid_status_cpp  (0x82,0x43) Gnss Pos Aid Status
/// Filter reported GNSS position aiding status
///
///@{

struct GnssPosAidStatus
{
    /// Parameters
    uint8_t receiver_id = 0;
    float time_of_week = 0; ///< Last GNSS aiding measurement time of week [seconds]
    GnssAidStatusFlags status; ///< Aiding measurement status bitfield
    uint8_t reserved[8] = {0};
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_GNSS_POS_AID_STATUS;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "GnssPosAidStatus";
    static constexpr const char* DOC_NAME = "GNSS Position Aiding Status";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(receiver_id,time_of_week,status,reserved);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(receiver_id),std::ref(time_of_week),std::ref(status),std::ref(reserved));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup filter_gnss_att_aid_status_cpp  (0x82,0x44) Gnss Att Aid Status
/// Filter reported dual antenna GNSS attitude aiding status
///
///@{

struct GnssAttAidStatus
{
    /// Parameters
    float time_of_week = 0; ///< Last valid aiding measurement time of week [seconds] [processed instead of measured?]
    GnssAidStatusFlags status; ///< Last valid aiding measurement status bitfield
    uint8_t reserved[8] = {0};
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_GNSS_ATT_AID_STATUS;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "GnssAttAidStatus";
    static constexpr const char* DOC_NAME = "GNSS Attitude Aiding Status";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(time_of_week,status,reserved);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(time_of_week),std::ref(status),std::ref(reserved));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup filter_head_aid_status_cpp  (0x82,0x45) Head Aid Status
/// Filter reported GNSS heading aiding status
///
///@{

struct HeadAidStatus
{
    enum class HeadingAidType : uint8_t
    {
        DUAL_ANTENNA     = 1,  ///<  
        EXTERNAL_MESSAGE = 2,  ///<  
    };
    
    /// Parameters
    float time_of_week = 0; ///< Last valid aiding measurement time of week [seconds] [processed instead of measured?]
    HeadingAidType type = static_cast<HeadingAidType>(0); ///< 1 - Dual antenna, 2 - External heading message (user supplied)
    float reserved[2] = {0};
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_HEAD_AID_STATUS;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "HeadAidStatus";
    static constexpr const char* DOC_NAME = "HeadAidStatus";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(time_of_week,type,reserved);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(time_of_week),std::ref(type),std::ref(reserved));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup filter_rel_pos_ned_cpp  (0x82,0x42) Rel Pos Ned
/// Filter reported relative position, with respect to configured reference position
///
///@{

struct RelPosNed
{
    /// Parameters
    Vector3d relative_position; ///< [meters, NED]
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_REL_POS_NED;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "RelPosNed";
    static constexpr const char* DOC_NAME = "NED Relative Position";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(relative_position[0],relative_position[1],relative_position[2],valid_flags);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(relative_position[0]),std::ref(relative_position[1]),std::ref(relative_position[2]),std::ref(valid_flags));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup filter_ecef_pos_cpp  (0x82,0x40) Ecef Pos
/// Filter reported ECEF position
///
///@{

struct EcefPos
{
    /// Parameters
    Vector3d position_ecef; ///< [meters, ECEF]
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 valid
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_ECEF_POS;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "EcefPos";
    static constexpr const char* DOC_NAME = "ECEF Position";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(position_ecef[0],position_ecef[1],position_ecef[2],valid_flags);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(position_ecef[0]),std::ref(position_ecef[1]),std::ref(position_ecef[2]),std::ref(valid_flags));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup filter_ecef_vel_cpp  (0x82,0x41) Ecef Vel
/// Filter reported ECEF velocity
///
///@{

struct EcefVel
{
    /// Parameters
    Vector3f velocity_ecef; ///< [meters/second, ECEF]
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 valid
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_ECEF_VEL;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "EcefVel";
    static constexpr const char* DOC_NAME = "ECEF Velocity";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(velocity_ecef[0],velocity_ecef[1],velocity_ecef[2],valid_flags);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(velocity_ecef[0]),std::ref(velocity_ecef[1]),std::ref(velocity_ecef[2]),std::ref(valid_flags));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup filter_ecef_pos_uncertainty_cpp  (0x82,0x36) Ecef Pos Uncertainty
/// Filter reported 1-sigma position uncertainty in the ECEF frame.
///
///@{

struct EcefPosUncertainty
{
    /// Parameters
    Vector3f pos_uncertainty; ///< [meters]
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_ECEF_POS_UNCERTAINTY;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "EcefPosUncertainty";
    static constexpr const char* DOC_NAME = "ECEF Position Uncertainty";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(pos_uncertainty[0],pos_uncertainty[1],pos_uncertainty[2],valid_flags);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(pos_uncertainty[0]),std::ref(pos_uncertainty[1]),std::ref(pos_uncertainty[2]),std::ref(valid_flags));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup filter_ecef_vel_uncertainty_cpp  (0x82,0x37) Ecef Vel Uncertainty
/// Filter reported 1-sigma velocity uncertainties in the ECEF frame.
///
///@{

struct EcefVelUncertainty
{
    /// Parameters
    Vector3f vel_uncertainty; ///< [meters/second]
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_ECEF_VEL_UNCERTAINTY;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "EcefVelUncertainty";
    static constexpr const char* DOC_NAME = "ECEF Velocity Uncertainty";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(vel_uncertainty[0],vel_uncertainty[1],vel_uncertainty[2],valid_flags);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(vel_uncertainty[0]),std::ref(vel_uncertainty[1]),std::ref(vel_uncertainty[2]),std::ref(valid_flags));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup filter_aiding_measurement_summary_cpp  (0x82,0x46) Aiding Measurement Summary
/// Filter reported aiding measurement summary. This message contains a summary of the specified aiding measurement over the previous measurement interval ending at the specified time.
///
///@{

struct AidingMeasurementSummary
{
    /// Parameters
    float time_of_week = 0; ///< [seconds]
    uint8_t source = 0;
    FilterAidingMeasurementType type = static_cast<FilterAidingMeasurementType>(0); ///< (see product manual for supported types) Note: values 0x20 and above correspond to commanded aiding measurements in the 0x13 Aiding command set.
    FilterMeasurementIndicator indicator;
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_AID_MEAS_SUMMARY;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "AidingMeasurementSummary";
    static constexpr const char* DOC_NAME = "AidingMeasurementSummary";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(time_of_week,source,type,indicator);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(time_of_week),std::ref(source),std::ref(type),std::ref(indicator));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup filter_odometer_scale_factor_error_cpp  (0x82,0x47) Odometer Scale Factor Error
/// Filter reported odometer scale factor error. The total scale factor estimate is the user indicated scale factor, plus the user indicated scale factor times the scale factor error.
///
///@{

struct OdometerScaleFactorError
{
    /// Parameters
    float scale_factor_error = 0; ///< [dimensionless]
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_ODOMETER_SCALE_FACTOR_ERROR;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "OdometerScaleFactorError";
    static constexpr const char* DOC_NAME = "Odometer Scale Factor Error";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(scale_factor_error,valid_flags);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(scale_factor_error),std::ref(valid_flags));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup filter_odometer_scale_factor_error_uncertainty_cpp  (0x82,0x48) Odometer Scale Factor Error Uncertainty
/// Filter reported odometer scale factor error uncertainty.
///
///@{

struct OdometerScaleFactorErrorUncertainty
{
    /// Parameters
    float scale_factor_error_uncertainty = 0; ///< [dimensionless]
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_ODOMETER_SCALE_FACTOR_ERROR_UNCERTAINTY;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "OdometerScaleFactorErrorUncertainty";
    static constexpr const char* DOC_NAME = "Odometer Scale Factor Error Uncertainty";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(scale_factor_error_uncertainty,valid_flags);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(scale_factor_error_uncertainty),std::ref(valid_flags));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup filter_gnss_dual_antenna_status_cpp  (0x82,0x49) Gnss Dual Antenna Status
/// Summary information for status of GNSS dual antenna heading estimate.
///
///@{

struct GnssDualAntennaStatus
{
    enum class FixType : uint8_t
    {
        FIX_NONE     = 0,  ///<  
        FIX_DA_FLOAT = 1,  ///<  
        FIX_DA_FIXED = 2,  ///<  
    };
    
    struct DualAntennaStatusFlags : Bitfield<DualAntennaStatusFlags>
    {
        typedef uint16_t Type;
        enum _enumType : uint16_t
        {
            NONE                  = 0x0000,
            RCV_1_DATA_VALID      = 0x0001,  ///<  
            RCV_2_DATA_VALID      = 0x0002,  ///<  
            ANTENNA_OFFSETS_VALID = 0x0004,  ///<  
            ALL                   = 0x0007,
        };
        uint16_t value = NONE;
        
        constexpr DualAntennaStatusFlags() : value(NONE) {}
        constexpr DualAntennaStatusFlags(int val) : value((uint16_t)val) {}
        constexpr operator uint16_t() const { return value; }
        constexpr DualAntennaStatusFlags& operator=(uint16_t val) { value = val; return *this; }
        constexpr DualAntennaStatusFlags& operator=(int val) { value = uint16_t(val); return *this; }
        constexpr DualAntennaStatusFlags& operator|=(uint16_t val) { return *this = value | val; }
        constexpr DualAntennaStatusFlags& operator&=(uint16_t val) { return *this = value & val; }
        
        constexpr bool rcv1DataValid() const { return (value & RCV_1_DATA_VALID) > 0; }
        constexpr void rcv1DataValid(bool val) { value &= ~RCV_1_DATA_VALID; if(val) value |= RCV_1_DATA_VALID; }
        constexpr bool rcv2DataValid() const { return (value & RCV_2_DATA_VALID) > 0; }
        constexpr void rcv2DataValid(bool val) { value &= ~RCV_2_DATA_VALID; if(val) value |= RCV_2_DATA_VALID; }
        constexpr bool antennaOffsetsValid() const { return (value & ANTENNA_OFFSETS_VALID) > 0; }
        constexpr void antennaOffsetsValid(bool val) { value &= ~ANTENNA_OFFSETS_VALID; if(val) value |= ANTENNA_OFFSETS_VALID; }
        constexpr bool allSet() const { return value == ALL; }
        constexpr void setAll() { value |= ALL; }
    };
    /// Parameters
    float time_of_week = 0; ///< Last dual-antenna GNSS aiding measurement time of week [seconds]
    float heading = 0; ///< [radians]
    float heading_unc = 0; ///< [radians]
    FixType fix_type = static_cast<FixType>(0); ///< Fix type indicator
    DualAntennaStatusFlags status_flags;
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_GNSS_DUAL_ANTENNA_STATUS;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "GnssDualAntennaStatus";
    static constexpr const char* DOC_NAME = "GNSS Dual Antenna Status";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(time_of_week,heading,heading_unc,fix_type,status_flags,valid_flags);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(time_of_week),std::ref(heading),std::ref(heading_unc),std::ref(fix_type),std::ref(status_flags),std::ref(valid_flags));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup filter_aiding_frame_config_error_cpp  (0x82,0x50) Aiding Frame Config Error
/// Filter reported aiding source frame configuration error
/// 
/// These estimates are used to compensate for small errors to the user-supplied aiding frame configurations (set with (0x13, 0x01) command ).
///
///@{

struct AidingFrameConfigError
{
    /// Parameters
    uint8_t frame_id = 0; ///< Frame ID for the receiver to which the antenna is attached
    Vector3f translation; ///< Translation config X, Y, and Z (m).
    Quatf attitude; ///< Attitude quaternion
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_FRAME_CONFIG_ERROR;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "AidingFrameConfigError";
    static constexpr const char* DOC_NAME = "Aiding Frame Configuration Error";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(frame_id,translation[0],translation[1],translation[2],attitude[0],attitude[1],attitude[2],attitude[3]);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(frame_id),std::ref(translation[0]),std::ref(translation[1]),std::ref(translation[2]),std::ref(attitude[0]),std::ref(attitude[1]),std::ref(attitude[2]),std::ref(attitude[3]));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup filter_aiding_frame_config_error_uncertainty_cpp  (0x82,0x51) Aiding Frame Config Error Uncertainty
/// Filter reported aiding source frame configuration error uncertainty
/// 
/// These estimates are used to compensate for small errors to the user-supplied aiding frame configurations (set with (0x13, 0x01) command ).
///
///@{

struct AidingFrameConfigErrorUncertainty
{
    /// Parameters
    uint8_t frame_id = 0; ///< Frame ID for the receiver to which the antenna is attached
    Vector3f translation_unc; ///< Translation uncertaint X, Y, and Z (m).
    Vector3f attitude_unc; ///< Attitude uncertainty, X, Y, and Z (radians).
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_FRAME_CONFIG_ERROR_UNCERTAINTY;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "AidingFrameConfigErrorUncertainty";
    static constexpr const char* DOC_NAME = "Aiding Frame Configuration Error Uncertainty";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(frame_id,translation_unc[0],translation_unc[1],translation_unc[2],attitude_unc[0],attitude_unc[1],attitude_unc[2]);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(frame_id),std::ref(translation_unc[0]),std::ref(translation_unc[1]),std::ref(translation_unc[2]),std::ref(attitude_unc[0]),std::ref(attitude_unc[1]),std::ref(attitude_unc[2]));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///

///@}
///@}
///
////////////////////////////////////////////////////////////////////////////////
} // namespace data_filter
} // namespace mip

