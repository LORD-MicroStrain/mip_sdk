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

namespace data_sensor {

////////////////////////////////////////////////////////////////////////////////
///@addtogroup MipData
///@{
///@defgroup sensor_data_cpp  SENSORData
///
///@{

////////////////////////////////////////////////////////////////////////////////
// Descriptors
////////////////////////////////////////////////////////////////////////////////

enum 
{
    DESCRIPTOR_SET                      = 0x80,
    
    DATA_ACCEL_RAW                      = 0x01,
    DATA_GYRO_RAW                       = 0x02,
    DATA_MAG_RAW                        = 0x03,
    DATA_ACCEL_SCALED                   = 0x04,
    DATA_GYRO_SCALED                    = 0x05,
    DATA_MAG_SCALED                     = 0x06,
    DATA_DELTA_THETA                    = 0x07,
    DATA_DELTA_VELOCITY                 = 0x08,
    DATA_COMP_ORIENTATION_MATRIX        = 0x09,
    DATA_COMP_QUATERNION                = 0x0A,
    DATA_COMP_ORIENTATION_UPDATE_MATRIX = 0x0B,
    DATA_COMP_EULER_ANGLES              = 0x0C,
    DATA_TEMPERATURE_RAW                = 0x0D,
    DATA_TIME_STAMP_INTERNAL            = 0x0E,
    DATA_TIME_STAMP_PPS                 = 0x0F,
    DATA_STAB_MAG                       = 0x10,
    DATA_STAB_ACCEL                     = 0x11,
    DATA_TIME_STAMP_GPS                 = 0x12,
    DATA_TEMPERATURE_ABS                = 0x14,
    DATA_RAW_CLIP_DATA                  = 0x15,
    DATA_PRESSURE_RAW                   = 0x16,
    DATA_PRESSURE_SCALED                = 0x17,
    DATA_OVERRANGE_STATUS               = 0x18,
    DATA_ODOMETER                       = 0x40,
    
    MIP_DATA_DESC_ASPP                  = 0x81,
    MIP_DATA_DESC_GXSB                  = 0x82,
};

////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_raw_accel  None
/// Three element vector representing the sensed acceleration.
/// This quantity is temperature compensated and expressed in the sensor body frame.
///
///@{

struct RawAccel
{
    static const uint8_t DESCRIPTOR_SET = ::mip::data_sensor::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::data_sensor::DATA_ACCEL_RAW;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    float raw_accel[3];
    
};
void insert(Serializer& serializer, const RawAccel& self);
void extract(Serializer& serializer, RawAccel& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_raw_gyro  None
/// Three element vector representing the sensed angular rate.
/// This quantity is temperature compensated and expressed in the sensor body frame.
///
///@{

struct RawGyro
{
    static const uint8_t DESCRIPTOR_SET = ::mip::data_sensor::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::data_sensor::DATA_GYRO_RAW;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    float raw_gyro[3];
    
};
void insert(Serializer& serializer, const RawGyro& self);
void extract(Serializer& serializer, RawGyro& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_raw_mag  None
/// Three element vector representing the sensed magnetic field.
/// This quantity is temperature compensated and expressed in the vehicle frame.
///
///@{

struct RawMag
{
    static const uint8_t DESCRIPTOR_SET = ::mip::data_sensor::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::data_sensor::DATA_MAG_RAW;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    float raw_mag[3];
    
};
void insert(Serializer& serializer, const RawMag& self);
void extract(Serializer& serializer, RawMag& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_raw_pressure  None
/// Scalar value representing the sensed ambient pressure.
/// This quantity is temperature compensated.
///
///@{

struct RawPressure
{
    static const uint8_t DESCRIPTOR_SET = ::mip::data_sensor::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::data_sensor::DATA_PRESSURE_RAW;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    float raw_pressure;
    
};
void insert(Serializer& serializer, const RawPressure& self);
void extract(Serializer& serializer, RawPressure& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_scaled_accel  None
/// 3-element vector representing the sensed acceleration.
/// This quantity is temperature compensated and expressed in the vehicle frame.
///
///@{

struct ScaledAccel
{
    static const uint8_t DESCRIPTOR_SET = ::mip::data_sensor::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::data_sensor::DATA_ACCEL_SCALED;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    float scaled_accel[3];
    
};
void insert(Serializer& serializer, const ScaledAccel& self);
void extract(Serializer& serializer, ScaledAccel& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_scaled_gyro  None
/// 3-element vector representing the sensed angular rate.
/// This quantity is temperature compensated and expressed in the vehicle frame.
///
///@{

struct ScaledGyro
{
    static const uint8_t DESCRIPTOR_SET = ::mip::data_sensor::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::data_sensor::DATA_GYRO_SCALED;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    float scaled_gyro[3];
    
};
void insert(Serializer& serializer, const ScaledGyro& self);
void extract(Serializer& serializer, ScaledGyro& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_scaled_mag  None
/// 3-element vector representing the sensed magnetic field.
/// This quantity is temperature compensated and expressed in the vehicle frame.
///
///@{

struct ScaledMag
{
    static const uint8_t DESCRIPTOR_SET = ::mip::data_sensor::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::data_sensor::DATA_MAG_SCALED;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    float scaled_mag[3];
    
};
void insert(Serializer& serializer, const ScaledMag& self);
void extract(Serializer& serializer, ScaledMag& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_scaled_pressure  None
/// Scalar value representing the sensed ambient pressure.
///
///@{

struct ScaledPressure
{
    static const uint8_t DESCRIPTOR_SET = ::mip::data_sensor::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::data_sensor::DATA_PRESSURE_SCALED;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    float scaled_pressure;
    
};
void insert(Serializer& serializer, const ScaledPressure& self);
void extract(Serializer& serializer, ScaledPressure& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_delta_theta  None
/// 3-element vector representing the time integral of angular rate.
/// This quantity is the integral of sensed angular rate over the period set by the IMU message format.  It is expressed in the vehicle frame.
///
///@{

struct DeltaTheta
{
    static const uint8_t DESCRIPTOR_SET = ::mip::data_sensor::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::data_sensor::DATA_DELTA_THETA;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    float delta_theta[3];
    
};
void insert(Serializer& serializer, const DeltaTheta& self);
void extract(Serializer& serializer, DeltaTheta& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_delta_velocity  None
/// 3-element vector representing the time integral of acceleration.
/// This quantity is the integral of sensed acceleration over the period set by the IMU message format.  It is expressed in the vehicle frame.
///
///@{

struct DeltaVelocity
{
    static const uint8_t DESCRIPTOR_SET = ::mip::data_sensor::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::data_sensor::DATA_DELTA_VELOCITY;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    float delta_velocity[3];
    
};
void insert(Serializer& serializer, const DeltaVelocity& self);
void extract(Serializer& serializer, DeltaVelocity& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_comp_orientation_matrix  Complementary Filter Orientation Matrix
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

struct CompOrientationMatrix
{
    static const uint8_t DESCRIPTOR_SET = ::mip::data_sensor::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::data_sensor::DATA_COMP_ORIENTATION_MATRIX;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    float m[9];
    
};
void insert(Serializer& serializer, const CompOrientationMatrix& self);
void extract(Serializer& serializer, CompOrientationMatrix& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_comp_quaternion  Complementary Filter Quaternion
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

struct CompQuaternion
{
    static const uint8_t DESCRIPTOR_SET = ::mip::data_sensor::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::data_sensor::DATA_COMP_QUATERNION;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    float q[4];
    
};
void insert(Serializer& serializer, const CompQuaternion& self);
void extract(Serializer& serializer, CompQuaternion& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_comp_euler_angles  Complementary Filter Euler Angles
/// Euler angles describing the orientation of the device with respect to the NED local-level frame.
/// The Euler angles are reported in 3-2-1 (Yaw-Pitch-Roll, AKA Aircraft) order.
///
///@{

struct CompEulerAngles
{
    static const uint8_t DESCRIPTOR_SET = ::mip::data_sensor::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::data_sensor::DATA_COMP_EULER_ANGLES;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    float roll;
    float pitch;
    float yaw;
    
};
void insert(Serializer& serializer, const CompEulerAngles& self);
void extract(Serializer& serializer, CompEulerAngles& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_comp_orientation_update_matrix  Complementary Filter Orientation Update Matrix
/// DEPRECATED!
///
///@{

struct CompOrientationUpdateMatrix
{
    static const uint8_t DESCRIPTOR_SET = ::mip::data_sensor::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::data_sensor::DATA_COMP_ORIENTATION_UPDATE_MATRIX;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    float m[9];
    
};
void insert(Serializer& serializer, const CompOrientationUpdateMatrix& self);
void extract(Serializer& serializer, CompOrientationUpdateMatrix& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_orientation_raw_temp  None
/// DEPRECATED!
///
///@{

struct OrientationRawTemp
{
    static const uint8_t DESCRIPTOR_SET = ::mip::data_sensor::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::data_sensor::DATA_TEMPERATURE_RAW;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    uint16_t raw_temp[4];
    
};
void insert(Serializer& serializer, const OrientationRawTemp& self);
void extract(Serializer& serializer, OrientationRawTemp& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_internal_timestamp  None
/// DEPRECATED!
///
///@{

struct InternalTimestamp
{
    static const uint8_t DESCRIPTOR_SET = ::mip::data_sensor::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::data_sensor::DATA_TIME_STAMP_INTERNAL;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    uint32_t counts;
    
};
void insert(Serializer& serializer, const InternalTimestamp& self);
void extract(Serializer& serializer, InternalTimestamp& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_pps_timestamp  PPS Timestamp
/// DEPRECATED!
///
///@{

struct PpsTimestamp
{
    static const uint8_t DESCRIPTOR_SET = ::mip::data_sensor::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::data_sensor::DATA_TIME_STAMP_PPS;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    uint32_t seconds;
    uint32_t useconds;
    
};
void insert(Serializer& serializer, const PpsTimestamp& self);
void extract(Serializer& serializer, PpsTimestamp& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_gps_timestamp  None
/// GPS timestamp of the SENSOR data
/// 
/// Should the PPS become unavailable, the device will revert to its internal clock, which will cause the reported time to drift from true GPS time.
/// Upon recovering from a PPS outage, the user should expect a jump in the reported GPS time due to the accumulation of internal clock error.
/// If synchronization to an external clock or onboard GNSS receiver (for products that have one) is disabled, this time is equivalent to internal system time.
/// 
/// Note: this data field may be deprecrated in the future. The more flexible shared data field (0x80, 0xD3) should be used instead.
///
///@{

struct GpsTimestamp
{
    static const uint8_t DESCRIPTOR_SET = ::mip::data_sensor::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::data_sensor::DATA_TIME_STAMP_GPS;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    struct ValidFlags : Bitfield<ValidFlags>
    {
        enum _enumType : uint16_t
        {
            NONE              = 0x0000,
            PPS_VALID         = 0x0001,
            TIME_REFRESH      = 0x0002,
            TIME_INITIALIZED  = 0x0004,
            TOW_VALID         = 0x0008,
            WEEK_NUMBER_VALID = 0x0010,
        };
        uint16_t value = NONE;
        
        ValidFlags() : value(NONE) {}
        ValidFlags(int val) : value(val) {}
        operator uint16_t() const { return value; }
        ValidFlags& operator=(uint16_t val) { value = val; return *this; }
        ValidFlags& operator=(int val) { value = val; return *this; }
        ValidFlags& operator|=(uint16_t val) { return *this = value | val; }
        ValidFlags& operator&=(uint16_t val) { return *this = value & val; }
    };
    
    double tow;
    uint16_t week_number;
    ValidFlags valid_flags;
    
};
void insert(Serializer& serializer, const GpsTimestamp& self);
void extract(Serializer& serializer, GpsTimestamp& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_temperature_abs  Temperature Statistics
/// SENSOR reported temperature statistics
/// 
/// Temperature may originate from the MEMS sensors, or be calculated in combination with board temperature sensors.
/// All quantities are calculated with respect to the last power on or reset, whichever is later.
/// 
///
///@{

struct TemperatureAbs
{
    static const uint8_t DESCRIPTOR_SET = ::mip::data_sensor::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::data_sensor::DATA_TEMPERATURE_ABS;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    float min_temp;
    float max_temp;
    float mean_temp;
    
};
void insert(Serializer& serializer, const TemperatureAbs& self);
void extract(Serializer& serializer, TemperatureAbs& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_up_vector  None
/// Gyro-stabilized 3-element vector representing the complementary filter's estimated vertical direction.
/// This quantity is expressed in the vehicle frame.
/// 
/// This quantity is sensitive to non-gravitational accelerations, which may cause notable deviations from the true vertical direction.
/// 
/// For legacy reasons, this vector is the inverse of the gravity vector.
/// 
///
///@{

struct UpVector
{
    static const uint8_t DESCRIPTOR_SET = ::mip::data_sensor::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::data_sensor::DATA_STAB_ACCEL;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    float up[3];
    
};
void insert(Serializer& serializer, const UpVector& self);
void extract(Serializer& serializer, UpVector& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_north_vector  None
/// Gyro-stabilized 3-element vector representing the complementary filter's estimate of magnetic north.
/// This quantity is expressed in the vehicle frame.
/// 
/// This quantity is sensitive to local magnetic field perturbations, which may cause notable deviations from true magnetic north.
///
///@{

struct NorthVector
{
    static const uint8_t DESCRIPTOR_SET = ::mip::data_sensor::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::data_sensor::DATA_STAB_MAG;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    float north[3];
    
};
void insert(Serializer& serializer, const NorthVector& self);
void extract(Serializer& serializer, NorthVector& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_overrange_status  None
///
///@{

struct OverrangeStatus
{
    static const uint8_t DESCRIPTOR_SET = ::mip::data_sensor::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::data_sensor::DATA_OVERRANGE_STATUS;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    struct Status : Bitfield<Status>
    {
        enum _enumType : uint16_t
        {
            NONE    = 0x0000,
            ACCEL_X = 0x0001,
            ACCEL_Y = 0x0002,
            ACCEL_Z = 0x0004,
            GYRO_X  = 0x0010,
            GYRO_Y  = 0x0020,
            GYRO_Z  = 0x0040,
            MAG_X   = 0x0100,
            MAG_Y   = 0x0200,
            MAG_Z   = 0x0400,
            PRESS   = 0x1000,
        };
        uint16_t value = NONE;
        
        Status() : value(NONE) {}
        Status(int val) : value(val) {}
        operator uint16_t() const { return value; }
        Status& operator=(uint16_t val) { value = val; return *this; }
        Status& operator=(int val) { value = val; return *this; }
        Status& operator|=(uint16_t val) { return *this = value | val; }
        Status& operator&=(uint16_t val) { return *this = value & val; }
    };
    
    Status status;
    
};
void insert(Serializer& serializer, const OverrangeStatus& self);
void extract(Serializer& serializer, OverrangeStatus& self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_odometer_data  None
///
///@{

struct OdometerData
{
    static const uint8_t DESCRIPTOR_SET = ::mip::data_sensor::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::data_sensor::DATA_ODOMETER;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    float speed;
    float uncertainty;
    uint16_t valid_flags;
    
};
void insert(Serializer& serializer, const OdometerData& self);
void extract(Serializer& serializer, OdometerData& self);

///@}
///

///@}
///@}
///
////////////////////////////////////////////////////////////////////////////////
} // namespace data_sensor
} // namespace mip

