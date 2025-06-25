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

namespace data_sensor {

////////////////////////////////////////////////////////////////////////////////
///@addtogroup MipData_cpp
///@{
///@defgroup sensor_data_cpp  Sensor Data
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
///@defgroup sensor_raw_accel_cpp  (0x80,0x01) Raw Accel
/// Three element vector representing the sensed acceleration.
/// This quantity is temperature compensated and expressed in the sensor body frame.
///
///@{

struct RawAccel
{
    /// Parameters
    Vector3f raw_accel; ///< Native sensor counts
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_sensor::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_sensor::DATA_ACCEL_RAW;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "RawAccel";
    static constexpr const char* DOC_NAME = "RawAccel";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(raw_accel[0],raw_accel[1],raw_accel[2]);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(raw_accel[0]),std::ref(raw_accel[1]),std::ref(raw_accel[2]));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup sensor_raw_gyro_cpp  (0x80,0x02) Raw Gyro
/// Three element vector representing the sensed angular rate.
/// This quantity is temperature compensated and expressed in the sensor body frame.
///
///@{

struct RawGyro
{
    /// Parameters
    Vector3f raw_gyro; ///< Native sensor counts
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_sensor::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_sensor::DATA_GYRO_RAW;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "RawGyro";
    static constexpr const char* DOC_NAME = "RawGyro";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(raw_gyro[0],raw_gyro[1],raw_gyro[2]);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(raw_gyro[0]),std::ref(raw_gyro[1]),std::ref(raw_gyro[2]));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup sensor_raw_mag_cpp  (0x80,0x03) Raw Mag
/// Three element vector representing the sensed magnetic field.
/// This quantity is temperature compensated and expressed in the vehicle frame.
///
///@{

struct RawMag
{
    /// Parameters
    Vector3f raw_mag; ///< Native sensor counts
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_sensor::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_sensor::DATA_MAG_RAW;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "RawMag";
    static constexpr const char* DOC_NAME = "RawMag";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(raw_mag[0],raw_mag[1],raw_mag[2]);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(raw_mag[0]),std::ref(raw_mag[1]),std::ref(raw_mag[2]));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup sensor_raw_pressure_cpp  (0x80,0x16) Raw Pressure
/// Scalar value representing the sensed ambient pressure.
/// This quantity is temperature compensated.
///
///@{

struct RawPressure
{
    /// Parameters
    float raw_pressure = 0; ///< Native sensor counts
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_sensor::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_sensor::DATA_PRESSURE_RAW;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "RawPressure";
    static constexpr const char* DOC_NAME = "RawPressure";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(raw_pressure);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(raw_pressure));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup sensor_scaled_accel_cpp  (0x80,0x04) Scaled Accel
/// 3-element vector representing the sensed acceleration.
/// This quantity is temperature compensated and expressed in the vehicle frame.
///
///@{

struct ScaledAccel
{
    /// Parameters
    Vector3f scaled_accel; ///< (x, y, z)[g]
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_sensor::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_sensor::DATA_ACCEL_SCALED;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "ScaledAccel";
    static constexpr const char* DOC_NAME = "ScaledAccel";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(scaled_accel[0],scaled_accel[1],scaled_accel[2]);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(scaled_accel[0]),std::ref(scaled_accel[1]),std::ref(scaled_accel[2]));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup sensor_scaled_gyro_cpp  (0x80,0x05) Scaled Gyro
/// 3-element vector representing the sensed angular rate.
/// This quantity is temperature compensated and expressed in the vehicle frame.
///
///@{

struct ScaledGyro
{
    /// Parameters
    Vector3f scaled_gyro; ///< (x, y, z) [radians/second]
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_sensor::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_sensor::DATA_GYRO_SCALED;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "ScaledGyro";
    static constexpr const char* DOC_NAME = "ScaledGyro";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(scaled_gyro[0],scaled_gyro[1],scaled_gyro[2]);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(scaled_gyro[0]),std::ref(scaled_gyro[1]),std::ref(scaled_gyro[2]));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup sensor_scaled_mag_cpp  (0x80,0x06) Scaled Mag
/// 3-element vector representing the sensed magnetic field.
/// This quantity is temperature compensated and expressed in the vehicle frame.
///
///@{

struct ScaledMag
{
    /// Parameters
    Vector3f scaled_mag; ///< (x, y, z) [Gauss]
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_sensor::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_sensor::DATA_MAG_SCALED;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "ScaledMag";
    static constexpr const char* DOC_NAME = "ScaledMag";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(scaled_mag[0],scaled_mag[1],scaled_mag[2]);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(scaled_mag[0]),std::ref(scaled_mag[1]),std::ref(scaled_mag[2]));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup sensor_scaled_pressure_cpp  (0x80,0x17) Scaled Pressure
/// Scalar value representing the sensed ambient pressure.
///
///@{

struct ScaledPressure
{
    /// Parameters
    float scaled_pressure = 0; ///< [mBar]
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_sensor::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_sensor::DATA_PRESSURE_SCALED;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "ScaledPressure";
    static constexpr const char* DOC_NAME = "ScaledPressure";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(scaled_pressure);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(scaled_pressure));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup sensor_delta_theta_cpp  (0x80,0x07) Delta Theta
/// 3-element vector representing the time integral of angular rate.
/// This quantity is the integral of sensed angular rate over the period set by the IMU message format.  It is expressed in the vehicle frame.
///
///@{

struct DeltaTheta
{
    /// Parameters
    Vector3f delta_theta; ///< (x, y, z) [radians]
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_sensor::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_sensor::DATA_DELTA_THETA;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "DeltaTheta";
    static constexpr const char* DOC_NAME = "DeltaTheta";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(delta_theta[0],delta_theta[1],delta_theta[2]);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(delta_theta[0]),std::ref(delta_theta[1]),std::ref(delta_theta[2]));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup sensor_delta_velocity_cpp  (0x80,0x08) Delta Velocity
/// 3-element vector representing the time integral of acceleration.
/// This quantity is the integral of sensed acceleration over the period set by the IMU message format.  It is expressed in the vehicle frame.
///
///@{

struct DeltaVelocity
{
    /// Parameters
    Vector3f delta_velocity; ///< (x, y, z) [g*sec]
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_sensor::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_sensor::DATA_DELTA_VELOCITY;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "DeltaVelocity";
    static constexpr const char* DOC_NAME = "DeltaVelocity";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(delta_velocity[0],delta_velocity[1],delta_velocity[2]);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(delta_velocity[0]),std::ref(delta_velocity[1]),std::ref(delta_velocity[2]));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup sensor_comp_orientation_matrix_cpp  (0x80,0x09) Comp Orientation Matrix
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
/// The matrix elements are stored is row-major order: EQSTART M = \\begin{bmatrix} M_{11}, M_{12}, M_{13}, M_{21}, M_{22}, M_{23}, M_{31}, M_{32}, M_{33} \\end{bmatrix} EQEND
///
///@{

struct CompOrientationMatrix
{
    /// Parameters
    Matrix3f m; ///< Matrix elements in row-major order.
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_sensor::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_sensor::DATA_COMP_ORIENTATION_MATRIX;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "CompOrientationMatrix";
    static constexpr const char* DOC_NAME = "Complementary Filter Orientation Matrix";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(m[0],m[1],m[2],m[3],m[4],m[5],m[6],m[7],m[8]);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(m[0]),std::ref(m[1]),std::ref(m[2]),std::ref(m[3]),std::ref(m[4]),std::ref(m[5]),std::ref(m[6]),std::ref(m[7]),std::ref(m[8]));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup sensor_comp_quaternion_cpp  (0x80,0x0A) Comp Quaternion
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

struct CompQuaternion
{
    /// Parameters
    Quatf q; ///< Quaternion elements EQSTART q = (q_w, q_x, q_y, q_z) EQEND
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_sensor::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_sensor::DATA_COMP_QUATERNION;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "CompQuaternion";
    static constexpr const char* DOC_NAME = "Complementary Filter Quaternion";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(q[0],q[1],q[2],q[3]);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(q[0]),std::ref(q[1]),std::ref(q[2]),std::ref(q[3]));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup sensor_comp_euler_angles_cpp  (0x80,0x0C) Comp Euler Angles
/// Euler angles describing the orientation of the device with respect to the NED local-level frame.
/// The Euler angles are reported in 3-2-1 (Yaw-Pitch-Roll, AKA Aircraft) order.
///
///@{

struct CompEulerAngles
{
    /// Parameters
    float roll = 0; ///< [radians]
    float pitch = 0; ///< [radians]
    float yaw = 0; ///< [radians]
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_sensor::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_sensor::DATA_COMP_EULER_ANGLES;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "CompEulerAngles";
    static constexpr const char* DOC_NAME = "Complementary Filter Euler Angles";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(roll,pitch,yaw);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(roll),std::ref(pitch),std::ref(yaw));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup sensor_comp_orientation_update_matrix_cpp  (0x80,0x0B) Comp Orientation Update Matrix
/// DEPRECATED!
///
///@{

struct CompOrientationUpdateMatrix
{
    /// Parameters
    Matrix3f m;
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_sensor::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_sensor::DATA_COMP_ORIENTATION_UPDATE_MATRIX;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "CompOrientationUpdateMatrix";
    static constexpr const char* DOC_NAME = "Complementary Filter Orientation Update Matrix";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(m[0],m[1],m[2],m[3],m[4],m[5],m[6],m[7],m[8]);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(m[0]),std::ref(m[1]),std::ref(m[2]),std::ref(m[3]),std::ref(m[4]),std::ref(m[5]),std::ref(m[6]),std::ref(m[7]),std::ref(m[8]));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup sensor_orientation_raw_temp_cpp  (0x80,0x0D) Orientation Raw Temp
/// DEPRECATED!
///
///@{

struct OrientationRawTemp
{
    /// Parameters
    uint16_t raw_temp[4] = {0};
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_sensor::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_sensor::DATA_TEMPERATURE_RAW;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "OrientationRawTemp";
    static constexpr const char* DOC_NAME = "OrientationRawTemp";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(raw_temp);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(raw_temp));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup sensor_internal_timestamp_cpp  (0x80,0x0E) Internal Timestamp
/// DEPRECATED!
///
///@{

struct InternalTimestamp
{
    /// Parameters
    uint32_t counts = 0;
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_sensor::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_sensor::DATA_TIME_STAMP_INTERNAL;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "InternalTimestamp";
    static constexpr const char* DOC_NAME = "InternalTimestamp";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(counts);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(counts));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup sensor_pps_timestamp_cpp  (0x80,0x0F) Pps Timestamp
/// DEPRECATED!
///
///@{

struct PpsTimestamp
{
    /// Parameters
    uint32_t seconds = 0;
    uint32_t useconds = 0;
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_sensor::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_sensor::DATA_TIME_STAMP_PPS;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "PpsTimestamp";
    static constexpr const char* DOC_NAME = "PPS Timestamp";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(seconds,useconds);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(seconds),std::ref(useconds));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup sensor_gps_timestamp_cpp  (0x80,0x12) Gps Timestamp
/// GPS timestamp of the SENSOR data
/// 
/// Should the PPS become unavailable, the device will revert to its internal clock, which will cause the reported time to drift from true GPS time.
/// Upon recovering from a PPS outage, the user should expect a jump in the reported GPS time due to the accumulation of internal clock error.
/// If synchronization to an external clock or onboard GNSS receiver (for products that have one) is disabled, this time is equivalent to internal system time.
/// 
/// Note: this data field may be deprecated in the future. The more flexible shared data field (0x80, 0xD3) should be used instead.
///
///@{

struct GpsTimestamp
{
    struct ValidFlags : Bitfield<ValidFlags>
    {
        typedef uint16_t Type;
        enum _enumType : uint16_t
        {
            NONE              = 0x0000,
            PPS_VALID         = 0x0001,  ///<  True when the PPS signal is present.
            TIME_REFRESH      = 0x0002,  ///<  Toggles each time the time is updated via internal GPS or the GPS Time Update command (0x01, 0x72).
            TIME_INITIALIZED  = 0x0004,  ///<  True if the time has ever been set.
            TOW_VALID         = 0x0008,  ///<  True if the time of week is valid.
            WEEK_NUMBER_VALID = 0x0010,  ///<  True if the week number is valid.
            ALL               = 0x001F,
        };
        uint16_t value = NONE;
        
        constexpr ValidFlags() : value(NONE) {}
        constexpr ValidFlags(int val) : value((uint16_t)val) {}
        constexpr operator uint16_t() const { return value; }
        constexpr ValidFlags& operator=(uint16_t val) { value = val; return *this; }
        constexpr ValidFlags& operator=(int val) { value = uint16_t(val); return *this; }
        constexpr ValidFlags& operator|=(uint16_t val) { return *this = value | val; }
        constexpr ValidFlags& operator&=(uint16_t val) { return *this = value & val; }
        
        constexpr bool ppsValid() const { return (value & PPS_VALID) > 0; }
        constexpr void ppsValid(bool val) { value &= ~PPS_VALID; if(val) value |= PPS_VALID; }
        constexpr bool timeRefresh() const { return (value & TIME_REFRESH) > 0; }
        constexpr void timeRefresh(bool val) { value &= ~TIME_REFRESH; if(val) value |= TIME_REFRESH; }
        constexpr bool timeInitialized() const { return (value & TIME_INITIALIZED) > 0; }
        constexpr void timeInitialized(bool val) { value &= ~TIME_INITIALIZED; if(val) value |= TIME_INITIALIZED; }
        constexpr bool towValid() const { return (value & TOW_VALID) > 0; }
        constexpr void towValid(bool val) { value &= ~TOW_VALID; if(val) value |= TOW_VALID; }
        constexpr bool weekNumberValid() const { return (value & WEEK_NUMBER_VALID) > 0; }
        constexpr void weekNumberValid(bool val) { value &= ~WEEK_NUMBER_VALID; if(val) value |= WEEK_NUMBER_VALID; }
        constexpr bool allSet() const { return value == ALL; }
        constexpr void setAll() { value |= ALL; }
    };
    /// Parameters
    double tow = 0; ///< GPS Time of Week [seconds]
    uint16_t week_number = 0; ///< GPS Week Number since 1980 [weeks]
    ValidFlags valid_flags;
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_sensor::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_sensor::DATA_TIME_STAMP_GPS;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "GpsTimestamp";
    static constexpr const char* DOC_NAME = "GpsTimestamp";
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
///@defgroup sensor_temperature_abs_cpp  (0x80,0x14) Temperature Abs
/// SENSOR reported temperature statistics
/// 
/// Temperature may originate from the MEMS sensors, or be calculated in combination with board temperature sensors.
/// All quantities are calculated with respect to the last power on or reset, whichever is later.
/// 
///
///@{

struct TemperatureAbs
{
    /// Parameters
    float min_temp = 0; ///< [degC]
    float max_temp = 0; ///< [degC]
    float mean_temp = 0; ///< [degC]
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_sensor::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_sensor::DATA_TEMPERATURE_ABS;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "TemperatureAbs";
    static constexpr const char* DOC_NAME = "Temperature Statistics";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(min_temp,max_temp,mean_temp);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(min_temp),std::ref(max_temp),std::ref(mean_temp));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup sensor_up_vector_cpp  (0x80,0x11) Up Vector
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
    /// Parameters
    Vector3f up; ///< [Gs]
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_sensor::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_sensor::DATA_STAB_ACCEL;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "UpVector";
    static constexpr const char* DOC_NAME = "UpVector";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(up[0],up[1],up[2]);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(up[0]),std::ref(up[1]),std::ref(up[2]));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup sensor_north_vector_cpp  (0x80,0x10) North Vector
/// Gyro-stabilized 3-element vector representing the complementary filter's estimate of magnetic north.
/// This quantity is expressed in the vehicle frame.
/// 
/// This quantity is sensitive to local magnetic field perturbations, which may cause notable deviations from true magnetic north.
///
///@{

struct NorthVector
{
    /// Parameters
    Vector3f north; ///< [Gauss]
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_sensor::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_sensor::DATA_STAB_MAG;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "NorthVector";
    static constexpr const char* DOC_NAME = "NorthVector";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(north[0],north[1],north[2]);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(north[0]),std::ref(north[1]),std::ref(north[2]));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup sensor_overrange_status_cpp  (0x80,0x18) Overrange Status
///
///@{

struct OverrangeStatus
{
    struct Status : Bitfield<Status>
    {
        typedef uint16_t Type;
        enum _enumType : uint16_t
        {
            NONE    = 0x0000,
            ACCEL_X = 0x0001,  ///<  
            ACCEL_Y = 0x0002,  ///<  
            ACCEL_Z = 0x0004,  ///<  
            GYRO_X  = 0x0010,  ///<  
            GYRO_Y  = 0x0020,  ///<  
            GYRO_Z  = 0x0040,  ///<  
            MAG_X   = 0x0100,  ///<  
            MAG_Y   = 0x0200,  ///<  
            MAG_Z   = 0x0400,  ///<  
            PRESS   = 0x1000,  ///<  
            ALL     = 0x1777,
        };
        uint16_t value = NONE;
        
        constexpr Status() : value(NONE) {}
        constexpr Status(int val) : value((uint16_t)val) {}
        constexpr operator uint16_t() const { return value; }
        constexpr Status& operator=(uint16_t val) { value = val; return *this; }
        constexpr Status& operator=(int val) { value = uint16_t(val); return *this; }
        constexpr Status& operator|=(uint16_t val) { return *this = value | val; }
        constexpr Status& operator&=(uint16_t val) { return *this = value & val; }
        
        constexpr bool accelX() const { return (value & ACCEL_X) > 0; }
        constexpr void accelX(bool val) { value &= ~ACCEL_X; if(val) value |= ACCEL_X; }
        constexpr bool accelY() const { return (value & ACCEL_Y) > 0; }
        constexpr void accelY(bool val) { value &= ~ACCEL_Y; if(val) value |= ACCEL_Y; }
        constexpr bool accelZ() const { return (value & ACCEL_Z) > 0; }
        constexpr void accelZ(bool val) { value &= ~ACCEL_Z; if(val) value |= ACCEL_Z; }
        constexpr bool gyroX() const { return (value & GYRO_X) > 0; }
        constexpr void gyroX(bool val) { value &= ~GYRO_X; if(val) value |= GYRO_X; }
        constexpr bool gyroY() const { return (value & GYRO_Y) > 0; }
        constexpr void gyroY(bool val) { value &= ~GYRO_Y; if(val) value |= GYRO_Y; }
        constexpr bool gyroZ() const { return (value & GYRO_Z) > 0; }
        constexpr void gyroZ(bool val) { value &= ~GYRO_Z; if(val) value |= GYRO_Z; }
        constexpr bool magX() const { return (value & MAG_X) > 0; }
        constexpr void magX(bool val) { value &= ~MAG_X; if(val) value |= MAG_X; }
        constexpr bool magY() const { return (value & MAG_Y) > 0; }
        constexpr void magY(bool val) { value &= ~MAG_Y; if(val) value |= MAG_Y; }
        constexpr bool magZ() const { return (value & MAG_Z) > 0; }
        constexpr void magZ(bool val) { value &= ~MAG_Z; if(val) value |= MAG_Z; }
        constexpr bool press() const { return (value & PRESS) > 0; }
        constexpr void press(bool val) { value &= ~PRESS; if(val) value |= PRESS; }
        constexpr bool allSet() const { return value == ALL; }
        constexpr void setAll() { value |= ALL; }
    };
    /// Parameters
    Status status;
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_sensor::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_sensor::DATA_OVERRANGE_STATUS;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "OverrangeStatus";
    static constexpr const char* DOC_NAME = "OverrangeStatus";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(status);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(status));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
};

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup sensor_odometer_data_cpp  (0x80,0x40) Odometer Data
///
///@{

struct OdometerData
{
    /// Parameters
    float speed = 0; ///< Average speed over the time interval [m/s]. Can be negative for quadrature encoders.
    float uncertainty = 0; ///< Uncertainty of velocity [m/s].
    uint16_t valid_flags = 0; ///< If odometer is configured, bit 0 will be set to 1.
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_sensor::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_sensor::DATA_ODOMETER;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "OdometerData";
    static constexpr const char* DOC_NAME = "OdometerData";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(speed,uncertainty,valid_flags);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(speed),std::ref(uncertainty),std::ref(valid_flags));
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
} // namespace data_sensor
} // namespace mip

