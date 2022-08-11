
#include "commands_filter.hpp"

#include "../../utils/serialization.h"
#include "../mip_interface.h"

#include <assert.h>


namespace mip {
class Serializer;

namespace C {
struct mip_interface;
} // namespace C

namespace commands_filter {

using ::mip::insert;
using ::mip::extract;
using namespace ::mip::C;

////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

void insert(Serializer& serializer, const Reset& self)
{
    (void)serializer;
    (void)self;
}

void extract(Serializer& serializer, Reset& self)
{
    (void)serializer;
    (void)self;
}

/// @brief Resets the filter to the initialization state.
/// 
/// If the auto-initialization feature is disabled, the initial attitude or heading must be set in
/// order to enter the run state after a reset.
/// 
/// @returns CmdResult
/// 
CmdResult reset(C::mip_interface& device)
{
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_RESET_FILTER, NULL, 0);
}

void insert(Serializer& serializer, const SetInitialAttitude& self)
{
    insert(serializer, self.roll);
    insert(serializer, self.pitch);
    insert(serializer, self.heading);
}

void extract(Serializer& serializer, SetInitialAttitude& self)
{
    extract(serializer, self.roll);
    extract(serializer, self.pitch);
    extract(serializer, self.heading);
}

/// @brief Set the sensor initial attitude.
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
/// @param roll [radians]
/// @param pitch [radians]
/// @param heading [radians]
/// 
/// @returns CmdResult
/// 
CmdResult setInitialAttitude(C::mip_interface& device, float roll, float pitch, float heading)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, roll);
    insert(serializer, pitch);
    insert(serializer, heading);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SET_INITIAL_ATTITUDE, buffer, serializer.offset);
}

void insert(Serializer& serializer, const EstimationControl& self)
{
    insert(serializer, self.function);
    insert(serializer, self.enable);
}

void extract(Serializer& serializer, EstimationControl& self)
{
    extract(serializer, self.function);
    extract(serializer, self.enable);
}

/// @brief Estimation Control Flags
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
/// @param enable See above
/// 
/// @returns CmdResult
/// 
CmdResult writeEstimationControl(C::mip_interface& device, EstimationControl::EnableFlags enable)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, enable);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ESTIMATION_CONTROL_FLAGS, buffer, serializer.offset);
}

/// @brief Estimation Control Flags
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
/// @param[out] enable See above
/// 
/// @returns CmdResult
/// 
CmdResult readEstimationControl(C::mip_interface& device, EstimationControl::EnableFlags& enable)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(!!serializer);
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_ESTIMATION_CONTROL_FLAGS, buffer, serializer.offset, REPLY_ESTIMATION_CONTROL_FLAGS, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        Serializer serializer(buffer, sizeof(buffer));
        
        extract(serializer, enable);
        
        if( !!!serializer )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Estimation Control Flags
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
/// @returns CmdResult
/// 
CmdResult saveEstimationControl(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ESTIMATION_CONTROL_FLAGS, buffer, serializer.offset);
}

/// @brief Estimation Control Flags
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
/// @returns CmdResult
/// 
CmdResult loadEstimationControl(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ESTIMATION_CONTROL_FLAGS, buffer, serializer.offset);
}

/// @brief Estimation Control Flags
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
/// @returns CmdResult
/// 
CmdResult defaultEstimationControl(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ESTIMATION_CONTROL_FLAGS, buffer, serializer.offset);
}

void insert(Serializer& serializer, const ExternalGnssUpdate& self)
{
    insert(serializer, self.gps_time);
    insert(serializer, self.gps_week);
    insert(serializer, self.latitude);
    insert(serializer, self.longitude);
    insert(serializer, self.height);
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.velocity[i]);
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.pos_uncertainty[i]);
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.vel_uncertainty[i]);
}

void extract(Serializer& serializer, ExternalGnssUpdate& self)
{
    extract(serializer, self.gps_time);
    extract(serializer, self.gps_week);
    extract(serializer, self.latitude);
    extract(serializer, self.longitude);
    extract(serializer, self.height);
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.velocity[i]);
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.pos_uncertainty[i]);
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.vel_uncertainty[i]);
}

/// @brief Provide a filter measurement from an external GNSS
/// 
/// The GNSS source control must be set to "external" for this command to succeed, otherwise it will be NACK'd.
/// Please refer to your device user manual for information on the maximum rate of this message.
/// 
/// @param gps_time [seconds]
/// @param gps_week [GPS week number, not modulus 1024]
/// @param latitude [degrees]
/// @param longitude [degrees]
/// @param height Above WGS84 ellipsoid [meters]
/// @param velocity NED Frame [meters/second]
/// @param pos_uncertainty NED Frame, 1-sigma [meters]
/// @param vel_uncertainty NED Frame, 1-sigma [meters/second]
/// 
/// @returns CmdResult
/// 
CmdResult externalGnssUpdate(C::mip_interface& device, double gps_time, uint16_t gps_week, double latitude, double longitude, double height, const float* velocity, const float* pos_uncertainty, const float* vel_uncertainty)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, gps_time);
    insert(serializer, gps_week);
    insert(serializer, latitude);
    insert(serializer, longitude);
    insert(serializer, height);
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, velocity[i]);
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, pos_uncertainty[i]);
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, vel_uncertainty[i]);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_EXTERNAL_GNSS_UPDATE, buffer, serializer.offset);
}

void insert(Serializer& serializer, const ExternalHeadingUpdate& self)
{
    insert(serializer, self.heading);
    insert(serializer, self.heading_uncertainty);
    insert(serializer, self.type);
}

void extract(Serializer& serializer, ExternalHeadingUpdate& self)
{
    extract(serializer, self.heading);
    extract(serializer, self.heading_uncertainty);
    extract(serializer, self.type);
}

/// @brief Provide a filter measurement from an external heading source
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
/// @param heading Bounded by +-PI [radians]
/// @param heading_uncertainty 1-sigma [radians]
/// @param type 1 - True, 2 - Magnetic
/// 
/// @returns CmdResult
/// 
CmdResult externalHeadingUpdate(C::mip_interface& device, float heading, float heading_uncertainty, uint8_t type)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, heading);
    insert(serializer, heading_uncertainty);
    insert(serializer, type);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_EXTERNAL_HEADING_UPDATE, buffer, serializer.offset);
}

void insert(Serializer& serializer, const ExternalHeadingUpdateWithTime& self)
{
    insert(serializer, self.gps_time);
    insert(serializer, self.gps_week);
    insert(serializer, self.heading);
    insert(serializer, self.heading_uncertainty);
    insert(serializer, self.type);
}

void extract(Serializer& serializer, ExternalHeadingUpdateWithTime& self)
{
    extract(serializer, self.gps_time);
    extract(serializer, self.gps_week);
    extract(serializer, self.heading);
    extract(serializer, self.heading_uncertainty);
    extract(serializer, self.type);
}

/// @brief Provide a filter measurement from an external heading source at a specific GPS time
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
/// @param gps_time [seconds]
/// @param gps_week [GPS week number, not modulus 1024]
/// @param heading Relative to true north, bounded by +-PI [radians]
/// @param heading_uncertainty 1-sigma [radians]
/// @param type 1 - True, 2 - Magnetic
/// 
/// @returns CmdResult
/// 
CmdResult externalHeadingUpdateWithTime(C::mip_interface& device, double gps_time, uint16_t gps_week, float heading, float heading_uncertainty, uint8_t type)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, gps_time);
    insert(serializer, gps_week);
    insert(serializer, heading);
    insert(serializer, heading_uncertainty);
    insert(serializer, type);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_EXTERNAL_HEADING_UPDATE_WITH_TIME, buffer, serializer.offset);
}

void insert(Serializer& serializer, const TareOrientation& self)
{
    insert(serializer, self.function);
    insert(serializer, self.axes);
}

void extract(Serializer& serializer, TareOrientation& self)
{
    extract(serializer, self.function);
    extract(serializer, self.axes);
}

/// @brief Tare the device orientation.
/// 
/// This function uses the current device orientation relative to the NED frame as the current sensor to vehicle transformation.
/// This command is provided as a convenient way to set the sensor to vehicle frame transformation.
/// The filter must be initialized and have a valid attitude output. If the attitude is not valid, an error will be returned.
/// @param axes Axes to tare
/// 
/// @returns CmdResult
/// 
CmdResult writeTareOrientation(C::mip_interface& device, TareOrientation::MipTareAxes axes)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, axes);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_TARE_ORIENTATION, buffer, serializer.offset);
}

/// @brief Tare the device orientation.
/// 
/// This function uses the current device orientation relative to the NED frame as the current sensor to vehicle transformation.
/// This command is provided as a convenient way to set the sensor to vehicle frame transformation.
/// The filter must be initialized and have a valid attitude output. If the attitude is not valid, an error will be returned.
/// @param[out] axes Axes to tare
/// 
/// @returns CmdResult
/// 
CmdResult readTareOrientation(C::mip_interface& device, TareOrientation::MipTareAxes& axes)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(!!serializer);
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_TARE_ORIENTATION, buffer, serializer.offset, REPLY_TARE_ORIENTATION, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        Serializer serializer(buffer, sizeof(buffer));
        
        extract(serializer, axes);
        
        if( !!!serializer )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Tare the device orientation.
/// 
/// This function uses the current device orientation relative to the NED frame as the current sensor to vehicle transformation.
/// This command is provided as a convenient way to set the sensor to vehicle frame transformation.
/// The filter must be initialized and have a valid attitude output. If the attitude is not valid, an error will be returned.
/// 
/// @returns CmdResult
/// 
CmdResult saveTareOrientation(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_TARE_ORIENTATION, buffer, serializer.offset);
}

/// @brief Tare the device orientation.
/// 
/// This function uses the current device orientation relative to the NED frame as the current sensor to vehicle transformation.
/// This command is provided as a convenient way to set the sensor to vehicle frame transformation.
/// The filter must be initialized and have a valid attitude output. If the attitude is not valid, an error will be returned.
/// 
/// @returns CmdResult
/// 
CmdResult loadTareOrientation(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_TARE_ORIENTATION, buffer, serializer.offset);
}

/// @brief Tare the device orientation.
/// 
/// This function uses the current device orientation relative to the NED frame as the current sensor to vehicle transformation.
/// This command is provided as a convenient way to set the sensor to vehicle frame transformation.
/// The filter must be initialized and have a valid attitude output. If the attitude is not valid, an error will be returned.
/// 
/// @returns CmdResult
/// 
CmdResult defaultTareOrientation(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_TARE_ORIENTATION, buffer, serializer.offset);
}

void insert(Serializer& serializer, const SensorToVehicleRotationEuler& self)
{
    insert(serializer, self.function);
    insert(serializer, self.roll);
    insert(serializer, self.pitch);
    insert(serializer, self.yaw);
}

void extract(Serializer& serializer, SensorToVehicleRotationEuler& self)
{
    extract(serializer, self.function);
    extract(serializer, self.roll);
    extract(serializer, self.pitch);
    extract(serializer, self.yaw);
}

/// @brief Set the sensor to vehicle frame rotation using Yaw, Pitch, Roll Euler angles.
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
/// @param roll [radians]
/// @param pitch [radians]
/// @param yaw [radians]
/// 
/// @returns CmdResult
/// 
CmdResult writeSensorToVehicleRotationEuler(C::mip_interface& device, float roll, float pitch, float yaw)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, roll);
    insert(serializer, pitch);
    insert(serializer, yaw);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_ROTATION_EULER, buffer, serializer.offset);
}

/// @brief Set the sensor to vehicle frame rotation using Yaw, Pitch, Roll Euler angles.
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
/// @param[out] roll [radians]
/// @param[out] pitch [radians]
/// @param[out] yaw [radians]
/// 
/// @returns CmdResult
/// 
CmdResult readSensorToVehicleRotationEuler(C::mip_interface& device, float& roll, float& pitch, float& yaw)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(!!serializer);
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_ROTATION_EULER, buffer, serializer.offset, REPLY_SENSOR2VEHICLE_ROTATION_EULER, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        Serializer serializer(buffer, sizeof(buffer));
        
        extract(serializer, roll);
        extract(serializer, pitch);
        extract(serializer, yaw);
        
        if( !!!serializer )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Set the sensor to vehicle frame rotation using Yaw, Pitch, Roll Euler angles.
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
/// @returns CmdResult
/// 
CmdResult saveSensorToVehicleRotationEuler(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_ROTATION_EULER, buffer, serializer.offset);
}

/// @brief Set the sensor to vehicle frame rotation using Yaw, Pitch, Roll Euler angles.
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
/// @returns CmdResult
/// 
CmdResult loadSensorToVehicleRotationEuler(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_ROTATION_EULER, buffer, serializer.offset);
}

/// @brief Set the sensor to vehicle frame rotation using Yaw, Pitch, Roll Euler angles.
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
/// @returns CmdResult
/// 
CmdResult defaultSensorToVehicleRotationEuler(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_ROTATION_EULER, buffer, serializer.offset);
}

void insert(Serializer& serializer, const SensorToVehicleRotationDcm& self)
{
    insert(serializer, self.function);
    for(unsigned int i=0; i < 9; i++)
        insert(serializer, self.dcm[i]);
}

void extract(Serializer& serializer, SensorToVehicleRotationDcm& self)
{
    extract(serializer, self.function);
    for(unsigned int i=0; i < 9; i++)
        extract(serializer, self.dcm[i]);
}

/// @brief Set the sensor to vehicle frame rotation using a row-major direction cosine matrix.
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
/// @param dcm 
/// 
/// @returns CmdResult
/// 
CmdResult writeSensorToVehicleRotationDcm(C::mip_interface& device, const float* dcm)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    for(unsigned int i=0; i < 9; i++)
        insert(serializer, dcm[i]);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_ROTATION_DCM, buffer, serializer.offset);
}

/// @brief Set the sensor to vehicle frame rotation using a row-major direction cosine matrix.
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
/// @param[out] dcm 
/// 
/// @returns CmdResult
/// 
CmdResult readSensorToVehicleRotationDcm(C::mip_interface& device, float* dcm)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(!!serializer);
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_ROTATION_DCM, buffer, serializer.offset, REPLY_SENSOR2VEHICLE_ROTATION_DCM, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        Serializer serializer(buffer, sizeof(buffer));
        
        for(unsigned int i=0; i < 9; i++)
            extract(serializer, dcm[i]);
        
        if( !!!serializer )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Set the sensor to vehicle frame rotation using a row-major direction cosine matrix.
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
/// @returns CmdResult
/// 
CmdResult saveSensorToVehicleRotationDcm(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_ROTATION_DCM, buffer, serializer.offset);
}

/// @brief Set the sensor to vehicle frame rotation using a row-major direction cosine matrix.
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
/// @returns CmdResult
/// 
CmdResult loadSensorToVehicleRotationDcm(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_ROTATION_DCM, buffer, serializer.offset);
}

/// @brief Set the sensor to vehicle frame rotation using a row-major direction cosine matrix.
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
/// @returns CmdResult
/// 
CmdResult defaultSensorToVehicleRotationDcm(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_ROTATION_DCM, buffer, serializer.offset);
}

void insert(Serializer& serializer, const SensorToVehicleRotationQuaternion& self)
{
    insert(serializer, self.function);
    for(unsigned int i=0; i < 4; i++)
        insert(serializer, self.quat[i]);
}

void extract(Serializer& serializer, SensorToVehicleRotationQuaternion& self)
{
    extract(serializer, self.function);
    for(unsigned int i=0; i < 4; i++)
        extract(serializer, self.quat[i]);
}

/// @brief Set the sensor to vehicle frame rotation using a quaternion.
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
/// @param quat 
/// 
/// @returns CmdResult
/// 
CmdResult writeSensorToVehicleRotationQuaternion(C::mip_interface& device, const float* quat)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    for(unsigned int i=0; i < 4; i++)
        insert(serializer, quat[i]);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_ROTATION_QUATERNION, buffer, serializer.offset);
}

/// @brief Set the sensor to vehicle frame rotation using a quaternion.
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
/// @param[out] quat 
/// 
/// @returns CmdResult
/// 
CmdResult readSensorToVehicleRotationQuaternion(C::mip_interface& device, float* quat)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(!!serializer);
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_ROTATION_QUATERNION, buffer, serializer.offset, REPLY_SENSOR2VEHICLE_ROTATION_QUATERNION, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        Serializer serializer(buffer, sizeof(buffer));
        
        for(unsigned int i=0; i < 4; i++)
            extract(serializer, quat[i]);
        
        if( !!!serializer )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Set the sensor to vehicle frame rotation using a quaternion.
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
/// @returns CmdResult
/// 
CmdResult saveSensorToVehicleRotationQuaternion(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_ROTATION_QUATERNION, buffer, serializer.offset);
}

/// @brief Set the sensor to vehicle frame rotation using a quaternion.
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
/// @returns CmdResult
/// 
CmdResult loadSensorToVehicleRotationQuaternion(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_ROTATION_QUATERNION, buffer, serializer.offset);
}

/// @brief Set the sensor to vehicle frame rotation using a quaternion.
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
/// @returns CmdResult
/// 
CmdResult defaultSensorToVehicleRotationQuaternion(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_ROTATION_QUATERNION, buffer, serializer.offset);
}

void insert(Serializer& serializer, const SensorToVehicleOffset& self)
{
    insert(serializer, self.function);
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.offset[i]);
}

void extract(Serializer& serializer, SensorToVehicleOffset& self)
{
    extract(serializer, self.function);
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.offset[i]);
}

/// @brief Set the sensor to vehicle frame offset, expressed in the sensor frame.
/// 
/// This is a simple offset, not a lever arm.  It does not compensate for inertial effects experienced from being offset from the center of gravity/rotation of the vehicle.
/// It simply adds the offset to the position output to express it in the origin of the user's vehicle frame.
/// 
/// This offset affects the following output quantities:
/// Estimated LLH Position
/// 
/// The magnitude of the offset vector is limited to 10 meters
/// @param offset [meters]
/// 
/// @returns CmdResult
/// 
CmdResult writeSensorToVehicleOffset(C::mip_interface& device, const float* offset)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, offset[i]);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_OFFSET, buffer, serializer.offset);
}

/// @brief Set the sensor to vehicle frame offset, expressed in the sensor frame.
/// 
/// This is a simple offset, not a lever arm.  It does not compensate for inertial effects experienced from being offset from the center of gravity/rotation of the vehicle.
/// It simply adds the offset to the position output to express it in the origin of the user's vehicle frame.
/// 
/// This offset affects the following output quantities:
/// Estimated LLH Position
/// 
/// The magnitude of the offset vector is limited to 10 meters
/// @param[out] offset [meters]
/// 
/// @returns CmdResult
/// 
CmdResult readSensorToVehicleOffset(C::mip_interface& device, float* offset)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(!!serializer);
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_OFFSET, buffer, serializer.offset, REPLY_SENSOR2VEHICLE_OFFSET, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        Serializer serializer(buffer, sizeof(buffer));
        
        for(unsigned int i=0; i < 3; i++)
            extract(serializer, offset[i]);
        
        if( !!!serializer )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Set the sensor to vehicle frame offset, expressed in the sensor frame.
/// 
/// This is a simple offset, not a lever arm.  It does not compensate for inertial effects experienced from being offset from the center of gravity/rotation of the vehicle.
/// It simply adds the offset to the position output to express it in the origin of the user's vehicle frame.
/// 
/// This offset affects the following output quantities:
/// Estimated LLH Position
/// 
/// The magnitude of the offset vector is limited to 10 meters
/// 
/// @returns CmdResult
/// 
CmdResult saveSensorToVehicleOffset(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_OFFSET, buffer, serializer.offset);
}

/// @brief Set the sensor to vehicle frame offset, expressed in the sensor frame.
/// 
/// This is a simple offset, not a lever arm.  It does not compensate for inertial effects experienced from being offset from the center of gravity/rotation of the vehicle.
/// It simply adds the offset to the position output to express it in the origin of the user's vehicle frame.
/// 
/// This offset affects the following output quantities:
/// Estimated LLH Position
/// 
/// The magnitude of the offset vector is limited to 10 meters
/// 
/// @returns CmdResult
/// 
CmdResult loadSensorToVehicleOffset(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_OFFSET, buffer, serializer.offset);
}

/// @brief Set the sensor to vehicle frame offset, expressed in the sensor frame.
/// 
/// This is a simple offset, not a lever arm.  It does not compensate for inertial effects experienced from being offset from the center of gravity/rotation of the vehicle.
/// It simply adds the offset to the position output to express it in the origin of the user's vehicle frame.
/// 
/// This offset affects the following output quantities:
/// Estimated LLH Position
/// 
/// The magnitude of the offset vector is limited to 10 meters
/// 
/// @returns CmdResult
/// 
CmdResult defaultSensorToVehicleOffset(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_OFFSET, buffer, serializer.offset);
}

void insert(Serializer& serializer, const AntennaOffset& self)
{
    insert(serializer, self.function);
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.offset[i]);
}

void extract(Serializer& serializer, AntennaOffset& self)
{
    extract(serializer, self.function);
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.offset[i]);
}

/// @brief Set the sensor to GNSS antenna offset.
/// 
/// This is expressed in the sensor frame, from the sensor origin to the GNSS antenna RF center.
/// 
/// The magnitude of the offset vector is limited to 10 meters
/// 
/// @param offset [meters]
/// 
/// @returns CmdResult
/// 
CmdResult writeAntennaOffset(C::mip_interface& device, const float* offset)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, offset[i]);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ANTENNA_OFFSET, buffer, serializer.offset);
}

/// @brief Set the sensor to GNSS antenna offset.
/// 
/// This is expressed in the sensor frame, from the sensor origin to the GNSS antenna RF center.
/// 
/// The magnitude of the offset vector is limited to 10 meters
/// 
/// @param[out] offset [meters]
/// 
/// @returns CmdResult
/// 
CmdResult readAntennaOffset(C::mip_interface& device, float* offset)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(!!serializer);
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_ANTENNA_OFFSET, buffer, serializer.offset, REPLY_ANTENNA_OFFSET, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        Serializer serializer(buffer, sizeof(buffer));
        
        for(unsigned int i=0; i < 3; i++)
            extract(serializer, offset[i]);
        
        if( !!!serializer )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Set the sensor to GNSS antenna offset.
/// 
/// This is expressed in the sensor frame, from the sensor origin to the GNSS antenna RF center.
/// 
/// The magnitude of the offset vector is limited to 10 meters
/// 
/// 
/// @returns CmdResult
/// 
CmdResult saveAntennaOffset(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ANTENNA_OFFSET, buffer, serializer.offset);
}

/// @brief Set the sensor to GNSS antenna offset.
/// 
/// This is expressed in the sensor frame, from the sensor origin to the GNSS antenna RF center.
/// 
/// The magnitude of the offset vector is limited to 10 meters
/// 
/// 
/// @returns CmdResult
/// 
CmdResult loadAntennaOffset(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ANTENNA_OFFSET, buffer, serializer.offset);
}

/// @brief Set the sensor to GNSS antenna offset.
/// 
/// This is expressed in the sensor frame, from the sensor origin to the GNSS antenna RF center.
/// 
/// The magnitude of the offset vector is limited to 10 meters
/// 
/// 
/// @returns CmdResult
/// 
CmdResult defaultAntennaOffset(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ANTENNA_OFFSET, buffer, serializer.offset);
}

void insert(Serializer& serializer, const GnssSource& self)
{
    insert(serializer, self.function);
    insert(serializer, self.source);
}

void extract(Serializer& serializer, GnssSource& self)
{
    extract(serializer, self.function);
    extract(serializer, self.source);
}

/// @brief Control the source of GNSS information used to update the Kalman Filter.
/// 
/// Changing the GNSS source while the sensor is in the "running" state may temporarily place
/// it back in the "init" state until the new source of GNSS data is received.
/// 
/// @param source 
/// 
/// @returns CmdResult
/// 
CmdResult writeGnssSource(C::mip_interface& device, GnssSource::Source source)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, source);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GNSS_SOURCE_CONTROL, buffer, serializer.offset);
}

/// @brief Control the source of GNSS information used to update the Kalman Filter.
/// 
/// Changing the GNSS source while the sensor is in the "running" state may temporarily place
/// it back in the "init" state until the new source of GNSS data is received.
/// 
/// @param[out] source 
/// 
/// @returns CmdResult
/// 
CmdResult readGnssSource(C::mip_interface& device, GnssSource::Source& source)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(!!serializer);
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GNSS_SOURCE_CONTROL, buffer, serializer.offset, REPLY_GNSS_SOURCE_CONTROL, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        Serializer serializer(buffer, sizeof(buffer));
        
        extract(serializer, source);
        
        if( !!!serializer )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Control the source of GNSS information used to update the Kalman Filter.
/// 
/// Changing the GNSS source while the sensor is in the "running" state may temporarily place
/// it back in the "init" state until the new source of GNSS data is received.
/// 
/// 
/// @returns CmdResult
/// 
CmdResult saveGnssSource(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GNSS_SOURCE_CONTROL, buffer, serializer.offset);
}

/// @brief Control the source of GNSS information used to update the Kalman Filter.
/// 
/// Changing the GNSS source while the sensor is in the "running" state may temporarily place
/// it back in the "init" state until the new source of GNSS data is received.
/// 
/// 
/// @returns CmdResult
/// 
CmdResult loadGnssSource(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GNSS_SOURCE_CONTROL, buffer, serializer.offset);
}

/// @brief Control the source of GNSS information used to update the Kalman Filter.
/// 
/// Changing the GNSS source while the sensor is in the "running" state may temporarily place
/// it back in the "init" state until the new source of GNSS data is received.
/// 
/// 
/// @returns CmdResult
/// 
CmdResult defaultGnssSource(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GNSS_SOURCE_CONTROL, buffer, serializer.offset);
}

void insert(Serializer& serializer, const HeadingSource& self)
{
    insert(serializer, self.function);
    insert(serializer, self.source);
}

void extract(Serializer& serializer, HeadingSource& self)
{
    extract(serializer, self.function);
    extract(serializer, self.source);
}

/// @brief Control the source of heading information used to update the Kalman Filter.
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
/// @param source 
/// 
/// @returns CmdResult
/// 
CmdResult writeHeadingSource(C::mip_interface& device, HeadingSource::Source source)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, source);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_HEADING_UPDATE_CONTROL, buffer, serializer.offset);
}

/// @brief Control the source of heading information used to update the Kalman Filter.
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
/// @param[out] source 
/// 
/// @returns CmdResult
/// 
CmdResult readHeadingSource(C::mip_interface& device, HeadingSource::Source& source)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(!!serializer);
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_HEADING_UPDATE_CONTROL, buffer, serializer.offset, REPLY_HEADING_UPDATE_CONTROL, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        Serializer serializer(buffer, sizeof(buffer));
        
        extract(serializer, source);
        
        if( !!!serializer )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Control the source of heading information used to update the Kalman Filter.
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
/// @returns CmdResult
/// 
CmdResult saveHeadingSource(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_HEADING_UPDATE_CONTROL, buffer, serializer.offset);
}

/// @brief Control the source of heading information used to update the Kalman Filter.
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
/// @returns CmdResult
/// 
CmdResult loadHeadingSource(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_HEADING_UPDATE_CONTROL, buffer, serializer.offset);
}

/// @brief Control the source of heading information used to update the Kalman Filter.
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
/// @returns CmdResult
/// 
CmdResult defaultHeadingSource(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_HEADING_UPDATE_CONTROL, buffer, serializer.offset);
}

void insert(Serializer& serializer, const AltitudeAiding& self)
{
    insert(serializer, self.function);
    insert(serializer, self.aiding_selector);
}

void extract(Serializer& serializer, AltitudeAiding& self)
{
    extract(serializer, self.function);
    extract(serializer, self.aiding_selector);
}

/// @brief Altitude Aiding Control
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
/// @param aiding_selector See above
/// 
/// @returns CmdResult
/// 
CmdResult writeAltitudeAiding(C::mip_interface& device, uint8_t aiding_selector)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, aiding_selector);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ALTITUDE_AIDING_CONTROL, buffer, serializer.offset);
}

/// @brief Altitude Aiding Control
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
/// @param[out] aiding_selector See above
/// 
/// @returns CmdResult
/// 
CmdResult readAltitudeAiding(C::mip_interface& device, uint8_t& aiding_selector)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(!!serializer);
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_ALTITUDE_AIDING_CONTROL, buffer, serializer.offset, REPLY_ALTITUDE_AIDING_CONTROL, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        Serializer serializer(buffer, sizeof(buffer));
        
        extract(serializer, aiding_selector);
        
        if( !!!serializer )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Altitude Aiding Control
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
/// @returns CmdResult
/// 
CmdResult saveAltitudeAiding(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ALTITUDE_AIDING_CONTROL, buffer, serializer.offset);
}

/// @brief Altitude Aiding Control
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
/// @returns CmdResult
/// 
CmdResult loadAltitudeAiding(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ALTITUDE_AIDING_CONTROL, buffer, serializer.offset);
}

/// @brief Altitude Aiding Control
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
/// @returns CmdResult
/// 
CmdResult defaultAltitudeAiding(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ALTITUDE_AIDING_CONTROL, buffer, serializer.offset);
}

void insert(Serializer& serializer, const AutoZupt& self)
{
    insert(serializer, self.function);
    insert(serializer, self.enable);
    insert(serializer, self.threshold);
}

void extract(Serializer& serializer, AutoZupt& self)
{
    extract(serializer, self.function);
    extract(serializer, self.enable);
    extract(serializer, self.threshold);
}

/// @brief Zero Velocity Update
/// The ZUPT is triggered when the scalar magnitude of the GNSS reported velocity vector is equal-to or less than the threshold value.
/// The device will NACK threshold values that are less than zero (i.e.negative.)
/// @param enable 0 - Disable, 1 - Enable
/// @param threshold [meters/second]
/// 
/// @returns CmdResult
/// 
CmdResult writeAutoZupt(C::mip_interface& device, uint8_t enable, float threshold)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, enable);
    insert(serializer, threshold);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ZUPT_CONTROL, buffer, serializer.offset);
}

/// @brief Zero Velocity Update
/// The ZUPT is triggered when the scalar magnitude of the GNSS reported velocity vector is equal-to or less than the threshold value.
/// The device will NACK threshold values that are less than zero (i.e.negative.)
/// @param[out] enable 0 - Disable, 1 - Enable
/// @param[out] threshold [meters/second]
/// 
/// @returns CmdResult
/// 
CmdResult readAutoZupt(C::mip_interface& device, uint8_t& enable, float& threshold)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(!!serializer);
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_ZUPT_CONTROL, buffer, serializer.offset, REPLY_ZUPT_CONTROL, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        Serializer serializer(buffer, sizeof(buffer));
        
        extract(serializer, enable);
        extract(serializer, threshold);
        
        if( !!!serializer )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Zero Velocity Update
/// The ZUPT is triggered when the scalar magnitude of the GNSS reported velocity vector is equal-to or less than the threshold value.
/// The device will NACK threshold values that are less than zero (i.e.negative.)
/// 
/// @returns CmdResult
/// 
CmdResult saveAutoZupt(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ZUPT_CONTROL, buffer, serializer.offset);
}

/// @brief Zero Velocity Update
/// The ZUPT is triggered when the scalar magnitude of the GNSS reported velocity vector is equal-to or less than the threshold value.
/// The device will NACK threshold values that are less than zero (i.e.negative.)
/// 
/// @returns CmdResult
/// 
CmdResult loadAutoZupt(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ZUPT_CONTROL, buffer, serializer.offset);
}

/// @brief Zero Velocity Update
/// The ZUPT is triggered when the scalar magnitude of the GNSS reported velocity vector is equal-to or less than the threshold value.
/// The device will NACK threshold values that are less than zero (i.e.negative.)
/// 
/// @returns CmdResult
/// 
CmdResult defaultAutoZupt(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ZUPT_CONTROL, buffer, serializer.offset);
}

void insert(Serializer& serializer, const AutoAngularZupt& self)
{
    insert(serializer, self.function);
    insert(serializer, self.enable);
    insert(serializer, self.threshold);
}

void extract(Serializer& serializer, AutoAngularZupt& self)
{
    extract(serializer, self.function);
    extract(serializer, self.enable);
    extract(serializer, self.threshold);
}

/// @brief Zero Angular Rate Update
/// The ZUPT is triggered when the scalar magnitude of the angular rate vector is equal-to or less than the threshold value.
/// The device will NACK threshold values that are less than zero (i.e.negative.)
/// @param enable 0 - Disable, 1 - Enable
/// @param threshold [radians/second]
/// 
/// @returns CmdResult
/// 
CmdResult writeAutoAngularZupt(C::mip_interface& device, uint8_t enable, float threshold)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, enable);
    insert(serializer, threshold);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ANGULAR_ZUPT_CONTROL, buffer, serializer.offset);
}

/// @brief Zero Angular Rate Update
/// The ZUPT is triggered when the scalar magnitude of the angular rate vector is equal-to or less than the threshold value.
/// The device will NACK threshold values that are less than zero (i.e.negative.)
/// @param[out] enable 0 - Disable, 1 - Enable
/// @param[out] threshold [radians/second]
/// 
/// @returns CmdResult
/// 
CmdResult readAutoAngularZupt(C::mip_interface& device, uint8_t& enable, float& threshold)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(!!serializer);
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_ANGULAR_ZUPT_CONTROL, buffer, serializer.offset, REPLY_ANGULAR_ZUPT_CONTROL, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        Serializer serializer(buffer, sizeof(buffer));
        
        extract(serializer, enable);
        extract(serializer, threshold);
        
        if( !!!serializer )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Zero Angular Rate Update
/// The ZUPT is triggered when the scalar magnitude of the angular rate vector is equal-to or less than the threshold value.
/// The device will NACK threshold values that are less than zero (i.e.negative.)
/// 
/// @returns CmdResult
/// 
CmdResult saveAutoAngularZupt(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ANGULAR_ZUPT_CONTROL, buffer, serializer.offset);
}

/// @brief Zero Angular Rate Update
/// The ZUPT is triggered when the scalar magnitude of the angular rate vector is equal-to or less than the threshold value.
/// The device will NACK threshold values that are less than zero (i.e.negative.)
/// 
/// @returns CmdResult
/// 
CmdResult loadAutoAngularZupt(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ANGULAR_ZUPT_CONTROL, buffer, serializer.offset);
}

/// @brief Zero Angular Rate Update
/// The ZUPT is triggered when the scalar magnitude of the angular rate vector is equal-to or less than the threshold value.
/// The device will NACK threshold values that are less than zero (i.e.negative.)
/// 
/// @returns CmdResult
/// 
CmdResult defaultAutoAngularZupt(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ANGULAR_ZUPT_CONTROL, buffer, serializer.offset);
}

void insert(Serializer& serializer, const CommandedZupt& self)
{
    (void)serializer;
    (void)self;
}

void extract(Serializer& serializer, CommandedZupt& self)
{
    (void)serializer;
    (void)self;
}

/// @brief Commanded Zero Velocity Update
/// Please see the device user manual for the maximum rate of this message.
/// 
/// @returns CmdResult
/// 
CmdResult commandedZupt(C::mip_interface& device)
{
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_COMMANDED_ZUPT, NULL, 0);
}

void insert(Serializer& serializer, const CommandedAngularZupt& self)
{
    (void)serializer;
    (void)self;
}

void extract(Serializer& serializer, CommandedAngularZupt& self)
{
    (void)serializer;
    (void)self;
}

/// @brief Commanded Zero Angular Rate Update
/// Please see the device user manual for the maximum rate of this message.
/// 
/// @returns CmdResult
/// 
CmdResult commandedAngularZupt(C::mip_interface& device)
{
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_COMMANDED_ANGULAR_ZUPT, NULL, 0);
}

void insert(Serializer& serializer, const AidingMeasurementEnable& self)
{
    insert(serializer, self.function);
    insert(serializer, self.aiding_source);
    insert(serializer, self.enable);
}

void extract(Serializer& serializer, AidingMeasurementEnable& self)
{
    extract(serializer, self.function);
    extract(serializer, self.aiding_source);
    extract(serializer, self.enable);
}

/// @brief Enables / disables the specified aiding measurement source.
/// 
/// 
/// @param aiding_source Aiding measurement source
/// @param enable Controls the aiding sorce
/// 
/// @returns CmdResult
/// 
CmdResult writeAidingMeasurementEnable(C::mip_interface& device, AidingMeasurementEnable::AidingSource aiding_source, bool enable)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, aiding_source);
    insert(serializer, enable);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_AIDING_MEASUREMENT_ENABLE, buffer, serializer.offset);
}

/// @brief Enables / disables the specified aiding measurement source.
/// 
/// 
/// @param aiding_source Aiding measurement source
/// @param[out] aiding_source Aiding measurement source
/// @param[out] enable Controls the aiding sorce
/// 
/// @returns CmdResult
/// 
CmdResult readAidingMeasurementEnable(C::mip_interface& device, AidingMeasurementEnable::AidingSource aiding_source, bool& enable)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    insert(serializer, aiding_source);
    assert(!!serializer);
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_AIDING_MEASUREMENT_ENABLE, buffer, serializer.offset, REPLY_AIDING_MEASUREMENT_ENABLE, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        Serializer serializer(buffer, sizeof(buffer));
        
        extract(serializer, aiding_source);
        extract(serializer, enable);
        
        if( !!!serializer )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Enables / disables the specified aiding measurement source.
/// 
/// 
/// @param aiding_source Aiding measurement source
/// 
/// @returns CmdResult
/// 
CmdResult saveAidingMeasurementEnable(C::mip_interface& device, AidingMeasurementEnable::AidingSource aiding_source)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    insert(serializer, aiding_source);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_AIDING_MEASUREMENT_ENABLE, buffer, serializer.offset);
}

/// @brief Enables / disables the specified aiding measurement source.
/// 
/// 
/// @param aiding_source Aiding measurement source
/// 
/// @returns CmdResult
/// 
CmdResult loadAidingMeasurementEnable(C::mip_interface& device, AidingMeasurementEnable::AidingSource aiding_source)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    insert(serializer, aiding_source);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_AIDING_MEASUREMENT_ENABLE, buffer, serializer.offset);
}

/// @brief Enables / disables the specified aiding measurement source.
/// 
/// 
/// @param aiding_source Aiding measurement source
/// 
/// @returns CmdResult
/// 
CmdResult defaultAidingMeasurementEnable(C::mip_interface& device, AidingMeasurementEnable::AidingSource aiding_source)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    insert(serializer, aiding_source);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_AIDING_MEASUREMENT_ENABLE, buffer, serializer.offset);
}

void insert(Serializer& serializer, const Run& self)
{
    (void)serializer;
    (void)self;
}

void extract(Serializer& serializer, Run& self)
{
    (void)serializer;
    (void)self;
}

/// @brief Manual run command.
/// 
/// If the initialization configuration has the "wait_for_run_command" option enabled, the filter will wait until it receives this command before commencing integration and enabling the Kalman filter. Prior to the receipt of this command, the filter will remain in the filter initialization mode.
/// 
/// @returns CmdResult
/// 
CmdResult run(C::mip_interface& device)
{
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_RUN, NULL, 0);
}

void insert(Serializer& serializer, const KinematicConstraint& self)
{
    insert(serializer, self.function);
    insert(serializer, self.acceleration_constraint_selection);
    insert(serializer, self.velocity_constraint_selection);
    insert(serializer, self.angular_constraint_selection);
}

void extract(Serializer& serializer, KinematicConstraint& self)
{
    extract(serializer, self.function);
    extract(serializer, self.acceleration_constraint_selection);
    extract(serializer, self.velocity_constraint_selection);
    extract(serializer, self.angular_constraint_selection);
}

/// @brief Controls kinematic constraint model selection for the navigation filter.
/// 
/// See manual for explanation of how the kinematic constraints are applied.
/// @param acceleration_constraint_selection Acceleration constraint: <br/> 0=None (default), <br/> 1=Zero-acceleration.
/// @param velocity_constraint_selection 0=None (default), <br/> 1=Zero-velocity, <br/> 2=Wheeled-vehicle. <br/>
/// @param angular_constraint_selection 0=None (default), 1=Zero-angular rate (ZUPT).
/// 
/// @returns CmdResult
/// 
CmdResult writeKinematicConstraint(C::mip_interface& device, uint8_t acceleration_constraint_selection, uint8_t velocity_constraint_selection, uint8_t angular_constraint_selection)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, acceleration_constraint_selection);
    insert(serializer, velocity_constraint_selection);
    insert(serializer, angular_constraint_selection);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_KINEMATIC_CONSTRAINT, buffer, serializer.offset);
}

/// @brief Controls kinematic constraint model selection for the navigation filter.
/// 
/// See manual for explanation of how the kinematic constraints are applied.
/// @param[out] acceleration_constraint_selection Acceleration constraint: <br/> 0=None (default), <br/> 1=Zero-acceleration.
/// @param[out] velocity_constraint_selection 0=None (default), <br/> 1=Zero-velocity, <br/> 2=Wheeled-vehicle. <br/>
/// @param[out] angular_constraint_selection 0=None (default), 1=Zero-angular rate (ZUPT).
/// 
/// @returns CmdResult
/// 
CmdResult readKinematicConstraint(C::mip_interface& device, uint8_t& acceleration_constraint_selection, uint8_t& velocity_constraint_selection, uint8_t& angular_constraint_selection)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(!!serializer);
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_KINEMATIC_CONSTRAINT, buffer, serializer.offset, REPLY_KINEMATIC_CONSTRAINT, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        Serializer serializer(buffer, sizeof(buffer));
        
        extract(serializer, acceleration_constraint_selection);
        extract(serializer, velocity_constraint_selection);
        extract(serializer, angular_constraint_selection);
        
        if( !!!serializer )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Controls kinematic constraint model selection for the navigation filter.
/// 
/// See manual for explanation of how the kinematic constraints are applied.
/// 
/// @returns CmdResult
/// 
CmdResult saveKinematicConstraint(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_KINEMATIC_CONSTRAINT, buffer, serializer.offset);
}

/// @brief Controls kinematic constraint model selection for the navigation filter.
/// 
/// See manual for explanation of how the kinematic constraints are applied.
/// 
/// @returns CmdResult
/// 
CmdResult loadKinematicConstraint(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_KINEMATIC_CONSTRAINT, buffer, serializer.offset);
}

/// @brief Controls kinematic constraint model selection for the navigation filter.
/// 
/// See manual for explanation of how the kinematic constraints are applied.
/// 
/// @returns CmdResult
/// 
CmdResult defaultKinematicConstraint(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_KINEMATIC_CONSTRAINT, buffer, serializer.offset);
}

void insert(Serializer& serializer, const InitializationConfiguration& self)
{
    insert(serializer, self.function);
    insert(serializer, self.wait_for_run_command);
    insert(serializer, self.initial_cond_src);
    insert(serializer, self.auto_heading_alignment_selector);
    insert(serializer, self.initial_heading);
    insert(serializer, self.initial_pitch);
    insert(serializer, self.initial_roll);
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.initial_position[i]);
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.initial_velocity[i]);
    insert(serializer, self.reference_frame_selector);
}

void extract(Serializer& serializer, InitializationConfiguration& self)
{
    extract(serializer, self.function);
    extract(serializer, self.wait_for_run_command);
    extract(serializer, self.initial_cond_src);
    extract(serializer, self.auto_heading_alignment_selector);
    extract(serializer, self.initial_heading);
    extract(serializer, self.initial_pitch);
    extract(serializer, self.initial_roll);
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.initial_position[i]);
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.initial_velocity[i]);
    extract(serializer, self.reference_frame_selector);
}

/// @brief Controls the source and values used for initial conditions of the navigation solution.
/// 
/// Notes: Initial conditions are the position, velocity, and attitude of the platform used when the filter starts running or is reset.
/// For the user specified position array, the units are meters if the ECEF frame is selected, and degrees latitude, degrees longitude, and meters above ellipsoid if the latitude/longitude/height frame is selected.
/// For the user specified velocity array, the units are meters per second, but the reference frame depends on the reference frame selector (ECEF or NED).
/// @param wait_for_run_command Initialize filter only after receiving "run" command
/// @param initial_cond_src Initial condition source:
/// @param auto_heading_alignment_selector Bitfield specifying the allowed automatic heading alignment methods for automatic initial conditions. Bits are set to 1 to enable, and the correspond to the following: <br/>
/// @param initial_heading User-specified initial platform heading (degrees).
/// @param initial_pitch User-specified initial platform pitch (degrees)
/// @param initial_roll User-specified initial platform roll (degrees)
/// @param initial_position User-specified initial platform position (units determined by reference frame selector, see note.)
/// @param initial_velocity User-specified initial platform velocity (units determined by reference frame selector, see note.)
/// @param reference_frame_selector User-specified initial position/velocity reference frames
/// 
/// @returns CmdResult
/// 
CmdResult writeInitializationConfiguration(C::mip_interface& device, uint8_t wait_for_run_command, InitializationConfiguration::InitialConditionSource initial_cond_src, InitializationConfiguration::AlignmentSelector auto_heading_alignment_selector, float initial_heading, float initial_pitch, float initial_roll, const float* initial_position, const float* initial_velocity, FilterReferenceFrame reference_frame_selector)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, wait_for_run_command);
    insert(serializer, initial_cond_src);
    insert(serializer, auto_heading_alignment_selector);
    insert(serializer, initial_heading);
    insert(serializer, initial_pitch);
    insert(serializer, initial_roll);
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, initial_position[i]);
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, initial_velocity[i]);
    insert(serializer, reference_frame_selector);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_INITIALIZATION_CONFIGURATION, buffer, serializer.offset);
}

/// @brief Controls the source and values used for initial conditions of the navigation solution.
/// 
/// Notes: Initial conditions are the position, velocity, and attitude of the platform used when the filter starts running or is reset.
/// For the user specified position array, the units are meters if the ECEF frame is selected, and degrees latitude, degrees longitude, and meters above ellipsoid if the latitude/longitude/height frame is selected.
/// For the user specified velocity array, the units are meters per second, but the reference frame depends on the reference frame selector (ECEF or NED).
/// @param[out] wait_for_run_command Initialize filter only after receiving "run" command
/// @param[out] initial_cond_src Initial condition source:
/// @param[out] auto_heading_alignment_selector Bitfield specifying the allowed automatic heading alignment methods for automatic initial conditions. Bits are set to 1 to enable, and the correspond to the following: <br/>
/// @param[out] initial_heading User-specified initial platform heading (degrees).
/// @param[out] initial_pitch User-specified initial platform pitch (degrees)
/// @param[out] initial_roll User-specified initial platform roll (degrees)
/// @param[out] initial_position User-specified initial platform position (units determined by reference frame selector, see note.)
/// @param[out] initial_velocity User-specified initial platform velocity (units determined by reference frame selector, see note.)
/// @param[out] reference_frame_selector User-specified initial position/velocity reference frames
/// 
/// @returns CmdResult
/// 
CmdResult readInitializationConfiguration(C::mip_interface& device, uint8_t& wait_for_run_command, InitializationConfiguration::InitialConditionSource& initial_cond_src, InitializationConfiguration::AlignmentSelector& auto_heading_alignment_selector, float& initial_heading, float& initial_pitch, float& initial_roll, float* initial_position, float* initial_velocity, FilterReferenceFrame& reference_frame_selector)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(!!serializer);
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_INITIALIZATION_CONFIGURATION, buffer, serializer.offset, REPLY_INITIALIZATION_CONFIGURATION, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        Serializer serializer(buffer, sizeof(buffer));
        
        extract(serializer, wait_for_run_command);
        extract(serializer, initial_cond_src);
        extract(serializer, auto_heading_alignment_selector);
        extract(serializer, initial_heading);
        extract(serializer, initial_pitch);
        extract(serializer, initial_roll);
        for(unsigned int i=0; i < 3; i++)
            extract(serializer, initial_position[i]);
        for(unsigned int i=0; i < 3; i++)
            extract(serializer, initial_velocity[i]);
        extract(serializer, reference_frame_selector);
        
        if( !!!serializer )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Controls the source and values used for initial conditions of the navigation solution.
/// 
/// Notes: Initial conditions are the position, velocity, and attitude of the platform used when the filter starts running or is reset.
/// For the user specified position array, the units are meters if the ECEF frame is selected, and degrees latitude, degrees longitude, and meters above ellipsoid if the latitude/longitude/height frame is selected.
/// For the user specified velocity array, the units are meters per second, but the reference frame depends on the reference frame selector (ECEF or NED).
/// 
/// @returns CmdResult
/// 
CmdResult saveInitializationConfiguration(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_INITIALIZATION_CONFIGURATION, buffer, serializer.offset);
}

/// @brief Controls the source and values used for initial conditions of the navigation solution.
/// 
/// Notes: Initial conditions are the position, velocity, and attitude of the platform used when the filter starts running or is reset.
/// For the user specified position array, the units are meters if the ECEF frame is selected, and degrees latitude, degrees longitude, and meters above ellipsoid if the latitude/longitude/height frame is selected.
/// For the user specified velocity array, the units are meters per second, but the reference frame depends on the reference frame selector (ECEF or NED).
/// 
/// @returns CmdResult
/// 
CmdResult loadInitializationConfiguration(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_INITIALIZATION_CONFIGURATION, buffer, serializer.offset);
}

/// @brief Controls the source and values used for initial conditions of the navigation solution.
/// 
/// Notes: Initial conditions are the position, velocity, and attitude of the platform used when the filter starts running or is reset.
/// For the user specified position array, the units are meters if the ECEF frame is selected, and degrees latitude, degrees longitude, and meters above ellipsoid if the latitude/longitude/height frame is selected.
/// For the user specified velocity array, the units are meters per second, but the reference frame depends on the reference frame selector (ECEF or NED).
/// 
/// @returns CmdResult
/// 
CmdResult defaultInitializationConfiguration(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_INITIALIZATION_CONFIGURATION, buffer, serializer.offset);
}

void insert(Serializer& serializer, const AdaptiveFilterOptions& self)
{
    insert(serializer, self.function);
    insert(serializer, self.level);
    insert(serializer, self.time_limit);
}

void extract(Serializer& serializer, AdaptiveFilterOptions& self)
{
    extract(serializer, self.function);
    extract(serializer, self.level);
    extract(serializer, self.time_limit);
}

/// @brief Configures the basic setup for auto-adaptive filtering. See product manual for a detailed description of this feature.
/// 
/// @param level Auto-adaptive operating level: <br/> 0=Off, <br/> 1=Conservative, <br/> 2=Moderate (default), <br/> 3=Aggressive.
/// @param time_limit Maximum duration of measurement rejection before entering recovery mode    (ms)
/// 
/// @returns CmdResult
/// 
CmdResult writeAdaptiveFilterOptions(C::mip_interface& device, uint8_t level, uint16_t time_limit)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, level);
    insert(serializer, time_limit);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ADAPTIVE_FILTER_OPTIONS, buffer, serializer.offset);
}

/// @brief Configures the basic setup for auto-adaptive filtering. See product manual for a detailed description of this feature.
/// 
/// @param[out] level Auto-adaptive operating level: <br/> 0=Off, <br/> 1=Conservative, <br/> 2=Moderate (default), <br/> 3=Aggressive.
/// @param[out] time_limit Maximum duration of measurement rejection before entering recovery mode    (ms)
/// 
/// @returns CmdResult
/// 
CmdResult readAdaptiveFilterOptions(C::mip_interface& device, uint8_t& level, uint16_t& time_limit)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(!!serializer);
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_ADAPTIVE_FILTER_OPTIONS, buffer, serializer.offset, REPLY_ADAPTIVE_FILTER_OPTIONS, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        Serializer serializer(buffer, sizeof(buffer));
        
        extract(serializer, level);
        extract(serializer, time_limit);
        
        if( !!!serializer )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Configures the basic setup for auto-adaptive filtering. See product manual for a detailed description of this feature.
/// 
/// 
/// @returns CmdResult
/// 
CmdResult saveAdaptiveFilterOptions(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ADAPTIVE_FILTER_OPTIONS, buffer, serializer.offset);
}

/// @brief Configures the basic setup for auto-adaptive filtering. See product manual for a detailed description of this feature.
/// 
/// 
/// @returns CmdResult
/// 
CmdResult loadAdaptiveFilterOptions(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ADAPTIVE_FILTER_OPTIONS, buffer, serializer.offset);
}

/// @brief Configures the basic setup for auto-adaptive filtering. See product manual for a detailed description of this feature.
/// 
/// 
/// @returns CmdResult
/// 
CmdResult defaultAdaptiveFilterOptions(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ADAPTIVE_FILTER_OPTIONS, buffer, serializer.offset);
}

void insert(Serializer& serializer, const MultiAntennaOffset& self)
{
    insert(serializer, self.function);
    insert(serializer, self.receiver_id);
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.antenna_offset[i]);
}

void extract(Serializer& serializer, MultiAntennaOffset& self)
{
    extract(serializer, self.function);
    extract(serializer, self.receiver_id);
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.antenna_offset[i]);
}

/// @brief Set the antenna lever arm.
/// 
/// This command works with devices that utilize multiple antennas.
/// @param receiver_id Receiver: 1, 2, etc...
/// @param antenna_offset Antenna lever arm offset vector in the vehicle frame (m)
/// 
/// @returns CmdResult
/// 
CmdResult writeMultiAntennaOffset(C::mip_interface& device, uint8_t receiver_id, const float* antenna_offset)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, receiver_id);
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, antenna_offset[i]);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_MULTI_ANTENNA_OFFSET, buffer, serializer.offset);
}

/// @brief Set the antenna lever arm.
/// 
/// This command works with devices that utilize multiple antennas.
/// @param receiver_id Receiver: 1, 2, etc...
/// @param[out] receiver_id 
/// @param[out] antenna_offset 
/// 
/// @returns CmdResult
/// 
CmdResult readMultiAntennaOffset(C::mip_interface& device, uint8_t receiver_id, float* antenna_offset)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    insert(serializer, receiver_id);
    assert(!!serializer);
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_MULTI_ANTENNA_OFFSET, buffer, serializer.offset, REPLY_MULTI_ANTENNA_OFFSET, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        Serializer serializer(buffer, sizeof(buffer));
        
        extract(serializer, receiver_id);
        for(unsigned int i=0; i < 3; i++)
            extract(serializer, antenna_offset[i]);
        
        if( !!!serializer )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Set the antenna lever arm.
/// 
/// This command works with devices that utilize multiple antennas.
/// @param receiver_id Receiver: 1, 2, etc...
/// 
/// @returns CmdResult
/// 
CmdResult saveMultiAntennaOffset(C::mip_interface& device, uint8_t receiver_id)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    insert(serializer, receiver_id);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_MULTI_ANTENNA_OFFSET, buffer, serializer.offset);
}

/// @brief Set the antenna lever arm.
/// 
/// This command works with devices that utilize multiple antennas.
/// @param receiver_id Receiver: 1, 2, etc...
/// 
/// @returns CmdResult
/// 
CmdResult loadMultiAntennaOffset(C::mip_interface& device, uint8_t receiver_id)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    insert(serializer, receiver_id);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_MULTI_ANTENNA_OFFSET, buffer, serializer.offset);
}

/// @brief Set the antenna lever arm.
/// 
/// This command works with devices that utilize multiple antennas.
/// @param receiver_id Receiver: 1, 2, etc...
/// 
/// @returns CmdResult
/// 
CmdResult defaultMultiAntennaOffset(C::mip_interface& device, uint8_t receiver_id)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    insert(serializer, receiver_id);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_MULTI_ANTENNA_OFFSET, buffer, serializer.offset);
}

void insert(Serializer& serializer, const RelPosConfiguration& self)
{
    insert(serializer, self.function);
    insert(serializer, self.source);
    insert(serializer, self.reference_frame_selector);
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.reference_coordinates[i]);
}

void extract(Serializer& serializer, RelPosConfiguration& self)
{
    extract(serializer, self.function);
    extract(serializer, self.source);
    extract(serializer, self.reference_frame_selector);
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.reference_coordinates[i]);
}

/// @brief Configure the reference location for filter relative positioning outputs
/// 
/// @param source 0 - auto (RTK base station), 1 - manual
/// @param reference_frame_selector ECEF or LLH
/// @param reference_coordinates reference coordinates, units determined by source selection
/// 
/// @returns CmdResult
/// 
CmdResult writeRelPosConfiguration(C::mip_interface& device, uint8_t source, FilterReferenceFrame reference_frame_selector, const double* reference_coordinates)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, source);
    insert(serializer, reference_frame_selector);
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, reference_coordinates[i]);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_REL_POS_CONFIGURATION, buffer, serializer.offset);
}

/// @brief Configure the reference location for filter relative positioning outputs
/// 
/// @param[out] source 0 - auto (RTK base station), 1 - manual
/// @param[out] reference_frame_selector ECEF or LLH
/// @param[out] reference_coordinates reference coordinates, units determined by source selection
/// 
/// @returns CmdResult
/// 
CmdResult readRelPosConfiguration(C::mip_interface& device, uint8_t& source, FilterReferenceFrame& reference_frame_selector, double* reference_coordinates)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(!!serializer);
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_REL_POS_CONFIGURATION, buffer, serializer.offset, REPLY_REL_POS_CONFIGURATION, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        Serializer serializer(buffer, sizeof(buffer));
        
        extract(serializer, source);
        extract(serializer, reference_frame_selector);
        for(unsigned int i=0; i < 3; i++)
            extract(serializer, reference_coordinates[i]);
        
        if( !!!serializer )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Configure the reference location for filter relative positioning outputs
/// 
/// 
/// @returns CmdResult
/// 
CmdResult saveRelPosConfiguration(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_REL_POS_CONFIGURATION, buffer, serializer.offset);
}

/// @brief Configure the reference location for filter relative positioning outputs
/// 
/// 
/// @returns CmdResult
/// 
CmdResult loadRelPosConfiguration(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_REL_POS_CONFIGURATION, buffer, serializer.offset);
}

/// @brief Configure the reference location for filter relative positioning outputs
/// 
/// 
/// @returns CmdResult
/// 
CmdResult defaultRelPosConfiguration(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_REL_POS_CONFIGURATION, buffer, serializer.offset);
}

void insert(Serializer& serializer, const RefPointLeverArm& self)
{
    insert(serializer, self.function);
    insert(serializer, self.ref_point_sel);
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.lever_arm_offset[i]);
}

void extract(Serializer& serializer, RefPointLeverArm& self)
{
    extract(serializer, self.function);
    extract(serializer, self.ref_point_sel);
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.lever_arm_offset[i]);
}

/// @brief Lever arm offset with respect to the sensor for the indicated point of reference.
/// This is used to change the location of the indicated point of reference, and will affect filter position and velocity outputs.
/// Changing this setting from default will result in a global position offset that depends on vehicle attitude,
/// and a velocity offset that depends on vehicle attitude and angular rate.
/// The lever arm is defined by a 3-element vector that points from the sensor to the desired reference point, with (x,y,z) components given in the vehicle's reference frame.
/// Note, if the reference point selector is set to VEH (1), this setting will affect the following data fields: (0x82, 0x01), (0x82, 0x02), (0x82, 0x40), (0x82, 0x41), and (0x82, 42)
/// @param ref_point_sel Reserved, must be 1
/// @param lever_arm_offset [m] Lever arm offset vector in the vehicle's reference frame.
/// 
/// @returns CmdResult
/// 
CmdResult writeRefPointLeverArm(C::mip_interface& device, RefPointLeverArm::ReferencePointSelector ref_point_sel, const float* lever_arm_offset)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, ref_point_sel);
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, lever_arm_offset[i]);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_REF_POINT_LEVER_ARM, buffer, serializer.offset);
}

/// @brief Lever arm offset with respect to the sensor for the indicated point of reference.
/// This is used to change the location of the indicated point of reference, and will affect filter position and velocity outputs.
/// Changing this setting from default will result in a global position offset that depends on vehicle attitude,
/// and a velocity offset that depends on vehicle attitude and angular rate.
/// The lever arm is defined by a 3-element vector that points from the sensor to the desired reference point, with (x,y,z) components given in the vehicle's reference frame.
/// Note, if the reference point selector is set to VEH (1), this setting will affect the following data fields: (0x82, 0x01), (0x82, 0x02), (0x82, 0x40), (0x82, 0x41), and (0x82, 42)
/// @param[out] ref_point_sel Reserved, must be 1
/// @param[out] lever_arm_offset [m] Lever arm offset vector in the vehicle's reference frame.
/// 
/// @returns CmdResult
/// 
CmdResult readRefPointLeverArm(C::mip_interface& device, RefPointLeverArm::ReferencePointSelector& ref_point_sel, float* lever_arm_offset)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(!!serializer);
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_REF_POINT_LEVER_ARM, buffer, serializer.offset, REPLY_REF_POINT_LEVER_ARM, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        Serializer serializer(buffer, sizeof(buffer));
        
        extract(serializer, ref_point_sel);
        for(unsigned int i=0; i < 3; i++)
            extract(serializer, lever_arm_offset[i]);
        
        if( !!!serializer )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Lever arm offset with respect to the sensor for the indicated point of reference.
/// This is used to change the location of the indicated point of reference, and will affect filter position and velocity outputs.
/// Changing this setting from default will result in a global position offset that depends on vehicle attitude,
/// and a velocity offset that depends on vehicle attitude and angular rate.
/// The lever arm is defined by a 3-element vector that points from the sensor to the desired reference point, with (x,y,z) components given in the vehicle's reference frame.
/// Note, if the reference point selector is set to VEH (1), this setting will affect the following data fields: (0x82, 0x01), (0x82, 0x02), (0x82, 0x40), (0x82, 0x41), and (0x82, 42)
/// 
/// @returns CmdResult
/// 
CmdResult saveRefPointLeverArm(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_REF_POINT_LEVER_ARM, buffer, serializer.offset);
}

/// @brief Lever arm offset with respect to the sensor for the indicated point of reference.
/// This is used to change the location of the indicated point of reference, and will affect filter position and velocity outputs.
/// Changing this setting from default will result in a global position offset that depends on vehicle attitude,
/// and a velocity offset that depends on vehicle attitude and angular rate.
/// The lever arm is defined by a 3-element vector that points from the sensor to the desired reference point, with (x,y,z) components given in the vehicle's reference frame.
/// Note, if the reference point selector is set to VEH (1), this setting will affect the following data fields: (0x82, 0x01), (0x82, 0x02), (0x82, 0x40), (0x82, 0x41), and (0x82, 42)
/// 
/// @returns CmdResult
/// 
CmdResult loadRefPointLeverArm(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_REF_POINT_LEVER_ARM, buffer, serializer.offset);
}

/// @brief Lever arm offset with respect to the sensor for the indicated point of reference.
/// This is used to change the location of the indicated point of reference, and will affect filter position and velocity outputs.
/// Changing this setting from default will result in a global position offset that depends on vehicle attitude,
/// and a velocity offset that depends on vehicle attitude and angular rate.
/// The lever arm is defined by a 3-element vector that points from the sensor to the desired reference point, with (x,y,z) components given in the vehicle's reference frame.
/// Note, if the reference point selector is set to VEH (1), this setting will affect the following data fields: (0x82, 0x01), (0x82, 0x02), (0x82, 0x40), (0x82, 0x41), and (0x82, 42)
/// 
/// @returns CmdResult
/// 
CmdResult defaultRefPointLeverArm(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_REF_POINT_LEVER_ARM, buffer, serializer.offset);
}

void insert(Serializer& serializer, const SpeedMeasurement& self)
{
    insert(serializer, self.source);
    insert(serializer, self.time_of_week);
    insert(serializer, self.speed);
    insert(serializer, self.speed_uncertainty);
}

void extract(Serializer& serializer, SpeedMeasurement& self)
{
    extract(serializer, self.source);
    extract(serializer, self.time_of_week);
    extract(serializer, self.speed);
    extract(serializer, self.speed_uncertainty);
}

/// @brief Speed aiding measurement, where speed is defined as rate of motion along the vehicle's x-axis direction.
/// Can be used by an external odometer/speedometer, for example.
/// This command cannot be used if the internal odometer is configured.
/// @param source Reserved, must be 1.
/// @param time_of_week GPS time of week when speed was sampled
/// @param speed Estimated speed along vehicle's x-axis (may be positive or negative) [meters/second]
/// @param speed_uncertainty Estimated uncertainty in the speed measurement (1-sigma value) [meters/second]
/// 
/// @returns CmdResult
/// 
CmdResult speedMeasurement(C::mip_interface& device, uint8_t source, float time_of_week, float speed, float speed_uncertainty)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, source);
    insert(serializer, time_of_week);
    insert(serializer, speed);
    insert(serializer, speed_uncertainty);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SPEED_MEASUREMENT, buffer, serializer.offset);
}

void insert(Serializer& serializer, const SpeedLeverArm& self)
{
    insert(serializer, self.function);
    insert(serializer, self.source);
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.lever_arm_offset[i]);
}

void extract(Serializer& serializer, SpeedLeverArm& self)
{
    extract(serializer, self.function);
    extract(serializer, self.source);
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.lever_arm_offset[i]);
}

/// @brief Lever arm offset for speed measurements.
/// This is used to compensate for an off-center measurement point
/// having a different speed due to rotation of the vehicle.
/// The typical use case for this would be an odometer attached to a wheel
/// on a standard 4-wheeled vehicle. If the odometer is on the left wheel,
/// it will report higher speed on right turns and lower speed on left turns.
/// This is because the outside edge of the curve is longer than the inside edge.
/// @param source Reserved, must be 1.
/// @param lever_arm_offset [m] Lever arm offset vector in the vehicle's reference frame.
/// 
/// @returns CmdResult
/// 
CmdResult writeSpeedLeverArm(C::mip_interface& device, uint8_t source, const float* lever_arm_offset)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, source);
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, lever_arm_offset[i]);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SPEED_LEVER_ARM, buffer, serializer.offset);
}

/// @brief Lever arm offset for speed measurements.
/// This is used to compensate for an off-center measurement point
/// having a different speed due to rotation of the vehicle.
/// The typical use case for this would be an odometer attached to a wheel
/// on a standard 4-wheeled vehicle. If the odometer is on the left wheel,
/// it will report higher speed on right turns and lower speed on left turns.
/// This is because the outside edge of the curve is longer than the inside edge.
/// @param source Reserved, must be 1.
/// @param[out] source Reserved, must be 1.
/// @param[out] lever_arm_offset [m] Lever arm offset vector in the vehicle's reference frame.
/// 
/// @returns CmdResult
/// 
CmdResult readSpeedLeverArm(C::mip_interface& device, uint8_t source, float* lever_arm_offset)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    insert(serializer, source);
    assert(!!serializer);
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_SPEED_LEVER_ARM, buffer, serializer.offset, REPLY_SPEED_LEVER_ARM, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        Serializer serializer(buffer, sizeof(buffer));
        
        extract(serializer, source);
        for(unsigned int i=0; i < 3; i++)
            extract(serializer, lever_arm_offset[i]);
        
        if( !!!serializer )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Lever arm offset for speed measurements.
/// This is used to compensate for an off-center measurement point
/// having a different speed due to rotation of the vehicle.
/// The typical use case for this would be an odometer attached to a wheel
/// on a standard 4-wheeled vehicle. If the odometer is on the left wheel,
/// it will report higher speed on right turns and lower speed on left turns.
/// This is because the outside edge of the curve is longer than the inside edge.
/// @param source Reserved, must be 1.
/// 
/// @returns CmdResult
/// 
CmdResult saveSpeedLeverArm(C::mip_interface& device, uint8_t source)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    insert(serializer, source);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SPEED_LEVER_ARM, buffer, serializer.offset);
}

/// @brief Lever arm offset for speed measurements.
/// This is used to compensate for an off-center measurement point
/// having a different speed due to rotation of the vehicle.
/// The typical use case for this would be an odometer attached to a wheel
/// on a standard 4-wheeled vehicle. If the odometer is on the left wheel,
/// it will report higher speed on right turns and lower speed on left turns.
/// This is because the outside edge of the curve is longer than the inside edge.
/// @param source Reserved, must be 1.
/// 
/// @returns CmdResult
/// 
CmdResult loadSpeedLeverArm(C::mip_interface& device, uint8_t source)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    insert(serializer, source);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SPEED_LEVER_ARM, buffer, serializer.offset);
}

/// @brief Lever arm offset for speed measurements.
/// This is used to compensate for an off-center measurement point
/// having a different speed due to rotation of the vehicle.
/// The typical use case for this would be an odometer attached to a wheel
/// on a standard 4-wheeled vehicle. If the odometer is on the left wheel,
/// it will report higher speed on right turns and lower speed on left turns.
/// This is because the outside edge of the curve is longer than the inside edge.
/// @param source Reserved, must be 1.
/// 
/// @returns CmdResult
/// 
CmdResult defaultSpeedLeverArm(C::mip_interface& device, uint8_t source)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    insert(serializer, source);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SPEED_LEVER_ARM, buffer, serializer.offset);
}

void insert(Serializer& serializer, const WheeledVehicleConstraintControl& self)
{
    insert(serializer, self.function);
    insert(serializer, self.enable);
}

void extract(Serializer& serializer, WheeledVehicleConstraintControl& self)
{
    extract(serializer, self.function);
    extract(serializer, self.enable);
}

/// @brief Configure the wheeled vehicle kinematic constraint.
/// 
/// When enabled, the filter uses the assumption that velocity is constrained to the primary vehicle axis.
/// By convention, the primary vehicle axis is the vehicle X-axis (note: the sensor may be physically installed in
/// any orientation on the vehicle if the appropriate mounting transformation has been specified).
/// This constraint will typically improve heading estimates for vehicles where the assumption is valid, such
/// as an automobile, particulary when GNSS coverage is intermittent.
/// @param enable 0 - Disable, 1 - Enable
/// 
/// @returns CmdResult
/// 
CmdResult writeWheeledVehicleConstraintControl(C::mip_interface& device, uint8_t enable)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, enable);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_VEHICLE_CONSTRAINT_CONTROL, buffer, serializer.offset);
}

/// @brief Configure the wheeled vehicle kinematic constraint.
/// 
/// When enabled, the filter uses the assumption that velocity is constrained to the primary vehicle axis.
/// By convention, the primary vehicle axis is the vehicle X-axis (note: the sensor may be physically installed in
/// any orientation on the vehicle if the appropriate mounting transformation has been specified).
/// This constraint will typically improve heading estimates for vehicles where the assumption is valid, such
/// as an automobile, particulary when GNSS coverage is intermittent.
/// @param[out] enable 0 - Disable, 1 - Enable
/// 
/// @returns CmdResult
/// 
CmdResult readWheeledVehicleConstraintControl(C::mip_interface& device, uint8_t& enable)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(!!serializer);
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_VEHICLE_CONSTRAINT_CONTROL, buffer, serializer.offset, REPLY_VEHICLE_CONSTRAINT_CONTROL, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        Serializer serializer(buffer, sizeof(buffer));
        
        extract(serializer, enable);
        
        if( !!!serializer )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Configure the wheeled vehicle kinematic constraint.
/// 
/// When enabled, the filter uses the assumption that velocity is constrained to the primary vehicle axis.
/// By convention, the primary vehicle axis is the vehicle X-axis (note: the sensor may be physically installed in
/// any orientation on the vehicle if the appropriate mounting transformation has been specified).
/// This constraint will typically improve heading estimates for vehicles where the assumption is valid, such
/// as an automobile, particulary when GNSS coverage is intermittent.
/// 
/// @returns CmdResult
/// 
CmdResult saveWheeledVehicleConstraintControl(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_VEHICLE_CONSTRAINT_CONTROL, buffer, serializer.offset);
}

/// @brief Configure the wheeled vehicle kinematic constraint.
/// 
/// When enabled, the filter uses the assumption that velocity is constrained to the primary vehicle axis.
/// By convention, the primary vehicle axis is the vehicle X-axis (note: the sensor may be physically installed in
/// any orientation on the vehicle if the appropriate mounting transformation has been specified).
/// This constraint will typically improve heading estimates for vehicles where the assumption is valid, such
/// as an automobile, particulary when GNSS coverage is intermittent.
/// 
/// @returns CmdResult
/// 
CmdResult loadWheeledVehicleConstraintControl(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_VEHICLE_CONSTRAINT_CONTROL, buffer, serializer.offset);
}

/// @brief Configure the wheeled vehicle kinematic constraint.
/// 
/// When enabled, the filter uses the assumption that velocity is constrained to the primary vehicle axis.
/// By convention, the primary vehicle axis is the vehicle X-axis (note: the sensor may be physically installed in
/// any orientation on the vehicle if the appropriate mounting transformation has been specified).
/// This constraint will typically improve heading estimates for vehicles where the assumption is valid, such
/// as an automobile, particulary when GNSS coverage is intermittent.
/// 
/// @returns CmdResult
/// 
CmdResult defaultWheeledVehicleConstraintControl(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_VEHICLE_CONSTRAINT_CONTROL, buffer, serializer.offset);
}

void insert(Serializer& serializer, const VerticalGyroConstraintControl& self)
{
    insert(serializer, self.function);
    insert(serializer, self.enable);
}

void extract(Serializer& serializer, VerticalGyroConstraintControl& self)
{
    extract(serializer, self.function);
    extract(serializer, self.enable);
}

/// @brief Configure the vertical gyro kinematic constraint.
/// 
/// When enabled and no valid GNSS measurements are available, the filter uses the accelerometers to track pitch
/// and roll under the assumption that the sensor platform is not undergoing linear acceleration.
/// This constraint is useful to maintain accurate pitch and roll during GNSS signal outages.
/// @param enable 0 - Disable, 1 - Enable
/// 
/// @returns CmdResult
/// 
CmdResult writeVerticalGyroConstraintControl(C::mip_interface& device, uint8_t enable)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, enable);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GYRO_CONSTRAINT_CONTROL, buffer, serializer.offset);
}

/// @brief Configure the vertical gyro kinematic constraint.
/// 
/// When enabled and no valid GNSS measurements are available, the filter uses the accelerometers to track pitch
/// and roll under the assumption that the sensor platform is not undergoing linear acceleration.
/// This constraint is useful to maintain accurate pitch and roll during GNSS signal outages.
/// @param[out] enable 0 - Disable, 1 - Enable
/// 
/// @returns CmdResult
/// 
CmdResult readVerticalGyroConstraintControl(C::mip_interface& device, uint8_t& enable)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(!!serializer);
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GYRO_CONSTRAINT_CONTROL, buffer, serializer.offset, REPLY_GYRO_CONSTRAINT_CONTROL, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        Serializer serializer(buffer, sizeof(buffer));
        
        extract(serializer, enable);
        
        if( !!!serializer )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Configure the vertical gyro kinematic constraint.
/// 
/// When enabled and no valid GNSS measurements are available, the filter uses the accelerometers to track pitch
/// and roll under the assumption that the sensor platform is not undergoing linear acceleration.
/// This constraint is useful to maintain accurate pitch and roll during GNSS signal outages.
/// 
/// @returns CmdResult
/// 
CmdResult saveVerticalGyroConstraintControl(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GYRO_CONSTRAINT_CONTROL, buffer, serializer.offset);
}

/// @brief Configure the vertical gyro kinematic constraint.
/// 
/// When enabled and no valid GNSS measurements are available, the filter uses the accelerometers to track pitch
/// and roll under the assumption that the sensor platform is not undergoing linear acceleration.
/// This constraint is useful to maintain accurate pitch and roll during GNSS signal outages.
/// 
/// @returns CmdResult
/// 
CmdResult loadVerticalGyroConstraintControl(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GYRO_CONSTRAINT_CONTROL, buffer, serializer.offset);
}

/// @brief Configure the vertical gyro kinematic constraint.
/// 
/// When enabled and no valid GNSS measurements are available, the filter uses the accelerometers to track pitch
/// and roll under the assumption that the sensor platform is not undergoing linear acceleration.
/// This constraint is useful to maintain accurate pitch and roll during GNSS signal outages.
/// 
/// @returns CmdResult
/// 
CmdResult defaultVerticalGyroConstraintControl(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GYRO_CONSTRAINT_CONTROL, buffer, serializer.offset);
}

void insert(Serializer& serializer, const GnssAntennaCalControl& self)
{
    insert(serializer, self.function);
    insert(serializer, self.enable);
    insert(serializer, self.max_offset);
}

void extract(Serializer& serializer, GnssAntennaCalControl& self)
{
    extract(serializer, self.function);
    extract(serializer, self.enable);
    extract(serializer, self.max_offset);
}

/// @brief Configure the GNSS antenna lever arm calibration.
/// 
/// When enabled, the filter will enable lever arm error tracking, up to the maximum offset specified.
/// @param enable 0 - Disable, 1 - Enable
/// @param max_offset Maximum absolute value of lever arm offset error in the vehicle frame [meters]. See device user manual for the valid range of this parameter.
/// 
/// @returns CmdResult
/// 
CmdResult writeGnssAntennaCalControl(C::mip_interface& device, uint8_t enable, float max_offset)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, enable);
    insert(serializer, max_offset);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ANTENNA_CALIBRATION_CONTROL, buffer, serializer.offset);
}

/// @brief Configure the GNSS antenna lever arm calibration.
/// 
/// When enabled, the filter will enable lever arm error tracking, up to the maximum offset specified.
/// @param[out] enable 0 - Disable, 1 - Enable
/// @param[out] max_offset Maximum absolute value of lever arm offset error in the vehicle frame [meters]. See device user manual for the valid range of this parameter.
/// 
/// @returns CmdResult
/// 
CmdResult readGnssAntennaCalControl(C::mip_interface& device, uint8_t& enable, float& max_offset)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(!!serializer);
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_ANTENNA_CALIBRATION_CONTROL, buffer, serializer.offset, REPLY_ANTENNA_CALIBRATION_CONTROL, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        Serializer serializer(buffer, sizeof(buffer));
        
        extract(serializer, enable);
        extract(serializer, max_offset);
        
        if( !!!serializer )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Configure the GNSS antenna lever arm calibration.
/// 
/// When enabled, the filter will enable lever arm error tracking, up to the maximum offset specified.
/// 
/// @returns CmdResult
/// 
CmdResult saveGnssAntennaCalControl(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ANTENNA_CALIBRATION_CONTROL, buffer, serializer.offset);
}

/// @brief Configure the GNSS antenna lever arm calibration.
/// 
/// When enabled, the filter will enable lever arm error tracking, up to the maximum offset specified.
/// 
/// @returns CmdResult
/// 
CmdResult loadGnssAntennaCalControl(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ANTENNA_CALIBRATION_CONTROL, buffer, serializer.offset);
}

/// @brief Configure the GNSS antenna lever arm calibration.
/// 
/// When enabled, the filter will enable lever arm error tracking, up to the maximum offset specified.
/// 
/// @returns CmdResult
/// 
CmdResult defaultGnssAntennaCalControl(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ANTENNA_CALIBRATION_CONTROL, buffer, serializer.offset);
}

void insert(Serializer& serializer, const MagneticDeclinationSource& self)
{
    insert(serializer, self.function);
    insert(serializer, self.source);
    insert(serializer, self.declination);
}

void extract(Serializer& serializer, MagneticDeclinationSource& self)
{
    extract(serializer, self.function);
    extract(serializer, self.source);
    extract(serializer, self.declination);
}

/// @brief Source for magnetic declination angle, and user supplied value for manual selection.
/// 
/// @param source Magnetic field declination angle source
/// @param declination Declination angle used when 'source' is set to 'MANUAL' (radians)
/// 
/// @returns CmdResult
/// 
CmdResult writeMagneticDeclinationSource(C::mip_interface& device, FilterMagDeclinationSource source, float declination)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, source);
    insert(serializer, declination);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_DECLINATION_SOURCE, buffer, serializer.offset);
}

/// @brief Source for magnetic declination angle, and user supplied value for manual selection.
/// 
/// @param[out] source Magnetic field declination angle source
/// @param[out] declination Declination angle used when 'source' is set to 'MANUAL' (radians)
/// 
/// @returns CmdResult
/// 
CmdResult readMagneticDeclinationSource(C::mip_interface& device, FilterMagDeclinationSource& source, float& declination)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(!!serializer);
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_DECLINATION_SOURCE, buffer, serializer.offset, REPLY_DECLINATION_SOURCE, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        Serializer serializer(buffer, sizeof(buffer));
        
        extract(serializer, source);
        extract(serializer, declination);
        
        if( !!!serializer )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Source for magnetic declination angle, and user supplied value for manual selection.
/// 
/// 
/// @returns CmdResult
/// 
CmdResult saveMagneticDeclinationSource(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_DECLINATION_SOURCE, buffer, serializer.offset);
}

/// @brief Source for magnetic declination angle, and user supplied value for manual selection.
/// 
/// 
/// @returns CmdResult
/// 
CmdResult loadMagneticDeclinationSource(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_DECLINATION_SOURCE, buffer, serializer.offset);
}

/// @brief Source for magnetic declination angle, and user supplied value for manual selection.
/// 
/// 
/// @returns CmdResult
/// 
CmdResult defaultMagneticDeclinationSource(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_DECLINATION_SOURCE, buffer, serializer.offset);
}

void insert(Serializer& serializer, const SetInitialHeading& self)
{
    insert(serializer, self.heading);
}

void extract(Serializer& serializer, SetInitialHeading& self)
{
    extract(serializer, self.heading);
}

/// @brief Set the initial heading angle.
/// 
/// The estimation filter will reset the heading estimate to provided value. If the product supports magnetometer aiding and this feature has been enabled, the heading
/// argument will be ignored and the filter will initialize using the inferred magnetic heading.
/// @param heading Initial heading in radians [-pi, pi]
/// 
/// @returns CmdResult
/// 
CmdResult setInitialHeading(C::mip_interface& device, float heading)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, heading);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SET_INITIAL_HEADING, buffer, serializer.offset);
}


} // namespace commands_filter
} // namespace mip

