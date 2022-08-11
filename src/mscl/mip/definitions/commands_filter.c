
#include "commands_filter.h"

#include "../../utils/serialization.h"
#include "../mip_interface.h"

#include <assert.h>


#ifdef __cplusplus
namespace mip {
namespace C {
extern "C" {

#endif // __cplusplus
struct mip_interface;
struct mip_serializer;


////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////

void insert_mip_filter_reference_frame(struct mip_serializer* serializer, const enum mip_filter_reference_frame self)
{
    return insert_u8(serializer, (uint8_t)(self));
}
void extract_mip_filter_reference_frame(struct mip_serializer* serializer, enum mip_filter_reference_frame* self)
{
    uint8_t tmp = 0;
    extract_u8(serializer, &tmp);
    *self = tmp;
}

void insert_mip_filter_mag_declination_source(struct mip_serializer* serializer, const enum mip_filter_mag_declination_source self)
{
    return insert_u8(serializer, (uint8_t)(self));
}
void extract_mip_filter_mag_declination_source(struct mip_serializer* serializer, enum mip_filter_mag_declination_source* self)
{
    uint8_t tmp = 0;
    extract_u8(serializer, &tmp);
    *self = tmp;
}


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

/// @brief Resets the filter to the initialization state.
/// 
/// If the auto-initialization feature is disabled, the initial attitude or heading must be set in
/// order to enter the run state after a reset.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_reset(struct mip_interface* device)
{
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_RESET_FILTER, NULL, 0);
}

void insert_mip_filter_set_initial_attitude_command(struct mip_serializer* serializer, const struct mip_filter_set_initial_attitude_command* self)
{
    insert_float(serializer, self->roll);
    insert_float(serializer, self->pitch);
    insert_float(serializer, self->heading);
}

void extract_mip_filter_set_initial_attitude_command(struct mip_serializer* serializer, struct mip_filter_set_initial_attitude_command* self)
{
    extract_float(serializer, &self->roll);
    extract_float(serializer, &self->pitch);
    extract_float(serializer, &self->heading);
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
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_set_initial_attitude(struct mip_interface* device, float roll, float pitch, float heading)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_float(&serializer, roll);
    insert_float(&serializer, pitch);
    insert_float(&serializer, heading);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SET_INITIAL_ATTITUDE, buffer, serializer.offset);
}

void insert_mip_filter_estimation_control_command(struct mip_serializer* serializer, const struct mip_filter_estimation_control_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    insert_mip_filter_estimation_control_command_enable_flags(serializer, self->enable);
}

void extract_mip_filter_estimation_control_command(struct mip_serializer* serializer, struct mip_filter_estimation_control_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    extract_mip_filter_estimation_control_command_enable_flags(serializer, &self->enable);
}

void insert_mip_filter_estimation_control_command_enable_flags(struct mip_serializer* serializer, const enum mip_filter_estimation_control_command_enable_flags self)
{
    return insert_u16(serializer, (uint16_t)(self));
}
void extract_mip_filter_estimation_control_command_enable_flags(struct mip_serializer* serializer, enum mip_filter_estimation_control_command_enable_flags* self)
{
    uint16_t tmp = 0;
    extract_u16(serializer, &tmp);
    *self = tmp;
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
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_write_estimation_control(struct mip_interface* device, enum mip_filter_estimation_control_command_enable_flags enable)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    insert_mip_filter_estimation_control_command_enable_flags(&serializer, enable);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ESTIMATION_CONTROL_FLAGS, buffer, serializer.offset);
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
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_read_estimation_control(struct mip_interface* device, enum mip_filter_estimation_control_command_enable_flags* enable)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ESTIMATION_CONTROL_FLAGS, buffer, serializer.offset, MIP_REPLY_DESC_FILTER_ESTIMATION_CONTROL_FLAGS, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        struct mip_serializer serializer;
        mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
        
        extract_mip_filter_estimation_control_command_enable_flags(&serializer, enable);
        
        if( !mip_serializer_ok(&serializer) )
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
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_save_estimation_control(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ESTIMATION_CONTROL_FLAGS, buffer, serializer.offset);
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
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_load_estimation_control(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ESTIMATION_CONTROL_FLAGS, buffer, serializer.offset);
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
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_default_estimation_control(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ESTIMATION_CONTROL_FLAGS, buffer, serializer.offset);
}

void insert_mip_filter_external_gnss_update_command(struct mip_serializer* serializer, const struct mip_filter_external_gnss_update_command* self)
{
    insert_double(serializer, self->gps_time);
    insert_u16(serializer, self->gps_week);
    insert_double(serializer, self->latitude);
    insert_double(serializer, self->longitude);
    insert_double(serializer, self->height);
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->velocity[i]);
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->pos_uncertainty[i]);
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->vel_uncertainty[i]);
}

void extract_mip_filter_external_gnss_update_command(struct mip_serializer* serializer, struct mip_filter_external_gnss_update_command* self)
{
    extract_double(serializer, &self->gps_time);
    extract_u16(serializer, &self->gps_week);
    extract_double(serializer, &self->latitude);
    extract_double(serializer, &self->longitude);
    extract_double(serializer, &self->height);
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->velocity[i]);
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->pos_uncertainty[i]);
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->vel_uncertainty[i]);
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
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_external_gnss_update(struct mip_interface* device, double gps_time, uint16_t gps_week, double latitude, double longitude, double height, const float* velocity, const float* pos_uncertainty, const float* vel_uncertainty)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_double(&serializer, gps_time);
    insert_u16(&serializer, gps_week);
    insert_double(&serializer, latitude);
    insert_double(&serializer, longitude);
    insert_double(&serializer, height);
    for(unsigned int i=0; i < 3; i++)
        insert_float(&serializer, velocity[i]);
    for(unsigned int i=0; i < 3; i++)
        insert_float(&serializer, pos_uncertainty[i]);
    for(unsigned int i=0; i < 3; i++)
        insert_float(&serializer, vel_uncertainty[i]);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_EXTERNAL_GNSS_UPDATE, buffer, serializer.offset);
}

void insert_mip_filter_external_heading_update_command(struct mip_serializer* serializer, const struct mip_filter_external_heading_update_command* self)
{
    insert_float(serializer, self->heading);
    insert_float(serializer, self->heading_uncertainty);
    insert_u8(serializer, self->type);
}

void extract_mip_filter_external_heading_update_command(struct mip_serializer* serializer, struct mip_filter_external_heading_update_command* self)
{
    extract_float(serializer, &self->heading);
    extract_float(serializer, &self->heading_uncertainty);
    extract_u8(serializer, &self->type);
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
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_external_heading_update(struct mip_interface* device, float heading, float heading_uncertainty, uint8_t type)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_float(&serializer, heading);
    insert_float(&serializer, heading_uncertainty);
    insert_u8(&serializer, type);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_EXTERNAL_HEADING_UPDATE, buffer, serializer.offset);
}

void insert_mip_filter_external_heading_update_with_time_command(struct mip_serializer* serializer, const struct mip_filter_external_heading_update_with_time_command* self)
{
    insert_double(serializer, self->gps_time);
    insert_u16(serializer, self->gps_week);
    insert_float(serializer, self->heading);
    insert_float(serializer, self->heading_uncertainty);
    insert_u8(serializer, self->type);
}

void extract_mip_filter_external_heading_update_with_time_command(struct mip_serializer* serializer, struct mip_filter_external_heading_update_with_time_command* self)
{
    extract_double(serializer, &self->gps_time);
    extract_u16(serializer, &self->gps_week);
    extract_float(serializer, &self->heading);
    extract_float(serializer, &self->heading_uncertainty);
    extract_u8(serializer, &self->type);
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
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_external_heading_update_with_time(struct mip_interface* device, double gps_time, uint16_t gps_week, float heading, float heading_uncertainty, uint8_t type)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_double(&serializer, gps_time);
    insert_u16(&serializer, gps_week);
    insert_float(&serializer, heading);
    insert_float(&serializer, heading_uncertainty);
    insert_u8(&serializer, type);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_EXTERNAL_HEADING_UPDATE_WITH_TIME, buffer, serializer.offset);
}

void insert_mip_filter_tare_orientation_command(struct mip_serializer* serializer, const struct mip_filter_tare_orientation_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    insert_mip_filter_tare_orientation_command_mip_tare_axes(serializer, self->axes);
}

void extract_mip_filter_tare_orientation_command(struct mip_serializer* serializer, struct mip_filter_tare_orientation_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    extract_mip_filter_tare_orientation_command_mip_tare_axes(serializer, &self->axes);
}

void insert_mip_filter_tare_orientation_command_mip_tare_axes(struct mip_serializer* serializer, const enum mip_filter_tare_orientation_command_mip_tare_axes self)
{
    return insert_u8(serializer, (uint8_t)(self));
}
void extract_mip_filter_tare_orientation_command_mip_tare_axes(struct mip_serializer* serializer, enum mip_filter_tare_orientation_command_mip_tare_axes* self)
{
    uint8_t tmp = 0;
    extract_u8(serializer, &tmp);
    *self = tmp;
}

/// @brief Tare the device orientation.
/// 
/// This function uses the current device orientation relative to the NED frame as the current sensor to vehicle transformation.
/// This command is provided as a convenient way to set the sensor to vehicle frame transformation.
/// The filter must be initialized and have a valid attitude output. If the attitude is not valid, an error will be returned.
/// @param axes Axes to tare
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_write_tare_orientation(struct mip_interface* device, enum mip_filter_tare_orientation_command_mip_tare_axes axes)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    insert_mip_filter_tare_orientation_command_mip_tare_axes(&serializer, axes);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_TARE_ORIENTATION, buffer, serializer.offset);
}

/// @brief Tare the device orientation.
/// 
/// This function uses the current device orientation relative to the NED frame as the current sensor to vehicle transformation.
/// This command is provided as a convenient way to set the sensor to vehicle frame transformation.
/// The filter must be initialized and have a valid attitude output. If the attitude is not valid, an error will be returned.
/// @param[out] axes Axes to tare
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_read_tare_orientation(struct mip_interface* device, enum mip_filter_tare_orientation_command_mip_tare_axes* axes)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_TARE_ORIENTATION, buffer, serializer.offset, MIP_REPLY_DESC_FILTER_TARE_ORIENTATION, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        struct mip_serializer serializer;
        mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
        
        extract_mip_filter_tare_orientation_command_mip_tare_axes(&serializer, axes);
        
        if( !mip_serializer_ok(&serializer) )
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
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_save_tare_orientation(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_TARE_ORIENTATION, buffer, serializer.offset);
}

/// @brief Tare the device orientation.
/// 
/// This function uses the current device orientation relative to the NED frame as the current sensor to vehicle transformation.
/// This command is provided as a convenient way to set the sensor to vehicle frame transformation.
/// The filter must be initialized and have a valid attitude output. If the attitude is not valid, an error will be returned.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_load_tare_orientation(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_TARE_ORIENTATION, buffer, serializer.offset);
}

/// @brief Tare the device orientation.
/// 
/// This function uses the current device orientation relative to the NED frame as the current sensor to vehicle transformation.
/// This command is provided as a convenient way to set the sensor to vehicle frame transformation.
/// The filter must be initialized and have a valid attitude output. If the attitude is not valid, an error will be returned.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_default_tare_orientation(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_TARE_ORIENTATION, buffer, serializer.offset);
}

void insert_mip_filter_sensor_to_vehicle_rotation_euler_command(struct mip_serializer* serializer, const struct mip_filter_sensor_to_vehicle_rotation_euler_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    insert_float(serializer, self->roll);
    insert_float(serializer, self->pitch);
    insert_float(serializer, self->yaw);
}

void extract_mip_filter_sensor_to_vehicle_rotation_euler_command(struct mip_serializer* serializer, struct mip_filter_sensor_to_vehicle_rotation_euler_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    extract_float(serializer, &self->roll);
    extract_float(serializer, &self->pitch);
    extract_float(serializer, &self->yaw);
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
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_write_sensor_to_vehicle_rotation_euler(struct mip_interface* device, float roll, float pitch, float yaw)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    insert_float(&serializer, roll);
    insert_float(&serializer, pitch);
    insert_float(&serializer, yaw);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_ROTATION_EULER, buffer, serializer.offset);
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
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_read_sensor_to_vehicle_rotation_euler(struct mip_interface* device, float* roll, float* pitch, float* yaw)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_ROTATION_EULER, buffer, serializer.offset, MIP_REPLY_DESC_FILTER_SENSOR2VEHICLE_ROTATION_EULER, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        struct mip_serializer serializer;
        mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
        
        extract_float(&serializer, roll);
        extract_float(&serializer, pitch);
        extract_float(&serializer, yaw);
        
        if( !mip_serializer_ok(&serializer) )
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
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_save_sensor_to_vehicle_rotation_euler(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_ROTATION_EULER, buffer, serializer.offset);
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
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_load_sensor_to_vehicle_rotation_euler(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_ROTATION_EULER, buffer, serializer.offset);
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
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_default_sensor_to_vehicle_rotation_euler(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_ROTATION_EULER, buffer, serializer.offset);
}

void insert_mip_filter_sensor_to_vehicle_rotation_dcm_command(struct mip_serializer* serializer, const struct mip_filter_sensor_to_vehicle_rotation_dcm_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    for(unsigned int i=0; i < 9; i++)
        insert_float(serializer, self->dcm[i]);
}

void extract_mip_filter_sensor_to_vehicle_rotation_dcm_command(struct mip_serializer* serializer, struct mip_filter_sensor_to_vehicle_rotation_dcm_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    for(unsigned int i=0; i < 9; i++)
        extract_float(serializer, &self->dcm[i]);
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
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_write_sensor_to_vehicle_rotation_dcm(struct mip_interface* device, const float* dcm)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    for(unsigned int i=0; i < 9; i++)
        insert_float(&serializer, dcm[i]);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_ROTATION_DCM, buffer, serializer.offset);
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
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_read_sensor_to_vehicle_rotation_dcm(struct mip_interface* device, float* dcm)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_ROTATION_DCM, buffer, serializer.offset, MIP_REPLY_DESC_FILTER_SENSOR2VEHICLE_ROTATION_DCM, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        struct mip_serializer serializer;
        mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
        
        for(unsigned int i=0; i < 9; i++)
            extract_float(&serializer, &dcm[i]);
        
        if( !mip_serializer_ok(&serializer) )
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
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_save_sensor_to_vehicle_rotation_dcm(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_ROTATION_DCM, buffer, serializer.offset);
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
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_load_sensor_to_vehicle_rotation_dcm(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_ROTATION_DCM, buffer, serializer.offset);
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
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_default_sensor_to_vehicle_rotation_dcm(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_ROTATION_DCM, buffer, serializer.offset);
}

void insert_mip_filter_sensor_to_vehicle_rotation_quaternion_command(struct mip_serializer* serializer, const struct mip_filter_sensor_to_vehicle_rotation_quaternion_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    for(unsigned int i=0; i < 4; i++)
        insert_float(serializer, self->quat[i]);
}

void extract_mip_filter_sensor_to_vehicle_rotation_quaternion_command(struct mip_serializer* serializer, struct mip_filter_sensor_to_vehicle_rotation_quaternion_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    for(unsigned int i=0; i < 4; i++)
        extract_float(serializer, &self->quat[i]);
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
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_write_sensor_to_vehicle_rotation_quaternion(struct mip_interface* device, const float* quat)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    for(unsigned int i=0; i < 4; i++)
        insert_float(&serializer, quat[i]);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_ROTATION_QUATERNION, buffer, serializer.offset);
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
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_read_sensor_to_vehicle_rotation_quaternion(struct mip_interface* device, float* quat)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_ROTATION_QUATERNION, buffer, serializer.offset, MIP_REPLY_DESC_FILTER_SENSOR2VEHICLE_ROTATION_QUATERNION, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        struct mip_serializer serializer;
        mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
        
        for(unsigned int i=0; i < 4; i++)
            extract_float(&serializer, &quat[i]);
        
        if( !mip_serializer_ok(&serializer) )
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
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_save_sensor_to_vehicle_rotation_quaternion(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_ROTATION_QUATERNION, buffer, serializer.offset);
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
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_load_sensor_to_vehicle_rotation_quaternion(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_ROTATION_QUATERNION, buffer, serializer.offset);
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
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_default_sensor_to_vehicle_rotation_quaternion(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_ROTATION_QUATERNION, buffer, serializer.offset);
}

void insert_mip_filter_sensor_to_vehicle_offset_command(struct mip_serializer* serializer, const struct mip_filter_sensor_to_vehicle_offset_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->offset[i]);
}

void extract_mip_filter_sensor_to_vehicle_offset_command(struct mip_serializer* serializer, struct mip_filter_sensor_to_vehicle_offset_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->offset[i]);
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
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_write_sensor_to_vehicle_offset(struct mip_interface* device, const float* offset)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    for(unsigned int i=0; i < 3; i++)
        insert_float(&serializer, offset[i]);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_OFFSET, buffer, serializer.offset);
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
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_read_sensor_to_vehicle_offset(struct mip_interface* device, float* offset)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_OFFSET, buffer, serializer.offset, MIP_REPLY_DESC_FILTER_SENSOR2VEHICLE_OFFSET, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        struct mip_serializer serializer;
        mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
        
        for(unsigned int i=0; i < 3; i++)
            extract_float(&serializer, &offset[i]);
        
        if( !mip_serializer_ok(&serializer) )
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
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_save_sensor_to_vehicle_offset(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_OFFSET, buffer, serializer.offset);
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
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_load_sensor_to_vehicle_offset(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_OFFSET, buffer, serializer.offset);
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
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_default_sensor_to_vehicle_offset(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_OFFSET, buffer, serializer.offset);
}

void insert_mip_filter_antenna_offset_command(struct mip_serializer* serializer, const struct mip_filter_antenna_offset_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->offset[i]);
}

void extract_mip_filter_antenna_offset_command(struct mip_serializer* serializer, struct mip_filter_antenna_offset_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->offset[i]);
}

/// @brief Set the sensor to GNSS antenna offset.
/// 
/// This is expressed in the sensor frame, from the sensor origin to the GNSS antenna RF center.
/// 
/// The magnitude of the offset vector is limited to 10 meters
/// 
/// @param offset [meters]
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_write_antenna_offset(struct mip_interface* device, const float* offset)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    for(unsigned int i=0; i < 3; i++)
        insert_float(&serializer, offset[i]);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ANTENNA_OFFSET, buffer, serializer.offset);
}

/// @brief Set the sensor to GNSS antenna offset.
/// 
/// This is expressed in the sensor frame, from the sensor origin to the GNSS antenna RF center.
/// 
/// The magnitude of the offset vector is limited to 10 meters
/// 
/// @param[out] offset [meters]
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_read_antenna_offset(struct mip_interface* device, float* offset)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ANTENNA_OFFSET, buffer, serializer.offset, MIP_REPLY_DESC_FILTER_ANTENNA_OFFSET, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        struct mip_serializer serializer;
        mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
        
        for(unsigned int i=0; i < 3; i++)
            extract_float(&serializer, &offset[i]);
        
        if( !mip_serializer_ok(&serializer) )
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
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_save_antenna_offset(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ANTENNA_OFFSET, buffer, serializer.offset);
}

/// @brief Set the sensor to GNSS antenna offset.
/// 
/// This is expressed in the sensor frame, from the sensor origin to the GNSS antenna RF center.
/// 
/// The magnitude of the offset vector is limited to 10 meters
/// 
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_load_antenna_offset(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ANTENNA_OFFSET, buffer, serializer.offset);
}

/// @brief Set the sensor to GNSS antenna offset.
/// 
/// This is expressed in the sensor frame, from the sensor origin to the GNSS antenna RF center.
/// 
/// The magnitude of the offset vector is limited to 10 meters
/// 
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_default_antenna_offset(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ANTENNA_OFFSET, buffer, serializer.offset);
}

void insert_mip_filter_gnss_source_command(struct mip_serializer* serializer, const struct mip_filter_gnss_source_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    insert_mip_filter_gnss_source_command_source(serializer, self->source);
}

void extract_mip_filter_gnss_source_command(struct mip_serializer* serializer, struct mip_filter_gnss_source_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    extract_mip_filter_gnss_source_command_source(serializer, &self->source);
}

void insert_mip_filter_gnss_source_command_source(struct mip_serializer* serializer, const enum mip_filter_gnss_source_command_source self)
{
    return insert_u8(serializer, (uint8_t)(self));
}
void extract_mip_filter_gnss_source_command_source(struct mip_serializer* serializer, enum mip_filter_gnss_source_command_source* self)
{
    uint8_t tmp = 0;
    extract_u8(serializer, &tmp);
    *self = tmp;
}

/// @brief Control the source of GNSS information used to update the Kalman Filter.
/// 
/// Changing the GNSS source while the sensor is in the "running" state may temporarily place
/// it back in the "init" state until the new source of GNSS data is received.
/// 
/// @param source 
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_write_gnss_source(struct mip_interface* device, enum mip_filter_gnss_source_command_source source)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    insert_mip_filter_gnss_source_command_source(&serializer, source);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_GNSS_SOURCE_CONTROL, buffer, serializer.offset);
}

/// @brief Control the source of GNSS information used to update the Kalman Filter.
/// 
/// Changing the GNSS source while the sensor is in the "running" state may temporarily place
/// it back in the "init" state until the new source of GNSS data is received.
/// 
/// @param[out] source 
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_read_gnss_source(struct mip_interface* device, enum mip_filter_gnss_source_command_source* source)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_GNSS_SOURCE_CONTROL, buffer, serializer.offset, MIP_REPLY_DESC_FILTER_GNSS_SOURCE_CONTROL, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        struct mip_serializer serializer;
        mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
        
        extract_mip_filter_gnss_source_command_source(&serializer, source);
        
        if( !mip_serializer_ok(&serializer) )
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
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_save_gnss_source(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_GNSS_SOURCE_CONTROL, buffer, serializer.offset);
}

/// @brief Control the source of GNSS information used to update the Kalman Filter.
/// 
/// Changing the GNSS source while the sensor is in the "running" state may temporarily place
/// it back in the "init" state until the new source of GNSS data is received.
/// 
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_load_gnss_source(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_GNSS_SOURCE_CONTROL, buffer, serializer.offset);
}

/// @brief Control the source of GNSS information used to update the Kalman Filter.
/// 
/// Changing the GNSS source while the sensor is in the "running" state may temporarily place
/// it back in the "init" state until the new source of GNSS data is received.
/// 
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_default_gnss_source(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_GNSS_SOURCE_CONTROL, buffer, serializer.offset);
}

void insert_mip_filter_heading_source_command(struct mip_serializer* serializer, const struct mip_filter_heading_source_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    insert_mip_filter_heading_source_command_source(serializer, self->source);
}

void extract_mip_filter_heading_source_command(struct mip_serializer* serializer, struct mip_filter_heading_source_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    extract_mip_filter_heading_source_command_source(serializer, &self->source);
}

void insert_mip_filter_heading_source_command_source(struct mip_serializer* serializer, const enum mip_filter_heading_source_command_source self)
{
    return insert_u8(serializer, (uint8_t)(self));
}
void extract_mip_filter_heading_source_command_source(struct mip_serializer* serializer, enum mip_filter_heading_source_command_source* self)
{
    uint8_t tmp = 0;
    extract_u8(serializer, &tmp);
    *self = tmp;
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
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_write_heading_source(struct mip_interface* device, enum mip_filter_heading_source_command_source source)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    insert_mip_filter_heading_source_command_source(&serializer, source);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_HEADING_UPDATE_CONTROL, buffer, serializer.offset);
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
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_read_heading_source(struct mip_interface* device, enum mip_filter_heading_source_command_source* source)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_HEADING_UPDATE_CONTROL, buffer, serializer.offset, MIP_REPLY_DESC_FILTER_HEADING_UPDATE_CONTROL, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        struct mip_serializer serializer;
        mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
        
        extract_mip_filter_heading_source_command_source(&serializer, source);
        
        if( !mip_serializer_ok(&serializer) )
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
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_save_heading_source(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_HEADING_UPDATE_CONTROL, buffer, serializer.offset);
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
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_load_heading_source(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_HEADING_UPDATE_CONTROL, buffer, serializer.offset);
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
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_default_heading_source(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_HEADING_UPDATE_CONTROL, buffer, serializer.offset);
}

void insert_mip_filter_altitude_aiding_command(struct mip_serializer* serializer, const struct mip_filter_altitude_aiding_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    insert_u8(serializer, self->aiding_selector);
}

void extract_mip_filter_altitude_aiding_command(struct mip_serializer* serializer, struct mip_filter_altitude_aiding_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    extract_u8(serializer, &self->aiding_selector);
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
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_write_altitude_aiding(struct mip_interface* device, uint8_t aiding_selector)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    insert_u8(&serializer, aiding_selector);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ALTITUDE_AIDING_CONTROL, buffer, serializer.offset);
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
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_read_altitude_aiding(struct mip_interface* device, uint8_t* aiding_selector)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ALTITUDE_AIDING_CONTROL, buffer, serializer.offset, MIP_REPLY_DESC_FILTER_ALTITUDE_AIDING_CONTROL, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        struct mip_serializer serializer;
        mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
        
        extract_u8(&serializer, aiding_selector);
        
        if( !mip_serializer_ok(&serializer) )
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
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_save_altitude_aiding(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ALTITUDE_AIDING_CONTROL, buffer, serializer.offset);
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
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_load_altitude_aiding(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ALTITUDE_AIDING_CONTROL, buffer, serializer.offset);
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
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_default_altitude_aiding(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ALTITUDE_AIDING_CONTROL, buffer, serializer.offset);
}

void insert_mip_filter_auto_zupt_command(struct mip_serializer* serializer, const struct mip_filter_auto_zupt_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    insert_u8(serializer, self->enable);
    insert_float(serializer, self->threshold);
}

void extract_mip_filter_auto_zupt_command(struct mip_serializer* serializer, struct mip_filter_auto_zupt_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    extract_u8(serializer, &self->enable);
    extract_float(serializer, &self->threshold);
}

/// @brief Zero Velocity Update
/// The ZUPT is triggered when the scalar magnitude of the GNSS reported velocity vector is equal-to or less than the threshold value.
/// The device will NACK threshold values that are less than zero (i.e.negative.)
/// @param enable 0 - Disable, 1 - Enable
/// @param threshold [meters/second]
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_write_auto_zupt(struct mip_interface* device, uint8_t enable, float threshold)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    insert_u8(&serializer, enable);
    insert_float(&serializer, threshold);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ZUPT_CONTROL, buffer, serializer.offset);
}

/// @brief Zero Velocity Update
/// The ZUPT is triggered when the scalar magnitude of the GNSS reported velocity vector is equal-to or less than the threshold value.
/// The device will NACK threshold values that are less than zero (i.e.negative.)
/// @param[out] enable 0 - Disable, 1 - Enable
/// @param[out] threshold [meters/second]
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_read_auto_zupt(struct mip_interface* device, uint8_t* enable, float* threshold)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ZUPT_CONTROL, buffer, serializer.offset, MIP_REPLY_DESC_FILTER_ZUPT_CONTROL, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        struct mip_serializer serializer;
        mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
        
        extract_u8(&serializer, enable);
        extract_float(&serializer, threshold);
        
        if( !mip_serializer_ok(&serializer) )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Zero Velocity Update
/// The ZUPT is triggered when the scalar magnitude of the GNSS reported velocity vector is equal-to or less than the threshold value.
/// The device will NACK threshold values that are less than zero (i.e.negative.)
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_save_auto_zupt(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ZUPT_CONTROL, buffer, serializer.offset);
}

/// @brief Zero Velocity Update
/// The ZUPT is triggered when the scalar magnitude of the GNSS reported velocity vector is equal-to or less than the threshold value.
/// The device will NACK threshold values that are less than zero (i.e.negative.)
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_load_auto_zupt(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ZUPT_CONTROL, buffer, serializer.offset);
}

/// @brief Zero Velocity Update
/// The ZUPT is triggered when the scalar magnitude of the GNSS reported velocity vector is equal-to or less than the threshold value.
/// The device will NACK threshold values that are less than zero (i.e.negative.)
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_default_auto_zupt(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ZUPT_CONTROL, buffer, serializer.offset);
}

void insert_mip_filter_auto_angular_zupt_command(struct mip_serializer* serializer, const struct mip_filter_auto_angular_zupt_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    insert_u8(serializer, self->enable);
    insert_float(serializer, self->threshold);
}

void extract_mip_filter_auto_angular_zupt_command(struct mip_serializer* serializer, struct mip_filter_auto_angular_zupt_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    extract_u8(serializer, &self->enable);
    extract_float(serializer, &self->threshold);
}

/// @brief Zero Angular Rate Update
/// The ZUPT is triggered when the scalar magnitude of the angular rate vector is equal-to or less than the threshold value.
/// The device will NACK threshold values that are less than zero (i.e.negative.)
/// @param enable 0 - Disable, 1 - Enable
/// @param threshold [radians/second]
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_write_auto_angular_zupt(struct mip_interface* device, uint8_t enable, float threshold)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    insert_u8(&serializer, enable);
    insert_float(&serializer, threshold);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ANGULAR_ZUPT_CONTROL, buffer, serializer.offset);
}

/// @brief Zero Angular Rate Update
/// The ZUPT is triggered when the scalar magnitude of the angular rate vector is equal-to or less than the threshold value.
/// The device will NACK threshold values that are less than zero (i.e.negative.)
/// @param[out] enable 0 - Disable, 1 - Enable
/// @param[out] threshold [radians/second]
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_read_auto_angular_zupt(struct mip_interface* device, uint8_t* enable, float* threshold)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ANGULAR_ZUPT_CONTROL, buffer, serializer.offset, MIP_REPLY_DESC_FILTER_ANGULAR_ZUPT_CONTROL, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        struct mip_serializer serializer;
        mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
        
        extract_u8(&serializer, enable);
        extract_float(&serializer, threshold);
        
        if( !mip_serializer_ok(&serializer) )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Zero Angular Rate Update
/// The ZUPT is triggered when the scalar magnitude of the angular rate vector is equal-to or less than the threshold value.
/// The device will NACK threshold values that are less than zero (i.e.negative.)
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_save_auto_angular_zupt(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ANGULAR_ZUPT_CONTROL, buffer, serializer.offset);
}

/// @brief Zero Angular Rate Update
/// The ZUPT is triggered when the scalar magnitude of the angular rate vector is equal-to or less than the threshold value.
/// The device will NACK threshold values that are less than zero (i.e.negative.)
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_load_auto_angular_zupt(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ANGULAR_ZUPT_CONTROL, buffer, serializer.offset);
}

/// @brief Zero Angular Rate Update
/// The ZUPT is triggered when the scalar magnitude of the angular rate vector is equal-to or less than the threshold value.
/// The device will NACK threshold values that are less than zero (i.e.negative.)
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_default_auto_angular_zupt(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ANGULAR_ZUPT_CONTROL, buffer, serializer.offset);
}

/// @brief Commanded Zero Velocity Update
/// Please see the device user manual for the maximum rate of this message.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_commanded_zupt(struct mip_interface* device)
{
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_COMMANDED_ZUPT, NULL, 0);
}

/// @brief Commanded Zero Angular Rate Update
/// Please see the device user manual for the maximum rate of this message.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_commanded_angular_zupt(struct mip_interface* device)
{
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_COMMANDED_ANGULAR_ZUPT, NULL, 0);
}

void insert_mip_filter_aiding_measurement_enable_command(struct mip_serializer* serializer, const struct mip_filter_aiding_measurement_enable_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    insert_mip_filter_aiding_measurement_enable_command_aiding_source(serializer, self->aiding_source);
    insert_bool(serializer, self->enable);
}

void extract_mip_filter_aiding_measurement_enable_command(struct mip_serializer* serializer, struct mip_filter_aiding_measurement_enable_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    extract_mip_filter_aiding_measurement_enable_command_aiding_source(serializer, &self->aiding_source);
    extract_bool(serializer, &self->enable);
}

void insert_mip_filter_aiding_measurement_enable_command_aiding_source(struct mip_serializer* serializer, const enum mip_filter_aiding_measurement_enable_command_aiding_source self)
{
    return insert_u16(serializer, (uint16_t)(self));
}
void extract_mip_filter_aiding_measurement_enable_command_aiding_source(struct mip_serializer* serializer, enum mip_filter_aiding_measurement_enable_command_aiding_source* self)
{
    uint16_t tmp = 0;
    extract_u16(serializer, &tmp);
    *self = tmp;
}

/// @brief Enables / disables the specified aiding measurement source.
/// 
/// 
/// @param aiding_source Aiding measurement source
/// @param enable Controls the aiding sorce
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_write_aiding_measurement_enable(struct mip_interface* device, enum mip_filter_aiding_measurement_enable_command_aiding_source aiding_source, bool enable)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    insert_mip_filter_aiding_measurement_enable_command_aiding_source(&serializer, aiding_source);
    insert_bool(&serializer, enable);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_AIDING_MEASUREMENT_ENABLE, buffer, serializer.offset);
}

/// @brief Enables / disables the specified aiding measurement source.
/// 
/// 
/// @param aiding_source Aiding measurement source
/// @param[out] aiding_source Aiding measurement source
/// @param[out] enable Controls the aiding sorce
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_read_aiding_measurement_enable(struct mip_interface* device, enum mip_filter_aiding_measurement_enable_command_aiding_source aiding_source, bool* enable)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    insert_mip_filter_aiding_measurement_enable_command_aiding_source(&serializer, aiding_source);
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_AIDING_MEASUREMENT_ENABLE, buffer, serializer.offset, MIP_REPLY_DESC_FILTER_AIDING_MEASUREMENT_ENABLE, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        struct mip_serializer serializer;
        mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
        
        extract_mip_filter_aiding_measurement_enable_command_aiding_source(&serializer, &aiding_source);
        extract_bool(&serializer, enable);
        
        if( !mip_serializer_ok(&serializer) )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Enables / disables the specified aiding measurement source.
/// 
/// 
/// @param aiding_source Aiding measurement source
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_save_aiding_measurement_enable(struct mip_interface* device, enum mip_filter_aiding_measurement_enable_command_aiding_source aiding_source)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    insert_mip_filter_aiding_measurement_enable_command_aiding_source(&serializer, aiding_source);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_AIDING_MEASUREMENT_ENABLE, buffer, serializer.offset);
}

/// @brief Enables / disables the specified aiding measurement source.
/// 
/// 
/// @param aiding_source Aiding measurement source
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_load_aiding_measurement_enable(struct mip_interface* device, enum mip_filter_aiding_measurement_enable_command_aiding_source aiding_source)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    insert_mip_filter_aiding_measurement_enable_command_aiding_source(&serializer, aiding_source);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_AIDING_MEASUREMENT_ENABLE, buffer, serializer.offset);
}

/// @brief Enables / disables the specified aiding measurement source.
/// 
/// 
/// @param aiding_source Aiding measurement source
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_default_aiding_measurement_enable(struct mip_interface* device, enum mip_filter_aiding_measurement_enable_command_aiding_source aiding_source)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    insert_mip_filter_aiding_measurement_enable_command_aiding_source(&serializer, aiding_source);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_AIDING_MEASUREMENT_ENABLE, buffer, serializer.offset);
}

/// @brief Manual run command.
/// 
/// If the initialization configuration has the "wait_for_run_command" option enabled, the filter will wait until it receives this command before commencing integration and enabling the Kalman filter. Prior to the receipt of this command, the filter will remain in the filter initialization mode.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_run(struct mip_interface* device)
{
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_RUN, NULL, 0);
}

void insert_mip_filter_kinematic_constraint_command(struct mip_serializer* serializer, const struct mip_filter_kinematic_constraint_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    insert_u8(serializer, self->acceleration_constraint_selection);
    insert_u8(serializer, self->velocity_constraint_selection);
    insert_u8(serializer, self->angular_constraint_selection);
}

void extract_mip_filter_kinematic_constraint_command(struct mip_serializer* serializer, struct mip_filter_kinematic_constraint_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    extract_u8(serializer, &self->acceleration_constraint_selection);
    extract_u8(serializer, &self->velocity_constraint_selection);
    extract_u8(serializer, &self->angular_constraint_selection);
}

/// @brief Controls kinematic constraint model selection for the navigation filter.
/// 
/// See manual for explanation of how the kinematic constraints are applied.
/// @param acceleration_constraint_selection Acceleration constraint: <br/> 0=None (default), <br/> 1=Zero-acceleration.
/// @param velocity_constraint_selection 0=None (default), <br/> 1=Zero-velocity, <br/> 2=Wheeled-vehicle. <br/>
/// @param angular_constraint_selection 0=None (default), 1=Zero-angular rate (ZUPT).
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_write_kinematic_constraint(struct mip_interface* device, uint8_t acceleration_constraint_selection, uint8_t velocity_constraint_selection, uint8_t angular_constraint_selection)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    insert_u8(&serializer, acceleration_constraint_selection);
    insert_u8(&serializer, velocity_constraint_selection);
    insert_u8(&serializer, angular_constraint_selection);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_KINEMATIC_CONSTRAINT, buffer, serializer.offset);
}

/// @brief Controls kinematic constraint model selection for the navigation filter.
/// 
/// See manual for explanation of how the kinematic constraints are applied.
/// @param[out] acceleration_constraint_selection Acceleration constraint: <br/> 0=None (default), <br/> 1=Zero-acceleration.
/// @param[out] velocity_constraint_selection 0=None (default), <br/> 1=Zero-velocity, <br/> 2=Wheeled-vehicle. <br/>
/// @param[out] angular_constraint_selection 0=None (default), 1=Zero-angular rate (ZUPT).
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_read_kinematic_constraint(struct mip_interface* device, uint8_t* acceleration_constraint_selection, uint8_t* velocity_constraint_selection, uint8_t* angular_constraint_selection)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_KINEMATIC_CONSTRAINT, buffer, serializer.offset, MIP_REPLY_DESC_FILTER_KINEMATIC_CONSTRAINT, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        struct mip_serializer serializer;
        mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
        
        extract_u8(&serializer, acceleration_constraint_selection);
        extract_u8(&serializer, velocity_constraint_selection);
        extract_u8(&serializer, angular_constraint_selection);
        
        if( !mip_serializer_ok(&serializer) )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Controls kinematic constraint model selection for the navigation filter.
/// 
/// See manual for explanation of how the kinematic constraints are applied.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_save_kinematic_constraint(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_KINEMATIC_CONSTRAINT, buffer, serializer.offset);
}

/// @brief Controls kinematic constraint model selection for the navigation filter.
/// 
/// See manual for explanation of how the kinematic constraints are applied.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_load_kinematic_constraint(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_KINEMATIC_CONSTRAINT, buffer, serializer.offset);
}

/// @brief Controls kinematic constraint model selection for the navigation filter.
/// 
/// See manual for explanation of how the kinematic constraints are applied.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_default_kinematic_constraint(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_KINEMATIC_CONSTRAINT, buffer, serializer.offset);
}

void insert_mip_filter_initialization_configuration_command(struct mip_serializer* serializer, const struct mip_filter_initialization_configuration_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    insert_u8(serializer, self->wait_for_run_command);
    insert_mip_filter_initialization_configuration_command_initial_condition_source(serializer, self->initial_cond_src);
    insert_mip_filter_initialization_configuration_command_alignment_selector(serializer, self->auto_heading_alignment_selector);
    insert_float(serializer, self->initial_heading);
    insert_float(serializer, self->initial_pitch);
    insert_float(serializer, self->initial_roll);
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->initial_position[i]);
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->initial_velocity[i]);
    insert_mip_filter_reference_frame(serializer, self->reference_frame_selector);
}

void extract_mip_filter_initialization_configuration_command(struct mip_serializer* serializer, struct mip_filter_initialization_configuration_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    extract_u8(serializer, &self->wait_for_run_command);
    extract_mip_filter_initialization_configuration_command_initial_condition_source(serializer, &self->initial_cond_src);
    extract_mip_filter_initialization_configuration_command_alignment_selector(serializer, &self->auto_heading_alignment_selector);
    extract_float(serializer, &self->initial_heading);
    extract_float(serializer, &self->initial_pitch);
    extract_float(serializer, &self->initial_roll);
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->initial_position[i]);
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->initial_velocity[i]);
    extract_mip_filter_reference_frame(serializer, &self->reference_frame_selector);
}

void insert_mip_filter_initialization_configuration_command_alignment_selector(struct mip_serializer* serializer, const enum mip_filter_initialization_configuration_command_alignment_selector self)
{
    return insert_u8(serializer, (uint8_t)(self));
}
void extract_mip_filter_initialization_configuration_command_alignment_selector(struct mip_serializer* serializer, enum mip_filter_initialization_configuration_command_alignment_selector* self)
{
    uint8_t tmp = 0;
    extract_u8(serializer, &tmp);
    *self = tmp;
}

void insert_mip_filter_initialization_configuration_command_initial_condition_source(struct mip_serializer* serializer, const enum mip_filter_initialization_configuration_command_initial_condition_source self)
{
    return insert_u8(serializer, (uint8_t)(self));
}
void extract_mip_filter_initialization_configuration_command_initial_condition_source(struct mip_serializer* serializer, enum mip_filter_initialization_configuration_command_initial_condition_source* self)
{
    uint8_t tmp = 0;
    extract_u8(serializer, &tmp);
    *self = tmp;
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
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_write_initialization_configuration(struct mip_interface* device, uint8_t wait_for_run_command, enum mip_filter_initialization_configuration_command_initial_condition_source initial_cond_src, enum mip_filter_initialization_configuration_command_alignment_selector auto_heading_alignment_selector, float initial_heading, float initial_pitch, float initial_roll, const float* initial_position, const float* initial_velocity, enum mip_filter_reference_frame reference_frame_selector)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    insert_u8(&serializer, wait_for_run_command);
    insert_mip_filter_initialization_configuration_command_initial_condition_source(&serializer, initial_cond_src);
    insert_mip_filter_initialization_configuration_command_alignment_selector(&serializer, auto_heading_alignment_selector);
    insert_float(&serializer, initial_heading);
    insert_float(&serializer, initial_pitch);
    insert_float(&serializer, initial_roll);
    for(unsigned int i=0; i < 3; i++)
        insert_float(&serializer, initial_position[i]);
    for(unsigned int i=0; i < 3; i++)
        insert_float(&serializer, initial_velocity[i]);
    insert_mip_filter_reference_frame(&serializer, reference_frame_selector);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_INITIALIZATION_CONFIGURATION, buffer, serializer.offset);
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
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_read_initialization_configuration(struct mip_interface* device, uint8_t* wait_for_run_command, enum mip_filter_initialization_configuration_command_initial_condition_source* initial_cond_src, enum mip_filter_initialization_configuration_command_alignment_selector* auto_heading_alignment_selector, float* initial_heading, float* initial_pitch, float* initial_roll, float* initial_position, float* initial_velocity, enum mip_filter_reference_frame* reference_frame_selector)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_INITIALIZATION_CONFIGURATION, buffer, serializer.offset, MIP_REPLY_DESC_FILTER_INITIALIZATION_CONFIGURATION, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        struct mip_serializer serializer;
        mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
        
        extract_u8(&serializer, wait_for_run_command);
        extract_mip_filter_initialization_configuration_command_initial_condition_source(&serializer, initial_cond_src);
        extract_mip_filter_initialization_configuration_command_alignment_selector(&serializer, auto_heading_alignment_selector);
        extract_float(&serializer, initial_heading);
        extract_float(&serializer, initial_pitch);
        extract_float(&serializer, initial_roll);
        for(unsigned int i=0; i < 3; i++)
            extract_float(&serializer, &initial_position[i]);
        for(unsigned int i=0; i < 3; i++)
            extract_float(&serializer, &initial_velocity[i]);
        extract_mip_filter_reference_frame(&serializer, reference_frame_selector);
        
        if( !mip_serializer_ok(&serializer) )
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
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_save_initialization_configuration(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_INITIALIZATION_CONFIGURATION, buffer, serializer.offset);
}

/// @brief Controls the source and values used for initial conditions of the navigation solution.
/// 
/// Notes: Initial conditions are the position, velocity, and attitude of the platform used when the filter starts running or is reset.
/// For the user specified position array, the units are meters if the ECEF frame is selected, and degrees latitude, degrees longitude, and meters above ellipsoid if the latitude/longitude/height frame is selected.
/// For the user specified velocity array, the units are meters per second, but the reference frame depends on the reference frame selector (ECEF or NED).
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_load_initialization_configuration(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_INITIALIZATION_CONFIGURATION, buffer, serializer.offset);
}

/// @brief Controls the source and values used for initial conditions of the navigation solution.
/// 
/// Notes: Initial conditions are the position, velocity, and attitude of the platform used when the filter starts running or is reset.
/// For the user specified position array, the units are meters if the ECEF frame is selected, and degrees latitude, degrees longitude, and meters above ellipsoid if the latitude/longitude/height frame is selected.
/// For the user specified velocity array, the units are meters per second, but the reference frame depends on the reference frame selector (ECEF or NED).
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_default_initialization_configuration(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_INITIALIZATION_CONFIGURATION, buffer, serializer.offset);
}

void insert_mip_filter_adaptive_filter_options_command(struct mip_serializer* serializer, const struct mip_filter_adaptive_filter_options_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    insert_u8(serializer, self->level);
    insert_u16(serializer, self->time_limit);
}

void extract_mip_filter_adaptive_filter_options_command(struct mip_serializer* serializer, struct mip_filter_adaptive_filter_options_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    extract_u8(serializer, &self->level);
    extract_u16(serializer, &self->time_limit);
}

/// @brief Configures the basic setup for auto-adaptive filtering. See product manual for a detailed description of this feature.
/// 
/// @param level Auto-adaptive operating level: <br/> 0=Off, <br/> 1=Conservative, <br/> 2=Moderate (default), <br/> 3=Aggressive.
/// @param time_limit Maximum duration of measurement rejection before entering recovery mode    (ms)
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_write_adaptive_filter_options(struct mip_interface* device, uint8_t level, uint16_t time_limit)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    insert_u8(&serializer, level);
    insert_u16(&serializer, time_limit);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ADAPTIVE_FILTER_OPTIONS, buffer, serializer.offset);
}

/// @brief Configures the basic setup for auto-adaptive filtering. See product manual for a detailed description of this feature.
/// 
/// @param[out] level Auto-adaptive operating level: <br/> 0=Off, <br/> 1=Conservative, <br/> 2=Moderate (default), <br/> 3=Aggressive.
/// @param[out] time_limit Maximum duration of measurement rejection before entering recovery mode    (ms)
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_read_adaptive_filter_options(struct mip_interface* device, uint8_t* level, uint16_t* time_limit)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ADAPTIVE_FILTER_OPTIONS, buffer, serializer.offset, MIP_REPLY_DESC_FILTER_ADAPTIVE_FILTER_OPTIONS, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        struct mip_serializer serializer;
        mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
        
        extract_u8(&serializer, level);
        extract_u16(&serializer, time_limit);
        
        if( !mip_serializer_ok(&serializer) )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Configures the basic setup for auto-adaptive filtering. See product manual for a detailed description of this feature.
/// 
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_save_adaptive_filter_options(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ADAPTIVE_FILTER_OPTIONS, buffer, serializer.offset);
}

/// @brief Configures the basic setup for auto-adaptive filtering. See product manual for a detailed description of this feature.
/// 
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_load_adaptive_filter_options(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ADAPTIVE_FILTER_OPTIONS, buffer, serializer.offset);
}

/// @brief Configures the basic setup for auto-adaptive filtering. See product manual for a detailed description of this feature.
/// 
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_default_adaptive_filter_options(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ADAPTIVE_FILTER_OPTIONS, buffer, serializer.offset);
}

void insert_mip_filter_multi_antenna_offset_command(struct mip_serializer* serializer, const struct mip_filter_multi_antenna_offset_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    insert_u8(serializer, self->receiver_id);
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->antenna_offset[i]);
}

void extract_mip_filter_multi_antenna_offset_command(struct mip_serializer* serializer, struct mip_filter_multi_antenna_offset_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    extract_u8(serializer, &self->receiver_id);
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->antenna_offset[i]);
}

/// @brief Set the antenna lever arm.
/// 
/// This command works with devices that utilize multiple antennas.
/// @param receiver_id Receiver: 1, 2, etc...
/// @param antenna_offset Antenna lever arm offset vector in the vehicle frame (m)
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_write_multi_antenna_offset(struct mip_interface* device, uint8_t receiver_id, const float* antenna_offset)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    insert_u8(&serializer, receiver_id);
    for(unsigned int i=0; i < 3; i++)
        insert_float(&serializer, antenna_offset[i]);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_MULTI_ANTENNA_OFFSET, buffer, serializer.offset);
}

/// @brief Set the antenna lever arm.
/// 
/// This command works with devices that utilize multiple antennas.
/// @param receiver_id Receiver: 1, 2, etc...
/// @param[out] receiver_id 
/// @param[out] antenna_offset 
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_read_multi_antenna_offset(struct mip_interface* device, uint8_t receiver_id, float* antenna_offset)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    insert_u8(&serializer, receiver_id);
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_MULTI_ANTENNA_OFFSET, buffer, serializer.offset, MIP_REPLY_DESC_FILTER_MULTI_ANTENNA_OFFSET, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        struct mip_serializer serializer;
        mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
        
        extract_u8(&serializer, &receiver_id);
        for(unsigned int i=0; i < 3; i++)
            extract_float(&serializer, &antenna_offset[i]);
        
        if( !mip_serializer_ok(&serializer) )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Set the antenna lever arm.
/// 
/// This command works with devices that utilize multiple antennas.
/// @param receiver_id Receiver: 1, 2, etc...
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_save_multi_antenna_offset(struct mip_interface* device, uint8_t receiver_id)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    insert_u8(&serializer, receiver_id);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_MULTI_ANTENNA_OFFSET, buffer, serializer.offset);
}

/// @brief Set the antenna lever arm.
/// 
/// This command works with devices that utilize multiple antennas.
/// @param receiver_id Receiver: 1, 2, etc...
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_load_multi_antenna_offset(struct mip_interface* device, uint8_t receiver_id)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    insert_u8(&serializer, receiver_id);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_MULTI_ANTENNA_OFFSET, buffer, serializer.offset);
}

/// @brief Set the antenna lever arm.
/// 
/// This command works with devices that utilize multiple antennas.
/// @param receiver_id Receiver: 1, 2, etc...
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_default_multi_antenna_offset(struct mip_interface* device, uint8_t receiver_id)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    insert_u8(&serializer, receiver_id);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_MULTI_ANTENNA_OFFSET, buffer, serializer.offset);
}

void insert_mip_filter_rel_pos_configuration_command(struct mip_serializer* serializer, const struct mip_filter_rel_pos_configuration_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    insert_u8(serializer, self->source);
    insert_mip_filter_reference_frame(serializer, self->reference_frame_selector);
    for(unsigned int i=0; i < 3; i++)
        insert_double(serializer, self->reference_coordinates[i]);
}

void extract_mip_filter_rel_pos_configuration_command(struct mip_serializer* serializer, struct mip_filter_rel_pos_configuration_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    extract_u8(serializer, &self->source);
    extract_mip_filter_reference_frame(serializer, &self->reference_frame_selector);
    for(unsigned int i=0; i < 3; i++)
        extract_double(serializer, &self->reference_coordinates[i]);
}

/// @brief Configure the reference location for filter relative positioning outputs
/// 
/// @param source 0 - auto (RTK base station), 1 - manual
/// @param reference_frame_selector ECEF or LLH
/// @param reference_coordinates reference coordinates, units determined by source selection
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_write_rel_pos_configuration(struct mip_interface* device, uint8_t source, enum mip_filter_reference_frame reference_frame_selector, const double* reference_coordinates)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    insert_u8(&serializer, source);
    insert_mip_filter_reference_frame(&serializer, reference_frame_selector);
    for(unsigned int i=0; i < 3; i++)
        insert_double(&serializer, reference_coordinates[i]);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_REL_POS_CONFIGURATION, buffer, serializer.offset);
}

/// @brief Configure the reference location for filter relative positioning outputs
/// 
/// @param[out] source 0 - auto (RTK base station), 1 - manual
/// @param[out] reference_frame_selector ECEF or LLH
/// @param[out] reference_coordinates reference coordinates, units determined by source selection
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_read_rel_pos_configuration(struct mip_interface* device, uint8_t* source, enum mip_filter_reference_frame* reference_frame_selector, double* reference_coordinates)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_REL_POS_CONFIGURATION, buffer, serializer.offset, MIP_REPLY_DESC_FILTER_REL_POS_CONFIGURATION, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        struct mip_serializer serializer;
        mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
        
        extract_u8(&serializer, source);
        extract_mip_filter_reference_frame(&serializer, reference_frame_selector);
        for(unsigned int i=0; i < 3; i++)
            extract_double(&serializer, &reference_coordinates[i]);
        
        if( !mip_serializer_ok(&serializer) )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Configure the reference location for filter relative positioning outputs
/// 
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_save_rel_pos_configuration(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_REL_POS_CONFIGURATION, buffer, serializer.offset);
}

/// @brief Configure the reference location for filter relative positioning outputs
/// 
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_load_rel_pos_configuration(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_REL_POS_CONFIGURATION, buffer, serializer.offset);
}

/// @brief Configure the reference location for filter relative positioning outputs
/// 
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_default_rel_pos_configuration(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_REL_POS_CONFIGURATION, buffer, serializer.offset);
}

void insert_mip_filter_ref_point_lever_arm_command(struct mip_serializer* serializer, const struct mip_filter_ref_point_lever_arm_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    insert_mip_filter_ref_point_lever_arm_command_reference_point_selector(serializer, self->ref_point_sel);
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->lever_arm_offset[i]);
}

void extract_mip_filter_ref_point_lever_arm_command(struct mip_serializer* serializer, struct mip_filter_ref_point_lever_arm_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    extract_mip_filter_ref_point_lever_arm_command_reference_point_selector(serializer, &self->ref_point_sel);
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->lever_arm_offset[i]);
}

void insert_mip_filter_ref_point_lever_arm_command_reference_point_selector(struct mip_serializer* serializer, const enum mip_filter_ref_point_lever_arm_command_reference_point_selector self)
{
    return insert_u8(serializer, (uint8_t)(self));
}
void extract_mip_filter_ref_point_lever_arm_command_reference_point_selector(struct mip_serializer* serializer, enum mip_filter_ref_point_lever_arm_command_reference_point_selector* self)
{
    uint8_t tmp = 0;
    extract_u8(serializer, &tmp);
    *self = tmp;
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
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_write_ref_point_lever_arm(struct mip_interface* device, enum mip_filter_ref_point_lever_arm_command_reference_point_selector ref_point_sel, const float* lever_arm_offset)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    insert_mip_filter_ref_point_lever_arm_command_reference_point_selector(&serializer, ref_point_sel);
    for(unsigned int i=0; i < 3; i++)
        insert_float(&serializer, lever_arm_offset[i]);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_REF_POINT_LEVER_ARM, buffer, serializer.offset);
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
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_read_ref_point_lever_arm(struct mip_interface* device, enum mip_filter_ref_point_lever_arm_command_reference_point_selector* ref_point_sel, float* lever_arm_offset)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_REF_POINT_LEVER_ARM, buffer, serializer.offset, MIP_REPLY_DESC_FILTER_REF_POINT_LEVER_ARM, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        struct mip_serializer serializer;
        mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
        
        extract_mip_filter_ref_point_lever_arm_command_reference_point_selector(&serializer, ref_point_sel);
        for(unsigned int i=0; i < 3; i++)
            extract_float(&serializer, &lever_arm_offset[i]);
        
        if( !mip_serializer_ok(&serializer) )
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
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_save_ref_point_lever_arm(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_REF_POINT_LEVER_ARM, buffer, serializer.offset);
}

/// @brief Lever arm offset with respect to the sensor for the indicated point of reference.
/// This is used to change the location of the indicated point of reference, and will affect filter position and velocity outputs.
/// Changing this setting from default will result in a global position offset that depends on vehicle attitude,
/// and a velocity offset that depends on vehicle attitude and angular rate.
/// The lever arm is defined by a 3-element vector that points from the sensor to the desired reference point, with (x,y,z) components given in the vehicle's reference frame.
/// Note, if the reference point selector is set to VEH (1), this setting will affect the following data fields: (0x82, 0x01), (0x82, 0x02), (0x82, 0x40), (0x82, 0x41), and (0x82, 42)
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_load_ref_point_lever_arm(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_REF_POINT_LEVER_ARM, buffer, serializer.offset);
}

/// @brief Lever arm offset with respect to the sensor for the indicated point of reference.
/// This is used to change the location of the indicated point of reference, and will affect filter position and velocity outputs.
/// Changing this setting from default will result in a global position offset that depends on vehicle attitude,
/// and a velocity offset that depends on vehicle attitude and angular rate.
/// The lever arm is defined by a 3-element vector that points from the sensor to the desired reference point, with (x,y,z) components given in the vehicle's reference frame.
/// Note, if the reference point selector is set to VEH (1), this setting will affect the following data fields: (0x82, 0x01), (0x82, 0x02), (0x82, 0x40), (0x82, 0x41), and (0x82, 42)
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_default_ref_point_lever_arm(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_REF_POINT_LEVER_ARM, buffer, serializer.offset);
}

void insert_mip_filter_speed_measurement_command(struct mip_serializer* serializer, const struct mip_filter_speed_measurement_command* self)
{
    insert_u8(serializer, self->source);
    insert_float(serializer, self->time_of_week);
    insert_float(serializer, self->speed);
    insert_float(serializer, self->speed_uncertainty);
}

void extract_mip_filter_speed_measurement_command(struct mip_serializer* serializer, struct mip_filter_speed_measurement_command* self)
{
    extract_u8(serializer, &self->source);
    extract_float(serializer, &self->time_of_week);
    extract_float(serializer, &self->speed);
    extract_float(serializer, &self->speed_uncertainty);
}

/// @brief Speed aiding measurement, where speed is defined as rate of motion along the vehicle's x-axis direction.
/// Can be used by an external odometer/speedometer, for example.
/// This command cannot be used if the internal odometer is configured.
/// @param source Reserved, must be 1.
/// @param time_of_week GPS time of week when speed was sampled
/// @param speed Estimated speed along vehicle's x-axis (may be positive or negative) [meters/second]
/// @param speed_uncertainty Estimated uncertainty in the speed measurement (1-sigma value) [meters/second]
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_speed_measurement(struct mip_interface* device, uint8_t source, float time_of_week, float speed, float speed_uncertainty)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_u8(&serializer, source);
    insert_float(&serializer, time_of_week);
    insert_float(&serializer, speed);
    insert_float(&serializer, speed_uncertainty);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SPEED_MEASUREMENT, buffer, serializer.offset);
}

void insert_mip_filter_speed_lever_arm_command(struct mip_serializer* serializer, const struct mip_filter_speed_lever_arm_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    insert_u8(serializer, self->source);
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->lever_arm_offset[i]);
}

void extract_mip_filter_speed_lever_arm_command(struct mip_serializer* serializer, struct mip_filter_speed_lever_arm_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    extract_u8(serializer, &self->source);
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->lever_arm_offset[i]);
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
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_write_speed_lever_arm(struct mip_interface* device, uint8_t source, const float* lever_arm_offset)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    insert_u8(&serializer, source);
    for(unsigned int i=0; i < 3; i++)
        insert_float(&serializer, lever_arm_offset[i]);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SPEED_LEVER_ARM, buffer, serializer.offset);
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
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_read_speed_lever_arm(struct mip_interface* device, uint8_t source, float* lever_arm_offset)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    insert_u8(&serializer, source);
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SPEED_LEVER_ARM, buffer, serializer.offset, MIP_REPLY_DESC_FILTER_SPEED_LEVER_ARM, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        struct mip_serializer serializer;
        mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
        
        extract_u8(&serializer, &source);
        for(unsigned int i=0; i < 3; i++)
            extract_float(&serializer, &lever_arm_offset[i]);
        
        if( !mip_serializer_ok(&serializer) )
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
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_save_speed_lever_arm(struct mip_interface* device, uint8_t source)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    insert_u8(&serializer, source);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SPEED_LEVER_ARM, buffer, serializer.offset);
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
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_load_speed_lever_arm(struct mip_interface* device, uint8_t source)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    insert_u8(&serializer, source);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SPEED_LEVER_ARM, buffer, serializer.offset);
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
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_default_speed_lever_arm(struct mip_interface* device, uint8_t source)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    insert_u8(&serializer, source);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SPEED_LEVER_ARM, buffer, serializer.offset);
}

void insert_mip_filter_wheeled_vehicle_constraint_control_command(struct mip_serializer* serializer, const struct mip_filter_wheeled_vehicle_constraint_control_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    insert_u8(serializer, self->enable);
}

void extract_mip_filter_wheeled_vehicle_constraint_control_command(struct mip_serializer* serializer, struct mip_filter_wheeled_vehicle_constraint_control_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    extract_u8(serializer, &self->enable);
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
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_write_wheeled_vehicle_constraint_control(struct mip_interface* device, uint8_t enable)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    insert_u8(&serializer, enable);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_WHEELED_VEHICLE_CONSTRAINT_CONTROL, buffer, serializer.offset);
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
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_read_wheeled_vehicle_constraint_control(struct mip_interface* device, uint8_t* enable)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_WHEELED_VEHICLE_CONSTRAINT_CONTROL, buffer, serializer.offset, MIP_REPLY_DESC_WHEELED_VEHICLE_CONSTRAINT_CONTROL, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        struct mip_serializer serializer;
        mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
        
        extract_u8(&serializer, enable);
        
        if( !mip_serializer_ok(&serializer) )
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
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_save_wheeled_vehicle_constraint_control(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_WHEELED_VEHICLE_CONSTRAINT_CONTROL, buffer, serializer.offset);
}

/// @brief Configure the wheeled vehicle kinematic constraint.
/// 
/// When enabled, the filter uses the assumption that velocity is constrained to the primary vehicle axis.
/// By convention, the primary vehicle axis is the vehicle X-axis (note: the sensor may be physically installed in
/// any orientation on the vehicle if the appropriate mounting transformation has been specified).
/// This constraint will typically improve heading estimates for vehicles where the assumption is valid, such
/// as an automobile, particulary when GNSS coverage is intermittent.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_load_wheeled_vehicle_constraint_control(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_WHEELED_VEHICLE_CONSTRAINT_CONTROL, buffer, serializer.offset);
}

/// @brief Configure the wheeled vehicle kinematic constraint.
/// 
/// When enabled, the filter uses the assumption that velocity is constrained to the primary vehicle axis.
/// By convention, the primary vehicle axis is the vehicle X-axis (note: the sensor may be physically installed in
/// any orientation on the vehicle if the appropriate mounting transformation has been specified).
/// This constraint will typically improve heading estimates for vehicles where the assumption is valid, such
/// as an automobile, particulary when GNSS coverage is intermittent.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_default_wheeled_vehicle_constraint_control(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_WHEELED_VEHICLE_CONSTRAINT_CONTROL, buffer, serializer.offset);
}

void insert_mip_filter_vertical_gyro_constraint_control_command(struct mip_serializer* serializer, const struct mip_filter_vertical_gyro_constraint_control_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    insert_u8(serializer, self->enable);
}

void extract_mip_filter_vertical_gyro_constraint_control_command(struct mip_serializer* serializer, struct mip_filter_vertical_gyro_constraint_control_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    extract_u8(serializer, &self->enable);
}

/// @brief Configure the vertical gyro kinematic constraint.
/// 
/// When enabled and no valid GNSS measurements are available, the filter uses the accelerometers to track pitch
/// and roll under the assumption that the sensor platform is not undergoing linear acceleration.
/// This constraint is useful to maintain accurate pitch and roll during GNSS signal outages.
/// @param enable 0 - Disable, 1 - Enable
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_write_vertical_gyro_constraint_control(struct mip_interface* device, uint8_t enable)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    insert_u8(&serializer, enable);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_VERTICAL_GYRO_CONSTRAINT_CONTROL, buffer, serializer.offset);
}

/// @brief Configure the vertical gyro kinematic constraint.
/// 
/// When enabled and no valid GNSS measurements are available, the filter uses the accelerometers to track pitch
/// and roll under the assumption that the sensor platform is not undergoing linear acceleration.
/// This constraint is useful to maintain accurate pitch and roll during GNSS signal outages.
/// @param[out] enable 0 - Disable, 1 - Enable
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_read_vertical_gyro_constraint_control(struct mip_interface* device, uint8_t* enable)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_VERTICAL_GYRO_CONSTRAINT_CONTROL, buffer, serializer.offset, MIP_REPLY_DESC_VERTICAL_GYRO_CONSTRAINT_CONTROL, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        struct mip_serializer serializer;
        mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
        
        extract_u8(&serializer, enable);
        
        if( !mip_serializer_ok(&serializer) )
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
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_save_vertical_gyro_constraint_control(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_VERTICAL_GYRO_CONSTRAINT_CONTROL, buffer, serializer.offset);
}

/// @brief Configure the vertical gyro kinematic constraint.
/// 
/// When enabled and no valid GNSS measurements are available, the filter uses the accelerometers to track pitch
/// and roll under the assumption that the sensor platform is not undergoing linear acceleration.
/// This constraint is useful to maintain accurate pitch and roll during GNSS signal outages.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_load_vertical_gyro_constraint_control(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_VERTICAL_GYRO_CONSTRAINT_CONTROL, buffer, serializer.offset);
}

/// @brief Configure the vertical gyro kinematic constraint.
/// 
/// When enabled and no valid GNSS measurements are available, the filter uses the accelerometers to track pitch
/// and roll under the assumption that the sensor platform is not undergoing linear acceleration.
/// This constraint is useful to maintain accurate pitch and roll during GNSS signal outages.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_default_vertical_gyro_constraint_control(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_VERTICAL_GYRO_CONSTRAINT_CONTROL, buffer, serializer.offset);
}

void insert_mip_filter_gnss_antenna_cal_control_command(struct mip_serializer* serializer, const struct mip_filter_gnss_antenna_cal_control_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    insert_u8(serializer, self->enable);
    insert_float(serializer, self->max_offset);
}

void extract_mip_filter_gnss_antenna_cal_control_command(struct mip_serializer* serializer, struct mip_filter_gnss_antenna_cal_control_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    extract_u8(serializer, &self->enable);
    extract_float(serializer, &self->max_offset);
}

/// @brief Configure the GNSS antenna lever arm calibration.
/// 
/// When enabled, the filter will enable lever arm error tracking, up to the maximum offset specified.
/// @param enable 0 - Disable, 1 - Enable
/// @param max_offset Maximum absolute value of lever arm offset error in the vehicle frame [meters]. See device user manual for the valid range of this parameter.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_write_gnss_antenna_cal_control(struct mip_interface* device, uint8_t enable, float max_offset)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    insert_u8(&serializer, enable);
    insert_float(&serializer, max_offset);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_GNSS_ANTENNA_CALIBRATION_CONTROL, buffer, serializer.offset);
}

/// @brief Configure the GNSS antenna lever arm calibration.
/// 
/// When enabled, the filter will enable lever arm error tracking, up to the maximum offset specified.
/// @param[out] enable 0 - Disable, 1 - Enable
/// @param[out] max_offset Maximum absolute value of lever arm offset error in the vehicle frame [meters]. See device user manual for the valid range of this parameter.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_read_gnss_antenna_cal_control(struct mip_interface* device, uint8_t* enable, float* max_offset)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_GNSS_ANTENNA_CALIBRATION_CONTROL, buffer, serializer.offset, MIP_REPLY_DESC_GNSS_ANTENNA_CALIBRATION_CONTROL, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        struct mip_serializer serializer;
        mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
        
        extract_u8(&serializer, enable);
        extract_float(&serializer, max_offset);
        
        if( !mip_serializer_ok(&serializer) )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Configure the GNSS antenna lever arm calibration.
/// 
/// When enabled, the filter will enable lever arm error tracking, up to the maximum offset specified.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_save_gnss_antenna_cal_control(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_GNSS_ANTENNA_CALIBRATION_CONTROL, buffer, serializer.offset);
}

/// @brief Configure the GNSS antenna lever arm calibration.
/// 
/// When enabled, the filter will enable lever arm error tracking, up to the maximum offset specified.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_load_gnss_antenna_cal_control(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_GNSS_ANTENNA_CALIBRATION_CONTROL, buffer, serializer.offset);
}

/// @brief Configure the GNSS antenna lever arm calibration.
/// 
/// When enabled, the filter will enable lever arm error tracking, up to the maximum offset specified.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_default_gnss_antenna_cal_control(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_GNSS_ANTENNA_CALIBRATION_CONTROL, buffer, serializer.offset);
}

void insert_mip_filter_magnetic_declination_source_command(struct mip_serializer* serializer, const struct mip_filter_magnetic_declination_source_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    insert_mip_filter_mag_declination_source(serializer, self->source);
    insert_float(serializer, self->declination);
}

void extract_mip_filter_magnetic_declination_source_command(struct mip_serializer* serializer, struct mip_filter_magnetic_declination_source_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    extract_mip_filter_mag_declination_source(serializer, &self->source);
    extract_float(serializer, &self->declination);
}

/// @brief Source for magnetic declination angle, and user supplied value for manual selection.
/// 
/// @param source Magnetic field declination angle source
/// @param declination Declination angle used when 'source' is set to 'MANUAL' (radians)
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_write_magnetic_declination_source(struct mip_interface* device, enum mip_filter_mag_declination_source source, float declination)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    insert_mip_filter_mag_declination_source(&serializer, source);
    insert_float(&serializer, declination);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_DECLINATION_SOURCE, buffer, serializer.offset);
}

/// @brief Source for magnetic declination angle, and user supplied value for manual selection.
/// 
/// @param[out] source Magnetic field declination angle source
/// @param[out] declination Declination angle used when 'source' is set to 'MANUAL' (radians)
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_read_magnetic_declination_source(struct mip_interface* device, enum mip_filter_mag_declination_source* source, float* declination)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_DECLINATION_SOURCE, buffer, serializer.offset, MIP_REPLY_DESC_FILTER_DECLINATION_SOURCE, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        struct mip_serializer serializer;
        mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
        
        extract_mip_filter_mag_declination_source(&serializer, source);
        extract_float(&serializer, declination);
        
        if( !mip_serializer_ok(&serializer) )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Source for magnetic declination angle, and user supplied value for manual selection.
/// 
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_save_magnetic_declination_source(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_DECLINATION_SOURCE, buffer, serializer.offset);
}

/// @brief Source for magnetic declination angle, and user supplied value for manual selection.
/// 
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_load_magnetic_declination_source(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_DECLINATION_SOURCE, buffer, serializer.offset);
}

/// @brief Source for magnetic declination angle, and user supplied value for manual selection.
/// 
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_default_magnetic_declination_source(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_DECLINATION_SOURCE, buffer, serializer.offset);
}

void insert_mip_filter_set_initial_heading_command(struct mip_serializer* serializer, const struct mip_filter_set_initial_heading_command* self)
{
    insert_float(serializer, self->heading);
}

void extract_mip_filter_set_initial_heading_command(struct mip_serializer* serializer, struct mip_filter_set_initial_heading_command* self)
{
    extract_float(serializer, &self->heading);
}

/// @brief Set the initial heading angle.
/// 
/// The estimation filter will reset the heading estimate to provided value. If the product supports magnetometer aiding and this feature has been enabled, the heading
/// argument will be ignored and the filter will initialize using the inferred magnetic heading.
/// @param heading Initial heading in radians [-pi, pi]
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_filter_set_initial_heading(struct mip_interface* device, float heading)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_float(&serializer, heading);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SET_INITIAL_HEADING, buffer, serializer.offset);
}


#ifdef __cplusplus
} // namespace C
} // namespace mip
} // extern "C"
#endif // __cplusplus

