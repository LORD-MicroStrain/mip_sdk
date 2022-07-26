
#include "commands_filter.h"

#include "../../utils/serialization.h"
#include "../mip_interface.h"

#include <assert.h>


#ifdef __cplusplus
namespace mscl {
extern "C" {
#endif // __cplusplus


////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////

size_t insert_mip_filter_reference_frame(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_filter_reference_frame self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}
size_t extract_mip_filter_reference_frame(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_filter_reference_frame* self)
{
    uint8_t tmp;
    offset = extract_u8(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}

size_t insert_mip_filter_mag_declination_source(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_filter_mag_declination_source self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}
size_t extract_mip_filter_mag_declination_source(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_filter_mag_declination_source* self)
{
    uint8_t tmp;
    offset = extract_u8(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_reset_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_reset_command* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}

size_t extract_mip_filter_reset_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_reset_command* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}


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

////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_set_initial_attitude_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_set_initial_attitude_command* self)
{
    offset = insert_float(buffer, bufferSize, offset, self->roll);
    offset = insert_float(buffer, bufferSize, offset, self->pitch);
    offset = insert_float(buffer, bufferSize, offset, self->heading);
    
    return offset;
}

size_t extract_mip_filter_set_initial_attitude_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_set_initial_attitude_command* self)
{
    offset = extract_float(buffer, bufferSize, offset, &self->roll);
    offset = extract_float(buffer, bufferSize, offset, &self->pitch);
    offset = extract_float(buffer, bufferSize, offset, &self->heading);
    
    return offset;
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
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_float(buffer, sizeof(buffer), cmdUsed, roll);
    cmdUsed = insert_float(buffer, sizeof(buffer), cmdUsed, pitch);
    cmdUsed = insert_float(buffer, sizeof(buffer), cmdUsed, heading);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SET_INITIAL_ATTITUDE, buffer, cmdUsed);
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_estimation_control_command_enable_flags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_filter_estimation_control_command_enable_flags self)
{
    return insert_u16(buffer, bufferSize, offset, self);
}
size_t extract_mip_filter_estimation_control_command_enable_flags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_filter_estimation_control_command_enable_flags* self)
{
    uint16_t tmp;
    offset = extract_u16(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_mip_filter_estimation_control_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_estimation_control_command* self)
{
    offset = insert_mip_filter_estimation_control_command_enable_flags(buffer, bufferSize, offset, self->enable);
    
    return offset;
}

size_t extract_mip_filter_estimation_control_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_estimation_control_command* self)
{
    offset = extract_mip_filter_estimation_control_command_enable_flags(buffer, bufferSize, offset, &self->enable);
    
    return offset;
}


size_t insert_mip_filter_estimation_control_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_estimation_control_response* self)
{
    offset = insert_mip_filter_estimation_control_command_enable_flags(buffer, bufferSize, offset, self->enable);
    
    return offset;
}

size_t extract_mip_filter_estimation_control_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_estimation_control_response* self)
{
    offset = extract_mip_filter_estimation_control_command_enable_flags(buffer, bufferSize, offset, &self->enable);
    
    return offset;
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
mip_cmd_result write_mip_filter_estimation_control(struct mip_interface* device, enum mip_filter_estimation_control_command_enable_flags enable)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_WRITE);
    cmdUsed = insert_mip_filter_estimation_control_command_enable_flags(buffer, sizeof(buffer), cmdUsed, enable);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ESTIMATION_CONTROL_FLAGS, buffer, cmdUsed);
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
mip_cmd_result read_mip_filter_estimation_control(struct mip_interface* device, enum mip_filter_estimation_control_command_enable_flags* enable)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_READ);
    assert(cmdUsed <= sizeof(buffer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ESTIMATION_CONTROL_FLAGS, buffer, cmdUsed, MIP_REPLY_DESC_FILTER_ESTIMATION_CONTROL_FLAGS, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        size_t responseUsed = 0;
        responseUsed = extract_mip_filter_estimation_control_command_enable_flags(buffer, sizeof(buffer), responseUsed, enable);
        
        if( responseUsed != responseLength )
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
mip_cmd_result save_mip_filter_estimation_control(struct mip_interface* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_SAVE);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ESTIMATION_CONTROL_FLAGS, buffer, cmdUsed);
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
mip_cmd_result load_mip_filter_estimation_control(struct mip_interface* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_LOAD);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ESTIMATION_CONTROL_FLAGS, buffer, cmdUsed);
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
mip_cmd_result default_mip_filter_estimation_control(struct mip_interface* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_RESET);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ESTIMATION_CONTROL_FLAGS, buffer, cmdUsed);
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_external_gnss_update_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_external_gnss_update_command* self)
{
    offset = insert_double(buffer, bufferSize, offset, self->gps_time);
    offset = insert_u16(buffer, bufferSize, offset, self->gps_week);
    offset = insert_double(buffer, bufferSize, offset, self->latitude);
    offset = insert_double(buffer, bufferSize, offset, self->longitude);
    offset = insert_double(buffer, bufferSize, offset, self->height);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->velocity[i]);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->pos_uncertainty[i]);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->vel_uncertainty[i]);
    
    return offset;
}

size_t extract_mip_filter_external_gnss_update_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_external_gnss_update_command* self)
{
    offset = extract_double(buffer, bufferSize, offset, &self->gps_time);
    offset = extract_u16(buffer, bufferSize, offset, &self->gps_week);
    offset = extract_double(buffer, bufferSize, offset, &self->latitude);
    offset = extract_double(buffer, bufferSize, offset, &self->longitude);
    offset = extract_double(buffer, bufferSize, offset, &self->height);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->velocity[i]);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->pos_uncertainty[i]);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->vel_uncertainty[i]);
    
    return offset;
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
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_double(buffer, sizeof(buffer), cmdUsed, gps_time);
    cmdUsed = insert_u16(buffer, sizeof(buffer), cmdUsed, gps_week);
    cmdUsed = insert_double(buffer, sizeof(buffer), cmdUsed, latitude);
    cmdUsed = insert_double(buffer, sizeof(buffer), cmdUsed, longitude);
    cmdUsed = insert_double(buffer, sizeof(buffer), cmdUsed, height);
    for(unsigned int i=0; i < 3; i++)
        cmdUsed = insert_float(buffer, sizeof(buffer), cmdUsed, velocity[i]);
    for(unsigned int i=0; i < 3; i++)
        cmdUsed = insert_float(buffer, sizeof(buffer), cmdUsed, pos_uncertainty[i]);
    for(unsigned int i=0; i < 3; i++)
        cmdUsed = insert_float(buffer, sizeof(buffer), cmdUsed, vel_uncertainty[i]);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_EXTERNAL_GNSS_UPDATE, buffer, cmdUsed);
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_external_heading_update_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_external_heading_update_command* self)
{
    offset = insert_float(buffer, bufferSize, offset, self->heading);
    offset = insert_float(buffer, bufferSize, offset, self->heading_uncertainty);
    offset = insert_u8(buffer, bufferSize, offset, self->type);
    
    return offset;
}

size_t extract_mip_filter_external_heading_update_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_external_heading_update_command* self)
{
    offset = extract_float(buffer, bufferSize, offset, &self->heading);
    offset = extract_float(buffer, bufferSize, offset, &self->heading_uncertainty);
    offset = extract_u8(buffer, bufferSize, offset, &self->type);
    
    return offset;
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
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_float(buffer, sizeof(buffer), cmdUsed, heading);
    cmdUsed = insert_float(buffer, sizeof(buffer), cmdUsed, heading_uncertainty);
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, type);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_EXTERNAL_HEADING_UPDATE, buffer, cmdUsed);
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_external_heading_update_with_time_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_external_heading_update_with_time_command* self)
{
    offset = insert_double(buffer, bufferSize, offset, self->gps_time);
    offset = insert_u16(buffer, bufferSize, offset, self->gps_week);
    offset = insert_float(buffer, bufferSize, offset, self->heading);
    offset = insert_float(buffer, bufferSize, offset, self->heading_uncertainty);
    offset = insert_u8(buffer, bufferSize, offset, self->type);
    
    return offset;
}

size_t extract_mip_filter_external_heading_update_with_time_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_external_heading_update_with_time_command* self)
{
    offset = extract_double(buffer, bufferSize, offset, &self->gps_time);
    offset = extract_u16(buffer, bufferSize, offset, &self->gps_week);
    offset = extract_float(buffer, bufferSize, offset, &self->heading);
    offset = extract_float(buffer, bufferSize, offset, &self->heading_uncertainty);
    offset = extract_u8(buffer, bufferSize, offset, &self->type);
    
    return offset;
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
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_double(buffer, sizeof(buffer), cmdUsed, gps_time);
    cmdUsed = insert_u16(buffer, sizeof(buffer), cmdUsed, gps_week);
    cmdUsed = insert_float(buffer, sizeof(buffer), cmdUsed, heading);
    cmdUsed = insert_float(buffer, sizeof(buffer), cmdUsed, heading_uncertainty);
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, type);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_EXTERNAL_HEADING_UPDATE_WITH_TIME, buffer, cmdUsed);
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_tare_orientation_command_mip_tare_axes(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_filter_tare_orientation_command_mip_tare_axes self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}
size_t extract_mip_filter_tare_orientation_command_mip_tare_axes(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_filter_tare_orientation_command_mip_tare_axes* self)
{
    uint8_t tmp;
    offset = extract_u8(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_mip_filter_tare_orientation_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_tare_orientation_command* self)
{
    offset = insert_mip_filter_tare_orientation_command_mip_tare_axes(buffer, bufferSize, offset, self->axes);
    
    return offset;
}

size_t extract_mip_filter_tare_orientation_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_tare_orientation_command* self)
{
    offset = extract_mip_filter_tare_orientation_command_mip_tare_axes(buffer, bufferSize, offset, &self->axes);
    
    return offset;
}


size_t insert_mip_filter_tare_orientation_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_tare_orientation_response* self)
{
    offset = insert_mip_filter_tare_orientation_command_mip_tare_axes(buffer, bufferSize, offset, self->axes);
    
    return offset;
}

size_t extract_mip_filter_tare_orientation_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_tare_orientation_response* self)
{
    offset = extract_mip_filter_tare_orientation_command_mip_tare_axes(buffer, bufferSize, offset, &self->axes);
    
    return offset;
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
mip_cmd_result write_mip_filter_tare_orientation(struct mip_interface* device, enum mip_filter_tare_orientation_command_mip_tare_axes axes)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_WRITE);
    cmdUsed = insert_mip_filter_tare_orientation_command_mip_tare_axes(buffer, sizeof(buffer), cmdUsed, axes);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_TARE_ORIENTATION, buffer, cmdUsed);
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
mip_cmd_result read_mip_filter_tare_orientation(struct mip_interface* device, enum mip_filter_tare_orientation_command_mip_tare_axes* axes)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_READ);
    assert(cmdUsed <= sizeof(buffer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_TARE_ORIENTATION, buffer, cmdUsed, MIP_REPLY_DESC_FILTER_TARE_ORIENTATION, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        size_t responseUsed = 0;
        responseUsed = extract_mip_filter_tare_orientation_command_mip_tare_axes(buffer, sizeof(buffer), responseUsed, axes);
        
        if( responseUsed != responseLength )
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
mip_cmd_result save_mip_filter_tare_orientation(struct mip_interface* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_SAVE);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_TARE_ORIENTATION, buffer, cmdUsed);
}

/// @brief Tare the device orientation.
/// 
/// This function uses the current device orientation relative to the NED frame as the current sensor to vehicle transformation.
/// This command is provided as a convenient way to set the sensor to vehicle frame transformation.
/// The filter must be initialized and have a valid attitude output. If the attitude is not valid, an error will be returned.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result load_mip_filter_tare_orientation(struct mip_interface* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_LOAD);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_TARE_ORIENTATION, buffer, cmdUsed);
}

/// @brief Tare the device orientation.
/// 
/// This function uses the current device orientation relative to the NED frame as the current sensor to vehicle transformation.
/// This command is provided as a convenient way to set the sensor to vehicle frame transformation.
/// The filter must be initialized and have a valid attitude output. If the attitude is not valid, an error will be returned.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result default_mip_filter_tare_orientation(struct mip_interface* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_RESET);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_TARE_ORIENTATION, buffer, cmdUsed);
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_sensor_to_vehicle_rotation_euler_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_sensor_to_vehicle_rotation_euler_command* self)
{
    offset = insert_float(buffer, bufferSize, offset, self->roll);
    offset = insert_float(buffer, bufferSize, offset, self->pitch);
    offset = insert_float(buffer, bufferSize, offset, self->yaw);
    
    return offset;
}

size_t extract_mip_filter_sensor_to_vehicle_rotation_euler_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_sensor_to_vehicle_rotation_euler_command* self)
{
    offset = extract_float(buffer, bufferSize, offset, &self->roll);
    offset = extract_float(buffer, bufferSize, offset, &self->pitch);
    offset = extract_float(buffer, bufferSize, offset, &self->yaw);
    
    return offset;
}


size_t insert_mip_filter_sensor_to_vehicle_rotation_euler_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_sensor_to_vehicle_rotation_euler_response* self)
{
    offset = insert_float(buffer, bufferSize, offset, self->roll);
    offset = insert_float(buffer, bufferSize, offset, self->pitch);
    offset = insert_float(buffer, bufferSize, offset, self->yaw);
    
    return offset;
}

size_t extract_mip_filter_sensor_to_vehicle_rotation_euler_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_sensor_to_vehicle_rotation_euler_response* self)
{
    offset = extract_float(buffer, bufferSize, offset, &self->roll);
    offset = extract_float(buffer, bufferSize, offset, &self->pitch);
    offset = extract_float(buffer, bufferSize, offset, &self->yaw);
    
    return offset;
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
mip_cmd_result write_mip_filter_sensor_to_vehicle_rotation_euler(struct mip_interface* device, float roll, float pitch, float yaw)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_WRITE);
    cmdUsed = insert_float(buffer, sizeof(buffer), cmdUsed, roll);
    cmdUsed = insert_float(buffer, sizeof(buffer), cmdUsed, pitch);
    cmdUsed = insert_float(buffer, sizeof(buffer), cmdUsed, yaw);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_ROTATION_EULER, buffer, cmdUsed);
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
mip_cmd_result read_mip_filter_sensor_to_vehicle_rotation_euler(struct mip_interface* device, float* roll, float* pitch, float* yaw)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_READ);
    assert(cmdUsed <= sizeof(buffer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_ROTATION_EULER, buffer, cmdUsed, MIP_REPLY_DESC_FILTER_SENSOR2VEHICLE_ROTATION_EULER, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        size_t responseUsed = 0;
        responseUsed = extract_float(buffer, sizeof(buffer), responseUsed, roll);
        responseUsed = extract_float(buffer, sizeof(buffer), responseUsed, pitch);
        responseUsed = extract_float(buffer, sizeof(buffer), responseUsed, yaw);
        
        if( responseUsed != responseLength )
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
mip_cmd_result save_mip_filter_sensor_to_vehicle_rotation_euler(struct mip_interface* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_SAVE);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_ROTATION_EULER, buffer, cmdUsed);
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
mip_cmd_result load_mip_filter_sensor_to_vehicle_rotation_euler(struct mip_interface* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_LOAD);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_ROTATION_EULER, buffer, cmdUsed);
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
mip_cmd_result default_mip_filter_sensor_to_vehicle_rotation_euler(struct mip_interface* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_RESET);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_ROTATION_EULER, buffer, cmdUsed);
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_sensor_to_vehicle_rotation_dcm_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_sensor_to_vehicle_rotation_dcm_command* self)
{
    for(unsigned int i=0; i < 9; i++)
        offset = insert_float(buffer, bufferSize, offset, self->dcm[i]);
    
    return offset;
}

size_t extract_mip_filter_sensor_to_vehicle_rotation_dcm_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_sensor_to_vehicle_rotation_dcm_command* self)
{
    for(unsigned int i=0; i < 9; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->dcm[i]);
    
    return offset;
}


size_t insert_mip_filter_sensor_to_vehicle_rotation_dcm_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_sensor_to_vehicle_rotation_dcm_response* self)
{
    for(unsigned int i=0; i < 9; i++)
        offset = insert_float(buffer, bufferSize, offset, self->dcm[i]);
    
    return offset;
}

size_t extract_mip_filter_sensor_to_vehicle_rotation_dcm_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_sensor_to_vehicle_rotation_dcm_response* self)
{
    for(unsigned int i=0; i < 9; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->dcm[i]);
    
    return offset;
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
mip_cmd_result write_mip_filter_sensor_to_vehicle_rotation_dcm(struct mip_interface* device, const float* dcm)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_WRITE);
    for(unsigned int i=0; i < 9; i++)
        cmdUsed = insert_float(buffer, sizeof(buffer), cmdUsed, dcm[i]);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_ROTATION_DCM, buffer, cmdUsed);
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
mip_cmd_result read_mip_filter_sensor_to_vehicle_rotation_dcm(struct mip_interface* device, float* dcm)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_READ);
    assert(cmdUsed <= sizeof(buffer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_ROTATION_DCM, buffer, cmdUsed, MIP_REPLY_DESC_FILTER_SENSOR2VEHICLE_ROTATION_DCM, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        size_t responseUsed = 0;
        for(unsigned int i=0; i < 9; i++)
            responseUsed = extract_float(buffer, sizeof(buffer), responseUsed, &dcm[i]);
        
        if( responseUsed != responseLength )
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
mip_cmd_result save_mip_filter_sensor_to_vehicle_rotation_dcm(struct mip_interface* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_SAVE);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_ROTATION_DCM, buffer, cmdUsed);
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
mip_cmd_result load_mip_filter_sensor_to_vehicle_rotation_dcm(struct mip_interface* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_LOAD);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_ROTATION_DCM, buffer, cmdUsed);
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
mip_cmd_result default_mip_filter_sensor_to_vehicle_rotation_dcm(struct mip_interface* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_RESET);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_ROTATION_DCM, buffer, cmdUsed);
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_sensor_to_vehicle_rotation_quaternion_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_sensor_to_vehicle_rotation_quaternion_command* self)
{
    for(unsigned int i=0; i < 4; i++)
        offset = insert_float(buffer, bufferSize, offset, self->quat[i]);
    
    return offset;
}

size_t extract_mip_filter_sensor_to_vehicle_rotation_quaternion_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_sensor_to_vehicle_rotation_quaternion_command* self)
{
    for(unsigned int i=0; i < 4; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->quat[i]);
    
    return offset;
}


size_t insert_mip_filter_sensor_to_vehicle_rotation_quaternion_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_sensor_to_vehicle_rotation_quaternion_response* self)
{
    for(unsigned int i=0; i < 4; i++)
        offset = insert_float(buffer, bufferSize, offset, self->quat[i]);
    
    return offset;
}

size_t extract_mip_filter_sensor_to_vehicle_rotation_quaternion_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_sensor_to_vehicle_rotation_quaternion_response* self)
{
    for(unsigned int i=0; i < 4; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->quat[i]);
    
    return offset;
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
mip_cmd_result write_mip_filter_sensor_to_vehicle_rotation_quaternion(struct mip_interface* device, const float* quat)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_WRITE);
    for(unsigned int i=0; i < 4; i++)
        cmdUsed = insert_float(buffer, sizeof(buffer), cmdUsed, quat[i]);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_ROTATION_QUATERNION, buffer, cmdUsed);
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
mip_cmd_result read_mip_filter_sensor_to_vehicle_rotation_quaternion(struct mip_interface* device, float* quat)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_READ);
    assert(cmdUsed <= sizeof(buffer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_ROTATION_QUATERNION, buffer, cmdUsed, MIP_REPLY_DESC_FILTER_SENSOR2VEHICLE_ROTATION_QUATERNION, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        size_t responseUsed = 0;
        for(unsigned int i=0; i < 4; i++)
            responseUsed = extract_float(buffer, sizeof(buffer), responseUsed, &quat[i]);
        
        if( responseUsed != responseLength )
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
mip_cmd_result save_mip_filter_sensor_to_vehicle_rotation_quaternion(struct mip_interface* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_SAVE);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_ROTATION_QUATERNION, buffer, cmdUsed);
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
mip_cmd_result load_mip_filter_sensor_to_vehicle_rotation_quaternion(struct mip_interface* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_LOAD);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_ROTATION_QUATERNION, buffer, cmdUsed);
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
mip_cmd_result default_mip_filter_sensor_to_vehicle_rotation_quaternion(struct mip_interface* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_RESET);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_ROTATION_QUATERNION, buffer, cmdUsed);
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_sensor_to_vehicle_offset_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_sensor_to_vehicle_offset_command* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->offset[i]);
    
    return offset;
}

size_t extract_mip_filter_sensor_to_vehicle_offset_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_sensor_to_vehicle_offset_command* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->offset[i]);
    
    return offset;
}


size_t insert_mip_filter_sensor_to_vehicle_offset_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_sensor_to_vehicle_offset_response* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->offset[i]);
    
    return offset;
}

size_t extract_mip_filter_sensor_to_vehicle_offset_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_sensor_to_vehicle_offset_response* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->offset[i]);
    
    return offset;
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
mip_cmd_result write_mip_filter_sensor_to_vehicle_offset(struct mip_interface* device, const float* offset)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_WRITE);
    for(unsigned int i=0; i < 3; i++)
        cmdUsed = insert_float(buffer, sizeof(buffer), cmdUsed, offset[i]);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_OFFSET, buffer, cmdUsed);
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
mip_cmd_result read_mip_filter_sensor_to_vehicle_offset(struct mip_interface* device, float* offset)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_READ);
    assert(cmdUsed <= sizeof(buffer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_OFFSET, buffer, cmdUsed, MIP_REPLY_DESC_FILTER_SENSOR2VEHICLE_OFFSET, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        size_t responseUsed = 0;
        for(unsigned int i=0; i < 3; i++)
            responseUsed = extract_float(buffer, sizeof(buffer), responseUsed, &offset[i]);
        
        if( responseUsed != responseLength )
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
mip_cmd_result save_mip_filter_sensor_to_vehicle_offset(struct mip_interface* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_SAVE);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_OFFSET, buffer, cmdUsed);
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
mip_cmd_result load_mip_filter_sensor_to_vehicle_offset(struct mip_interface* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_LOAD);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_OFFSET, buffer, cmdUsed);
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
mip_cmd_result default_mip_filter_sensor_to_vehicle_offset(struct mip_interface* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_RESET);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_OFFSET, buffer, cmdUsed);
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_antenna_offset_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_antenna_offset_command* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->offset[i]);
    
    return offset;
}

size_t extract_mip_filter_antenna_offset_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_antenna_offset_command* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->offset[i]);
    
    return offset;
}


size_t insert_mip_filter_antenna_offset_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_antenna_offset_response* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->offset[i]);
    
    return offset;
}

size_t extract_mip_filter_antenna_offset_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_antenna_offset_response* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->offset[i]);
    
    return offset;
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
mip_cmd_result write_mip_filter_antenna_offset(struct mip_interface* device, const float* offset)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_WRITE);
    for(unsigned int i=0; i < 3; i++)
        cmdUsed = insert_float(buffer, sizeof(buffer), cmdUsed, offset[i]);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ANTENNA_OFFSET, buffer, cmdUsed);
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
mip_cmd_result read_mip_filter_antenna_offset(struct mip_interface* device, float* offset)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_READ);
    assert(cmdUsed <= sizeof(buffer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ANTENNA_OFFSET, buffer, cmdUsed, MIP_REPLY_DESC_FILTER_ANTENNA_OFFSET, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        size_t responseUsed = 0;
        for(unsigned int i=0; i < 3; i++)
            responseUsed = extract_float(buffer, sizeof(buffer), responseUsed, &offset[i]);
        
        if( responseUsed != responseLength )
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
mip_cmd_result save_mip_filter_antenna_offset(struct mip_interface* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_SAVE);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ANTENNA_OFFSET, buffer, cmdUsed);
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
mip_cmd_result load_mip_filter_antenna_offset(struct mip_interface* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_LOAD);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ANTENNA_OFFSET, buffer, cmdUsed);
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
mip_cmd_result default_mip_filter_antenna_offset(struct mip_interface* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_RESET);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ANTENNA_OFFSET, buffer, cmdUsed);
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_gnss_source_command_gnss_source(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_filter_gnss_source_command_gnss_source self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}
size_t extract_mip_filter_gnss_source_command_gnss_source(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_filter_gnss_source_command_gnss_source* self)
{
    uint8_t tmp;
    offset = extract_u8(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}

size_t insert_mip_filter_gnss_source_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_gnss_source_command* self)
{
    offset = insert_mip_filter_gnss_source_command_gnss_source(buffer, bufferSize, offset, self->source);
    
    return offset;
}

size_t extract_mip_filter_gnss_source_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_gnss_source_command* self)
{
    offset = extract_mip_filter_gnss_source_command_gnss_source(buffer, bufferSize, offset, &self->source);
    
    return offset;
}


size_t insert_mip_filter_gnss_source_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_gnss_source_response* self)
{
    offset = insert_mip_filter_gnss_source_command_gnss_source(buffer, bufferSize, offset, self->source);
    
    return offset;
}

size_t extract_mip_filter_gnss_source_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_gnss_source_response* self)
{
    offset = extract_mip_filter_gnss_source_command_gnss_source(buffer, bufferSize, offset, &self->source);
    
    return offset;
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
mip_cmd_result write_mip_filter_gnss_source(struct mip_interface* device, enum mip_filter_gnss_source_command_gnss_source source)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_WRITE);
    cmdUsed = insert_mip_filter_gnss_source_command_gnss_source(buffer, sizeof(buffer), cmdUsed, source);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_GNSS_SOURCE_CONTROL, buffer, cmdUsed);
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
mip_cmd_result read_mip_filter_gnss_source(struct mip_interface* device, enum mip_filter_gnss_source_command_gnss_source* source)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_READ);
    assert(cmdUsed <= sizeof(buffer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_GNSS_SOURCE_CONTROL, buffer, cmdUsed, MIP_REPLY_DESC_FILTER_GNSS_SOURCE_CONTROL, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        size_t responseUsed = 0;
        responseUsed = extract_mip_filter_gnss_source_command_gnss_source(buffer, sizeof(buffer), responseUsed, source);
        
        if( responseUsed != responseLength )
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
mip_cmd_result save_mip_filter_gnss_source(struct mip_interface* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_SAVE);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_GNSS_SOURCE_CONTROL, buffer, cmdUsed);
}

/// @brief Control the source of GNSS information used to update the Kalman Filter.
/// 
/// Changing the GNSS source while the sensor is in the "running" state may temporarily place
/// it back in the "init" state until the new source of GNSS data is received.
/// 
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result load_mip_filter_gnss_source(struct mip_interface* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_LOAD);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_GNSS_SOURCE_CONTROL, buffer, cmdUsed);
}

/// @brief Control the source of GNSS information used to update the Kalman Filter.
/// 
/// Changing the GNSS source while the sensor is in the "running" state may temporarily place
/// it back in the "init" state until the new source of GNSS data is received.
/// 
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result default_mip_filter_gnss_source(struct mip_interface* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_RESET);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_GNSS_SOURCE_CONTROL, buffer, cmdUsed);
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_heading_source_command_heading_source(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_filter_heading_source_command_heading_source self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}
size_t extract_mip_filter_heading_source_command_heading_source(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_filter_heading_source_command_heading_source* self)
{
    uint8_t tmp;
    offset = extract_u8(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}

size_t insert_mip_filter_heading_source_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_heading_source_command* self)
{
    offset = insert_mip_filter_heading_source_command_heading_source(buffer, bufferSize, offset, self->source);
    
    return offset;
}

size_t extract_mip_filter_heading_source_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_heading_source_command* self)
{
    offset = extract_mip_filter_heading_source_command_heading_source(buffer, bufferSize, offset, &self->source);
    
    return offset;
}


size_t insert_mip_filter_heading_source_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_heading_source_response* self)
{
    offset = insert_mip_filter_heading_source_command_heading_source(buffer, bufferSize, offset, self->source);
    
    return offset;
}

size_t extract_mip_filter_heading_source_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_heading_source_response* self)
{
    offset = extract_mip_filter_heading_source_command_heading_source(buffer, bufferSize, offset, &self->source);
    
    return offset;
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
mip_cmd_result write_mip_filter_heading_source(struct mip_interface* device, enum mip_filter_heading_source_command_heading_source source)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_WRITE);
    cmdUsed = insert_mip_filter_heading_source_command_heading_source(buffer, sizeof(buffer), cmdUsed, source);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_HEADING_UPDATE_CONTROL, buffer, cmdUsed);
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
mip_cmd_result read_mip_filter_heading_source(struct mip_interface* device, enum mip_filter_heading_source_command_heading_source* source)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_READ);
    assert(cmdUsed <= sizeof(buffer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_HEADING_UPDATE_CONTROL, buffer, cmdUsed, MIP_REPLY_DESC_FILTER_HEADING_UPDATE_CONTROL, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        size_t responseUsed = 0;
        responseUsed = extract_mip_filter_heading_source_command_heading_source(buffer, sizeof(buffer), responseUsed, source);
        
        if( responseUsed != responseLength )
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
mip_cmd_result save_mip_filter_heading_source(struct mip_interface* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_SAVE);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_HEADING_UPDATE_CONTROL, buffer, cmdUsed);
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
mip_cmd_result load_mip_filter_heading_source(struct mip_interface* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_LOAD);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_HEADING_UPDATE_CONTROL, buffer, cmdUsed);
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
mip_cmd_result default_mip_filter_heading_source(struct mip_interface* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_RESET);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_HEADING_UPDATE_CONTROL, buffer, cmdUsed);
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_altitude_aiding_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_altitude_aiding_command* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->aiding_selector);
    
    return offset;
}

size_t extract_mip_filter_altitude_aiding_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_altitude_aiding_command* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->aiding_selector);
    
    return offset;
}


size_t insert_mip_filter_altitude_aiding_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_altitude_aiding_response* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->aiding_selector);
    
    return offset;
}

size_t extract_mip_filter_altitude_aiding_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_altitude_aiding_response* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->aiding_selector);
    
    return offset;
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
mip_cmd_result write_mip_filter_altitude_aiding(struct mip_interface* device, uint8_t aiding_selector)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_WRITE);
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, aiding_selector);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ALTITUDE_AIDING_CONTROL, buffer, cmdUsed);
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
mip_cmd_result read_mip_filter_altitude_aiding(struct mip_interface* device, uint8_t* aiding_selector)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_READ);
    assert(cmdUsed <= sizeof(buffer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ALTITUDE_AIDING_CONTROL, buffer, cmdUsed, MIP_REPLY_DESC_FILTER_ALTITUDE_AIDING_CONTROL, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        size_t responseUsed = 0;
        responseUsed = extract_u8(buffer, sizeof(buffer), responseUsed, aiding_selector);
        
        if( responseUsed != responseLength )
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
mip_cmd_result save_mip_filter_altitude_aiding(struct mip_interface* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_SAVE);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ALTITUDE_AIDING_CONTROL, buffer, cmdUsed);
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
mip_cmd_result load_mip_filter_altitude_aiding(struct mip_interface* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_LOAD);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ALTITUDE_AIDING_CONTROL, buffer, cmdUsed);
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
mip_cmd_result default_mip_filter_altitude_aiding(struct mip_interface* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_RESET);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ALTITUDE_AIDING_CONTROL, buffer, cmdUsed);
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_auto_zupt_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_auto_zupt_command* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->enable);
    offset = insert_float(buffer, bufferSize, offset, self->threshold);
    
    return offset;
}

size_t extract_mip_filter_auto_zupt_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_auto_zupt_command* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->enable);
    offset = extract_float(buffer, bufferSize, offset, &self->threshold);
    
    return offset;
}


size_t insert_mip_filter_auto_zupt_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_auto_zupt_response* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->enable);
    offset = insert_float(buffer, bufferSize, offset, self->threshold);
    
    return offset;
}

size_t extract_mip_filter_auto_zupt_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_auto_zupt_response* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->enable);
    offset = extract_float(buffer, bufferSize, offset, &self->threshold);
    
    return offset;
}


/// @brief Zero Velocity Update
/// The ZUPT is triggered when the scalar magnitude of the GNSS reported velocity vector is equal-to or less than the threshold value.
/// The device will NACK threshold values that are less than zero (i.e.negative.)
/// @param enable 0 - Disable, 1 - Enable
/// @param threshold [meters/second]
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result write_mip_filter_auto_zupt(struct mip_interface* device, uint8_t enable, float threshold)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_WRITE);
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, enable);
    cmdUsed = insert_float(buffer, sizeof(buffer), cmdUsed, threshold);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ZUPT_CONTROL, buffer, cmdUsed);
}

/// @brief Zero Velocity Update
/// The ZUPT is triggered when the scalar magnitude of the GNSS reported velocity vector is equal-to or less than the threshold value.
/// The device will NACK threshold values that are less than zero (i.e.negative.)
/// @param[out] enable 0 - Disable, 1 - Enable
/// @param[out] threshold [meters/second]
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result read_mip_filter_auto_zupt(struct mip_interface* device, uint8_t* enable, float* threshold)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_READ);
    assert(cmdUsed <= sizeof(buffer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ZUPT_CONTROL, buffer, cmdUsed, MIP_REPLY_DESC_FILTER_ZUPT_CONTROL, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        size_t responseUsed = 0;
        responseUsed = extract_u8(buffer, sizeof(buffer), responseUsed, enable);
        responseUsed = extract_float(buffer, sizeof(buffer), responseUsed, threshold);
        
        if( responseUsed != responseLength )
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
mip_cmd_result save_mip_filter_auto_zupt(struct mip_interface* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_SAVE);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ZUPT_CONTROL, buffer, cmdUsed);
}

/// @brief Zero Velocity Update
/// The ZUPT is triggered when the scalar magnitude of the GNSS reported velocity vector is equal-to or less than the threshold value.
/// The device will NACK threshold values that are less than zero (i.e.negative.)
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result load_mip_filter_auto_zupt(struct mip_interface* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_LOAD);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ZUPT_CONTROL, buffer, cmdUsed);
}

/// @brief Zero Velocity Update
/// The ZUPT is triggered when the scalar magnitude of the GNSS reported velocity vector is equal-to or less than the threshold value.
/// The device will NACK threshold values that are less than zero (i.e.negative.)
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result default_mip_filter_auto_zupt(struct mip_interface* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_RESET);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ZUPT_CONTROL, buffer, cmdUsed);
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_auto_angular_zupt_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_auto_angular_zupt_command* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->enable);
    offset = insert_float(buffer, bufferSize, offset, self->threshold);
    
    return offset;
}

size_t extract_mip_filter_auto_angular_zupt_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_auto_angular_zupt_command* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->enable);
    offset = extract_float(buffer, bufferSize, offset, &self->threshold);
    
    return offset;
}


size_t insert_mip_filter_auto_angular_zupt_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_auto_angular_zupt_response* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->enable);
    offset = insert_float(buffer, bufferSize, offset, self->threshold);
    
    return offset;
}

size_t extract_mip_filter_auto_angular_zupt_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_auto_angular_zupt_response* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->enable);
    offset = extract_float(buffer, bufferSize, offset, &self->threshold);
    
    return offset;
}


/// @brief Zero Angular Rate Update
/// The ZUPT is triggered when the scalar magnitude of the angular rate vector is equal-to or less than the threshold value.
/// The device will NACK threshold values that are less than zero (i.e.negative.)
/// @param enable 0 - Disable, 1 - Enable
/// @param threshold [radians/second]
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result write_mip_filter_auto_angular_zupt(struct mip_interface* device, uint8_t enable, float threshold)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_WRITE);
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, enable);
    cmdUsed = insert_float(buffer, sizeof(buffer), cmdUsed, threshold);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ANGULAR_ZUPT_CONTROL, buffer, cmdUsed);
}

/// @brief Zero Angular Rate Update
/// The ZUPT is triggered when the scalar magnitude of the angular rate vector is equal-to or less than the threshold value.
/// The device will NACK threshold values that are less than zero (i.e.negative.)
/// @param[out] enable 0 - Disable, 1 - Enable
/// @param[out] threshold [radians/second]
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result read_mip_filter_auto_angular_zupt(struct mip_interface* device, uint8_t* enable, float* threshold)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_READ);
    assert(cmdUsed <= sizeof(buffer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ANGULAR_ZUPT_CONTROL, buffer, cmdUsed, MIP_REPLY_DESC_FILTER_ANGULAR_ZUPT_CONTROL, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        size_t responseUsed = 0;
        responseUsed = extract_u8(buffer, sizeof(buffer), responseUsed, enable);
        responseUsed = extract_float(buffer, sizeof(buffer), responseUsed, threshold);
        
        if( responseUsed != responseLength )
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
mip_cmd_result save_mip_filter_auto_angular_zupt(struct mip_interface* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_SAVE);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ANGULAR_ZUPT_CONTROL, buffer, cmdUsed);
}

/// @brief Zero Angular Rate Update
/// The ZUPT is triggered when the scalar magnitude of the angular rate vector is equal-to or less than the threshold value.
/// The device will NACK threshold values that are less than zero (i.e.negative.)
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result load_mip_filter_auto_angular_zupt(struct mip_interface* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_LOAD);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ANGULAR_ZUPT_CONTROL, buffer, cmdUsed);
}

/// @brief Zero Angular Rate Update
/// The ZUPT is triggered when the scalar magnitude of the angular rate vector is equal-to or less than the threshold value.
/// The device will NACK threshold values that are less than zero (i.e.negative.)
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result default_mip_filter_auto_angular_zupt(struct mip_interface* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_RESET);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ANGULAR_ZUPT_CONTROL, buffer, cmdUsed);
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_commanded_zupt_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_commanded_zupt_command* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}

size_t extract_mip_filter_commanded_zupt_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_commanded_zupt_command* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
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

////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_commanded_angular_zupt_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_commanded_angular_zupt_command* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}

size_t extract_mip_filter_commanded_angular_zupt_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_commanded_angular_zupt_command* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
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

////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_aiding_measurement_enable_command_aiding_source(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_filter_aiding_measurement_enable_command_aiding_source self)
{
    return insert_u16(buffer, bufferSize, offset, self);
}
size_t extract_mip_filter_aiding_measurement_enable_command_aiding_source(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_filter_aiding_measurement_enable_command_aiding_source* self)
{
    uint16_t tmp;
    offset = extract_u16(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}

size_t insert_mip_filter_aiding_measurement_enable_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_aiding_measurement_enable_command* self)
{
    offset = insert_mip_filter_aiding_measurement_enable_command_aiding_source(buffer, bufferSize, offset, self->aiding_source);
    offset = insert_bool(buffer, bufferSize, offset, self->enable);
    
    return offset;
}

size_t extract_mip_filter_aiding_measurement_enable_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_aiding_measurement_enable_command* self)
{
    offset = extract_mip_filter_aiding_measurement_enable_command_aiding_source(buffer, bufferSize, offset, &self->aiding_source);
    offset = extract_bool(buffer, bufferSize, offset, &self->enable);
    
    return offset;
}


size_t insert_mip_filter_aiding_measurement_enable_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_aiding_measurement_enable_response* self)
{
    offset = insert_mip_filter_aiding_measurement_enable_command_aiding_source(buffer, bufferSize, offset, self->aiding_source);
    offset = insert_bool(buffer, bufferSize, offset, self->enable);
    
    return offset;
}

size_t extract_mip_filter_aiding_measurement_enable_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_aiding_measurement_enable_response* self)
{
    offset = extract_mip_filter_aiding_measurement_enable_command_aiding_source(buffer, bufferSize, offset, &self->aiding_source);
    offset = extract_bool(buffer, bufferSize, offset, &self->enable);
    
    return offset;
}


/// @brief Enables / disables the specified aiding measurement source.
/// 
/// 
/// @param aiding_source Aiding measurement source
/// @param enable Controls the aiding sorce
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result write_mip_filter_aiding_measurement_enable(struct mip_interface* device, enum mip_filter_aiding_measurement_enable_command_aiding_source aiding_source, bool enable)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_WRITE);
    cmdUsed = insert_mip_filter_aiding_measurement_enable_command_aiding_source(buffer, sizeof(buffer), cmdUsed, aiding_source);
    cmdUsed = insert_bool(buffer, sizeof(buffer), cmdUsed, enable);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_AIDING_MEASUREMENT_ENABLE, buffer, cmdUsed);
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
mip_cmd_result read_mip_filter_aiding_measurement_enable(struct mip_interface* device, enum mip_filter_aiding_measurement_enable_command_aiding_source aiding_source, bool* enable)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_READ);
    cmdUsed = insert_mip_filter_aiding_measurement_enable_command_aiding_source(buffer, sizeof(buffer), cmdUsed, aiding_source);
    assert(cmdUsed <= sizeof(buffer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_AIDING_MEASUREMENT_ENABLE, buffer, cmdUsed, MIP_REPLY_DESC_FILTER_AIDING_MEASUREMENT_ENABLE, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        size_t responseUsed = 0;
        responseUsed = extract_mip_filter_aiding_measurement_enable_command_aiding_source(buffer, sizeof(buffer), responseUsed, &aiding_source);
        responseUsed = extract_bool(buffer, sizeof(buffer), responseUsed, enable);
        
        if( responseUsed != responseLength )
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
mip_cmd_result save_mip_filter_aiding_measurement_enable(struct mip_interface* device, enum mip_filter_aiding_measurement_enable_command_aiding_source aiding_source)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_SAVE);
    cmdUsed = insert_mip_filter_aiding_measurement_enable_command_aiding_source(buffer, sizeof(buffer), cmdUsed, aiding_source);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_AIDING_MEASUREMENT_ENABLE, buffer, cmdUsed);
}

/// @brief Enables / disables the specified aiding measurement source.
/// 
/// 
/// @param aiding_source Aiding measurement source
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result load_mip_filter_aiding_measurement_enable(struct mip_interface* device, enum mip_filter_aiding_measurement_enable_command_aiding_source aiding_source)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_LOAD);
    cmdUsed = insert_mip_filter_aiding_measurement_enable_command_aiding_source(buffer, sizeof(buffer), cmdUsed, aiding_source);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_AIDING_MEASUREMENT_ENABLE, buffer, cmdUsed);
}

/// @brief Enables / disables the specified aiding measurement source.
/// 
/// 
/// @param aiding_source Aiding measurement source
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result default_mip_filter_aiding_measurement_enable(struct mip_interface* device, enum mip_filter_aiding_measurement_enable_command_aiding_source aiding_source)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_RESET);
    cmdUsed = insert_mip_filter_aiding_measurement_enable_command_aiding_source(buffer, sizeof(buffer), cmdUsed, aiding_source);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_AIDING_MEASUREMENT_ENABLE, buffer, cmdUsed);
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_run_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_run_command* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}

size_t extract_mip_filter_run_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_run_command* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
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

////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_kinematic_constraint_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_kinematic_constraint_command* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->acceleration_constraint_selection);
    offset = insert_u8(buffer, bufferSize, offset, self->velocity_constraint_selection);
    offset = insert_u8(buffer, bufferSize, offset, self->angular_constraint_selection);
    
    return offset;
}

size_t extract_mip_filter_kinematic_constraint_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_kinematic_constraint_command* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->acceleration_constraint_selection);
    offset = extract_u8(buffer, bufferSize, offset, &self->velocity_constraint_selection);
    offset = extract_u8(buffer, bufferSize, offset, &self->angular_constraint_selection);
    
    return offset;
}


size_t insert_mip_filter_kinematic_constraint_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_kinematic_constraint_response* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->acceleration_constraint_selection);
    offset = insert_u8(buffer, bufferSize, offset, self->velocity_constraint_selection);
    offset = insert_u8(buffer, bufferSize, offset, self->angular_constraint_selection);
    
    return offset;
}

size_t extract_mip_filter_kinematic_constraint_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_kinematic_constraint_response* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->acceleration_constraint_selection);
    offset = extract_u8(buffer, bufferSize, offset, &self->velocity_constraint_selection);
    offset = extract_u8(buffer, bufferSize, offset, &self->angular_constraint_selection);
    
    return offset;
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
mip_cmd_result write_mip_filter_kinematic_constraint(struct mip_interface* device, uint8_t acceleration_constraint_selection, uint8_t velocity_constraint_selection, uint8_t angular_constraint_selection)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_WRITE);
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, acceleration_constraint_selection);
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, velocity_constraint_selection);
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, angular_constraint_selection);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_KINEMATIC_CONSTRAINT, buffer, cmdUsed);
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
mip_cmd_result read_mip_filter_kinematic_constraint(struct mip_interface* device, uint8_t* acceleration_constraint_selection, uint8_t* velocity_constraint_selection, uint8_t* angular_constraint_selection)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_READ);
    assert(cmdUsed <= sizeof(buffer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_KINEMATIC_CONSTRAINT, buffer, cmdUsed, MIP_REPLY_DESC_FILTER_KINEMATIC_CONSTRAINT, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        size_t responseUsed = 0;
        responseUsed = extract_u8(buffer, sizeof(buffer), responseUsed, acceleration_constraint_selection);
        responseUsed = extract_u8(buffer, sizeof(buffer), responseUsed, velocity_constraint_selection);
        responseUsed = extract_u8(buffer, sizeof(buffer), responseUsed, angular_constraint_selection);
        
        if( responseUsed != responseLength )
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
mip_cmd_result save_mip_filter_kinematic_constraint(struct mip_interface* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_SAVE);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_KINEMATIC_CONSTRAINT, buffer, cmdUsed);
}

/// @brief Controls kinematic constraint model selection for the navigation filter.
/// 
/// See manual for explanation of how the kinematic constraints are applied.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result load_mip_filter_kinematic_constraint(struct mip_interface* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_LOAD);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_KINEMATIC_CONSTRAINT, buffer, cmdUsed);
}

/// @brief Controls kinematic constraint model selection for the navigation filter.
/// 
/// See manual for explanation of how the kinematic constraints are applied.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result default_mip_filter_kinematic_constraint(struct mip_interface* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_RESET);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_KINEMATIC_CONSTRAINT, buffer, cmdUsed);
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_initialization_configuration_command_initial_condition_source(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_filter_initialization_configuration_command_initial_condition_source self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}
size_t extract_mip_filter_initialization_configuration_command_initial_condition_source(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_filter_initialization_configuration_command_initial_condition_source* self)
{
    uint8_t tmp;
    offset = extract_u8(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}

size_t insert_mip_filter_initialization_configuration_command_alignment_selector(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_filter_initialization_configuration_command_alignment_selector self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}
size_t extract_mip_filter_initialization_configuration_command_alignment_selector(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_filter_initialization_configuration_command_alignment_selector* self)
{
    uint8_t tmp;
    offset = extract_u8(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_mip_filter_initialization_configuration_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_initialization_configuration_command* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->wait_for_run_command);
    offset = insert_mip_filter_initialization_configuration_command_initial_condition_source(buffer, bufferSize, offset, self->initial_cond_src);
    offset = insert_mip_filter_initialization_configuration_command_alignment_selector(buffer, bufferSize, offset, self->auto_heading_alignment_selector);
    offset = insert_float(buffer, bufferSize, offset, self->initial_heading);
    offset = insert_float(buffer, bufferSize, offset, self->initial_pitch);
    offset = insert_float(buffer, bufferSize, offset, self->initial_roll);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->initial_position[i]);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->initial_velocity[i]);
    offset = insert_mip_filter_reference_frame(buffer, bufferSize, offset, self->reference_frame_selector);
    
    return offset;
}

size_t extract_mip_filter_initialization_configuration_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_initialization_configuration_command* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->wait_for_run_command);
    offset = extract_mip_filter_initialization_configuration_command_initial_condition_source(buffer, bufferSize, offset, &self->initial_cond_src);
    offset = extract_mip_filter_initialization_configuration_command_alignment_selector(buffer, bufferSize, offset, &self->auto_heading_alignment_selector);
    offset = extract_float(buffer, bufferSize, offset, &self->initial_heading);
    offset = extract_float(buffer, bufferSize, offset, &self->initial_pitch);
    offset = extract_float(buffer, bufferSize, offset, &self->initial_roll);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->initial_position[i]);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->initial_velocity[i]);
    offset = extract_mip_filter_reference_frame(buffer, bufferSize, offset, &self->reference_frame_selector);
    
    return offset;
}


size_t insert_mip_filter_initialization_configuration_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_initialization_configuration_response* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->wait_for_run_command);
    offset = insert_mip_filter_initialization_configuration_command_initial_condition_source(buffer, bufferSize, offset, self->initial_cond_src);
    offset = insert_mip_filter_initialization_configuration_command_alignment_selector(buffer, bufferSize, offset, self->auto_heading_alignment_selector);
    offset = insert_float(buffer, bufferSize, offset, self->initial_heading);
    offset = insert_float(buffer, bufferSize, offset, self->initial_pitch);
    offset = insert_float(buffer, bufferSize, offset, self->initial_roll);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->initial_position[i]);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->initial_velocity[i]);
    offset = insert_mip_filter_reference_frame(buffer, bufferSize, offset, self->reference_frame_selector);
    
    return offset;
}

size_t extract_mip_filter_initialization_configuration_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_initialization_configuration_response* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->wait_for_run_command);
    offset = extract_mip_filter_initialization_configuration_command_initial_condition_source(buffer, bufferSize, offset, &self->initial_cond_src);
    offset = extract_mip_filter_initialization_configuration_command_alignment_selector(buffer, bufferSize, offset, &self->auto_heading_alignment_selector);
    offset = extract_float(buffer, bufferSize, offset, &self->initial_heading);
    offset = extract_float(buffer, bufferSize, offset, &self->initial_pitch);
    offset = extract_float(buffer, bufferSize, offset, &self->initial_roll);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->initial_position[i]);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->initial_velocity[i]);
    offset = extract_mip_filter_reference_frame(buffer, bufferSize, offset, &self->reference_frame_selector);
    
    return offset;
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
mip_cmd_result write_mip_filter_initialization_configuration(struct mip_interface* device, uint8_t wait_for_run_command, enum mip_filter_initialization_configuration_command_initial_condition_source initial_cond_src, enum mip_filter_initialization_configuration_command_alignment_selector auto_heading_alignment_selector, float initial_heading, float initial_pitch, float initial_roll, const float* initial_position, const float* initial_velocity, enum mip_filter_reference_frame reference_frame_selector)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_WRITE);
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, wait_for_run_command);
    cmdUsed = insert_mip_filter_initialization_configuration_command_initial_condition_source(buffer, sizeof(buffer), cmdUsed, initial_cond_src);
    cmdUsed = insert_mip_filter_initialization_configuration_command_alignment_selector(buffer, sizeof(buffer), cmdUsed, auto_heading_alignment_selector);
    cmdUsed = insert_float(buffer, sizeof(buffer), cmdUsed, initial_heading);
    cmdUsed = insert_float(buffer, sizeof(buffer), cmdUsed, initial_pitch);
    cmdUsed = insert_float(buffer, sizeof(buffer), cmdUsed, initial_roll);
    for(unsigned int i=0; i < 3; i++)
        cmdUsed = insert_float(buffer, sizeof(buffer), cmdUsed, initial_position[i]);
    for(unsigned int i=0; i < 3; i++)
        cmdUsed = insert_float(buffer, sizeof(buffer), cmdUsed, initial_velocity[i]);
    cmdUsed = insert_mip_filter_reference_frame(buffer, sizeof(buffer), cmdUsed, reference_frame_selector);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_INITIALIZATION_CONFIGURATION, buffer, cmdUsed);
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
mip_cmd_result read_mip_filter_initialization_configuration(struct mip_interface* device, uint8_t* wait_for_run_command, enum mip_filter_initialization_configuration_command_initial_condition_source* initial_cond_src, enum mip_filter_initialization_configuration_command_alignment_selector* auto_heading_alignment_selector, float* initial_heading, float* initial_pitch, float* initial_roll, float* initial_position, float* initial_velocity, enum mip_filter_reference_frame* reference_frame_selector)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_READ);
    assert(cmdUsed <= sizeof(buffer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_INITIALIZATION_CONFIGURATION, buffer, cmdUsed, MIP_REPLY_DESC_FILTER_INITIALIZATION_CONFIGURATION, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        size_t responseUsed = 0;
        responseUsed = extract_u8(buffer, sizeof(buffer), responseUsed, wait_for_run_command);
        responseUsed = extract_mip_filter_initialization_configuration_command_initial_condition_source(buffer, sizeof(buffer), responseUsed, initial_cond_src);
        responseUsed = extract_mip_filter_initialization_configuration_command_alignment_selector(buffer, sizeof(buffer), responseUsed, auto_heading_alignment_selector);
        responseUsed = extract_float(buffer, sizeof(buffer), responseUsed, initial_heading);
        responseUsed = extract_float(buffer, sizeof(buffer), responseUsed, initial_pitch);
        responseUsed = extract_float(buffer, sizeof(buffer), responseUsed, initial_roll);
        for(unsigned int i=0; i < 3; i++)
            responseUsed = extract_float(buffer, sizeof(buffer), responseUsed, &initial_position[i]);
        for(unsigned int i=0; i < 3; i++)
            responseUsed = extract_float(buffer, sizeof(buffer), responseUsed, &initial_velocity[i]);
        responseUsed = extract_mip_filter_reference_frame(buffer, sizeof(buffer), responseUsed, reference_frame_selector);
        
        if( responseUsed != responseLength )
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
mip_cmd_result save_mip_filter_initialization_configuration(struct mip_interface* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_SAVE);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_INITIALIZATION_CONFIGURATION, buffer, cmdUsed);
}

/// @brief Controls the source and values used for initial conditions of the navigation solution.
/// 
/// Notes: Initial conditions are the position, velocity, and attitude of the platform used when the filter starts running or is reset.
/// For the user specified position array, the units are meters if the ECEF frame is selected, and degrees latitude, degrees longitude, and meters above ellipsoid if the latitude/longitude/height frame is selected.
/// For the user specified velocity array, the units are meters per second, but the reference frame depends on the reference frame selector (ECEF or NED).
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result load_mip_filter_initialization_configuration(struct mip_interface* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_LOAD);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_INITIALIZATION_CONFIGURATION, buffer, cmdUsed);
}

/// @brief Controls the source and values used for initial conditions of the navigation solution.
/// 
/// Notes: Initial conditions are the position, velocity, and attitude of the platform used when the filter starts running or is reset.
/// For the user specified position array, the units are meters if the ECEF frame is selected, and degrees latitude, degrees longitude, and meters above ellipsoid if the latitude/longitude/height frame is selected.
/// For the user specified velocity array, the units are meters per second, but the reference frame depends on the reference frame selector (ECEF or NED).
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result default_mip_filter_initialization_configuration(struct mip_interface* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_RESET);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_INITIALIZATION_CONFIGURATION, buffer, cmdUsed);
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_adaptive_filter_options_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_adaptive_filter_options_command* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->level);
    offset = insert_u16(buffer, bufferSize, offset, self->time_limit);
    
    return offset;
}

size_t extract_mip_filter_adaptive_filter_options_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_adaptive_filter_options_command* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->level);
    offset = extract_u16(buffer, bufferSize, offset, &self->time_limit);
    
    return offset;
}


size_t insert_mip_filter_adaptive_filter_options_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_adaptive_filter_options_response* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->level);
    offset = insert_u16(buffer, bufferSize, offset, self->time_limit);
    
    return offset;
}

size_t extract_mip_filter_adaptive_filter_options_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_adaptive_filter_options_response* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->level);
    offset = extract_u16(buffer, bufferSize, offset, &self->time_limit);
    
    return offset;
}


/// @brief Configures the basic setup for auto-adaptive filtering. See product manual for a detailed description of this feature.
/// 
/// @param level Auto-adaptive operating level: <br/> 0=Off, <br/> 1=Conservative, <br/> 2=Moderate (default), <br/> 3=Aggressive.
/// @param time_limit Maximum duration of measurement rejection before entering recovery mode    (ms)
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result write_mip_filter_adaptive_filter_options(struct mip_interface* device, uint8_t level, uint16_t time_limit)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_WRITE);
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, level);
    cmdUsed = insert_u16(buffer, sizeof(buffer), cmdUsed, time_limit);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ADAPTIVE_FILTER_OPTIONS, buffer, cmdUsed);
}

/// @brief Configures the basic setup for auto-adaptive filtering. See product manual for a detailed description of this feature.
/// 
/// @param[out] level Auto-adaptive operating level: <br/> 0=Off, <br/> 1=Conservative, <br/> 2=Moderate (default), <br/> 3=Aggressive.
/// @param[out] time_limit Maximum duration of measurement rejection before entering recovery mode    (ms)
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result read_mip_filter_adaptive_filter_options(struct mip_interface* device, uint8_t* level, uint16_t* time_limit)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_READ);
    assert(cmdUsed <= sizeof(buffer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ADAPTIVE_FILTER_OPTIONS, buffer, cmdUsed, MIP_REPLY_DESC_FILTER_ADAPTIVE_FILTER_OPTIONS, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        size_t responseUsed = 0;
        responseUsed = extract_u8(buffer, sizeof(buffer), responseUsed, level);
        responseUsed = extract_u16(buffer, sizeof(buffer), responseUsed, time_limit);
        
        if( responseUsed != responseLength )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Configures the basic setup for auto-adaptive filtering. See product manual for a detailed description of this feature.
/// 
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result save_mip_filter_adaptive_filter_options(struct mip_interface* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_SAVE);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ADAPTIVE_FILTER_OPTIONS, buffer, cmdUsed);
}

/// @brief Configures the basic setup for auto-adaptive filtering. See product manual for a detailed description of this feature.
/// 
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result load_mip_filter_adaptive_filter_options(struct mip_interface* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_LOAD);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ADAPTIVE_FILTER_OPTIONS, buffer, cmdUsed);
}

/// @brief Configures the basic setup for auto-adaptive filtering. See product manual for a detailed description of this feature.
/// 
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result default_mip_filter_adaptive_filter_options(struct mip_interface* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_RESET);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ADAPTIVE_FILTER_OPTIONS, buffer, cmdUsed);
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_multi_antenna_offset_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_multi_antenna_offset_command* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->receiver_id);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->antenna_offset[i]);
    
    return offset;
}

size_t extract_mip_filter_multi_antenna_offset_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_multi_antenna_offset_command* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->receiver_id);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->antenna_offset[i]);
    
    return offset;
}


size_t insert_mip_filter_multi_antenna_offset_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_multi_antenna_offset_response* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->receiver_id);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->antenna_offset[i]);
    
    return offset;
}

size_t extract_mip_filter_multi_antenna_offset_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_multi_antenna_offset_response* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->receiver_id);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->antenna_offset[i]);
    
    return offset;
}


/// @brief Set the antenna lever arm.
/// 
/// This command works with devices that utilize multiple antennas.
/// @param receiver_id Receiver: 1, 2, etc...
/// @param antenna_offset Antenna lever arm offset vector in the vehicle frame (m)
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result write_mip_filter_multi_antenna_offset(struct mip_interface* device, uint8_t receiver_id, const float* antenna_offset)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_WRITE);
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, receiver_id);
    for(unsigned int i=0; i < 3; i++)
        cmdUsed = insert_float(buffer, sizeof(buffer), cmdUsed, antenna_offset[i]);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_MULTI_ANTENNA_OFFSET, buffer, cmdUsed);
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
mip_cmd_result read_mip_filter_multi_antenna_offset(struct mip_interface* device, uint8_t receiver_id, float* antenna_offset)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_READ);
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, receiver_id);
    assert(cmdUsed <= sizeof(buffer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_MULTI_ANTENNA_OFFSET, buffer, cmdUsed, MIP_REPLY_DESC_FILTER_MULTI_ANTENNA_OFFSET, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        size_t responseUsed = 0;
        responseUsed = extract_u8(buffer, sizeof(buffer), responseUsed, &receiver_id);
        for(unsigned int i=0; i < 3; i++)
            responseUsed = extract_float(buffer, sizeof(buffer), responseUsed, &antenna_offset[i]);
        
        if( responseUsed != responseLength )
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
mip_cmd_result save_mip_filter_multi_antenna_offset(struct mip_interface* device, uint8_t receiver_id)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_SAVE);
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, receiver_id);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_MULTI_ANTENNA_OFFSET, buffer, cmdUsed);
}

/// @brief Set the antenna lever arm.
/// 
/// This command works with devices that utilize multiple antennas.
/// @param receiver_id Receiver: 1, 2, etc...
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result load_mip_filter_multi_antenna_offset(struct mip_interface* device, uint8_t receiver_id)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_LOAD);
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, receiver_id);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_MULTI_ANTENNA_OFFSET, buffer, cmdUsed);
}

/// @brief Set the antenna lever arm.
/// 
/// This command works with devices that utilize multiple antennas.
/// @param receiver_id Receiver: 1, 2, etc...
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result default_mip_filter_multi_antenna_offset(struct mip_interface* device, uint8_t receiver_id)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_RESET);
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, receiver_id);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_MULTI_ANTENNA_OFFSET, buffer, cmdUsed);
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_rel_pos_configuration_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_rel_pos_configuration_command* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->source);
    offset = insert_mip_filter_reference_frame(buffer, bufferSize, offset, self->reference_frame_selector);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_double(buffer, bufferSize, offset, self->reference_coordinates[i]);
    
    return offset;
}

size_t extract_mip_filter_rel_pos_configuration_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_rel_pos_configuration_command* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->source);
    offset = extract_mip_filter_reference_frame(buffer, bufferSize, offset, &self->reference_frame_selector);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_double(buffer, bufferSize, offset, &self->reference_coordinates[i]);
    
    return offset;
}


size_t insert_mip_filter_rel_pos_configuration_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_rel_pos_configuration_response* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->source);
    offset = insert_mip_filter_reference_frame(buffer, bufferSize, offset, self->reference_frame_selector);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_double(buffer, bufferSize, offset, self->reference_coordinates[i]);
    
    return offset;
}

size_t extract_mip_filter_rel_pos_configuration_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_rel_pos_configuration_response* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->source);
    offset = extract_mip_filter_reference_frame(buffer, bufferSize, offset, &self->reference_frame_selector);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_double(buffer, bufferSize, offset, &self->reference_coordinates[i]);
    
    return offset;
}


/// @brief Configure the reference location for filter relative positioning outputs
/// 
/// @param source 0 - auto (RTK base station), 1 - manual
/// @param reference_frame_selector ECEF or LLH
/// @param reference_coordinates reference coordinates, units determined by source selection
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result write_mip_filter_rel_pos_configuration(struct mip_interface* device, uint8_t source, enum mip_filter_reference_frame reference_frame_selector, const double* reference_coordinates)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_WRITE);
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, source);
    cmdUsed = insert_mip_filter_reference_frame(buffer, sizeof(buffer), cmdUsed, reference_frame_selector);
    for(unsigned int i=0; i < 3; i++)
        cmdUsed = insert_double(buffer, sizeof(buffer), cmdUsed, reference_coordinates[i]);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_REL_POS_CONFIGURATION, buffer, cmdUsed);
}

/// @brief Configure the reference location for filter relative positioning outputs
/// 
/// @param[out] source 0 - auto (RTK base station), 1 - manual
/// @param[out] reference_frame_selector ECEF or LLH
/// @param[out] reference_coordinates reference coordinates, units determined by source selection
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result read_mip_filter_rel_pos_configuration(struct mip_interface* device, uint8_t* source, enum mip_filter_reference_frame* reference_frame_selector, double* reference_coordinates)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_READ);
    assert(cmdUsed <= sizeof(buffer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_REL_POS_CONFIGURATION, buffer, cmdUsed, MIP_REPLY_DESC_FILTER_REL_POS_CONFIGURATION, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        size_t responseUsed = 0;
        responseUsed = extract_u8(buffer, sizeof(buffer), responseUsed, source);
        responseUsed = extract_mip_filter_reference_frame(buffer, sizeof(buffer), responseUsed, reference_frame_selector);
        for(unsigned int i=0; i < 3; i++)
            responseUsed = extract_double(buffer, sizeof(buffer), responseUsed, &reference_coordinates[i]);
        
        if( responseUsed != responseLength )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Configure the reference location for filter relative positioning outputs
/// 
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result save_mip_filter_rel_pos_configuration(struct mip_interface* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_SAVE);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_REL_POS_CONFIGURATION, buffer, cmdUsed);
}

/// @brief Configure the reference location for filter relative positioning outputs
/// 
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result load_mip_filter_rel_pos_configuration(struct mip_interface* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_LOAD);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_REL_POS_CONFIGURATION, buffer, cmdUsed);
}

/// @brief Configure the reference location for filter relative positioning outputs
/// 
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result default_mip_filter_rel_pos_configuration(struct mip_interface* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_RESET);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_REL_POS_CONFIGURATION, buffer, cmdUsed);
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_ref_point_lever_arm_command_reference_point_selector(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_filter_ref_point_lever_arm_command_reference_point_selector self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}
size_t extract_mip_filter_ref_point_lever_arm_command_reference_point_selector(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_filter_ref_point_lever_arm_command_reference_point_selector* self)
{
    uint8_t tmp;
    offset = extract_u8(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}

size_t insert_mip_filter_ref_point_lever_arm_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_ref_point_lever_arm_command* self)
{
    offset = insert_mip_filter_ref_point_lever_arm_command_reference_point_selector(buffer, bufferSize, offset, self->ref_point_sel);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->lever_arm_offset[i]);
    
    return offset;
}

size_t extract_mip_filter_ref_point_lever_arm_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_ref_point_lever_arm_command* self)
{
    offset = extract_mip_filter_ref_point_lever_arm_command_reference_point_selector(buffer, bufferSize, offset, &self->ref_point_sel);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->lever_arm_offset[i]);
    
    return offset;
}


size_t insert_mip_filter_ref_point_lever_arm_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_ref_point_lever_arm_response* self)
{
    offset = insert_mip_filter_ref_point_lever_arm_command_reference_point_selector(buffer, bufferSize, offset, self->ref_point_sel);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->lever_arm_offset[i]);
    
    return offset;
}

size_t extract_mip_filter_ref_point_lever_arm_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_ref_point_lever_arm_response* self)
{
    offset = extract_mip_filter_ref_point_lever_arm_command_reference_point_selector(buffer, bufferSize, offset, &self->ref_point_sel);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->lever_arm_offset[i]);
    
    return offset;
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
mip_cmd_result write_mip_filter_ref_point_lever_arm(struct mip_interface* device, enum mip_filter_ref_point_lever_arm_command_reference_point_selector ref_point_sel, const float* lever_arm_offset)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_WRITE);
    cmdUsed = insert_mip_filter_ref_point_lever_arm_command_reference_point_selector(buffer, sizeof(buffer), cmdUsed, ref_point_sel);
    for(unsigned int i=0; i < 3; i++)
        cmdUsed = insert_float(buffer, sizeof(buffer), cmdUsed, lever_arm_offset[i]);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_REF_POINT_LEVER_ARM, buffer, cmdUsed);
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
mip_cmd_result read_mip_filter_ref_point_lever_arm(struct mip_interface* device, enum mip_filter_ref_point_lever_arm_command_reference_point_selector* ref_point_sel, float* lever_arm_offset)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_READ);
    assert(cmdUsed <= sizeof(buffer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_REF_POINT_LEVER_ARM, buffer, cmdUsed, MIP_REPLY_DESC_FILTER_REF_POINT_LEVER_ARM, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        size_t responseUsed = 0;
        responseUsed = extract_mip_filter_ref_point_lever_arm_command_reference_point_selector(buffer, sizeof(buffer), responseUsed, ref_point_sel);
        for(unsigned int i=0; i < 3; i++)
            responseUsed = extract_float(buffer, sizeof(buffer), responseUsed, &lever_arm_offset[i]);
        
        if( responseUsed != responseLength )
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
mip_cmd_result save_mip_filter_ref_point_lever_arm(struct mip_interface* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_SAVE);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_REF_POINT_LEVER_ARM, buffer, cmdUsed);
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
mip_cmd_result load_mip_filter_ref_point_lever_arm(struct mip_interface* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_LOAD);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_REF_POINT_LEVER_ARM, buffer, cmdUsed);
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
mip_cmd_result default_mip_filter_ref_point_lever_arm(struct mip_interface* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_RESET);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_REF_POINT_LEVER_ARM, buffer, cmdUsed);
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_speed_measurement_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_speed_measurement_command* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->source);
    offset = insert_float(buffer, bufferSize, offset, self->time_of_week);
    offset = insert_float(buffer, bufferSize, offset, self->speed);
    offset = insert_float(buffer, bufferSize, offset, self->speed_uncertainty);
    
    return offset;
}

size_t extract_mip_filter_speed_measurement_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_speed_measurement_command* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->source);
    offset = extract_float(buffer, bufferSize, offset, &self->time_of_week);
    offset = extract_float(buffer, bufferSize, offset, &self->speed);
    offset = extract_float(buffer, bufferSize, offset, &self->speed_uncertainty);
    
    return offset;
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
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, source);
    cmdUsed = insert_float(buffer, sizeof(buffer), cmdUsed, time_of_week);
    cmdUsed = insert_float(buffer, sizeof(buffer), cmdUsed, speed);
    cmdUsed = insert_float(buffer, sizeof(buffer), cmdUsed, speed_uncertainty);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SPEED_MEASUREMENT, buffer, cmdUsed);
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_speed_lever_arm_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_speed_lever_arm_command* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->source);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->lever_arm_offset[i]);
    
    return offset;
}

size_t extract_mip_filter_speed_lever_arm_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_speed_lever_arm_command* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->source);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->lever_arm_offset[i]);
    
    return offset;
}


size_t insert_mip_filter_speed_lever_arm_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_speed_lever_arm_response* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->source);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->lever_arm_offset[i]);
    
    return offset;
}

size_t extract_mip_filter_speed_lever_arm_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_speed_lever_arm_response* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->source);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->lever_arm_offset[i]);
    
    return offset;
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
mip_cmd_result write_mip_filter_speed_lever_arm(struct mip_interface* device, uint8_t source, const float* lever_arm_offset)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_WRITE);
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, source);
    for(unsigned int i=0; i < 3; i++)
        cmdUsed = insert_float(buffer, sizeof(buffer), cmdUsed, lever_arm_offset[i]);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SPEED_LEVER_ARM, buffer, cmdUsed);
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
mip_cmd_result read_mip_filter_speed_lever_arm(struct mip_interface* device, uint8_t source, float* lever_arm_offset)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_READ);
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, source);
    assert(cmdUsed <= sizeof(buffer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SPEED_LEVER_ARM, buffer, cmdUsed, MIP_REPLY_DESC_FILTER_SPEED_LEVER_ARM, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        size_t responseUsed = 0;
        responseUsed = extract_u8(buffer, sizeof(buffer), responseUsed, &source);
        for(unsigned int i=0; i < 3; i++)
            responseUsed = extract_float(buffer, sizeof(buffer), responseUsed, &lever_arm_offset[i]);
        
        if( responseUsed != responseLength )
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
mip_cmd_result save_mip_filter_speed_lever_arm(struct mip_interface* device, uint8_t source)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_SAVE);
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, source);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SPEED_LEVER_ARM, buffer, cmdUsed);
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
mip_cmd_result load_mip_filter_speed_lever_arm(struct mip_interface* device, uint8_t source)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_LOAD);
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, source);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SPEED_LEVER_ARM, buffer, cmdUsed);
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
mip_cmd_result default_mip_filter_speed_lever_arm(struct mip_interface* device, uint8_t source)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_RESET);
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, source);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SPEED_LEVER_ARM, buffer, cmdUsed);
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_wheeled_vehicle_constraint_control_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_wheeled_vehicle_constraint_control_command* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->enable);
    
    return offset;
}

size_t extract_mip_filter_wheeled_vehicle_constraint_control_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_wheeled_vehicle_constraint_control_command* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->enable);
    
    return offset;
}


size_t insert_mip_filter_wheeled_vehicle_constraint_control_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_wheeled_vehicle_constraint_control_response* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->enable);
    
    return offset;
}

size_t extract_mip_filter_wheeled_vehicle_constraint_control_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_wheeled_vehicle_constraint_control_response* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->enable);
    
    return offset;
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
mip_cmd_result write_mip_filter_wheeled_vehicle_constraint_control(struct mip_interface* device, uint8_t enable)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_WRITE);
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, enable);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_WHEELED_VEHICLE_CONSTRAINT_CONTROL, buffer, cmdUsed);
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
mip_cmd_result read_mip_filter_wheeled_vehicle_constraint_control(struct mip_interface* device, uint8_t* enable)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_READ);
    assert(cmdUsed <= sizeof(buffer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_WHEELED_VEHICLE_CONSTRAINT_CONTROL, buffer, cmdUsed, MIP_REPLY_DESC_WHEELED_VEHICLE_CONSTRAINT_CONTROL, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        size_t responseUsed = 0;
        responseUsed = extract_u8(buffer, sizeof(buffer), responseUsed, enable);
        
        if( responseUsed != responseLength )
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
mip_cmd_result save_mip_filter_wheeled_vehicle_constraint_control(struct mip_interface* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_SAVE);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_WHEELED_VEHICLE_CONSTRAINT_CONTROL, buffer, cmdUsed);
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
mip_cmd_result load_mip_filter_wheeled_vehicle_constraint_control(struct mip_interface* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_LOAD);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_WHEELED_VEHICLE_CONSTRAINT_CONTROL, buffer, cmdUsed);
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
mip_cmd_result default_mip_filter_wheeled_vehicle_constraint_control(struct mip_interface* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_RESET);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_WHEELED_VEHICLE_CONSTRAINT_CONTROL, buffer, cmdUsed);
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_vertical_gyro_constraint_control_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_vertical_gyro_constraint_control_command* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->enable);
    
    return offset;
}

size_t extract_mip_filter_vertical_gyro_constraint_control_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_vertical_gyro_constraint_control_command* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->enable);
    
    return offset;
}


size_t insert_mip_filter_vertical_gyro_constraint_control_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_vertical_gyro_constraint_control_response* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->enable);
    
    return offset;
}

size_t extract_mip_filter_vertical_gyro_constraint_control_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_vertical_gyro_constraint_control_response* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->enable);
    
    return offset;
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
mip_cmd_result write_mip_filter_vertical_gyro_constraint_control(struct mip_interface* device, uint8_t enable)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_WRITE);
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, enable);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_VERTICAL_GYRO_CONSTRAINT_CONTROL, buffer, cmdUsed);
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
mip_cmd_result read_mip_filter_vertical_gyro_constraint_control(struct mip_interface* device, uint8_t* enable)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_READ);
    assert(cmdUsed <= sizeof(buffer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_VERTICAL_GYRO_CONSTRAINT_CONTROL, buffer, cmdUsed, MIP_REPLY_DESC_VERTICAL_GYRO_CONSTRAINT_CONTROL, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        size_t responseUsed = 0;
        responseUsed = extract_u8(buffer, sizeof(buffer), responseUsed, enable);
        
        if( responseUsed != responseLength )
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
mip_cmd_result save_mip_filter_vertical_gyro_constraint_control(struct mip_interface* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_SAVE);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_VERTICAL_GYRO_CONSTRAINT_CONTROL, buffer, cmdUsed);
}

/// @brief Configure the vertical gyro kinematic constraint.
/// 
/// When enabled and no valid GNSS measurements are available, the filter uses the accelerometers to track pitch
/// and roll under the assumption that the sensor platform is not undergoing linear acceleration.
/// This constraint is useful to maintain accurate pitch and roll during GNSS signal outages.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result load_mip_filter_vertical_gyro_constraint_control(struct mip_interface* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_LOAD);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_VERTICAL_GYRO_CONSTRAINT_CONTROL, buffer, cmdUsed);
}

/// @brief Configure the vertical gyro kinematic constraint.
/// 
/// When enabled and no valid GNSS measurements are available, the filter uses the accelerometers to track pitch
/// and roll under the assumption that the sensor platform is not undergoing linear acceleration.
/// This constraint is useful to maintain accurate pitch and roll during GNSS signal outages.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result default_mip_filter_vertical_gyro_constraint_control(struct mip_interface* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_RESET);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_VERTICAL_GYRO_CONSTRAINT_CONTROL, buffer, cmdUsed);
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_gnss_antenna_cal_control_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_gnss_antenna_cal_control_command* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->enable);
    offset = insert_float(buffer, bufferSize, offset, self->max_offset);
    
    return offset;
}

size_t extract_mip_filter_gnss_antenna_cal_control_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_gnss_antenna_cal_control_command* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->enable);
    offset = extract_float(buffer, bufferSize, offset, &self->max_offset);
    
    return offset;
}


size_t insert_mip_filter_gnss_antenna_cal_control_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_gnss_antenna_cal_control_response* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->enable);
    offset = insert_float(buffer, bufferSize, offset, self->max_offset);
    
    return offset;
}

size_t extract_mip_filter_gnss_antenna_cal_control_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_gnss_antenna_cal_control_response* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->enable);
    offset = extract_float(buffer, bufferSize, offset, &self->max_offset);
    
    return offset;
}


/// @brief Configure the GNSS antenna lever arm calibration.
/// 
/// When enabled, the filter will enable lever arm error tracking, up to the maximum offset specified.
/// @param enable 0 - Disable, 1 - Enable
/// @param max_offset Maximum absolute value of lever arm offset error in the vehicle frame [meters]. See device user manual for the valid range of this parameter.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result write_mip_filter_gnss_antenna_cal_control(struct mip_interface* device, uint8_t enable, float max_offset)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_WRITE);
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, enable);
    cmdUsed = insert_float(buffer, sizeof(buffer), cmdUsed, max_offset);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_GNSS_ANTENNA_CALIBRATION_CONTROL, buffer, cmdUsed);
}

/// @brief Configure the GNSS antenna lever arm calibration.
/// 
/// When enabled, the filter will enable lever arm error tracking, up to the maximum offset specified.
/// @param[out] enable 0 - Disable, 1 - Enable
/// @param[out] max_offset Maximum absolute value of lever arm offset error in the vehicle frame [meters]. See device user manual for the valid range of this parameter.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result read_mip_filter_gnss_antenna_cal_control(struct mip_interface* device, uint8_t* enable, float* max_offset)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_READ);
    assert(cmdUsed <= sizeof(buffer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_GNSS_ANTENNA_CALIBRATION_CONTROL, buffer, cmdUsed, MIP_REPLY_DESC_GNSS_ANTENNA_CALIBRATION_CONTROL, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        size_t responseUsed = 0;
        responseUsed = extract_u8(buffer, sizeof(buffer), responseUsed, enable);
        responseUsed = extract_float(buffer, sizeof(buffer), responseUsed, max_offset);
        
        if( responseUsed != responseLength )
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
mip_cmd_result save_mip_filter_gnss_antenna_cal_control(struct mip_interface* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_SAVE);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_GNSS_ANTENNA_CALIBRATION_CONTROL, buffer, cmdUsed);
}

/// @brief Configure the GNSS antenna lever arm calibration.
/// 
/// When enabled, the filter will enable lever arm error tracking, up to the maximum offset specified.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result load_mip_filter_gnss_antenna_cal_control(struct mip_interface* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_LOAD);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_GNSS_ANTENNA_CALIBRATION_CONTROL, buffer, cmdUsed);
}

/// @brief Configure the GNSS antenna lever arm calibration.
/// 
/// When enabled, the filter will enable lever arm error tracking, up to the maximum offset specified.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result default_mip_filter_gnss_antenna_cal_control(struct mip_interface* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_RESET);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_GNSS_ANTENNA_CALIBRATION_CONTROL, buffer, cmdUsed);
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_magnetic_declination_source_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_magnetic_declination_source_command* self)
{
    offset = insert_mip_filter_mag_declination_source(buffer, bufferSize, offset, self->source);
    offset = insert_float(buffer, bufferSize, offset, self->declination);
    
    return offset;
}

size_t extract_mip_filter_magnetic_declination_source_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_magnetic_declination_source_command* self)
{
    offset = extract_mip_filter_mag_declination_source(buffer, bufferSize, offset, &self->source);
    offset = extract_float(buffer, bufferSize, offset, &self->declination);
    
    return offset;
}


size_t insert_mip_filter_magnetic_declination_source_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_magnetic_declination_source_response* self)
{
    offset = insert_mip_filter_mag_declination_source(buffer, bufferSize, offset, self->source);
    offset = insert_float(buffer, bufferSize, offset, self->declination);
    
    return offset;
}

size_t extract_mip_filter_magnetic_declination_source_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_magnetic_declination_source_response* self)
{
    offset = extract_mip_filter_mag_declination_source(buffer, bufferSize, offset, &self->source);
    offset = extract_float(buffer, bufferSize, offset, &self->declination);
    
    return offset;
}


/// @brief Source for magnetic declination angle, and user supplied value for manual selection.
/// 
/// @param source Magnetic field declination angle source
/// @param declination Declination angle used when 'source' is set to 'MANUAL' (radians)
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result write_mip_filter_magnetic_declination_source(struct mip_interface* device, enum mip_filter_mag_declination_source source, float declination)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_WRITE);
    cmdUsed = insert_mip_filter_mag_declination_source(buffer, sizeof(buffer), cmdUsed, source);
    cmdUsed = insert_float(buffer, sizeof(buffer), cmdUsed, declination);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_DECLINATION_SOURCE, buffer, cmdUsed);
}

/// @brief Source for magnetic declination angle, and user supplied value for manual selection.
/// 
/// @param[out] source Magnetic field declination angle source
/// @param[out] declination Declination angle used when 'source' is set to 'MANUAL' (radians)
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result read_mip_filter_magnetic_declination_source(struct mip_interface* device, enum mip_filter_mag_declination_source* source, float* declination)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_READ);
    assert(cmdUsed <= sizeof(buffer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_DECLINATION_SOURCE, buffer, cmdUsed, MIP_REPLY_DESC_FILTER_DECLINATION_SOURCE, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        size_t responseUsed = 0;
        responseUsed = extract_mip_filter_mag_declination_source(buffer, sizeof(buffer), responseUsed, source);
        responseUsed = extract_float(buffer, sizeof(buffer), responseUsed, declination);
        
        if( responseUsed != responseLength )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Source for magnetic declination angle, and user supplied value for manual selection.
/// 
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result save_mip_filter_magnetic_declination_source(struct mip_interface* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_SAVE);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_DECLINATION_SOURCE, buffer, cmdUsed);
}

/// @brief Source for magnetic declination angle, and user supplied value for manual selection.
/// 
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result load_mip_filter_magnetic_declination_source(struct mip_interface* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_LOAD);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_DECLINATION_SOURCE, buffer, cmdUsed);
}

/// @brief Source for magnetic declination angle, and user supplied value for manual selection.
/// 
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result default_mip_filter_magnetic_declination_source(struct mip_interface* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_RESET);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_DECLINATION_SOURCE, buffer, cmdUsed);
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_set_initial_heading_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_set_initial_heading_command* self)
{
    offset = insert_float(buffer, bufferSize, offset, self->heading);
    
    return offset;
}

size_t extract_mip_filter_set_initial_heading_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_set_initial_heading_command* self)
{
    offset = extract_float(buffer, bufferSize, offset, &self->heading);
    
    return offset;
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
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_float(buffer, sizeof(buffer), cmdUsed, heading);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SET_INITIAL_HEADING, buffer, cmdUsed);
}


#ifdef __cplusplus
} // extern "C"
} // namespace mscl
#endif // __cplusplus
