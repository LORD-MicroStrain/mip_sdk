
#include "commands_filter.h"

#include <mip/mip_serialization.h>
#include <mip/mip_interface.h>

#include <assert.h>


#ifdef __cplusplus
namespace mip {
namespace C {
extern "C" {

#endif // __cplusplus


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

mip_cmd_result mip_filter_reset(mip_interface* device)
{
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_RESET_FILTER, NULL, 0);
}
void insert_mip_filter_set_initial_attitude_command(microstrain_serializer* serializer, const mip_filter_set_initial_attitude_command* self)
{
    microstrain_insert_float(serializer, self->roll);
    
    microstrain_insert_float(serializer, self->pitch);
    
    microstrain_insert_float(serializer, self->heading);
    
}
void extract_mip_filter_set_initial_attitude_command(microstrain_serializer* serializer, mip_filter_set_initial_attitude_command* self)
{
    microstrain_extract_float(serializer, &self->roll);
    
    microstrain_extract_float(serializer, &self->pitch);
    
    microstrain_extract_float(serializer, &self->heading);
    
}

mip_cmd_result mip_filter_set_initial_attitude(mip_interface* device, float roll, float pitch, float heading)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    microstrain_insert_float(&serializer, roll);
    
    microstrain_insert_float(&serializer, pitch);
    
    microstrain_insert_float(&serializer, heading);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SET_INITIAL_ATTITUDE, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
void insert_mip_filter_estimation_control_command(microstrain_serializer* serializer, const mip_filter_estimation_control_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        insert_mip_filter_estimation_control_command_enable_flags(serializer, self->enable);
        
    }
}
void extract_mip_filter_estimation_control_command(microstrain_serializer* serializer, mip_filter_estimation_control_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        extract_mip_filter_estimation_control_command_enable_flags(serializer, &self->enable);
        
    }
}

void insert_mip_filter_estimation_control_response(microstrain_serializer* serializer, const mip_filter_estimation_control_response* self)
{
    insert_mip_filter_estimation_control_command_enable_flags(serializer, self->enable);
    
}
void extract_mip_filter_estimation_control_response(microstrain_serializer* serializer, mip_filter_estimation_control_response* self)
{
    extract_mip_filter_estimation_control_command_enable_flags(serializer, &self->enable);
    
}

mip_cmd_result mip_filter_write_estimation_control(mip_interface* device, mip_filter_estimation_control_command_enable_flags enable)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    insert_mip_filter_estimation_control_command_enable_flags(&serializer, enable);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ESTIMATION_CONTROL_FLAGS, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_read_estimation_control(mip_interface* device, mip_filter_estimation_control_command_enable_flags* enable_out)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ESTIMATION_CONTROL_FLAGS, buffer, (uint8_t)microstrain_serializer_length(&serializer), MIP_REPLY_DESC_FILTER_ESTIMATION_CONTROL_FLAGS, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        microstrain_serializer deserializer;
        microstrain_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(enable_out);
        extract_mip_filter_estimation_control_command_enable_flags(&deserializer, enable_out);
        
        if( microstrain_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_filter_save_estimation_control(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ESTIMATION_CONTROL_FLAGS, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_load_estimation_control(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ESTIMATION_CONTROL_FLAGS, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_default_estimation_control(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ESTIMATION_CONTROL_FLAGS, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
void insert_mip_filter_external_gnss_update_command(microstrain_serializer* serializer, const mip_filter_external_gnss_update_command* self)
{
    microstrain_insert_double(serializer, self->gps_time);
    
    microstrain_insert_u16(serializer, self->gps_week);
    
    microstrain_insert_double(serializer, self->latitude);
    
    microstrain_insert_double(serializer, self->longitude);
    
    microstrain_insert_double(serializer, self->height);
    
    insert_mip_vector3f(serializer, self->velocity);
    
    insert_mip_vector3f(serializer, self->pos_uncertainty);
    
    insert_mip_vector3f(serializer, self->vel_uncertainty);
    
}
void extract_mip_filter_external_gnss_update_command(microstrain_serializer* serializer, mip_filter_external_gnss_update_command* self)
{
    microstrain_extract_double(serializer, &self->gps_time);
    
    microstrain_extract_u16(serializer, &self->gps_week);
    
    microstrain_extract_double(serializer, &self->latitude);
    
    microstrain_extract_double(serializer, &self->longitude);
    
    microstrain_extract_double(serializer, &self->height);
    
    extract_mip_vector3f(serializer, self->velocity);
    
    extract_mip_vector3f(serializer, self->pos_uncertainty);
    
    extract_mip_vector3f(serializer, self->vel_uncertainty);
    
}

mip_cmd_result mip_filter_external_gnss_update(mip_interface* device, double gps_time, uint16_t gps_week, double latitude, double longitude, double height, const float* velocity, const float* pos_uncertainty, const float* vel_uncertainty)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    microstrain_insert_double(&serializer, gps_time);
    
    microstrain_insert_u16(&serializer, gps_week);
    
    microstrain_insert_double(&serializer, latitude);
    
    microstrain_insert_double(&serializer, longitude);
    
    microstrain_insert_double(&serializer, height);
    
    assert(velocity);
    for(unsigned int i=0; i < 3; i++)
        microstrain_insert_float(&serializer, velocity[i]);
    
    assert(pos_uncertainty);
    for(unsigned int i=0; i < 3; i++)
        microstrain_insert_float(&serializer, pos_uncertainty[i]);
    
    assert(vel_uncertainty);
    for(unsigned int i=0; i < 3; i++)
        microstrain_insert_float(&serializer, vel_uncertainty[i]);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_EXTERNAL_GNSS_UPDATE, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
void insert_mip_filter_external_heading_update_command(microstrain_serializer* serializer, const mip_filter_external_heading_update_command* self)
{
    microstrain_insert_float(serializer, self->heading);
    
    microstrain_insert_float(serializer, self->heading_uncertainty);
    
    microstrain_insert_u8(serializer, self->type);
    
}
void extract_mip_filter_external_heading_update_command(microstrain_serializer* serializer, mip_filter_external_heading_update_command* self)
{
    microstrain_extract_float(serializer, &self->heading);
    
    microstrain_extract_float(serializer, &self->heading_uncertainty);
    
    microstrain_extract_u8(serializer, &self->type);
    
}

mip_cmd_result mip_filter_external_heading_update(mip_interface* device, float heading, float heading_uncertainty, uint8_t type)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    microstrain_insert_float(&serializer, heading);
    
    microstrain_insert_float(&serializer, heading_uncertainty);
    
    microstrain_insert_u8(&serializer, type);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_EXTERNAL_HEADING_UPDATE, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
void insert_mip_filter_external_heading_update_with_time_command(microstrain_serializer* serializer, const mip_filter_external_heading_update_with_time_command* self)
{
    microstrain_insert_double(serializer, self->gps_time);
    
    microstrain_insert_u16(serializer, self->gps_week);
    
    microstrain_insert_float(serializer, self->heading);
    
    microstrain_insert_float(serializer, self->heading_uncertainty);
    
    microstrain_insert_u8(serializer, self->type);
    
}
void extract_mip_filter_external_heading_update_with_time_command(microstrain_serializer* serializer, mip_filter_external_heading_update_with_time_command* self)
{
    microstrain_extract_double(serializer, &self->gps_time);
    
    microstrain_extract_u16(serializer, &self->gps_week);
    
    microstrain_extract_float(serializer, &self->heading);
    
    microstrain_extract_float(serializer, &self->heading_uncertainty);
    
    microstrain_extract_u8(serializer, &self->type);
    
}

mip_cmd_result mip_filter_external_heading_update_with_time(mip_interface* device, double gps_time, uint16_t gps_week, float heading, float heading_uncertainty, uint8_t type)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    microstrain_insert_double(&serializer, gps_time);
    
    microstrain_insert_u16(&serializer, gps_week);
    
    microstrain_insert_float(&serializer, heading);
    
    microstrain_insert_float(&serializer, heading_uncertainty);
    
    microstrain_insert_u8(&serializer, type);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_EXTERNAL_HEADING_UPDATE_WITH_TIME, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
void insert_mip_filter_tare_orientation_command(microstrain_serializer* serializer, const mip_filter_tare_orientation_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        insert_mip_filter_tare_orientation_command_mip_tare_axes(serializer, self->axes);
        
    }
}
void extract_mip_filter_tare_orientation_command(microstrain_serializer* serializer, mip_filter_tare_orientation_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        extract_mip_filter_tare_orientation_command_mip_tare_axes(serializer, &self->axes);
        
    }
}

void insert_mip_filter_tare_orientation_response(microstrain_serializer* serializer, const mip_filter_tare_orientation_response* self)
{
    insert_mip_filter_tare_orientation_command_mip_tare_axes(serializer, self->axes);
    
}
void extract_mip_filter_tare_orientation_response(microstrain_serializer* serializer, mip_filter_tare_orientation_response* self)
{
    extract_mip_filter_tare_orientation_command_mip_tare_axes(serializer, &self->axes);
    
}

mip_cmd_result mip_filter_write_tare_orientation(mip_interface* device, mip_filter_tare_orientation_command_mip_tare_axes axes)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    insert_mip_filter_tare_orientation_command_mip_tare_axes(&serializer, axes);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_TARE_ORIENTATION, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_read_tare_orientation(mip_interface* device, mip_filter_tare_orientation_command_mip_tare_axes* axes_out)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_TARE_ORIENTATION, buffer, (uint8_t)microstrain_serializer_length(&serializer), MIP_REPLY_DESC_FILTER_TARE_ORIENTATION, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        microstrain_serializer deserializer;
        microstrain_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(axes_out);
        extract_mip_filter_tare_orientation_command_mip_tare_axes(&deserializer, axes_out);
        
        if( microstrain_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_filter_save_tare_orientation(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_TARE_ORIENTATION, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_load_tare_orientation(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_TARE_ORIENTATION, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_default_tare_orientation(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_TARE_ORIENTATION, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
void insert_mip_filter_vehicle_dynamics_mode_command(microstrain_serializer* serializer, const mip_filter_vehicle_dynamics_mode_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        insert_mip_filter_vehicle_dynamics_mode_command_dynamics_mode(serializer, self->mode);
        
    }
}
void extract_mip_filter_vehicle_dynamics_mode_command(microstrain_serializer* serializer, mip_filter_vehicle_dynamics_mode_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        extract_mip_filter_vehicle_dynamics_mode_command_dynamics_mode(serializer, &self->mode);
        
    }
}

void insert_mip_filter_vehicle_dynamics_mode_response(microstrain_serializer* serializer, const mip_filter_vehicle_dynamics_mode_response* self)
{
    insert_mip_filter_vehicle_dynamics_mode_command_dynamics_mode(serializer, self->mode);
    
}
void extract_mip_filter_vehicle_dynamics_mode_response(microstrain_serializer* serializer, mip_filter_vehicle_dynamics_mode_response* self)
{
    extract_mip_filter_vehicle_dynamics_mode_command_dynamics_mode(serializer, &self->mode);
    
}

mip_cmd_result mip_filter_write_vehicle_dynamics_mode(mip_interface* device, mip_filter_vehicle_dynamics_mode_command_dynamics_mode mode)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    insert_mip_filter_vehicle_dynamics_mode_command_dynamics_mode(&serializer, mode);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_VEHICLE_DYNAMICS_MODE, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_read_vehicle_dynamics_mode(mip_interface* device, mip_filter_vehicle_dynamics_mode_command_dynamics_mode* mode_out)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_VEHICLE_DYNAMICS_MODE, buffer, (uint8_t)microstrain_serializer_length(&serializer), MIP_REPLY_DESC_FILTER_VEHICLE_DYNAMICS_MODE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        microstrain_serializer deserializer;
        microstrain_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(mode_out);
        extract_mip_filter_vehicle_dynamics_mode_command_dynamics_mode(&deserializer, mode_out);
        
        if( microstrain_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_filter_save_vehicle_dynamics_mode(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_VEHICLE_DYNAMICS_MODE, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_load_vehicle_dynamics_mode(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_VEHICLE_DYNAMICS_MODE, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_default_vehicle_dynamics_mode(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_VEHICLE_DYNAMICS_MODE, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
void insert_mip_filter_sensor_to_vehicle_rotation_euler_command(microstrain_serializer* serializer, const mip_filter_sensor_to_vehicle_rotation_euler_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        microstrain_insert_float(serializer, self->roll);
        
        microstrain_insert_float(serializer, self->pitch);
        
        microstrain_insert_float(serializer, self->yaw);
        
    }
}
void extract_mip_filter_sensor_to_vehicle_rotation_euler_command(microstrain_serializer* serializer, mip_filter_sensor_to_vehicle_rotation_euler_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        microstrain_extract_float(serializer, &self->roll);
        
        microstrain_extract_float(serializer, &self->pitch);
        
        microstrain_extract_float(serializer, &self->yaw);
        
    }
}

void insert_mip_filter_sensor_to_vehicle_rotation_euler_response(microstrain_serializer* serializer, const mip_filter_sensor_to_vehicle_rotation_euler_response* self)
{
    microstrain_insert_float(serializer, self->roll);
    
    microstrain_insert_float(serializer, self->pitch);
    
    microstrain_insert_float(serializer, self->yaw);
    
}
void extract_mip_filter_sensor_to_vehicle_rotation_euler_response(microstrain_serializer* serializer, mip_filter_sensor_to_vehicle_rotation_euler_response* self)
{
    microstrain_extract_float(serializer, &self->roll);
    
    microstrain_extract_float(serializer, &self->pitch);
    
    microstrain_extract_float(serializer, &self->yaw);
    
}

mip_cmd_result mip_filter_write_sensor_to_vehicle_rotation_euler(mip_interface* device, float roll, float pitch, float yaw)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    microstrain_insert_float(&serializer, roll);
    
    microstrain_insert_float(&serializer, pitch);
    
    microstrain_insert_float(&serializer, yaw);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_ROTATION_EULER, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_read_sensor_to_vehicle_rotation_euler(mip_interface* device, float* roll_out, float* pitch_out, float* yaw_out)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_ROTATION_EULER, buffer, (uint8_t)microstrain_serializer_length(&serializer), MIP_REPLY_DESC_FILTER_SENSOR2VEHICLE_ROTATION_EULER, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        microstrain_serializer deserializer;
        microstrain_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(roll_out);
        microstrain_extract_float(&deserializer, roll_out);
        
        assert(pitch_out);
        microstrain_extract_float(&deserializer, pitch_out);
        
        assert(yaw_out);
        microstrain_extract_float(&deserializer, yaw_out);
        
        if( microstrain_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_filter_save_sensor_to_vehicle_rotation_euler(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_ROTATION_EULER, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_load_sensor_to_vehicle_rotation_euler(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_ROTATION_EULER, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_default_sensor_to_vehicle_rotation_euler(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_ROTATION_EULER, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
void insert_mip_filter_sensor_to_vehicle_rotation_dcm_command(microstrain_serializer* serializer, const mip_filter_sensor_to_vehicle_rotation_dcm_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        insert_mip_matrix3f(serializer, self->dcm);
        
    }
}
void extract_mip_filter_sensor_to_vehicle_rotation_dcm_command(microstrain_serializer* serializer, mip_filter_sensor_to_vehicle_rotation_dcm_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        extract_mip_matrix3f(serializer, self->dcm);
        
    }
}

void insert_mip_filter_sensor_to_vehicle_rotation_dcm_response(microstrain_serializer* serializer, const mip_filter_sensor_to_vehicle_rotation_dcm_response* self)
{
    insert_mip_matrix3f(serializer, self->dcm);
    
}
void extract_mip_filter_sensor_to_vehicle_rotation_dcm_response(microstrain_serializer* serializer, mip_filter_sensor_to_vehicle_rotation_dcm_response* self)
{
    extract_mip_matrix3f(serializer, self->dcm);
    
}

mip_cmd_result mip_filter_write_sensor_to_vehicle_rotation_dcm(mip_interface* device, const float* dcm)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    assert(dcm);
    for(unsigned int i=0; i < 9; i++)
        microstrain_insert_float(&serializer, dcm[i]);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_ROTATION_DCM, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_read_sensor_to_vehicle_rotation_dcm(mip_interface* device, float* dcm_out)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_ROTATION_DCM, buffer, (uint8_t)microstrain_serializer_length(&serializer), MIP_REPLY_DESC_FILTER_SENSOR2VEHICLE_ROTATION_DCM, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        microstrain_serializer deserializer;
        microstrain_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(dcm_out);
        for(unsigned int i=0; i < 9; i++)
            microstrain_extract_float(&deserializer, &dcm_out[i]);
        
        if( microstrain_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_filter_save_sensor_to_vehicle_rotation_dcm(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_ROTATION_DCM, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_load_sensor_to_vehicle_rotation_dcm(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_ROTATION_DCM, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_default_sensor_to_vehicle_rotation_dcm(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_ROTATION_DCM, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
void insert_mip_filter_sensor_to_vehicle_rotation_quaternion_command(microstrain_serializer* serializer, const mip_filter_sensor_to_vehicle_rotation_quaternion_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        insert_mip_quatf(serializer, self->quat);
        
    }
}
void extract_mip_filter_sensor_to_vehicle_rotation_quaternion_command(microstrain_serializer* serializer, mip_filter_sensor_to_vehicle_rotation_quaternion_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        extract_mip_quatf(serializer, self->quat);
        
    }
}

void insert_mip_filter_sensor_to_vehicle_rotation_quaternion_response(microstrain_serializer* serializer, const mip_filter_sensor_to_vehicle_rotation_quaternion_response* self)
{
    insert_mip_quatf(serializer, self->quat);
    
}
void extract_mip_filter_sensor_to_vehicle_rotation_quaternion_response(microstrain_serializer* serializer, mip_filter_sensor_to_vehicle_rotation_quaternion_response* self)
{
    extract_mip_quatf(serializer, self->quat);
    
}

mip_cmd_result mip_filter_write_sensor_to_vehicle_rotation_quaternion(mip_interface* device, const float* quat)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    assert(quat);
    for(unsigned int i=0; i < 4; i++)
        microstrain_insert_float(&serializer, quat[i]);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_ROTATION_QUATERNION, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_read_sensor_to_vehicle_rotation_quaternion(mip_interface* device, float* quat_out)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_ROTATION_QUATERNION, buffer, (uint8_t)microstrain_serializer_length(&serializer), MIP_REPLY_DESC_FILTER_SENSOR2VEHICLE_ROTATION_QUATERNION, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        microstrain_serializer deserializer;
        microstrain_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(quat_out);
        for(unsigned int i=0; i < 4; i++)
            microstrain_extract_float(&deserializer, &quat_out[i]);
        
        if( microstrain_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_filter_save_sensor_to_vehicle_rotation_quaternion(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_ROTATION_QUATERNION, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_load_sensor_to_vehicle_rotation_quaternion(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_ROTATION_QUATERNION, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_default_sensor_to_vehicle_rotation_quaternion(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_ROTATION_QUATERNION, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
void insert_mip_filter_sensor_to_vehicle_offset_command(microstrain_serializer* serializer, const mip_filter_sensor_to_vehicle_offset_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        insert_mip_vector3f(serializer, self->offset);
        
    }
}
void extract_mip_filter_sensor_to_vehicle_offset_command(microstrain_serializer* serializer, mip_filter_sensor_to_vehicle_offset_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        extract_mip_vector3f(serializer, self->offset);
        
    }
}

void insert_mip_filter_sensor_to_vehicle_offset_response(microstrain_serializer* serializer, const mip_filter_sensor_to_vehicle_offset_response* self)
{
    insert_mip_vector3f(serializer, self->offset);
    
}
void extract_mip_filter_sensor_to_vehicle_offset_response(microstrain_serializer* serializer, mip_filter_sensor_to_vehicle_offset_response* self)
{
    extract_mip_vector3f(serializer, self->offset);
    
}

mip_cmd_result mip_filter_write_sensor_to_vehicle_offset(mip_interface* device, const float* offset)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    assert(offset);
    for(unsigned int i=0; i < 3; i++)
        microstrain_insert_float(&serializer, offset[i]);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_OFFSET, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_read_sensor_to_vehicle_offset(mip_interface* device, float* offset_out)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_OFFSET, buffer, (uint8_t)microstrain_serializer_length(&serializer), MIP_REPLY_DESC_FILTER_SENSOR2VEHICLE_OFFSET, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        microstrain_serializer deserializer;
        microstrain_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(offset_out);
        for(unsigned int i=0; i < 3; i++)
            microstrain_extract_float(&deserializer, &offset_out[i]);
        
        if( microstrain_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_filter_save_sensor_to_vehicle_offset(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_OFFSET, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_load_sensor_to_vehicle_offset(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_OFFSET, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_default_sensor_to_vehicle_offset(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_OFFSET, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
void insert_mip_filter_antenna_offset_command(microstrain_serializer* serializer, const mip_filter_antenna_offset_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        insert_mip_vector3f(serializer, self->offset);
        
    }
}
void extract_mip_filter_antenna_offset_command(microstrain_serializer* serializer, mip_filter_antenna_offset_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        extract_mip_vector3f(serializer, self->offset);
        
    }
}

void insert_mip_filter_antenna_offset_response(microstrain_serializer* serializer, const mip_filter_antenna_offset_response* self)
{
    insert_mip_vector3f(serializer, self->offset);
    
}
void extract_mip_filter_antenna_offset_response(microstrain_serializer* serializer, mip_filter_antenna_offset_response* self)
{
    extract_mip_vector3f(serializer, self->offset);
    
}

mip_cmd_result mip_filter_write_antenna_offset(mip_interface* device, const float* offset)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    assert(offset);
    for(unsigned int i=0; i < 3; i++)
        microstrain_insert_float(&serializer, offset[i]);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ANTENNA_OFFSET, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_read_antenna_offset(mip_interface* device, float* offset_out)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ANTENNA_OFFSET, buffer, (uint8_t)microstrain_serializer_length(&serializer), MIP_REPLY_DESC_FILTER_ANTENNA_OFFSET, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        microstrain_serializer deserializer;
        microstrain_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(offset_out);
        for(unsigned int i=0; i < 3; i++)
            microstrain_extract_float(&deserializer, &offset_out[i]);
        
        if( microstrain_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_filter_save_antenna_offset(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ANTENNA_OFFSET, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_load_antenna_offset(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ANTENNA_OFFSET, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_default_antenna_offset(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ANTENNA_OFFSET, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
void insert_mip_filter_gnss_source_command(microstrain_serializer* serializer, const mip_filter_gnss_source_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        insert_mip_filter_gnss_source_command_source(serializer, self->source);
        
    }
}
void extract_mip_filter_gnss_source_command(microstrain_serializer* serializer, mip_filter_gnss_source_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        extract_mip_filter_gnss_source_command_source(serializer, &self->source);
        
    }
}

void insert_mip_filter_gnss_source_response(microstrain_serializer* serializer, const mip_filter_gnss_source_response* self)
{
    insert_mip_filter_gnss_source_command_source(serializer, self->source);
    
}
void extract_mip_filter_gnss_source_response(microstrain_serializer* serializer, mip_filter_gnss_source_response* self)
{
    extract_mip_filter_gnss_source_command_source(serializer, &self->source);
    
}

mip_cmd_result mip_filter_write_gnss_source(mip_interface* device, mip_filter_gnss_source_command_source source)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    insert_mip_filter_gnss_source_command_source(&serializer, source);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_GNSS_SOURCE_CONTROL, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_read_gnss_source(mip_interface* device, mip_filter_gnss_source_command_source* source_out)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_GNSS_SOURCE_CONTROL, buffer, (uint8_t)microstrain_serializer_length(&serializer), MIP_REPLY_DESC_FILTER_GNSS_SOURCE_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        microstrain_serializer deserializer;
        microstrain_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(source_out);
        extract_mip_filter_gnss_source_command_source(&deserializer, source_out);
        
        if( microstrain_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_filter_save_gnss_source(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_GNSS_SOURCE_CONTROL, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_load_gnss_source(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_GNSS_SOURCE_CONTROL, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_default_gnss_source(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_GNSS_SOURCE_CONTROL, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
void insert_mip_filter_heading_source_command(microstrain_serializer* serializer, const mip_filter_heading_source_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        insert_mip_filter_heading_source_command_source(serializer, self->source);
        
    }
}
void extract_mip_filter_heading_source_command(microstrain_serializer* serializer, mip_filter_heading_source_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        extract_mip_filter_heading_source_command_source(serializer, &self->source);
        
    }
}

void insert_mip_filter_heading_source_response(microstrain_serializer* serializer, const mip_filter_heading_source_response* self)
{
    insert_mip_filter_heading_source_command_source(serializer, self->source);
    
}
void extract_mip_filter_heading_source_response(microstrain_serializer* serializer, mip_filter_heading_source_response* self)
{
    extract_mip_filter_heading_source_command_source(serializer, &self->source);
    
}

mip_cmd_result mip_filter_write_heading_source(mip_interface* device, mip_filter_heading_source_command_source source)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    insert_mip_filter_heading_source_command_source(&serializer, source);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_HEADING_UPDATE_CONTROL, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_read_heading_source(mip_interface* device, mip_filter_heading_source_command_source* source_out)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_HEADING_UPDATE_CONTROL, buffer, (uint8_t)microstrain_serializer_length(&serializer), MIP_REPLY_DESC_FILTER_HEADING_UPDATE_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        microstrain_serializer deserializer;
        microstrain_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(source_out);
        extract_mip_filter_heading_source_command_source(&deserializer, source_out);
        
        if( microstrain_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_filter_save_heading_source(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_HEADING_UPDATE_CONTROL, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_load_heading_source(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_HEADING_UPDATE_CONTROL, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_default_heading_source(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_HEADING_UPDATE_CONTROL, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
void insert_mip_filter_auto_init_control_command(microstrain_serializer* serializer, const mip_filter_auto_init_control_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        microstrain_insert_u8(serializer, self->enable);
        
    }
}
void extract_mip_filter_auto_init_control_command(microstrain_serializer* serializer, mip_filter_auto_init_control_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        microstrain_extract_u8(serializer, &self->enable);
        
    }
}

void insert_mip_filter_auto_init_control_response(microstrain_serializer* serializer, const mip_filter_auto_init_control_response* self)
{
    microstrain_insert_u8(serializer, self->enable);
    
}
void extract_mip_filter_auto_init_control_response(microstrain_serializer* serializer, mip_filter_auto_init_control_response* self)
{
    microstrain_extract_u8(serializer, &self->enable);
    
}

mip_cmd_result mip_filter_write_auto_init_control(mip_interface* device, uint8_t enable)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    microstrain_insert_u8(&serializer, enable);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_AUTOINIT_CONTROL, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_read_auto_init_control(mip_interface* device, uint8_t* enable_out)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_AUTOINIT_CONTROL, buffer, (uint8_t)microstrain_serializer_length(&serializer), MIP_REPLY_DESC_FILTER_AUTOINIT_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        microstrain_serializer deserializer;
        microstrain_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(enable_out);
        microstrain_extract_u8(&deserializer, enable_out);
        
        if( microstrain_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_filter_save_auto_init_control(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_AUTOINIT_CONTROL, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_load_auto_init_control(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_AUTOINIT_CONTROL, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_default_auto_init_control(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_AUTOINIT_CONTROL, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
void insert_mip_filter_accel_noise_command(microstrain_serializer* serializer, const mip_filter_accel_noise_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        insert_mip_vector3f(serializer, self->noise);
        
    }
}
void extract_mip_filter_accel_noise_command(microstrain_serializer* serializer, mip_filter_accel_noise_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        extract_mip_vector3f(serializer, self->noise);
        
    }
}

void insert_mip_filter_accel_noise_response(microstrain_serializer* serializer, const mip_filter_accel_noise_response* self)
{
    insert_mip_vector3f(serializer, self->noise);
    
}
void extract_mip_filter_accel_noise_response(microstrain_serializer* serializer, mip_filter_accel_noise_response* self)
{
    extract_mip_vector3f(serializer, self->noise);
    
}

mip_cmd_result mip_filter_write_accel_noise(mip_interface* device, const float* noise)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    assert(noise);
    for(unsigned int i=0; i < 3; i++)
        microstrain_insert_float(&serializer, noise[i]);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ACCEL_NOISE, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_read_accel_noise(mip_interface* device, float* noise_out)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ACCEL_NOISE, buffer, (uint8_t)microstrain_serializer_length(&serializer), MIP_REPLY_DESC_FILTER_ACCEL_NOISE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        microstrain_serializer deserializer;
        microstrain_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(noise_out);
        for(unsigned int i=0; i < 3; i++)
            microstrain_extract_float(&deserializer, &noise_out[i]);
        
        if( microstrain_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_filter_save_accel_noise(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ACCEL_NOISE, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_load_accel_noise(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ACCEL_NOISE, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_default_accel_noise(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ACCEL_NOISE, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
void insert_mip_filter_gyro_noise_command(microstrain_serializer* serializer, const mip_filter_gyro_noise_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        insert_mip_vector3f(serializer, self->noise);
        
    }
}
void extract_mip_filter_gyro_noise_command(microstrain_serializer* serializer, mip_filter_gyro_noise_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        extract_mip_vector3f(serializer, self->noise);
        
    }
}

void insert_mip_filter_gyro_noise_response(microstrain_serializer* serializer, const mip_filter_gyro_noise_response* self)
{
    insert_mip_vector3f(serializer, self->noise);
    
}
void extract_mip_filter_gyro_noise_response(microstrain_serializer* serializer, mip_filter_gyro_noise_response* self)
{
    extract_mip_vector3f(serializer, self->noise);
    
}

mip_cmd_result mip_filter_write_gyro_noise(mip_interface* device, const float* noise)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    assert(noise);
    for(unsigned int i=0; i < 3; i++)
        microstrain_insert_float(&serializer, noise[i]);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_GYRO_NOISE, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_read_gyro_noise(mip_interface* device, float* noise_out)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_GYRO_NOISE, buffer, (uint8_t)microstrain_serializer_length(&serializer), MIP_REPLY_DESC_FILTER_GYRO_NOISE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        microstrain_serializer deserializer;
        microstrain_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(noise_out);
        for(unsigned int i=0; i < 3; i++)
            microstrain_extract_float(&deserializer, &noise_out[i]);
        
        if( microstrain_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_filter_save_gyro_noise(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_GYRO_NOISE, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_load_gyro_noise(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_GYRO_NOISE, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_default_gyro_noise(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_GYRO_NOISE, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
void insert_mip_filter_accel_bias_model_command(microstrain_serializer* serializer, const mip_filter_accel_bias_model_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        insert_mip_vector3f(serializer, self->beta);
        
        insert_mip_vector3f(serializer, self->noise);
        
    }
}
void extract_mip_filter_accel_bias_model_command(microstrain_serializer* serializer, mip_filter_accel_bias_model_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        extract_mip_vector3f(serializer, self->beta);
        
        extract_mip_vector3f(serializer, self->noise);
        
    }
}

void insert_mip_filter_accel_bias_model_response(microstrain_serializer* serializer, const mip_filter_accel_bias_model_response* self)
{
    insert_mip_vector3f(serializer, self->beta);
    
    insert_mip_vector3f(serializer, self->noise);
    
}
void extract_mip_filter_accel_bias_model_response(microstrain_serializer* serializer, mip_filter_accel_bias_model_response* self)
{
    extract_mip_vector3f(serializer, self->beta);
    
    extract_mip_vector3f(serializer, self->noise);
    
}

mip_cmd_result mip_filter_write_accel_bias_model(mip_interface* device, const float* beta, const float* noise)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    assert(beta);
    for(unsigned int i=0; i < 3; i++)
        microstrain_insert_float(&serializer, beta[i]);
    
    assert(noise);
    for(unsigned int i=0; i < 3; i++)
        microstrain_insert_float(&serializer, noise[i]);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ACCEL_BIAS_MODEL, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_read_accel_bias_model(mip_interface* device, float* beta_out, float* noise_out)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ACCEL_BIAS_MODEL, buffer, (uint8_t)microstrain_serializer_length(&serializer), MIP_REPLY_DESC_FILTER_ACCEL_BIAS_MODEL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        microstrain_serializer deserializer;
        microstrain_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(beta_out);
        for(unsigned int i=0; i < 3; i++)
            microstrain_extract_float(&deserializer, &beta_out[i]);
        
        assert(noise_out);
        for(unsigned int i=0; i < 3; i++)
            microstrain_extract_float(&deserializer, &noise_out[i]);
        
        if( microstrain_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_filter_save_accel_bias_model(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ACCEL_BIAS_MODEL, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_load_accel_bias_model(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ACCEL_BIAS_MODEL, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_default_accel_bias_model(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ACCEL_BIAS_MODEL, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
void insert_mip_filter_gyro_bias_model_command(microstrain_serializer* serializer, const mip_filter_gyro_bias_model_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        insert_mip_vector3f(serializer, self->beta);
        
        insert_mip_vector3f(serializer, self->noise);
        
    }
}
void extract_mip_filter_gyro_bias_model_command(microstrain_serializer* serializer, mip_filter_gyro_bias_model_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        extract_mip_vector3f(serializer, self->beta);
        
        extract_mip_vector3f(serializer, self->noise);
        
    }
}

void insert_mip_filter_gyro_bias_model_response(microstrain_serializer* serializer, const mip_filter_gyro_bias_model_response* self)
{
    insert_mip_vector3f(serializer, self->beta);
    
    insert_mip_vector3f(serializer, self->noise);
    
}
void extract_mip_filter_gyro_bias_model_response(microstrain_serializer* serializer, mip_filter_gyro_bias_model_response* self)
{
    extract_mip_vector3f(serializer, self->beta);
    
    extract_mip_vector3f(serializer, self->noise);
    
}

mip_cmd_result mip_filter_write_gyro_bias_model(mip_interface* device, const float* beta, const float* noise)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    assert(beta);
    for(unsigned int i=0; i < 3; i++)
        microstrain_insert_float(&serializer, beta[i]);
    
    assert(noise);
    for(unsigned int i=0; i < 3; i++)
        microstrain_insert_float(&serializer, noise[i]);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_GYRO_BIAS_MODEL, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_read_gyro_bias_model(mip_interface* device, float* beta_out, float* noise_out)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_GYRO_BIAS_MODEL, buffer, (uint8_t)microstrain_serializer_length(&serializer), MIP_REPLY_DESC_FILTER_GYRO_BIAS_MODEL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        microstrain_serializer deserializer;
        microstrain_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(beta_out);
        for(unsigned int i=0; i < 3; i++)
            microstrain_extract_float(&deserializer, &beta_out[i]);
        
        assert(noise_out);
        for(unsigned int i=0; i < 3; i++)
            microstrain_extract_float(&deserializer, &noise_out[i]);
        
        if( microstrain_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_filter_save_gyro_bias_model(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_GYRO_BIAS_MODEL, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_load_gyro_bias_model(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_GYRO_BIAS_MODEL, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_default_gyro_bias_model(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_GYRO_BIAS_MODEL, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
void insert_mip_filter_altitude_aiding_command(microstrain_serializer* serializer, const mip_filter_altitude_aiding_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        insert_mip_filter_altitude_aiding_command_aiding_selector(serializer, self->selector);
        
    }
}
void extract_mip_filter_altitude_aiding_command(microstrain_serializer* serializer, mip_filter_altitude_aiding_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        extract_mip_filter_altitude_aiding_command_aiding_selector(serializer, &self->selector);
        
    }
}

void insert_mip_filter_altitude_aiding_response(microstrain_serializer* serializer, const mip_filter_altitude_aiding_response* self)
{
    insert_mip_filter_altitude_aiding_command_aiding_selector(serializer, self->selector);
    
}
void extract_mip_filter_altitude_aiding_response(microstrain_serializer* serializer, mip_filter_altitude_aiding_response* self)
{
    extract_mip_filter_altitude_aiding_command_aiding_selector(serializer, &self->selector);
    
}

mip_cmd_result mip_filter_write_altitude_aiding(mip_interface* device, mip_filter_altitude_aiding_command_aiding_selector selector)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    insert_mip_filter_altitude_aiding_command_aiding_selector(&serializer, selector);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ALTITUDE_AIDING_CONTROL, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_read_altitude_aiding(mip_interface* device, mip_filter_altitude_aiding_command_aiding_selector* selector_out)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ALTITUDE_AIDING_CONTROL, buffer, (uint8_t)microstrain_serializer_length(&serializer), MIP_REPLY_DESC_FILTER_ALTITUDE_AIDING_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        microstrain_serializer deserializer;
        microstrain_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(selector_out);
        extract_mip_filter_altitude_aiding_command_aiding_selector(&deserializer, selector_out);
        
        if( microstrain_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_filter_save_altitude_aiding(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ALTITUDE_AIDING_CONTROL, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_load_altitude_aiding(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ALTITUDE_AIDING_CONTROL, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_default_altitude_aiding(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ALTITUDE_AIDING_CONTROL, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
void insert_mip_filter_pitch_roll_aiding_command(microstrain_serializer* serializer, const mip_filter_pitch_roll_aiding_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        insert_mip_filter_pitch_roll_aiding_command_aiding_source(serializer, self->source);
        
    }
}
void extract_mip_filter_pitch_roll_aiding_command(microstrain_serializer* serializer, mip_filter_pitch_roll_aiding_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        extract_mip_filter_pitch_roll_aiding_command_aiding_source(serializer, &self->source);
        
    }
}

void insert_mip_filter_pitch_roll_aiding_response(microstrain_serializer* serializer, const mip_filter_pitch_roll_aiding_response* self)
{
    insert_mip_filter_pitch_roll_aiding_command_aiding_source(serializer, self->source);
    
}
void extract_mip_filter_pitch_roll_aiding_response(microstrain_serializer* serializer, mip_filter_pitch_roll_aiding_response* self)
{
    extract_mip_filter_pitch_roll_aiding_command_aiding_source(serializer, &self->source);
    
}

mip_cmd_result mip_filter_write_pitch_roll_aiding(mip_interface* device, mip_filter_pitch_roll_aiding_command_aiding_source source)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    insert_mip_filter_pitch_roll_aiding_command_aiding_source(&serializer, source);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SECONDARY_PITCH_ROLL_AIDING_CONTROL, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_read_pitch_roll_aiding(mip_interface* device, mip_filter_pitch_roll_aiding_command_aiding_source* source_out)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SECONDARY_PITCH_ROLL_AIDING_CONTROL, buffer, (uint8_t)microstrain_serializer_length(&serializer), MIP_REPLY_DESC_FILTER_SECONDARY_PITCH_ROLL_AIDING_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        microstrain_serializer deserializer;
        microstrain_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(source_out);
        extract_mip_filter_pitch_roll_aiding_command_aiding_source(&deserializer, source_out);
        
        if( microstrain_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_filter_save_pitch_roll_aiding(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SECONDARY_PITCH_ROLL_AIDING_CONTROL, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_load_pitch_roll_aiding(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SECONDARY_PITCH_ROLL_AIDING_CONTROL, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_default_pitch_roll_aiding(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SECONDARY_PITCH_ROLL_AIDING_CONTROL, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
void insert_mip_filter_auto_zupt_command(microstrain_serializer* serializer, const mip_filter_auto_zupt_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        microstrain_insert_u8(serializer, self->enable);
        
        microstrain_insert_float(serializer, self->threshold);
        
    }
}
void extract_mip_filter_auto_zupt_command(microstrain_serializer* serializer, mip_filter_auto_zupt_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        microstrain_extract_u8(serializer, &self->enable);
        
        microstrain_extract_float(serializer, &self->threshold);
        
    }
}

void insert_mip_filter_auto_zupt_response(microstrain_serializer* serializer, const mip_filter_auto_zupt_response* self)
{
    microstrain_insert_u8(serializer, self->enable);
    
    microstrain_insert_float(serializer, self->threshold);
    
}
void extract_mip_filter_auto_zupt_response(microstrain_serializer* serializer, mip_filter_auto_zupt_response* self)
{
    microstrain_extract_u8(serializer, &self->enable);
    
    microstrain_extract_float(serializer, &self->threshold);
    
}

mip_cmd_result mip_filter_write_auto_zupt(mip_interface* device, uint8_t enable, float threshold)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    microstrain_insert_u8(&serializer, enable);
    
    microstrain_insert_float(&serializer, threshold);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ZUPT_CONTROL, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_read_auto_zupt(mip_interface* device, uint8_t* enable_out, float* threshold_out)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ZUPT_CONTROL, buffer, (uint8_t)microstrain_serializer_length(&serializer), MIP_REPLY_DESC_FILTER_ZUPT_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        microstrain_serializer deserializer;
        microstrain_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(enable_out);
        microstrain_extract_u8(&deserializer, enable_out);
        
        assert(threshold_out);
        microstrain_extract_float(&deserializer, threshold_out);
        
        if( microstrain_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_filter_save_auto_zupt(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ZUPT_CONTROL, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_load_auto_zupt(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ZUPT_CONTROL, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_default_auto_zupt(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ZUPT_CONTROL, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
void insert_mip_filter_auto_angular_zupt_command(microstrain_serializer* serializer, const mip_filter_auto_angular_zupt_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        microstrain_insert_u8(serializer, self->enable);
        
        microstrain_insert_float(serializer, self->threshold);
        
    }
}
void extract_mip_filter_auto_angular_zupt_command(microstrain_serializer* serializer, mip_filter_auto_angular_zupt_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        microstrain_extract_u8(serializer, &self->enable);
        
        microstrain_extract_float(serializer, &self->threshold);
        
    }
}

void insert_mip_filter_auto_angular_zupt_response(microstrain_serializer* serializer, const mip_filter_auto_angular_zupt_response* self)
{
    microstrain_insert_u8(serializer, self->enable);
    
    microstrain_insert_float(serializer, self->threshold);
    
}
void extract_mip_filter_auto_angular_zupt_response(microstrain_serializer* serializer, mip_filter_auto_angular_zupt_response* self)
{
    microstrain_extract_u8(serializer, &self->enable);
    
    microstrain_extract_float(serializer, &self->threshold);
    
}

mip_cmd_result mip_filter_write_auto_angular_zupt(mip_interface* device, uint8_t enable, float threshold)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    microstrain_insert_u8(&serializer, enable);
    
    microstrain_insert_float(&serializer, threshold);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ANGULAR_ZUPT_CONTROL, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_read_auto_angular_zupt(mip_interface* device, uint8_t* enable_out, float* threshold_out)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ANGULAR_ZUPT_CONTROL, buffer, (uint8_t)microstrain_serializer_length(&serializer), MIP_REPLY_DESC_FILTER_ANGULAR_ZUPT_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        microstrain_serializer deserializer;
        microstrain_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(enable_out);
        microstrain_extract_u8(&deserializer, enable_out);
        
        assert(threshold_out);
        microstrain_extract_float(&deserializer, threshold_out);
        
        if( microstrain_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_filter_save_auto_angular_zupt(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ANGULAR_ZUPT_CONTROL, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_load_auto_angular_zupt(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ANGULAR_ZUPT_CONTROL, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_default_auto_angular_zupt(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ANGULAR_ZUPT_CONTROL, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_commanded_zupt(mip_interface* device)
{
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_COMMANDED_ZUPT, NULL, 0);
}
mip_cmd_result mip_filter_commanded_angular_zupt(mip_interface* device)
{
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_COMMANDED_ANGULAR_ZUPT, NULL, 0);
}
void insert_mip_filter_mag_capture_auto_cal_command(microstrain_serializer* serializer, const mip_filter_mag_capture_auto_cal_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
}
void extract_mip_filter_mag_capture_auto_cal_command(microstrain_serializer* serializer, mip_filter_mag_capture_auto_cal_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
}

mip_cmd_result mip_filter_write_mag_capture_auto_cal(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_MAG_CAPTURE_AUTO_CALIBRATION, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_save_mag_capture_auto_cal(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_MAG_CAPTURE_AUTO_CALIBRATION, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
void insert_mip_filter_gravity_noise_command(microstrain_serializer* serializer, const mip_filter_gravity_noise_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        insert_mip_vector3f(serializer, self->noise);
        
    }
}
void extract_mip_filter_gravity_noise_command(microstrain_serializer* serializer, mip_filter_gravity_noise_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        extract_mip_vector3f(serializer, self->noise);
        
    }
}

void insert_mip_filter_gravity_noise_response(microstrain_serializer* serializer, const mip_filter_gravity_noise_response* self)
{
    insert_mip_vector3f(serializer, self->noise);
    
}
void extract_mip_filter_gravity_noise_response(microstrain_serializer* serializer, mip_filter_gravity_noise_response* self)
{
    extract_mip_vector3f(serializer, self->noise);
    
}

mip_cmd_result mip_filter_write_gravity_noise(mip_interface* device, const float* noise)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    assert(noise);
    for(unsigned int i=0; i < 3; i++)
        microstrain_insert_float(&serializer, noise[i]);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_GRAVITY_NOISE, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_read_gravity_noise(mip_interface* device, float* noise_out)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_GRAVITY_NOISE, buffer, (uint8_t)microstrain_serializer_length(&serializer), MIP_REPLY_DESC_FILTER_GRAVITY_NOISE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        microstrain_serializer deserializer;
        microstrain_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(noise_out);
        for(unsigned int i=0; i < 3; i++)
            microstrain_extract_float(&deserializer, &noise_out[i]);
        
        if( microstrain_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_filter_save_gravity_noise(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_GRAVITY_NOISE, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_load_gravity_noise(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_GRAVITY_NOISE, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_default_gravity_noise(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_GRAVITY_NOISE, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
void insert_mip_filter_pressure_altitude_noise_command(microstrain_serializer* serializer, const mip_filter_pressure_altitude_noise_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        microstrain_insert_float(serializer, self->noise);
        
    }
}
void extract_mip_filter_pressure_altitude_noise_command(microstrain_serializer* serializer, mip_filter_pressure_altitude_noise_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        microstrain_extract_float(serializer, &self->noise);
        
    }
}

void insert_mip_filter_pressure_altitude_noise_response(microstrain_serializer* serializer, const mip_filter_pressure_altitude_noise_response* self)
{
    microstrain_insert_float(serializer, self->noise);
    
}
void extract_mip_filter_pressure_altitude_noise_response(microstrain_serializer* serializer, mip_filter_pressure_altitude_noise_response* self)
{
    microstrain_extract_float(serializer, &self->noise);
    
}

mip_cmd_result mip_filter_write_pressure_altitude_noise(mip_interface* device, float noise)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    microstrain_insert_float(&serializer, noise);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_PRESSURE_NOISE, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_read_pressure_altitude_noise(mip_interface* device, float* noise_out)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_PRESSURE_NOISE, buffer, (uint8_t)microstrain_serializer_length(&serializer), MIP_REPLY_DESC_FILTER_PRESSURE_NOISE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        microstrain_serializer deserializer;
        microstrain_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(noise_out);
        microstrain_extract_float(&deserializer, noise_out);
        
        if( microstrain_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_filter_save_pressure_altitude_noise(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_PRESSURE_NOISE, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_load_pressure_altitude_noise(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_PRESSURE_NOISE, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_default_pressure_altitude_noise(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_PRESSURE_NOISE, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
void insert_mip_filter_hard_iron_offset_noise_command(microstrain_serializer* serializer, const mip_filter_hard_iron_offset_noise_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        insert_mip_vector3f(serializer, self->noise);
        
    }
}
void extract_mip_filter_hard_iron_offset_noise_command(microstrain_serializer* serializer, mip_filter_hard_iron_offset_noise_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        extract_mip_vector3f(serializer, self->noise);
        
    }
}

void insert_mip_filter_hard_iron_offset_noise_response(microstrain_serializer* serializer, const mip_filter_hard_iron_offset_noise_response* self)
{
    insert_mip_vector3f(serializer, self->noise);
    
}
void extract_mip_filter_hard_iron_offset_noise_response(microstrain_serializer* serializer, mip_filter_hard_iron_offset_noise_response* self)
{
    extract_mip_vector3f(serializer, self->noise);
    
}

mip_cmd_result mip_filter_write_hard_iron_offset_noise(mip_interface* device, const float* noise)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    assert(noise);
    for(unsigned int i=0; i < 3; i++)
        microstrain_insert_float(&serializer, noise[i]);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_HARD_IRON_OFFSET_NOISE, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_read_hard_iron_offset_noise(mip_interface* device, float* noise_out)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_HARD_IRON_OFFSET_NOISE, buffer, (uint8_t)microstrain_serializer_length(&serializer), MIP_REPLY_DESC_FILTER_HARD_IRON_OFFSET_NOISE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        microstrain_serializer deserializer;
        microstrain_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(noise_out);
        for(unsigned int i=0; i < 3; i++)
            microstrain_extract_float(&deserializer, &noise_out[i]);
        
        if( microstrain_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_filter_save_hard_iron_offset_noise(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_HARD_IRON_OFFSET_NOISE, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_load_hard_iron_offset_noise(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_HARD_IRON_OFFSET_NOISE, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_default_hard_iron_offset_noise(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_HARD_IRON_OFFSET_NOISE, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
void insert_mip_filter_soft_iron_matrix_noise_command(microstrain_serializer* serializer, const mip_filter_soft_iron_matrix_noise_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        insert_mip_matrix3f(serializer, self->noise);
        
    }
}
void extract_mip_filter_soft_iron_matrix_noise_command(microstrain_serializer* serializer, mip_filter_soft_iron_matrix_noise_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        extract_mip_matrix3f(serializer, self->noise);
        
    }
}

void insert_mip_filter_soft_iron_matrix_noise_response(microstrain_serializer* serializer, const mip_filter_soft_iron_matrix_noise_response* self)
{
    insert_mip_matrix3f(serializer, self->noise);
    
}
void extract_mip_filter_soft_iron_matrix_noise_response(microstrain_serializer* serializer, mip_filter_soft_iron_matrix_noise_response* self)
{
    extract_mip_matrix3f(serializer, self->noise);
    
}

mip_cmd_result mip_filter_write_soft_iron_matrix_noise(mip_interface* device, const float* noise)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    assert(noise);
    for(unsigned int i=0; i < 9; i++)
        microstrain_insert_float(&serializer, noise[i]);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SOFT_IRON_MATRIX_NOISE, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_read_soft_iron_matrix_noise(mip_interface* device, float* noise_out)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SOFT_IRON_MATRIX_NOISE, buffer, (uint8_t)microstrain_serializer_length(&serializer), MIP_REPLY_DESC_FILTER_SOFT_IRON_MATRIX_NOISE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        microstrain_serializer deserializer;
        microstrain_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(noise_out);
        for(unsigned int i=0; i < 9; i++)
            microstrain_extract_float(&deserializer, &noise_out[i]);
        
        if( microstrain_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_filter_save_soft_iron_matrix_noise(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SOFT_IRON_MATRIX_NOISE, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_load_soft_iron_matrix_noise(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SOFT_IRON_MATRIX_NOISE, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_default_soft_iron_matrix_noise(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SOFT_IRON_MATRIX_NOISE, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
void insert_mip_filter_mag_noise_command(microstrain_serializer* serializer, const mip_filter_mag_noise_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        insert_mip_vector3f(serializer, self->noise);
        
    }
}
void extract_mip_filter_mag_noise_command(microstrain_serializer* serializer, mip_filter_mag_noise_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        extract_mip_vector3f(serializer, self->noise);
        
    }
}

void insert_mip_filter_mag_noise_response(microstrain_serializer* serializer, const mip_filter_mag_noise_response* self)
{
    insert_mip_vector3f(serializer, self->noise);
    
}
void extract_mip_filter_mag_noise_response(microstrain_serializer* serializer, mip_filter_mag_noise_response* self)
{
    extract_mip_vector3f(serializer, self->noise);
    
}

mip_cmd_result mip_filter_write_mag_noise(mip_interface* device, const float* noise)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    assert(noise);
    for(unsigned int i=0; i < 3; i++)
        microstrain_insert_float(&serializer, noise[i]);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_MAG_NOISE, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_read_mag_noise(mip_interface* device, float* noise_out)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_MAG_NOISE, buffer, (uint8_t)microstrain_serializer_length(&serializer), MIP_REPLY_DESC_FILTER_MAG_NOISE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        microstrain_serializer deserializer;
        microstrain_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(noise_out);
        for(unsigned int i=0; i < 3; i++)
            microstrain_extract_float(&deserializer, &noise_out[i]);
        
        if( microstrain_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_filter_save_mag_noise(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_MAG_NOISE, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_load_mag_noise(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_MAG_NOISE, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_default_mag_noise(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_MAG_NOISE, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
void insert_mip_filter_inclination_source_command(microstrain_serializer* serializer, const mip_filter_inclination_source_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        insert_mip_filter_mag_param_source(serializer, self->source);
        
        microstrain_insert_float(serializer, self->inclination);
        
    }
}
void extract_mip_filter_inclination_source_command(microstrain_serializer* serializer, mip_filter_inclination_source_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        extract_mip_filter_mag_param_source(serializer, &self->source);
        
        microstrain_extract_float(serializer, &self->inclination);
        
    }
}

void insert_mip_filter_inclination_source_response(microstrain_serializer* serializer, const mip_filter_inclination_source_response* self)
{
    insert_mip_filter_mag_param_source(serializer, self->source);
    
    microstrain_insert_float(serializer, self->inclination);
    
}
void extract_mip_filter_inclination_source_response(microstrain_serializer* serializer, mip_filter_inclination_source_response* self)
{
    extract_mip_filter_mag_param_source(serializer, &self->source);
    
    microstrain_extract_float(serializer, &self->inclination);
    
}

mip_cmd_result mip_filter_write_inclination_source(mip_interface* device, mip_filter_mag_param_source source, float inclination)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    insert_mip_filter_mag_param_source(&serializer, source);
    
    microstrain_insert_float(&serializer, inclination);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_INCLINATION_SOURCE, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_read_inclination_source(mip_interface* device, mip_filter_mag_param_source* source_out, float* inclination_out)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_INCLINATION_SOURCE, buffer, (uint8_t)microstrain_serializer_length(&serializer), MIP_REPLY_DESC_FILTER_INCLINATION_SOURCE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        microstrain_serializer deserializer;
        microstrain_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(source_out);
        extract_mip_filter_mag_param_source(&deserializer, source_out);
        
        assert(inclination_out);
        microstrain_extract_float(&deserializer, inclination_out);
        
        if( microstrain_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_filter_save_inclination_source(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_INCLINATION_SOURCE, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_load_inclination_source(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_INCLINATION_SOURCE, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_default_inclination_source(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_INCLINATION_SOURCE, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
void insert_mip_filter_magnetic_declination_source_command(microstrain_serializer* serializer, const mip_filter_magnetic_declination_source_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        insert_mip_filter_mag_param_source(serializer, self->source);
        
        microstrain_insert_float(serializer, self->declination);
        
    }
}
void extract_mip_filter_magnetic_declination_source_command(microstrain_serializer* serializer, mip_filter_magnetic_declination_source_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        extract_mip_filter_mag_param_source(serializer, &self->source);
        
        microstrain_extract_float(serializer, &self->declination);
        
    }
}

void insert_mip_filter_magnetic_declination_source_response(microstrain_serializer* serializer, const mip_filter_magnetic_declination_source_response* self)
{
    insert_mip_filter_mag_param_source(serializer, self->source);
    
    microstrain_insert_float(serializer, self->declination);
    
}
void extract_mip_filter_magnetic_declination_source_response(microstrain_serializer* serializer, mip_filter_magnetic_declination_source_response* self)
{
    extract_mip_filter_mag_param_source(serializer, &self->source);
    
    microstrain_extract_float(serializer, &self->declination);
    
}

mip_cmd_result mip_filter_write_magnetic_declination_source(mip_interface* device, mip_filter_mag_param_source source, float declination)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    insert_mip_filter_mag_param_source(&serializer, source);
    
    microstrain_insert_float(&serializer, declination);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_DECLINATION_SOURCE, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_read_magnetic_declination_source(mip_interface* device, mip_filter_mag_param_source* source_out, float* declination_out)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_DECLINATION_SOURCE, buffer, (uint8_t)microstrain_serializer_length(&serializer), MIP_REPLY_DESC_FILTER_DECLINATION_SOURCE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        microstrain_serializer deserializer;
        microstrain_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(source_out);
        extract_mip_filter_mag_param_source(&deserializer, source_out);
        
        assert(declination_out);
        microstrain_extract_float(&deserializer, declination_out);
        
        if( microstrain_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_filter_save_magnetic_declination_source(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_DECLINATION_SOURCE, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_load_magnetic_declination_source(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_DECLINATION_SOURCE, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_default_magnetic_declination_source(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_DECLINATION_SOURCE, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
void insert_mip_filter_mag_field_magnitude_source_command(microstrain_serializer* serializer, const mip_filter_mag_field_magnitude_source_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        insert_mip_filter_mag_param_source(serializer, self->source);
        
        microstrain_insert_float(serializer, self->magnitude);
        
    }
}
void extract_mip_filter_mag_field_magnitude_source_command(microstrain_serializer* serializer, mip_filter_mag_field_magnitude_source_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        extract_mip_filter_mag_param_source(serializer, &self->source);
        
        microstrain_extract_float(serializer, &self->magnitude);
        
    }
}

void insert_mip_filter_mag_field_magnitude_source_response(microstrain_serializer* serializer, const mip_filter_mag_field_magnitude_source_response* self)
{
    insert_mip_filter_mag_param_source(serializer, self->source);
    
    microstrain_insert_float(serializer, self->magnitude);
    
}
void extract_mip_filter_mag_field_magnitude_source_response(microstrain_serializer* serializer, mip_filter_mag_field_magnitude_source_response* self)
{
    extract_mip_filter_mag_param_source(serializer, &self->source);
    
    microstrain_extract_float(serializer, &self->magnitude);
    
}

mip_cmd_result mip_filter_write_mag_field_magnitude_source(mip_interface* device, mip_filter_mag_param_source source, float magnitude)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    insert_mip_filter_mag_param_source(&serializer, source);
    
    microstrain_insert_float(&serializer, magnitude);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_MAGNETIC_MAGNITUDE_SOURCE, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_read_mag_field_magnitude_source(mip_interface* device, mip_filter_mag_param_source* source_out, float* magnitude_out)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_MAGNETIC_MAGNITUDE_SOURCE, buffer, (uint8_t)microstrain_serializer_length(&serializer), MIP_REPLY_DESC_FILTER_MAGNETIC_MAGNITUDE_SOURCE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        microstrain_serializer deserializer;
        microstrain_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(source_out);
        extract_mip_filter_mag_param_source(&deserializer, source_out);
        
        assert(magnitude_out);
        microstrain_extract_float(&deserializer, magnitude_out);
        
        if( microstrain_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_filter_save_mag_field_magnitude_source(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_MAGNETIC_MAGNITUDE_SOURCE, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_load_mag_field_magnitude_source(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_MAGNETIC_MAGNITUDE_SOURCE, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_default_mag_field_magnitude_source(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_MAGNETIC_MAGNITUDE_SOURCE, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
void insert_mip_filter_reference_position_command(microstrain_serializer* serializer, const mip_filter_reference_position_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        microstrain_insert_bool(serializer, self->enable);
        
        microstrain_insert_double(serializer, self->latitude);
        
        microstrain_insert_double(serializer, self->longitude);
        
        microstrain_insert_double(serializer, self->altitude);
        
    }
}
void extract_mip_filter_reference_position_command(microstrain_serializer* serializer, mip_filter_reference_position_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        microstrain_extract_bool(serializer, &self->enable);
        
        microstrain_extract_double(serializer, &self->latitude);
        
        microstrain_extract_double(serializer, &self->longitude);
        
        microstrain_extract_double(serializer, &self->altitude);
        
    }
}

void insert_mip_filter_reference_position_response(microstrain_serializer* serializer, const mip_filter_reference_position_response* self)
{
    microstrain_insert_bool(serializer, self->enable);
    
    microstrain_insert_double(serializer, self->latitude);
    
    microstrain_insert_double(serializer, self->longitude);
    
    microstrain_insert_double(serializer, self->altitude);
    
}
void extract_mip_filter_reference_position_response(microstrain_serializer* serializer, mip_filter_reference_position_response* self)
{
    microstrain_extract_bool(serializer, &self->enable);
    
    microstrain_extract_double(serializer, &self->latitude);
    
    microstrain_extract_double(serializer, &self->longitude);
    
    microstrain_extract_double(serializer, &self->altitude);
    
}

mip_cmd_result mip_filter_write_reference_position(mip_interface* device, bool enable, double latitude, double longitude, double altitude)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    microstrain_insert_bool(&serializer, enable);
    
    microstrain_insert_double(&serializer, latitude);
    
    microstrain_insert_double(&serializer, longitude);
    
    microstrain_insert_double(&serializer, altitude);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_REFERENCE_POSITION, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_read_reference_position(mip_interface* device, bool* enable_out, double* latitude_out, double* longitude_out, double* altitude_out)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_REFERENCE_POSITION, buffer, (uint8_t)microstrain_serializer_length(&serializer), MIP_REPLY_DESC_FILTER_REFERENCE_POSITION, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        microstrain_serializer deserializer;
        microstrain_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(enable_out);
        microstrain_extract_bool(&deserializer, enable_out);
        
        assert(latitude_out);
        microstrain_extract_double(&deserializer, latitude_out);
        
        assert(longitude_out);
        microstrain_extract_double(&deserializer, longitude_out);
        
        assert(altitude_out);
        microstrain_extract_double(&deserializer, altitude_out);
        
        if( microstrain_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_filter_save_reference_position(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_REFERENCE_POSITION, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_load_reference_position(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_REFERENCE_POSITION, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_default_reference_position(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_REFERENCE_POSITION, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
void insert_mip_filter_accel_magnitude_error_adaptive_measurement_command(microstrain_serializer* serializer, const mip_filter_accel_magnitude_error_adaptive_measurement_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        insert_mip_filter_adaptive_measurement(serializer, self->adaptive_measurement);
        
        microstrain_insert_float(serializer, self->frequency);
        
        microstrain_insert_float(serializer, self->low_limit);
        
        microstrain_insert_float(serializer, self->high_limit);
        
        microstrain_insert_float(serializer, self->low_limit_uncertainty);
        
        microstrain_insert_float(serializer, self->high_limit_uncertainty);
        
        microstrain_insert_float(serializer, self->minimum_uncertainty);
        
    }
}
void extract_mip_filter_accel_magnitude_error_adaptive_measurement_command(microstrain_serializer* serializer, mip_filter_accel_magnitude_error_adaptive_measurement_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        extract_mip_filter_adaptive_measurement(serializer, &self->adaptive_measurement);
        
        microstrain_extract_float(serializer, &self->frequency);
        
        microstrain_extract_float(serializer, &self->low_limit);
        
        microstrain_extract_float(serializer, &self->high_limit);
        
        microstrain_extract_float(serializer, &self->low_limit_uncertainty);
        
        microstrain_extract_float(serializer, &self->high_limit_uncertainty);
        
        microstrain_extract_float(serializer, &self->minimum_uncertainty);
        
    }
}

void insert_mip_filter_accel_magnitude_error_adaptive_measurement_response(microstrain_serializer* serializer, const mip_filter_accel_magnitude_error_adaptive_measurement_response* self)
{
    insert_mip_filter_adaptive_measurement(serializer, self->adaptive_measurement);
    
    microstrain_insert_float(serializer, self->frequency);
    
    microstrain_insert_float(serializer, self->low_limit);
    
    microstrain_insert_float(serializer, self->high_limit);
    
    microstrain_insert_float(serializer, self->low_limit_uncertainty);
    
    microstrain_insert_float(serializer, self->high_limit_uncertainty);
    
    microstrain_insert_float(serializer, self->minimum_uncertainty);
    
}
void extract_mip_filter_accel_magnitude_error_adaptive_measurement_response(microstrain_serializer* serializer, mip_filter_accel_magnitude_error_adaptive_measurement_response* self)
{
    extract_mip_filter_adaptive_measurement(serializer, &self->adaptive_measurement);
    
    microstrain_extract_float(serializer, &self->frequency);
    
    microstrain_extract_float(serializer, &self->low_limit);
    
    microstrain_extract_float(serializer, &self->high_limit);
    
    microstrain_extract_float(serializer, &self->low_limit_uncertainty);
    
    microstrain_extract_float(serializer, &self->high_limit_uncertainty);
    
    microstrain_extract_float(serializer, &self->minimum_uncertainty);
    
}

mip_cmd_result mip_filter_write_accel_magnitude_error_adaptive_measurement(mip_interface* device, mip_filter_adaptive_measurement adaptive_measurement, float frequency, float low_limit, float high_limit, float low_limit_uncertainty, float high_limit_uncertainty, float minimum_uncertainty)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    insert_mip_filter_adaptive_measurement(&serializer, adaptive_measurement);
    
    microstrain_insert_float(&serializer, frequency);
    
    microstrain_insert_float(&serializer, low_limit);
    
    microstrain_insert_float(&serializer, high_limit);
    
    microstrain_insert_float(&serializer, low_limit_uncertainty);
    
    microstrain_insert_float(&serializer, high_limit_uncertainty);
    
    microstrain_insert_float(&serializer, minimum_uncertainty);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ACCEL_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_read_accel_magnitude_error_adaptive_measurement(mip_interface* device, mip_filter_adaptive_measurement* adaptive_measurement_out, float* frequency_out, float* low_limit_out, float* high_limit_out, float* low_limit_uncertainty_out, float* high_limit_uncertainty_out, float* minimum_uncertainty_out)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ACCEL_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, (uint8_t)microstrain_serializer_length(&serializer), MIP_REPLY_DESC_FILTER_ACCEL_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        microstrain_serializer deserializer;
        microstrain_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(adaptive_measurement_out);
        extract_mip_filter_adaptive_measurement(&deserializer, adaptive_measurement_out);
        
        assert(frequency_out);
        microstrain_extract_float(&deserializer, frequency_out);
        
        assert(low_limit_out);
        microstrain_extract_float(&deserializer, low_limit_out);
        
        assert(high_limit_out);
        microstrain_extract_float(&deserializer, high_limit_out);
        
        assert(low_limit_uncertainty_out);
        microstrain_extract_float(&deserializer, low_limit_uncertainty_out);
        
        assert(high_limit_uncertainty_out);
        microstrain_extract_float(&deserializer, high_limit_uncertainty_out);
        
        assert(minimum_uncertainty_out);
        microstrain_extract_float(&deserializer, minimum_uncertainty_out);
        
        if( microstrain_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_filter_save_accel_magnitude_error_adaptive_measurement(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ACCEL_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_load_accel_magnitude_error_adaptive_measurement(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ACCEL_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_default_accel_magnitude_error_adaptive_measurement(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ACCEL_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
void insert_mip_filter_mag_magnitude_error_adaptive_measurement_command(microstrain_serializer* serializer, const mip_filter_mag_magnitude_error_adaptive_measurement_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        insert_mip_filter_adaptive_measurement(serializer, self->adaptive_measurement);
        
        microstrain_insert_float(serializer, self->frequency);
        
        microstrain_insert_float(serializer, self->low_limit);
        
        microstrain_insert_float(serializer, self->high_limit);
        
        microstrain_insert_float(serializer, self->low_limit_uncertainty);
        
        microstrain_insert_float(serializer, self->high_limit_uncertainty);
        
        microstrain_insert_float(serializer, self->minimum_uncertainty);
        
    }
}
void extract_mip_filter_mag_magnitude_error_adaptive_measurement_command(microstrain_serializer* serializer, mip_filter_mag_magnitude_error_adaptive_measurement_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        extract_mip_filter_adaptive_measurement(serializer, &self->adaptive_measurement);
        
        microstrain_extract_float(serializer, &self->frequency);
        
        microstrain_extract_float(serializer, &self->low_limit);
        
        microstrain_extract_float(serializer, &self->high_limit);
        
        microstrain_extract_float(serializer, &self->low_limit_uncertainty);
        
        microstrain_extract_float(serializer, &self->high_limit_uncertainty);
        
        microstrain_extract_float(serializer, &self->minimum_uncertainty);
        
    }
}

void insert_mip_filter_mag_magnitude_error_adaptive_measurement_response(microstrain_serializer* serializer, const mip_filter_mag_magnitude_error_adaptive_measurement_response* self)
{
    insert_mip_filter_adaptive_measurement(serializer, self->adaptive_measurement);
    
    microstrain_insert_float(serializer, self->frequency);
    
    microstrain_insert_float(serializer, self->low_limit);
    
    microstrain_insert_float(serializer, self->high_limit);
    
    microstrain_insert_float(serializer, self->low_limit_uncertainty);
    
    microstrain_insert_float(serializer, self->high_limit_uncertainty);
    
    microstrain_insert_float(serializer, self->minimum_uncertainty);
    
}
void extract_mip_filter_mag_magnitude_error_adaptive_measurement_response(microstrain_serializer* serializer, mip_filter_mag_magnitude_error_adaptive_measurement_response* self)
{
    extract_mip_filter_adaptive_measurement(serializer, &self->adaptive_measurement);
    
    microstrain_extract_float(serializer, &self->frequency);
    
    microstrain_extract_float(serializer, &self->low_limit);
    
    microstrain_extract_float(serializer, &self->high_limit);
    
    microstrain_extract_float(serializer, &self->low_limit_uncertainty);
    
    microstrain_extract_float(serializer, &self->high_limit_uncertainty);
    
    microstrain_extract_float(serializer, &self->minimum_uncertainty);
    
}

mip_cmd_result mip_filter_write_mag_magnitude_error_adaptive_measurement(mip_interface* device, mip_filter_adaptive_measurement adaptive_measurement, float frequency, float low_limit, float high_limit, float low_limit_uncertainty, float high_limit_uncertainty, float minimum_uncertainty)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    insert_mip_filter_adaptive_measurement(&serializer, adaptive_measurement);
    
    microstrain_insert_float(&serializer, frequency);
    
    microstrain_insert_float(&serializer, low_limit);
    
    microstrain_insert_float(&serializer, high_limit);
    
    microstrain_insert_float(&serializer, low_limit_uncertainty);
    
    microstrain_insert_float(&serializer, high_limit_uncertainty);
    
    microstrain_insert_float(&serializer, minimum_uncertainty);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_MAG_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_read_mag_magnitude_error_adaptive_measurement(mip_interface* device, mip_filter_adaptive_measurement* adaptive_measurement_out, float* frequency_out, float* low_limit_out, float* high_limit_out, float* low_limit_uncertainty_out, float* high_limit_uncertainty_out, float* minimum_uncertainty_out)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_MAG_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, (uint8_t)microstrain_serializer_length(&serializer), MIP_REPLY_DESC_FILTER_MAG_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        microstrain_serializer deserializer;
        microstrain_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(adaptive_measurement_out);
        extract_mip_filter_adaptive_measurement(&deserializer, adaptive_measurement_out);
        
        assert(frequency_out);
        microstrain_extract_float(&deserializer, frequency_out);
        
        assert(low_limit_out);
        microstrain_extract_float(&deserializer, low_limit_out);
        
        assert(high_limit_out);
        microstrain_extract_float(&deserializer, high_limit_out);
        
        assert(low_limit_uncertainty_out);
        microstrain_extract_float(&deserializer, low_limit_uncertainty_out);
        
        assert(high_limit_uncertainty_out);
        microstrain_extract_float(&deserializer, high_limit_uncertainty_out);
        
        assert(minimum_uncertainty_out);
        microstrain_extract_float(&deserializer, minimum_uncertainty_out);
        
        if( microstrain_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_filter_save_mag_magnitude_error_adaptive_measurement(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_MAG_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_load_mag_magnitude_error_adaptive_measurement(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_MAG_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_default_mag_magnitude_error_adaptive_measurement(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_MAG_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
void insert_mip_filter_mag_dip_angle_error_adaptive_measurement_command(microstrain_serializer* serializer, const mip_filter_mag_dip_angle_error_adaptive_measurement_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        microstrain_insert_bool(serializer, self->enable);
        
        microstrain_insert_float(serializer, self->frequency);
        
        microstrain_insert_float(serializer, self->high_limit);
        
        microstrain_insert_float(serializer, self->high_limit_uncertainty);
        
        microstrain_insert_float(serializer, self->minimum_uncertainty);
        
    }
}
void extract_mip_filter_mag_dip_angle_error_adaptive_measurement_command(microstrain_serializer* serializer, mip_filter_mag_dip_angle_error_adaptive_measurement_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        microstrain_extract_bool(serializer, &self->enable);
        
        microstrain_extract_float(serializer, &self->frequency);
        
        microstrain_extract_float(serializer, &self->high_limit);
        
        microstrain_extract_float(serializer, &self->high_limit_uncertainty);
        
        microstrain_extract_float(serializer, &self->minimum_uncertainty);
        
    }
}

void insert_mip_filter_mag_dip_angle_error_adaptive_measurement_response(microstrain_serializer* serializer, const mip_filter_mag_dip_angle_error_adaptive_measurement_response* self)
{
    microstrain_insert_bool(serializer, self->enable);
    
    microstrain_insert_float(serializer, self->frequency);
    
    microstrain_insert_float(serializer, self->high_limit);
    
    microstrain_insert_float(serializer, self->high_limit_uncertainty);
    
    microstrain_insert_float(serializer, self->minimum_uncertainty);
    
}
void extract_mip_filter_mag_dip_angle_error_adaptive_measurement_response(microstrain_serializer* serializer, mip_filter_mag_dip_angle_error_adaptive_measurement_response* self)
{
    microstrain_extract_bool(serializer, &self->enable);
    
    microstrain_extract_float(serializer, &self->frequency);
    
    microstrain_extract_float(serializer, &self->high_limit);
    
    microstrain_extract_float(serializer, &self->high_limit_uncertainty);
    
    microstrain_extract_float(serializer, &self->minimum_uncertainty);
    
}

mip_cmd_result mip_filter_write_mag_dip_angle_error_adaptive_measurement(mip_interface* device, bool enable, float frequency, float high_limit, float high_limit_uncertainty, float minimum_uncertainty)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    microstrain_insert_bool(&serializer, enable);
    
    microstrain_insert_float(&serializer, frequency);
    
    microstrain_insert_float(&serializer, high_limit);
    
    microstrain_insert_float(&serializer, high_limit_uncertainty);
    
    microstrain_insert_float(&serializer, minimum_uncertainty);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_MAG_DIP_ANGLE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_read_mag_dip_angle_error_adaptive_measurement(mip_interface* device, bool* enable_out, float* frequency_out, float* high_limit_out, float* high_limit_uncertainty_out, float* minimum_uncertainty_out)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_MAG_DIP_ANGLE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, (uint8_t)microstrain_serializer_length(&serializer), MIP_REPLY_DESC_FILTER_MAG_DIP_ANGLE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        microstrain_serializer deserializer;
        microstrain_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(enable_out);
        microstrain_extract_bool(&deserializer, enable_out);
        
        assert(frequency_out);
        microstrain_extract_float(&deserializer, frequency_out);
        
        assert(high_limit_out);
        microstrain_extract_float(&deserializer, high_limit_out);
        
        assert(high_limit_uncertainty_out);
        microstrain_extract_float(&deserializer, high_limit_uncertainty_out);
        
        assert(minimum_uncertainty_out);
        microstrain_extract_float(&deserializer, minimum_uncertainty_out);
        
        if( microstrain_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_filter_save_mag_dip_angle_error_adaptive_measurement(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_MAG_DIP_ANGLE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_load_mag_dip_angle_error_adaptive_measurement(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_MAG_DIP_ANGLE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_default_mag_dip_angle_error_adaptive_measurement(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_MAG_DIP_ANGLE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
void insert_mip_filter_aiding_measurement_enable_command(microstrain_serializer* serializer, const mip_filter_aiding_measurement_enable_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    insert_mip_filter_aiding_measurement_enable_command_aiding_source(serializer, self->aiding_source);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        microstrain_insert_bool(serializer, self->enable);
        
    }
}
void extract_mip_filter_aiding_measurement_enable_command(microstrain_serializer* serializer, mip_filter_aiding_measurement_enable_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    extract_mip_filter_aiding_measurement_enable_command_aiding_source(serializer, &self->aiding_source);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        microstrain_extract_bool(serializer, &self->enable);
        
    }
}

void insert_mip_filter_aiding_measurement_enable_response(microstrain_serializer* serializer, const mip_filter_aiding_measurement_enable_response* self)
{
    insert_mip_filter_aiding_measurement_enable_command_aiding_source(serializer, self->aiding_source);
    
    microstrain_insert_bool(serializer, self->enable);
    
}
void extract_mip_filter_aiding_measurement_enable_response(microstrain_serializer* serializer, mip_filter_aiding_measurement_enable_response* self)
{
    extract_mip_filter_aiding_measurement_enable_command_aiding_source(serializer, &self->aiding_source);
    
    microstrain_extract_bool(serializer, &self->enable);
    
}

mip_cmd_result mip_filter_write_aiding_measurement_enable(mip_interface* device, mip_filter_aiding_measurement_enable_command_aiding_source aiding_source, bool enable)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    insert_mip_filter_aiding_measurement_enable_command_aiding_source(&serializer, aiding_source);
    
    microstrain_insert_bool(&serializer, enable);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_AIDING_MEASUREMENT_ENABLE, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_read_aiding_measurement_enable(mip_interface* device, mip_filter_aiding_measurement_enable_command_aiding_source aiding_source, bool* enable_out)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    insert_mip_filter_aiding_measurement_enable_command_aiding_source(&serializer, aiding_source);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_AIDING_MEASUREMENT_ENABLE, buffer, (uint8_t)microstrain_serializer_length(&serializer), MIP_REPLY_DESC_FILTER_AIDING_MEASUREMENT_ENABLE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        microstrain_serializer deserializer;
        microstrain_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        extract_mip_filter_aiding_measurement_enable_command_aiding_source(&deserializer, &aiding_source);
        
        assert(enable_out);
        microstrain_extract_bool(&deserializer, enable_out);
        
        if( microstrain_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_filter_save_aiding_measurement_enable(mip_interface* device, mip_filter_aiding_measurement_enable_command_aiding_source aiding_source)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    insert_mip_filter_aiding_measurement_enable_command_aiding_source(&serializer, aiding_source);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_AIDING_MEASUREMENT_ENABLE, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_load_aiding_measurement_enable(mip_interface* device, mip_filter_aiding_measurement_enable_command_aiding_source aiding_source)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    insert_mip_filter_aiding_measurement_enable_command_aiding_source(&serializer, aiding_source);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_AIDING_MEASUREMENT_ENABLE, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_default_aiding_measurement_enable(mip_interface* device, mip_filter_aiding_measurement_enable_command_aiding_source aiding_source)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    insert_mip_filter_aiding_measurement_enable_command_aiding_source(&serializer, aiding_source);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_AIDING_MEASUREMENT_ENABLE, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_run(mip_interface* device)
{
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_RUN, NULL, 0);
}
void insert_mip_filter_kinematic_constraint_command(microstrain_serializer* serializer, const mip_filter_kinematic_constraint_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        microstrain_insert_u8(serializer, self->acceleration_constraint_selection);
        
        microstrain_insert_u8(serializer, self->velocity_constraint_selection);
        
        microstrain_insert_u8(serializer, self->angular_constraint_selection);
        
    }
}
void extract_mip_filter_kinematic_constraint_command(microstrain_serializer* serializer, mip_filter_kinematic_constraint_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        microstrain_extract_u8(serializer, &self->acceleration_constraint_selection);
        
        microstrain_extract_u8(serializer, &self->velocity_constraint_selection);
        
        microstrain_extract_u8(serializer, &self->angular_constraint_selection);
        
    }
}

void insert_mip_filter_kinematic_constraint_response(microstrain_serializer* serializer, const mip_filter_kinematic_constraint_response* self)
{
    microstrain_insert_u8(serializer, self->acceleration_constraint_selection);
    
    microstrain_insert_u8(serializer, self->velocity_constraint_selection);
    
    microstrain_insert_u8(serializer, self->angular_constraint_selection);
    
}
void extract_mip_filter_kinematic_constraint_response(microstrain_serializer* serializer, mip_filter_kinematic_constraint_response* self)
{
    microstrain_extract_u8(serializer, &self->acceleration_constraint_selection);
    
    microstrain_extract_u8(serializer, &self->velocity_constraint_selection);
    
    microstrain_extract_u8(serializer, &self->angular_constraint_selection);
    
}

mip_cmd_result mip_filter_write_kinematic_constraint(mip_interface* device, uint8_t acceleration_constraint_selection, uint8_t velocity_constraint_selection, uint8_t angular_constraint_selection)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    microstrain_insert_u8(&serializer, acceleration_constraint_selection);
    
    microstrain_insert_u8(&serializer, velocity_constraint_selection);
    
    microstrain_insert_u8(&serializer, angular_constraint_selection);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_KINEMATIC_CONSTRAINT, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_read_kinematic_constraint(mip_interface* device, uint8_t* acceleration_constraint_selection_out, uint8_t* velocity_constraint_selection_out, uint8_t* angular_constraint_selection_out)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_KINEMATIC_CONSTRAINT, buffer, (uint8_t)microstrain_serializer_length(&serializer), MIP_REPLY_DESC_FILTER_KINEMATIC_CONSTRAINT, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        microstrain_serializer deserializer;
        microstrain_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(acceleration_constraint_selection_out);
        microstrain_extract_u8(&deserializer, acceleration_constraint_selection_out);
        
        assert(velocity_constraint_selection_out);
        microstrain_extract_u8(&deserializer, velocity_constraint_selection_out);
        
        assert(angular_constraint_selection_out);
        microstrain_extract_u8(&deserializer, angular_constraint_selection_out);
        
        if( microstrain_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_filter_save_kinematic_constraint(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_KINEMATIC_CONSTRAINT, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_load_kinematic_constraint(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_KINEMATIC_CONSTRAINT, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_default_kinematic_constraint(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_KINEMATIC_CONSTRAINT, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
void insert_mip_filter_initialization_configuration_command(microstrain_serializer* serializer, const mip_filter_initialization_configuration_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        microstrain_insert_u8(serializer, self->wait_for_run_command);
        
        insert_mip_filter_initialization_configuration_command_initial_condition_source(serializer, self->initial_cond_src);
        
        insert_mip_filter_initialization_configuration_command_alignment_selector(serializer, self->auto_heading_alignment_selector);
        
        microstrain_insert_float(serializer, self->initial_heading);
        
        microstrain_insert_float(serializer, self->initial_pitch);
        
        microstrain_insert_float(serializer, self->initial_roll);
        
        insert_mip_vector3f(serializer, self->initial_position);
        
        insert_mip_vector3f(serializer, self->initial_velocity);
        
        insert_mip_filter_reference_frame(serializer, self->reference_frame_selector);
        
    }
}
void extract_mip_filter_initialization_configuration_command(microstrain_serializer* serializer, mip_filter_initialization_configuration_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        microstrain_extract_u8(serializer, &self->wait_for_run_command);
        
        extract_mip_filter_initialization_configuration_command_initial_condition_source(serializer, &self->initial_cond_src);
        
        extract_mip_filter_initialization_configuration_command_alignment_selector(serializer, &self->auto_heading_alignment_selector);
        
        microstrain_extract_float(serializer, &self->initial_heading);
        
        microstrain_extract_float(serializer, &self->initial_pitch);
        
        microstrain_extract_float(serializer, &self->initial_roll);
        
        extract_mip_vector3f(serializer, self->initial_position);
        
        extract_mip_vector3f(serializer, self->initial_velocity);
        
        extract_mip_filter_reference_frame(serializer, &self->reference_frame_selector);
        
    }
}

void insert_mip_filter_initialization_configuration_response(microstrain_serializer* serializer, const mip_filter_initialization_configuration_response* self)
{
    microstrain_insert_u8(serializer, self->wait_for_run_command);
    
    insert_mip_filter_initialization_configuration_command_initial_condition_source(serializer, self->initial_cond_src);
    
    insert_mip_filter_initialization_configuration_command_alignment_selector(serializer, self->auto_heading_alignment_selector);
    
    microstrain_insert_float(serializer, self->initial_heading);
    
    microstrain_insert_float(serializer, self->initial_pitch);
    
    microstrain_insert_float(serializer, self->initial_roll);
    
    insert_mip_vector3f(serializer, self->initial_position);
    
    insert_mip_vector3f(serializer, self->initial_velocity);
    
    insert_mip_filter_reference_frame(serializer, self->reference_frame_selector);
    
}
void extract_mip_filter_initialization_configuration_response(microstrain_serializer* serializer, mip_filter_initialization_configuration_response* self)
{
    microstrain_extract_u8(serializer, &self->wait_for_run_command);
    
    extract_mip_filter_initialization_configuration_command_initial_condition_source(serializer, &self->initial_cond_src);
    
    extract_mip_filter_initialization_configuration_command_alignment_selector(serializer, &self->auto_heading_alignment_selector);
    
    microstrain_extract_float(serializer, &self->initial_heading);
    
    microstrain_extract_float(serializer, &self->initial_pitch);
    
    microstrain_extract_float(serializer, &self->initial_roll);
    
    extract_mip_vector3f(serializer, self->initial_position);
    
    extract_mip_vector3f(serializer, self->initial_velocity);
    
    extract_mip_filter_reference_frame(serializer, &self->reference_frame_selector);
    
}

mip_cmd_result mip_filter_write_initialization_configuration(mip_interface* device, uint8_t wait_for_run_command, mip_filter_initialization_configuration_command_initial_condition_source initial_cond_src, mip_filter_initialization_configuration_command_alignment_selector auto_heading_alignment_selector, float initial_heading, float initial_pitch, float initial_roll, const float* initial_position, const float* initial_velocity, mip_filter_reference_frame reference_frame_selector)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    microstrain_insert_u8(&serializer, wait_for_run_command);
    
    insert_mip_filter_initialization_configuration_command_initial_condition_source(&serializer, initial_cond_src);
    
    insert_mip_filter_initialization_configuration_command_alignment_selector(&serializer, auto_heading_alignment_selector);
    
    microstrain_insert_float(&serializer, initial_heading);
    
    microstrain_insert_float(&serializer, initial_pitch);
    
    microstrain_insert_float(&serializer, initial_roll);
    
    assert(initial_position);
    for(unsigned int i=0; i < 3; i++)
        microstrain_insert_float(&serializer, initial_position[i]);
    
    assert(initial_velocity);
    for(unsigned int i=0; i < 3; i++)
        microstrain_insert_float(&serializer, initial_velocity[i]);
    
    insert_mip_filter_reference_frame(&serializer, reference_frame_selector);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_INITIALIZATION_CONFIGURATION, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_read_initialization_configuration(mip_interface* device, uint8_t* wait_for_run_command_out, mip_filter_initialization_configuration_command_initial_condition_source* initial_cond_src_out, mip_filter_initialization_configuration_command_alignment_selector* auto_heading_alignment_selector_out, float* initial_heading_out, float* initial_pitch_out, float* initial_roll_out, float* initial_position_out, float* initial_velocity_out, mip_filter_reference_frame* reference_frame_selector_out)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_INITIALIZATION_CONFIGURATION, buffer, (uint8_t)microstrain_serializer_length(&serializer), MIP_REPLY_DESC_FILTER_INITIALIZATION_CONFIGURATION, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        microstrain_serializer deserializer;
        microstrain_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(wait_for_run_command_out);
        microstrain_extract_u8(&deserializer, wait_for_run_command_out);
        
        assert(initial_cond_src_out);
        extract_mip_filter_initialization_configuration_command_initial_condition_source(&deserializer, initial_cond_src_out);
        
        assert(auto_heading_alignment_selector_out);
        extract_mip_filter_initialization_configuration_command_alignment_selector(&deserializer, auto_heading_alignment_selector_out);
        
        assert(initial_heading_out);
        microstrain_extract_float(&deserializer, initial_heading_out);
        
        assert(initial_pitch_out);
        microstrain_extract_float(&deserializer, initial_pitch_out);
        
        assert(initial_roll_out);
        microstrain_extract_float(&deserializer, initial_roll_out);
        
        assert(initial_position_out);
        for(unsigned int i=0; i < 3; i++)
            microstrain_extract_float(&deserializer, &initial_position_out[i]);
        
        assert(initial_velocity_out);
        for(unsigned int i=0; i < 3; i++)
            microstrain_extract_float(&deserializer, &initial_velocity_out[i]);
        
        assert(reference_frame_selector_out);
        extract_mip_filter_reference_frame(&deserializer, reference_frame_selector_out);
        
        if( microstrain_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_filter_save_initialization_configuration(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_INITIALIZATION_CONFIGURATION, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_load_initialization_configuration(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_INITIALIZATION_CONFIGURATION, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_default_initialization_configuration(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_INITIALIZATION_CONFIGURATION, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
void insert_mip_filter_adaptive_filter_options_command(microstrain_serializer* serializer, const mip_filter_adaptive_filter_options_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        microstrain_insert_u8(serializer, self->level);
        
        microstrain_insert_u16(serializer, self->time_limit);
        
    }
}
void extract_mip_filter_adaptive_filter_options_command(microstrain_serializer* serializer, mip_filter_adaptive_filter_options_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        microstrain_extract_u8(serializer, &self->level);
        
        microstrain_extract_u16(serializer, &self->time_limit);
        
    }
}

void insert_mip_filter_adaptive_filter_options_response(microstrain_serializer* serializer, const mip_filter_adaptive_filter_options_response* self)
{
    microstrain_insert_u8(serializer, self->level);
    
    microstrain_insert_u16(serializer, self->time_limit);
    
}
void extract_mip_filter_adaptive_filter_options_response(microstrain_serializer* serializer, mip_filter_adaptive_filter_options_response* self)
{
    microstrain_extract_u8(serializer, &self->level);
    
    microstrain_extract_u16(serializer, &self->time_limit);
    
}

mip_cmd_result mip_filter_write_adaptive_filter_options(mip_interface* device, uint8_t level, uint16_t time_limit)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    microstrain_insert_u8(&serializer, level);
    
    microstrain_insert_u16(&serializer, time_limit);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ADAPTIVE_FILTER_OPTIONS, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_read_adaptive_filter_options(mip_interface* device, uint8_t* level_out, uint16_t* time_limit_out)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ADAPTIVE_FILTER_OPTIONS, buffer, (uint8_t)microstrain_serializer_length(&serializer), MIP_REPLY_DESC_FILTER_ADAPTIVE_FILTER_OPTIONS, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        microstrain_serializer deserializer;
        microstrain_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(level_out);
        microstrain_extract_u8(&deserializer, level_out);
        
        assert(time_limit_out);
        microstrain_extract_u16(&deserializer, time_limit_out);
        
        if( microstrain_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_filter_save_adaptive_filter_options(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ADAPTIVE_FILTER_OPTIONS, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_load_adaptive_filter_options(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ADAPTIVE_FILTER_OPTIONS, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_default_adaptive_filter_options(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ADAPTIVE_FILTER_OPTIONS, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
void insert_mip_filter_multi_antenna_offset_command(microstrain_serializer* serializer, const mip_filter_multi_antenna_offset_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    microstrain_insert_u8(serializer, self->receiver_id);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        insert_mip_vector3f(serializer, self->antenna_offset);
        
    }
}
void extract_mip_filter_multi_antenna_offset_command(microstrain_serializer* serializer, mip_filter_multi_antenna_offset_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    microstrain_extract_u8(serializer, &self->receiver_id);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        extract_mip_vector3f(serializer, self->antenna_offset);
        
    }
}

void insert_mip_filter_multi_antenna_offset_response(microstrain_serializer* serializer, const mip_filter_multi_antenna_offset_response* self)
{
    microstrain_insert_u8(serializer, self->receiver_id);
    
    insert_mip_vector3f(serializer, self->antenna_offset);
    
}
void extract_mip_filter_multi_antenna_offset_response(microstrain_serializer* serializer, mip_filter_multi_antenna_offset_response* self)
{
    microstrain_extract_u8(serializer, &self->receiver_id);
    
    extract_mip_vector3f(serializer, self->antenna_offset);
    
}

mip_cmd_result mip_filter_write_multi_antenna_offset(mip_interface* device, uint8_t receiver_id, const float* antenna_offset)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    microstrain_insert_u8(&serializer, receiver_id);
    
    assert(antenna_offset);
    for(unsigned int i=0; i < 3; i++)
        microstrain_insert_float(&serializer, antenna_offset[i]);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_MULTI_ANTENNA_OFFSET, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_read_multi_antenna_offset(mip_interface* device, uint8_t receiver_id, float* antenna_offset_out)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    microstrain_insert_u8(&serializer, receiver_id);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_MULTI_ANTENNA_OFFSET, buffer, (uint8_t)microstrain_serializer_length(&serializer), MIP_REPLY_DESC_FILTER_MULTI_ANTENNA_OFFSET, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        microstrain_serializer deserializer;
        microstrain_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        microstrain_extract_u8(&deserializer, &receiver_id);
        
        assert(antenna_offset_out);
        for(unsigned int i=0; i < 3; i++)
            microstrain_extract_float(&deserializer, &antenna_offset_out[i]);
        
        if( microstrain_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_filter_save_multi_antenna_offset(mip_interface* device, uint8_t receiver_id)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    microstrain_insert_u8(&serializer, receiver_id);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_MULTI_ANTENNA_OFFSET, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_load_multi_antenna_offset(mip_interface* device, uint8_t receiver_id)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    microstrain_insert_u8(&serializer, receiver_id);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_MULTI_ANTENNA_OFFSET, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_default_multi_antenna_offset(mip_interface* device, uint8_t receiver_id)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    microstrain_insert_u8(&serializer, receiver_id);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_MULTI_ANTENNA_OFFSET, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
void insert_mip_filter_rel_pos_configuration_command(microstrain_serializer* serializer, const mip_filter_rel_pos_configuration_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        microstrain_insert_u8(serializer, self->source);
        
        insert_mip_filter_reference_frame(serializer, self->reference_frame_selector);
        
        insert_mip_vector3d(serializer, self->reference_coordinates);
        
    }
}
void extract_mip_filter_rel_pos_configuration_command(microstrain_serializer* serializer, mip_filter_rel_pos_configuration_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        microstrain_extract_u8(serializer, &self->source);
        
        extract_mip_filter_reference_frame(serializer, &self->reference_frame_selector);
        
        extract_mip_vector3d(serializer, self->reference_coordinates);
        
    }
}

void insert_mip_filter_rel_pos_configuration_response(microstrain_serializer* serializer, const mip_filter_rel_pos_configuration_response* self)
{
    microstrain_insert_u8(serializer, self->source);
    
    insert_mip_filter_reference_frame(serializer, self->reference_frame_selector);
    
    insert_mip_vector3d(serializer, self->reference_coordinates);
    
}
void extract_mip_filter_rel_pos_configuration_response(microstrain_serializer* serializer, mip_filter_rel_pos_configuration_response* self)
{
    microstrain_extract_u8(serializer, &self->source);
    
    extract_mip_filter_reference_frame(serializer, &self->reference_frame_selector);
    
    extract_mip_vector3d(serializer, self->reference_coordinates);
    
}

mip_cmd_result mip_filter_write_rel_pos_configuration(mip_interface* device, uint8_t source, mip_filter_reference_frame reference_frame_selector, const double* reference_coordinates)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    microstrain_insert_u8(&serializer, source);
    
    insert_mip_filter_reference_frame(&serializer, reference_frame_selector);
    
    assert(reference_coordinates);
    for(unsigned int i=0; i < 3; i++)
        microstrain_insert_double(&serializer, reference_coordinates[i]);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_REL_POS_CONFIGURATION, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_read_rel_pos_configuration(mip_interface* device, uint8_t* source_out, mip_filter_reference_frame* reference_frame_selector_out, double* reference_coordinates_out)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_REL_POS_CONFIGURATION, buffer, (uint8_t)microstrain_serializer_length(&serializer), MIP_REPLY_DESC_FILTER_REL_POS_CONFIGURATION, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        microstrain_serializer deserializer;
        microstrain_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(source_out);
        microstrain_extract_u8(&deserializer, source_out);
        
        assert(reference_frame_selector_out);
        extract_mip_filter_reference_frame(&deserializer, reference_frame_selector_out);
        
        assert(reference_coordinates_out);
        for(unsigned int i=0; i < 3; i++)
            microstrain_extract_double(&deserializer, &reference_coordinates_out[i]);
        
        if( microstrain_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_filter_save_rel_pos_configuration(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_REL_POS_CONFIGURATION, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_load_rel_pos_configuration(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_REL_POS_CONFIGURATION, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_default_rel_pos_configuration(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_REL_POS_CONFIGURATION, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
void insert_mip_filter_ref_point_lever_arm_command(microstrain_serializer* serializer, const mip_filter_ref_point_lever_arm_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        insert_mip_filter_ref_point_lever_arm_command_reference_point_selector(serializer, self->ref_point_sel);
        
        insert_mip_vector3f(serializer, self->lever_arm_offset);
        
    }
}
void extract_mip_filter_ref_point_lever_arm_command(microstrain_serializer* serializer, mip_filter_ref_point_lever_arm_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        extract_mip_filter_ref_point_lever_arm_command_reference_point_selector(serializer, &self->ref_point_sel);
        
        extract_mip_vector3f(serializer, self->lever_arm_offset);
        
    }
}

void insert_mip_filter_ref_point_lever_arm_response(microstrain_serializer* serializer, const mip_filter_ref_point_lever_arm_response* self)
{
    insert_mip_filter_ref_point_lever_arm_command_reference_point_selector(serializer, self->ref_point_sel);
    
    insert_mip_vector3f(serializer, self->lever_arm_offset);
    
}
void extract_mip_filter_ref_point_lever_arm_response(microstrain_serializer* serializer, mip_filter_ref_point_lever_arm_response* self)
{
    extract_mip_filter_ref_point_lever_arm_command_reference_point_selector(serializer, &self->ref_point_sel);
    
    extract_mip_vector3f(serializer, self->lever_arm_offset);
    
}

mip_cmd_result mip_filter_write_ref_point_lever_arm(mip_interface* device, mip_filter_ref_point_lever_arm_command_reference_point_selector ref_point_sel, const float* lever_arm_offset)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    insert_mip_filter_ref_point_lever_arm_command_reference_point_selector(&serializer, ref_point_sel);
    
    assert(lever_arm_offset);
    for(unsigned int i=0; i < 3; i++)
        microstrain_insert_float(&serializer, lever_arm_offset[i]);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_REF_POINT_LEVER_ARM, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_read_ref_point_lever_arm(mip_interface* device, mip_filter_ref_point_lever_arm_command_reference_point_selector* ref_point_sel_out, float* lever_arm_offset_out)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_REF_POINT_LEVER_ARM, buffer, (uint8_t)microstrain_serializer_length(&serializer), MIP_REPLY_DESC_FILTER_REF_POINT_LEVER_ARM, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        microstrain_serializer deserializer;
        microstrain_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(ref_point_sel_out);
        extract_mip_filter_ref_point_lever_arm_command_reference_point_selector(&deserializer, ref_point_sel_out);
        
        assert(lever_arm_offset_out);
        for(unsigned int i=0; i < 3; i++)
            microstrain_extract_float(&deserializer, &lever_arm_offset_out[i]);
        
        if( microstrain_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_filter_save_ref_point_lever_arm(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_REF_POINT_LEVER_ARM, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_load_ref_point_lever_arm(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_REF_POINT_LEVER_ARM, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_default_ref_point_lever_arm(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_REF_POINT_LEVER_ARM, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
void insert_mip_filter_speed_measurement_command(microstrain_serializer* serializer, const mip_filter_speed_measurement_command* self)
{
    microstrain_insert_u8(serializer, self->source);
    
    microstrain_insert_float(serializer, self->time_of_week);
    
    microstrain_insert_float(serializer, self->speed);
    
    microstrain_insert_float(serializer, self->speed_uncertainty);
    
}
void extract_mip_filter_speed_measurement_command(microstrain_serializer* serializer, mip_filter_speed_measurement_command* self)
{
    microstrain_extract_u8(serializer, &self->source);
    
    microstrain_extract_float(serializer, &self->time_of_week);
    
    microstrain_extract_float(serializer, &self->speed);
    
    microstrain_extract_float(serializer, &self->speed_uncertainty);
    
}

mip_cmd_result mip_filter_speed_measurement(mip_interface* device, uint8_t source, float time_of_week, float speed, float speed_uncertainty)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    microstrain_insert_u8(&serializer, source);
    
    microstrain_insert_float(&serializer, time_of_week);
    
    microstrain_insert_float(&serializer, speed);
    
    microstrain_insert_float(&serializer, speed_uncertainty);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SPEED_MEASUREMENT, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
void insert_mip_filter_speed_lever_arm_command(microstrain_serializer* serializer, const mip_filter_speed_lever_arm_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    microstrain_insert_u8(serializer, self->source);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        insert_mip_vector3f(serializer, self->lever_arm_offset);
        
    }
}
void extract_mip_filter_speed_lever_arm_command(microstrain_serializer* serializer, mip_filter_speed_lever_arm_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    microstrain_extract_u8(serializer, &self->source);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        extract_mip_vector3f(serializer, self->lever_arm_offset);
        
    }
}

void insert_mip_filter_speed_lever_arm_response(microstrain_serializer* serializer, const mip_filter_speed_lever_arm_response* self)
{
    microstrain_insert_u8(serializer, self->source);
    
    insert_mip_vector3f(serializer, self->lever_arm_offset);
    
}
void extract_mip_filter_speed_lever_arm_response(microstrain_serializer* serializer, mip_filter_speed_lever_arm_response* self)
{
    microstrain_extract_u8(serializer, &self->source);
    
    extract_mip_vector3f(serializer, self->lever_arm_offset);
    
}

mip_cmd_result mip_filter_write_speed_lever_arm(mip_interface* device, uint8_t source, const float* lever_arm_offset)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    microstrain_insert_u8(&serializer, source);
    
    assert(lever_arm_offset);
    for(unsigned int i=0; i < 3; i++)
        microstrain_insert_float(&serializer, lever_arm_offset[i]);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SPEED_LEVER_ARM, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_read_speed_lever_arm(mip_interface* device, uint8_t source, float* lever_arm_offset_out)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    microstrain_insert_u8(&serializer, source);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SPEED_LEVER_ARM, buffer, (uint8_t)microstrain_serializer_length(&serializer), MIP_REPLY_DESC_FILTER_SPEED_LEVER_ARM, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        microstrain_serializer deserializer;
        microstrain_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        microstrain_extract_u8(&deserializer, &source);
        
        assert(lever_arm_offset_out);
        for(unsigned int i=0; i < 3; i++)
            microstrain_extract_float(&deserializer, &lever_arm_offset_out[i]);
        
        if( microstrain_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_filter_save_speed_lever_arm(mip_interface* device, uint8_t source)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    microstrain_insert_u8(&serializer, source);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SPEED_LEVER_ARM, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_load_speed_lever_arm(mip_interface* device, uint8_t source)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    microstrain_insert_u8(&serializer, source);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SPEED_LEVER_ARM, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_default_speed_lever_arm(mip_interface* device, uint8_t source)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    microstrain_insert_u8(&serializer, source);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SPEED_LEVER_ARM, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
void insert_mip_filter_wheeled_vehicle_constraint_control_command(microstrain_serializer* serializer, const mip_filter_wheeled_vehicle_constraint_control_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        microstrain_insert_u8(serializer, self->enable);
        
    }
}
void extract_mip_filter_wheeled_vehicle_constraint_control_command(microstrain_serializer* serializer, mip_filter_wheeled_vehicle_constraint_control_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        microstrain_extract_u8(serializer, &self->enable);
        
    }
}

void insert_mip_filter_wheeled_vehicle_constraint_control_response(microstrain_serializer* serializer, const mip_filter_wheeled_vehicle_constraint_control_response* self)
{
    microstrain_insert_u8(serializer, self->enable);
    
}
void extract_mip_filter_wheeled_vehicle_constraint_control_response(microstrain_serializer* serializer, mip_filter_wheeled_vehicle_constraint_control_response* self)
{
    microstrain_extract_u8(serializer, &self->enable);
    
}

mip_cmd_result mip_filter_write_wheeled_vehicle_constraint_control(mip_interface* device, uint8_t enable)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    microstrain_insert_u8(&serializer, enable);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_WHEELED_VEHICLE_CONSTRAINT_CONTROL, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_read_wheeled_vehicle_constraint_control(mip_interface* device, uint8_t* enable_out)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_WHEELED_VEHICLE_CONSTRAINT_CONTROL, buffer, (uint8_t)microstrain_serializer_length(&serializer), MIP_REPLY_DESC_WHEELED_VEHICLE_CONSTRAINT_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        microstrain_serializer deserializer;
        microstrain_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(enable_out);
        microstrain_extract_u8(&deserializer, enable_out);
        
        if( microstrain_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_filter_save_wheeled_vehicle_constraint_control(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_WHEELED_VEHICLE_CONSTRAINT_CONTROL, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_load_wheeled_vehicle_constraint_control(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_WHEELED_VEHICLE_CONSTRAINT_CONTROL, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_default_wheeled_vehicle_constraint_control(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_WHEELED_VEHICLE_CONSTRAINT_CONTROL, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
void insert_mip_filter_vertical_gyro_constraint_control_command(microstrain_serializer* serializer, const mip_filter_vertical_gyro_constraint_control_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        microstrain_insert_u8(serializer, self->enable);
        
    }
}
void extract_mip_filter_vertical_gyro_constraint_control_command(microstrain_serializer* serializer, mip_filter_vertical_gyro_constraint_control_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        microstrain_extract_u8(serializer, &self->enable);
        
    }
}

void insert_mip_filter_vertical_gyro_constraint_control_response(microstrain_serializer* serializer, const mip_filter_vertical_gyro_constraint_control_response* self)
{
    microstrain_insert_u8(serializer, self->enable);
    
}
void extract_mip_filter_vertical_gyro_constraint_control_response(microstrain_serializer* serializer, mip_filter_vertical_gyro_constraint_control_response* self)
{
    microstrain_extract_u8(serializer, &self->enable);
    
}

mip_cmd_result mip_filter_write_vertical_gyro_constraint_control(mip_interface* device, uint8_t enable)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    microstrain_insert_u8(&serializer, enable);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_VERTICAL_GYRO_CONSTRAINT_CONTROL, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_read_vertical_gyro_constraint_control(mip_interface* device, uint8_t* enable_out)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_VERTICAL_GYRO_CONSTRAINT_CONTROL, buffer, (uint8_t)microstrain_serializer_length(&serializer), MIP_REPLY_DESC_VERTICAL_GYRO_CONSTRAINT_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        microstrain_serializer deserializer;
        microstrain_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(enable_out);
        microstrain_extract_u8(&deserializer, enable_out);
        
        if( microstrain_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_filter_save_vertical_gyro_constraint_control(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_VERTICAL_GYRO_CONSTRAINT_CONTROL, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_load_vertical_gyro_constraint_control(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_VERTICAL_GYRO_CONSTRAINT_CONTROL, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_default_vertical_gyro_constraint_control(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_VERTICAL_GYRO_CONSTRAINT_CONTROL, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
void insert_mip_filter_gnss_antenna_cal_control_command(microstrain_serializer* serializer, const mip_filter_gnss_antenna_cal_control_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        microstrain_insert_u8(serializer, self->enable);
        
        microstrain_insert_float(serializer, self->max_offset);
        
    }
}
void extract_mip_filter_gnss_antenna_cal_control_command(microstrain_serializer* serializer, mip_filter_gnss_antenna_cal_control_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        microstrain_extract_u8(serializer, &self->enable);
        
        microstrain_extract_float(serializer, &self->max_offset);
        
    }
}

void insert_mip_filter_gnss_antenna_cal_control_response(microstrain_serializer* serializer, const mip_filter_gnss_antenna_cal_control_response* self)
{
    microstrain_insert_u8(serializer, self->enable);
    
    microstrain_insert_float(serializer, self->max_offset);
    
}
void extract_mip_filter_gnss_antenna_cal_control_response(microstrain_serializer* serializer, mip_filter_gnss_antenna_cal_control_response* self)
{
    microstrain_extract_u8(serializer, &self->enable);
    
    microstrain_extract_float(serializer, &self->max_offset);
    
}

mip_cmd_result mip_filter_write_gnss_antenna_cal_control(mip_interface* device, uint8_t enable, float max_offset)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    microstrain_insert_u8(&serializer, enable);
    
    microstrain_insert_float(&serializer, max_offset);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_GNSS_ANTENNA_CALIBRATION_CONTROL, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_read_gnss_antenna_cal_control(mip_interface* device, uint8_t* enable_out, float* max_offset_out)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_GNSS_ANTENNA_CALIBRATION_CONTROL, buffer, (uint8_t)microstrain_serializer_length(&serializer), MIP_REPLY_DESC_GNSS_ANTENNA_CALIBRATION_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        microstrain_serializer deserializer;
        microstrain_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(enable_out);
        microstrain_extract_u8(&deserializer, enable_out);
        
        assert(max_offset_out);
        microstrain_extract_float(&deserializer, max_offset_out);
        
        if( microstrain_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_filter_save_gnss_antenna_cal_control(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_GNSS_ANTENNA_CALIBRATION_CONTROL, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_load_gnss_antenna_cal_control(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_GNSS_ANTENNA_CALIBRATION_CONTROL, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_filter_default_gnss_antenna_cal_control(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_GNSS_ANTENNA_CALIBRATION_CONTROL, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
void insert_mip_filter_set_initial_heading_command(microstrain_serializer* serializer, const mip_filter_set_initial_heading_command* self)
{
    microstrain_insert_float(serializer, self->heading);
    
}
void extract_mip_filter_set_initial_heading_command(microstrain_serializer* serializer, mip_filter_set_initial_heading_command* self)
{
    microstrain_extract_float(serializer, &self->heading);
    
}

mip_cmd_result mip_filter_set_initial_heading(mip_interface* device, float heading)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    microstrain_insert_float(&serializer, heading);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SET_INITIAL_HEADING, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}

#ifdef __cplusplus
} // extern "C"
} // namespace C
} // namespace mip
#endif // __cplusplus

