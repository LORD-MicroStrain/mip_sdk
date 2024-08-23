
#include "commands_aiding.h"

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

void insert_mip_time(microstrain_serializer* serializer, const mip_time* self)
{
    insert_mip_time_timebase(serializer, self->timebase);
    
    microstrain_insert_u8(serializer, self->reserved);
    
    microstrain_insert_u64(serializer, self->nanoseconds);
    
}
void extract_mip_time(microstrain_serializer* serializer, mip_time* self)
{
    extract_mip_time_timebase(serializer, &self->timebase);
    
    microstrain_extract_u8(serializer, &self->reserved);
    
    microstrain_extract_u64(serializer, &self->nanoseconds);
    
}

void insert_mip_aiding_frame_config_command(microstrain_serializer* serializer, const mip_aiding_frame_config_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    microstrain_insert_u8(serializer, self->frame_id);
    
    if( self->function == MIP_FUNCTION_WRITE || self->function == MIP_FUNCTION_READ )
    {
        insert_mip_aiding_frame_config_command_format(serializer, self->format);
        
    }
    if( self->function == MIP_FUNCTION_WRITE )
    {
        microstrain_insert_bool(serializer, self->tracking_enabled);
        
        insert_mip_vector3f(serializer, self->translation);
        
        if( self->format == MIP_AIDING_FRAME_CONFIG_COMMAND_FORMAT_EULER )
        {
            insert_mip_vector3f(serializer, self->rotation.euler);
            
        }
        if( self->format == MIP_AIDING_FRAME_CONFIG_COMMAND_FORMAT_QUATERNION )
        {
            insert_mip_quatf(serializer, self->rotation.quaternion);
            
        }
    }
}
void extract_mip_aiding_frame_config_command(microstrain_serializer* serializer, mip_aiding_frame_config_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    microstrain_extract_u8(serializer, &self->frame_id);
    
    if( self->function == MIP_FUNCTION_WRITE || self->function == MIP_FUNCTION_READ )
    {
        extract_mip_aiding_frame_config_command_format(serializer, &self->format);
        
    }
    if( self->function == MIP_FUNCTION_WRITE )
    {
        microstrain_extract_bool(serializer, &self->tracking_enabled);
        
        extract_mip_vector3f(serializer, self->translation);
        
        if( self->format == MIP_AIDING_FRAME_CONFIG_COMMAND_FORMAT_EULER )
        {
            extract_mip_vector3f(serializer, self->rotation.euler);
            
        }
        if( self->format == MIP_AIDING_FRAME_CONFIG_COMMAND_FORMAT_QUATERNION )
        {
            extract_mip_quatf(serializer, self->rotation.quaternion);
            
        }
    }
}

void insert_mip_aiding_frame_config_response(microstrain_serializer* serializer, const mip_aiding_frame_config_response* self)
{
    microstrain_insert_u8(serializer, self->frame_id);
    
    insert_mip_aiding_frame_config_command_format(serializer, self->format);
    
    microstrain_insert_bool(serializer, self->tracking_enabled);
    
    insert_mip_vector3f(serializer, self->translation);
    
    if( self->format == MIP_AIDING_FRAME_CONFIG_COMMAND_FORMAT_EULER )
    {
        insert_mip_vector3f(serializer, self->rotation.euler);
        
    }
    if( self->format == MIP_AIDING_FRAME_CONFIG_COMMAND_FORMAT_QUATERNION )
    {
        insert_mip_quatf(serializer, self->rotation.quaternion);
        
    }
}
void extract_mip_aiding_frame_config_response(microstrain_serializer* serializer, mip_aiding_frame_config_response* self)
{
    microstrain_extract_u8(serializer, &self->frame_id);
    
    extract_mip_aiding_frame_config_command_format(serializer, &self->format);
    
    microstrain_extract_bool(serializer, &self->tracking_enabled);
    
    extract_mip_vector3f(serializer, self->translation);
    
    if( self->format == MIP_AIDING_FRAME_CONFIG_COMMAND_FORMAT_EULER )
    {
        extract_mip_vector3f(serializer, self->rotation.euler);
        
    }
    if( self->format == MIP_AIDING_FRAME_CONFIG_COMMAND_FORMAT_QUATERNION )
    {
        extract_mip_quatf(serializer, self->rotation.quaternion);
        
    }
}

mip_cmd_result mip_aiding_write_frame_config(mip_interface* device, uint8_t frame_id, mip_aiding_frame_config_command_format format, bool tracking_enabled, const float* translation, const mip_aiding_frame_config_command_rotation* rotation)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    microstrain_insert_u8(&serializer, frame_id);
    
    insert_mip_aiding_frame_config_command_format(&serializer, format);
    
    microstrain_insert_bool(&serializer, tracking_enabled);
    
    assert(translation);
    for(unsigned int i=0; i < 3; i++)
        microstrain_insert_float(&serializer, translation[i]);
    
    if( format == MIP_AIDING_FRAME_CONFIG_COMMAND_FORMAT_EULER )
    {
        insert_mip_vector3f(&serializer, rotation->euler);
        
    }
    if( format == MIP_AIDING_FRAME_CONFIG_COMMAND_FORMAT_QUATERNION )
    {
        insert_mip_quatf(&serializer, rotation->quaternion);
        
    }
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_AIDING_CMD_DESC_SET, MIP_CMD_DESC_AIDING_FRAME_CONFIG, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_aiding_read_frame_config(mip_interface* device, uint8_t frame_id, mip_aiding_frame_config_command_format format, bool* tracking_enabled_out, float* translation_out, mip_aiding_frame_config_command_rotation* rotation_out)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    microstrain_insert_u8(&serializer, frame_id);
    
    insert_mip_aiding_frame_config_command_format(&serializer, format);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_AIDING_CMD_DESC_SET, MIP_CMD_DESC_AIDING_FRAME_CONFIG, buffer, (uint8_t)microstrain_serializer_length(&serializer), MIP_REPLY_DESC_AIDING_FRAME_CONFIG, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        microstrain_serializer deserializer;
        microstrain_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        microstrain_extract_u8(&deserializer, &frame_id);
        
        extract_mip_aiding_frame_config_command_format(&deserializer, &format);
        
        assert(tracking_enabled_out);
        microstrain_extract_bool(&deserializer, tracking_enabled_out);
        
        assert(translation_out);
        for(unsigned int i=0; i < 3; i++)
            microstrain_extract_float(&deserializer, &translation_out[i]);
        
        if( format == MIP_AIDING_FRAME_CONFIG_COMMAND_FORMAT_EULER )
        {
            extract_mip_vector3f(&deserializer, rotation_out->euler);
            
        }
        if( format == MIP_AIDING_FRAME_CONFIG_COMMAND_FORMAT_QUATERNION )
        {
            extract_mip_quatf(&deserializer, rotation_out->quaternion);
            
        }
        if( microstrain_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_aiding_save_frame_config(mip_interface* device, uint8_t frame_id)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    microstrain_insert_u8(&serializer, frame_id);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_AIDING_CMD_DESC_SET, MIP_CMD_DESC_AIDING_FRAME_CONFIG, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_aiding_load_frame_config(mip_interface* device, uint8_t frame_id)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    microstrain_insert_u8(&serializer, frame_id);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_AIDING_CMD_DESC_SET, MIP_CMD_DESC_AIDING_FRAME_CONFIG, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_aiding_default_frame_config(mip_interface* device, uint8_t frame_id)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    microstrain_insert_u8(&serializer, frame_id);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_AIDING_CMD_DESC_SET, MIP_CMD_DESC_AIDING_FRAME_CONFIG, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
void insert_mip_aiding_echo_control_command(microstrain_serializer* serializer, const mip_aiding_echo_control_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        insert_mip_aiding_echo_control_command_mode(serializer, self->mode);
        
    }
}
void extract_mip_aiding_echo_control_command(microstrain_serializer* serializer, mip_aiding_echo_control_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        extract_mip_aiding_echo_control_command_mode(serializer, &self->mode);
        
    }
}

void insert_mip_aiding_echo_control_response(microstrain_serializer* serializer, const mip_aiding_echo_control_response* self)
{
    insert_mip_aiding_echo_control_command_mode(serializer, self->mode);
    
}
void extract_mip_aiding_echo_control_response(microstrain_serializer* serializer, mip_aiding_echo_control_response* self)
{
    extract_mip_aiding_echo_control_command_mode(serializer, &self->mode);
    
}

mip_cmd_result mip_aiding_write_echo_control(mip_interface* device, mip_aiding_echo_control_command_mode mode)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    insert_mip_aiding_echo_control_command_mode(&serializer, mode);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_AIDING_CMD_DESC_SET, MIP_CMD_DESC_AIDING_ECHO_CONTROL, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_aiding_read_echo_control(mip_interface* device, mip_aiding_echo_control_command_mode* mode_out)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_AIDING_CMD_DESC_SET, MIP_CMD_DESC_AIDING_ECHO_CONTROL, buffer, (uint8_t)microstrain_serializer_length(&serializer), MIP_REPLY_DESC_AIDING_ECHO_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        microstrain_serializer deserializer;
        microstrain_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(mode_out);
        extract_mip_aiding_echo_control_command_mode(&deserializer, mode_out);
        
        if( microstrain_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_aiding_save_echo_control(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_AIDING_CMD_DESC_SET, MIP_CMD_DESC_AIDING_ECHO_CONTROL, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_aiding_load_echo_control(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_AIDING_CMD_DESC_SET, MIP_CMD_DESC_AIDING_ECHO_CONTROL, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_aiding_default_echo_control(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_AIDING_CMD_DESC_SET, MIP_CMD_DESC_AIDING_ECHO_CONTROL, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
void insert_mip_aiding_pos_ecef_command(microstrain_serializer* serializer, const mip_aiding_pos_ecef_command* self)
{
    insert_mip_time(serializer, &self->time);
    
    microstrain_insert_u8(serializer, self->frame_id);
    
    insert_mip_vector3d(serializer, self->position);
    
    insert_mip_vector3f(serializer, self->uncertainty);
    
    insert_mip_aiding_pos_ecef_command_valid_flags(serializer, self->valid_flags);
    
}
void extract_mip_aiding_pos_ecef_command(microstrain_serializer* serializer, mip_aiding_pos_ecef_command* self)
{
    extract_mip_time(serializer, &self->time);
    
    microstrain_extract_u8(serializer, &self->frame_id);
    
    extract_mip_vector3d(serializer, self->position);
    
    extract_mip_vector3f(serializer, self->uncertainty);
    
    extract_mip_aiding_pos_ecef_command_valid_flags(serializer, &self->valid_flags);
    
}

mip_cmd_result mip_aiding_pos_ecef(mip_interface* device, const mip_time* time, uint8_t frame_id, const double* position, const float* uncertainty, mip_aiding_pos_ecef_command_valid_flags valid_flags)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    assert(time);
    insert_mip_time(&serializer, time);
    
    microstrain_insert_u8(&serializer, frame_id);
    
    assert(position);
    for(unsigned int i=0; i < 3; i++)
        microstrain_insert_double(&serializer, position[i]);
    
    assert(uncertainty);
    for(unsigned int i=0; i < 3; i++)
        microstrain_insert_float(&serializer, uncertainty[i]);
    
    insert_mip_aiding_pos_ecef_command_valid_flags(&serializer, valid_flags);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_AIDING_CMD_DESC_SET, MIP_CMD_DESC_AIDING_POS_ECEF, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
void insert_mip_aiding_pos_llh_command(microstrain_serializer* serializer, const mip_aiding_pos_llh_command* self)
{
    insert_mip_time(serializer, &self->time);
    
    microstrain_insert_u8(serializer, self->frame_id);
    
    microstrain_insert_double(serializer, self->latitude);
    
    microstrain_insert_double(serializer, self->longitude);
    
    microstrain_insert_double(serializer, self->height);
    
    insert_mip_vector3f(serializer, self->uncertainty);
    
    insert_mip_aiding_pos_llh_command_valid_flags(serializer, self->valid_flags);
    
}
void extract_mip_aiding_pos_llh_command(microstrain_serializer* serializer, mip_aiding_pos_llh_command* self)
{
    extract_mip_time(serializer, &self->time);
    
    microstrain_extract_u8(serializer, &self->frame_id);
    
    microstrain_extract_double(serializer, &self->latitude);
    
    microstrain_extract_double(serializer, &self->longitude);
    
    microstrain_extract_double(serializer, &self->height);
    
    extract_mip_vector3f(serializer, self->uncertainty);
    
    extract_mip_aiding_pos_llh_command_valid_flags(serializer, &self->valid_flags);
    
}

mip_cmd_result mip_aiding_pos_llh(mip_interface* device, const mip_time* time, uint8_t frame_id, double latitude, double longitude, double height, const float* uncertainty, mip_aiding_pos_llh_command_valid_flags valid_flags)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    assert(time);
    insert_mip_time(&serializer, time);
    
    microstrain_insert_u8(&serializer, frame_id);
    
    microstrain_insert_double(&serializer, latitude);
    
    microstrain_insert_double(&serializer, longitude);
    
    microstrain_insert_double(&serializer, height);
    
    assert(uncertainty);
    for(unsigned int i=0; i < 3; i++)
        microstrain_insert_float(&serializer, uncertainty[i]);
    
    insert_mip_aiding_pos_llh_command_valid_flags(&serializer, valid_flags);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_AIDING_CMD_DESC_SET, MIP_CMD_DESC_AIDING_POS_LLH, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
void insert_mip_aiding_height_above_ellipsoid_command(microstrain_serializer* serializer, const mip_aiding_height_above_ellipsoid_command* self)
{
    insert_mip_time(serializer, &self->time);
    
    microstrain_insert_u8(serializer, self->frame_id);
    
    microstrain_insert_float(serializer, self->height);
    
    microstrain_insert_float(serializer, self->uncertainty);
    
    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_aiding_height_above_ellipsoid_command(microstrain_serializer* serializer, mip_aiding_height_above_ellipsoid_command* self)
{
    extract_mip_time(serializer, &self->time);
    
    microstrain_extract_u8(serializer, &self->frame_id);
    
    microstrain_extract_float(serializer, &self->height);
    
    microstrain_extract_float(serializer, &self->uncertainty);
    
    microstrain_extract_u16(serializer, &self->valid_flags);
    
}

mip_cmd_result mip_aiding_height_above_ellipsoid(mip_interface* device, const mip_time* time, uint8_t frame_id, float height, float uncertainty, uint16_t valid_flags)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    assert(time);
    insert_mip_time(&serializer, time);
    
    microstrain_insert_u8(&serializer, frame_id);
    
    microstrain_insert_float(&serializer, height);
    
    microstrain_insert_float(&serializer, uncertainty);
    
    microstrain_insert_u16(&serializer, valid_flags);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_AIDING_CMD_DESC_SET, MIP_CMD_DESC_AIDING_HEIGHT_ABOVE_ELLIPSOID, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
void insert_mip_aiding_vel_ecef_command(microstrain_serializer* serializer, const mip_aiding_vel_ecef_command* self)
{
    insert_mip_time(serializer, &self->time);
    
    microstrain_insert_u8(serializer, self->frame_id);
    
    insert_mip_vector3f(serializer, self->velocity);
    
    insert_mip_vector3f(serializer, self->uncertainty);
    
    insert_mip_aiding_vel_ecef_command_valid_flags(serializer, self->valid_flags);
    
}
void extract_mip_aiding_vel_ecef_command(microstrain_serializer* serializer, mip_aiding_vel_ecef_command* self)
{
    extract_mip_time(serializer, &self->time);
    
    microstrain_extract_u8(serializer, &self->frame_id);
    
    extract_mip_vector3f(serializer, self->velocity);
    
    extract_mip_vector3f(serializer, self->uncertainty);
    
    extract_mip_aiding_vel_ecef_command_valid_flags(serializer, &self->valid_flags);
    
}

mip_cmd_result mip_aiding_vel_ecef(mip_interface* device, const mip_time* time, uint8_t frame_id, const float* velocity, const float* uncertainty, mip_aiding_vel_ecef_command_valid_flags valid_flags)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    assert(time);
    insert_mip_time(&serializer, time);
    
    microstrain_insert_u8(&serializer, frame_id);
    
    assert(velocity);
    for(unsigned int i=0; i < 3; i++)
        microstrain_insert_float(&serializer, velocity[i]);
    
    assert(uncertainty);
    for(unsigned int i=0; i < 3; i++)
        microstrain_insert_float(&serializer, uncertainty[i]);
    
    insert_mip_aiding_vel_ecef_command_valid_flags(&serializer, valid_flags);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_AIDING_CMD_DESC_SET, MIP_CMD_DESC_AIDING_VEL_ECEF, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
void insert_mip_aiding_vel_ned_command(microstrain_serializer* serializer, const mip_aiding_vel_ned_command* self)
{
    insert_mip_time(serializer, &self->time);
    
    microstrain_insert_u8(serializer, self->frame_id);
    
    insert_mip_vector3f(serializer, self->velocity);
    
    insert_mip_vector3f(serializer, self->uncertainty);
    
    insert_mip_aiding_vel_ned_command_valid_flags(serializer, self->valid_flags);
    
}
void extract_mip_aiding_vel_ned_command(microstrain_serializer* serializer, mip_aiding_vel_ned_command* self)
{
    extract_mip_time(serializer, &self->time);
    
    microstrain_extract_u8(serializer, &self->frame_id);
    
    extract_mip_vector3f(serializer, self->velocity);
    
    extract_mip_vector3f(serializer, self->uncertainty);
    
    extract_mip_aiding_vel_ned_command_valid_flags(serializer, &self->valid_flags);
    
}

mip_cmd_result mip_aiding_vel_ned(mip_interface* device, const mip_time* time, uint8_t frame_id, const float* velocity, const float* uncertainty, mip_aiding_vel_ned_command_valid_flags valid_flags)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    assert(time);
    insert_mip_time(&serializer, time);
    
    microstrain_insert_u8(&serializer, frame_id);
    
    assert(velocity);
    for(unsigned int i=0; i < 3; i++)
        microstrain_insert_float(&serializer, velocity[i]);
    
    assert(uncertainty);
    for(unsigned int i=0; i < 3; i++)
        microstrain_insert_float(&serializer, uncertainty[i]);
    
    insert_mip_aiding_vel_ned_command_valid_flags(&serializer, valid_flags);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_AIDING_CMD_DESC_SET, MIP_CMD_DESC_AIDING_VEL_NED, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
void insert_mip_aiding_vel_body_frame_command(microstrain_serializer* serializer, const mip_aiding_vel_body_frame_command* self)
{
    insert_mip_time(serializer, &self->time);
    
    microstrain_insert_u8(serializer, self->frame_id);
    
    insert_mip_vector3f(serializer, self->velocity);
    
    insert_mip_vector3f(serializer, self->uncertainty);
    
    insert_mip_aiding_vel_body_frame_command_valid_flags(serializer, self->valid_flags);
    
}
void extract_mip_aiding_vel_body_frame_command(microstrain_serializer* serializer, mip_aiding_vel_body_frame_command* self)
{
    extract_mip_time(serializer, &self->time);
    
    microstrain_extract_u8(serializer, &self->frame_id);
    
    extract_mip_vector3f(serializer, self->velocity);
    
    extract_mip_vector3f(serializer, self->uncertainty);
    
    extract_mip_aiding_vel_body_frame_command_valid_flags(serializer, &self->valid_flags);
    
}

mip_cmd_result mip_aiding_vel_body_frame(mip_interface* device, const mip_time* time, uint8_t frame_id, const float* velocity, const float* uncertainty, mip_aiding_vel_body_frame_command_valid_flags valid_flags)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    assert(time);
    insert_mip_time(&serializer, time);
    
    microstrain_insert_u8(&serializer, frame_id);
    
    assert(velocity);
    for(unsigned int i=0; i < 3; i++)
        microstrain_insert_float(&serializer, velocity[i]);
    
    assert(uncertainty);
    for(unsigned int i=0; i < 3; i++)
        microstrain_insert_float(&serializer, uncertainty[i]);
    
    insert_mip_aiding_vel_body_frame_command_valid_flags(&serializer, valid_flags);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_AIDING_CMD_DESC_SET, MIP_CMD_DESC_AIDING_VEL_BODY_FRAME, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
void insert_mip_aiding_heading_true_command(microstrain_serializer* serializer, const mip_aiding_heading_true_command* self)
{
    insert_mip_time(serializer, &self->time);
    
    microstrain_insert_u8(serializer, self->frame_id);
    
    microstrain_insert_float(serializer, self->heading);
    
    microstrain_insert_float(serializer, self->uncertainty);
    
    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_aiding_heading_true_command(microstrain_serializer* serializer, mip_aiding_heading_true_command* self)
{
    extract_mip_time(serializer, &self->time);
    
    microstrain_extract_u8(serializer, &self->frame_id);
    
    microstrain_extract_float(serializer, &self->heading);
    
    microstrain_extract_float(serializer, &self->uncertainty);
    
    microstrain_extract_u16(serializer, &self->valid_flags);
    
}

mip_cmd_result mip_aiding_heading_true(mip_interface* device, const mip_time* time, uint8_t frame_id, float heading, float uncertainty, uint16_t valid_flags)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    assert(time);
    insert_mip_time(&serializer, time);
    
    microstrain_insert_u8(&serializer, frame_id);
    
    microstrain_insert_float(&serializer, heading);
    
    microstrain_insert_float(&serializer, uncertainty);
    
    microstrain_insert_u16(&serializer, valid_flags);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_AIDING_CMD_DESC_SET, MIP_CMD_DESC_AIDING_HEADING_TRUE, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
void insert_mip_aiding_magnetic_field_command(microstrain_serializer* serializer, const mip_aiding_magnetic_field_command* self)
{
    insert_mip_time(serializer, &self->time);
    
    microstrain_insert_u8(serializer, self->frame_id);
    
    insert_mip_vector3f(serializer, self->magnetic_field);
    
    insert_mip_vector3f(serializer, self->uncertainty);
    
    insert_mip_aiding_magnetic_field_command_valid_flags(serializer, self->valid_flags);
    
}
void extract_mip_aiding_magnetic_field_command(microstrain_serializer* serializer, mip_aiding_magnetic_field_command* self)
{
    extract_mip_time(serializer, &self->time);
    
    microstrain_extract_u8(serializer, &self->frame_id);
    
    extract_mip_vector3f(serializer, self->magnetic_field);
    
    extract_mip_vector3f(serializer, self->uncertainty);
    
    extract_mip_aiding_magnetic_field_command_valid_flags(serializer, &self->valid_flags);
    
}

mip_cmd_result mip_aiding_magnetic_field(mip_interface* device, const mip_time* time, uint8_t frame_id, const float* magnetic_field, const float* uncertainty, mip_aiding_magnetic_field_command_valid_flags valid_flags)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    assert(time);
    insert_mip_time(&serializer, time);
    
    microstrain_insert_u8(&serializer, frame_id);
    
    assert(magnetic_field);
    for(unsigned int i=0; i < 3; i++)
        microstrain_insert_float(&serializer, magnetic_field[i]);
    
    assert(uncertainty);
    for(unsigned int i=0; i < 3; i++)
        microstrain_insert_float(&serializer, uncertainty[i]);
    
    insert_mip_aiding_magnetic_field_command_valid_flags(&serializer, valid_flags);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_AIDING_CMD_DESC_SET, MIP_CMD_DESC_AIDING_MAGNETIC_FIELD, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
void insert_mip_aiding_pressure_command(microstrain_serializer* serializer, const mip_aiding_pressure_command* self)
{
    insert_mip_time(serializer, &self->time);
    
    microstrain_insert_u8(serializer, self->frame_id);
    
    microstrain_insert_float(serializer, self->pressure);
    
    microstrain_insert_float(serializer, self->uncertainty);
    
    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_aiding_pressure_command(microstrain_serializer* serializer, mip_aiding_pressure_command* self)
{
    extract_mip_time(serializer, &self->time);
    
    microstrain_extract_u8(serializer, &self->frame_id);
    
    microstrain_extract_float(serializer, &self->pressure);
    
    microstrain_extract_float(serializer, &self->uncertainty);
    
    microstrain_extract_u16(serializer, &self->valid_flags);
    
}

mip_cmd_result mip_aiding_pressure(mip_interface* device, const mip_time* time, uint8_t frame_id, float pressure, float uncertainty, uint16_t valid_flags)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    assert(time);
    insert_mip_time(&serializer, time);
    
    microstrain_insert_u8(&serializer, frame_id);
    
    microstrain_insert_float(&serializer, pressure);
    
    microstrain_insert_float(&serializer, uncertainty);
    
    microstrain_insert_u16(&serializer, valid_flags);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_AIDING_CMD_DESC_SET, MIP_CMD_DESC_AIDING_PRESSURE, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}

#ifdef __cplusplus
} // extern "C"
} // namespace C
} // namespace mip
#endif // __cplusplus

