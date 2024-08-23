
#include "commands_base.h"

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

void insert_mip_base_device_info(microstrain_serializer* serializer, const mip_base_device_info* self)
{
    microstrain_insert_u16(serializer, self->firmware_version);
    
    for(unsigned int i=0; i < 16; i++)
        microstrain_insert_char(serializer, self->model_name[i]);
    
    for(unsigned int i=0; i < 16; i++)
        microstrain_insert_char(serializer, self->model_number[i]);
    
    for(unsigned int i=0; i < 16; i++)
        microstrain_insert_char(serializer, self->serial_number[i]);
    
    for(unsigned int i=0; i < 16; i++)
        microstrain_insert_char(serializer, self->lot_number[i]);
    
    for(unsigned int i=0; i < 16; i++)
        microstrain_insert_char(serializer, self->device_options[i]);
    
}
void extract_mip_base_device_info(microstrain_serializer* serializer, mip_base_device_info* self)
{
    microstrain_extract_u16(serializer, &self->firmware_version);
    
    for(unsigned int i=0; i < 16; i++)
        microstrain_extract_char(serializer, &self->model_name[i]);
    
    for(unsigned int i=0; i < 16; i++)
        microstrain_extract_char(serializer, &self->model_number[i]);
    
    for(unsigned int i=0; i < 16; i++)
        microstrain_extract_char(serializer, &self->serial_number[i]);
    
    for(unsigned int i=0; i < 16; i++)
        microstrain_extract_char(serializer, &self->lot_number[i]);
    
    for(unsigned int i=0; i < 16; i++)
        microstrain_extract_char(serializer, &self->device_options[i]);
    
}

mip_cmd_result mip_base_ping(mip_interface* device)
{
    return mip_interface_run_command(device, MIP_BASE_CMD_DESC_SET, MIP_CMD_DESC_BASE_PING, NULL, 0);
}
mip_cmd_result mip_base_set_idle(mip_interface* device)
{
    return mip_interface_run_command(device, MIP_BASE_CMD_DESC_SET, MIP_CMD_DESC_BASE_SET_TO_IDLE, NULL, 0);
}
mip_cmd_result mip_base_get_device_info(mip_interface* device, mip_base_device_info* device_info_out)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_BASE_CMD_DESC_SET, MIP_CMD_DESC_BASE_GET_DEVICE_INFO, NULL, 0, MIP_REPLY_DESC_BASE_DEVICE_INFO, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        microstrain_serializer deserializer;
        microstrain_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(device_info_out);
        extract_mip_base_device_info(&deserializer, device_info_out);
        
        if( microstrain_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_base_get_device_descriptors(mip_interface* device, uint16_t* descriptors_out, size_t descriptors_out_max, uint8_t* descriptors_out_count)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_BASE_CMD_DESC_SET, MIP_CMD_DESC_BASE_GET_DEVICE_DESCRIPTORS, NULL, 0, MIP_REPLY_DESC_BASE_DEVICE_DESCRIPTORS, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        microstrain_serializer deserializer;
        microstrain_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        for(*descriptors_out_count = 0; (*descriptors_out_count < descriptors_out_max) && (microstrain_serializer_remaining(&deserializer) > 0); (*descriptors_out_count)++)
            microstrain_extract_u16(&deserializer, &descriptors_out[*descriptors_out_count]);
        
        if( microstrain_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_base_built_in_test(mip_interface* device, uint32_t* result_out)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_BASE_CMD_DESC_SET, MIP_CMD_DESC_BASE_BUILT_IN_TEST, NULL, 0, MIP_REPLY_DESC_BASE_BUILT_IN_TEST, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        microstrain_serializer deserializer;
        microstrain_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(result_out);
        microstrain_extract_u32(&deserializer, result_out);
        
        if( microstrain_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_base_resume(mip_interface* device)
{
    return mip_interface_run_command(device, MIP_BASE_CMD_DESC_SET, MIP_CMD_DESC_BASE_RESUME, NULL, 0);
}
mip_cmd_result mip_base_get_extended_descriptors(mip_interface* device, uint16_t* descriptors_out, size_t descriptors_out_max, uint8_t* descriptors_out_count)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_BASE_CMD_DESC_SET, MIP_CMD_DESC_BASE_GET_EXTENDED_DESCRIPTORS, NULL, 0, MIP_REPLY_DESC_BASE_GET_EXTENDED_DESCRIPTORS, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        microstrain_serializer deserializer;
        microstrain_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        for(*descriptors_out_count = 0; (*descriptors_out_count < descriptors_out_max) && (microstrain_serializer_remaining(&deserializer) > 0); (*descriptors_out_count)++)
            microstrain_extract_u16(&deserializer, &descriptors_out[*descriptors_out_count]);
        
        if( microstrain_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_base_continuous_bit(mip_interface* device, uint8_t* result_out)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_BASE_CMD_DESC_SET, MIP_CMD_DESC_BASE_CONTINUOUS_BIT, NULL, 0, MIP_REPLY_DESC_BASE_CONTINUOUS_BIT, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        microstrain_serializer deserializer;
        microstrain_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(result_out);
        for(unsigned int i=0; i < 16; i++)
            microstrain_extract_u8(&deserializer, &result_out[i]);
        
        if( microstrain_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
void insert_mip_base_comm_speed_command(microstrain_serializer* serializer, const mip_base_comm_speed_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    microstrain_insert_u8(serializer, self->port);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        microstrain_insert_u32(serializer, self->baud);
        
    }
}
void extract_mip_base_comm_speed_command(microstrain_serializer* serializer, mip_base_comm_speed_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    microstrain_extract_u8(serializer, &self->port);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        microstrain_extract_u32(serializer, &self->baud);
        
    }
}

void insert_mip_base_comm_speed_response(microstrain_serializer* serializer, const mip_base_comm_speed_response* self)
{
    microstrain_insert_u8(serializer, self->port);
    
    microstrain_insert_u32(serializer, self->baud);
    
}
void extract_mip_base_comm_speed_response(microstrain_serializer* serializer, mip_base_comm_speed_response* self)
{
    microstrain_extract_u8(serializer, &self->port);
    
    microstrain_extract_u32(serializer, &self->baud);
    
}

mip_cmd_result mip_base_write_comm_speed(mip_interface* device, uint8_t port, uint32_t baud)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    microstrain_insert_u8(&serializer, port);
    
    microstrain_insert_u32(&serializer, baud);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_BASE_CMD_DESC_SET, MIP_CMD_DESC_BASE_COMM_SPEED, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_base_read_comm_speed(mip_interface* device, uint8_t port, uint32_t* baud_out)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    microstrain_insert_u8(&serializer, port);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_BASE_CMD_DESC_SET, MIP_CMD_DESC_BASE_COMM_SPEED, buffer, (uint8_t)microstrain_serializer_length(&serializer), MIP_REPLY_DESC_BASE_COMM_SPEED, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        microstrain_serializer deserializer;
        microstrain_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        microstrain_extract_u8(&deserializer, &port);
        
        assert(baud_out);
        microstrain_extract_u32(&deserializer, baud_out);
        
        if( microstrain_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_base_save_comm_speed(mip_interface* device, uint8_t port)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    microstrain_insert_u8(&serializer, port);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_BASE_CMD_DESC_SET, MIP_CMD_DESC_BASE_COMM_SPEED, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_base_load_comm_speed(mip_interface* device, uint8_t port)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    microstrain_insert_u8(&serializer, port);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_BASE_CMD_DESC_SET, MIP_CMD_DESC_BASE_COMM_SPEED, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_base_default_comm_speed(mip_interface* device, uint8_t port)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    microstrain_insert_u8(&serializer, port);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_BASE_CMD_DESC_SET, MIP_CMD_DESC_BASE_COMM_SPEED, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
void insert_mip_base_gps_time_update_command(microstrain_serializer* serializer, const mip_base_gps_time_update_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        insert_mip_base_gps_time_update_command_field_id(serializer, self->field_id);
        
        microstrain_insert_u32(serializer, self->value);
        
    }
}
void extract_mip_base_gps_time_update_command(microstrain_serializer* serializer, mip_base_gps_time_update_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        extract_mip_base_gps_time_update_command_field_id(serializer, &self->field_id);
        
        microstrain_extract_u32(serializer, &self->value);
        
    }
}

mip_cmd_result mip_base_write_gps_time_update(mip_interface* device, mip_base_gps_time_update_command_field_id field_id, uint32_t value)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    insert_mip_base_gps_time_update_command_field_id(&serializer, field_id);
    
    microstrain_insert_u32(&serializer, value);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_BASE_CMD_DESC_SET, MIP_CMD_DESC_BASE_GPS_TIME_UPDATE, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_base_soft_reset(mip_interface* device)
{
    return mip_interface_run_command(device, MIP_BASE_CMD_DESC_SET, MIP_CMD_DESC_BASE_SOFT_RESET, NULL, 0);
}

#ifdef __cplusplus
} // extern "C"
} // namespace C
} // namespace mip
#endif // __cplusplus

