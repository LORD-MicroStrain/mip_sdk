
#include "commands_system.h"

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

void insert_mip_system_comm_mode_command(microstrain_serializer* serializer, const mip_system_comm_mode_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        microstrain_insert_u8(serializer, self->mode);
        
    }
}
void extract_mip_system_comm_mode_command(microstrain_serializer* serializer, mip_system_comm_mode_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        microstrain_extract_u8(serializer, &self->mode);
        
    }
}

void insert_mip_system_comm_mode_response(microstrain_serializer* serializer, const mip_system_comm_mode_response* self)
{
    microstrain_insert_u8(serializer, self->mode);
    
}
void extract_mip_system_comm_mode_response(microstrain_serializer* serializer, mip_system_comm_mode_response* self)
{
    microstrain_extract_u8(serializer, &self->mode);
    
}

mip_cmd_result mip_system_write_comm_mode(mip_interface* device, uint8_t mode)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    microstrain_insert_u8(&serializer, mode);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_SYSTEM_CMD_DESC_SET, MIP_CMD_DESC_SYSTEM_COM_MODE, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_system_read_comm_mode(mip_interface* device, uint8_t* mode_out)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_SYSTEM_CMD_DESC_SET, MIP_CMD_DESC_SYSTEM_COM_MODE, buffer, (uint8_t)microstrain_serializer_length(&serializer), MIP_REPLY_DESC_SYSTEM_COM_MODE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        microstrain_serializer deserializer;
        microstrain_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(mode_out);
        microstrain_extract_u8(&deserializer, mode_out);
        
        if( microstrain_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_system_default_comm_mode(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_SYSTEM_CMD_DESC_SET, MIP_CMD_DESC_SYSTEM_COM_MODE, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
void insert_mip_system_interface_control_command(microstrain_serializer* serializer, const mip_system_interface_control_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    insert_mip_comms_interface(serializer, self->interface);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        insert_mip_comms_protocol(serializer, self->protocols_in);
        
        insert_mip_comms_protocol(serializer, self->protocols_out);
        
    }
}
void extract_mip_system_interface_control_command(microstrain_serializer* serializer, mip_system_interface_control_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    extract_mip_comms_interface(serializer, &self->interface);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        extract_mip_comms_protocol(serializer, &self->protocols_in);
        
        extract_mip_comms_protocol(serializer, &self->protocols_out);
        
    }
}

void insert_mip_system_interface_control_response(microstrain_serializer* serializer, const mip_system_interface_control_response* self)
{
    insert_mip_comms_interface(serializer, self->interface);
    
    insert_mip_comms_protocol(serializer, self->protocols_in);
    
    insert_mip_comms_protocol(serializer, self->protocols_out);
    
}
void extract_mip_system_interface_control_response(microstrain_serializer* serializer, mip_system_interface_control_response* self)
{
    extract_mip_comms_interface(serializer, &self->interface);
    
    extract_mip_comms_protocol(serializer, &self->protocols_in);
    
    extract_mip_comms_protocol(serializer, &self->protocols_out);
    
}

mip_cmd_result mip_system_write_interface_control(mip_interface* device, mip_comms_interface interface, mip_comms_protocol protocols_in, mip_comms_protocol protocols_out)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    insert_mip_comms_interface(&serializer, interface);
    
    insert_mip_comms_protocol(&serializer, protocols_in);
    
    insert_mip_comms_protocol(&serializer, protocols_out);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_SYSTEM_CMD_DESC_SET, MIP_CMD_DESC_SYSTEM_INTERFACE_CONTROL, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_system_read_interface_control(mip_interface* device, mip_comms_interface interface, mip_comms_protocol* protocols_in_out, mip_comms_protocol* protocols_out_out)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    insert_mip_comms_interface(&serializer, interface);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_SYSTEM_CMD_DESC_SET, MIP_CMD_DESC_SYSTEM_INTERFACE_CONTROL, buffer, (uint8_t)microstrain_serializer_length(&serializer), MIP_REPLY_DESC_SYSTEM_INTERFACE_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        microstrain_serializer deserializer;
        microstrain_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        extract_mip_comms_interface(&deserializer, &interface);
        
        assert(protocols_in_out);
        extract_mip_comms_protocol(&deserializer, protocols_in_out);
        
        assert(protocols_out_out);
        extract_mip_comms_protocol(&deserializer, protocols_out_out);
        
        if( microstrain_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_system_save_interface_control(mip_interface* device, mip_comms_interface interface)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    insert_mip_comms_interface(&serializer, interface);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_SYSTEM_CMD_DESC_SET, MIP_CMD_DESC_SYSTEM_INTERFACE_CONTROL, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_system_load_interface_control(mip_interface* device, mip_comms_interface interface)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    insert_mip_comms_interface(&serializer, interface);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_SYSTEM_CMD_DESC_SET, MIP_CMD_DESC_SYSTEM_INTERFACE_CONTROL, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_system_default_interface_control(mip_interface* device, mip_comms_interface interface)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    insert_mip_comms_interface(&serializer, interface);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_SYSTEM_CMD_DESC_SET, MIP_CMD_DESC_SYSTEM_INTERFACE_CONTROL, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}

#ifdef __cplusplus
} // extern "C"
} // namespace C
} // namespace mip
#endif // __cplusplus

