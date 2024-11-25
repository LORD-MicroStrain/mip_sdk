
#include "commands_gnss.h"

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

void insert_mip_gnss_receiver_info_command_info(microstrain_serializer* serializer, const mip_gnss_receiver_info_command_info* self)
{
    microstrain_insert_u8(serializer, self->receiver_id);
    
    microstrain_insert_u8(serializer, self->mip_data_descriptor_set);
    
    for(unsigned int i=0; i < 32; i++)
        microstrain_insert_char(serializer, self->description[i]);
    
}
void extract_mip_gnss_receiver_info_command_info(microstrain_serializer* serializer, mip_gnss_receiver_info_command_info* self)
{
    microstrain_extract_u8(serializer, &self->receiver_id);
    
    microstrain_extract_u8(serializer, &self->mip_data_descriptor_set);
    
    for(unsigned int i=0; i < 32; i++)
        microstrain_extract_char(serializer, &self->description[i]);
    
}

mip_cmd_result mip_gnss_receiver_info(mip_interface* device, uint8_t* num_receivers_out, uint8_t num_receivers_out_max, mip_gnss_receiver_info_command_info* receiver_info_out)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_GNSS_CMD_DESC_SET, MIP_CMD_DESC_GNSS_LIST_RECEIVERS, NULL, 0, MIP_REPLY_DESC_GNSS_LIST_RECEIVERS, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        microstrain_serializer deserializer;
        microstrain_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(num_receivers_out);
        microstrain_extract_count(&deserializer, num_receivers_out, num_receivers_out_max);
        
        assert(receiver_info_out || (num_receivers_out == 0));
        for(unsigned int i=0; i < *num_receivers_out; i++)
            extract_mip_gnss_receiver_info_command_info(&deserializer, &receiver_info_out[i]);
        
        if( microstrain_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
void insert_mip_gnss_signal_configuration_command(microstrain_serializer* serializer, const mip_gnss_signal_configuration_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        microstrain_insert_u8(serializer, self->gps_enable);
        
        microstrain_insert_u8(serializer, self->glonass_enable);
        
        microstrain_insert_u8(serializer, self->galileo_enable);
        
        microstrain_insert_u8(serializer, self->beidou_enable);
        
        for(unsigned int i=0; i < 4; i++)
            microstrain_insert_u8(serializer, self->reserved[i]);
        
    }
}
void extract_mip_gnss_signal_configuration_command(microstrain_serializer* serializer, mip_gnss_signal_configuration_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        microstrain_extract_u8(serializer, &self->gps_enable);
        
        microstrain_extract_u8(serializer, &self->glonass_enable);
        
        microstrain_extract_u8(serializer, &self->galileo_enable);
        
        microstrain_extract_u8(serializer, &self->beidou_enable);
        
        for(unsigned int i=0; i < 4; i++)
            microstrain_extract_u8(serializer, &self->reserved[i]);
        
    }
}

void insert_mip_gnss_signal_configuration_response(microstrain_serializer* serializer, const mip_gnss_signal_configuration_response* self)
{
    microstrain_insert_u8(serializer, self->gps_enable);
    
    microstrain_insert_u8(serializer, self->glonass_enable);
    
    microstrain_insert_u8(serializer, self->galileo_enable);
    
    microstrain_insert_u8(serializer, self->beidou_enable);
    
    for(unsigned int i=0; i < 4; i++)
        microstrain_insert_u8(serializer, self->reserved[i]);
    
}
void extract_mip_gnss_signal_configuration_response(microstrain_serializer* serializer, mip_gnss_signal_configuration_response* self)
{
    microstrain_extract_u8(serializer, &self->gps_enable);
    
    microstrain_extract_u8(serializer, &self->glonass_enable);
    
    microstrain_extract_u8(serializer, &self->galileo_enable);
    
    microstrain_extract_u8(serializer, &self->beidou_enable);
    
    for(unsigned int i=0; i < 4; i++)
        microstrain_extract_u8(serializer, &self->reserved[i]);
    
}

mip_cmd_result mip_gnss_write_signal_configuration(mip_interface* device, uint8_t gps_enable, uint8_t glonass_enable, uint8_t galileo_enable, uint8_t beidou_enable, const uint8_t* reserved)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    microstrain_insert_u8(&serializer, gps_enable);
    
    microstrain_insert_u8(&serializer, glonass_enable);
    
    microstrain_insert_u8(&serializer, galileo_enable);
    
    microstrain_insert_u8(&serializer, beidou_enable);
    
    assert(reserved);
    for(unsigned int i=0; i < 4; i++)
        microstrain_insert_u8(&serializer, reserved[i]);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_GNSS_CMD_DESC_SET, MIP_CMD_DESC_GNSS_SIGNAL_CONFIGURATION, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_gnss_read_signal_configuration(mip_interface* device, uint8_t* gps_enable_out, uint8_t* glonass_enable_out, uint8_t* galileo_enable_out, uint8_t* beidou_enable_out, uint8_t* reserved_out)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_GNSS_CMD_DESC_SET, MIP_CMD_DESC_GNSS_SIGNAL_CONFIGURATION, buffer, (uint8_t)microstrain_serializer_length(&serializer), MIP_REPLY_DESC_GNSS_SIGNAL_CONFIGURATION, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        microstrain_serializer deserializer;
        microstrain_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(gps_enable_out);
        microstrain_extract_u8(&deserializer, gps_enable_out);
        
        assert(glonass_enable_out);
        microstrain_extract_u8(&deserializer, glonass_enable_out);
        
        assert(galileo_enable_out);
        microstrain_extract_u8(&deserializer, galileo_enable_out);
        
        assert(beidou_enable_out);
        microstrain_extract_u8(&deserializer, beidou_enable_out);
        
        assert(reserved_out);
        for(unsigned int i=0; i < 4; i++)
            microstrain_extract_u8(&deserializer, &reserved_out[i]);
        
        if( microstrain_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_gnss_save_signal_configuration(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_GNSS_CMD_DESC_SET, MIP_CMD_DESC_GNSS_SIGNAL_CONFIGURATION, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_gnss_load_signal_configuration(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_GNSS_CMD_DESC_SET, MIP_CMD_DESC_GNSS_SIGNAL_CONFIGURATION, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_gnss_default_signal_configuration(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_GNSS_CMD_DESC_SET, MIP_CMD_DESC_GNSS_SIGNAL_CONFIGURATION, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
void insert_mip_gnss_spartn_configuration_command(microstrain_serializer* serializer, const mip_gnss_spartn_configuration_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        microstrain_insert_u8(serializer, self->enable);
        
        microstrain_insert_u8(serializer, self->type);
        
        microstrain_insert_u32(serializer, self->current_key_tow);
        
        microstrain_insert_u16(serializer, self->current_key_week);
        
        for(unsigned int i=0; i < 32; i++)
            microstrain_insert_u8(serializer, self->current_key[i]);
        
        microstrain_insert_u32(serializer, self->next_key_tow);
        
        microstrain_insert_u16(serializer, self->next_key_week);
        
        for(unsigned int i=0; i < 32; i++)
            microstrain_insert_u8(serializer, self->next_key[i]);
        
    }
}
void extract_mip_gnss_spartn_configuration_command(microstrain_serializer* serializer, mip_gnss_spartn_configuration_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        microstrain_extract_u8(serializer, &self->enable);
        
        microstrain_extract_u8(serializer, &self->type);
        
        microstrain_extract_u32(serializer, &self->current_key_tow);
        
        microstrain_extract_u16(serializer, &self->current_key_week);
        
        for(unsigned int i=0; i < 32; i++)
            microstrain_extract_u8(serializer, &self->current_key[i]);
        
        microstrain_extract_u32(serializer, &self->next_key_tow);
        
        microstrain_extract_u16(serializer, &self->next_key_week);
        
        for(unsigned int i=0; i < 32; i++)
            microstrain_extract_u8(serializer, &self->next_key[i]);
        
    }
}

void insert_mip_gnss_spartn_configuration_response(microstrain_serializer* serializer, const mip_gnss_spartn_configuration_response* self)
{
    microstrain_insert_u8(serializer, self->enable);
    
    microstrain_insert_u8(serializer, self->type);
    
    microstrain_insert_u32(serializer, self->current_key_tow);
    
    microstrain_insert_u16(serializer, self->current_key_week);
    
    for(unsigned int i=0; i < 32; i++)
        microstrain_insert_u8(serializer, self->current_key[i]);
    
    microstrain_insert_u32(serializer, self->next_key_tow);
    
    microstrain_insert_u16(serializer, self->next_key_week);
    
    for(unsigned int i=0; i < 32; i++)
        microstrain_insert_u8(serializer, self->next_key[i]);
    
}
void extract_mip_gnss_spartn_configuration_response(microstrain_serializer* serializer, mip_gnss_spartn_configuration_response* self)
{
    microstrain_extract_u8(serializer, &self->enable);
    
    microstrain_extract_u8(serializer, &self->type);
    
    microstrain_extract_u32(serializer, &self->current_key_tow);
    
    microstrain_extract_u16(serializer, &self->current_key_week);
    
    for(unsigned int i=0; i < 32; i++)
        microstrain_extract_u8(serializer, &self->current_key[i]);
    
    microstrain_extract_u32(serializer, &self->next_key_tow);
    
    microstrain_extract_u16(serializer, &self->next_key_week);
    
    for(unsigned int i=0; i < 32; i++)
        microstrain_extract_u8(serializer, &self->next_key[i]);
    
}

mip_cmd_result mip_gnss_write_spartn_configuration(mip_interface* device, uint8_t enable, uint8_t type, uint32_t current_key_tow, uint16_t current_key_week, const uint8_t* current_key, uint32_t next_key_tow, uint16_t next_key_week, const uint8_t* next_key)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    microstrain_insert_u8(&serializer, enable);
    
    microstrain_insert_u8(&serializer, type);
    
    microstrain_insert_u32(&serializer, current_key_tow);
    
    microstrain_insert_u16(&serializer, current_key_week);
    
    assert(current_key);
    for(unsigned int i=0; i < 32; i++)
        microstrain_insert_u8(&serializer, current_key[i]);
    
    microstrain_insert_u32(&serializer, next_key_tow);
    
    microstrain_insert_u16(&serializer, next_key_week);
    
    assert(next_key);
    for(unsigned int i=0; i < 32; i++)
        microstrain_insert_u8(&serializer, next_key[i]);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_GNSS_CMD_DESC_SET, MIP_CMD_DESC_GNSS_SPARTN_CONFIGURATION, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_gnss_read_spartn_configuration(mip_interface* device, uint8_t* enable_out, uint8_t* type_out, uint32_t* current_key_tow_out, uint16_t* current_key_week_out, uint8_t* current_key_out, uint32_t* next_key_tow_out, uint16_t* next_key_week_out, uint8_t* next_key_out)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_GNSS_CMD_DESC_SET, MIP_CMD_DESC_GNSS_SPARTN_CONFIGURATION, buffer, (uint8_t)microstrain_serializer_length(&serializer), MIP_REPLY_DESC_GNSS_SPARTN_CONFIGURATION, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        microstrain_serializer deserializer;
        microstrain_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(enable_out);
        microstrain_extract_u8(&deserializer, enable_out);
        
        assert(type_out);
        microstrain_extract_u8(&deserializer, type_out);
        
        assert(current_key_tow_out);
        microstrain_extract_u32(&deserializer, current_key_tow_out);
        
        assert(current_key_week_out);
        microstrain_extract_u16(&deserializer, current_key_week_out);
        
        assert(current_key_out);
        for(unsigned int i=0; i < 32; i++)
            microstrain_extract_u8(&deserializer, &current_key_out[i]);
        
        assert(next_key_tow_out);
        microstrain_extract_u32(&deserializer, next_key_tow_out);
        
        assert(next_key_week_out);
        microstrain_extract_u16(&deserializer, next_key_week_out);
        
        assert(next_key_out);
        for(unsigned int i=0; i < 32; i++)
            microstrain_extract_u8(&deserializer, &next_key_out[i]);
        
        if( microstrain_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_gnss_save_spartn_configuration(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_GNSS_CMD_DESC_SET, MIP_CMD_DESC_GNSS_SPARTN_CONFIGURATION, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_gnss_load_spartn_configuration(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_GNSS_CMD_DESC_SET, MIP_CMD_DESC_GNSS_SPARTN_CONFIGURATION, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_gnss_default_spartn_configuration(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_GNSS_CMD_DESC_SET, MIP_CMD_DESC_GNSS_SPARTN_CONFIGURATION, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
void insert_mip_gnss_rtk_dongle_configuration_command(microstrain_serializer* serializer, const mip_gnss_rtk_dongle_configuration_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        microstrain_insert_u8(serializer, self->enable);
        
        for(unsigned int i=0; i < 3; i++)
            microstrain_insert_u8(serializer, self->reserved[i]);
        
    }
}
void extract_mip_gnss_rtk_dongle_configuration_command(microstrain_serializer* serializer, mip_gnss_rtk_dongle_configuration_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        microstrain_extract_u8(serializer, &self->enable);
        
        for(unsigned int i=0; i < 3; i++)
            microstrain_extract_u8(serializer, &self->reserved[i]);
        
    }
}

void insert_mip_gnss_rtk_dongle_configuration_response(microstrain_serializer* serializer, const mip_gnss_rtk_dongle_configuration_response* self)
{
    microstrain_insert_u8(serializer, self->enable);
    
    for(unsigned int i=0; i < 3; i++)
        microstrain_insert_u8(serializer, self->reserved[i]);
    
}
void extract_mip_gnss_rtk_dongle_configuration_response(microstrain_serializer* serializer, mip_gnss_rtk_dongle_configuration_response* self)
{
    microstrain_extract_u8(serializer, &self->enable);
    
    for(unsigned int i=0; i < 3; i++)
        microstrain_extract_u8(serializer, &self->reserved[i]);
    
}

mip_cmd_result mip_gnss_write_rtk_dongle_configuration(mip_interface* device, uint8_t enable, const uint8_t* reserved)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    microstrain_insert_u8(&serializer, enable);
    
    assert(reserved);
    for(unsigned int i=0; i < 3; i++)
        microstrain_insert_u8(&serializer, reserved[i]);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_GNSS_CMD_DESC_SET, MIP_CMD_DESC_GNSS_RTK_DONGLE_CONFIGURATION, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_gnss_read_rtk_dongle_configuration(mip_interface* device, uint8_t* enable_out, uint8_t* reserved_out)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_GNSS_CMD_DESC_SET, MIP_CMD_DESC_GNSS_RTK_DONGLE_CONFIGURATION, buffer, (uint8_t)microstrain_serializer_length(&serializer), MIP_REPLY_DESC_GNSS_RTK_DONGLE_CONFIGURATION, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        microstrain_serializer deserializer;
        microstrain_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(enable_out);
        microstrain_extract_u8(&deserializer, enable_out);
        
        assert(reserved_out);
        for(unsigned int i=0; i < 3; i++)
            microstrain_extract_u8(&deserializer, &reserved_out[i]);
        
        if( microstrain_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_gnss_save_rtk_dongle_configuration(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_GNSS_CMD_DESC_SET, MIP_CMD_DESC_GNSS_RTK_DONGLE_CONFIGURATION, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_gnss_load_rtk_dongle_configuration(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_GNSS_CMD_DESC_SET, MIP_CMD_DESC_GNSS_RTK_DONGLE_CONFIGURATION, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_gnss_default_rtk_dongle_configuration(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_GNSS_CMD_DESC_SET, MIP_CMD_DESC_GNSS_RTK_DONGLE_CONFIGURATION, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}

#ifdef __cplusplus
} // extern "C"
} // namespace C
} // namespace mip
#endif // __cplusplus

