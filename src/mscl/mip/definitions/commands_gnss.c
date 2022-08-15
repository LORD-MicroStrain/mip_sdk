
#include "commands_gnss.h"

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


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

void insert_mip_gnss_receiver_info_command_info(struct mip_serializer* serializer, const struct mip_gnss_receiver_info_command_info* self)
{
    insert_u8(serializer, self->receiver_id);
    
    insert_u8(serializer, self->mip_data_descriptor_set);
    
    for(unsigned int i=0; i < 32; i++)
        insert_char(serializer, self->description[i]);
    
}
void extract_mip_gnss_receiver_info_command_info(struct mip_serializer* serializer, struct mip_gnss_receiver_info_command_info* self)
{
    extract_u8(serializer, &self->receiver_id);
    
    extract_u8(serializer, &self->mip_data_descriptor_set);
    
    for(unsigned int i=0; i < 32; i++)
        extract_char(serializer, &self->description[i]);
    
}

enum mip_cmd_result mip_gnss_receiver_info(struct mip_interface* device, uint8_t* num_receivers_out, uint8_t num_receivers_out_max, struct mip_gnss_receiver_info_command_info* receiver_info_out)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    enum mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_GNSS_CMD_DESC_SET, MIP_CMD_DESC_GNSS_LIST_RECEIVERS, NULL, 0, MIP_REPLY_DESC_GNSS_LIST_RECEIVERS, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        struct mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, sizeof(buffer));
        
        assert(num_receivers_out);
        extract_count(&deserializer, num_receivers_out, num_receivers_out_max);
        
        assert(receiver_info_out);
        assert(num_receivers_out);
        for(unsigned int i=0; i < *num_receivers_out; i++)
            extract_mip_gnss_receiver_info_command_info(&deserializer, &receiver_info_out[i]);
        
        if( !mip_serializer_ok(&deserializer) )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
void insert_mip_gnss_signal_configuration_command(struct mip_serializer* serializer, const struct mip_gnss_signal_configuration_command* self)
{
    insert_u8(serializer, self->gps_enable);
    
    insert_u8(serializer, self->glonass_enable);
    
    insert_u8(serializer, self->galileo_enable);
    
    insert_u8(serializer, self->beidou_enable);
    
    for(unsigned int i=0; i < 4; i++)
        insert_u8(serializer, self->reserved[i]);
    
}
void extract_mip_gnss_signal_configuration_command(struct mip_serializer* serializer, struct mip_gnss_signal_configuration_command* self)
{
    extract_u8(serializer, &self->gps_enable);
    
    extract_u8(serializer, &self->glonass_enable);
    
    extract_u8(serializer, &self->galileo_enable);
    
    extract_u8(serializer, &self->beidou_enable);
    
    for(unsigned int i=0; i < 4; i++)
        extract_u8(serializer, &self->reserved[i]);
    
}

enum mip_cmd_result mip_gnss_write_signal_configuration(struct mip_interface* device, uint8_t gps_enable, uint8_t glonass_enable, uint8_t galileo_enable, uint8_t beidou_enable, const uint8_t* reserved)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    insert_u8(&serializer, gps_enable);
    
    insert_u8(&serializer, glonass_enable);
    
    insert_u8(&serializer, galileo_enable);
    
    insert_u8(&serializer, beidou_enable);
    
    assert(reserved);
    for(unsigned int i=0; i < 4; i++)
        insert_u8(&serializer, reserved[i]);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_GNSS_CMD_DESC_SET, MIP_CMD_DESC_GNSS_SIGNAL_CONFIGURATION, buffer, serializer.offset);
}
enum mip_cmd_result mip_gnss_read_signal_configuration(struct mip_interface* device, uint8_t* gps_enable_out, uint8_t* glonass_enable_out, uint8_t* galileo_enable_out, uint8_t* beidou_enable_out, uint8_t* reserved_out)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength;
    enum mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_GNSS_CMD_DESC_SET, MIP_CMD_DESC_GNSS_SIGNAL_CONFIGURATION, buffer, serializer.offset, MIP_REPLY_DESC_GNSS_SIGNAL_CONFIGURATION, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        struct mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, sizeof(buffer));
        
        assert(gps_enable_out);
        extract_u8(&deserializer, gps_enable_out);
        
        assert(glonass_enable_out);
        extract_u8(&deserializer, glonass_enable_out);
        
        assert(galileo_enable_out);
        extract_u8(&deserializer, galileo_enable_out);
        
        assert(beidou_enable_out);
        extract_u8(&deserializer, beidou_enable_out);
        
        assert(reserved_out);
        for(unsigned int i=0; i < 4; i++)
            extract_u8(&deserializer, &reserved_out[i]);
        
        if( !mip_serializer_ok(&deserializer) )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
enum mip_cmd_result mip_gnss_save_signal_configuration(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_GNSS_CMD_DESC_SET, MIP_CMD_DESC_GNSS_SIGNAL_CONFIGURATION, buffer, serializer.offset);
}
enum mip_cmd_result mip_gnss_load_signal_configuration(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_GNSS_CMD_DESC_SET, MIP_CMD_DESC_GNSS_SIGNAL_CONFIGURATION, buffer, serializer.offset);
}
enum mip_cmd_result mip_gnss_default_signal_configuration(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_GNSS_CMD_DESC_SET, MIP_CMD_DESC_GNSS_SIGNAL_CONFIGURATION, buffer, serializer.offset);
}
void insert_mip_gnss_rtk_dongle_configuration_command(struct mip_serializer* serializer, const struct mip_gnss_rtk_dongle_configuration_command* self)
{
    insert_u8(serializer, self->enable);
    
    for(unsigned int i=0; i < 3; i++)
        insert_u8(serializer, self->reserved[i]);
    
}
void extract_mip_gnss_rtk_dongle_configuration_command(struct mip_serializer* serializer, struct mip_gnss_rtk_dongle_configuration_command* self)
{
    extract_u8(serializer, &self->enable);
    
    for(unsigned int i=0; i < 3; i++)
        extract_u8(serializer, &self->reserved[i]);
    
}

enum mip_cmd_result mip_gnss_write_rtk_dongle_configuration(struct mip_interface* device, uint8_t enable, const uint8_t* reserved)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    insert_u8(&serializer, enable);
    
    assert(reserved);
    for(unsigned int i=0; i < 3; i++)
        insert_u8(&serializer, reserved[i]);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_GNSS_CMD_DESC_SET, MIP_CMD_DESC_GNSS_RTK_DONGLE_CONFIGURATION, buffer, serializer.offset);
}
enum mip_cmd_result mip_gnss_read_rtk_dongle_configuration(struct mip_interface* device, uint8_t* enable_out, uint8_t* reserved_out)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength;
    enum mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_GNSS_CMD_DESC_SET, MIP_CMD_DESC_GNSS_RTK_DONGLE_CONFIGURATION, buffer, serializer.offset, MIP_REPLY_DESC_GNSS_RTK_DONGLE_CONFIGURATION, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        struct mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, sizeof(buffer));
        
        assert(enable_out);
        extract_u8(&deserializer, enable_out);
        
        assert(reserved_out);
        for(unsigned int i=0; i < 3; i++)
            extract_u8(&deserializer, &reserved_out[i]);
        
        if( !mip_serializer_ok(&deserializer) )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
enum mip_cmd_result mip_gnss_save_rtk_dongle_configuration(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_GNSS_CMD_DESC_SET, MIP_CMD_DESC_GNSS_RTK_DONGLE_CONFIGURATION, buffer, serializer.offset);
}
enum mip_cmd_result mip_gnss_load_rtk_dongle_configuration(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_GNSS_CMD_DESC_SET, MIP_CMD_DESC_GNSS_RTK_DONGLE_CONFIGURATION, buffer, serializer.offset);
}
enum mip_cmd_result mip_gnss_default_rtk_dongle_configuration(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_GNSS_CMD_DESC_SET, MIP_CMD_DESC_GNSS_RTK_DONGLE_CONFIGURATION, buffer, serializer.offset);
}
void insert_mip_gnss_receiver_safe_mode_command(struct mip_serializer* serializer, const struct mip_gnss_receiver_safe_mode_command* self)
{
    insert_u8(serializer, self->receiver_id);
    
    insert_u8(serializer, self->enable);
    
}
void extract_mip_gnss_receiver_safe_mode_command(struct mip_serializer* serializer, struct mip_gnss_receiver_safe_mode_command* self)
{
    extract_u8(serializer, &self->receiver_id);
    
    extract_u8(serializer, &self->enable);
    
}

enum mip_cmd_result mip_gnss_receiver_safe_mode(struct mip_interface* device, uint8_t receiver_id, uint8_t enable)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_u8(&serializer, receiver_id);
    
    insert_u8(&serializer, enable);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_GNSS_CMD_DESC_SET, MIP_CMD_DESC_GNSS_RECEIVER_SAFE_MODE, buffer, serializer.offset);
}

#ifdef __cplusplus
} // namespace C
} // namespace mip
} // extern "C"
#endif // __cplusplus

