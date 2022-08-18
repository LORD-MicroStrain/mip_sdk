
#include "commands_system.h"

#include "../utils/serialization.h"
#include "../mip_interface.h"

#include <assert.h>


#ifdef __cplusplus
namespace mip {
namespace C {
extern "C" {

#endif // __cplusplus
struct mip_interface;
struct mip_serializer;
struct mip_field;


////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

void insert_mip_system_comm_mode_command(struct mip_serializer* serializer, const struct mip_system_comm_mode_command* self)
{
    insert_u8(serializer, self->mode);
    
}
void extract_mip_system_comm_mode_command(struct mip_serializer* serializer, struct mip_system_comm_mode_command* self)
{
    extract_u8(serializer, &self->mode);
    
}

enum mip_cmd_result mip_system_write_comm_mode(struct mip_interface* device, uint8_t mode)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    insert_u8(&serializer, mode);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_SYSTEM_CMD_DESC_SET, MIP_CMD_DESC_SYSTEM_COM_MODE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
enum mip_cmd_result mip_system_read_comm_mode(struct mip_interface* device, uint8_t* mode_out)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(mip_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    enum mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_SYSTEM_CMD_DESC_SET, MIP_CMD_DESC_SYSTEM_COM_MODE, buffer, (uint8_t)mip_serializer_length(&serializer), MIP_REPLY_DESC_SYSTEM_COM_MODE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        struct mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(mode_out);
        extract_u8(&deserializer, mode_out);
        
        if( !mip_serializer_is_complete(&deserializer) )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
enum mip_cmd_result mip_system_default_comm_mode(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_SYSTEM_CMD_DESC_SET, MIP_CMD_DESC_SYSTEM_COM_MODE, buffer, (uint8_t)mip_serializer_length(&serializer));
}

#ifdef __cplusplus
} // namespace C
} // namespace mip
} // extern "C"
#endif // __cplusplus

