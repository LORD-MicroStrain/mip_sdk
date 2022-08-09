
#include "commands_system.h"

#include "../../utils/serialization.h"
#include "../mip_interface.h"

#include <assert.h>


#ifdef __cplusplus
namespace mscl {
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

void insert_mip_system_comm_mode_command(struct mip_serializer* serializer, const struct mip_system_comm_mode_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    insert_u8(serializer, self->mode);
}

void extract_mip_system_comm_mode_command(struct mip_serializer* serializer, struct mip_system_comm_mode_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    extract_u8(serializer, &self->mode);
}

/// @brief Advanced specialized communication modes.
/// 
/// This command allows the user to communicate directly with various subsystems which may be present in MIP devices (i.e. IMU, GNSS, etc.)
/// Please see the specific device's user manual for possible modes.
/// 
/// This command responds with an ACK/NACK just prior to switching to the new protocol.
/// For all functions except 0x01 (use new settings), the new communications mode value is ignored.
/// 
/// 
/// @param mode 
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_system_write_comm_mode(struct mip_interface* device, uint8_t mode)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    insert_u8(&serializer, mode);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_SYSTEM_CMD_DESC_SET, MIP_CMD_DESC_SYSTEM_COM_MODE, buffer, serializer.offset);
}

/// @brief Advanced specialized communication modes.
/// 
/// This command allows the user to communicate directly with various subsystems which may be present in MIP devices (i.e. IMU, GNSS, etc.)
/// Please see the specific device's user manual for possible modes.
/// 
/// This command responds with an ACK/NACK just prior to switching to the new protocol.
/// For all functions except 0x01 (use new settings), the new communications mode value is ignored.
/// 
/// 
/// @param[out] mode 
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_system_read_comm_mode(struct mip_interface* device, uint8_t* mode)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_SYSTEM_CMD_DESC_SET, MIP_CMD_DESC_SYSTEM_COM_MODE, buffer, serializer.offset, MIP_REPLY_DESC_SYSTEM_COM_MODE, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        struct mip_serializer serializer;
        mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
        
        extract_u8(&serializer, mode);
        
        if( !mip_serializer_ok(&serializer) )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Advanced specialized communication modes.
/// 
/// This command allows the user to communicate directly with various subsystems which may be present in MIP devices (i.e. IMU, GNSS, etc.)
/// Please see the specific device's user manual for possible modes.
/// 
/// This command responds with an ACK/NACK just prior to switching to the new protocol.
/// For all functions except 0x01 (use new settings), the new communications mode value is ignored.
/// 
/// 
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_system_default_comm_mode(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_SYSTEM_CMD_DESC_SET, MIP_CMD_DESC_SYSTEM_COM_MODE, buffer, serializer.offset);
}


#ifdef __cplusplus
} // namespace C
} // namespace mscl
} // extern "C"
#endif // __cplusplus

