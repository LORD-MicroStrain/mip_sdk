
#include "commands_system.h"

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


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_system_comm_mode_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_system_comm_mode_command* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->mode);
    
    return offset;
}

size_t extract_mip_system_comm_mode_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_system_comm_mode_command* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->mode);
    
    return offset;
}


size_t insert_mip_system_comm_mode_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_system_comm_mode_response* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->mode);
    
    return offset;
}

size_t extract_mip_system_comm_mode_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_system_comm_mode_response* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->mode);
    
    return offset;
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
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_WRITE);
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, mode);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_SYSTEM_CMD_DESC_SET, MIP_CMD_DESC_SYSTEM_COM_MODE, buffer, cmdUsed);
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
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_READ);
    assert(cmdUsed <= sizeof(buffer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_SYSTEM_CMD_DESC_SET, MIP_CMD_DESC_SYSTEM_COM_MODE, buffer, cmdUsed, MIP_REPLY_DESC_SYSTEM_COM_MODE, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        size_t responseUsed = 0;
        responseUsed = extract_u8(buffer, sizeof(buffer), responseUsed, mode);
        
        if( responseUsed != responseLength )
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
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_RESET);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_SYSTEM_CMD_DESC_SET, MIP_CMD_DESC_SYSTEM_COM_MODE, buffer, cmdUsed);
}


#ifdef __cplusplus
} // extern "C"
} // namespace mscl
#endif // __cplusplus
