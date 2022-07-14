
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
size_t insert_MipCmd_System_CommMode(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_System_CommMode* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->mode);
    
    return offset;
}

size_t extract_MipCmd_System_CommMode(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_System_CommMode* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->mode);
    
    return offset;
}


size_t insert_MipCmd_System_CommMode_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_System_CommMode_Response* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->mode);
    
    return offset;
}

size_t extract_MipCmd_System_CommMode_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_System_CommMode_Response* self)
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
/// @returns MipCmdResult
/// 
MipCmdResult write_mip_cmd_system_comm_mode(struct MipInterfaceState* device, uint8_t mode)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 1);;
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, mode);
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_SYSTEM_COMMAND_DESC_SET, MIP_CMD_DESC_SYSTEM_COM_MODE, buffer, cmdUsed);
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
/// @returns MipCmdResult
/// 
MipCmdResult read_mip_cmd_system_comm_mode(struct MipInterfaceState* device, uint8_t* mode)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 2);;
    assert(cmdUsed <= sizeof(buffer));
    
    uint8_t responseLength;
    MipCmdResult result_local = MipInterface_runCommandWithResponse(device, MIP_SYSTEM_COMMAND_DESC_SET, MIP_CMD_DESC_SYSTEM_COM_MODE, buffer, cmdUsed, MIP_REPLY_DESC_SYSTEM_COM_MODE, buffer, &responseLength);
    
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
/// @returns MipCmdResult
/// 
MipCmdResult save_mip_cmd_system_comm_mode(struct MipInterfaceState* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 3);;
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_SYSTEM_COMMAND_DESC_SET, MIP_CMD_DESC_SYSTEM_COM_MODE, buffer, cmdUsed);
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
/// @returns MipCmdResult
/// 
MipCmdResult load_mip_cmd_system_comm_mode(struct MipInterfaceState* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 4);;
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_SYSTEM_COMMAND_DESC_SET, MIP_CMD_DESC_SYSTEM_COM_MODE, buffer, cmdUsed);
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
/// @returns MipCmdResult
/// 
MipCmdResult default_mip_cmd_system_comm_mode(struct MipInterfaceState* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 5);;
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_SYSTEM_COMMAND_DESC_SET, MIP_CMD_DESC_SYSTEM_COM_MODE, buffer, cmdUsed);
}


#ifdef __cplusplus
} // extern "C"
} // namespace mscl
#endif // __cplusplus
