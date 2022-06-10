#pragma once

#include "descriptors.h"

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
namespace mscl {
extern "C" {
#endif // __cplusplus

////////////////////////////////////////////////////////////////////////////////
///@addtogroup MipCommands
///@{
///@defgroup SYSTEM_COMMAND  SYSTEM COMMAND
///
///@{

////////////////////////////////////////////////////////////////////////////////
// Descriptors
////////////////////////////////////////////////////////////////////////////////

enum MipSystemCommand_Descriptors
{
    MIP_SYSTEM_COMMAND_DESC_SET    = 0x7F,
    
    MIP_CMD_DESC_SYSTEM_COM_MODE   = 0x10,
    
    MIP_REPLY_DESC_SYSTEM_COM_MODE = 0x90,
};

////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////

#define MIPMIP_SYSTEM_COMMAND_COMM_MODE_PASSTHRU_MIP_SYSTEM_COMMAND_COMM_MODE_PASSTHRU 0x00
#define MIPMIP_SYSTEM_COMMAND_COMM_MODE_NORMAL_MIP_SYSTEM_COMMAND_COMM_MODE_NORMAL 0x01
#define MIPMIP_SYSTEM_COMMAND_COMM_MODE_IMU_MIP_SYSTEM_COMMAND_COMM_MODE_IMU 0x02
#define MIPMIP_SYSTEM_COMMAND_COMM_MODE_GPS_MIP_SYSTEM_COMMAND_COMM_MODE_GPS 0x03

////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_cmd_system_comm_mode  System Comm Mode
/// Advanced specialized communication modes.
/// 
/// This command allows the user to communicate directly with various subsystems which may be present in MIP devices (i.e. IMU, GNSS, etc.)
/// Please see the specific device's user manual for possible modes.
/// 
/// This command responds with an ACK/NACK just prior to switching to the new protocol.
/// For all functions except 0x01 (use new settings), the new communications mode value is ignored.
/// 
/// 
///
///@{

struct MipCmd_System_CommMode
{
    enum MipFunctionSelector                          function;
    uint8_t                                           mode;
};
size_t insert_MipCmd_System_CommMode(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_System_CommMode* self);
size_t extract_MipCmd_System_CommMode(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_System_CommMode* self);

struct MipCmd_System_CommMode_Response
{
    uint8_t                                           mode;
};
size_t insert_MipCmd_System_CommMode_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_System_CommMode_Response* self);
size_t extract_MipCmd_System_CommMode_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_System_CommMode_Response* self);

///@}
///

///@}
///@}
///
////////////////////////////////////////////////////////////////////////////////

#ifdef __cplusplus
} // extern "C"
} // namespace mscl
#endif // __cplusplus
