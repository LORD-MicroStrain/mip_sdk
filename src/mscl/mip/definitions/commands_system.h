#pragma once

#include "descriptors.h"
#include "../mip_result.h"

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
namespace mscl {
extern "C" {
#endif // __cplusplus

struct MipInterfaceState;

////////////////////////////////////////////////////////////////////////////////
///@addtogroup MipCommands
///@{
///@defgroup SYSTEM_COMMAND  SYSTEM COMMAND
///
///@{

////////////////////////////////////////////////////////////////////////////////
// Descriptors
////////////////////////////////////////////////////////////////////////////////

enum MipSystemCommandDescriptors
{
    MIP_SYSTEM_COMMAND_DESC_SET    = 0x7F,
    
    MIP_CMD_DESC_SYSTEM_COM_MODE   = 0x10,
    
    MIP_REPLY_DESC_SYSTEM_COM_MODE = 0x90,
};

////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////

#define MIPMIPSYSTEMCOMMANDCOMMMODEPASSTHRU_MIP_SYSTEM_COMMAND_COMM_MODE_PASSTHRU 0x00
#define MIPMIPSYSTEMCOMMANDCOMMMODENORMAL_MIP_SYSTEM_COMMAND_COMM_MODE_NORMAL 0x01
#define MIPMIPSYSTEMCOMMANDCOMMMODEIMU_MIP_SYSTEM_COMMAND_COMM_MODE_IMU 0x02
#define MIPMIPSYSTEMCOMMANDCOMMMODEGPS_MIP_SYSTEM_COMMAND_COMM_MODE_GPS 0x03

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

MipCmdResult write_mip_cmd_system_comm_mode(struct MipInterfaceState* device, uint8_t mode);
MipCmdResult read_mip_cmd_system_comm_mode(struct MipInterfaceState* device, uint8_t* mode);
MipCmdResult save_mip_cmd_system_comm_mode(struct MipInterfaceState* device);
MipCmdResult load_mip_cmd_system_comm_mode(struct MipInterfaceState* device);
MipCmdResult default_mip_cmd_system_comm_mode(struct MipInterfaceState* device);
///@}
///

///@}
///@}
///
////////////////////////////////////////////////////////////////////////////////

#ifdef __cplusplus
} // extern "C"


template<>
struct MipFieldInfo<MipCmd_System_CommMode>
{
    static const uint8_t descriptorSet = MIP_SYSTEM_COMMAND_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_SYSTEM_COM_MODE;
    static const uint8_t responseDescriptor = MIP_REPLY_DESC_SYSTEM_COM_MODE;
    typedef MipCmd_System_CommMode_Response Response;
    
    static const bool hasFunctionSelector = true;
    static const bool canWrite = true;
    static const bool canRead = true;
    static const bool canSave = true;
    static const bool canLoad = true;
    static const bool canReset = true;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipCmd_System_CommMode& self)
    {
        return insert_MipCmd_System_CommMode(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipCmd_System_CommMode& self)
    {
        return extract_MipCmd_System_CommMode(buffer, bufferSize, offset, &self);
    }
    static inline size_t insert_response(uint8_t* buffer, size_t bufferSize, size_t offset, const MipCmd_System_CommMode_Response& self)
    {
        return insert_MipCmd_System_CommMode_Response(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract_response(const uint8_t* buffer, size_t bufferSize, size_t offset, MipCmd_System_CommMode_Response& self)
    {
        return extract_MipCmd_System_CommMode_Response(buffer, bufferSize, offset, &self);
    }
};



} // namespace mscl
#endif // __cplusplus
