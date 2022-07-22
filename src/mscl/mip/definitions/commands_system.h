#pragma once

#include "descriptors.h"
#include "../mip_result.h"

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
namespace mscl {
#endif // __cplusplus

struct mip_interface;

////////////////////////////////////////////////////////////////////////////////
///@addtogroup MipCommands
///@{
///@defgroup SYSTEM_COMMAND  SYSTEM COMMAND
///
///@{

////////////////////////////////////////////////////////////////////////////////
// Descriptors
////////////////////////////////////////////////////////////////////////////////

enum mip_system_command_descriptors
{
    MIP_SYSTEM_COMMAND_DESC_SET    = 0x7F,
    
    MIP_CMD_DESC_SYSTEM_COM_MODE   = 0x10,
    
    MIP_REPLY_DESC_SYSTEM_COM_MODE = 0x90,
};
#ifdef __cplusplus
namespace C {
extern "C" {
#endif // __cplusplus


////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////

#define MIP_SYSTEM_COMMAND_COMM_MODE_PASSTHRU_MIP_SYSTEM_COMMAND_COMM_MODE_PASSTHRU 0x00
#define MIP_SYSTEM_COMMAND_COMM_MODE_NORMAL_MIP_SYSTEM_COMMAND_COMM_MODE_NORMAL 0x01
#define MIP_SYSTEM_COMMAND_COMM_MODE_IMU_MIP_SYSTEM_COMMAND_COMM_MODE_IMU 0x02
#define MIP_SYSTEM_COMMAND_COMM_MODE_GPS_MIP_SYSTEM_COMMAND_COMM_MODE_GPS 0x03

////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_system_comm_mode  None
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

struct mip_system_comm_mode_command
{
    enum mip_function_selector                        function;
    uint8_t                                           mode;
};
size_t insert_mip_system_comm_mode_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_system_comm_mode_command* self);
size_t extract_mip_system_comm_mode_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_system_comm_mode_command* self);

struct mip_system_comm_mode_response
{
    uint8_t                                           mode;
};
size_t insert_mip_system_comm_mode_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_system_comm_mode_response* self);
size_t extract_mip_system_comm_mode_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_system_comm_mode_response* self);

mip_cmd_result write_mip_system_comm_mode(struct mip_interface* device, uint8_t mode);
mip_cmd_result read_mip_system_comm_mode(struct mip_interface* device, uint8_t* mode);
mip_cmd_result save_mip_system_comm_mode(struct mip_interface* device);
mip_cmd_result load_mip_system_comm_mode(struct mip_interface* device);
mip_cmd_result default_mip_system_comm_mode(struct mip_interface* device);
///@}
///

///@}
///@}
///
////////////////////////////////////////////////////////////////////////////////

#ifdef __cplusplus
} // extern "C"
} // namespace C


template<>
struct MipFieldInfo<C::mip_system_comm_mode_command>
{
    static const uint8_t descriptorSet = MIP_SYSTEM_COMMAND_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_SYSTEM_COM_MODE;
    static const uint8_t responseDescriptor = MIP_REPLY_DESC_SYSTEM_COM_MODE;
    typedef C::mip_system_comm_mode_response Response;
    
    static const bool hasFunctionSelector = true;
    static const bool canWrite = true;
    static const bool canRead = true;
    static const bool canSave = true;
    static const bool canLoad = true;
    static const bool canReset = true;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_system_comm_mode_command& self)
    {
        return C::insert_mip_system_comm_mode_command(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_system_comm_mode_command& self)
    {
        return C::extract_mip_system_comm_mode_command(buffer, bufferSize, offset, &self);
    }
    static inline size_t insert_response(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_system_comm_mode_response& self)
    {
        return C::insert_mip_system_comm_mode_response(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract_response(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_system_comm_mode_response& self)
    {
        return C::extract_mip_system_comm_mode_response(buffer, bufferSize, offset, &self);
    }
};



} // namespace mscl
#endif // __cplusplus
