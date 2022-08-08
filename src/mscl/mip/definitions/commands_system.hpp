#pragma once

#include "descriptors.h"
#include "../mip_result.h"

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

namespace mscl {
class MipSerializer;
namespace commands_system {

namespace C {
struct mip_interface;
} // namespace C

////////////////////////////////////////////////////////////////////////////////
///@addtogroup MipCommands
///@{
///@defgroup system_commands_cpp  SYSTEMCommands
///
///@{

////////////////////////////////////////////////////////////////////////////////
// Descriptors
////////////////////////////////////////////////////////////////////////////////

enum 
{
    DESCRIPTOR_SET           = 0x7F,
    
    CMD_ENUMERATE            = 0x01,
    CMD_COM_MODE             = 0x10,
    CMD_HARDWARE_CONTROL     = 0x11,
    CMD_HARDWARE_CONTROL_2   = 0x12,
    
    REPLY_COM_MODE           = 0x90,
    REPLY_HARDWARE_CONTROL   = 0x91,
    REPLY_HARDWARE_CONTROL_2 = 0x92,
};

////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////

static const uint8_t MIP_SYSTEM_COMMAND_COMM_MODE_PASSTHRU = 0x00;
static const uint8_t MIP_SYSTEM_COMMAND_COMM_MODE_NORMAL = 0x01;
static const uint8_t MIP_SYSTEM_COMMAND_COMM_MODE_IMU = 0x02;
static const uint8_t MIP_SYSTEM_COMMAND_COMM_MODE_GPS = 0x03;

////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_comm_mode  None
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

struct CommMode
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::commands_system::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::commands_system::CMD_COM_MODE;
    
    static const bool HAS_WRITE_FUNCTION = true;
    static const bool HAS_READ_FUNCTION = true;
    static const bool HAS_SAVE_FUNCTION = false;
    static const bool HAS_LOAD_FUNCTION = false;
    static const bool HAS_RESET_FUNCTION = true;
    
    MipFunctionSelector function;
    uint8_t mode;
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mscl::commands_system::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mscl::commands_system::REPLY_COM_MODE;
        
        uint8_t mode;
        
    };
};
void insert(MipSerializer& serializer, const CommMode& self);
void extract(MipSerializer& serializer, CommMode& self);

void insert(MipSerializer& serializer, const CommMode::Response& self);
void extract(MipSerializer& serializer, CommMode::Response& self);

///@}
///

///@}
///@}
///
////////////////////////////////////////////////////////////////////////////////
} // namespace commands_system
} // namespace mscl

