#pragma once

#include <mip/definitions/common.h>
#include <mip/mip_descriptors.h>
#include <mip/mip_result.h>
#include <mip/mip_interface.h>

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
namespace mip {
namespace C {
extern "C" {

#endif // __cplusplus

////////////////////////////////////////////////////////////////////////////////
///@addtogroup MipCommands_c
///@{
///@defgroup system_commands_c  System Commands
///
///@{

////////////////////////////////////////////////////////////////////////////////
// Descriptors
////////////////////////////////////////////////////////////////////////////////

enum 
{
    MIP_SYSTEM_CMD_DESC_SET                  = 0x7F,
    
    MIP_CMD_DESC_MIPNET_ENUMERATE            = 0x01,
    MIP_CMD_DESC_SYSTEM_INTERFACE_CONTROL    = 0x02,
    MIP_CMD_DESC_SYSTEM_COM_MODE             = 0x10,
    MIP_CMD_DESC_SYSTEM_HARDWARE_CONTROL     = 0x11,
    MIP_CMD_DESC_SYSTEM_HARDWARE_CONTROL_2   = 0x12,
    
    MIP_REPLY_DESC_SYSTEM_COM_MODE           = 0x90,
    MIP_REPLY_DESC_SYSTEM_HARDWARE_CONTROL   = 0x91,
    MIP_REPLY_DESC_SYSTEM_HARDWARE_CONTROL_2 = 0x92,
    MIP_REPLY_DESC_SYSTEM_INTERFACE_CONTROL  = 0x82,
};

////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////

enum { MIP_SYSTEM_COMMAND_COMM_MODE_PASSTHRU = 0x00 };
enum { MIP_SYSTEM_COMMAND_COMM_MODE_NORMAL = 0x01 };
enum { MIP_SYSTEM_COMMAND_COMM_MODE_IMU = 0x02 };
enum { MIP_SYSTEM_COMMAND_COMM_MODE_GPS = 0x03 };
enum mip_comms_interface
{
    MIP_COMMS_INTERFACE_ALL    = 0,  ///<  
    MIP_COMMS_INTERFACE_MAIN   = 1,  ///<  An alias that directs to Main USB if it's connected, or Main UART otherwise
    MIP_COMMS_INTERFACE_UART_1 = 17,  ///<  Depending on your device, this may mean either the first UART *currently configured*, or the first port on which UART *can be configured*. Refer to your device manual.
    MIP_COMMS_INTERFACE_UART_2 = 18,  ///<  
    MIP_COMMS_INTERFACE_UART_3 = 19,  ///<  
    MIP_COMMS_INTERFACE_USB_1  = 33,  ///<  The first virtual serial port over USB (ie. COM5)
    MIP_COMMS_INTERFACE_USB_2  = 34,  ///<  The second virtual serial port over USB (ie. COM6), only available on GNSS/INS devices. Recommended for NMEA/RTCM.
};
typedef enum mip_comms_interface mip_comms_interface;

static inline void insert_mip_comms_interface(microstrain_serializer* serializer, const mip_comms_interface self)
{
    microstrain_insert_u8(serializer, (uint8_t)(self));
}
static inline void extract_mip_comms_interface(microstrain_serializer* serializer, mip_comms_interface* self)
{
    uint8_t tmp = 0;
    microstrain_extract_u8(serializer, &tmp);
    *self = (mip_comms_interface)tmp;
}

typedef uint32_t mip_comms_protocol;
static const mip_comms_protocol MIP_COMMS_PROTOCOL_NONE   = 0x00000000;
static const mip_comms_protocol MIP_COMMS_PROTOCOL_MIP    = 0x00000001; ///<  Microstrain Inertial Protocol
static const mip_comms_protocol MIP_COMMS_PROTOCOL_NMEA   = 0x00000100; ///<  
static const mip_comms_protocol MIP_COMMS_PROTOCOL_RTCM   = 0x00000200; ///<  
static const mip_comms_protocol MIP_COMMS_PROTOCOL_SPARTN = 0x01000000; ///<  
static const mip_comms_protocol MIP_COMMS_PROTOCOL_ALL    = 0x01000301;
static inline void insert_mip_comms_protocol(microstrain_serializer* serializer, const mip_comms_protocol self)
{
    microstrain_insert_u32(serializer, (uint32_t)(self));
}
static inline void extract_mip_comms_protocol(microstrain_serializer* serializer, mip_comms_protocol* self)
{
    uint32_t tmp = 0;
    microstrain_extract_u32(serializer, &tmp);
    *self = (mip_comms_protocol)tmp;
}


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
///@defgroup system_comm_mode_c  (0x7F,0x10) Comm Mode
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
    mip_function_selector function;
    uint8_t mode;
};
typedef struct mip_system_comm_mode_command mip_system_comm_mode_command;

void insert_mip_system_comm_mode_command(microstrain_serializer* serializer, const mip_system_comm_mode_command* self);
void extract_mip_system_comm_mode_command(microstrain_serializer* serializer, mip_system_comm_mode_command* self);

struct mip_system_comm_mode_response
{
    uint8_t mode;
};
typedef struct mip_system_comm_mode_response mip_system_comm_mode_response;

void insert_mip_system_comm_mode_response(microstrain_serializer* serializer, const mip_system_comm_mode_response* self);
void extract_mip_system_comm_mode_response(microstrain_serializer* serializer, mip_system_comm_mode_response* self);

mip_cmd_result mip_system_write_comm_mode(mip_interface* device, uint8_t mode);
mip_cmd_result mip_system_read_comm_mode(mip_interface* device, uint8_t* mode_out);
mip_cmd_result mip_system_default_comm_mode(mip_interface* device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup system_interface_control_c  (0x7F,0x02) Interface Control
/// Reassign data protocols, both incoming and outgoing.
/// 
/// Responds over the port that sent the command with an ACK/NACK immediately after the operation is complete. It is the user's responsibility to not
/// send any critical information or commands while awaiting a response! Doing so while this command processes may cause those packets to be dropped.
/// 
/// Constraints:
/// - Limited parsers and data streams are available. Refer to your device manual for more information.
/// - The Main port always has a MIP parser and MIP data stream bound. Additionally, Main is the only port that can process interface control commands.
/// 
/// If response is NACK, no change was made. Here's what can cause a NACK:
/// - The requested protocol isn't supported on this device, or on this port, or this device doesn't support that many parsers.
/// - The request would break the general constraints listed above, or a device-specific constraint.
/// 
/// 
///
///@{

struct mip_system_interface_control_command
{
    mip_function_selector function;
    mip_comms_interface port; ///< Which physical interface is being selected (USB, serial, etc)
    mip_comms_protocol protocols_incoming; ///< Input protocol(s) the port will accept. If the protocol supports ACK/NACK or detailed responses, it will be sent over this port even if no corresponding output protocol is set.
    mip_comms_protocol protocols_outgoing; ///< Data protocol(s) the port will output
};
typedef struct mip_system_interface_control_command mip_system_interface_control_command;

void insert_mip_system_interface_control_command(microstrain_serializer* serializer, const mip_system_interface_control_command* self);
void extract_mip_system_interface_control_command(microstrain_serializer* serializer, mip_system_interface_control_command* self);

struct mip_system_interface_control_response
{
    mip_comms_interface port; ///< Which physical interface is being selected (USB, serial, etc)
    mip_comms_protocol protocols_incoming; ///< Input protocol(s) the port will accept. If the protocol supports ACK/NACK or detailed responses, it will be sent over this port even if no corresponding output protocol is set.
    mip_comms_protocol protocols_outgoing; ///< Data protocol(s) the port will output
};
typedef struct mip_system_interface_control_response mip_system_interface_control_response;

void insert_mip_system_interface_control_response(microstrain_serializer* serializer, const mip_system_interface_control_response* self);
void extract_mip_system_interface_control_response(microstrain_serializer* serializer, mip_system_interface_control_response* self);

mip_cmd_result mip_system_write_interface_control(mip_interface* device, mip_comms_interface port, mip_comms_protocol protocols_incoming, mip_comms_protocol protocols_outgoing);
mip_cmd_result mip_system_read_interface_control(mip_interface* device, mip_comms_interface port, mip_comms_protocol* protocols_incoming_out, mip_comms_protocol* protocols_outgoing_out);
mip_cmd_result mip_system_save_interface_control(mip_interface* device, mip_comms_interface port);
mip_cmd_result mip_system_load_interface_control(mip_interface* device, mip_comms_interface port);
mip_cmd_result mip_system_default_interface_control(mip_interface* device, mip_comms_interface port);

///@}
///

///@}
///@}
///
////////////////////////////////////////////////////////////////////////////////
#ifdef __cplusplus
} // extern "C"
} // namespace C
} // namespace mip
#endif // __cplusplus

