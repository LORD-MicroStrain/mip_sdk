#pragma once

#include <mip/definitions/common.hpp>
#include <mip/mip_descriptors.hpp>
#include <mip/mip_result.hpp>
#include <mip/mip_interface.hpp>

#include <stdint.h>
#include <stddef.h>

namespace mip {
namespace C {
struct mip_interface;
} // namespace C

namespace commands_system {

////////////////////////////////////////////////////////////////////////////////
///@addtogroup MipCommands_cpp
///@{
///@defgroup system_commands_cpp  System Commands
///
///@{

////////////////////////////////////////////////////////////////////////////////
// Descriptors
////////////////////////////////////////////////////////////////////////////////

enum 
{
    DESCRIPTOR_SET           = 0x7F,
    
    CMD_ENUMERATE            = 0x01,
    CMD_INTERFACE_CONTROL    = 0x02,
    CMD_COM_MODE             = 0x10,
    CMD_HARDWARE_CONTROL     = 0x11,
    CMD_HARDWARE_CONTROL_2   = 0x12,
    
    REPLY_COM_MODE           = 0x90,
    REPLY_HARDWARE_CONTROL   = 0x91,
    REPLY_HARDWARE_CONTROL_2 = 0x92,
    REPLY_INTERFACE_CONTROL  = 0x82,
};

////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////

static constexpr const uint8_t MIP_SYSTEM_COMMAND_COMM_MODE_PASSTHRU = 0x00;
static constexpr const uint8_t MIP_SYSTEM_COMMAND_COMM_MODE_NORMAL = 0x01;
static constexpr const uint8_t MIP_SYSTEM_COMMAND_COMM_MODE_IMU = 0x02;
static constexpr const uint8_t MIP_SYSTEM_COMMAND_COMM_MODE_GPS = 0x03;
enum class CommsInterface : uint8_t
{
    ALL    = 0,  ///<  
    MAIN   = 1,  ///<  An alias that directs to Main USB if it's connected, or Main UART otherwise
    UART_1 = 17,  ///<  First configured UART. Note that this may not mean the first GPIO, if that pin is not set for UART.
    UART_2 = 18,  ///<  
    UART_3 = 19,  ///<  
    USB_1  = 33,  ///<  The first virtual serial port over USB (ie. COM5)
    USB_2  = 34,  ///<  The second virtual serial port over USB (ie. COM6), only available on GNSS/INS devices. Recommended for NMEA/RTCM.
};

struct CommsProtocol : Bitfield<CommsProtocol>
{
    typedef uint32_t Type;
    enum _enumType : uint32_t
    {
        NONE         = 0x00000000,
        MIP_COMMANDS = 0x00000001,  ///<  
        MIP_DATA     = 0x00000002,  ///<  
        NMEA         = 0x00000004,  ///<  
        RTCM         = 0x00000008,  ///<  
        SPARTN       = 0x00000010,  ///<  
        ALL          = 0x0000001F,
    };
    uint32_t value = NONE;
    
    CommsProtocol() : value(NONE) {}
    CommsProtocol(int val) : value((uint32_t)val) {}
    operator uint32_t() const { return value; }
    CommsProtocol& operator=(uint32_t val) { value = val; return *this; }
    CommsProtocol& operator=(int val) { value = uint32_t(val); return *this; }
    CommsProtocol& operator|=(uint32_t val) { return *this = value | val; }
    CommsProtocol& operator&=(uint32_t val) { return *this = value & val; }
    
    bool mipCommands() const { return (value & MIP_COMMANDS) > 0; }
    void mipCommands(bool val) { value &= ~MIP_COMMANDS; if(val) value |= MIP_COMMANDS; }
    bool mipData() const { return (value & MIP_DATA) > 0; }
    void mipData(bool val) { value &= ~MIP_DATA; if(val) value |= MIP_DATA; }
    bool nmea() const { return (value & NMEA) > 0; }
    void nmea(bool val) { value &= ~NMEA; if(val) value |= NMEA; }
    bool rtcm() const { return (value & RTCM) > 0; }
    void rtcm(bool val) { value &= ~RTCM; if(val) value |= RTCM; }
    bool spartn() const { return (value & SPARTN) > 0; }
    void spartn(bool val) { value &= ~SPARTN; if(val) value |= SPARTN; }
    bool allSet() const { return value == ALL; }
    void setAll() { value |= ALL; }
};

////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
///@defgroup system_comm_mode_cpp  (0x7F,0x10) Comm Mode
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
    /// Parameters
    FunctionSelector function = static_cast<FunctionSelector>(0);
    uint8_t mode = 0;
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_system::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_system::CMD_COM_MODE;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "CommMode";
    static constexpr const char* DOC_NAME = "CommMode";
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    
    auto asTuple() const
    {
        return std::make_tuple(mode);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(mode));
    }
    
    static CommMode create_sld_all(::mip::FunctionSelector function)
    {
        CommMode cmd;
        cmd.function = function;
        return cmd;
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
    struct Response
    {
        /// Parameters
        uint8_t mode = 0;
        
        /// Descriptors
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_system::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_system::REPLY_COM_MODE;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "CommMode::Response";
        static constexpr const char* DOC_NAME = "CommMode Response";
        static constexpr const bool HAS_FUNCTION_SELECTOR = false;
        
        auto asTuple() const
        {
            return std::make_tuple(mode);
        }
        
        auto asTuple()
        {
            return std::make_tuple(std::ref(mode));
        }
        
        /// Serialization
        void insert(Serializer& serializer) const;
        void extract(Serializer& serializer);
        
    };
};
TypedResult<CommMode> writeCommMode(C::mip_interface& device, uint8_t mode);
TypedResult<CommMode> readCommMode(C::mip_interface& device, uint8_t* modeOut);
TypedResult<CommMode> defaultCommMode(C::mip_interface& device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup system_interface_control_cpp  (0x7F,0x02) Interface Control
/// Reassign port functions.
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

struct InterfaceControl
{
    /// Parameters
    FunctionSelector function = static_cast<FunctionSelector>(0);
    CommsInterface interface = static_cast<CommsInterface>(0); ///< Which physical interface is being selected (USB, serial, etc)
    CommsProtocol protocols_in; ///< Input protocol(s) enabled. If the protocol supports ACK/NACK or detailed responses, it will be sent over this port even if no corresponding output protocol is set.
    CommsProtocol protocols_out; ///< Output data protocol(s) enabled.
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_system::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_system::CMD_INTERFACE_CONTROL;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "InterfaceControl";
    static constexpr const char* DOC_NAME = "Interface Control";
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    
    auto asTuple() const
    {
        return std::make_tuple(interface,protocols_in,protocols_out);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(interface),std::ref(protocols_in),std::ref(protocols_out));
    }
    
    static InterfaceControl create_sld_all(::mip::FunctionSelector function)
    {
        InterfaceControl cmd;
        cmd.function = function;
        cmd.interface = ::mip::commands_system::CommsInterface::ALL;
        return cmd;
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
    struct Response
    {
        /// Parameters
        CommsInterface interface = static_cast<CommsInterface>(0); ///< Which physical interface is being selected (USB, serial, etc)
        CommsProtocol protocols_in; ///< Input protocol(s) enabled. If the protocol supports ACK/NACK or detailed responses, it will be sent over this port even if no corresponding output protocol is set.
        CommsProtocol protocols_out; ///< Output data protocol(s) enabled.
        
        /// Descriptors
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_system::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_system::REPLY_INTERFACE_CONTROL;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "InterfaceControl::Response";
        static constexpr const char* DOC_NAME = "Interface Control Response";
        static constexpr const bool HAS_FUNCTION_SELECTOR = false;
        
        auto asTuple() const
        {
            return std::make_tuple(interface,protocols_in,protocols_out);
        }
        
        auto asTuple()
        {
            return std::make_tuple(std::ref(interface),std::ref(protocols_in),std::ref(protocols_out));
        }
        
        /// Serialization
        void insert(Serializer& serializer) const;
        void extract(Serializer& serializer);
        
    };
};
TypedResult<InterfaceControl> writeInterfaceControl(C::mip_interface& device, CommsInterface interface, CommsProtocol protocolsIn, CommsProtocol protocolsOut);
TypedResult<InterfaceControl> readInterfaceControl(C::mip_interface& device, CommsInterface interface, CommsProtocol* protocolsInOut, CommsProtocol* protocolsOutOut);
TypedResult<InterfaceControl> saveInterfaceControl(C::mip_interface& device, CommsInterface interface);
TypedResult<InterfaceControl> loadInterfaceControl(C::mip_interface& device, CommsInterface interface);
TypedResult<InterfaceControl> defaultInterfaceControl(C::mip_interface& device, CommsInterface interface);

///@}
///

///@}
///@}
///
////////////////////////////////////////////////////////////////////////////////
} // namespace commands_system
} // namespace mip

