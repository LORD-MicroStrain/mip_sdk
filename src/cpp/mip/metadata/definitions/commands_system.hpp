#pragma once

#include <mip/metadata/definitions/common.hpp>

#include <mip/definitions/commands_system.hpp>


#include <mip/metadata/mip_metadata.hpp>

namespace mip::metadata
{


template<>
struct MetadataFor<commands_system::CommMode::Response>
{
    using type = commands_system::CommMode::Response;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "mode",
            /* .docs          = */ "",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::mode>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "commands_system::CommMode::Response",
        /* .title       = */ "response",
        /* .docs        = */ "",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<commands_system::CommMode>
{
    using type = commands_system::CommMode;

    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name          = */ "mode",
            /* .docs          = */ "",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::mode>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "commands_system::CommMode",
        /* .title       = */ "comm_mode",
        /* .docs        = */ "Advanced specialized communication modes.\n\nThis command allows the user to communicate directly with various subsystems which may be present in MIP devices (i.e. IMU, GNSS, etc.)\nPlease see the specific device's user manual for possible modes.\n\nThis command responds with an ACK/NACK just prior to switching to the new protocol.\nFor all functions except 0x01 (use new settings), the new communications mode value is ignored.\n\n",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ {true, true, false, false, true},
        /* .response    = */ &MetadataFor<type::Response>::value,
    };
};

template<>
struct MetadataFor<commands_system::CommsInterface>
{
    using type = commands_system::CommsInterface;

    static constexpr inline EnumInfo::Entry entries[] = {
        { uint32_t(0), "ALL", "" },
        { uint32_t(1), "MAIN", "An alias that directs to Main USB if it's connected, or Main UART otherwise" },
        { uint32_t(17), "UART_1", "Depending on your device, this may mean either the first UART *currently configured*, or the first port on which UART *can be configured*. Refer to your device manual." },
        { uint32_t(18), "UART_2", "" },
        { uint32_t(19), "UART_3", "" },
        { uint32_t(33), "USB_1", "The first virtual serial port over USB (ie. COM5)" },
        { uint32_t(34), "USB_2", "The second virtual serial port over USB (ie. COM6), only available on GNSS/INS devices. Recommended for NMEA/RTCM." },
    };

    static constexpr inline EnumInfo value = {
        /* .name    = */ "CommsInterface",
        /* .docs    = */ "",
        /* .type    = */ Type::U8,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<commands_system::CommsProtocol>
{
    using type = commands_system::CommsProtocol;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { uint32_t(1), "MIP", "Microstrain Inertial Protocol" },
        { uint32_t(256), "NMEA", "" },
        { uint32_t(512), "RTCM", "" },
        { uint32_t(16777216), "SPARTN", "" },
    };

    static constexpr inline BitfieldInfo value = {
        /* .name    = */ "CommsProtocol",
        /* .docs    = */ "",
        /* .type    = */ Type::U32,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<commands_system::InterfaceControl::Response>
{
    using type = commands_system::InterfaceControl::Response;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "port",
            /* .docs          = */ "Which physical interface is being selected (USB, serial, etc)",
            /* .type          = */ {Type::ENUM, &MetadataFor<commands_system::CommsInterface>::value},
            /* .accessor      = */ nullptr, //utils::access<type, commands_system::CommsInterface, &type::port>,
            /* .attributes    = */ {true, true, true, true, true, /*echo*/true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "protocols_incoming",
            /* .docs          = */ "Input protocol(s) the port will accept. If the protocol supports ACK/NACK or detailed responses, it will be sent over this port even if no corresponding output protocol is set.",
            /* .type          = */ {Type::BITS, &MetadataFor<commands_system::CommsProtocol>::value},
            /* .accessor      = */ nullptr, //utils::access<type, commands_system::CommsProtocol, &type::protocols_incoming>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "protocols_outgoing",
            /* .docs          = */ "Data protocol(s) the port will output",
            /* .type          = */ {Type::BITS, &MetadataFor<commands_system::CommsProtocol>::value},
            /* .accessor      = */ nullptr, //utils::access<type, commands_system::CommsProtocol, &type::protocols_outgoing>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "commands_system::InterfaceControl::Response",
        /* .title       = */ "response",
        /* .docs        = */ "",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<commands_system::InterfaceControl>
{
    using type = commands_system::InterfaceControl;

    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name          = */ "port",
            /* .docs          = */ "Which physical interface is being selected (USB, serial, etc)",
            /* .type          = */ {Type::ENUM, &MetadataFor<commands_system::CommsInterface>::value},
            /* .accessor      = */ nullptr, //utils::access<type, commands_system::CommsInterface, &type::port>,
            /* .attributes    = */ {true, true, true, true, true, /*echo*/true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "protocols_incoming",
            /* .docs          = */ "Input protocol(s) the port will accept. If the protocol supports ACK/NACK or detailed responses, it will be sent over this port even if no corresponding output protocol is set.",
            /* .type          = */ {Type::BITS, &MetadataFor<commands_system::CommsProtocol>::value},
            /* .accessor      = */ nullptr, //utils::access<type, commands_system::CommsProtocol, &type::protocols_incoming>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "protocols_outgoing",
            /* .docs          = */ "Data protocol(s) the port will output",
            /* .type          = */ {Type::BITS, &MetadataFor<commands_system::CommsProtocol>::value},
            /* .accessor      = */ nullptr, //utils::access<type, commands_system::CommsProtocol, &type::protocols_outgoing>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "commands_system::InterfaceControl",
        /* .title       = */ "Interface Control",
        /* .docs        = */ "Reassign data protocols, both incoming and outgoing.\n\nResponds over the port that sent the command with an ACK/NACK immediately after the operation is complete. It is the user's responsibility to not\nsend any critical information or commands while awaiting a response! Doing so while this command processes may cause those packets to be dropped.\n\nConstraints:\n- Limited parsers and data streams are available. Refer to your device manual for more information.\n- The Main port always has a MIP parser and MIP data stream bound. Additionally, Main is the only port that can process interface control commands.\n\nIf response is NACK, no change was made. Here's what can cause a NACK:\n- The requested protocol isn't supported on this device, or on this port, or this device doesn't support that many parsers.\n- The request would break the general constraints listed above, or a device-specific constraint.\n\n",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ {true, true, true, true, true},
        /* .response    = */ &MetadataFor<type::Response>::value,
    };
};


static constexpr inline const FieldInfo* COMMANDS_SYSTEM_FIELDS[] = {
    &MetadataFor<commands_system::InterfaceControl>::value,
    &MetadataFor<commands_system::CommMode>::value,
    &MetadataFor<commands_system::InterfaceControl::Response>::value,
    &MetadataFor<commands_system::CommMode::Response>::value,
};

static constexpr DescriptorSetInfo COMMANDS_SYSTEM = {
    /*.descriptor =*/ mip::commands_system::DESCRIPTOR_SET,
    /*.name       =*/ "System Commands",
    /*.fields     =*/ COMMANDS_SYSTEM_FIELDS,
};

} // namespace mip::metadata

