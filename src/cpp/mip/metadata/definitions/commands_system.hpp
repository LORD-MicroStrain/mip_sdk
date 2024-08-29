#pragma once

#include "common.hpp"

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
            /* .accessor      = */ utils::access<type, uint8_t, &type::mode>,
            /* .functions     = */ {true, false, false, false, false,  true},
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
        /* .proprietary = */ false,
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
            /* .accessor      = */ utils::access<type, uint8_t, &type::mode>,
            /* .functions     = */ {true, false, false, false, false,  true},
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
        /* .functions   = */ {true, true, false, false, true,  true},
        /* .proprietary = */ false,
        /* .response    = */ &MetadataFor<type::Response>::value,
    };
};


static constexpr inline const FieldInfo* COMMANDS_SYSTEM_FIELDS[] = {
    &MetadataFor<commands_system::CommMode>::value,
    &MetadataFor<commands_system::CommMode::Response>::value,
};

static constexpr inline const DescriptorSet COMMANDS_SYSTEM_DS(commands_system::DESCRIPTOR_SET, "commands_system", COMMANDS_SYSTEM_FIELDS);

} // namespace mip::metadata

