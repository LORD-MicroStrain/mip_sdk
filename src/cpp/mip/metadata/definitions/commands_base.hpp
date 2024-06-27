#pragma once

#include <mip/definitions/commands_base.hpp>

#include <mip/metadata/mip_metadata.hpp>


namespace mip::metadata
{

template<>
struct MetadataFor<commands_base::Ping>
{
    using type = commands_base::Ping;

    static constexpr inline FieldInfo value = {
        /* .name        = */ "Ping",
        /* .title       = */ "Ping",
        /* .docs        = */ "Pings the device.",
        /* .parameters  = */ {
        },
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .proprietary = */ false,
        /* .response    = */ nullptr,
    };
};


template<>
struct MetadataFor<commands_base::CommSpeed>
{
    using type = commands_base::CommSpeed;
    //using Rsp = Cmd::Response;
    //
    //static constexpr inline const char* NAME = "CommSpeed";
    //static constexpr inline const char* DOC_NAME = "Comm Port Speed";
    //static constexpr CompositeDescriptor DESCRIPTOR = Cmd::DESCRIPTOR;

    static constexpr inline std::initializer_list<ParameterInfo> PARAMETERS = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name          = */ "port",
            /* .docs          = */ "Port ID number, starting with 1. When function is SAVE, LOAD, or DEFAULT, this can be 0 to apply to all ports.",
            /* .type          = */ {Type::U8},
            /* .accessor      = */ utils::access<type,uint8_t,&type::port>,
            /* .functions     = */ {true, true, true, true, true, true},
        },
        {
            /* .name          = */ "baud",
            /* .docs          = */ "Port baud rate. Must be a supported rate.",
            /* .type          = */ {Type::U32},
            /* .accessor      = */ utils::access<type,uint32_t,&type::baud>,
            /* .functions     = */ {true,false,false,false,false},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "CommSpeed",
        /* .title       = */ "Comm Port Speed",
        /* .docs        = */ "Changes the comm port speed.",
        /* .parameters  = */ PARAMETERS,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ {true,true,true,true,true},
        /* .proprietary = */ false,
        /* .response    = */ nullptr,
    };
};


static constexpr inline std::initializer_list<const FieldInfo*> ALL_BASE_COMMANDS = {
    &MetadataFor<commands_base::Ping>::value,
    &MetadataFor<commands_base::CommSpeed>::value,
};



} // namespace mip
