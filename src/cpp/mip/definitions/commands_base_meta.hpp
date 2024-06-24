#pragma once

#include "commands_base.hpp"

#include <mip/mip_metadata.hpp>


namespace mip
{

template<>
struct FieldInfo<commands_base::Ping>
{
    using Cmd = commands_base::Ping;
    using Rsp = Cmd::Response;

    static constexpr inline const char* NAME = "Ping";
    static constexpr inline const char* DOC_NAME = "Ping";
    static constexpr CompositeDescriptor DESCRIPTOR = Cmd::DESCRIPTOR;

    static constexpr inline std::initializer_list<ParameterInfo> PARAMETERS = {};
};


template<>
struct FieldInfo<commands_base::CommSpeed>
{
    using Cmd = commands_base::CommSpeed;
    using Rsp = Cmd::Response;

    static constexpr inline const char* NAME = "CommSpeed";
    static constexpr inline const char* DOC_NAME = "Comm Port Speed";
    static constexpr CompositeDescriptor DESCRIPTOR = Cmd::DESCRIPTOR;

    static constexpr inline std::initializer_list<ParameterInfo> PARAMETERS = {
        FUNCTION_PARAMETER,
        {
            /* .name          = */ "port",
            /* .docs          = */ "Port ID number, starting with 1. When function is SAVE, LOAD, or DEFAULT, this can be 0 to apply to all ports.",
            /* .type          = */ ParameterInfo::Type::U8,
            /* .accessor      = */ utils::access<Cmd,uint8_t,&Cmd::port>,
            /* .byte_offset   = */ 1,
            /* .functions     = */ {true, true, true, true, true, true},
        },
        {
            /* .name          = */ "baud",
            /* .docs          = */ "Port baud rate. Must be a supported rate.",
            /* .type          = */ ParameterInfo::Type::U32,
            /* .accessor      = */ utils::access<Cmd,uint32_t,&Cmd::baud>,
            /* .byte_offset   = */ 2,
            /* .functions     = */ {true,false,false,false,false},
        },
    };
};


using CommandsBase = std::tuple<
    FieldInfo<commands_base::Ping>,
    FieldInfo<commands_base::CommSpeed>,
    void
>;

} // namespace mip
