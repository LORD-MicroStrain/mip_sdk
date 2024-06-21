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
            /* .type          = */ ParameterInfo::Type::U8,
            /* .name          = */ "port",
            /* .docs          = */ "Port ID number, starting with 1. When function is SAVE, LOAD, or DEFAULT, this can be 0 to apply to all ports.",
            /* .byte_offset   = */ 1,
            /* .struct_offset = */ offsetof(Cmd,port),
            /* .functions     = */ {true, true, true, true, true, true},
        },
        {
            /* .type          = */ ParameterInfo::Type::U32,
            /* .name          = */ "baud",
            /* .docs          = */ "Port baud rate. Must be a supported rate.",
            /* .byte_offset   = */ 2,
            /* .struct_offset = */ offsetof(Cmd,baud),
            /* .functions     = */ {true,false,false,false,false},
        },
    };
};


} // namespace mip
