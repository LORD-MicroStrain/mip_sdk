#pragma once

#include "mip/metadata/common.hpp"
#include "mip/metadata/mip_metadata.hpp"

#include <mip/definitions/commands_rtk.hpp>

namespace mip::metadata
{


template<>
struct MetadataFor<commands_rtk::GetStatusFlags::StatusFlagsLegacy>
{
    using type = commands_rtk::GetStatusFlags::StatusFlagsLegacy;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { uint32_t(7), "controllerState", "" },
        { uint32_t(248), "platformState", "" },
        { uint32_t(1792), "controllerStatusCode", "" },
        { uint32_t(14336), "platformStatusCode", "" },
        { uint32_t(49152), "resetCode", "" },
        { uint32_t(983040), "signalQuality", "" },
        { uint32_t(4293918720), "reserved", "" },
        { uint32_t(66060288), "rssi", "" },
        { uint32_t(201326592), "rsrp", "" },
        { uint32_t(805306368), "rsrq", "" },
        { uint32_t(3221225472), "sinr", "" },
    };

    static constexpr inline BitfieldInfo value = {
        /* .name    = */ "StatusFlagsLegacy",
        /* .docs    = */ "",
        /* .type    = */ Type::U32,
        /* .entries = */ entries,
    };

};

template<> struct TypeForBitsInfo< &MetadataFor<commands_rtk::GetStatusFlags::StatusFlagsLegacy>::value > { using type = commands_rtk::GetStatusFlags::StatusFlagsLegacy; };

template<>
struct MetadataFor<commands_rtk::GetStatusFlags::StatusFlags>
{
    using type = commands_rtk::GetStatusFlags::StatusFlags;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { uint32_t(15), "modem_state", "" },
        { uint32_t(240), "connection_type", "" },
        { uint32_t(65280), "rssi", "" },
        { uint32_t(983040), "signal_quality", "" },
        { uint32_t(15728640), "tower_change_indicator", "" },
        { uint32_t(16777216), "nmea_timeout", "" },
        { uint32_t(33554432), "server_timeout", "" },
        { uint32_t(67108864), "corrections_timeout", "" },
        { uint32_t(134217728), "device_out_of_range", "" },
        { uint32_t(268435456), "corrections_unavailable", "" },
        { uint32_t(536870912), "reserved", "" },
        { uint32_t(3221225472), "version", "" },
    };

    static constexpr inline BitfieldInfo value = {
        /* .name    = */ "StatusFlags",
        /* .docs    = */ "",
        /* .type    = */ Type::U32,
        /* .entries = */ entries,
    };

};

template<> struct TypeForBitsInfo< &MetadataFor<commands_rtk::GetStatusFlags::StatusFlags>::value > { using type = commands_rtk::GetStatusFlags::StatusFlags; };

template<>
struct MetadataFor<commands_rtk::GetStatusFlags::Response>
{
    using type = commands_rtk::GetStatusFlags::Response;

    using ParamTypes = std::tuple<
        commands_rtk::GetStatusFlags::StatusFlags
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.flags;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "flags",
            /* .docs          = */ "Model number dependent. See above structures.",
            /* .type          = */ {Type::BITS, &MetadataFor<commands_rtk::GetStatusFlags::StatusFlags>::value},
            /* .accessor      = */ nullptr, //utils::access<type, commands_rtk::GetStatusFlags::StatusFlags, &type::flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_rtk::GetStatusFlags::Response",
            /* .title       = */ "response",
            /* .docs        = */ "",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ NO_FUNCTIONS,
            /* .response    = */ nullptr,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_rtk::GetStatusFlags::Response>::value > { using type = commands_rtk::GetStatusFlags::Response; };

template<>
struct MetadataFor<commands_rtk::GetStatusFlags>
{
    using type = commands_rtk::GetStatusFlags;

    using ParamTypes = std::tuple<>;

    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_rtk::GetStatusFlags",
            /* .title       = */ "Get RTK Device Status Flags",
            /* .docs        = */ "",
            /* .parameters  = */ {},
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ NO_FUNCTIONS,
            /* .response    = */ &MetadataFor<type::Response>::value,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_rtk::GetStatusFlags>::value > { using type = commands_rtk::GetStatusFlags; };
template<> struct TypeForDescriptor<commands_rtk::GetStatusFlags::DESCRIPTOR.as_u16()> { using type = commands_rtk::GetStatusFlags; };

template<>
struct MetadataFor<commands_rtk::GetImei::Response>
{
    using type = commands_rtk::GetImei::Response;

    using ParamTypes = std::tuple<
        char
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.IMEI;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "IMEI",
            /* .docs          = */ "",
            /* .type          = */ {Type::CHAR, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, char, &type::IMEI>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 32,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_rtk::GetImei::Response",
            /* .title       = */ "response",
            /* .docs        = */ "",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ NO_FUNCTIONS,
            /* .response    = */ nullptr,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_rtk::GetImei::Response>::value > { using type = commands_rtk::GetImei::Response; };

template<>
struct MetadataFor<commands_rtk::GetImei>
{
    using type = commands_rtk::GetImei;

    using ParamTypes = std::tuple<>;

    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_rtk::GetImei",
            /* .title       = */ "Get RTK Device IMEI (International Mobile Equipment Identifier)",
            /* .docs        = */ "",
            /* .parameters  = */ {},
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ NO_FUNCTIONS,
            /* .response    = */ &MetadataFor<type::Response>::value,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_rtk::GetImei>::value > { using type = commands_rtk::GetImei; };
template<> struct TypeForDescriptor<commands_rtk::GetImei::DESCRIPTOR.as_u16()> { using type = commands_rtk::GetImei; };

template<>
struct MetadataFor<commands_rtk::GetImsi::Response>
{
    using type = commands_rtk::GetImsi::Response;

    using ParamTypes = std::tuple<
        char
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.IMSI;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "IMSI",
            /* .docs          = */ "",
            /* .type          = */ {Type::CHAR, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, char, &type::IMSI>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 32,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_rtk::GetImsi::Response",
            /* .title       = */ "response",
            /* .docs        = */ "",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ NO_FUNCTIONS,
            /* .response    = */ nullptr,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_rtk::GetImsi::Response>::value > { using type = commands_rtk::GetImsi::Response; };

template<>
struct MetadataFor<commands_rtk::GetImsi>
{
    using type = commands_rtk::GetImsi;

    using ParamTypes = std::tuple<>;

    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_rtk::GetImsi",
            /* .title       = */ "Get RTK Device IMSI (International Mobile Subscriber Identifier)",
            /* .docs        = */ "",
            /* .parameters  = */ {},
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ NO_FUNCTIONS,
            /* .response    = */ &MetadataFor<type::Response>::value,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_rtk::GetImsi>::value > { using type = commands_rtk::GetImsi; };
template<> struct TypeForDescriptor<commands_rtk::GetImsi::DESCRIPTOR.as_u16()> { using type = commands_rtk::GetImsi; };

template<>
struct MetadataFor<commands_rtk::GetIccid::Response>
{
    using type = commands_rtk::GetIccid::Response;

    using ParamTypes = std::tuple<
        char
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.ICCID;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "ICCID",
            /* .docs          = */ "",
            /* .type          = */ {Type::CHAR, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, char, &type::ICCID>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 32,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_rtk::GetIccid::Response",
            /* .title       = */ "response",
            /* .docs        = */ "",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ NO_FUNCTIONS,
            /* .response    = */ nullptr,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_rtk::GetIccid::Response>::value > { using type = commands_rtk::GetIccid::Response; };

template<>
struct MetadataFor<commands_rtk::GetIccid>
{
    using type = commands_rtk::GetIccid;

    using ParamTypes = std::tuple<>;

    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_rtk::GetIccid",
            /* .title       = */ "Get RTK Device ICCID (Integrated Circuit Card Identification [SIM Number])",
            /* .docs        = */ "",
            /* .parameters  = */ {},
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ NO_FUNCTIONS,
            /* .response    = */ &MetadataFor<type::Response>::value,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_rtk::GetIccid>::value > { using type = commands_rtk::GetIccid; };
template<> struct TypeForDescriptor<commands_rtk::GetIccid::DESCRIPTOR.as_u16()> { using type = commands_rtk::GetIccid; };

template<>
struct MetadataFor<commands_rtk::ConnectedDeviceType::Type>
{
    using type = commands_rtk::ConnectedDeviceType::Type;

    static constexpr inline EnumInfo::Entry entries[] = {
        { uint32_t(0), "GENERIC", "" },
        { uint32_t(1), "GQ7", "" },
    };

    static constexpr inline EnumInfo value = {
        /* .name    = */ "Type",
        /* .docs    = */ "",
        /* .type    = */ Type::U8,
        /* .entries = */ entries,
    };

};

template<> struct TypeForEnumInfo< &MetadataFor<commands_rtk::ConnectedDeviceType::Type>::value > { using type = commands_rtk::ConnectedDeviceType::Type; };

template<>
struct MetadataFor<commands_rtk::ConnectedDeviceType::Response>
{
    using type = commands_rtk::ConnectedDeviceType::Response;

    using ParamTypes = std::tuple<
        commands_rtk::ConnectedDeviceType::Type
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.devType;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "devType",
            /* .docs          = */ "",
            /* .type          = */ {Type::ENUM, &MetadataFor<commands_rtk::ConnectedDeviceType::Type>::value},
            /* .accessor      = */ nullptr, //utils::access<type, commands_rtk::ConnectedDeviceType::Type, &type::devType>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_rtk::ConnectedDeviceType::Response",
            /* .title       = */ "response",
            /* .docs        = */ "",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ NO_FUNCTIONS,
            /* .response    = */ nullptr,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_rtk::ConnectedDeviceType::Response>::value > { using type = commands_rtk::ConnectedDeviceType::Response; };

template<>
struct MetadataFor<commands_rtk::ConnectedDeviceType>
{
    using type = commands_rtk::ConnectedDeviceType;

    using ParamTypes = std::tuple<
        FunctionSelector,
        commands_rtk::ConnectedDeviceType::Type
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.function;
        if constexpr(I == 1) return value_.devType;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name          = */ "devType",
            /* .docs          = */ "",
            /* .type          = */ {Type::ENUM, &MetadataFor<commands_rtk::ConnectedDeviceType::Type>::value},
            /* .accessor      = */ nullptr, //utils::access<type, commands_rtk::ConnectedDeviceType::Type, &type::devType>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_rtk::ConnectedDeviceType",
            /* .title       = */ "Configure or read the type of the connected device",
            /* .docs        = */ "",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ {true, true, true, true, true},
            /* .response    = */ &MetadataFor<type::Response>::value,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_rtk::ConnectedDeviceType>::value > { using type = commands_rtk::ConnectedDeviceType; };
template<> struct TypeForDescriptor<commands_rtk::ConnectedDeviceType::DESCRIPTOR.as_u16()> { using type = commands_rtk::ConnectedDeviceType; };

template<>
struct MetadataFor<commands_rtk::GetActCode::Response>
{
    using type = commands_rtk::GetActCode::Response;

    using ParamTypes = std::tuple<
        char
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.ActivationCode;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "ActivationCode",
            /* .docs          = */ "",
            /* .type          = */ {Type::CHAR, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, char, &type::ActivationCode>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 32,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_rtk::GetActCode::Response",
            /* .title       = */ "response",
            /* .docs        = */ "",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ NO_FUNCTIONS,
            /* .response    = */ nullptr,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_rtk::GetActCode::Response>::value > { using type = commands_rtk::GetActCode::Response; };

template<>
struct MetadataFor<commands_rtk::GetActCode>
{
    using type = commands_rtk::GetActCode;

    using ParamTypes = std::tuple<>;

    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_rtk::GetActCode",
            /* .title       = */ "Get RTK Device Activation Code",
            /* .docs        = */ "",
            /* .parameters  = */ {},
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ NO_FUNCTIONS,
            /* .response    = */ &MetadataFor<type::Response>::value,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_rtk::GetActCode>::value > { using type = commands_rtk::GetActCode; };
template<> struct TypeForDescriptor<commands_rtk::GetActCode::DESCRIPTOR.as_u16()> { using type = commands_rtk::GetActCode; };

template<>
struct MetadataFor<commands_rtk::GetModemFirmwareVersion::Response>
{
    using type = commands_rtk::GetModemFirmwareVersion::Response;

    using ParamTypes = std::tuple<
        char
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.ModemFirmwareVersion;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "ModemFirmwareVersion",
            /* .docs          = */ "",
            /* .type          = */ {Type::CHAR, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, char, &type::ModemFirmwareVersion>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 32,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_rtk::GetModemFirmwareVersion::Response",
            /* .title       = */ "response",
            /* .docs        = */ "",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ NO_FUNCTIONS,
            /* .response    = */ nullptr,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_rtk::GetModemFirmwareVersion::Response>::value > { using type = commands_rtk::GetModemFirmwareVersion::Response; };

template<>
struct MetadataFor<commands_rtk::GetModemFirmwareVersion>
{
    using type = commands_rtk::GetModemFirmwareVersion;

    using ParamTypes = std::tuple<>;

    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_rtk::GetModemFirmwareVersion",
            /* .title       = */ "Get RTK Device's Cell Modem Firmware version number",
            /* .docs        = */ "",
            /* .parameters  = */ {},
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ NO_FUNCTIONS,
            /* .response    = */ &MetadataFor<type::Response>::value,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_rtk::GetModemFirmwareVersion>::value > { using type = commands_rtk::GetModemFirmwareVersion; };
template<> struct TypeForDescriptor<commands_rtk::GetModemFirmwareVersion::DESCRIPTOR.as_u16()> { using type = commands_rtk::GetModemFirmwareVersion; };

template<>
struct MetadataFor<commands_rtk::GetRssi::Response>
{
    using type = commands_rtk::GetRssi::Response;

    using ParamTypes = std::tuple<
        bool,
        int32_t,
        int32_t
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.valid;
        if constexpr(I == 1) return value_.rssi;
        if constexpr(I == 2) return value_.signalQuality;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "valid",
            /* .docs          = */ "",
            /* .type          = */ {Type::BOOL, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, bool, &type::valid>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "rssi",
            /* .docs          = */ "",
            /* .type          = */ {Type::S32, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, int32_t, &type::rssi>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "signalQuality",
            /* .docs          = */ "",
            /* .type          = */ {Type::S32, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, int32_t, &type::signalQuality>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_rtk::GetRssi::Response",
            /* .title       = */ "response",
            /* .docs        = */ "",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ NO_FUNCTIONS,
            /* .response    = */ nullptr,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_rtk::GetRssi::Response>::value > { using type = commands_rtk::GetRssi::Response; };

template<>
struct MetadataFor<commands_rtk::GetRssi>
{
    using type = commands_rtk::GetRssi;

    using ParamTypes = std::tuple<>;

    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_rtk::GetRssi",
            /* .title       = */ "get_rssi",
            /* .docs        = */ "Get the RSSI and connected/disconnected status of modem",
            /* .parameters  = */ {},
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ NO_FUNCTIONS,
            /* .response    = */ &MetadataFor<type::Response>::value,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_rtk::GetRssi>::value > { using type = commands_rtk::GetRssi; };
template<> struct TypeForDescriptor<commands_rtk::GetRssi::DESCRIPTOR.as_u16()> { using type = commands_rtk::GetRssi; };

template<>
struct MetadataFor<commands_rtk::ServiceStatus::ServiceFlags>
{
    using type = commands_rtk::ServiceStatus::ServiceFlags;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { uint32_t(1), "throttle", "" },
        { uint32_t(2), "corrections_unavailable", "" },
        { uint32_t(252), "reserved", "" },
    };

    static constexpr inline BitfieldInfo value = {
        /* .name    = */ "ServiceFlags",
        /* .docs    = */ "",
        /* .type    = */ Type::U8,
        /* .entries = */ entries,
    };

};

template<> struct TypeForBitsInfo< &MetadataFor<commands_rtk::ServiceStatus::ServiceFlags>::value > { using type = commands_rtk::ServiceStatus::ServiceFlags; };

template<>
struct MetadataFor<commands_rtk::ServiceStatus::Response>
{
    using type = commands_rtk::ServiceStatus::Response;

    using ParamTypes = std::tuple<
        commands_rtk::ServiceStatus::ServiceFlags,
        uint32_t,
        uint32_t,
        uint64_t
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.flags;
        if constexpr(I == 1) return value_.receivedBytes;
        if constexpr(I == 2) return value_.lastBytes;
        if constexpr(I == 3) return value_.lastBytesTime;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "flags",
            /* .docs          = */ "",
            /* .type          = */ {Type::BITS, &MetadataFor<commands_rtk::ServiceStatus::ServiceFlags>::value},
            /* .accessor      = */ nullptr, //utils::access<type, commands_rtk::ServiceStatus::ServiceFlags, &type::flags>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "receivedBytes",
            /* .docs          = */ "",
            /* .type          = */ {Type::U32, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint32_t, &type::receivedBytes>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "lastBytes",
            /* .docs          = */ "",
            /* .type          = */ {Type::U32, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint32_t, &type::lastBytes>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "lastBytesTime",
            /* .docs          = */ "",
            /* .type          = */ {Type::U64, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint64_t, &type::lastBytesTime>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_rtk::ServiceStatus::Response",
            /* .title       = */ "response",
            /* .docs        = */ "",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ NO_FUNCTIONS,
            /* .response    = */ nullptr,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_rtk::ServiceStatus::Response>::value > { using type = commands_rtk::ServiceStatus::Response; };

template<>
struct MetadataFor<commands_rtk::ServiceStatus>
{
    using type = commands_rtk::ServiceStatus;

    using ParamTypes = std::tuple<
        uint32_t,
        uint32_t
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.reserved1;
        if constexpr(I == 1) return value_.reserved2;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "reserved1",
            /* .docs          = */ "",
            /* .type          = */ {Type::U32, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint32_t, &type::reserved1>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "reserved2",
            /* .docs          = */ "",
            /* .type          = */ {Type::U32, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint32_t, &type::reserved2>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_rtk::ServiceStatus",
            /* .title       = */ "service_status",
            /* .docs        = */ "The 3DMRTK will send this message to the server to indicate that the connection should remain open. The Server will respond with information and status.",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ NO_FUNCTIONS,
            /* .response    = */ &MetadataFor<type::Response>::value,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_rtk::ServiceStatus>::value > { using type = commands_rtk::ServiceStatus; };
template<> struct TypeForDescriptor<commands_rtk::ServiceStatus::DESCRIPTOR.as_u16()> { using type = commands_rtk::ServiceStatus; };

template<>
struct MetadataFor<commands_rtk::MediaSelector>
{
    using type = commands_rtk::MediaSelector;

    static constexpr inline EnumInfo::Entry entries[] = {
        { uint32_t(0), "MEDIA_ExternalFlash", "" },
        { uint32_t(1), "MEDIA_SD", "" },
    };

    static constexpr inline EnumInfo value = {
        /* .name    = */ "MediaSelector",
        /* .docs    = */ "",
        /* .type    = */ Type::U8,
        /* .entries = */ entries,
    };

};

template<> struct TypeForEnumInfo< &MetadataFor<commands_rtk::MediaSelector>::value > { using type = commands_rtk::MediaSelector; };

template<>
struct MetadataFor<commands_rtk::ProdEraseStorage>
{
    using type = commands_rtk::ProdEraseStorage;

    using ParamTypes = std::tuple<
        commands_rtk::MediaSelector
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.media;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "media",
            /* .docs          = */ "",
            /* .type          = */ {Type::ENUM, &MetadataFor<commands_rtk::MediaSelector>::value},
            /* .accessor      = */ nullptr, //utils::access<type, commands_rtk::MediaSelector, &type::media>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_rtk::ProdEraseStorage",
            /* .title       = */ "prod_erase_storage",
            /* .docs        = */ "This command will erase the selected media to a raw and uninitialized state. ALL DATA WILL BE LOST.\nThis command is only available in calibration mode.",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ NO_FUNCTIONS,
            /* .response    = */ nullptr,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_rtk::ProdEraseStorage>::value > { using type = commands_rtk::ProdEraseStorage; };
template<> struct TypeForDescriptor<commands_rtk::ProdEraseStorage::DESCRIPTOR.as_u16()> { using type = commands_rtk::ProdEraseStorage; };

template<>
struct MetadataFor<commands_rtk::LedAction>
{
    using type = commands_rtk::LedAction;

    static constexpr inline EnumInfo::Entry entries[] = {
        { uint32_t(0), "LED_NONE", "" },
        { uint32_t(1), "LED_FLASH", "" },
        { uint32_t(2), "LED_PULSATE", "" },
    };

    static constexpr inline EnumInfo value = {
        /* .name    = */ "LedAction",
        /* .docs    = */ "",
        /* .type    = */ Type::U8,
        /* .entries = */ entries,
    };

};

template<> struct TypeForEnumInfo< &MetadataFor<commands_rtk::LedAction>::value > { using type = commands_rtk::LedAction; };

template<>
struct MetadataFor<commands_rtk::LedControl>
{
    using type = commands_rtk::LedControl;

    using ParamTypes = std::tuple<
        uint8_t,
        uint8_t,
        commands_rtk::LedAction,
        uint32_t
    >;

    template<size_t I, class T = type>
    static auto& access(T& value_) {
        if constexpr(I == 0) return value_.primaryColor;
        if constexpr(I == 1) return value_.altColor;
        if constexpr(I == 2) return value_.act;
        if constexpr(I == 3) return value_.period;
    }
    
    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "primaryColor",
            /* .docs          = */ "",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::primaryColor>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 3,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "altColor",
            /* .docs          = */ "",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::altColor>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 3,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "act",
            /* .docs          = */ "",
            /* .type          = */ {Type::ENUM, &MetadataFor<commands_rtk::LedAction>::value},
            /* .accessor      = */ nullptr, //utils::access<type, commands_rtk::LedAction, &type::act>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "period",
            /* .docs          = */ "",
            /* .type          = */ {Type::U32, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint32_t, &type::period>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };
    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_rtk::LedControl",
            /* .title       = */ "led_control",
            /* .docs        = */ "This command allows direct control of the LED on the 3DM RTK. This command is only available in calibration mode or Production Test Mode.",
            /* .parameters  = */ parameters,
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ NO_FUNCTIONS,
            /* .response    = */ nullptr,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_rtk::LedControl>::value > { using type = commands_rtk::LedControl; };
template<> struct TypeForDescriptor<commands_rtk::LedControl::DESCRIPTOR.as_u16()> { using type = commands_rtk::LedControl; };

template<>
struct MetadataFor<commands_rtk::ModemHardReset>
{
    using type = commands_rtk::ModemHardReset;

    using ParamTypes = std::tuple<>;

    static constexpr inline FieldInfo value = {
        {
            /* .name        = */ "commands_rtk::ModemHardReset",
            /* .title       = */ "modem_hard_reset",
            /* .docs        = */ "This command will clear the modem flash.  THIS MUST NOT BE DONE OFTEN AS IT CAN DAMAGE THE FLASH!\nThis command is only available in calibration mode.",
            /* .parameters  = */ {},
        },
            /* .descriptor  = */ type::DESCRIPTOR,
            /* .functions   = */ NO_FUNCTIONS,
            /* .response    = */ nullptr,
    };
};

template<> struct TypeForFieldInfo< &MetadataFor<commands_rtk::ModemHardReset>::value > { using type = commands_rtk::ModemHardReset; };
template<> struct TypeForDescriptor<commands_rtk::ModemHardReset::DESCRIPTOR.as_u16()> { using type = commands_rtk::ModemHardReset; };


static constexpr inline const FieldInfo* COMMANDS_RTK_FIELDS[] = {
    &MetadataFor<commands_rtk::GetStatusFlags>::value,
    &MetadataFor<commands_rtk::GetImei>::value,
    &MetadataFor<commands_rtk::GetImsi>::value,
    &MetadataFor<commands_rtk::GetIccid>::value,
    &MetadataFor<commands_rtk::GetRssi>::value,
    &MetadataFor<commands_rtk::ConnectedDeviceType>::value,
    &MetadataFor<commands_rtk::GetActCode>::value,
    &MetadataFor<commands_rtk::GetModemFirmwareVersion>::value,
    &MetadataFor<commands_rtk::ServiceStatus>::value,
    &MetadataFor<commands_rtk::ProdEraseStorage>::value,
    &MetadataFor<commands_rtk::LedControl>::value,
    &MetadataFor<commands_rtk::ModemHardReset>::value,
    &MetadataFor<commands_rtk::GetStatusFlags::Response>::value,
    &MetadataFor<commands_rtk::GetImei::Response>::value,
    &MetadataFor<commands_rtk::GetImsi::Response>::value,
    &MetadataFor<commands_rtk::GetIccid::Response>::value,
    &MetadataFor<commands_rtk::GetRssi::Response>::value,
    &MetadataFor<commands_rtk::ConnectedDeviceType::Response>::value,
    &MetadataFor<commands_rtk::GetActCode::Response>::value,
    &MetadataFor<commands_rtk::GetModemFirmwareVersion::Response>::value,
    &MetadataFor<commands_rtk::ServiceStatus::Response>::value,
};

//namespace commands_rtk
//{
struct CommandSetRtk
{
    static inline constexpr uint8_t DESCRIPTOR_SET = commands_rtk::DESCRIPTOR_SET;
    static inline constexpr CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, INVALID_FIELD_DESCRIPTOR};

    using Fields = std::tuple<
        ::mip::commands_rtk::GetStatusFlags,
        ::mip::commands_rtk::GetImei,
        ::mip::commands_rtk::GetImsi,
        ::mip::commands_rtk::GetIccid,
        ::mip::commands_rtk::GetRssi,
        ::mip::commands_rtk::ConnectedDeviceType,
        ::mip::commands_rtk::GetActCode,
        ::mip::commands_rtk::GetModemFirmwareVersion,
        ::mip::commands_rtk::ServiceStatus,
        ::mip::commands_rtk::ProdEraseStorage,
        ::mip::commands_rtk::LedControl,
        ::mip::commands_rtk::ModemHardReset,
        ::mip::commands_rtk::GetStatusFlags::Response,
        ::mip::commands_rtk::GetImei::Response,
        ::mip::commands_rtk::GetImsi::Response,
        ::mip::commands_rtk::GetIccid::Response,
        ::mip::commands_rtk::GetRssi::Response,
        ::mip::commands_rtk::ConnectedDeviceType::Response,
        ::mip::commands_rtk::GetActCode::Response,
        ::mip::commands_rtk::GetModemFirmwareVersion::Response,
        ::mip::commands_rtk::ServiceStatus::Response
    >;
};

//} // namespace commands_rtk

template<>
struct MetadataFor<CommandSetRtk>
{
    using type = CommandSetRtk;
    
    static inline constexpr DescriptorSetInfo value = {
        /* .descriptor = */ commands_rtk::DESCRIPTOR_SET,
        /* .name       = */ "Rtk Commands",
        /* .fields     = */ COMMANDS_RTK_FIELDS,
    };
};
template<> struct TypeForDescriptor< (commands_rtk::DESCRIPTOR_SET << 8) > { using type = CommandSetRtk; };

static constexpr DescriptorSetInfo COMMANDS_RTK = {
    /* .descriptor = */ mip::commands_rtk::DESCRIPTOR_SET,
    /* .name       = */ "Rtk Commands",
    /* .fields     = */ COMMANDS_RTK_FIELDS,
};

} // namespace mip::metadata

