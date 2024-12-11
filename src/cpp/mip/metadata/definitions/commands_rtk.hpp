#pragma once

#include <mip/metadata/definitions/common.hpp>

#include <mip/definitions/commands_rtk.hpp>


#include <mip/metadata/mip_metadata.hpp>

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

template<>
struct MetadataFor<commands_rtk::GetStatusFlags::Response>
{
    using type = commands_rtk::GetStatusFlags::Response;

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
        /* .name        = */ "commands_rtk::GetStatusFlags::Response",
        /* .title       = */ "response",
        /* .docs        = */ "",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<commands_rtk::GetStatusFlags>
{
    using type = commands_rtk::GetStatusFlags;

    static constexpr inline FieldInfo value = {
        /* .name        = */ "commands_rtk::GetStatusFlags",
        /* .title       = */ "Get RTK Device Status Flags",
        /* .docs        = */ "",
        /* .parameters  = */ {},
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ &MetadataFor<type::Response>::value,
    };
};

template<>
struct MetadataFor<commands_rtk::GetImei::Response>
{
    using type = commands_rtk::GetImei::Response;

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
        /* .name        = */ "commands_rtk::GetImei::Response",
        /* .title       = */ "response",
        /* .docs        = */ "",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<commands_rtk::GetImei>
{
    using type = commands_rtk::GetImei;

    static constexpr inline FieldInfo value = {
        /* .name        = */ "commands_rtk::GetImei",
        /* .title       = */ "Get RTK Device IMEI (International Mobile Equipment Identifier)",
        /* .docs        = */ "",
        /* .parameters  = */ {},
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ &MetadataFor<type::Response>::value,
    };
};

template<>
struct MetadataFor<commands_rtk::GetImsi::Response>
{
    using type = commands_rtk::GetImsi::Response;

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
        /* .name        = */ "commands_rtk::GetImsi::Response",
        /* .title       = */ "response",
        /* .docs        = */ "",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<commands_rtk::GetImsi>
{
    using type = commands_rtk::GetImsi;

    static constexpr inline FieldInfo value = {
        /* .name        = */ "commands_rtk::GetImsi",
        /* .title       = */ "Get RTK Device IMSI (International Mobile Subscriber Identifier)",
        /* .docs        = */ "",
        /* .parameters  = */ {},
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ &MetadataFor<type::Response>::value,
    };
};

template<>
struct MetadataFor<commands_rtk::GetIccid::Response>
{
    using type = commands_rtk::GetIccid::Response;

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
        /* .name        = */ "commands_rtk::GetIccid::Response",
        /* .title       = */ "response",
        /* .docs        = */ "",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<commands_rtk::GetIccid>
{
    using type = commands_rtk::GetIccid;

    static constexpr inline FieldInfo value = {
        /* .name        = */ "commands_rtk::GetIccid",
        /* .title       = */ "Get RTK Device ICCID (Integrated Circuit Card Identification [SIM Number])",
        /* .docs        = */ "",
        /* .parameters  = */ {},
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ &MetadataFor<type::Response>::value,
    };
};

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

template<>
struct MetadataFor<commands_rtk::ConnectedDeviceType::Response>
{
    using type = commands_rtk::ConnectedDeviceType::Response;

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
        /* .name        = */ "commands_rtk::ConnectedDeviceType::Response",
        /* .title       = */ "response",
        /* .docs        = */ "",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<commands_rtk::ConnectedDeviceType>
{
    using type = commands_rtk::ConnectedDeviceType;

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
        /* .name        = */ "commands_rtk::ConnectedDeviceType",
        /* .title       = */ "Configure or read the type of the connected device",
        /* .docs        = */ "",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ {true, true, true, true, true},
        /* .response    = */ &MetadataFor<type::Response>::value,
    };
};

template<>
struct MetadataFor<commands_rtk::GetActCode::Response>
{
    using type = commands_rtk::GetActCode::Response;

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
        /* .name        = */ "commands_rtk::GetActCode::Response",
        /* .title       = */ "response",
        /* .docs        = */ "",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<commands_rtk::GetActCode>
{
    using type = commands_rtk::GetActCode;

    static constexpr inline FieldInfo value = {
        /* .name        = */ "commands_rtk::GetActCode",
        /* .title       = */ "Get RTK Device Activation Code",
        /* .docs        = */ "",
        /* .parameters  = */ {},
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ &MetadataFor<type::Response>::value,
    };
};

template<>
struct MetadataFor<commands_rtk::GetModemFirmwareVersion::Response>
{
    using type = commands_rtk::GetModemFirmwareVersion::Response;

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
        /* .name        = */ "commands_rtk::GetModemFirmwareVersion::Response",
        /* .title       = */ "response",
        /* .docs        = */ "",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<commands_rtk::GetModemFirmwareVersion>
{
    using type = commands_rtk::GetModemFirmwareVersion;

    static constexpr inline FieldInfo value = {
        /* .name        = */ "commands_rtk::GetModemFirmwareVersion",
        /* .title       = */ "Get RTK Device's Cell Modem Firmware version number",
        /* .docs        = */ "",
        /* .parameters  = */ {},
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ &MetadataFor<type::Response>::value,
    };
};

template<>
struct MetadataFor<commands_rtk::GetRssi::Response>
{
    using type = commands_rtk::GetRssi::Response;

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
        /* .name        = */ "commands_rtk::GetRssi::Response",
        /* .title       = */ "response",
        /* .docs        = */ "",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<commands_rtk::GetRssi>
{
    using type = commands_rtk::GetRssi;

    static constexpr inline FieldInfo value = {
        /* .name        = */ "commands_rtk::GetRssi",
        /* .title       = */ "get_rssi",
        /* .docs        = */ "Get the RSSI and connected/disconnected status of modem",
        /* .parameters  = */ {},
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ &MetadataFor<type::Response>::value,
    };
};

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

template<>
struct MetadataFor<commands_rtk::ServiceStatus::Response>
{
    using type = commands_rtk::ServiceStatus::Response;

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
        /* .name        = */ "commands_rtk::ServiceStatus::Response",
        /* .title       = */ "response",
        /* .docs        = */ "",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<commands_rtk::ServiceStatus>
{
    using type = commands_rtk::ServiceStatus;

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
        /* .name        = */ "commands_rtk::ServiceStatus",
        /* .title       = */ "service_status",
        /* .docs        = */ "The 3DMRTK will send this message to the server to indicate that the connection should remain open. The Server will respond with information and status.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ &MetadataFor<type::Response>::value,
    };
};

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

template<>
struct MetadataFor<commands_rtk::ProdEraseStorage>
{
    using type = commands_rtk::ProdEraseStorage;

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
        /* .name        = */ "commands_rtk::ProdEraseStorage",
        /* .title       = */ "prod_erase_storage",
        /* .docs        = */ "This command will erase the selected media to a raw and uninitialized state. ALL DATA WILL BE LOST.\nThis command is only available in calibration mode.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

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

template<>
struct MetadataFor<commands_rtk::LedControl>
{
    using type = commands_rtk::LedControl;

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
        /* .name        = */ "commands_rtk::LedControl",
        /* .title       = */ "led_control",
        /* .docs        = */ "This command allows direct control of the LED on the 3DM RTK. This command is only available in calibration mode or Production Test Mode.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<commands_rtk::ModemHardReset>
{
    using type = commands_rtk::ModemHardReset;

    static constexpr inline FieldInfo value = {
        /* .name        = */ "commands_rtk::ModemHardReset",
        /* .title       = */ "modem_hard_reset",
        /* .docs        = */ "This command will clear the modem flash.  THIS MUST NOT BE DONE OFTEN AS IT CAN DAMAGE THE FLASH!\nThis command is only available in calibration mode.",
        /* .parameters  = */ {},
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};


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

static constexpr DescriptorSetInfo COMMANDS_RTK = {
    /*.descriptor =*/ mip::commands_rtk::DESCRIPTOR_SET,
    /*.name       =*/ "Rtk Commands",
    /*.fields     =*/ COMMANDS_RTK_FIELDS,
};

} // namespace mip::metadata

