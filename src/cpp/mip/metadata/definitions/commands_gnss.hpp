#pragma once

#include <mip/metadata/definitions/common.hpp>

#include <mip/definitions/commands_gnss.hpp>


#include <mip/metadata/mip_metadata.hpp>

namespace mip::metadata
{


template<>
struct MetadataFor<commands_gnss::ReceiverInfo::Info>
{
    using type = commands_gnss::ReceiverInfo::Info;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "receiver_id",
            /* .docs          = */ "Receiver id: e.g. 1, 2, etc.",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::receiver_id>,
            /* .attributes    = */ NO_FUNCTIONS,
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "mip_data_descriptor_set",
            /* .docs          = */ "MIP descriptor set associated with this receiver",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::mip_data_descriptor_set>,
            /* .attributes    = */ NO_FUNCTIONS,
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "description",
            /* .docs          = */ "Ascii description of receiver. Contains the following info (comma-delimited):<br/>\nModule name/model<br/>\nFirmware version info",
            /* .type          = */ {Type::CHAR, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, char, &type::description>,
            /* .attributes    = */ NO_FUNCTIONS,
            /* .count         = */ 32,
            /* .condition     = */ {},
        },
    };

    static constexpr inline StructInfo value = {
        /* .name        = */ "Info",
        /* .title       = */ "Info",
        /* .docs        = */ "",
        /* .parameters  = */ parameters,
    };
};

template<>
struct MetadataFor<commands_gnss::ReceiverInfo::Response>
{
    using type = commands_gnss::ReceiverInfo::Response;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "num_receivers",
            /* .docs          = */ "Number of physical receivers in the device",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::num_receivers>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "receiver_info",
            /* .docs          = */ "",
            /* .type          = */ {Type::STRUCT, &MetadataFor<commands_gnss::ReceiverInfo::Info>::value},
            /* .accessor      = */ nullptr, //utils::access<type, commands_gnss::ReceiverInfo::Info, &type::receiver_info>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ {5, microstrain::Index(0) /* num_receivers */},
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "commands_gnss::ReceiverInfo::Response",
        /* .title       = */ "response",
        /* .docs        = */ "",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<commands_gnss::ReceiverInfo>
{
    using type = commands_gnss::ReceiverInfo;

    static constexpr inline FieldInfo value = {
        /* .name        = */ "commands_gnss::ReceiverInfo",
        /* .title       = */ "receiver_info",
        /* .docs        = */ "Return information about the GNSS receivers in the device.\n",
        /* .parameters  = */ {},
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ &MetadataFor<type::Response>::value,
    };
};

template<>
struct MetadataFor<commands_gnss::SignalConfiguration::Response>
{
    using type = commands_gnss::SignalConfiguration::Response;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "gps_enable",
            /* .docs          = */ "Bitfield 0: Enable L1CA, 1: Enable L2C, 2: Enable L5",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::gps_enable>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "glonass_enable",
            /* .docs          = */ "Bitfield 0: Enable L1OF, 1: Enable L2OF",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::glonass_enable>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "galileo_enable",
            /* .docs          = */ "Bitfield 0: Enable E1,   1: Enable E5B, 2: Enable E5A",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::galileo_enable>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "beidou_enable",
            /* .docs          = */ "Bitfield 0: Enable B1,   1: Enable B2,  2: Enable B2A",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::beidou_enable>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "reserved",
            /* .docs          = */ "",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::reserved>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 4,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "commands_gnss::SignalConfiguration::Response",
        /* .title       = */ "response",
        /* .docs        = */ "",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<commands_gnss::SignalConfiguration>
{
    using type = commands_gnss::SignalConfiguration;

    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name          = */ "gps_enable",
            /* .docs          = */ "Bitfield 0: Enable L1CA, 1: Enable L2C, 2: Enable L5",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::gps_enable>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "glonass_enable",
            /* .docs          = */ "Bitfield 0: Enable L1OF, 1: Enable L2OF",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::glonass_enable>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "galileo_enable",
            /* .docs          = */ "Bitfield 0: Enable E1,   1: Enable E5B, 2: Enable E5A",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::galileo_enable>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "beidou_enable",
            /* .docs          = */ "Bitfield 0: Enable B1,   1: Enable B2,  2: Enable B2A",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::beidou_enable>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "reserved",
            /* .docs          = */ "",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::reserved>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 4,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "commands_gnss::SignalConfiguration",
        /* .title       = */ "signal_configuration",
        /* .docs        = */ "Configure the GNSS signals used by the device.\n",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ {true, true, true, true, true},
        /* .response    = */ &MetadataFor<type::Response>::value,
    };
};

template<>
struct MetadataFor<commands_gnss::SpartnConfiguration::Response>
{
    using type = commands_gnss::SpartnConfiguration::Response;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "enable",
            /* .docs          = */ "Enable/Disable the SPARTN subsystem (0 = Disabled, 1 = Enabled)",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::enable>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "type",
            /* .docs          = */ "Connection type (0 - None, 1 = Network, 2 = L-Band)",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::type>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "current_key_tow",
            /* .docs          = */ "The GPS time of week the current key is valid until",
            /* .type          = */ {Type::U32, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint32_t, &type::current_key_tow>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "current_key_week",
            /* .docs          = */ "The GPS week number the current key is valid until",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint16_t, &type::current_key_week>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "current_key",
            /* .docs          = */ "32 character string of ASCII hex values for the current key (e.g. 'bc' for 0xBC)",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::current_key>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 32,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "next_key_tow",
            /* .docs          = */ "The GPS time of week the next key is valid until",
            /* .type          = */ {Type::U32, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint32_t, &type::next_key_tow>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "next_key_week",
            /* .docs          = */ "The GPS week number the next key is valid until",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint16_t, &type::next_key_week>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "next_key",
            /* .docs          = */ "32 character string of ASCII hex valuesfor the next key (e.g. 'bc' for 0xBC)",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::next_key>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 32,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "commands_gnss::SpartnConfiguration::Response",
        /* .title       = */ "response",
        /* .docs        = */ "",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<commands_gnss::SpartnConfiguration>
{
    using type = commands_gnss::SpartnConfiguration;

    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name          = */ "enable",
            /* .docs          = */ "Enable/Disable the SPARTN subsystem (0 = Disabled, 1 = Enabled)",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::enable>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "type",
            /* .docs          = */ "Connection type (0 - None, 1 = Network, 2 = L-Band)",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::type>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "current_key_tow",
            /* .docs          = */ "The GPS time of week the current key is valid until",
            /* .type          = */ {Type::U32, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint32_t, &type::current_key_tow>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "current_key_week",
            /* .docs          = */ "The GPS week number the current key is valid until",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint16_t, &type::current_key_week>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "current_key",
            /* .docs          = */ "32 character string of ASCII hex values for the current key (e.g. 'bc' for 0xBC)",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::current_key>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 32,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "next_key_tow",
            /* .docs          = */ "The GPS time of week the next key is valid until",
            /* .type          = */ {Type::U32, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint32_t, &type::next_key_tow>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "next_key_week",
            /* .docs          = */ "The GPS week number the next key is valid until",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint16_t, &type::next_key_week>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "next_key",
            /* .docs          = */ "32 character string of ASCII hex valuesfor the next key (e.g. 'bc' for 0xBC)",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::next_key>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 32,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "commands_gnss::SpartnConfiguration",
        /* .title       = */ "spartn_configuration",
        /* .docs        = */ "Configure the SPARTN corrections service parameters.\nNotes:<br/>\n- Enable and type settings will only update after a power cycle <br/>\n- Type settings will only take effect after a power cycle <br/>\n- Key information can be updated while running",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ {true, true, true, true, true},
        /* .response    = */ &MetadataFor<type::Response>::value,
    };
};

template<>
struct MetadataFor<commands_gnss::RtkDongleConfiguration::Response>
{
    using type = commands_gnss::RtkDongleConfiguration::Response;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "enable",
            /* .docs          = */ "",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::enable>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "reserved",
            /* .docs          = */ "",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::reserved>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 3,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "commands_gnss::RtkDongleConfiguration::Response",
        /* .title       = */ "response",
        /* .docs        = */ "",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<commands_gnss::RtkDongleConfiguration>
{
    using type = commands_gnss::RtkDongleConfiguration;

    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name          = */ "enable",
            /* .docs          = */ "0 - Disabled, 1- Enabled",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::enable>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "reserved",
            /* .docs          = */ "",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::reserved>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 3,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "commands_gnss::RtkDongleConfiguration",
        /* .title       = */ "rtk_dongle_configuration",
        /* .docs        = */ "Configure the communications with the RTK Dongle connected to the device.\n",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ {true, true, true, true, true},
        /* .response    = */ &MetadataFor<type::Response>::value,
    };
};


static constexpr inline const FieldInfo* COMMANDS_GNSS_FIELDS[] = {
    &MetadataFor<commands_gnss::ReceiverInfo>::value,
    &MetadataFor<commands_gnss::SignalConfiguration>::value,
    &MetadataFor<commands_gnss::RtkDongleConfiguration>::value,
    &MetadataFor<commands_gnss::SpartnConfiguration>::value,
    &MetadataFor<commands_gnss::ReceiverInfo::Response>::value,
    &MetadataFor<commands_gnss::SignalConfiguration::Response>::value,
    &MetadataFor<commands_gnss::RtkDongleConfiguration::Response>::value,
    &MetadataFor<commands_gnss::SpartnConfiguration::Response>::value,
};

static constexpr DescriptorSetInfo COMMANDS_GNSS = {
    /*.descriptor =*/ mip::commands_gnss::DESCRIPTOR_SET,
    /*.name       =*/ "Gnss Commands",
    /*.fields     =*/ COMMANDS_GNSS_FIELDS,
};

} // namespace mip::metadata

