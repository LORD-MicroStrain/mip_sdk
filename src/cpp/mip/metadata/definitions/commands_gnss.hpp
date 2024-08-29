#pragma once

#include "common.hpp"

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
            /* .accessor      = */ utils::access<type, uint8_t, &type::receiver_id>,
            /* .functions     = */ NO_FUNCTIONS,
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "mip_data_descriptor_set",
            /* .docs          = */ "MIP descriptor set associated with this receiver",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ utils::access<type, uint8_t, &type::mip_data_descriptor_set>,
            /* .functions     = */ NO_FUNCTIONS,
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "description",
            /* .docs          = */ "Ascii description of receiver. Contains the following info (comma-delimited):<br/>\nModule name/model<br/>\nFirmware version info",
            /* .type          = */ {Type::CHAR, nullptr},
            /* .accessor      = */ utils::access<type, char, &type::description>,
            /* .functions     = */ NO_FUNCTIONS,
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
            /* .accessor      = */ utils::access<type, uint8_t, &type::num_receivers>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "receiver_info",
            /* .docs          = */ "",
            /* .type          = */ {Type::STRUCT, &MetadataFor<commands_gnss::ReceiverInfo::Info>::value},
            /* .accessor      = */ utils::access<type, commands_gnss::ReceiverInfo::Info, &type::receiver_info>,
            /* .functions     = */ {true, false, false, false, false,  true},
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
        /* .proprietary = */ false,
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
        /* .proprietary = */ false,
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
            /* .docs          = */ "Bitfield 0: Enable L1CA, 1: Enable L2C",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ utils::access<type, uint8_t, &type::gps_enable>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "glonass_enable",
            /* .docs          = */ "Bitfield 0: Enable L1OF, 1: Enable L2OF",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ utils::access<type, uint8_t, &type::glonass_enable>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "galileo_enable",
            /* .docs          = */ "Bitfield 0: Enable E1,   1: Enable E5B",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ utils::access<type, uint8_t, &type::galileo_enable>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "beidou_enable",
            /* .docs          = */ "Bitfield 0: Enable B1,   1: Enable B2",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ utils::access<type, uint8_t, &type::beidou_enable>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "reserved",
            /* .docs          = */ "",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ utils::access<type, uint8_t, &type::reserved>,
            /* .functions     = */ {true, false, false, false, false,  true},
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
        /* .proprietary = */ false,
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
            /* .docs          = */ "Bitfield 0: Enable L1CA, 1: Enable L2C",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ utils::access<type, uint8_t, &type::gps_enable>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "glonass_enable",
            /* .docs          = */ "Bitfield 0: Enable L1OF, 1: Enable L2OF",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ utils::access<type, uint8_t, &type::glonass_enable>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "galileo_enable",
            /* .docs          = */ "Bitfield 0: Enable E1,   1: Enable E5B",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ utils::access<type, uint8_t, &type::galileo_enable>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "beidou_enable",
            /* .docs          = */ "Bitfield 0: Enable B1,   1: Enable B2",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ utils::access<type, uint8_t, &type::beidou_enable>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "reserved",
            /* .docs          = */ "",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ utils::access<type, uint8_t, &type::reserved>,
            /* .functions     = */ {true, false, false, false, false,  true},
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
        /* .functions   = */ {true, true, true, true, true,  true},
        /* .proprietary = */ false,
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
            /* .accessor      = */ utils::access<type, uint8_t, &type::enable>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "reserved",
            /* .docs          = */ "",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ utils::access<type, uint8_t, &type::reserved>,
            /* .functions     = */ {true, false, false, false, false,  true},
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
        /* .proprietary = */ false,
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
            /* .accessor      = */ utils::access<type, uint8_t, &type::enable>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "reserved",
            /* .docs          = */ "",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ utils::access<type, uint8_t, &type::reserved>,
            /* .functions     = */ {true, false, false, false, false,  true},
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
        /* .functions   = */ {true, true, true, true, true,  true},
        /* .proprietary = */ false,
        /* .response    = */ &MetadataFor<type::Response>::value,
    };
};


static constexpr inline const FieldInfo* COMMANDS_GNSS_FIELDS[] = {
    &MetadataFor<commands_gnss::ReceiverInfo>::value,
    &MetadataFor<commands_gnss::SignalConfiguration>::value,
    &MetadataFor<commands_gnss::RtkDongleConfiguration>::value,
    &MetadataFor<commands_gnss::ReceiverInfo::Response>::value,
    &MetadataFor<commands_gnss::SignalConfiguration::Response>::value,
    &MetadataFor<commands_gnss::RtkDongleConfiguration::Response>::value,
};

static constexpr inline const DescriptorSet COMMANDS_GNSS_DS(commands_gnss::DESCRIPTOR_SET, "commands_gnss", COMMANDS_GNSS_FIELDS);

} // namespace mip::metadata

