#pragma once

#include <mip/metadata/definitions/common.hpp>

#include <mip/definitions/commands_base.hpp>


#include <mip/metadata/mip_metadata.hpp>

namespace mip::metadata
{


template<>
struct MetadataFor<commands_base::Ping>
{
    using type = commands_base::Ping;

    static constexpr inline FieldInfo value = {
        /* .name        = */ "commands_base::Ping",
        /* .title       = */ "Ping",
        /* .docs        = */ "Test Communications with a device.\n\nThe Device will respond with an ACK, if present and operating correctly.\n\nIf the device is not in a normal operating mode, it may NACK.",
        /* .parameters  = */ {},
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<commands_base::SetIdle>
{
    using type = commands_base::SetIdle;

    static constexpr inline FieldInfo value = {
        /* .name        = */ "commands_base::SetIdle",
        /* .title       = */ "Set to idle",
        /* .docs        = */ "Turn off all device data streams.\n\nThe Device will respond with an ACK, if present and operating correctly.\nThis command will suspend streaming (if enabled) or wake the device from sleep (if sleeping) to allow it to respond to status and setup commands.\nYou may restore the device mode by issuing the Resume command.",
        /* .parameters  = */ {},
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<commands_base::BaseDeviceInfo>
{
    using type = commands_base::BaseDeviceInfo;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "firmware_version",
            /* .docs          = */ "",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint16_t, &type::firmware_version>,
            /* .attributes    = */ NO_FUNCTIONS,
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "model_name",
            /* .docs          = */ "",
            /* .type          = */ {Type::CHAR, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, char, &type::model_name>,
            /* .attributes    = */ NO_FUNCTIONS,
            /* .count         = */ 16,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "model_number",
            /* .docs          = */ "",
            /* .type          = */ {Type::CHAR, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, char, &type::model_number>,
            /* .attributes    = */ NO_FUNCTIONS,
            /* .count         = */ 16,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "serial_number",
            /* .docs          = */ "",
            /* .type          = */ {Type::CHAR, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, char, &type::serial_number>,
            /* .attributes    = */ NO_FUNCTIONS,
            /* .count         = */ 16,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "lot_number",
            /* .docs          = */ "",
            /* .type          = */ {Type::CHAR, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, char, &type::lot_number>,
            /* .attributes    = */ NO_FUNCTIONS,
            /* .count         = */ 16,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "device_options",
            /* .docs          = */ "",
            /* .type          = */ {Type::CHAR, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, char, &type::device_options>,
            /* .attributes    = */ NO_FUNCTIONS,
            /* .count         = */ 16,
            /* .condition     = */ {},
        },
    };

    static constexpr inline StructInfo value = {
        /* .name        = */ "BaseDeviceInfo",
        /* .title       = */ "Base Device Info",
        /* .docs        = */ "",
        /* .parameters  = */ parameters,
    };
};

template<>
struct MetadataFor<commands_base::GetDeviceInfo::Response>
{
    using type = commands_base::GetDeviceInfo::Response;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "device_info",
            /* .docs          = */ "",
            /* .type          = */ {Type::STRUCT, &MetadataFor<commands_base::BaseDeviceInfo>::value},
            /* .accessor      = */ nullptr, //utils::access<type, commands_base::BaseDeviceInfo, &type::device_info>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "commands_base::GetDeviceInfo::Response",
        /* .title       = */ "response",
        /* .docs        = */ "",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<commands_base::GetDeviceInfo>
{
    using type = commands_base::GetDeviceInfo;

    static constexpr inline FieldInfo value = {
        /* .name        = */ "commands_base::GetDeviceInfo",
        /* .title       = */ "Get device information",
        /* .docs        = */ "Get the device ID strings and firmware version number.",
        /* .parameters  = */ {},
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ &MetadataFor<type::Response>::value,
    };
};

template<>
struct MetadataFor<commands_base::GetDeviceDescriptors::Response>
{
    using type = commands_base::GetDeviceDescriptors::Response;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "descriptors",
            /* .docs          = */ "",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint16_t, &type::descriptors>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ {0, microstrain::Index(1) /* descriptors_count */},
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "descriptors_count",
            /* .docs          = */ "",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::descriptors_count>,
            /* .attributes    = */ {false, false, false, false, false, /*echo*/false, /*virtual*/true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "commands_base::GetDeviceDescriptors::Response",
        /* .title       = */ "response",
        /* .docs        = */ "",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<commands_base::GetDeviceDescriptors>
{
    using type = commands_base::GetDeviceDescriptors;

    static constexpr inline FieldInfo value = {
        /* .name        = */ "commands_base::GetDeviceDescriptors",
        /* .title       = */ "Get device descriptors",
        /* .docs        = */ "Get the command and data descriptors supported by the device.\n\nReply has two fields: 'ACK/NACK' and 'Descriptors'. The 'Descriptors' field is an array of 16 bit values.\nThe MSB specifies the descriptor set and the LSB specifies the descriptor.",
        /* .parameters  = */ {},
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ &MetadataFor<type::Response>::value,
    };
};

template<>
struct MetadataFor<commands_base::BuiltInTest::Response>
{
    using type = commands_base::BuiltInTest::Response;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "result",
            /* .docs          = */ "",
            /* .type          = */ {Type::U32, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint32_t, &type::result>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "commands_base::BuiltInTest::Response",
        /* .title       = */ "response",
        /* .docs        = */ "",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<commands_base::BuiltInTest>
{
    using type = commands_base::BuiltInTest;

    static constexpr inline FieldInfo value = {
        /* .name        = */ "commands_base::BuiltInTest",
        /* .title       = */ "Built in test",
        /* .docs        = */ "Run the device Built-In Test (BIT).\n\nThe Built-In Test command always returns a 32 bit value.\nA value of 0 means that all tests passed.\nA non-zero value indicates that not all tests passed.\nReference the device user manual to decode the result.",
        /* .parameters  = */ {},
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ &MetadataFor<type::Response>::value,
    };
};

template<>
struct MetadataFor<commands_base::Resume>
{
    using type = commands_base::Resume;

    static constexpr inline FieldInfo value = {
        /* .name        = */ "commands_base::Resume",
        /* .title       = */ "Resume",
        /* .docs        = */ "Take the device out of idle mode.\n\nThe device responds with ACK upon success.",
        /* .parameters  = */ {},
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<commands_base::GetExtendedDescriptors::Response>
{
    using type = commands_base::GetExtendedDescriptors::Response;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "descriptors",
            /* .docs          = */ "",
            /* .type          = */ {Type::U16, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint16_t, &type::descriptors>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ {0, microstrain::Index(1) /* descriptors_count */},
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "descriptors_count",
            /* .docs          = */ "",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::descriptors_count>,
            /* .attributes    = */ {false, false, false, false, false, /*echo*/false, /*virtual*/true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "commands_base::GetExtendedDescriptors::Response",
        /* .title       = */ "response",
        /* .docs        = */ "",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<commands_base::GetExtendedDescriptors>
{
    using type = commands_base::GetExtendedDescriptors;

    static constexpr inline FieldInfo value = {
        /* .name        = */ "commands_base::GetExtendedDescriptors",
        /* .title       = */ "Get device descriptors (extended)",
        /* .docs        = */ "Get the command and data descriptors supported by the device.\n\nReply has two fields: 'ACK/NACK' and 'Descriptors'. The 'Descriptors' field is an array of 16 bit values.\nThe MSB specifies the descriptor set and the LSB specifies the descriptor.",
        /* .parameters  = */ {},
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ &MetadataFor<type::Response>::value,
    };
};

template<>
struct MetadataFor<commands_base::ContinuousBit::Response>
{
    using type = commands_base::ContinuousBit::Response;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "result",
            /* .docs          = */ "Device-specific bitfield (128 bits). See device user manual.\nBits are least-significant-byte first. For example, bit 0 is\nlocated at bit 0 of result[0], bit 1 is located at bit 1 of result[0],\nbit 8 is located at bit 0 of result[1], and bit 127 is located at bit\n7 of result[15].",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::result>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 16,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "commands_base::ContinuousBit::Response",
        /* .title       = */ "response",
        /* .docs        = */ "",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<commands_base::ContinuousBit>
{
    using type = commands_base::ContinuousBit;

    static constexpr inline FieldInfo value = {
        /* .name        = */ "commands_base::ContinuousBit",
        /* .title       = */ "Continuous built-in test",
        /* .docs        = */ "Report result of continuous built-in test.\n\nThis test is non-disruptive but is not as thorough as the commanded BIT.",
        /* .parameters  = */ {},
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ &MetadataFor<type::Response>::value,
    };
};

template<>
struct MetadataFor<commands_base::CommSpeed::Response>
{
    using type = commands_base::CommSpeed::Response;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "port",
            /* .docs          = */ "Port ID number, starting with 1. When function is SAVE, LOAD, or DEFAULT, this can be 0 to apply to all ports. See the device user manual for details.",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::port>,
            /* .attributes    = */ {true, true, true, true, true, /*echo*/true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "baud",
            /* .docs          = */ "Port baud rate. Must be a supported rate.",
            /* .type          = */ {Type::U32, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint32_t, &type::baud>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "commands_base::CommSpeed::Response",
        /* .title       = */ "response",
        /* .docs        = */ "",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<commands_base::CommSpeed>
{
    using type = commands_base::CommSpeed;

    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name          = */ "port",
            /* .docs          = */ "Port ID number, starting with 1. When function is SAVE, LOAD, or DEFAULT, this can be 0 to apply to all ports. See the device user manual for details.",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::port>,
            /* .attributes    = */ {true, true, true, true, true, /*echo*/true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "baud",
            /* .docs          = */ "Port baud rate. Must be a supported rate.",
            /* .type          = */ {Type::U32, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint32_t, &type::baud>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "commands_base::CommSpeed",
        /* .title       = */ "Comm Port Speed",
        /* .docs        = */ "Controls the baud rate of a specific port on the device.\n\nPlease see the device user manual for supported baud rates on each port.\n\nThe device will wait until all incoming and outgoing data has been sent, up\nto a maximum of 250 ms, before applying any change.\n\nNo guarantee is provided as to what happens to commands issued during this\ndelay period; They may or may not be processed and any responses aren't\nguaranteed to be at one rate or the other. The same applies to data packets.\n\nIt is highly recommended that the device be idle before issuing this command\nand that it be issued in its own packet. Users should wait 250 ms after\nsending this command before further interaction.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ {true, true, true, true, true},
        /* .response    = */ &MetadataFor<type::Response>::value,
    };
};

template<>
struct MetadataFor<commands_base::GpsTimeUpdate::FieldId>
{
    using type = commands_base::GpsTimeUpdate::FieldId;

    static constexpr inline EnumInfo::Entry entries[] = {
        { uint32_t(1), "WEEK_NUMBER", "Week number." },
        { uint32_t(2), "TIME_OF_WEEK", "Time of week in seconds." },
    };

    static constexpr inline EnumInfo value = {
        /* .name    = */ "field_id",
        /* .docs    = */ "",
        /* .type    = */ Type::U8,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<commands_base::GpsTimeUpdate>
{
    using type = commands_base::GpsTimeUpdate;

    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name          = */ "field_id",
            /* .docs          = */ "Determines how to interpret value.",
            /* .type          = */ {Type::ENUM, &MetadataFor<commands_base::GpsTimeUpdate::FieldId>::value},
            /* .accessor      = */ nullptr, //utils::access<type, commands_base::GpsTimeUpdate::FieldId, &type::field_id>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "value",
            /* .docs          = */ "Week number or time of week, depending on the field_id.",
            /* .type          = */ {Type::U32, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint32_t, &type::value>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "commands_base::GpsTimeUpdate",
        /* .title       = */ "GPS Time Update Command",
        /* .docs        = */ "Set device internal GPS time\nWhen combined with a PPS input signal applied to the I/O connector, this command enables complete synchronization of data outputs\nwith an external time base, such as GPS system time. Since the hardware PPS synchronization can only detect the fractional number of seconds when pulses arrive,\ncomplete synchronization requires that the user provide the whole number of seconds via this command. After achieving PPS synchronization, this command should be sent twice: once to set the time-of-week and once to set the week number. PPS synchronization can be verified by monitoring the time sync status message (0xA0, 0x02) or the valid flags of any shared external timestamp (0x--, D7) data field.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ {true, false, false, false, false},
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<commands_base::SoftReset>
{
    using type = commands_base::SoftReset;

    static constexpr inline FieldInfo value = {
        /* .name        = */ "commands_base::SoftReset",
        /* .title       = */ "Reset device",
        /* .docs        = */ "Resets the device.\n\nDevice responds with ACK and immediately resets.",
        /* .parameters  = */ {},
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};


static constexpr inline const FieldInfo* COMMANDS_BASE_FIELDS[] = {
    &MetadataFor<commands_base::Ping>::value,
    &MetadataFor<commands_base::SetIdle>::value,
    &MetadataFor<commands_base::GetDeviceInfo>::value,
    &MetadataFor<commands_base::GetDeviceDescriptors>::value,
    &MetadataFor<commands_base::BuiltInTest>::value,
    &MetadataFor<commands_base::Resume>::value,
    &MetadataFor<commands_base::GetExtendedDescriptors>::value,
    &MetadataFor<commands_base::ContinuousBit>::value,
    &MetadataFor<commands_base::CommSpeed>::value,
    &MetadataFor<commands_base::GpsTimeUpdate>::value,
    &MetadataFor<commands_base::SoftReset>::value,
    &MetadataFor<commands_base::GetDeviceInfo::Response>::value,
    &MetadataFor<commands_base::GetDeviceDescriptors::Response>::value,
    &MetadataFor<commands_base::BuiltInTest::Response>::value,
    &MetadataFor<commands_base::GetExtendedDescriptors::Response>::value,
    &MetadataFor<commands_base::ContinuousBit::Response>::value,
    &MetadataFor<commands_base::CommSpeed::Response>::value,
};

static constexpr DescriptorSetInfo COMMANDS_BASE = {
    /*.descriptor =*/ mip::commands_base::DESCRIPTOR_SET,
    /*.name       =*/ "Base Commands",
    /*.fields     =*/ COMMANDS_BASE_FIELDS,
};

} // namespace mip::metadata

