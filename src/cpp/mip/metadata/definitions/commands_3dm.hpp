

#include "common.hpp"

#include <mip/definitions/commands_3dm.hpp>

#include <mip/metadata/mip_metadata.hpp>

namespace mip::metadata
{


template<>
struct MetadataFor<commands_3dm::PollImuMessage>
{
    using type = commands_3dm::PollImuMessage;

    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name        = */ "suppress_ack",
            /* .docs        = */ "Suppress the usual ACK/NACK reply.",
            /* .type        = */ {Type::BOOL, nullptr},
            /* .accessor    = */ utils::access<type, bool, &type::suppress_ack>,
            /* .functions   = */ {true, false, false, false, false},
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
        {
            /* .name        = */ "num_descriptors",
            /* .docs        = */ "Number of descriptors in the descriptor list.",
            /* .type        = */ {Type::U8, nullptr},
            /* .accessor    = */ utils::access<type, uint8_t, &type::num_descriptors>,
            /* .functions   = */ {true, false, false, false, false},
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
        {
            /* .name        = */ "descriptors",
            /* .docs        = */ "Descriptor list.",
            /* .type        = */ {Type::STRUCT, &MetadataFor<DescriptorRate>::value},
            /* .accessor    = */ utils::access<type, DescriptorRate, &type::descriptors>,
            /* .functions   = */ {true, false, false, false, false},
            /* .count       = */ 83,
            /* .counter_idx = */ microstrain::Index(1) /* num_descriptors */,
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
    };

    static constexpr inline StructInfo value = {
        /* .name        = */ "PollImuMessage",
        /* .title       = */ "None",
        /* .docs        = */ "Poll the device for an IMU message with the specified format\n\nThis function polls for an IMU message using the provided format. The resulting message\nwill maintain the order of descriptors sent in the command and any unrecognized\ndescriptors are ignored. If the format is not provided, the device will attempt to use the\nstored format (set with the Set IMU Message Format command.) If no format is provided\nand there is no stored format, the device will respond with a NACK. The reply packet contains\nan ACK/NACK field. The polled data packet is sent separately as an IMU Data packet.",
        /* .parameters  = */ parameters,
    };
};

template<>
struct MetadataFor<commands_3dm::PollGnssMessage>
{
    using type = commands_3dm::PollGnssMessage;

    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name        = */ "suppress_ack",
            /* .docs        = */ "Suppress the usual ACK/NACK reply.",
            /* .type        = */ {Type::BOOL, nullptr},
            /* .accessor    = */ utils::access<type, bool, &type::suppress_ack>,
            /* .functions   = */ {true, false, false, false, false},
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
        {
            /* .name        = */ "num_descriptors",
            /* .docs        = */ "Number of descriptors in the descriptor list.",
            /* .type        = */ {Type::U8, nullptr},
            /* .accessor    = */ utils::access<type, uint8_t, &type::num_descriptors>,
            /* .functions   = */ {true, false, false, false, false},
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
        {
            /* .name        = */ "descriptors",
            /* .docs        = */ "Descriptor list.",
            /* .type        = */ {Type::STRUCT, &MetadataFor<DescriptorRate>::value},
            /* .accessor    = */ utils::access<type, DescriptorRate, &type::descriptors>,
            /* .functions   = */ {true, false, false, false, false},
            /* .count       = */ 83,
            /* .counter_idx = */ microstrain::Index(1) /* num_descriptors */,
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
    };

    static constexpr inline StructInfo value = {
        /* .name        = */ "PollGnssMessage",
        /* .title       = */ "None",
        /* .docs        = */ "Poll the device for an GNSS message with the specified format\n\nThis function polls for a GNSS message using the provided format. The resulting message\nwill maintain the order of descriptors sent in the command and any unrecognized\ndescriptors are ignored. If the format is not provided, the device will attempt to use the\nstored format (set with the Set GNSS Message Format command.) If no format is provided\nand there is no stored format, the device will respond with a NACK. The reply packet contains\nan ACK/NACK field. The polled data packet is sent separately as a GNSS Data packet.",
        /* .parameters  = */ parameters,
    };
};

template<>
struct MetadataFor<commands_3dm::PollFilterMessage>
{
    using type = commands_3dm::PollFilterMessage;

    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name        = */ "suppress_ack",
            /* .docs        = */ "Suppress the usual ACK/NACK reply.",
            /* .type        = */ {Type::BOOL, nullptr},
            /* .accessor    = */ utils::access<type, bool, &type::suppress_ack>,
            /* .functions   = */ {true, false, false, false, false},
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
        {
            /* .name        = */ "num_descriptors",
            /* .docs        = */ "Number of descriptors in the format list.",
            /* .type        = */ {Type::U8, nullptr},
            /* .accessor    = */ utils::access<type, uint8_t, &type::num_descriptors>,
            /* .functions   = */ {true, false, false, false, false},
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
        {
            /* .name        = */ "descriptors",
            /* .docs        = */ "Descriptor format list.",
            /* .type        = */ {Type::STRUCT, &MetadataFor<DescriptorRate>::value},
            /* .accessor    = */ utils::access<type, DescriptorRate, &type::descriptors>,
            /* .functions   = */ {true, false, false, false, false},
            /* .count       = */ 83,
            /* .counter_idx = */ microstrain::Index(1) /* num_descriptors */,
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
    };

    static constexpr inline StructInfo value = {
        /* .name        = */ "PollFilterMessage",
        /* .title       = */ "None",
        /* .docs        = */ "Poll the device for an Estimation Filter message with the specified format\n\nThis function polls for an Estimation Filter message using the provided format. The resulting message\nwill maintain the order of descriptors sent in the command and any unrecognized\ndescriptors are ignored. If the format is not provided, the device will attempt to use the\nstored format (set with the Set Estimation Filter Message Format command.) If no format is provided\nand there is no stored format, the device will respond with a NACK. The reply packet contains\nan ACK/NACK field. The polled data packet is sent separately as an Estimation Filter Data packet.",
        /* .parameters  = */ parameters,
    };
};

template<>
struct MetadataFor<commands_3dm::ImuMessageFormat>
{
    using type = commands_3dm::ImuMessageFormat;

    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name        = */ "num_descriptors",
            /* .docs        = */ "Number of descriptors",
            /* .type        = */ {Type::U8, nullptr},
            /* .accessor    = */ utils::access<type, uint8_t, &type::num_descriptors>,
            /* .functions   = */ {true, false, false, false, false},
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
        {
            /* .name        = */ "descriptors",
            /* .docs        = */ "Descriptor format list.",
            /* .type        = */ {Type::STRUCT, &MetadataFor<DescriptorRate>::value},
            /* .accessor    = */ utils::access<type, DescriptorRate, &type::descriptors>,
            /* .functions   = */ {true, false, false, false, false},
            /* .count       = */ 82,
            /* .counter_idx = */ microstrain::Index(0) /* num_descriptors */,
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
    };

    static constexpr inline StructInfo value = {
        /* .name        = */ "ImuMessageFormat",
        /* .title       = */ "None",
        /* .docs        = */ "Set, read, or save the format of the IMU data packet.\n\nThe resulting data messages will maintain the order of descriptors sent in the command.",
        /* .parameters  = */ parameters,
    };
};

template<>
struct MetadataFor<commands_3dm::GpsMessageFormat>
{
    using type = commands_3dm::GpsMessageFormat;

    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name        = */ "num_descriptors",
            /* .docs        = */ "Number of descriptors",
            /* .type        = */ {Type::U8, nullptr},
            /* .accessor    = */ utils::access<type, uint8_t, &type::num_descriptors>,
            /* .functions   = */ {true, false, false, false, false},
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
        {
            /* .name        = */ "descriptors",
            /* .docs        = */ "Descriptor format list.",
            /* .type        = */ {Type::STRUCT, &MetadataFor<DescriptorRate>::value},
            /* .accessor    = */ utils::access<type, DescriptorRate, &type::descriptors>,
            /* .functions   = */ {true, false, false, false, false},
            /* .count       = */ 82,
            /* .counter_idx = */ microstrain::Index(0) /* num_descriptors */,
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
    };

    static constexpr inline StructInfo value = {
        /* .name        = */ "GpsMessageFormat",
        /* .title       = */ "None",
        /* .docs        = */ "Set, read, or save the format of the GNSS data packet.\n\nThe resulting data messages will maintain the order of descriptors sent in the command.",
        /* .parameters  = */ parameters,
    };
};

template<>
struct MetadataFor<commands_3dm::FilterMessageFormat>
{
    using type = commands_3dm::FilterMessageFormat;

    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name        = */ "num_descriptors",
            /* .docs        = */ "Number of descriptors (limited by payload size)",
            /* .type        = */ {Type::U8, nullptr},
            /* .accessor    = */ utils::access<type, uint8_t, &type::num_descriptors>,
            /* .functions   = */ {true, false, false, false, false},
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
        {
            /* .name        = */ "descriptors",
            /* .docs        = */ "",
            /* .type        = */ {Type::STRUCT, &MetadataFor<DescriptorRate>::value},
            /* .accessor    = */ utils::access<type, DescriptorRate, &type::descriptors>,
            /* .functions   = */ {true, false, false, false, false},
            /* .count       = */ 82,
            /* .counter_idx = */ microstrain::Index(0) /* num_descriptors */,
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
    };

    static constexpr inline StructInfo value = {
        /* .name        = */ "FilterMessageFormat",
        /* .title       = */ "None",
        /* .docs        = */ "Set, read, or save the format of the Estimation Filter data packet.\n\nThe resulting data messages will maintain the order of descriptors sent in the command.",
        /* .parameters  = */ parameters,
    };
};

template<>
struct MetadataFor<commands_3dm::ImuGetBaseRate>
{
    using type = commands_3dm::ImuGetBaseRate;

    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,

    };

    static constexpr inline StructInfo value = {
        /* .name        = */ "ImuGetBaseRate",
        /* .title       = */ "Get IMU Data Base Rate",
        /* .docs        = */ "Get the base rate for the IMU data in Hz\n\nThis is the fastest rate for this type of data available on the device.\nThis is used in conjunction with the IMU Message Format Command to set streaming data at a specified rate.",
        /* .parameters  = */ parameters,
    };
};

template<>
struct MetadataFor<commands_3dm::GpsGetBaseRate>
{
    using type = commands_3dm::GpsGetBaseRate;

    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,

    };

    static constexpr inline StructInfo value = {
        /* .name        = */ "GpsGetBaseRate",
        /* .title       = */ "Get GNSS Data Base Rate",
        /* .docs        = */ "Get the base rate for the GNSS data in Hz\n\nThis is the fastest rate for this type of data available on the device.\nThis is used in conjunction with the GNSS Message Format Command to set streaming data at a specified rate.",
        /* .parameters  = */ parameters,
    };
};

template<>
struct MetadataFor<commands_3dm::FilterGetBaseRate>
{
    using type = commands_3dm::FilterGetBaseRate;

    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,

    };

    static constexpr inline StructInfo value = {
        /* .name        = */ "FilterGetBaseRate",
        /* .title       = */ "Get Estimation Filter Data Base Rate",
        /* .docs        = */ "Get the base rate for the Estimation Filter data in Hz\n\nThis is the fastest rate for this type of data available on the device.\nThis is used in conjunction with the Estimation Filter Message Format Command to set streaming data at a specified rate.",
        /* .parameters  = */ parameters,
    };
};

template<>
struct MetadataFor<commands_3dm::PollData>
{
    using type = commands_3dm::PollData;

    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name        = */ "desc_set",
            /* .docs        = */ "Data descriptor set. Must be supported.",
            /* .type        = */ {Type::U8, nullptr},
            /* .accessor    = */ utils::access<type, uint8_t, &type::desc_set>,
            /* .functions   = */ {true, false, false, false, false},
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
        {
            /* .name        = */ "suppress_ack",
            /* .docs        = */ "Suppress the usual ACK/NACK reply.",
            /* .type        = */ {Type::BOOL, nullptr},
            /* .accessor    = */ utils::access<type, bool, &type::suppress_ack>,
            /* .functions   = */ {true, false, false, false, false},
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
        {
            /* .name        = */ "num_descriptors",
            /* .docs        = */ "Number of descriptors in the format list.",
            /* .type        = */ {Type::U8, nullptr},
            /* .accessor    = */ utils::access<type, uint8_t, &type::num_descriptors>,
            /* .functions   = */ {true, false, false, false, false},
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
        {
            /* .name        = */ "descriptors",
            /* .docs        = */ "Descriptor format list.",
            /* .type        = */ {Type::U8, nullptr},
            /* .accessor    = */ utils::access<type, uint8_t, &type::descriptors>,
            /* .functions   = */ {true, false, false, false, false},
            /* .count       = */ 82,
            /* .counter_idx = */ microstrain::Index(2) /* num_descriptors */,
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
    };

    static constexpr inline StructInfo value = {
        /* .name        = */ "PollData",
        /* .title       = */ "None",
        /* .docs        = */ "Poll the device for a message with the specified descriptor set and format.\n\nThis function polls for a message using the provided format. The resulting message\nwill maintain the order of descriptors sent in the command and any unrecognized\ndescriptors are ignored. If the format is not provided, the device will attempt to use the\nstored format (set with the Set Message Format command.) If no format is provided\nand there is no stored format, the device will respond with a NACK. The reply packet contains\nan ACK/NACK field. The polled data packet is sent separately as a normal Data packet.",
        /* .parameters  = */ parameters,
    };
};

template<>
struct MetadataFor<commands_3dm::GetBaseRate>
{
    using type = commands_3dm::GetBaseRate;

    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name        = */ "desc_set",
            /* .docs        = */ "This is the data descriptor set. It must be a supported descriptor.",
            /* .type        = */ {Type::U8, nullptr},
            /* .accessor    = */ utils::access<type, uint8_t, &type::desc_set>,
            /* .functions   = */ {true, false, false, false, false,  true},
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
    };

    static constexpr inline StructInfo value = {
        /* .name        = */ "GetBaseRate",
        /* .title       = */ "Get Data Base Rate",
        /* .docs        = */ "Get the base rate for the specified descriptor set in Hz.",
        /* .parameters  = */ parameters,
    };
};

template<>
struct MetadataFor<commands_3dm::MessageFormat>
{
    using type = commands_3dm::MessageFormat;

    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name        = */ "desc_set",
            /* .docs        = */ "Data descriptor set. Must be supported. When function is SAVE, LOAD, or DEFAULT, can be 0 to apply to all descriptor sets.",
            /* .type        = */ {Type::U8, nullptr},
            /* .accessor    = */ utils::access<type, uint8_t, &type::desc_set>,
            /* .functions   = */ {true, true, true, true, true,  true},
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
        {
            /* .name        = */ "num_descriptors",
            /* .docs        = */ "Number of descriptors (limited by payload size)",
            /* .type        = */ {Type::U8, nullptr},
            /* .accessor    = */ utils::access<type, uint8_t, &type::num_descriptors>,
            /* .functions   = */ {true, false, false, false, false},
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
        {
            /* .name        = */ "descriptors",
            /* .docs        = */ "List of descriptors and decimations.",
            /* .type        = */ {Type::STRUCT, &MetadataFor<DescriptorRate>::value},
            /* .accessor    = */ utils::access<type, DescriptorRate, &type::descriptors>,
            /* .functions   = */ {true, false, false, false, false},
            /* .count       = */ 82,
            /* .counter_idx = */ microstrain::Index(1) /* num_descriptors */,
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
    };

    static constexpr inline StructInfo value = {
        /* .name        = */ "MessageFormat",
        /* .title       = */ "None",
        /* .docs        = */ "Set, read, or save the format for a given data packet.\n\nThe resulting data messages will maintain the order of descriptors sent in the command.",
        /* .parameters  = */ parameters,
    };
};

template<>
struct MetadataFor<commands_3dm::NmeaMessage::MessageID>
{
    using type = commands_3dm::NmeaMessage::MessageID;

    static constexpr inline EnumInfo::Entry entries[] = {
        { 1, "GGA", "GPS System Fix Data. Source can be the Filter or GNSS1/2 datasets." },
        { 2, "GLL", "Geographic Position Lat/Lon. Source can be the Filter or GNSS1/2 datasets." },
        { 3, "GSV", "GNSS Satellites in View. Source must be either GNSS1 or GNSS2 datasets. The talker ID must be set to IGNORED." },
        { 4, "RMC", "Recommended Minimum Specific GNSS Data. Source can be the Filter or GNSS1/2 datasets." },
        { 5, "VTG", "Course over Ground. Source can be the Filter or GNSS1/2 datasets." },
        { 6, "HDT", "Heading, True. Source can be the Filter or GNSS1/2 datasets." },
        { 7, "ZDA", "Time & Date. Source must be the GNSS1 or GNSS2 datasets." },
        { 129, "PKRA", "Parker proprietary Euler angles. Source must be the Filter dataset. The talker ID must be set to IGNORED." },
        { 130, "PKRR", "Parker proprietary Angular Rate/Acceleration. Source must be the Sensor dataset. The talker ID must be set to IGNORED." },
    };

    static constexpr inline EnumInfo value = {
        /* .name    = */ "MessageID",
        /* .docs    = */ "",
        /* .type    = */ Type::U8,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<commands_3dm::NmeaMessage::TalkerID>
{
    using type = commands_3dm::NmeaMessage::TalkerID;

    static constexpr inline EnumInfo::Entry entries[] = {
        { 0, "IGNORED", "Talker ID cannot be changed." },
        { 1, "GNSS", "NMEA message will be produced with talker id 'GN'." },
        { 2, "GPS", "NMEA message will be produced with talker id 'GP'." },
        { 3, "GALILEO", "NMEA message will be produced with talker id 'GA'." },
        { 4, "GLONASS", "NMEA message will be produced with talker id 'GL'." },
    };

    static constexpr inline EnumInfo value = {
        /* .name    = */ "TalkerID",
        /* .docs    = */ "",
        /* .type    = */ Type::U8,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<commands_3dm::NmeaMessage>
{
    using type = commands_3dm::NmeaMessage;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name        = */ "message_id",
            /* .docs        = */ "NMEA sentence type.",
            /* .type        = */ {Type::ENUM, &MetadataFor<commands_3dm::NmeaMessage::MessageID>::value},
            /* .accessor    = */ utils::access<type, commands_3dm::NmeaMessage::MessageID, &type::message_id>,
            /* .functions   = */ NO_FUNCTIONS,
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
        {
            /* .name        = */ "talker_id",
            /* .docs        = */ "NMEA talker ID. Ignored for proprietary sentences.",
            /* .type        = */ {Type::ENUM, &MetadataFor<commands_3dm::NmeaMessage::TalkerID>::value},
            /* .accessor    = */ utils::access<type, commands_3dm::NmeaMessage::TalkerID, &type::talker_id>,
            /* .functions   = */ NO_FUNCTIONS,
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
        {
            /* .name        = */ "source_desc_set",
            /* .docs        = */ "Data descriptor set where the data will be sourced. Available options depend on the sentence.",
            /* .type        = */ {Type::U8, nullptr},
            /* .accessor    = */ utils::access<type, uint8_t, &type::source_desc_set>,
            /* .functions   = */ NO_FUNCTIONS,
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
        {
            /* .name        = */ "decimation",
            /* .docs        = */ "Decimation from the base rate for source_desc_set. Frequency is limited to 10 Hz or the base rate, whichever is lower. Must be 0 when polling.",
            /* .type        = */ {Type::U16, nullptr},
            /* .accessor    = */ utils::access<type, uint16_t, &type::decimation>,
            /* .functions   = */ NO_FUNCTIONS,
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
    };

    static constexpr inline StructInfo value = {
        /* .name        = */ "NmeaMessage",
        /* .title       = */ "Nmeamessage",
        /* .docs        = */ "",
        /* .parameters  = */ parameters,
    };
};

template<>
struct MetadataFor<commands_3dm::NmeaPollData>
{
    using type = commands_3dm::NmeaPollData;

    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name        = */ "suppress_ack",
            /* .docs        = */ "Suppress the usual ACK/NACK reply.",
            /* .type        = */ {Type::BOOL, nullptr},
            /* .accessor    = */ utils::access<type, bool, &type::suppress_ack>,
            /* .functions   = */ {true, false, false, false, false},
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
        {
            /* .name        = */ "count",
            /* .docs        = */ "Number of format entries (limited by payload size)",
            /* .type        = */ {Type::U8, nullptr},
            /* .accessor    = */ utils::access<type, uint8_t, &type::count>,
            /* .functions   = */ {true, false, false, false, false},
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
        {
            /* .name        = */ "format_entries",
            /* .docs        = */ "List of format entries.",
            /* .type        = */ {Type::STRUCT, &MetadataFor<commands_3dm::NmeaMessage>::value},
            /* .accessor    = */ utils::access<type, commands_3dm::NmeaMessage, &type::format_entries>,
            /* .functions   = */ {true, false, false, false, false},
            /* .count       = */ 40,
            /* .counter_idx = */ microstrain::Index(1) /* count */,
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
    };

    static constexpr inline StructInfo value = {
        /* .name        = */ "NmeaPollData",
        /* .title       = */ "None",
        /* .docs        = */ "Poll the device for a NMEA message with the specified format.\n\nThis function polls for a NMEA message using the provided format.\nIf the format is not provided, the device will attempt to use the\nstored format (set with the Set NMEA Message Format command.) If no format is provided\nand there is no stored format, the device will respond with a NACK. The reply packet contains\nan ACK/NACK field. The polled data packet is sent separately as normal NMEA messages.",
        /* .parameters  = */ parameters,
    };
};

template<>
struct MetadataFor<commands_3dm::NmeaMessageFormat>
{
    using type = commands_3dm::NmeaMessageFormat;

    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name        = */ "count",
            /* .docs        = */ "Number of format entries (limited by payload size)",
            /* .type        = */ {Type::U8, nullptr},
            /* .accessor    = */ utils::access<type, uint8_t, &type::count>,
            /* .functions   = */ {true, false, false, false, false},
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
        {
            /* .name        = */ "format_entries",
            /* .docs        = */ "List of format entries.",
            /* .type        = */ {Type::STRUCT, &MetadataFor<commands_3dm::NmeaMessage>::value},
            /* .accessor    = */ utils::access<type, commands_3dm::NmeaMessage, &type::format_entries>,
            /* .functions   = */ {true, false, false, false, false},
            /* .count       = */ 40,
            /* .counter_idx = */ microstrain::Index(0) /* count */,
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
    };

    static constexpr inline StructInfo value = {
        /* .name        = */ "NmeaMessageFormat",
        /* .title       = */ "None",
        /* .docs        = */ "Set, read, or save the NMEA message format.",
        /* .parameters  = */ parameters,
    };
};

template<>
struct MetadataFor<commands_3dm::DeviceSettings>
{
    using type = commands_3dm::DeviceSettings;

    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,

    };

    static constexpr inline StructInfo value = {
        /* .name        = */ "DeviceSettings",
        /* .title       = */ "None",
        /* .docs        = */ "Save, Load, or Reset to Default the values for all device settings.\n\nWhen a save current settings command is issued, a brief data disturbance may occur while all settings are written to non-volatile memory.\n\nThis command should have a long timeout as it may take up to 1 second to complete.",
        /* .parameters  = */ parameters,
    };
};

template<>
struct MetadataFor<commands_3dm::UartBaudrate>
{
    using type = commands_3dm::UartBaudrate;

    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name        = */ "baud",
            /* .docs        = */ "",
            /* .type        = */ {Type::U32, nullptr},
            /* .accessor    = */ utils::access<type, uint32_t, &type::baud>,
            /* .functions   = */ {true, false, false, false, false},
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
    };

    static constexpr inline StructInfo value = {
        /* .name        = */ "UartBaudrate",
        /* .title       = */ "None",
        /* .docs        = */ "Read, Save, Load, or Reset to Default the baud rate of the main communication channel.\n\nFor all functions except 0x01 (use new settings), the new baud rate value is ignored.\nPlease see the device user manual for supported baud rates.\n\nThe device will wait until all incoming and outgoing data has been sent, up\nto a maximum of 250 ms, before applying any change.\n\nNo guarantee is provided as to what happens to commands issued during this\ndelay period; They may or may not be processed and any responses aren't\nguaranteed to be at one rate or the other. The same applies to data packets.\n\nIt is highly recommended that the device be idle before issuing this command\nand that it be issued in its own packet. Users should wait 250 ms after\nsending this command before further interaction.",
        /* .parameters  = */ parameters,
    };
};

template<>
struct MetadataFor<commands_3dm::FactoryStreaming::Action>
{
    using type = commands_3dm::FactoryStreaming::Action;

    static constexpr inline EnumInfo::Entry entries[] = {
        { 0, "OVERWRITE", "Replaces the message format(s), removing any existing descriptors." },
        { 1, "MERGE", "Merges support descriptors into existing format(s). May reorder descriptors." },
        { 2, "ADD", "Adds descriptors to the current message format(s) without changing existing descriptors. May result in duplicates." },
    };

    static constexpr inline EnumInfo value = {
        /* .name    = */ "Action",
        /* .docs    = */ "",
        /* .type    = */ Type::U8,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<commands_3dm::FactoryStreaming>
{
    using type = commands_3dm::FactoryStreaming;

    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name        = */ "action",
            /* .docs        = */ "",
            /* .type        = */ {Type::ENUM, &MetadataFor<commands_3dm::FactoryStreaming::Action>::value},
            /* .accessor    = */ utils::access<type, commands_3dm::FactoryStreaming::Action, &type::action>,
            /* .functions   = */ {true, false, false, false, false},
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
        {
            /* .name        = */ "reserved",
            /* .docs        = */ "Reserved. Set to 0x00.",
            /* .type        = */ {Type::U8, nullptr},
            /* .accessor    = */ utils::access<type, uint8_t, &type::reserved>,
            /* .functions   = */ {true, false, false, false, false},
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
    };

    static constexpr inline StructInfo value = {
        /* .name        = */ "FactoryStreaming",
        /* .title       = */ "None",
        /* .docs        = */ "Configures the device for recording data for technical support.\n\nThis command will configure all available data streams to predefined\nformats designed to be used with technical support.",
        /* .parameters  = */ parameters,
    };
};

template<>
struct MetadataFor<commands_3dm::DatastreamControl>
{
    using type = commands_3dm::DatastreamControl;

    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name        = */ "desc_set",
            /* .docs        = */ "The descriptor set of the stream to control. When function is SAVE, LOAD, or DEFAULT, can be ALL_STREAMS(0) to apply to all descriptor sets.\nOn Generation 5 products, this must be one of the above legacy constants.",
            /* .type        = */ {Type::U8, nullptr},
            /* .accessor    = */ utils::access<type, uint8_t, &type::desc_set>,
            /* .functions   = */ {true, true, true, true, true,  true},
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
        {
            /* .name        = */ "enable",
            /* .docs        = */ "True or false to enable or disable the stream.",
            /* .type        = */ {Type::BOOL, nullptr},
            /* .accessor    = */ utils::access<type, bool, &type::enable>,
            /* .functions   = */ {true, false, false, false, false},
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
    };

    static constexpr inline StructInfo value = {
        /* .name        = */ "DatastreamControl",
        /* .title       = */ "None",
        /* .docs        = */ "Enable/disable the selected data stream.\n\nEach data stream (descriptor set) can be enabled or disabled.\nThe default for the device is all streams enabled.\nFor all functions except 0x01 (use new setting),\nthe new enable flag value is ignored and can be omitted.",
        /* .parameters  = */ parameters,
    };
};

template<>
struct MetadataFor<commands_3dm::ConstellationSettings::ConstellationId>
{
    using type = commands_3dm::ConstellationSettings::ConstellationId;

    static constexpr inline EnumInfo::Entry entries[] = {
        { 0, "GPS", "GPS (G1-G32)" },
        { 1, "SBAS", "SBAS (S120-S158)" },
        { 2, "GALILEO", "GALILEO (E1-E36)" },
        { 3, "BeiDou", "BeiDou (B1-B37)" },
        { 5, "QZSS", "QZSS (Q1-Q5)" },
        { 6, "GLONASS", "GLONASS (R1-R32)" },
    };

    static constexpr inline EnumInfo value = {
        /* .name    = */ "ConstellationId",
        /* .docs    = */ "",
        /* .type    = */ Type::U8,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<commands_3dm::ConstellationSettings::OptionFlags>
{
    using type = commands_3dm::ConstellationSettings::OptionFlags;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { 1, "L1SAIF", "Available only for QZSS" },
    };

    static constexpr inline BitfieldInfo value = {
        /* .name    = */ "OptionFlags",
        /* .docs    = */ "",
        /* .type    = */ Type::U16,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<commands_3dm::ConstellationSettings::Settings>
{
    using type = commands_3dm::ConstellationSettings::Settings;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name        = */ "constellation_id",
            /* .docs        = */ "Constellation ID",
            /* .type        = */ {Type::ENUM, &MetadataFor<commands_3dm::ConstellationSettings::ConstellationId>::value},
            /* .accessor    = */ utils::access<type, commands_3dm::ConstellationSettings::ConstellationId, &type::constellation_id>,
            /* .functions   = */ NO_FUNCTIONS,
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
        {
            /* .name        = */ "enable",
            /* .docs        = */ "Enable/Disable constellation",
            /* .type        = */ {Type::U8, nullptr},
            /* .accessor    = */ utils::access<type, uint8_t, &type::enable>,
            /* .functions   = */ NO_FUNCTIONS,
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
        {
            /* .name        = */ "reserved_channels",
            /* .docs        = */ "Minimum number of channels reserved for this constellation",
            /* .type        = */ {Type::U8, nullptr},
            /* .accessor    = */ utils::access<type, uint8_t, &type::reserved_channels>,
            /* .functions   = */ NO_FUNCTIONS,
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
        {
            /* .name        = */ "max_channels",
            /* .docs        = */ "Maximum number of channels to use for this constallation",
            /* .type        = */ {Type::U8, nullptr},
            /* .accessor    = */ utils::access<type, uint8_t, &type::max_channels>,
            /* .functions   = */ NO_FUNCTIONS,
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
        {
            /* .name        = */ "option_flags",
            /* .docs        = */ "Constellation option Flags",
            /* .type        = */ {Type::BITFIELD, &MetadataFor<commands_3dm::ConstellationSettings::OptionFlags>::value},
            /* .accessor    = */ utils::access<type, commands_3dm::ConstellationSettings::OptionFlags, &type::option_flags>,
            /* .functions   = */ NO_FUNCTIONS,
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
    };

    static constexpr inline StructInfo value = {
        /* .name        = */ "Settings",
        /* .title       = */ "Settings",
        /* .docs        = */ "",
        /* .parameters  = */ parameters,
    };
};

template<>
struct MetadataFor<commands_3dm::ConstellationSettings>
{
    using type = commands_3dm::ConstellationSettings;

    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name        = */ "max_channels",
            /* .docs        = */ "",
            /* .type        = */ {Type::U16, nullptr},
            /* .accessor    = */ utils::access<type, uint16_t, &type::max_channels>,
            /* .functions   = */ {true, false, false, false, false},
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
        {
            /* .name        = */ "config_count",
            /* .docs        = */ "",
            /* .type        = */ {Type::U8, nullptr},
            /* .accessor    = */ utils::access<type, uint8_t, &type::config_count>,
            /* .functions   = */ {true, false, false, false, false},
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
        {
            /* .name        = */ "settings",
            /* .docs        = */ "",
            /* .type        = */ {Type::STRUCT, &MetadataFor<commands_3dm::ConstellationSettings::Settings>::value},
            /* .accessor    = */ utils::access<type, commands_3dm::ConstellationSettings::Settings, &type::settings>,
            /* .functions   = */ {true, false, false, false, false},
            /* .count       = */ 0,
            /* .counter_idx = */ microstrain::Index(1) /* config_count */,
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
    };

    static constexpr inline StructInfo value = {
        /* .name        = */ "ConstellationSettings",
        /* .title       = */ "None",
        /* .docs        = */ "This command configures which satellite constellations are enabled and how many channels are dedicated to tracking each constellation.\n\nMaximum number of tracking channels to use (total for all constellations):\n0 to max_channels_available (from reply message)\n\nFor each constellation you wish to use, include a ConstellationSettings struct.  Note the following:\n\nTotal number of tracking channels (sum of 'reserved_channels' for all constellations) must be <= 32:\n0 -> 32 Number of reserved channels\n0 -> 32 Max number of channels (>= reserved channels)\n\nThe factory default setting is: GPS and GLONASS enabled.  Min/Max for GPS = 8/16, GLONASS = 8/14, SBAS = 1/3, QZSS = 0/3.\n\nWarning: SBAS functionality shall not be used in 'safety of life' applications!\nWarning: Any setting that causes the total reserved channels to exceed 32 will result in a NACK.\nWarning: You cannot enable GLONASS and BeiDou at the same time.\nNote:    Enabling SBAS and QZSS augments GPS accuracy.\nNote:    It is recommended to disable GLONASS and BeiDou if a GPS-only antenna or GPS-only SAW filter is used.",
        /* .parameters  = */ parameters,
    };
};

template<>
struct MetadataFor<commands_3dm::GnssSbasSettings::SBASOptions>
{
    using type = commands_3dm::GnssSbasSettings::SBASOptions;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { 1, "enable_ranging", "Use SBAS pseudo-ranges in position solution" },
        { 2, "enable_corrections", "Use SBAS differential corrections" },
        { 4, "apply_integrity", "Use SBAS integrity information.  If enabled, only GPS satellites for which integrity information is available will be used." },
    };

    static constexpr inline BitfieldInfo value = {
        /* .name    = */ "SBASOptions",
        /* .docs    = */ "",
        /* .type    = */ Type::U16,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<commands_3dm::GnssSbasSettings>
{
    using type = commands_3dm::GnssSbasSettings;

    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name        = */ "enable_sbas",
            /* .docs        = */ "0 - SBAS Disabled, 1 - SBAS enabled",
            /* .type        = */ {Type::U8, nullptr},
            /* .accessor    = */ utils::access<type, uint8_t, &type::enable_sbas>,
            /* .functions   = */ {true, false, false, false, false},
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
        {
            /* .name        = */ "sbas_options",
            /* .docs        = */ "SBAS options, see definition",
            /* .type        = */ {Type::BITFIELD, &MetadataFor<commands_3dm::GnssSbasSettings::SBASOptions>::value},
            /* .accessor    = */ utils::access<type, commands_3dm::GnssSbasSettings::SBASOptions, &type::sbas_options>,
            /* .functions   = */ {true, false, false, false, false},
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
        {
            /* .name        = */ "num_included_prns",
            /* .docs        = */ "Number of SBAS PRNs to include in search (0 = include all)",
            /* .type        = */ {Type::U8, nullptr},
            /* .accessor    = */ utils::access<type, uint8_t, &type::num_included_prns>,
            /* .functions   = */ {true, false, false, false, false},
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
        {
            /* .name        = */ "included_prns",
            /* .docs        = */ "List of specific SBAS PRNs to search for",
            /* .type        = */ {Type::U16, nullptr},
            /* .accessor    = */ utils::access<type, uint16_t, &type::included_prns>,
            /* .functions   = */ {true, false, false, false, false},
            /* .count       = */ 39,
            /* .counter_idx = */ microstrain::Index(2) /* num_included_prns */,
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
    };

    static constexpr inline StructInfo value = {
        /* .name        = */ "GnssSbasSettings",
        /* .title       = */ "SBAS Settings",
        /* .docs        = */ "Configure the SBAS subsystem\n\n\n",
        /* .parameters  = */ parameters,
    };
};

template<>
struct MetadataFor<commands_3dm::GnssAssistedFix::AssistedFixOption>
{
    using type = commands_3dm::GnssAssistedFix::AssistedFixOption;

    static constexpr inline EnumInfo::Entry entries[] = {
        { 0, "NONE", "No assisted fix (default)" },
        { 1, "ENABLED", "Enable assisted fix" },
    };

    static constexpr inline EnumInfo value = {
        /* .name    = */ "AssistedFixOption",
        /* .docs    = */ "",
        /* .type    = */ Type::U8,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<commands_3dm::GnssAssistedFix>
{
    using type = commands_3dm::GnssAssistedFix;

    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name        = */ "option",
            /* .docs        = */ "Assisted fix options",
            /* .type        = */ {Type::ENUM, &MetadataFor<commands_3dm::GnssAssistedFix::AssistedFixOption>::value},
            /* .accessor    = */ utils::access<type, commands_3dm::GnssAssistedFix::AssistedFixOption, &type::option>,
            /* .functions   = */ {true, false, false, false, false},
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
        {
            /* .name        = */ "flags",
            /* .docs        = */ "Assisted fix flags (set to 0xFF)",
            /* .type        = */ {Type::U8, nullptr},
            /* .accessor    = */ utils::access<type, uint8_t, &type::flags>,
            /* .functions   = */ {true, false, false, false, false},
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
    };

    static constexpr inline StructInfo value = {
        /* .name        = */ "GnssAssistedFix",
        /* .title       = */ "GNSS Assisted Fix Settings",
        /* .docs        = */ "Set the options for assisted GNSS fix.\n\nDevices that implement this command have a dedicated GNSS flash memory and a non-volatile FRAM.\nThese storage mechanisms are used to retain information about the last good GNSS fix. This can greatly reduces the TTFF (Time To First Fix) depending on the age of the stored information.\nThe TTFF can be as low as one second, or up to the equivalent of a cold start. There is a small increase in power used when enabling assisted fix.\n\nThe fastest fix will be obtained by supplying the device with a GNSS Assist Time Update message containing the current GPS time immediately after subsequent power up.\nThis allows the device to determine if the last GNSS information saved is still fresh enough to improve the TTFF.\n\nNOTE: Non-volatile GNSS memory is cleared when going from an enabled state to a disabled state.\nWARNING: The clearing operation results in an erase operation on the GNSS Flash. The flash has a limited durability of 100,000 write/erase cycles",
        /* .parameters  = */ parameters,
    };
};

template<>
struct MetadataFor<commands_3dm::GnssTimeAssistance>
{
    using type = commands_3dm::GnssTimeAssistance;

    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name        = */ "tow",
            /* .docs        = */ "GPS Time of week [seconds]",
            /* .type        = */ {Type::DOUBLE, nullptr},
            /* .accessor    = */ utils::access<type, double, &type::tow>,
            /* .functions   = */ {true, false, false, false, false},
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
        {
            /* .name        = */ "week_number",
            /* .docs        = */ "GPS Weeks since 1980 [weeks]",
            /* .type        = */ {Type::U16, nullptr},
            /* .accessor    = */ utils::access<type, uint16_t, &type::week_number>,
            /* .functions   = */ {true, false, false, false, false},
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
        {
            /* .name        = */ "accuracy",
            /* .docs        = */ "Accuracy of time information [seconds]",
            /* .type        = */ {Type::FLOAT, nullptr},
            /* .accessor    = */ utils::access<type, float, &type::accuracy>,
            /* .functions   = */ {true, false, false, false, false},
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
    };

    static constexpr inline StructInfo value = {
        /* .name        = */ "GnssTimeAssistance",
        /* .title       = */ "None",
        /* .docs        = */ "Provide the GNSS subsystem with initial time information.\n\nThis message is required immediately after power up if GNSS Assist was enabled when the device was powered off.\nThis will initialize the subsystem clock to help reduce the time to first fix (TTFF).",
        /* .parameters  = */ parameters,
    };
};

template<>
struct MetadataFor<commands_3dm::ImuLowpassFilter>
{
    using type = commands_3dm::ImuLowpassFilter;

    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name        = */ "target_descriptor",
            /* .docs        = */ "Field descriptor of filtered quantity within the Sensor data set.\nSupported values are accel (0x04), gyro (0x05), mag (0x06), and\npressure (0x17), provided the data is supported by the device.\nExcept with the READ function selector, this can be 0 to apply to all of the above quantities.",
            /* .type        = */ {Type::U8, nullptr},
            /* .accessor    = */ utils::access<type, uint8_t, &type::target_descriptor>,
            /* .functions   = */ {true, true, true, true, true,  true},
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
        {
            /* .name        = */ "enable",
            /* .docs        = */ "The target data will be filtered if this is true.",
            /* .type        = */ {Type::BOOL, nullptr},
            /* .accessor    = */ utils::access<type, bool, &type::enable>,
            /* .functions   = */ {true, false, false, false, false},
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
        {
            /* .name        = */ "manual",
            /* .docs        = */ "If false, the cutoff frequency is set to half of the\nstreaming rate as configured by the message format command.\nOtherwise, the cutoff frequency is set according to the\nfollowing 'frequency' parameter.",
            /* .type        = */ {Type::BOOL, nullptr},
            /* .accessor    = */ utils::access<type, bool, &type::manual>,
            /* .functions   = */ {true, false, false, false, false},
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
        {
            /* .name        = */ "frequency",
            /* .docs        = */ "-3dB cutoff frequency in Hz. Will not affect filtering if 'manual' is false.",
            /* .type        = */ {Type::U16, nullptr},
            /* .accessor    = */ utils::access<type, uint16_t, &type::frequency>,
            /* .functions   = */ {true, false, false, false, false},
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
        {
            /* .name        = */ "reserved",
            /* .docs        = */ "Reserved, set to 0x00.",
            /* .type        = */ {Type::U8, nullptr},
            /* .accessor    = */ utils::access<type, uint8_t, &type::reserved>,
            /* .functions   = */ {true, false, false, false, false},
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
    };

    static constexpr inline StructInfo value = {
        /* .name        = */ "ImuLowpassFilter",
        /* .title       = */ "Advanced Low-Pass Filter Settings",
        /* .docs        = */ "Advanced configuration for the IMU data quantity low-pass filters.\n\nDeprecated, use the lowpass filter (0x0C,0x54) command instead.\n\nThe scaled data quantities are by default filtered through a single-pole IIR low-pass filter\nwhich is configured with a -3dB cutoff frequency of half the reporting frequency (set by\ndecimation factor in the IMU Message Format command) to prevent aliasing on a per data\nquantity basis. This advanced configuration command allows for the cutoff frequency to\nbe configured independently of the data reporting frequency as well as allowing for a\ncomplete bypass of the digital low-pass filter.\n\nPossible data descriptors:\n0x04 - Scaled accelerometer data\n0x05 - Scaled gyro data\n0x06 - Scaled magnetometer data (if applicable)\n0x17 - Scaled pressure data (if applicable)",
        /* .parameters  = */ parameters,
    };
};

template<>
struct MetadataFor<commands_3dm::PpsSource::Source>
{
    using type = commands_3dm::PpsSource::Source;

    static constexpr inline EnumInfo::Entry entries[] = {
        { 0, "DISABLED", "PPS output is disabled. Not valid for PPS source command." },
        { 1, "RECEIVER_1", "PPS is provided by GNSS receiver 1." },
        { 2, "RECEIVER_2", "PPS is provided by GNSS receiver 2." },
        { 3, "GPIO", "PPS is provided to an external GPIO pin. Use the GPIO Setup command to choose and configure the pin." },
        { 4, "GENERATED", "PPS is generated from the system oscillator." },
    };

    static constexpr inline EnumInfo value = {
        /* .name    = */ "Source",
        /* .docs    = */ "",
        /* .type    = */ Type::U8,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<commands_3dm::PpsSource>
{
    using type = commands_3dm::PpsSource;

    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name        = */ "source",
            /* .docs        = */ "",
            /* .type        = */ {Type::ENUM, &MetadataFor<commands_3dm::PpsSource::Source>::value},
            /* .accessor    = */ utils::access<type, commands_3dm::PpsSource::Source, &type::source>,
            /* .functions   = */ {true, false, false, false, false},
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
    };

    static constexpr inline StructInfo value = {
        /* .name        = */ "PpsSource",
        /* .title       = */ "None",
        /* .docs        = */ "Controls the Pulse Per Second (PPS) source.",
        /* .parameters  = */ parameters,
    };
};

template<>
struct MetadataFor<commands_3dm::GpioConfig::Feature>
{
    using type = commands_3dm::GpioConfig::Feature;

    static constexpr inline EnumInfo::Entry entries[] = {
        { 0, "UNUSED", "The pin is not used. It may be technically possible to read the pin state in this mode, but this is not guaranteed to be true of all devices or pins." },
        { 1, "GPIO", "General purpose input or output. Use this for direct control of pin output state or to stream the state of the pin." },
        { 2, "PPS", "Pulse per second input or output." },
        { 3, "ENCODER", "Motor encoder/odometer input." },
        { 4, "TIMESTAMP", "Precision Timestamping. Use with Event Trigger Configuration (0x0C,0x2E)." },
        { 5, "UART", "UART data or control lines." },
    };

    static constexpr inline EnumInfo value = {
        /* .name    = */ "Feature",
        /* .docs    = */ "",
        /* .type    = */ Type::U8,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<commands_3dm::GpioConfig::Behavior>
{
    using type = commands_3dm::GpioConfig::Behavior;

    static constexpr inline EnumInfo::Entry entries[] = {
        { 0, "UNUSED", "Use 0 unless otherwise specified." },
        { 1, "GPIO_INPUT", "Pin will be an input. This can be used to stream or poll the value and is the default setting." },
        { 2, "GPIO_OUTPUT_LOW", "Pin is an output initially in the LOW state. This state will be restored during system startup if the configuration is saved." },
        { 3, "GPIO_OUTPUT_HIGH", "Pin is an output initially in the HIGH state. This state will be restored during system startup if the configuration is saved." },
        { 1, "PPS_INPUT", "Pin will receive the pulse-per-second signal. Only one pin can have this behavior. This will only work if the PPS Source command is configured to GPIO." },
        { 2, "PPS_OUTPUT", "Pin will transmit the pulse-per-second signal from the device." },
        { 1, "ENCODER_A", "Encoder 'A' quadrature input. Only one pin can have this behavior. The last command to set this behavior will take precedence." },
        { 2, "ENCODER_B", "Encoder 'B' quadrature input. Only one pin can have this behavior. The last command to set this behavior will take precedence." },
        { 1, "TIMESTAMP_RISING", "Rising edges will be timestamped." },
        { 2, "TIMESTAMP_FALLING", "Falling edges will be timestamped." },
        { 3, "TIMESTAMP_EITHER", "Both rising and falling edges will be timestamped." },
        { 33, "UART_PORT2_TX", "(0x21) UART port 2 transmit." },
        { 34, "UART_PORT2_RX", "(0x22) UART port 2 receive." },
        { 49, "UART_PORT3_TX", "(0x31) UART port 3 transmit." },
        { 50, "UART_PORT3_RX", "(0x32) UART port 3 receive." },
    };

    static constexpr inline EnumInfo value = {
        /* .name    = */ "Behavior",
        /* .docs    = */ "",
        /* .type    = */ Type::U8,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<commands_3dm::GpioConfig::PinMode>
{
    using type = commands_3dm::GpioConfig::PinMode;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { 1, "open_drain", "The pin will be an open-drain output. The state will be either LOW or FLOATING instead of LOW or HIGH, respectively. This is used to connect multiple open-drain outputs from several devices. An internal or external pull-up resistor is typically used in combination. The maximum voltage of an open drain output is subject to the device maximum input voltage range found in the specifications." },
        { 2, "pulldown", "The pin will have an internal pull-down resistor enabled. This is useful for connecting inputs to signals which can only be pulled high such as mechanical switches. Cannot be used in combination with pull-up. See the device specifications for the resistance value." },
        { 4, "pullup", "The pin will have an internal pull-up resistor enabled. Useful for connecting inputs to signals which can only be pulled low such as mechanical switches, or in combination with an open drain output. Cannot be used in combination with pull-down. See the device specifications for the resistance value. Use of this mode may restrict the maximum allowed input voltage. See the device datasheet for details." },
    };

    static constexpr inline BitfieldInfo value = {
        /* .name    = */ "PinMode",
        /* .docs    = */ "",
        /* .type    = */ Type::U8,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<commands_3dm::GpioConfig>
{
    using type = commands_3dm::GpioConfig;

    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name        = */ "pin",
            /* .docs        = */ "GPIO pin number counting from 1. For save, load, and default function selectors, this can be 0 to select all pins.",
            /* .type        = */ {Type::U8, nullptr},
            /* .accessor    = */ utils::access<type, uint8_t, &type::pin>,
            /* .functions   = */ {true, true, true, true, true,  true},
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
        {
            /* .name        = */ "feature",
            /* .docs        = */ "Determines how the pin will be used.",
            /* .type        = */ {Type::ENUM, &MetadataFor<commands_3dm::GpioConfig::Feature>::value},
            /* .accessor    = */ utils::access<type, commands_3dm::GpioConfig::Feature, &type::feature>,
            /* .functions   = */ {true, false, false, false, false},
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
        {
            /* .name        = */ "behavior",
            /* .docs        = */ "Select an appropriate value from the enumeration based on the selected feature (e.g. for PPS, select one of the values prefixed with PPS_.)",
            /* .type        = */ {Type::ENUM, &MetadataFor<commands_3dm::GpioConfig::Behavior>::value},
            /* .accessor    = */ utils::access<type, commands_3dm::GpioConfig::Behavior, &type::behavior>,
            /* .functions   = */ {true, false, false, false, false},
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
        {
            /* .name        = */ "pin_mode",
            /* .docs        = */ "GPIO configuration. May be restricted depending on device, pin, feature, and behavior. See device user manual.",
            /* .type        = */ {Type::BITFIELD, &MetadataFor<commands_3dm::GpioConfig::PinMode>::value},
            /* .accessor    = */ utils::access<type, commands_3dm::GpioConfig::PinMode, &type::pin_mode>,
            /* .functions   = */ {true, false, false, false, false},
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
    };

    static constexpr inline StructInfo value = {
        /* .name        = */ "GpioConfig",
        /* .title       = */ "GPIO Configuration",
        /* .docs        = */ "Configures the user GPIO pins on the connector for use with several built-in functions or for general input or output.\n\nGPIO pins are device-dependent. Some features are only available on\ncertain pins. Some behaviors require specific configurations.\nConsult the device user manual for restrictions and default settings.\n\nTo avoid glitches on GPIOs configured as an output in a mode other than\nGPIO, always configure the relevant function before setting up the pin\nwith this command. Otherwise, the pin state will be undefined between\nthis command and the one to set up the feature. For input pins, use\nthis command first so the state is well-defined when the feature is\ninitialized.\n\nSome configurations can only be active on one pin at a time. If such\nconfiguration is applied to a second pin, the second one will take\nprecedence and the original pin's configuration will be reset.\n",
        /* .parameters  = */ parameters,
    };
};

template<>
struct MetadataFor<commands_3dm::GpioState>
{
    using type = commands_3dm::GpioState;

    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name        = */ "pin",
            /* .docs        = */ "GPIO pin number counting from 1. Cannot be 0.",
            /* .type        = */ {Type::U8, nullptr},
            /* .accessor    = */ utils::access<type, uint8_t, &type::pin>,
            /* .functions   = */ {true, true, false, false, false,  true},
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
        {
            /* .name        = */ "state",
            /* .docs        = */ "The pin state.",
            /* .type        = */ {Type::BOOL, nullptr},
            /* .accessor    = */ utils::access<type, bool, &type::state>,
            /* .functions   = */ {true, false, false, false, false},
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
    };

    static constexpr inline StructInfo value = {
        /* .name        = */ "GpioState",
        /* .title       = */ "GPIO State",
        /* .docs        = */ "Allows the state of the pin to be read or controlled.\n\nThis command serves two purposes: 1) To allow reading the state of a pin via command,\nrather than polling a data quantity, and 2) to provide a way to set the output state\nwithout also having to specify the operating mode.\n\nThe state read back from the pin is the physical state of the pin, rather than a\nconfiguration value. The state can be read regardless of its configuration as long as\nthe device supports GPIO input on that pin. If the pin is set to an output, the read\nvalue would match the output value.\n\nWhile the state of a pin can always be set, it will only have an observable effect if\nthe pin is set to output mode.\n\nThis command does not support saving, loading, or resetting the state. Instead, use the\nGPIO Configuration command, which allows the initial state to be configured.",
        /* .parameters  = */ parameters,
    };
};

template<>
struct MetadataFor<commands_3dm::Odometer::Mode>
{
    using type = commands_3dm::Odometer::Mode;

    static constexpr inline EnumInfo::Entry entries[] = {
        { 0, "DISABLED", "Encoder is disabled." },
        { 2, "QUADRATURE", "Quadrature encoder mode." },
    };

    static constexpr inline EnumInfo value = {
        /* .name    = */ "Mode",
        /* .docs    = */ "",
        /* .type    = */ Type::U8,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<commands_3dm::Odometer>
{
    using type = commands_3dm::Odometer;

    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name        = */ "mode",
            /* .docs        = */ "Mode setting.",
            /* .type        = */ {Type::ENUM, &MetadataFor<commands_3dm::Odometer::Mode>::value},
            /* .accessor    = */ utils::access<type, commands_3dm::Odometer::Mode, &type::mode>,
            /* .functions   = */ {true, false, false, false, false},
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
        {
            /* .name        = */ "scaling",
            /* .docs        = */ "Encoder pulses per meter of distance traveled [pulses/m].\nDistance traveled is computed using the formula d = p / N * 2R * pi, where d is distance,\np is the number of pulses received, N is the encoder resolution, and R is the wheel radius.\nBy simplifying all of the parameters into one, the formula d = p / S is obtained, where s\nis the odometer scaling factor passed to this command. S is equivalent to N / (2R * pi) and\nhas units of pulses / meter. N is in units of 'A' pulses per revolution and R is in meters.\nMake this value negative if the odometer is mounted so that it rotates backwards.",
            /* .type        = */ {Type::FLOAT, nullptr},
            /* .accessor    = */ utils::access<type, float, &type::scaling>,
            /* .functions   = */ {true, false, false, false, false},
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
        {
            /* .name        = */ "uncertainty",
            /* .docs        = */ "Uncertainty in encoder counts to distance translation (1-sigma value) [m/m].",
            /* .type        = */ {Type::FLOAT, nullptr},
            /* .accessor    = */ utils::access<type, float, &type::uncertainty>,
            /* .functions   = */ {true, false, false, false, false},
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
    };

    static constexpr inline StructInfo value = {
        /* .name        = */ "Odometer",
        /* .title       = */ "Odometer Settings",
        /* .docs        = */ "Configures the hardware odometer interface.\n",
        /* .parameters  = */ parameters,
    };
};

template<>
struct MetadataFor<commands_3dm::GetEventSupport::Query>
{
    using type = commands_3dm::GetEventSupport::Query;

    static constexpr inline EnumInfo::Entry entries[] = {
        { 1, "TRIGGER_TYPES", "Query the supported trigger types and max count for each." },
        { 2, "ACTION_TYPES", "Query the supported action types and max count for each." },
    };

    static constexpr inline EnumInfo value = {
        /* .name    = */ "Query",
        /* .docs    = */ "",
        /* .type    = */ Type::U8,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<commands_3dm::GetEventSupport::Info>
{
    using type = commands_3dm::GetEventSupport::Info;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name        = */ "type",
            /* .docs        = */ "Trigger or action type, as defined in the respective setup command.",
            /* .type        = */ {Type::U8, nullptr},
            /* .accessor    = */ utils::access<type, uint8_t, &type::type>,
            /* .functions   = */ NO_FUNCTIONS,
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
        {
            /* .name        = */ "count",
            /* .docs        = */ "This is the maximum number of instances supported for this type.",
            /* .type        = */ {Type::U8, nullptr},
            /* .accessor    = */ utils::access<type, uint8_t, &type::count>,
            /* .functions   = */ NO_FUNCTIONS,
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
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
struct MetadataFor<commands_3dm::GetEventSupport>
{
    using type = commands_3dm::GetEventSupport;

    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name        = */ "query",
            /* .docs        = */ "What type of information to retrieve.",
            /* .type        = */ {Type::ENUM, &MetadataFor<commands_3dm::GetEventSupport::Query>::value},
            /* .accessor    = */ utils::access<type, commands_3dm::GetEventSupport::Query, &type::query>,
            /* .functions   = */ {true, false, false, false, false,  true},
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
    };

    static constexpr inline StructInfo value = {
        /* .name        = */ "GetEventSupport",
        /* .title       = */ "Get Supported Events",
        /* .docs        = */ "Lists the available trigger or action types.\n\nThere are a limited number of trigger and action slots available\nin the device. Up to M triggers and N actions can be configured at once\nin slots 1..M and 1..N respectively. M and N are identified by the\nmax_instances field in the response with the appropriate query selector.\n\nEach slot can be configured as one of a variety of different types of\ntriggers or actions. The supported types are enumerated in the response\nto this command. Additionally, there is a limit on the number of a given\ntype. In other words, while the device may support M triggers in total,\nonly a few of them maybe usable as a given type. This limit helps optimize\ndevice resources. The limit is identified in the count field.\n\nAll of the information in this command is available in the user manual.\nThis command provides a programmatic method for obtaining the information.\n",
        /* .parameters  = */ parameters,
    };
};

template<>
struct MetadataFor<commands_3dm::EventControl::Mode>
{
    using type = commands_3dm::EventControl::Mode;

    static constexpr inline EnumInfo::Entry entries[] = {
        { 0, "DISABLED", "Trigger is disabled." },
        { 1, "ENABLED", "Trigger is enabled and will work normally." },
        { 2, "TEST", "Forces the trigger to the active state for testing purposes." },
        { 3, "TEST_PULSE", "Trigger is forced to the active state for one event cycle only. After the test cycle, the mode reverts to the previous state (either enabled or disabled)." },
    };

    static constexpr inline EnumInfo value = {
        /* .name    = */ "Mode",
        /* .docs    = */ "",
        /* .type    = */ Type::U8,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<commands_3dm::EventControl>
{
    using type = commands_3dm::EventControl;

    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name        = */ "instance",
            /* .docs        = */ "Trigger instance to affect. 0 can be used to apply the mode to all\nconfigured triggers, except when the function selector is READ.",
            /* .type        = */ {Type::U8, nullptr},
            /* .accessor    = */ utils::access<type, uint8_t, &type::instance>,
            /* .functions   = */ {true, true, true, true, true,  true},
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
        {
            /* .name        = */ "mode",
            /* .docs        = */ "How to change the trigger state. Except when instance is 0, the\ncorresponding trigger must be configured, i.e. not have type 0.",
            /* .type        = */ {Type::ENUM, &MetadataFor<commands_3dm::EventControl::Mode>::value},
            /* .accessor    = */ utils::access<type, commands_3dm::EventControl::Mode, &type::mode>,
            /* .functions   = */ {true, false, false, false, false},
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
    };

    static constexpr inline StructInfo value = {
        /* .name        = */ "EventControl",
        /* .title       = */ "Event Control",
        /* .docs        = */ "Enables or disables event triggers.\n\nTriggers can be disabled, enabled, and tested. While disabled, a trigger will\nnot evaluate its logic and effective behave like no trigger is configured.\nA disabled trigger will not activate any actions. Triggers are disabled by default.\n\nUse this command to enable (or disable) a trigger, or to place it into a test mode.\nWhen in test mode, the trigger logic is disabled but the output is forced to\nthe active state, meaning that it will behave as if the trigger logic is satisfied\nand any associated actions will execute.",
        /* .parameters  = */ parameters,
    };
};

template<>
struct MetadataFor<commands_3dm::GetEventTriggerStatus::Status>
{
    using type = commands_3dm::GetEventTriggerStatus::Status;

    static constexpr inline BitfieldInfo::Entry entries[] = {
        { 1, "active", "True if the trigger is currently active (either due to its logic or being in test mode)." },
        { 2, "enabled", "True if the trigger is enabled." },
        { 4, "test", "True if the trigger is in test mode." },
    };

    static constexpr inline BitfieldInfo value = {
        /* .name    = */ "Status",
        /* .docs    = */ "",
        /* .type    = */ Type::U8,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<commands_3dm::GetEventTriggerStatus::Entry>
{
    using type = commands_3dm::GetEventTriggerStatus::Entry;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name        = */ "type",
            /* .docs        = */ "Configured trigger type.",
            /* .type        = */ {Type::U8, nullptr},
            /* .accessor    = */ utils::access<type, uint8_t, &type::type>,
            /* .functions   = */ NO_FUNCTIONS,
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
        {
            /* .name        = */ "status",
            /* .docs        = */ "Trigger status.",
            /* .type        = */ {Type::BITFIELD, &MetadataFor<commands_3dm::GetEventTriggerStatus::Status>::value},
            /* .accessor    = */ utils::access<type, commands_3dm::GetEventTriggerStatus::Status, &type::status>,
            /* .functions   = */ NO_FUNCTIONS,
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
    };

    static constexpr inline StructInfo value = {
        /* .name        = */ "Entry",
        /* .title       = */ "Entry",
        /* .docs        = */ "",
        /* .parameters  = */ parameters,
    };
};

template<>
struct MetadataFor<commands_3dm::GetEventTriggerStatus>
{
    using type = commands_3dm::GetEventTriggerStatus;

    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name        = */ "requested_count",
            /* .docs        = */ "Number of entries requested. If 0, requests all trigger slots.",
            /* .type        = */ {Type::U8, nullptr},
            /* .accessor    = */ utils::access<type, uint8_t, &type::requested_count>,
            /* .functions   = */ {true, false, false, false, false},
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
        {
            /* .name        = */ "requested_instances",
            /* .docs        = */ "List of trigger instances to query.",
            /* .type        = */ {Type::U8, nullptr},
            /* .accessor    = */ utils::access<type, uint8_t, &type::requested_instances>,
            /* .functions   = */ {true, false, false, false, false},
            /* .count       = */ 20,
            /* .counter_idx = */ microstrain::Index(0) /* requested_count */,
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
    };

    static constexpr inline StructInfo value = {
        /* .name        = */ "GetEventTriggerStatus",
        /* .title       = */ "Get Trigger Status",
        /* .docs        = */ "",
        /* .parameters  = */ parameters,
    };
};

template<>
struct MetadataFor<commands_3dm::GetEventActionStatus::Entry>
{
    using type = commands_3dm::GetEventActionStatus::Entry;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name        = */ "action_type",
            /* .docs        = */ "Configured action type.",
            /* .type        = */ {Type::U8, nullptr},
            /* .accessor    = */ utils::access<type, uint8_t, &type::action_type>,
            /* .functions   = */ NO_FUNCTIONS,
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
        {
            /* .name        = */ "trigger_id",
            /* .docs        = */ "Associated trigger instance.",
            /* .type        = */ {Type::U8, nullptr},
            /* .accessor    = */ utils::access<type, uint8_t, &type::trigger_id>,
            /* .functions   = */ NO_FUNCTIONS,
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
    };

    static constexpr inline StructInfo value = {
        /* .name        = */ "Entry",
        /* .title       = */ "Entry",
        /* .docs        = */ "",
        /* .parameters  = */ parameters,
    };
};

template<>
struct MetadataFor<commands_3dm::GetEventActionStatus>
{
    using type = commands_3dm::GetEventActionStatus;

    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name        = */ "requested_count",
            /* .docs        = */ "Number of entries requested. If 0, requests all action slots.",
            /* .type        = */ {Type::U8, nullptr},
            /* .accessor    = */ utils::access<type, uint8_t, &type::requested_count>,
            /* .functions   = */ {true, false, false, false, false},
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
        {
            /* .name        = */ "requested_instances",
            /* .docs        = */ "List of action instances to query.",
            /* .type        = */ {Type::U8, nullptr},
            /* .accessor    = */ utils::access<type, uint8_t, &type::requested_instances>,
            /* .functions   = */ {true, false, false, false, false},
            /* .count       = */ 20,
            /* .counter_idx = */ microstrain::Index(0) /* requested_count */,
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
    };

    static constexpr inline StructInfo value = {
        /* .name        = */ "GetEventActionStatus",
        /* .title       = */ "Get Action Status",
        /* .docs        = */ "",
        /* .parameters  = */ parameters,
    };
};

template<>
struct MetadataFor<commands_3dm::EventTrigger::GpioParams::Mode>
{
    using type = commands_3dm::EventTrigger::GpioParams::Mode;

    static constexpr inline EnumInfo::Entry entries[] = {
        { 0, "DISABLED", "The pin will have no effect and the trigger will never activate." },
        { 1, "WHILE_HIGH", "The trigger will be active while the pin is high." },
        { 2, "WHILE_LOW", "The trigger will be active while the pin is low." },
        { 4, "EDGE", "Use if the pin is configured for timestamping via the 3DM Gpio Configuration command (0x0C41)." },
    };

    static constexpr inline EnumInfo value = {
        /* .name    = */ "Mode",
        /* .docs    = */ "",
        /* .type    = */ Type::U8,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<commands_3dm::EventTrigger::GpioParams>
{
    using type = commands_3dm::EventTrigger::GpioParams;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name        = */ "pin",
            /* .docs        = */ "GPIO pin number.",
            /* .type        = */ {Type::U8, nullptr},
            /* .accessor    = */ utils::access<type, uint8_t, &type::pin>,
            /* .functions   = */ NO_FUNCTIONS,
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
        {
            /* .name        = */ "mode",
            /* .docs        = */ "How the pin state affects the trigger.",
            /* .type        = */ {Type::ENUM, &MetadataFor<commands_3dm::EventTrigger::GpioParams::Mode>::value},
            /* .accessor    = */ utils::access<type, commands_3dm::EventTrigger::GpioParams::Mode, &type::mode>,
            /* .functions   = */ NO_FUNCTIONS,
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
    };

    static constexpr inline StructInfo value = {
        /* .name        = */ "GpioParams",
        /* .title       = */ "Gpioparams",
        /* .docs        = */ "",
        /* .parameters  = */ parameters,
    };
};

template<>
struct MetadataFor<commands_3dm::EventTrigger::ThresholdParams::Type>
{
    using type = commands_3dm::EventTrigger::ThresholdParams::Type;

    static constexpr inline EnumInfo::Entry entries[] = {
        { 1, "WINDOW", "Window comparison. Trigger is active if low_thres &lt;= value &lt;= high_thres. If the thresholds are reversed, the trigger is active when value &lt; high_thres or value &gt; low_thres." },
        { 2, "INTERVAL", "Trigger at evenly-spaced intervals. Normally used with time fields to trigger periodically. Trigger is active when (value % interval) &lt;= int_thres. If the thresholds are reversed (high_thres &lt; low_thres) then the trigger is active when (value % low_thres) &gt; high_thres." },
    };

    static constexpr inline EnumInfo value = {
        /* .name    = */ "Type",
        /* .docs    = */ "",
        /* .type    = */ Type::U8,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<commands_3dm::EventTrigger::ThresholdParams>
{
    using type = commands_3dm::EventTrigger::ThresholdParams;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name        = */ "desc_set",
            /* .docs        = */ "Descriptor set of target data quantity.",
            /* .type        = */ {Type::U8, nullptr},
            /* .accessor    = */ utils::access<type, uint8_t, &type::desc_set>,
            /* .functions   = */ NO_FUNCTIONS,
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
        {
            /* .name        = */ "field_desc",
            /* .docs        = */ "Field descriptor of target data quantity.",
            /* .type        = */ {Type::U8, nullptr},
            /* .accessor    = */ utils::access<type, uint8_t, &type::field_desc>,
            /* .functions   = */ NO_FUNCTIONS,
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
        {
            /* .name        = */ "param_id",
            /* .docs        = */ "1-based index of the target parameter within the MIP field. E.g. for Scaled Accel (0x80,0x04) a value of 2 would represent the Y axis.",
            /* .type        = */ {Type::U8, nullptr},
            /* .accessor    = */ utils::access<type, uint8_t, &type::param_id>,
            /* .functions   = */ NO_FUNCTIONS,
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
        {
            /* .name        = */ "type",
            /* .docs        = */ "Determines the type of comparison.",
            /* .type        = */ {Type::ENUM, &MetadataFor<commands_3dm::EventTrigger::ThresholdParams::Type>::value},
            /* .accessor    = */ utils::access<type, commands_3dm::EventTrigger::ThresholdParams::Type, &type::type>,
            /* .functions   = */ NO_FUNCTIONS,
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
        {
            /* .name        = */ "",
            /* .docs        = */ "",
            /* .type        = */ {Type::UNION, &MetadataFor<commands_3dm::EventTrigger::ThresholdParams::>::value},
            /* .accessor    = */ utils::access<type, commands_3dm::EventTrigger::ThresholdParams::, &type::>,
            /* .functions   = */ NO_FUNCTIONS,
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
        {
            /* .name        = */ "",
            /* .docs        = */ "",
            /* .type        = */ {Type::UNION, &MetadataFor<commands_3dm::EventTrigger::ThresholdParams::>::value},
            /* .accessor    = */ utils::access<type, commands_3dm::EventTrigger::ThresholdParams::, &type::>,
            /* .functions   = */ NO_FUNCTIONS,
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
    };

    static constexpr inline StructInfo value = {
        /* .name        = */ "ThresholdParams",
        /* .title       = */ "Thresholdparams",
        /* .docs        = */ "Comparison of a supported MIP field parameter against a set of thresholds.\n\nTriggers when a data quantity meets the comparison criteria. The comparison can be either\na window comparison with high and low thresholds or a periodic interval.\n\nThe data quantity is identified by the MIP descriptor set, field descriptor, and parameter number.\nE.g. Scaled acceleration in the Z direction is specified with desc_set=0x80 (sensor data),\nfield_desc=0x04 (scaled accel), and param_id=3 (the third parameter and Z axis).\n\nThe window comparison can be used for a variety of purposes, such as disabling\na robot's drive motors if it tips over. In this case, a window comparison could\nbe set up to monitor the roll angle, (0x80,0x0C,3). The lower threshold would be set\nto -pi/2 radians and the upper threshold to pi/2 radians.\n\nThe interval trigger can be used to perform an action periodically if used with\na time field. E.g. to execute the action every 16 ms, set an interval comparison on\nthe GPS time of week parameter (0x80,0xD3,1) with high_thres set to 0.016. The lower\nthreshold determines how long the trigger is active within the 16-ms period.\n\nEither comparison type can be inverted by reversing the threshold values; setting\nlow_thres > high_thres will result in the reverse condition.\n",
        /* .parameters  = */ parameters,
    };
};

template<>
struct MetadataFor<commands_3dm::EventTrigger::CombinationParams>
{
    using type = commands_3dm::EventTrigger::CombinationParams;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name        = */ "logic_table",
            /* .docs        = */ "The last column of a truth table describing the output given the state of each input.",
            /* .type        = */ {Type::U16, nullptr},
            /* .accessor    = */ utils::access<type, uint16_t, &type::logic_table>,
            /* .functions   = */ NO_FUNCTIONS,
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
        {
            /* .name        = */ "input_triggers",
            /* .docs        = */ "List of trigger IDs for inputs. Use 0 for unused inputs.",
            /* .type        = */ {Type::U8, nullptr},
            /* .accessor    = */ utils::access<type, uint8_t, &type::input_triggers>,
            /* .functions   = */ NO_FUNCTIONS,
            /* .count       = */ 4,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
    };

    static constexpr inline StructInfo value = {
        /* .name        = */ "CombinationParams",
        /* .title       = */ "Combinationparams",
        /* .docs        = */ "",
        /* .parameters  = */ parameters,
    };
};

template<>
struct MetadataFor<commands_3dm::EventTrigger::Type>
{
    using type = commands_3dm::EventTrigger::Type;

    static constexpr inline EnumInfo::Entry entries[] = {
        { 0, "NONE", "No trigger selected. The state will always be inactive." },
        { 1, "GPIO", "Trigger based on the state of a GPIO pin. See GpioParams." },
        { 2, "THRESHOLD", "Compare a data quantity against a high and low threshold. See ThresholdParams." },
        { 3, "COMBINATION", "Logical combination of two or more triggers. See CombinationParams." },
    };

    static constexpr inline EnumInfo value = {
        /* .name    = */ "Type",
        /* .docs    = */ "",
        /* .type    = */ Type::U8,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<commands_3dm::EventTrigger::Parameters>
{
    using type = commands_3dm::EventTrigger::Parameters;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name        = */ "gpio",
            /* .docs        = */ "",
            /* .type        = */ {Type::STRUCT, &MetadataFor<commands_3dm::EventTrigger::GpioParams>::value},
            /* .accessor    = */ utils::access<type, commands_3dm::EventTrigger::GpioParams, &type::gpio>,
            /* .functions   = */ NO_FUNCTIONS,
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
        {
            /* .name        = */ "threshold",
            /* .docs        = */ "",
            /* .type        = */ {Type::STRUCT, &MetadataFor<commands_3dm::EventTrigger::ThresholdParams>::value},
            /* .accessor    = */ utils::access<type, commands_3dm::EventTrigger::ThresholdParams, &type::threshold>,
            /* .functions   = */ NO_FUNCTIONS,
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
        {
            /* .name        = */ "combination",
            /* .docs        = */ "",
            /* .type        = */ {Type::STRUCT, &MetadataFor<commands_3dm::EventTrigger::CombinationParams>::value},
            /* .accessor    = */ utils::access<type, commands_3dm::EventTrigger::CombinationParams, &type::combination>,
            /* .functions   = */ NO_FUNCTIONS,
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
    };

    static constexpr inline StructInfo value = {
        /* .name        = */ "Parameters",
        /* .title       = */ "None",
        /* .docs        = */ "",
        /* .parameters  = */ parameters,
    };
};

template<>
struct MetadataFor<commands_3dm::EventTrigger>
{
    using type = commands_3dm::EventTrigger;

    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name        = */ "instance",
            /* .docs        = */ "Trigger number. When function is SAVE, LOAD, or DEFAULT, this can be 0 to apply to all instances.",
            /* .type        = */ {Type::U8, nullptr},
            /* .accessor    = */ utils::access<type, uint8_t, &type::instance>,
            /* .functions   = */ {true, true, true, true, true,  true},
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
        {
            /* .name        = */ "type",
            /* .docs        = */ "Type of trigger to configure.",
            /* .type        = */ {Type::ENUM, &MetadataFor<commands_3dm::EventTrigger::Type>::value},
            /* .accessor    = */ utils::access<type, commands_3dm::EventTrigger::Type, &type::type>,
            /* .functions   = */ {true, false, false, false, false},
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
        {
            /* .name        = */ "parameters",
            /* .docs        = */ "",
            /* .type        = */ {Type::UNION, &MetadataFor<commands_3dm::EventTrigger::Parameters>::value},
            /* .accessor    = */ utils::access<type, commands_3dm::EventTrigger::Parameters, &type::parameters>,
            /* .functions   = */ {true, false, false, false, false},
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
    };

    static constexpr inline StructInfo value = {
        /* .name        = */ "EventTrigger",
        /* .title       = */ "Event Trigger Configuration",
        /* .docs        = */ "Configures various types of event triggers.",
        /* .parameters  = */ parameters,
    };
};

template<>
struct MetadataFor<commands_3dm::EventAction::GpioParams::Mode>
{
    using type = commands_3dm::EventAction::GpioParams::Mode;

    static constexpr inline EnumInfo::Entry entries[] = {
        { 0, "DISABLED", "Pin state will not be changed." },
        { 1, "ACTIVE_HIGH", "Pin will be set high when the trigger is active and low otherwise." },
        { 2, "ACTIVE_LOW", "Pin will be set low when the trigger is active and high otherwise." },
        { 5, "ONESHOT_HIGH", "Pin will be set high each time the trigger activates. It will not be set low." },
        { 6, "ONESHOT_LOW", "Pin will be set low each time the trigger activates. It will not be set high." },
        { 7, "TOGGLE", "Pin will change to the opposite state each time the trigger activates." },
    };

    static constexpr inline EnumInfo value = {
        /* .name    = */ "Mode",
        /* .docs    = */ "",
        /* .type    = */ Type::U8,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<commands_3dm::EventAction::GpioParams>
{
    using type = commands_3dm::EventAction::GpioParams;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name        = */ "pin",
            /* .docs        = */ "GPIO pin number.",
            /* .type        = */ {Type::U8, nullptr},
            /* .accessor    = */ utils::access<type, uint8_t, &type::pin>,
            /* .functions   = */ NO_FUNCTIONS,
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
        {
            /* .name        = */ "mode",
            /* .docs        = */ "Behavior of the pin.",
            /* .type        = */ {Type::ENUM, &MetadataFor<commands_3dm::EventAction::GpioParams::Mode>::value},
            /* .accessor    = */ utils::access<type, commands_3dm::EventAction::GpioParams::Mode, &type::mode>,
            /* .functions   = */ NO_FUNCTIONS,
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
    };

    static constexpr inline StructInfo value = {
        /* .name        = */ "GpioParams",
        /* .title       = */ "Gpioparams",
        /* .docs        = */ "",
        /* .parameters  = */ parameters,
    };
};

template<>
struct MetadataFor<commands_3dm::EventAction::MessageParams>
{
    using type = commands_3dm::EventAction::MessageParams;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name        = */ "desc_set",
            /* .docs        = */ "MIP data descriptor set.",
            /* .type        = */ {Type::U8, nullptr},
            /* .accessor    = */ utils::access<type, uint8_t, &type::desc_set>,
            /* .functions   = */ NO_FUNCTIONS,
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
        {
            /* .name        = */ "decimation",
            /* .docs        = */ "Decimation from the base rate.\nIf 0, a packet is emitted each time the trigger activates.\nOtherwise, packets will be streamed while the trigger is active.\nThe internal decimation counter is reset if the trigger deactivates.",
            /* .type        = */ {Type::U16, nullptr},
            /* .accessor    = */ utils::access<type, uint16_t, &type::decimation>,
            /* .functions   = */ NO_FUNCTIONS,
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
        {
            /* .name        = */ "num_fields",
            /* .docs        = */ "Number of mip fields in the packet. Limited to 12.",
            /* .type        = */ {Type::U8, nullptr},
            /* .accessor    = */ utils::access<type, uint8_t, &type::num_fields>,
            /* .functions   = */ NO_FUNCTIONS,
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
        {
            /* .name        = */ "descriptors",
            /* .docs        = */ "List of field descriptors.",
            /* .type        = */ {Type::U8, nullptr},
            /* .accessor    = */ utils::access<type, uint8_t, &type::descriptors>,
            /* .functions   = */ NO_FUNCTIONS,
            /* .count       = */ 20,
            /* .counter_idx = */ microstrain::Index(2) /* num_fields */,
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
    };

    static constexpr inline StructInfo value = {
        /* .name        = */ "MessageParams",
        /* .title       = */ "Messageparams",
        /* .docs        = */ "",
        /* .parameters  = */ parameters,
    };
};

template<>
struct MetadataFor<commands_3dm::EventAction::Type>
{
    using type = commands_3dm::EventAction::Type;

    static constexpr inline EnumInfo::Entry entries[] = {
        { 0, "NONE", "No action. Parameters should be empty." },
        { 1, "GPIO", "Control the state of a GPIO pin. See GpioParameters." },
        { 2, "MESSAGE", "Output a data packet. See MessageParameters." },
    };

    static constexpr inline EnumInfo value = {
        /* .name    = */ "Type",
        /* .docs    = */ "",
        /* .type    = */ Type::U8,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<commands_3dm::EventAction::Parameters>
{
    using type = commands_3dm::EventAction::Parameters;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name        = */ "gpio",
            /* .docs        = */ "Gpio parameters, if type == GPIO. Ignore otherwise.",
            /* .type        = */ {Type::STRUCT, &MetadataFor<commands_3dm::EventAction::GpioParams>::value},
            /* .accessor    = */ utils::access<type, commands_3dm::EventAction::GpioParams, &type::gpio>,
            /* .functions   = */ NO_FUNCTIONS,
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
        {
            /* .name        = */ "message",
            /* .docs        = */ "Message parameters, if type == MESSAGE. Ignore otherwise.",
            /* .type        = */ {Type::STRUCT, &MetadataFor<commands_3dm::EventAction::MessageParams>::value},
            /* .accessor    = */ utils::access<type, commands_3dm::EventAction::MessageParams, &type::message>,
            /* .functions   = */ NO_FUNCTIONS,
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
    };

    static constexpr inline StructInfo value = {
        /* .name        = */ "Parameters",
        /* .title       = */ "None",
        /* .docs        = */ "",
        /* .parameters  = */ parameters,
    };
};

template<>
struct MetadataFor<commands_3dm::EventAction>
{
    using type = commands_3dm::EventAction;

    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name        = */ "instance",
            /* .docs        = */ "Action number. When function is SAVE, LOAD, or DEFAULT, this can be 0 to apply to all instances.",
            /* .type        = */ {Type::U8, nullptr},
            /* .accessor    = */ utils::access<type, uint8_t, &type::instance>,
            /* .functions   = */ {true, true, true, true, true,  true},
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
        {
            /* .name        = */ "trigger",
            /* .docs        = */ "Trigger ID number.",
            /* .type        = */ {Type::U8, nullptr},
            /* .accessor    = */ utils::access<type, uint8_t, &type::trigger>,
            /* .functions   = */ {true, false, false, false, false},
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
        {
            /* .name        = */ "type",
            /* .docs        = */ "Type of action to configure.",
            /* .type        = */ {Type::ENUM, &MetadataFor<commands_3dm::EventAction::Type>::value},
            /* .accessor    = */ utils::access<type, commands_3dm::EventAction::Type, &type::type>,
            /* .functions   = */ {true, false, false, false, false},
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
        {
            /* .name        = */ "parameters",
            /* .docs        = */ "",
            /* .type        = */ {Type::UNION, &MetadataFor<commands_3dm::EventAction::Parameters>::value},
            /* .accessor    = */ utils::access<type, commands_3dm::EventAction::Parameters, &type::parameters>,
            /* .functions   = */ {true, false, false, false, false},
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
    };

    static constexpr inline StructInfo value = {
        /* .name        = */ "EventAction",
        /* .title       = */ "Event Action Configuration",
        /* .docs        = */ "Configures various types of event actions.",
        /* .parameters  = */ parameters,
    };
};

template<>
struct MetadataFor<commands_3dm::AccelBias>
{
    using type = commands_3dm::AccelBias;

    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name        = */ "bias",
            /* .docs        = */ "accelerometer bias in the sensor frame (x,y,z) [g]",
            /* .type        = */ {Type::STRUCT, &MetadataFor<commands_3dm::Vector3f>::value},
            /* .accessor    = */ utils::access<type, commands_3dm::Vector3f, &type::bias>,
            /* .functions   = */ {true, false, false, false, false},
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
    };

    static constexpr inline StructInfo value = {
        /* .name        = */ "AccelBias",
        /* .title       = */ "Configure Accel Bias",
        /* .docs        = */ "Configures the user specified accelerometer bias\n\nThe user specified bias is subtracted from the calibrated accelerometer output.  Value is input in the sensor frame.",
        /* .parameters  = */ parameters,
    };
};

template<>
struct MetadataFor<commands_3dm::GyroBias>
{
    using type = commands_3dm::GyroBias;

    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name        = */ "bias",
            /* .docs        = */ "gyro bias in the sensor frame (x,y,z) [radians/second]",
            /* .type        = */ {Type::STRUCT, &MetadataFor<commands_3dm::Vector3f>::value},
            /* .accessor    = */ utils::access<type, commands_3dm::Vector3f, &type::bias>,
            /* .functions   = */ {true, false, false, false, false},
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
    };

    static constexpr inline StructInfo value = {
        /* .name        = */ "GyroBias",
        /* .title       = */ "Configure Gyro Bias",
        /* .docs        = */ "Configures the user specified gyroscope bias\n\nThe user specified bias is subtracted from the calibrated angular rate output.  Value is input in the sensor frame.",
        /* .parameters  = */ parameters,
    };
};

template<>
struct MetadataFor<commands_3dm::CaptureGyroBias>
{
    using type = commands_3dm::CaptureGyroBias;

    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name        = */ "averaging_time_ms",
            /* .docs        = */ "Averaging time [milliseconds]",
            /* .type        = */ {Type::U16, nullptr},
            /* .accessor    = */ utils::access<type, uint16_t, &type::averaging_time_ms>,
            /* .functions   = */ {true, false, false, false, false},
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
    };

    static constexpr inline StructInfo value = {
        /* .name        = */ "CaptureGyroBias",
        /* .title       = */ "Capture Gyro Bias",
        /* .docs        = */ "Samples gyro for a specified time range and writes the averaged result to the Gyro Bias vector in RAM\n\nThe device will average the gyro output for the duration of 'averaging_time_ms.' To store the resulting vector\nin non-volatile memory, use the Set Gyro Bias command.\nIMPORTANT: The device must be stationary and experiencing minimum vibration for the duration of 'averaging_time_ms'\nAveraging Time range: 1000 to 30,000",
        /* .parameters  = */ parameters,
    };
};

template<>
struct MetadataFor<commands_3dm::MagHardIronOffset>
{
    using type = commands_3dm::MagHardIronOffset;

    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name        = */ "offset",
            /* .docs        = */ "hard iron offset in the sensor frame (x,y,z) [Gauss]",
            /* .type        = */ {Type::STRUCT, &MetadataFor<commands_3dm::Vector3f>::value},
            /* .accessor    = */ utils::access<type, commands_3dm::Vector3f, &type::offset>,
            /* .functions   = */ {true, false, false, false, false},
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
    };

    static constexpr inline StructInfo value = {
        /* .name        = */ "MagHardIronOffset",
        /* .title       = */ "Magnetometer Hard Iron Offset",
        /* .docs        = */ "Configure the user specified magnetometer hard iron offset vector\n\nThe values for this offset are determined empirically by external software algorithms\nbased on calibration data taken after the device is installed in its application. These values\ncan be obtained and set by using the LORD 'MIP Iron Calibration' application.\nAlternatively, on some systems, the auto-mag calibration feature may be used to capture these values in-run.\nThe offset is applied to the scaled magnetometer vector prior to output.",
        /* .parameters  = */ parameters,
    };
};

template<>
struct MetadataFor<commands_3dm::MagSoftIronMatrix>
{
    using type = commands_3dm::MagSoftIronMatrix;

    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name        = */ "offset",
            /* .docs        = */ "soft iron matrix [dimensionless]",
            /* .type        = */ {Type::STRUCT, &MetadataFor<commands_3dm::Matrix3f>::value},
            /* .accessor    = */ utils::access<type, commands_3dm::Matrix3f, &type::offset>,
            /* .functions   = */ {true, false, false, false, false},
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
    };

    static constexpr inline StructInfo value = {
        /* .name        = */ "MagSoftIronMatrix",
        /* .title       = */ "Magnetometer Soft Iron Matrix",
        /* .docs        = */ "Configure the user specified magnetometer soft iron offset matrix\n\nThe values for this matrix are determined empirically by external software algorithms\nbased on calibration data taken after the device is installed in its application. These values\ncan be obtained and set by using the LORD 'MIP Iron Calibration' application.\nAlternatively, on some systems, the auto-mag calibration feature may be used to capture these values in-run.\nThe matrix is applied to the scaled magnetometer vector prior to output.\n\nThe matrix is in row major order:\nEQSTART M = \begin{bmatrix} 0 &amp; 1 &amp; 2 \\ 3 &amp; 4 &amp; 5 \\ 6 &amp; 7 &amp; 8 \end{bmatrix} EQEND\n",
        /* .parameters  = */ parameters,
    };
};

template<>
struct MetadataFor<commands_3dm::ConingScullingEnable>
{
    using type = commands_3dm::ConingScullingEnable;

    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name        = */ "enable",
            /* .docs        = */ "If true, coning and sculling compensation is enabled.",
            /* .type        = */ {Type::BOOL, nullptr},
            /* .accessor    = */ utils::access<type, bool, &type::enable>,
            /* .functions   = */ {true, false, false, false, false},
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
    };

    static constexpr inline StructInfo value = {
        /* .name        = */ "ConingScullingEnable",
        /* .title       = */ "Coning and Sculling Enable",
        /* .docs        = */ "Controls the Coning and Sculling Compenstation setting.",
        /* .parameters  = */ parameters,
    };
};

template<>
struct MetadataFor<commands_3dm::Sensor2VehicleTransformEuler>
{
    using type = commands_3dm::Sensor2VehicleTransformEuler;

    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name        = */ "roll",
            /* .docs        = */ "[radians]",
            /* .type        = */ {Type::FLOAT, nullptr},
            /* .accessor    = */ utils::access<type, float, &type::roll>,
            /* .functions   = */ {true, false, false, false, false},
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
        {
            /* .name        = */ "pitch",
            /* .docs        = */ "[radians]",
            /* .type        = */ {Type::FLOAT, nullptr},
            /* .accessor    = */ utils::access<type, float, &type::pitch>,
            /* .functions   = */ {true, false, false, false, false},
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
        {
            /* .name        = */ "yaw",
            /* .docs        = */ "[radians]",
            /* .type        = */ {Type::FLOAT, nullptr},
            /* .accessor    = */ utils::access<type, float, &type::yaw>,
            /* .functions   = */ {true, false, false, false, false},
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
    };

    static constexpr inline StructInfo value = {
        /* .name        = */ "Sensor2VehicleTransformEuler",
        /* .title       = */ "Sensor to Vehicle Frame Transformation Euler",
        /* .docs        = */ "Sets the sensor-to-vehicle frame transformation using Yaw, Pitch, Roll Euler angles.\nThese are the Yaw, Pitch, and Roll mounting angles of the sensor with respect to vehicle frame of reference,\nand describe the transformation of vectors from the sensor body frame to the vehicle frame.<br/>\nNote: This is the transformation, the inverse of the rotation defined in our legacy products.<br/>\nThe transformation may be stored in the device as a matrix or quaternion.  When Euler angles are read back from the device, they may not\nbe exactly equal to the Euler angles used to set the transformation, but they are functionally equivalent, such that they result in the same transformation.<br/>\n<br/><br/>\nThis transformation to the vehicle frame will be applied to the following output quantities:<br/><br/>\nIMU:<br/>\nScaled Acceleration<br/>\nScaled Gyro<br/>\nScaled Magnetometer<br/>\nDelta Theta<br/>\nDelta Velocity<br/>\nComplementary Filter Orientation<br/>\n<br/><br/>\nEstimation Filter:<br/>\nEstimated Orientation, Quaternion<br/>\nEstimated Orientation, Matrix<br/>\nEstimated Orientation, Euler Angles<br/>\nEstimated Linear Acceleration<br/>\nEstimated Angular Rate<br/>\nEstimated Gravity Vector<br/>\n<br/>\nChanging this setting will force all low-pass filters, the complementary filter, and the estimation filter to reset.",
        /* .parameters  = */ parameters,
    };
};

template<>
struct MetadataFor<commands_3dm::Sensor2VehicleTransformQuaternion>
{
    using type = commands_3dm::Sensor2VehicleTransformQuaternion;

    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name        = */ "q",
            /* .docs        = */ "Unit length quaternion representing transform [w, i, j, k]",
            /* .type        = */ {Type::STRUCT, &MetadataFor<commands_3dm::Quatf>::value},
            /* .accessor    = */ utils::access<type, commands_3dm::Quatf, &type::q>,
            /* .functions   = */ {true, false, false, false, false},
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
    };

    static constexpr inline StructInfo value = {
        /* .name        = */ "Sensor2VehicleTransformQuaternion",
        /* .title       = */ "Sensor to Vehicle Frame Transformation Quaternion",
        /* .docs        = */ "Set the sensor to vehicle frame transformation using unit length quaternion.\n\nNote: This is the transformation, the inverse of the rotation.\n\nThis quaternion describes the transformation of vectors from the sensor body frame to the vehicle frame of reference, and satisfies the following relationship:<br/>\n\nEQSTART p^{veh} = q^{-1} p^{sen} q EQEND<br/>\n\nWhere:<br/>\nEQSTART q = (q_w, q_x, q_y, q_z) EQEND is the quaternion describing the transformation. <br/>\nEQSTART p^{sen} = (0, v^{sen}_x, v^{sen}_y, v^{sen}_z) EQEND and EQSTART v^{sen} EQEND is a 3-element vector expressed in the sensor body frame.<br/>\nEQSTART p^{veh} = (0, v^{veh}_x, v^{veh}_y, v^{veh}_z) EQEND and EQSTART v^{veh} EQEND is a 3-element vector expressed in the vehicle frame.<br/>\n\nThe transformation may be stored in the device as a matrix or a quaternion.  When the quaternion is read back from the device, it may not\nbe exactly equal to the quaternion used to set the transformation, but it is functionally equivalent.<br/>\n<br/><br/>\nThis transformation affects the following output quantities:<br/><br/>\nIMU:<br/>\nScaled Acceleration<br/>\nScaled Gyro<br/>\nScaled Magnetometer<br/>\nDelta Theta<br/>\nDelta Velocity<br/>\n<br/><br/>\nEstimation Filter:<br/>\nEstimated Orientation, Quaternion<br/>\nEstimated Orientation, Matrix<br/>\nEstimated Orientation, Euler Angles<br/>\nEstimated Linear Acceleration<br/>\nEstimated Angular Rate<br/>\nEstimated Gravity Vector<br/>\n<br/>\nChanging this setting will force all low-pass filters, the complementary filter, and the estimation filter to reset.",
        /* .parameters  = */ parameters,
    };
};

template<>
struct MetadataFor<commands_3dm::Sensor2VehicleTransformDcm>
{
    using type = commands_3dm::Sensor2VehicleTransformDcm;

    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name        = */ "dcm",
            /* .docs        = */ "3 x 3 direction cosine matrix, stored in row-major order",
            /* .type        = */ {Type::STRUCT, &MetadataFor<commands_3dm::Matrix3f>::value},
            /* .accessor    = */ utils::access<type, commands_3dm::Matrix3f, &type::dcm>,
            /* .functions   = */ {true, false, false, false, false},
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
    };

    static constexpr inline StructInfo value = {
        /* .name        = */ "Sensor2VehicleTransformDcm",
        /* .title       = */ "Sensor to Vehicle Frame Transformation Direction Cosine Matrix",
        /* .docs        = */ "Set the sensor to vehicle frame transformation using a using a 3 x 3 direction cosine matrix EQSTART M_{ned}^{veh} EQEND, stored in row-major order in a 9-element array.\n\nThese angles define the transformation of vectors from the sensor body frame to the fixed vehicle frame, according to:<br/>\nEQSTART v^{veh} = M_{sen}^{veh} v^{sen} EQEND<br/>\n\nWhere:<br/>\n\nEQSTART v^{sen} EQEND is a 3-element vector expressed in the sensor body frame. <br/>\nEQSTART v^{veh} EQEND is the same 3-element vector expressed in the vehicle frame.  <br/>\n<br/>\nThe matrix elements are stored is row-major order: EQSTART M_{sen}^{veh} = \begin{bmatrix} M_{11}, M_{12}, M_{13}, M_{21}, M_{22}, M_{23}, M_{31}, M_{32}, M_{33} \end{bmatrix} EQEND\nThe transformation may be stored in the device as a matrix or a quaternion. When EQSTART M_{sen}^{veh} EQEND is read back from the device, it may not\nbe exactly equal to array used to set the transformation, but it is functionally equivalent.<br/>\n<br/><br/>\nThis transformation affects the following output quantities:<br/><br/>\nIMU:<br/>\nScaled Acceleration<br/>\nScaled Gyro<br/>\nScaled Magnetometer<br/>\nDelta Theta<br/>\nDelta Velocity<br/>\n<br/><br/>\nEstimation Filter:<br/>\nEstimated Orientation, Quaternion<br/>\nEstimated Orientation, Matrix<br/>\nEstimated Orientation, Euler Angles<br/>\nEstimated Linear Acceleration<br/>\nEstimated Angular Rate<br/>\nEstimated Gravity Vector<br/>\n<br/>\nChanging this setting will force all low-pass filters, the complementary filter, and the estimation filter to reset.",
        /* .parameters  = */ parameters,
    };
};

template<>
struct MetadataFor<commands_3dm::ComplementaryFilter>
{
    using type = commands_3dm::ComplementaryFilter;

    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name        = */ "pitch_roll_enable",
            /* .docs        = */ "Enable Pitch/Roll corrections",
            /* .type        = */ {Type::BOOL, nullptr},
            /* .accessor    = */ utils::access<type, bool, &type::pitch_roll_enable>,
            /* .functions   = */ {true, false, false, false, false},
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
        {
            /* .name        = */ "heading_enable",
            /* .docs        = */ "Enable Heading corrections (only available on devices with magnetometer)",
            /* .type        = */ {Type::BOOL, nullptr},
            /* .accessor    = */ utils::access<type, bool, &type::heading_enable>,
            /* .functions   = */ {true, false, false, false, false},
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
        {
            /* .name        = */ "pitch_roll_time_constant",
            /* .docs        = */ "Time constant associated with the pitch/roll corrections [s]",
            /* .type        = */ {Type::FLOAT, nullptr},
            /* .accessor    = */ utils::access<type, float, &type::pitch_roll_time_constant>,
            /* .functions   = */ {true, false, false, false, false},
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
        {
            /* .name        = */ "heading_time_constant",
            /* .docs        = */ "Time constant associated with the heading corrections [s]",
            /* .type        = */ {Type::FLOAT, nullptr},
            /* .accessor    = */ utils::access<type, float, &type::heading_time_constant>,
            /* .functions   = */ {true, false, false, false, false},
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
    };

    static constexpr inline StructInfo value = {
        /* .name        = */ "ComplementaryFilter",
        /* .title       = */ "Complementary filter settings",
        /* .docs        = */ "Configure the settings for the complementary filter which produces the following (0x80) descriptor set values: attitude matrix (0x80,09), quaternion (0x80,0A), and  Euler angle (0x80,0C) outputs.\n\nThe filter can be configured to correct for pitch and roll using the accelerometer (with the assumption that linear acceleration is minimal),\nand to correct for heading using the magnetometer (with the assumption that the local magnetic field is dominated by the Earth's own magnetic field).\nPitch/roll and heading corrections each have their own configurable time constants, with a valid range of 1-1000 seconds. The default time constant is 10 seconds.",
        /* .parameters  = */ parameters,
    };
};

template<>
struct MetadataFor<commands_3dm::SensorRangeType>
{
    using type = commands_3dm::SensorRangeType;

    static constexpr inline EnumInfo::Entry entries[] = {
        { 0, "ALL", "Only allowed for SAVE, LOAD, and DEFAULT function selectors." },
        { 1, "ACCEL", "Accelerometer. Range is specified in g." },
        { 2, "GYRO", "Gyroscope. Range is specified in degrees/s." },
        { 3, "MAG", "Magnetometer. Range is specified in Gauss." },
        { 4, "PRESS", "Pressure sensor. Range is specified in hPa." },
    };

    static constexpr inline EnumInfo value = {
        /* .name    = */ "SensorRangeType",
        /* .docs    = */ "",
        /* .type    = */ Type::U8,
        /* .entries = */ entries,
    };

};

template<>
struct MetadataFor<commands_3dm::SensorRange>
{
    using type = commands_3dm::SensorRange;

    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name        = */ "sensor",
            /* .docs        = */ "Which type of sensor will get the new range value.",
            /* .type        = */ {Type::ENUM, &MetadataFor<commands_3dm::SensorRangeType>::value},
            /* .accessor    = */ utils::access<type, commands_3dm::SensorRangeType, &type::sensor>,
            /* .functions   = */ {true, true, true, true, true,  true},
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
        {
            /* .name        = */ "setting",
            /* .docs        = */ "Use the 3DM Get Calibrated Sensor Ranges (0x0C,0x53) command to determine this value.",
            /* .type        = */ {Type::U8, nullptr},
            /* .accessor    = */ utils::access<type, uint8_t, &type::setting>,
            /* .functions   = */ {true, false, false, false, false},
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
    };

    static constexpr inline StructInfo value = {
        /* .name        = */ "SensorRange",
        /* .title       = */ "Sensor Range",
        /* .docs        = */ "Changes the IMU sensor gain.\n\nThis allows you to optimize the range to get the best accuracy and performance\nwhile minimizing over-range events.\n\nUse the 3DM Get Calibrated Sensor Ranges (0x0C,0x53) command to determine\nthe appropriate setting value for your application. Using values other than\nthose specified may result in a NACK or inaccurate measurement data.",
        /* .parameters  = */ parameters,
    };
};

template<>
struct MetadataFor<commands_3dm::CalibratedSensorRanges::Entry>
{
    using type = commands_3dm::CalibratedSensorRanges::Entry;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name        = */ "setting",
            /* .docs        = */ "The value used in the 3DM Sensor Range command and response.",
            /* .type        = */ {Type::U8, nullptr},
            /* .accessor    = */ utils::access<type, uint8_t, &type::setting>,
            /* .functions   = */ NO_FUNCTIONS,
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
        {
            /* .name        = */ "range",
            /* .docs        = */ "The actual range value. Units depend on the sensor type.",
            /* .type        = */ {Type::FLOAT, nullptr},
            /* .accessor    = */ utils::access<type, float, &type::range>,
            /* .functions   = */ NO_FUNCTIONS,
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
    };

    static constexpr inline StructInfo value = {
        /* .name        = */ "Entry",
        /* .title       = */ "Entry",
        /* .docs        = */ "",
        /* .parameters  = */ parameters,
    };
};

template<>
struct MetadataFor<commands_3dm::CalibratedSensorRanges>
{
    using type = commands_3dm::CalibratedSensorRanges;

    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name        = */ "sensor",
            /* .docs        = */ "The sensor to query. Cannot be ALL.",
            /* .type        = */ {Type::ENUM, &MetadataFor<commands_3dm::SensorRangeType>::value},
            /* .accessor    = */ utils::access<type, commands_3dm::SensorRangeType, &type::sensor>,
            /* .functions   = */ {true, false, false, false, false,  true},
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
    };

    static constexpr inline StructInfo value = {
        /* .name        = */ "CalibratedSensorRanges",
        /* .title       = */ "Get Calibrated Sensor Ranges",
        /* .docs        = */ "Returns the supported sensor ranges which may be used with the 3DM Sensor Range (0x0C,0x52) command.\n\nThe response includes an array of (u8, float) pairs which map each allowed setting\nto the corresponding maximum range in physical units. See SensorRangeType for units.",
        /* .parameters  = */ parameters,
    };
};

template<>
struct MetadataFor<commands_3dm::LowpassFilter>
{
    using type = commands_3dm::LowpassFilter;

    static constexpr inline ParameterInfo parameters[] = {
        FUNCTION_SELECTOR_PARAM,
        {
            /* .name        = */ "desc_set",
            /* .docs        = */ "Descriptor set of the quantity to be filtered.",
            /* .type        = */ {Type::U8, nullptr},
            /* .accessor    = */ utils::access<type, uint8_t, &type::desc_set>,
            /* .functions   = */ {true, true, true, true, true,  true},
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
        {
            /* .name        = */ "field_desc",
            /* .docs        = */ "Field descriptor of the quantity to be filtered.",
            /* .type        = */ {Type::U8, nullptr},
            /* .accessor    = */ utils::access<type, uint8_t, &type::field_desc>,
            /* .functions   = */ {true, true, true, true, true,  true},
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
        {
            /* .name        = */ "enable",
            /* .docs        = */ "The filter will be enabled if this is true.",
            /* .type        = */ {Type::BOOL, nullptr},
            /* .accessor    = */ utils::access<type, bool, &type::enable>,
            /* .functions   = */ {true, false, false, false, false},
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
        {
            /* .name        = */ "manual",
            /* .docs        = */ "If false, the frequency parameter is ignored and the filter\nwill track to half of the configured message format frequency.",
            /* .type        = */ {Type::BOOL, nullptr},
            /* .accessor    = */ utils::access<type, bool, &type::manual>,
            /* .functions   = */ {true, false, false, false, false},
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
        {
            /* .name        = */ "frequency",
            /* .docs        = */ "Cutoff frequency in Hz. This will return the actual frequency\nwhen read out in automatic mode.",
            /* .type        = */ {Type::FLOAT, nullptr},
            /* .accessor    = */ utils::access<type, float, &type::frequency>,
            /* .functions   = */ {true, false, false, false, false},
            /* .count       = */ 1,
            /* .counter_idx = */ {},
            /* .union_index = */ {},
            /* .union_value = */ 0,
        },
    };

    static constexpr inline StructInfo value = {
        /* .name        = */ "LowpassFilter",
        /* .title       = */ "Low-pass anti-aliasing filter",
        /* .docs        = */ "This command controls the low-pass anti-aliasing filter supported data quantities.\n\nSee the device user manual for data quantities which support the anti-aliasing filter.\n\nIf set to automatic mode, the frequency will track half of the transmission rate\nof the target descriptor according to the configured message format (0x0C,0x0F).\nFor example, if scaled accel (0x80,0x04) is set to stream at 100 Hz, the filter would\nbe set to 50 Hz. Changing the message format to 200 Hz would automatically adjust the\nfilter to 100 Hz.\n\nFor WRITE, SAVE, LOAD, and DEFAULT function selectors, the descriptor set and/or field descriptor\nmay be 0x00 to set, save, load, or reset the setting for all supported descriptors. The\nfield descriptor must be 0x00 if the descriptor set is 0x00.\n",
        /* .parameters  = */ parameters,
    };
};


} // namespace mip::metadata

