#pragma once

#include <mip/metadata/definitions/common.hpp>

#include <mip/definitions/data_system.hpp>


#include <mip/metadata/mip_metadata.hpp>

namespace mip::metadata
{


template<>
struct MetadataFor<data_system::BuiltInTest>
{
    using type = data_system::BuiltInTest;

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
        /* .name        = */ "data_system::BuiltInTest",
        /* .title       = */ "built_in_test",
        /* .docs        = */ "Contains the continuous built-in-test (BIT) results.\n\nDue to the large size of this field, it is recommended to stream it at\na low rate or poll it on demand.\n\nThese bits are 'sticky' until the next output message. If a fault occurs\nin between scheduled messages or while the device is idle, the next\npacket with this field will have the corresponding flags set. The flag\nis then cleared unless the fault persists.\n\nUnlike the commanded BIT, some bits may be 1 in certain\nnon-fault situations, so simply checking if the result is all 0s is\nnot very useful. For example, on devices with a built-in GNSS receiver,\na 'solution fault' bit may be set before the receiver has obtained\na position fix. Consult the device manual to determine which bits are\nof interest for your application.\n\nAll unspecified bits are reserved for future use and must be ignored.\n",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_system::TimeSyncStatus>
{
    using type = data_system::TimeSyncStatus;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "time_sync",
            /* .docs          = */ "True if sync with the PPS signal is currently valid. False if PPS\nfeature is disabled or a PPS signal is not detected.",
            /* .type          = */ {Type::BOOL, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, bool, &type::time_sync>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "last_pps_rcvd",
            /* .docs          = */ "Elapsed time in seconds since last PPS was received, with a maximum\nvalue of 255.",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::last_pps_rcvd>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_system::TimeSyncStatus",
        /* .title       = */ "time_sync_status",
        /* .docs        = */ "Indicates whether a sync has been achieved using the PPS signal.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_system::GpioState>
{
    using type = data_system::GpioState;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "states",
            /* .docs          = */ "Bitfield containing the states for each GPIO pin.<br/>\nBit 0 (0x01): pin 1<br/>\nBit 1 (0x02): pin 2<br/>\nBit 2 (0x04): pin 3<br/>\nBit 3 (0x08): pin 4<br/>\nBits for pins that don't exist will read as 0.",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::states>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_system::GpioState",
        /* .title       = */ "gpio_state",
        /* .docs        = */ "Indicates the state of all of the user GPIO pins.\n\nThis message can be used to correlate external signals\nwith the device time or other data quantities. It should\ngenerally be used with slow GPIO signals as brief pulses\nshorter than the scheduled data rate will be missed.\n\nTo synchronize with faster signals and pulses, or for more accurate timestamping,\nutilize the event system and set the GPIO feature to TIMESTAMP in the 3DM GPIO\nConfiguration command (0x0C,0x41).\n\nThese GPIO states are sampled within one base period\nof the system data descriptor set.\n\nTo obtain valid readings, the desired pin(s) must be configured to the GPIO feature\n(either input or output behavior) using the 3DM GPIO Configuration command\n(0x0C,0x41). Other gpio features may work on some devices but this is not guaranteed.\nConsult the factory before producing a design relying on reading pins configured\nto other feature types.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};

template<>
struct MetadataFor<data_system::GpioAnalogValue>
{
    using type = data_system::GpioAnalogValue;

    static constexpr inline ParameterInfo parameters[] = {
        {
            /* .name          = */ "gpio_id",
            /* .docs          = */ "GPIO pin number starting with 1.",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, uint8_t, &type::gpio_id>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "value",
            /* .docs          = */ "Value of the GPIO line in scaled volts.",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ nullptr, //utils::access<type, float, &type::value>,
            /* .attributes    = */ {true, false, false, false, false},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_system::GpioAnalogValue",
        /* .title       = */ "gpio_analog_value",
        /* .docs        = */ "Indicates the analog value of the given user GPIO.\nThe pin must be configured for analog input.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .response    = */ nullptr,
    };
};


static constexpr inline const FieldInfo* DATA_SYSTEM_FIELDS[] = {
    &MetadataFor<data_system::BuiltInTest>::value,
    &MetadataFor<data_system::TimeSyncStatus>::value,
    &MetadataFor<data_system::GpioState>::value,
    &MetadataFor<data_system::GpioAnalogValue>::value,
};

static constexpr DescriptorSetInfo DATA_SYSTEM = {
    /*.descriptor =*/ mip::data_system::DESCRIPTOR_SET,
    /*.name       =*/ "System Data",
    /*.fields     =*/ DATA_SYSTEM_FIELDS,
};

} // namespace mip::metadata

