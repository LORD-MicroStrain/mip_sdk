#pragma once

#include "common.hpp"

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
            /* .docs          = */ "Device-specific bitfield (128 bits). See device user manual.nBits are least-significant-byte first. For example, bit 0 isnlocated at bit 0 of result[0], bit 1 is located at bit 1 of result[0],nbit 8 is located at bit 0 of result[1], and bit 127 is located at bitn7 of result[15].",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ utils::access<type, uint8_t, &type::result>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 16,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_system::BuiltInTest",
        /* .title       = */ "None",
        /* .docs        = */ "Contains the continuous built-in-test (BIT) results.nnDue to the large size of this field, it is recommended to stream it atna low rate or poll it on demand.nnThese bits are 'sticky' until the next output message. If a fault occursnin between scheduled messages or while the device is idle, the nextnpacket with this field will have the corresponding flags set. The flagnis then cleared unless the fault persists.nnUnlike the commanded BIT, some bits may be 1 in certainnnon-fault situations, so simply checking if the result is all 0s isnnot very useful. For example, on devices with a built-in GNSS receiver,na 'solution fault' bit may be set before the receiver has obtainedna position fix. Consult the device manual to determine which bits arenof interest for your application.nnAll unspecified bits are reserved for future use and must be ignored.n",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .proprietary = */ false,
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
            /* .docs          = */ "True if sync with the PPS signal is currently valid. False if PPSnfeature is disabled or a PPS signal is not detected.",
            /* .type          = */ {Type::BOOL, nullptr},
            /* .accessor      = */ utils::access<type, bool, &type::time_sync>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "last_pps_rcvd",
            /* .docs          = */ "Elapsed time in seconds since last PPS was received, with a maximumnvalue of 255.",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ utils::access<type, uint8_t, &type::last_pps_rcvd>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_system::TimeSyncStatus",
        /* .title       = */ "None",
        /* .docs        = */ "Indicates whether a sync has been achieved using the PPS signal.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .proprietary = */ false,
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
            /* .docs          = */ "Bitfield containing the states for each GPIO pin.<br/>nBit 0 (0x01): pin 1<br/>nBit 1 (0x02): pin 2<br/>nBit 2 (0x04): pin 3<br/>nBit 3 (0x08): pin 4<br/>nBits for pins that don't exist will read as 0.",
            /* .type          = */ {Type::U8, nullptr},
            /* .accessor      = */ utils::access<type, uint8_t, &type::states>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_system::GpioState",
        /* .title       = */ "None",
        /* .docs        = */ "Indicates the state of all of the user GPIO pins.nnThis message can be used to correlate external signalsnwith the device time or other data quantities. It shouldngenerally be used with slow GPIO signals as brief pulsesnshorter than the scheduled data rate will be missed.nnTo synchronize with faster signals and pulses, or for more accurate timestamping,nutilize the event system and set the GPIO feature to TIMESTAMP in the 3DM GPIOnConfiguration command (0x0C,0x41).nnThese GPIO states are sampled within one base periodnof the system data descriptor set.nnTo obtain valid readings, the desired pin(s) must be configured to the GPIO featuren(either input or output behavior) using the 3DM GPIO Configuration commandn(0x0C,0x41). Other gpio features may work on some devices but this is not guaranteed.nConsult the factory before producing a design relying on reading pins configurednto other feature types.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .proprietary = */ false,
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
            /* .accessor      = */ utils::access<type, uint8_t, &type::gpio_id>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
        {
            /* .name          = */ "value",
            /* .docs          = */ "Value of the GPIO line in scaled volts.",
            /* .type          = */ {Type::FLOAT, nullptr},
            /* .accessor      = */ utils::access<type, float, &type::value>,
            /* .functions     = */ {true, false, false, false, false,  true},
            /* .count         = */ 1,
            /* .condition     = */ {},
        },
    };

    static constexpr inline FieldInfo value = {
        /* .name        = */ "data_system::GpioAnalogValue",
        /* .title       = */ "None",
        /* .docs        = */ "Indicates the analog value of the given user GPIO.nThe pin must be configured for analog input.",
        /* .parameters  = */ parameters,
        /* .descriptor  = */ type::DESCRIPTOR,
        /* .functions   = */ NO_FUNCTIONS,
        /* .proprietary = */ false,
        /* .response    = */ nullptr,
    };
};


static constexpr inline std::initializer_list<const FieldInfo*> ALL_DATA_SYSTEM = {
    &MetadataFor<data_system::BuiltInTest>::value,
    &MetadataFor<data_system::TimeSyncStatus>::value,
    &MetadataFor<data_system::GpioState>::value,
    &MetadataFor<data_system::GpioAnalogValue>::value,
};


} // namespace mip::metadata

