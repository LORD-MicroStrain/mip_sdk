#pragma once

#include "descriptors.h"

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
namespace mscl {
extern "C" {
#endif // __cplusplus

////////////////////////////////////////////////////////////////////////////////
///@addtogroup MipData
///@{
///@defgroup SYSTEM_DATA  SYSTEM DATA
///
///@{

////////////////////////////////////////////////////////////////////////////////
// Descriptors
////////////////////////////////////////////////////////////////////////////////

enum MipSystemData_Descriptors
{
    MIP_SYSTEM_DATA_DESC_SET               = 0xA0,
    
    MIP_DATA_DESC_SYSTEM_BUILT_IN_TEST     = 0x01,
    MIP_DATA_DESC_SYSTEM_TIME_SYNC_STATUS  = 0x02,
    MIP_DATA_DESC_SYSTEM_GPIO_STATE        = 0x03,
    MIP_DATA_DESC_SYSTEM_GPIO_ANALOG_VALUE = 0x04,
    
};

////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_system_built_in_test  System Built In Test
/// Contains the continuous built-in-test (BIT) results.
/// 
/// Due to the large size of this field, it is recommended to stream it at
/// a low rate or poll it on demand.
/// 
/// These bits are "sticky" until the next output message. If a fault occurs
/// in between scheduled messages or while the device is idle, the next
/// packet with this field will have the corresponding flags set. The flag
/// is then cleared unless the fault persists.
/// 
/// Unlike the commanded BIT, some bits may be 1 in certain
/// non-fault situations, so simply checking if the result is all 0s is
/// not very useful. For example, on devices with a built-in GNSS receiver,
/// a "solution fault" bit may be set before the receiver has obtained
/// a position fix. Consult the device manual to determine which bits are
/// of interest for your application.
/// 
/// All unspecified bits are reserved for future use and must be ignored.
/// 
///
///@{

struct MipData_System_BuiltInTest
{
    uint8_t                                           result[16];
};
size_t insert_MipData_System_BuiltInTest(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_System_BuiltInTest* self);
size_t extract_MipData_System_BuiltInTest(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_System_BuiltInTest* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_system_time_sync_status  System Time Sync Status
/// Indicates whether a sync has been achieved using the PPS signal.
///
///@{

struct MipData_System_TimeSyncStatus
{
    bool                                              time_sync;
    uint8_t                                           last_pps_rcvd;
};
size_t insert_MipData_System_TimeSyncStatus(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_System_TimeSyncStatus* self);
size_t extract_MipData_System_TimeSyncStatus(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_System_TimeSyncStatus* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_system_gpio_state  System Gpio State
/// Indicates the state of all of the user GPIO pins.
/// 
/// This message can be used to correlate external signals
/// with the device time or other data quantities. It should
/// generally be used with slow GPIO signals as brief pulses
/// shorter than the scheduled data rate will be missed.
/// 
/// To synchronize with faster signals and pulses, or for more accurate timestamping,
/// utilize the event system and set the GPIO feature to TIMESTAMP in the 3DM GPIO
/// Configuration command (0x0C,0x41).
/// 
/// These GPIO states are sampled within one base period
/// of the system data descriptor set.
/// 
/// To obtain valid readings, the desired pin(s) must be configured to the GPIO feature
/// (either input or output behavior) using the 3DM GPIO Configuration command
/// (0x0C,0x41). Other gpio features may work on some devices but this is not guaranteed.
/// Consult the factory before producing a design relying on reading pins configured
/// to other feature types.
///
///@{

struct MipData_System_GpioState
{
    uint8_t                                           states;
};
size_t insert_MipData_System_GpioState(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_System_GpioState* self);
size_t extract_MipData_System_GpioState(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_System_GpioState* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_system_gpio_analog_value  System Gpio Analog Value
/// Indicates the analog value of the given user GPIO.
/// The pin must be configured for analog input.
///
///@{

struct MipData_System_GpioAnalogValue
{
    uint8_t                                           gpio_id;
    float                                             value;
};
size_t insert_MipData_System_GpioAnalogValue(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_System_GpioAnalogValue* self);
size_t extract_MipData_System_GpioAnalogValue(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_System_GpioAnalogValue* self);

///@}
///

///@}
///@}
///
////////////////////////////////////////////////////////////////////////////////

#ifdef __cplusplus
} // extern "C"
} // namespace mscl
#endif // __cplusplus
