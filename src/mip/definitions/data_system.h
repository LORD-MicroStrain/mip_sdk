#pragma once

#include "descriptors.h"
#include "../mip_result.h"

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
namespace mip {
namespace C {
extern "C" {

#endif // __cplusplus
struct mip_interface;
struct mip_serializer;
struct mip_field;

////////////////////////////////////////////////////////////////////////////////
///@addtogroup MipData
///@{
///@defgroup system_data_c  SYSTEMData
///
///@{

////////////////////////////////////////////////////////////////////////////////
// Descriptors
////////////////////////////////////////////////////////////////////////////////

enum 
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
///@defgroup c_built_in_test  None
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

struct mip_system_built_in_test_data
{
    uint8_t result[16];
    
};
void insert_mip_system_built_in_test_data(struct mip_serializer* serializer, const struct mip_system_built_in_test_data* self);
void extract_mip_system_built_in_test_data(struct mip_serializer* serializer, struct mip_system_built_in_test_data* self);
bool extract_mip_system_built_in_test_data_from_field(const struct mip_field* field, void* ptr);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_time_sync_status  None
/// Indicates whether a sync has been achieved using the PPS signal.
///
///@{

struct mip_system_time_sync_status_data
{
    bool time_sync;
    uint8_t last_pps_rcvd;
    
};
void insert_mip_system_time_sync_status_data(struct mip_serializer* serializer, const struct mip_system_time_sync_status_data* self);
void extract_mip_system_time_sync_status_data(struct mip_serializer* serializer, struct mip_system_time_sync_status_data* self);
bool extract_mip_system_time_sync_status_data_from_field(const struct mip_field* field, void* ptr);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_gpio_state  None
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

struct mip_system_gpio_state_data
{
    uint8_t states;
    
};
void insert_mip_system_gpio_state_data(struct mip_serializer* serializer, const struct mip_system_gpio_state_data* self);
void extract_mip_system_gpio_state_data(struct mip_serializer* serializer, struct mip_system_gpio_state_data* self);
bool extract_mip_system_gpio_state_data_from_field(const struct mip_field* field, void* ptr);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_gpio_analog_value  None
/// Indicates the analog value of the given user GPIO.
/// The pin must be configured for analog input.
///
///@{

struct mip_system_gpio_analog_value_data
{
    uint8_t gpio_id;
    float value;
    
};
void insert_mip_system_gpio_analog_value_data(struct mip_serializer* serializer, const struct mip_system_gpio_analog_value_data* self);
void extract_mip_system_gpio_analog_value_data(struct mip_serializer* serializer, struct mip_system_gpio_analog_value_data* self);
bool extract_mip_system_gpio_analog_value_data_from_field(const struct mip_field* field, void* ptr);

///@}
///

///@}
///@}
///
////////////////////////////////////////////////////////////////////////////////
#ifdef __cplusplus
} // namespace C
} // namespace mip
} // extern "C"
#endif // __cplusplus

