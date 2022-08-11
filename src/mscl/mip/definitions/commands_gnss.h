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

////////////////////////////////////////////////////////////////////////////////
///@addtogroup MipCommands
///@{
///@defgroup gnss_commands_c  GNSSCommands
///
///@{

////////////////////////////////////////////////////////////////////////////////
// Descriptors
////////////////////////////////////////////////////////////////////////////////

enum 
{
    MIP_GNSS_CMD_DESC_SET                        = 0x0E,
    
    MIP_CMD_DESC_GNSS_LIST_RECEIVERS             = 0x01,
    MIP_CMD_DESC_GNSS_SIGNAL_CONFIGURATION       = 0x02,
    MIP_CMD_DESC_GNSS_RTK_DONGLE_CONFIGURATION   = 0x10,
    MIP_CMD_DESC_GNSS_RECEIVER_SAFE_MODE         = 0x60,
    
    MIP_REPLY_DESC_GNSS_LIST_RECEIVERS           = 0x81,
    MIP_REPLY_DESC_GNSS_SIGNAL_CONFIGURATION     = 0x82,
    MIP_REPLY_DESC_GNSS_RTK_DONGLE_CONFIGURATION = 0x90,
};

////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////

enum { GNSS_GPS_ENABLE_L1CA = 0x0001 };
enum { GNSS_GPS_ENABLE_L2C = 0x0002 };
enum { GNSS_GLONASS_ENABLE_L1OF = 0x0001 };
enum { GNSS_GLONASS_ENABLE_L2OF = 0x0002 };
enum { GNSS_GALILEO_ENABLE_E1 = 0x0001 };
enum { GNSS_GALILEO_ENABLE_E5B = 0x0002 };
enum { GNSS_BEIDOU_ENABLE_B1 = 0x0001 };
enum { GNSS_BEIDOU_ENABLE_B2 = 0x0002 };

////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
///@defgroup c_receiver_info  None
/// Return information about the GNSS receivers in the device.
/// 
///
///@{

struct mip_gnss_receiver_info_command_info
{
    uint8_t receiver_id;
    uint8_t mip_data_descriptor_set;
    char description[32];
    
};
void insert_mip_gnss_receiver_info_command_info(struct mip_serializer* serializer, const struct mip_gnss_receiver_info_command_info* self);
void extract_mip_gnss_receiver_info_command_info(struct mip_serializer* serializer, struct mip_gnss_receiver_info_command_info* self);

struct mip_gnss_receiver_info_response
{
    uint8_t num_receivers;
    struct mip_gnss_receiver_info_command_info* receiver_info;
    
};
void insert_mip_gnss_receiver_info_response(struct mip_serializer* serializer, const struct mip_gnss_receiver_info_response* self);
void extract_mip_gnss_receiver_info_response(struct mip_serializer* serializer, struct mip_gnss_receiver_info_response* self);

mip_cmd_result mip_gnss_receiver_info(struct mip_interface* device, uint8_t* num_receivers, struct mip_gnss_receiver_info_command_info* receiver_info);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_signal_configuration  None
/// Configure the GNSS signals used by the device.
/// 
///
///@{

struct mip_gnss_signal_configuration_command
{
    enum mip_function_selector function;
    uint8_t gps_enable;
    uint8_t glonass_enable;
    uint8_t galileo_enable;
    uint8_t beidou_enable;
    uint8_t reserved[4];
    
};
void insert_mip_gnss_signal_configuration_command(struct mip_serializer* serializer, const struct mip_gnss_signal_configuration_command* self);
void extract_mip_gnss_signal_configuration_command(struct mip_serializer* serializer, struct mip_gnss_signal_configuration_command* self);

struct mip_gnss_signal_configuration_response
{
    uint8_t gps_enable;
    uint8_t glonass_enable;
    uint8_t galileo_enable;
    uint8_t beidou_enable;
    uint8_t reserved[4];
    
};
void insert_mip_gnss_signal_configuration_response(struct mip_serializer* serializer, const struct mip_gnss_signal_configuration_response* self);
void extract_mip_gnss_signal_configuration_response(struct mip_serializer* serializer, struct mip_gnss_signal_configuration_response* self);

mip_cmd_result mip_gnss_write_signal_configuration(struct mip_interface* device, uint8_t gps_enable, uint8_t glonass_enable, uint8_t galileo_enable, uint8_t beidou_enable, const uint8_t* reserved);
mip_cmd_result mip_gnss_read_signal_configuration(struct mip_interface* device, uint8_t* gps_enable, uint8_t* glonass_enable, uint8_t* galileo_enable, uint8_t* beidou_enable, uint8_t* reserved);
mip_cmd_result mip_gnss_save_signal_configuration(struct mip_interface* device);
mip_cmd_result mip_gnss_load_signal_configuration(struct mip_interface* device);
mip_cmd_result mip_gnss_default_signal_configuration(struct mip_interface* device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_rtk_dongle_configuration  None
/// Configure the communications with the RTK Dongle connected to the device.
/// 
///
///@{

struct mip_gnss_rtk_dongle_configuration_command
{
    enum mip_function_selector function;
    uint8_t enable;
    uint8_t reserved[3];
    
};
void insert_mip_gnss_rtk_dongle_configuration_command(struct mip_serializer* serializer, const struct mip_gnss_rtk_dongle_configuration_command* self);
void extract_mip_gnss_rtk_dongle_configuration_command(struct mip_serializer* serializer, struct mip_gnss_rtk_dongle_configuration_command* self);

struct mip_gnss_rtk_dongle_configuration_response
{
    uint8_t enable;
    uint8_t reserved[3];
    
};
void insert_mip_gnss_rtk_dongle_configuration_response(struct mip_serializer* serializer, const struct mip_gnss_rtk_dongle_configuration_response* self);
void extract_mip_gnss_rtk_dongle_configuration_response(struct mip_serializer* serializer, struct mip_gnss_rtk_dongle_configuration_response* self);

mip_cmd_result mip_gnss_write_rtk_dongle_configuration(struct mip_interface* device, uint8_t enable, const uint8_t* reserved);
mip_cmd_result mip_gnss_read_rtk_dongle_configuration(struct mip_interface* device, uint8_t* enable, uint8_t* reserved);
mip_cmd_result mip_gnss_save_rtk_dongle_configuration(struct mip_interface* device);
mip_cmd_result mip_gnss_load_rtk_dongle_configuration(struct mip_interface* device);
mip_cmd_result mip_gnss_default_rtk_dongle_configuration(struct mip_interface* device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_receiver_safe_mode  GNSS Receiver Safe Mode
/// Enable/disable safe mode for the provided receiver ID.
/// Note: Receivers in safe mode will not output valid GNSS data.
/// 
///
///@{

struct mip_gnss_receiver_safe_mode_command
{
    uint8_t receiver_id;
    uint8_t enable;
    
};
void insert_mip_gnss_receiver_safe_mode_command(struct mip_serializer* serializer, const struct mip_gnss_receiver_safe_mode_command* self);
void extract_mip_gnss_receiver_safe_mode_command(struct mip_serializer* serializer, struct mip_gnss_receiver_safe_mode_command* self);

mip_cmd_result mip_gnss_receiver_safe_mode(struct mip_interface* device, uint8_t receiver_id, uint8_t enable);
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

