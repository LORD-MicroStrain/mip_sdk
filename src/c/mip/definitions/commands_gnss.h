#pragma once

#include <mip/definitions/common.h>
#include <mip/mip_descriptors.h>
#include <mip/mip_result.h>
#include <mip/mip_interface.h>

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
namespace mip {
namespace C {
extern "C" {

#endif // __cplusplus

////////////////////////////////////////////////////////////////////////////////
///@addtogroup MipCommands_c
///@{
///@defgroup gnss_commands_c  Gnss Commands
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
    MIP_CMD_DESC_GNSS_SPARTN_CONFIGURATION       = 0x20,
    
    MIP_REPLY_DESC_GNSS_LIST_RECEIVERS           = 0x81,
    MIP_REPLY_DESC_GNSS_SIGNAL_CONFIGURATION     = 0x82,
    MIP_REPLY_DESC_GNSS_RTK_DONGLE_CONFIGURATION = 0x90,
    MIP_REPLY_DESC_GNSS_SPARTN_CONFIGURATION     = 0xA0,
};

////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////

enum { MIP_GNSS_GPS_ENABLE_L1CA = 0x0001 };
enum { MIP_GNSS_GPS_ENABLE_L2C = 0x0002 };
enum { MIP_GNSS_GPS_ENABLE_L5 = 0x0004 };
enum { MIP_GNSS_GLONASS_ENABLE_L1OF = 0x0001 };
enum { MIP_GNSS_GLONASS_ENABLE_L2OF = 0x0002 };
enum { MIP_GNSS_GALILEO_ENABLE_E1 = 0x0001 };
enum { MIP_GNSS_GALILEO_ENABLE_E5B = 0x0002 };
enum { MIP_GNSS_GALILEO_ENABLE_E5A = 0x0004 };
enum { MIP_GNSS_BEIDOU_ENABLE_B1 = 0x0001 };
enum { MIP_GNSS_BEIDOU_ENABLE_B2 = 0x0002 };
enum { MIP_GNSS_BEIDOU_ENABLE_B2A = 0x0004 };

////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
///@defgroup gnss_receiver_info_c  (0x0E,0x01) Receiver Info
/// Return information about the GNSS receivers in the device.
/// 
///
///@{

struct mip_gnss_receiver_info_command_info
{
    uint8_t receiver_id; ///< Receiver id: e.g. 1, 2, etc.
    uint8_t mip_data_descriptor_set; ///< MIP descriptor set associated with this receiver
    char description[32]; ///< Ascii description of receiver. Contains the following info (comma-delimited):<br/> Module name/model<br/> Firmware version info
};
typedef struct mip_gnss_receiver_info_command_info mip_gnss_receiver_info_command_info;

void insert_mip_gnss_receiver_info_command_info(microstrain_serializer* serializer, const mip_gnss_receiver_info_command_info* self);
void extract_mip_gnss_receiver_info_command_info(microstrain_serializer* serializer, mip_gnss_receiver_info_command_info* self);


typedef struct mip_gnss_receiver_info_command mip_gnss_receiver_info_command; ///< No parameters (empty struct not allowed in C)

struct mip_gnss_receiver_info_response
{
    uint8_t num_receivers; ///< Number of physical receivers in the device
    mip_gnss_receiver_info_command_info receiver_info[5];
};
typedef struct mip_gnss_receiver_info_response mip_gnss_receiver_info_response;

void insert_mip_gnss_receiver_info_response(microstrain_serializer* serializer, const mip_gnss_receiver_info_response* self);
void extract_mip_gnss_receiver_info_response(microstrain_serializer* serializer, mip_gnss_receiver_info_response* self);

mip_cmd_result mip_gnss_receiver_info(mip_interface* device, uint8_t* num_receivers_out, uint8_t num_receivers_out_max, mip_gnss_receiver_info_command_info* receiver_info_out);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup gnss_signal_configuration_c  (0x0E,0x02) Signal Configuration
/// Configure the GNSS signals used by the device.
/// 
///
///@{

struct mip_gnss_signal_configuration_command
{
    mip_function_selector function;
    uint8_t gps_enable; ///< Bitfield 0: Enable L1CA, 1: Enable L2C, 2: Enable L5
    uint8_t glonass_enable; ///< Bitfield 0: Enable L1OF, 1: Enable L2OF
    uint8_t galileo_enable; ///< Bitfield 0: Enable E1,   1: Enable E5B, 2: Enable E5A
    uint8_t beidou_enable; ///< Bitfield 0: Enable B1,   1: Enable B2,  2: Enable B2A
    uint8_t reserved[4];
};
typedef struct mip_gnss_signal_configuration_command mip_gnss_signal_configuration_command;

void insert_mip_gnss_signal_configuration_command(microstrain_serializer* serializer, const mip_gnss_signal_configuration_command* self);
void extract_mip_gnss_signal_configuration_command(microstrain_serializer* serializer, mip_gnss_signal_configuration_command* self);

struct mip_gnss_signal_configuration_response
{
    uint8_t gps_enable; ///< Bitfield 0: Enable L1CA, 1: Enable L2C, 2: Enable L5
    uint8_t glonass_enable; ///< Bitfield 0: Enable L1OF, 1: Enable L2OF
    uint8_t galileo_enable; ///< Bitfield 0: Enable E1,   1: Enable E5B, 2: Enable E5A
    uint8_t beidou_enable; ///< Bitfield 0: Enable B1,   1: Enable B2,  2: Enable B2A
    uint8_t reserved[4];
};
typedef struct mip_gnss_signal_configuration_response mip_gnss_signal_configuration_response;

void insert_mip_gnss_signal_configuration_response(microstrain_serializer* serializer, const mip_gnss_signal_configuration_response* self);
void extract_mip_gnss_signal_configuration_response(microstrain_serializer* serializer, mip_gnss_signal_configuration_response* self);

mip_cmd_result mip_gnss_write_signal_configuration(mip_interface* device, uint8_t gps_enable, uint8_t glonass_enable, uint8_t galileo_enable, uint8_t beidou_enable, const uint8_t* reserved);
mip_cmd_result mip_gnss_read_signal_configuration(mip_interface* device, uint8_t* gps_enable_out, uint8_t* glonass_enable_out, uint8_t* galileo_enable_out, uint8_t* beidou_enable_out, uint8_t* reserved_out);
mip_cmd_result mip_gnss_save_signal_configuration(mip_interface* device);
mip_cmd_result mip_gnss_load_signal_configuration(mip_interface* device);
mip_cmd_result mip_gnss_default_signal_configuration(mip_interface* device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup gnss_spartn_configuration_c  (0x0E,0x20) Spartn Configuration
/// Configure the SPARTN corrections service parameters.
/// Notes:<br/>
/// - Enable and type settings will only update after a power cycle <br/>
/// - Type settings will only take effect after a power cycle <br/>
/// - Key information can be updated while running
///
///@{

struct mip_gnss_spartn_configuration_command
{
    mip_function_selector function;
    uint8_t enable; ///< Enable/Disable the SPARTN subsystem (0 = Disabled, 1 = Enabled)
    uint8_t type; ///< Connection type (0 - None, 1 = Network, 2 = L-Band)
    uint32_t current_key_tow; ///< The GPS time of week the current key is valid until
    uint16_t current_key_week; ///< The GPS week number the current key is valid until
    uint8_t current_key[32]; ///< 32 character string of ASCII hex values for the current key (e.g. "bc" for 0xBC)
    uint32_t next_key_tow; ///< The GPS time of week the next key is valid until
    uint16_t next_key_week; ///< The GPS week number the next key is valid until
    uint8_t next_key[32]; ///< 32 character string of ASCII hex valuesfor the next key (e.g. "bc" for 0xBC)
};
typedef struct mip_gnss_spartn_configuration_command mip_gnss_spartn_configuration_command;

void insert_mip_gnss_spartn_configuration_command(microstrain_serializer* serializer, const mip_gnss_spartn_configuration_command* self);
void extract_mip_gnss_spartn_configuration_command(microstrain_serializer* serializer, mip_gnss_spartn_configuration_command* self);

struct mip_gnss_spartn_configuration_response
{
    uint8_t enable; ///< Enable/Disable the SPARTN subsystem (0 = Disabled, 1 = Enabled)
    uint8_t type; ///< Connection type (0 - None, 1 = Network, 2 = L-Band)
    uint32_t current_key_tow; ///< The GPS time of week the current key is valid until
    uint16_t current_key_week; ///< The GPS week number the current key is valid until
    uint8_t current_key[32]; ///< 32 character string of ASCII hex values for the current key (e.g. "bc" for 0xBC)
    uint32_t next_key_tow; ///< The GPS time of week the next key is valid until
    uint16_t next_key_week; ///< The GPS week number the next key is valid until
    uint8_t next_key[32]; ///< 32 character string of ASCII hex valuesfor the next key (e.g. "bc" for 0xBC)
};
typedef struct mip_gnss_spartn_configuration_response mip_gnss_spartn_configuration_response;

void insert_mip_gnss_spartn_configuration_response(microstrain_serializer* serializer, const mip_gnss_spartn_configuration_response* self);
void extract_mip_gnss_spartn_configuration_response(microstrain_serializer* serializer, mip_gnss_spartn_configuration_response* self);

mip_cmd_result mip_gnss_write_spartn_configuration(mip_interface* device, uint8_t enable, uint8_t type, uint32_t current_key_tow, uint16_t current_key_week, const uint8_t* current_key, uint32_t next_key_tow, uint16_t next_key_week, const uint8_t* next_key);
mip_cmd_result mip_gnss_read_spartn_configuration(mip_interface* device, uint8_t* enable_out, uint8_t* type_out, uint32_t* current_key_tow_out, uint16_t* current_key_week_out, uint8_t* current_key_out, uint32_t* next_key_tow_out, uint16_t* next_key_week_out, uint8_t* next_key_out);
mip_cmd_result mip_gnss_save_spartn_configuration(mip_interface* device);
mip_cmd_result mip_gnss_load_spartn_configuration(mip_interface* device);
mip_cmd_result mip_gnss_default_spartn_configuration(mip_interface* device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup gnss_rtk_dongle_configuration_c  (0x0E,0x10) Rtk Dongle Configuration
/// Configure the communications with the RTK Dongle connected to the device.
/// 
///
///@{

struct mip_gnss_rtk_dongle_configuration_command
{
    mip_function_selector function;
    uint8_t enable; ///< 0 - Disabled, 1- Enabled
    uint8_t reserved[3];
};
typedef struct mip_gnss_rtk_dongle_configuration_command mip_gnss_rtk_dongle_configuration_command;

void insert_mip_gnss_rtk_dongle_configuration_command(microstrain_serializer* serializer, const mip_gnss_rtk_dongle_configuration_command* self);
void extract_mip_gnss_rtk_dongle_configuration_command(microstrain_serializer* serializer, mip_gnss_rtk_dongle_configuration_command* self);

struct mip_gnss_rtk_dongle_configuration_response
{
    uint8_t enable;
    uint8_t reserved[3];
};
typedef struct mip_gnss_rtk_dongle_configuration_response mip_gnss_rtk_dongle_configuration_response;

void insert_mip_gnss_rtk_dongle_configuration_response(microstrain_serializer* serializer, const mip_gnss_rtk_dongle_configuration_response* self);
void extract_mip_gnss_rtk_dongle_configuration_response(microstrain_serializer* serializer, mip_gnss_rtk_dongle_configuration_response* self);

mip_cmd_result mip_gnss_write_rtk_dongle_configuration(mip_interface* device, uint8_t enable, const uint8_t* reserved);
mip_cmd_result mip_gnss_read_rtk_dongle_configuration(mip_interface* device, uint8_t* enable_out, uint8_t* reserved_out);
mip_cmd_result mip_gnss_save_rtk_dongle_configuration(mip_interface* device);
mip_cmd_result mip_gnss_load_rtk_dongle_configuration(mip_interface* device);
mip_cmd_result mip_gnss_default_rtk_dongle_configuration(mip_interface* device);

///@}
///

///@}
///@}
///
////////////////////////////////////////////////////////////////////////////////
#ifdef __cplusplus
} // extern "C"
} // namespace C
} // namespace mip
#endif // __cplusplus

