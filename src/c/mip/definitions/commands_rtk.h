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
///@defgroup rtk_commands_c  Rtk Commands
///
///@{

////////////////////////////////////////////////////////////////////////////////
// Descriptors
////////////////////////////////////////////////////////////////////////////////

enum 
{
    MIP_RTK_CMD_DESC_SET                          = 0x0F,
    
    MIP_CMD_DESC_RTK_GET_STATUS_FLAGS             = 0x01,
    MIP_CMD_DESC_RTK_GET_IMEI                     = 0x02,
    MIP_CMD_DESC_RTK_GET_IMSI                     = 0x03,
    MIP_CMD_DESC_RTK_GET_ICCID                    = 0x04,
    MIP_CMD_DESC_RTK_GET_RSSI                     = 0x05,
    MIP_CMD_DESC_RTK_CONNECTED_DEVICE_TYPE        = 0x06,
    MIP_CMD_DESC_RTK_GET_ACT_CODE                 = 0x07,
    MIP_CMD_DESC_RTK_GET_MODEM_FIRMWARE_VERSION   = 0x08,
    MIP_CMD_DESC_RTK_SERVICE_STATUS               = 0x0A,
    MIP_CMD_DESC_RTK_PROD_ERASE_STORAGE           = 0x20,
    MIP_CMD_DESC_LED_CONTROL                      = 0x21,
    MIP_CMD_DESC_RTK_MODEM_HARD_RESET             = 0x22,
    
    MIP_REPLY_DESC_RTK_GET_STATUS_FLAGS           = 0x81,
    MIP_REPLY_DESC_RTK_GET_IMEI                   = 0x82,
    MIP_REPLY_DESC_RTK_GET_IMSI                   = 0x83,
    MIP_REPLY_DESC_RTK_GET_ICCID                  = 0x84,
    MIP_REPLY_DESC_RTK_CONNECTED_DEVICE_TYPE      = 0x86,
    MIP_REPLY_DESC_RTK_GET_ACT_CODE               = 0x87,
    MIP_REPLY_DESC_RTK_GET_MODEM_FIRMWARE_VERSION = 0x88,
    MIP_REPLY_DESC_RTK_GET_RSSI                   = 0x85,
    MIP_REPLY_DESC_RTK_SERVICE_STATUS             = 0x8A,
};

////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////

enum mip_media_selector
{
    MIP_MEDIA_SELECTOR_MEDIA_EXTERNALFLASH = 0,  ///<  
    MIP_MEDIA_SELECTOR_MEDIA_SD            = 1,  ///<  
};
typedef enum mip_media_selector mip_media_selector;

static inline void insert_mip_media_selector(microstrain_serializer* serializer, const mip_media_selector self)
{
    microstrain_insert_u8(serializer, (uint8_t)(self));
}
static inline void extract_mip_media_selector(microstrain_serializer* serializer, mip_media_selector* self)
{
    uint8_t tmp = 0;
    microstrain_extract_u8(serializer, &tmp);
    *self = (mip_media_selector)tmp;
}

enum mip_led_action
{
    MIP_LED_ACTION_LED_NONE    = 0,  ///<  
    MIP_LED_ACTION_LED_FLASH   = 1,  ///<  
    MIP_LED_ACTION_LED_PULSATE = 2,  ///<  
};
typedef enum mip_led_action mip_led_action;

static inline void insert_mip_led_action(microstrain_serializer* serializer, const mip_led_action self)
{
    microstrain_insert_u8(serializer, (uint8_t)(self));
}
static inline void extract_mip_led_action(microstrain_serializer* serializer, mip_led_action* self)
{
    uint8_t tmp = 0;
    microstrain_extract_u8(serializer, &tmp);
    *self = (mip_led_action)tmp;
}


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
///@defgroup rtk_get_status_flags_c  (0x0F,0x01) Get Status Flags
///
///@{

typedef uint32_t mip_rtk_get_status_flags_command_status_flags_legacy;
static const mip_rtk_get_status_flags_command_status_flags_legacy MIP_RTK_GET_STATUS_FLAGS_COMMAND_STATUS_FLAGS_LEGACY_NONE                 = 0x00000000;
static const mip_rtk_get_status_flags_command_status_flags_legacy MIP_RTK_GET_STATUS_FLAGS_COMMAND_STATUS_FLAGS_LEGACY_CONTROLLERSTATE      = 0x00000007; ///<  
static const mip_rtk_get_status_flags_command_status_flags_legacy MIP_RTK_GET_STATUS_FLAGS_COMMAND_STATUS_FLAGS_LEGACY_PLATFORMSTATE        = 0x000000F8; ///<  
static const mip_rtk_get_status_flags_command_status_flags_legacy MIP_RTK_GET_STATUS_FLAGS_COMMAND_STATUS_FLAGS_LEGACY_CONTROLLERSTATUSCODE = 0x00000700; ///<  
static const mip_rtk_get_status_flags_command_status_flags_legacy MIP_RTK_GET_STATUS_FLAGS_COMMAND_STATUS_FLAGS_LEGACY_PLATFORMSTATUSCODE   = 0x00003800; ///<  
static const mip_rtk_get_status_flags_command_status_flags_legacy MIP_RTK_GET_STATUS_FLAGS_COMMAND_STATUS_FLAGS_LEGACY_RESETCODE            = 0x0000C000; ///<  
static const mip_rtk_get_status_flags_command_status_flags_legacy MIP_RTK_GET_STATUS_FLAGS_COMMAND_STATUS_FLAGS_LEGACY_SIGNALQUALITY        = 0x000F0000; ///<  
static const mip_rtk_get_status_flags_command_status_flags_legacy MIP_RTK_GET_STATUS_FLAGS_COMMAND_STATUS_FLAGS_LEGACY_RESERVED             = 0xFFF00000; ///<  
static const mip_rtk_get_status_flags_command_status_flags_legacy MIP_RTK_GET_STATUS_FLAGS_COMMAND_STATUS_FLAGS_LEGACY_RSSI                 = 0x03F00000; ///<  
static const mip_rtk_get_status_flags_command_status_flags_legacy MIP_RTK_GET_STATUS_FLAGS_COMMAND_STATUS_FLAGS_LEGACY_RSRP                 = 0x0C000000; ///<  
static const mip_rtk_get_status_flags_command_status_flags_legacy MIP_RTK_GET_STATUS_FLAGS_COMMAND_STATUS_FLAGS_LEGACY_RSRQ                 = 0x30000000; ///<  
static const mip_rtk_get_status_flags_command_status_flags_legacy MIP_RTK_GET_STATUS_FLAGS_COMMAND_STATUS_FLAGS_LEGACY_SINR                 = 0xC0000000; ///<  
static const mip_rtk_get_status_flags_command_status_flags_legacy MIP_RTK_GET_STATUS_FLAGS_COMMAND_STATUS_FLAGS_LEGACY_ALL                  = 0xFFFFFFFF;
static inline void insert_mip_rtk_get_status_flags_command_status_flags_legacy(microstrain_serializer* serializer, const mip_rtk_get_status_flags_command_status_flags_legacy self)
{
    microstrain_insert_u32(serializer, (uint32_t)(self));
}
static inline void extract_mip_rtk_get_status_flags_command_status_flags_legacy(microstrain_serializer* serializer, mip_rtk_get_status_flags_command_status_flags_legacy* self)
{
    uint32_t tmp = 0;
    microstrain_extract_u32(serializer, &tmp);
    *self = (mip_rtk_get_status_flags_command_status_flags_legacy)tmp;
}

typedef uint32_t mip_rtk_get_status_flags_command_status_flags;
static const mip_rtk_get_status_flags_command_status_flags MIP_RTK_GET_STATUS_FLAGS_COMMAND_STATUS_FLAGS_NONE                    = 0x00000000;
static const mip_rtk_get_status_flags_command_status_flags MIP_RTK_GET_STATUS_FLAGS_COMMAND_STATUS_FLAGS_MODEM_STATE             = 0x0000000F; ///<  
static const mip_rtk_get_status_flags_command_status_flags MIP_RTK_GET_STATUS_FLAGS_COMMAND_STATUS_FLAGS_CONNECTION_TYPE         = 0x000000F0; ///<  
static const mip_rtk_get_status_flags_command_status_flags MIP_RTK_GET_STATUS_FLAGS_COMMAND_STATUS_FLAGS_RSSI                    = 0x0000FF00; ///<  
static const mip_rtk_get_status_flags_command_status_flags MIP_RTK_GET_STATUS_FLAGS_COMMAND_STATUS_FLAGS_SIGNAL_QUALITY          = 0x000F0000; ///<  
static const mip_rtk_get_status_flags_command_status_flags MIP_RTK_GET_STATUS_FLAGS_COMMAND_STATUS_FLAGS_TOWER_CHANGE_INDICATOR  = 0x00F00000; ///<  
static const mip_rtk_get_status_flags_command_status_flags MIP_RTK_GET_STATUS_FLAGS_COMMAND_STATUS_FLAGS_NMEA_TIMEOUT            = 0x01000000; ///<  
static const mip_rtk_get_status_flags_command_status_flags MIP_RTK_GET_STATUS_FLAGS_COMMAND_STATUS_FLAGS_SERVER_TIMEOUT          = 0x02000000; ///<  
static const mip_rtk_get_status_flags_command_status_flags MIP_RTK_GET_STATUS_FLAGS_COMMAND_STATUS_FLAGS_CORRECTIONS_TIMEOUT     = 0x04000000; ///<  
static const mip_rtk_get_status_flags_command_status_flags MIP_RTK_GET_STATUS_FLAGS_COMMAND_STATUS_FLAGS_DEVICE_OUT_OF_RANGE     = 0x08000000; ///<  
static const mip_rtk_get_status_flags_command_status_flags MIP_RTK_GET_STATUS_FLAGS_COMMAND_STATUS_FLAGS_CORRECTIONS_UNAVAILABLE = 0x10000000; ///<  
static const mip_rtk_get_status_flags_command_status_flags MIP_RTK_GET_STATUS_FLAGS_COMMAND_STATUS_FLAGS_RESERVED                = 0x20000000; ///<  
static const mip_rtk_get_status_flags_command_status_flags MIP_RTK_GET_STATUS_FLAGS_COMMAND_STATUS_FLAGS_VERSION                 = 0xC0000000; ///<  
static const mip_rtk_get_status_flags_command_status_flags MIP_RTK_GET_STATUS_FLAGS_COMMAND_STATUS_FLAGS_ALL                     = 0xFFFFFFFF;
static inline void insert_mip_rtk_get_status_flags_command_status_flags(microstrain_serializer* serializer, const mip_rtk_get_status_flags_command_status_flags self)
{
    microstrain_insert_u32(serializer, (uint32_t)(self));
}
static inline void extract_mip_rtk_get_status_flags_command_status_flags(microstrain_serializer* serializer, mip_rtk_get_status_flags_command_status_flags* self)
{
    uint32_t tmp = 0;
    microstrain_extract_u32(serializer, &tmp);
    *self = (mip_rtk_get_status_flags_command_status_flags)tmp;
}


typedef struct mip_rtk_get_status_flags_command mip_rtk_get_status_flags_command; ///< No parameters (empty struct not allowed in C)

struct mip_rtk_get_status_flags_response
{
    mip_rtk_get_status_flags_command_status_flags flags; ///< Model number dependent. See above structures.
};
typedef struct mip_rtk_get_status_flags_response mip_rtk_get_status_flags_response;

void insert_mip_rtk_get_status_flags_response(microstrain_serializer* serializer, const mip_rtk_get_status_flags_response* self);
void extract_mip_rtk_get_status_flags_response(microstrain_serializer* serializer, mip_rtk_get_status_flags_response* self);

mip_cmd_result mip_rtk_get_status_flags(mip_interface* device, mip_rtk_get_status_flags_command_status_flags* flags_out);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup rtk_get_imei_c  (0x0F,0x02) Get Imei
///
///@{

typedef struct mip_rtk_get_imei_command mip_rtk_get_imei_command; ///< No parameters (empty struct not allowed in C)

struct mip_rtk_get_imei_response
{
    char IMEI[32];
};
typedef struct mip_rtk_get_imei_response mip_rtk_get_imei_response;

void insert_mip_rtk_get_imei_response(microstrain_serializer* serializer, const mip_rtk_get_imei_response* self);
void extract_mip_rtk_get_imei_response(microstrain_serializer* serializer, mip_rtk_get_imei_response* self);

mip_cmd_result mip_rtk_get_imei(mip_interface* device, char* imei_out);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup rtk_get_imsi_c  (0x0F,0x03) Get Imsi
///
///@{

typedef struct mip_rtk_get_imsi_command mip_rtk_get_imsi_command; ///< No parameters (empty struct not allowed in C)

struct mip_rtk_get_imsi_response
{
    char IMSI[32];
};
typedef struct mip_rtk_get_imsi_response mip_rtk_get_imsi_response;

void insert_mip_rtk_get_imsi_response(microstrain_serializer* serializer, const mip_rtk_get_imsi_response* self);
void extract_mip_rtk_get_imsi_response(microstrain_serializer* serializer, mip_rtk_get_imsi_response* self);

mip_cmd_result mip_rtk_get_imsi(mip_interface* device, char* imsi_out);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup rtk_get_iccid_c  (0x0F,0x04) Get Iccid
///
///@{

typedef struct mip_rtk_get_iccid_command mip_rtk_get_iccid_command; ///< No parameters (empty struct not allowed in C)

struct mip_rtk_get_iccid_response
{
    char ICCID[32];
};
typedef struct mip_rtk_get_iccid_response mip_rtk_get_iccid_response;

void insert_mip_rtk_get_iccid_response(microstrain_serializer* serializer, const mip_rtk_get_iccid_response* self);
void extract_mip_rtk_get_iccid_response(microstrain_serializer* serializer, mip_rtk_get_iccid_response* self);

mip_cmd_result mip_rtk_get_iccid(mip_interface* device, char* iccid_out);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup rtk_connected_device_type_c  (0x0F,0x06) Connected Device Type
///
///@{

enum mip_rtk_connected_device_type_command_type
{
    MIP_RTK_CONNECTED_DEVICE_TYPE_COMMAND_TYPE_GENERIC = 0,  ///<  
    MIP_RTK_CONNECTED_DEVICE_TYPE_COMMAND_TYPE_GQ7     = 1,  ///<  
};
typedef enum mip_rtk_connected_device_type_command_type mip_rtk_connected_device_type_command_type;

static inline void insert_mip_rtk_connected_device_type_command_type(microstrain_serializer* serializer, const mip_rtk_connected_device_type_command_type self)
{
    microstrain_insert_u8(serializer, (uint8_t)(self));
}
static inline void extract_mip_rtk_connected_device_type_command_type(microstrain_serializer* serializer, mip_rtk_connected_device_type_command_type* self)
{
    uint8_t tmp = 0;
    microstrain_extract_u8(serializer, &tmp);
    *self = (mip_rtk_connected_device_type_command_type)tmp;
}


struct mip_rtk_connected_device_type_command
{
    mip_function_selector function;
    mip_rtk_connected_device_type_command_type devType;
};
typedef struct mip_rtk_connected_device_type_command mip_rtk_connected_device_type_command;

void insert_mip_rtk_connected_device_type_command(microstrain_serializer* serializer, const mip_rtk_connected_device_type_command* self);
void extract_mip_rtk_connected_device_type_command(microstrain_serializer* serializer, mip_rtk_connected_device_type_command* self);

struct mip_rtk_connected_device_type_response
{
    mip_rtk_connected_device_type_command_type devType;
};
typedef struct mip_rtk_connected_device_type_response mip_rtk_connected_device_type_response;

void insert_mip_rtk_connected_device_type_response(microstrain_serializer* serializer, const mip_rtk_connected_device_type_response* self);
void extract_mip_rtk_connected_device_type_response(microstrain_serializer* serializer, mip_rtk_connected_device_type_response* self);

mip_cmd_result mip_rtk_write_connected_device_type(mip_interface* device, mip_rtk_connected_device_type_command_type dev_type);
mip_cmd_result mip_rtk_read_connected_device_type(mip_interface* device, mip_rtk_connected_device_type_command_type* dev_type_out);
mip_cmd_result mip_rtk_save_connected_device_type(mip_interface* device);
mip_cmd_result mip_rtk_load_connected_device_type(mip_interface* device);
mip_cmd_result mip_rtk_default_connected_device_type(mip_interface* device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup rtk_get_act_code_c  (0x0F,0x07) Get Act Code
///
///@{

typedef struct mip_rtk_get_act_code_command mip_rtk_get_act_code_command; ///< No parameters (empty struct not allowed in C)

struct mip_rtk_get_act_code_response
{
    char ActivationCode[32];
};
typedef struct mip_rtk_get_act_code_response mip_rtk_get_act_code_response;

void insert_mip_rtk_get_act_code_response(microstrain_serializer* serializer, const mip_rtk_get_act_code_response* self);
void extract_mip_rtk_get_act_code_response(microstrain_serializer* serializer, mip_rtk_get_act_code_response* self);

mip_cmd_result mip_rtk_get_act_code(mip_interface* device, char* activation_code_out);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup rtk_get_modem_firmware_version_c  (0x0F,0x08) Get Modem Firmware Version
///
///@{

typedef struct mip_rtk_get_modem_firmware_version_command mip_rtk_get_modem_firmware_version_command; ///< No parameters (empty struct not allowed in C)

struct mip_rtk_get_modem_firmware_version_response
{
    char ModemFirmwareVersion[32];
};
typedef struct mip_rtk_get_modem_firmware_version_response mip_rtk_get_modem_firmware_version_response;

void insert_mip_rtk_get_modem_firmware_version_response(microstrain_serializer* serializer, const mip_rtk_get_modem_firmware_version_response* self);
void extract_mip_rtk_get_modem_firmware_version_response(microstrain_serializer* serializer, mip_rtk_get_modem_firmware_version_response* self);

mip_cmd_result mip_rtk_get_modem_firmware_version(mip_interface* device, char* modem_firmware_version_out);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup rtk_get_rssi_c  (0x0F,0x05) Get Rssi
/// Get the RSSI and connected/disconnected status of modem
///
///@{

typedef struct mip_rtk_get_rssi_command mip_rtk_get_rssi_command; ///< No parameters (empty struct not allowed in C)

struct mip_rtk_get_rssi_response
{
    bool valid;
    int32_t rssi;
    int32_t signalQuality;
};
typedef struct mip_rtk_get_rssi_response mip_rtk_get_rssi_response;

void insert_mip_rtk_get_rssi_response(microstrain_serializer* serializer, const mip_rtk_get_rssi_response* self);
void extract_mip_rtk_get_rssi_response(microstrain_serializer* serializer, mip_rtk_get_rssi_response* self);

mip_cmd_result mip_rtk_get_rssi(mip_interface* device, bool* valid_out, int32_t* rssi_out, int32_t* signal_quality_out);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup rtk_service_status_c  (0x0F,0x0A) Service Status
/// The 3DMRTK will send this message to the server to indicate that the connection should remain open. The Server will respond with information and status.
///
///@{

typedef uint8_t mip_rtk_service_status_command_service_flags;
static const mip_rtk_service_status_command_service_flags MIP_RTK_SERVICE_STATUS_COMMAND_SERVICE_FLAGS_NONE                    = 0x00;
static const mip_rtk_service_status_command_service_flags MIP_RTK_SERVICE_STATUS_COMMAND_SERVICE_FLAGS_THROTTLE                = 0x01; ///<  
static const mip_rtk_service_status_command_service_flags MIP_RTK_SERVICE_STATUS_COMMAND_SERVICE_FLAGS_CORRECTIONS_UNAVAILABLE = 0x02; ///<  
static const mip_rtk_service_status_command_service_flags MIP_RTK_SERVICE_STATUS_COMMAND_SERVICE_FLAGS_RESERVED                = 0xFC; ///<  
static const mip_rtk_service_status_command_service_flags MIP_RTK_SERVICE_STATUS_COMMAND_SERVICE_FLAGS_ALL                     = 0xFF;
static inline void insert_mip_rtk_service_status_command_service_flags(microstrain_serializer* serializer, const mip_rtk_service_status_command_service_flags self)
{
    microstrain_insert_u8(serializer, (uint8_t)(self));
}
static inline void extract_mip_rtk_service_status_command_service_flags(microstrain_serializer* serializer, mip_rtk_service_status_command_service_flags* self)
{
    uint8_t tmp = 0;
    microstrain_extract_u8(serializer, &tmp);
    *self = (mip_rtk_service_status_command_service_flags)tmp;
}


struct mip_rtk_service_status_command
{
    uint32_t reserved1;
    uint32_t reserved2;
};
typedef struct mip_rtk_service_status_command mip_rtk_service_status_command;

void insert_mip_rtk_service_status_command(microstrain_serializer* serializer, const mip_rtk_service_status_command* self);
void extract_mip_rtk_service_status_command(microstrain_serializer* serializer, mip_rtk_service_status_command* self);

struct mip_rtk_service_status_response
{
    mip_rtk_service_status_command_service_flags flags;
    uint32_t receivedBytes;
    uint32_t lastBytes;
    uint64_t lastBytesTime;
};
typedef struct mip_rtk_service_status_response mip_rtk_service_status_response;

void insert_mip_rtk_service_status_response(microstrain_serializer* serializer, const mip_rtk_service_status_response* self);
void extract_mip_rtk_service_status_response(microstrain_serializer* serializer, mip_rtk_service_status_response* self);

mip_cmd_result mip_rtk_service_status(mip_interface* device, uint32_t reserved1, uint32_t reserved2, mip_rtk_service_status_command_service_flags* flags_out, uint32_t* received_bytes_out, uint32_t* last_bytes_out, uint64_t* last_bytes_time_out);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup rtk_prod_erase_storage_c  (0x0F,0x20) Prod Erase Storage
/// This command will erase the selected media to a raw and uninitialized state. ALL DATA WILL BE LOST.
/// This command is only available in calibration mode.
///
///@{

struct mip_rtk_prod_erase_storage_command
{
    mip_media_selector media;
};
typedef struct mip_rtk_prod_erase_storage_command mip_rtk_prod_erase_storage_command;

void insert_mip_rtk_prod_erase_storage_command(microstrain_serializer* serializer, const mip_rtk_prod_erase_storage_command* self);
void extract_mip_rtk_prod_erase_storage_command(microstrain_serializer* serializer, mip_rtk_prod_erase_storage_command* self);

mip_cmd_result mip_rtk_prod_erase_storage(mip_interface* device, mip_media_selector media);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup rtk_led_control_c  (0x0F,0x21) Led Control
/// This command allows direct control of the LED on the 3DM RTK. This command is only available in calibration mode or Production Test Mode.
///
///@{

struct mip_rtk_led_control_command
{
    uint8_t primaryColor[3];
    uint8_t altColor[3];
    mip_led_action act;
    uint32_t period;
};
typedef struct mip_rtk_led_control_command mip_rtk_led_control_command;

void insert_mip_rtk_led_control_command(microstrain_serializer* serializer, const mip_rtk_led_control_command* self);
void extract_mip_rtk_led_control_command(microstrain_serializer* serializer, mip_rtk_led_control_command* self);

mip_cmd_result mip_rtk_led_control(mip_interface* device, const uint8_t* primary_color, const uint8_t* alt_color, mip_led_action act, uint32_t period);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup rtk_modem_hard_reset_c  (0x0F,0x22) Modem Hard Reset
/// This command will clear the modem flash.  THIS MUST NOT BE DONE OFTEN AS IT CAN DAMAGE THE FLASH!
/// This command is only available in calibration mode.
///
///@{

typedef struct mip_rtk_modem_hard_reset_command mip_rtk_modem_hard_reset_command; ///< No parameters (empty struct not allowed in C)

mip_cmd_result mip_rtk_modem_hard_reset(mip_interface* device);

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

