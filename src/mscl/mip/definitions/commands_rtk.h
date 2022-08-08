#pragma once

#include "descriptors.h"
#include "../mip_result.h"

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
namespace mscl {
namespace C {
extern "C" {

#endif // __cplusplus
struct mip_interface;
struct mip_serializer;

////////////////////////////////////////////////////////////////////////////////
///@addtogroup MipCommands
///@{
///@defgroup rtk_commands_c  RTKCommands
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

void insert_mip_media_selector(struct mip_serializer* serializer, const enum mip_media_selector self);
void extract_mip_media_selector(struct mip_serializer* serializer, enum mip_media_selector* self);

enum mip_led_action
{
    MIP_LED_ACTION_LED_NONE    = 0,  ///<  
    MIP_LED_ACTION_LED_FLASH   = 1,  ///<  
    MIP_LED_ACTION_LED_PULSATE = 2,  ///<  
};

void insert_mip_led_action(struct mip_serializer* serializer, const enum mip_led_action self);
void extract_mip_led_action(struct mip_serializer* serializer, enum mip_led_action* self);


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
///@defgroup c_get_status_flags  Get RTK Device Status Flags
///
///@{

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_get_imei  Get RTK Device IMEI (International Mobile Equipment Identifier)
///
///@{

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_get_imsi  Get RTK Device IMSI (International Mobile Subscriber Identifier)
///
///@{

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_get_iccid  Get RTK Device ICCID (Integrated Circuit Card Identification [SIM Number])
///
///@{

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_connected_device_type  Configure or read the type of the connected device
///
///@{

enum mip_rtk_connected_device_type_command_type
{
    MIP_RTK_CONNECTED_DEVICE_TYPE_COMMAND_TYPE_GENERIC = 0,  ///<  
    MIP_RTK_CONNECTED_DEVICE_TYPE_COMMAND_TYPE_GQ7     = 1,  ///<  
};

struct mip_rtk_connected_device_type_command
{
    enum mip_function_selector function;
    enum mip_rtk_connected_device_type_command_type devType;
    
};
struct mip_rtk_connected_device_type_response
{
    enum mip_rtk_connected_device_type_command_type devType;
    
};
void insert_mip_rtk_connected_device_type_command(struct mip_serializer* serializer, const struct mip_rtk_connected_device_type_command* self);
void extract_mip_rtk_connected_device_type_command(struct mip_serializer* serializer, struct mip_rtk_connected_device_type_command* self);

void insert_mip_rtk_connected_device_type_command_type(struct mip_serializer* serializer, const enum mip_rtk_connected_device_type_command_type self);
void extract_mip_rtk_connected_device_type_command_type(struct mip_serializer* serializer, enum mip_rtk_connected_device_type_command_type* self);

void insert_mip_rtk_connected_device_type_response(struct mip_serializer* serializer, const struct mip_rtk_connected_device_type_response* self);
void extract_mip_rtk_connected_device_type_response(struct mip_serializer* serializer, struct mip_rtk_connected_device_type_response* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_get_act_code  Get RTK Device Activation Code
///
///@{

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_get_modem_firmware_version  Get RTK Device's Cell Modem Firmware version number
///
///@{

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_get_rssi  None
/// Get the RSSI and connected/disconnected status of modem
///
///@{

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_service_status  None
/// The 3DMRTK will send this message to the server to indicate that the connection should remain open. The Server will respond with information and status.
///
///@{

enum mip_rtk_service_status_command_service_flags
{
    MIP_RTK_SERVICE_STATUS_COMMAND_SERVICE_FLAGS_THROTTLE                = 0x01,
    MIP_RTK_SERVICE_STATUS_COMMAND_SERVICE_FLAGS_CORRECTIONS_UNAVAILABLE = 0x02,
    MIP_RTK_SERVICE_STATUS_COMMAND_SERVICE_FLAGS_RESERVED                = 0xFC,
};

struct mip_rtk_service_status_command
{
    uint32_t reserved1;
    uint32_t reserved2;
    
};
struct mip_rtk_service_status_response
{
    enum mip_rtk_service_status_command_service_flags flags;
    uint32_t recievedBytes;
    uint32_t lastBytes;
    uint64_t lastBytesTime;
    
};
void insert_mip_rtk_service_status_command(struct mip_serializer* serializer, const struct mip_rtk_service_status_command* self);
void extract_mip_rtk_service_status_command(struct mip_serializer* serializer, struct mip_rtk_service_status_command* self);

void insert_mip_rtk_service_status_command_service_flags(struct mip_serializer* serializer, const enum mip_rtk_service_status_command_service_flags self);
void extract_mip_rtk_service_status_command_service_flags(struct mip_serializer* serializer, enum mip_rtk_service_status_command_service_flags* self);

void insert_mip_rtk_service_status_response(struct mip_serializer* serializer, const struct mip_rtk_service_status_response* self);
void extract_mip_rtk_service_status_response(struct mip_serializer* serializer, struct mip_rtk_service_status_response* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_prod_erase_storage  None
/// This command will erase the selected media to a raw and unitialized state. ALL DATA WILL BE LOST.
/// This command is only available in calibration mode.
///
///@{

struct mip_rtk_prod_erase_storage_command
{
    enum mip_media_selector media;
    
};
void insert_mip_rtk_prod_erase_storage_command(struct mip_serializer* serializer, const struct mip_rtk_prod_erase_storage_command* self);
void extract_mip_rtk_prod_erase_storage_command(struct mip_serializer* serializer, struct mip_rtk_prod_erase_storage_command* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_led_control  None
/// This command allows direct control of the LED on the 3DM RTK. This command is only available in calibration mode or Production Test Mode.
///
///@{

struct mip_rtk_led_control_command
{
    uint8_t primaryColor[3];
    uint8_t altColor[3];
    enum mip_led_action act;
    uint32_t period;
    
};
void insert_mip_rtk_led_control_command(struct mip_serializer* serializer, const struct mip_rtk_led_control_command* self);
void extract_mip_rtk_led_control_command(struct mip_serializer* serializer, struct mip_rtk_led_control_command* self);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_modem_hard_reset  None
/// This command will clear the modem flash.  THIS MUST NOT BE DONE OFTEN AS IT CAN DAMAGE THE FLASH!
/// This command is only available in calibration mode.
///
///@{

///@}
///

///@}
///@}
///
////////////////////////////////////////////////////////////////////////////////
#ifdef __cplusplus
} // namespace C
} // namespace mscl
} // extern "C"
#endif // __cplusplus

