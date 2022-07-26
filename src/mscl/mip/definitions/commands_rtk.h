#pragma once

#include "descriptors.h"
#include "../mip_result.h"

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
namespace mscl {
#endif // __cplusplus

struct mip_interface;

////////////////////////////////////////////////////////////////////////////////
///@addtogroup MipCommands
///@{
///@defgroup RTKCommands  RTK
///
///@{

////////////////////////////////////////////////////////////////////////////////
// Descriptors
////////////////////////////////////////////////////////////////////////////////

enum mip_rtk_commands_descriptors
{
    MIP_RTK_CMD_DESC_SET                          = 0x0F,
    
    MIP_CMD_DESC_RTK_GET_STATUS_FLAGS             = 0x01,
    MIP_CMD_DESC_RTK_GET_IMEI                     = 0x02,
    MIP_CMD_DESC_RTK_GET_IMSI                     = 0x03,
    MIP_CMD_DESC_RTK_GET_ICCID                    = 0x04,
    MIP_CMD_DESC_RTK_CONNECTED_DEVICE_TYPE        = 0x06,
    MIP_CMD_DESC_RTK_GET_ACT_CODE                 = 0x07,
    MIP_CMD_DESC_RTK_GET_MODEM_FIRMWARE_VERSION   = 0x08,
    MIP_CMD_DESC_RTK_GET_RSSI                     = 0x05,
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
#ifdef __cplusplus
namespace C {
extern "C" {
#endif // __cplusplus


////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////

enum mip_connected_device_type
{
    MIP_CONNECTED_DEVICE_TYPE_GENERIC = 0,  ///<  
    MIP_CONNECTED_DEVICE_TYPE_GQ7     = 1,  ///<  
};
size_t insert_mip_connected_device_type(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_connected_device_type self);
size_t extract_mip_connected_device_type(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_connected_device_type* self);

enum mip_media_selector
{
    MIP_MEDIA_SELECTOR_MEDIA_EXTERNALFLASH = 0,  ///<  
    MIP_MEDIA_SELECTOR_MEDIA_SD            = 1,  ///<  
};
size_t insert_mip_media_selector(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_media_selector self);
size_t extract_mip_media_selector(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_media_selector* self);

enum mip_led_action
{
    MIP_LED_ACTION_LED_NONE    = 0,  ///<  
    MIP_LED_ACTION_LED_FLASH   = 1,  ///<  
    MIP_LED_ACTION_LED_PULSATE = 2,  ///<  
};
size_t insert_mip_led_action(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_led_action self);
size_t extract_mip_led_action(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_led_action* self);


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
///@defgroup get_status_flags  Get RTK Device Status Flags
///
///@{

struct mip_rtk_get_status_flags_command
{
};
size_t insert_mip_rtk_get_status_flags_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_rtk_get_status_flags_command* self);
size_t extract_mip_rtk_get_status_flags_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_rtk_get_status_flags_command* self);

enum mip_rtk_get_status_flags_command_status_flags
{
    MIP_RTK_GET_STATUS_FLAGS_COMMAND_STATUS_FLAGS_MODEM_STATE             = 0x0F,
    MIP_RTK_GET_STATUS_FLAGS_COMMAND_STATUS_FLAGS_CONNECTION_TYPE         = 0xF0,
    MIP_RTK_GET_STATUS_FLAGS_COMMAND_STATUS_FLAGS_RSSI                    = 0xFF00,
    MIP_RTK_GET_STATUS_FLAGS_COMMAND_STATUS_FLAGS_SIGNAL_QUALITY          = 0xF0000,
    MIP_RTK_GET_STATUS_FLAGS_COMMAND_STATUS_FLAGS_TOWER_CHANGE_INDICATOR  = 0xF00000,
    MIP_RTK_GET_STATUS_FLAGS_COMMAND_STATUS_FLAGS_NMEA_TIMEOUT            = 0x1000000,
    MIP_RTK_GET_STATUS_FLAGS_COMMAND_STATUS_FLAGS_SERVER_TIMEOUT          = 0x2000000,
    MIP_RTK_GET_STATUS_FLAGS_COMMAND_STATUS_FLAGS_RTCM_TIMEOUT            = 0x4000000,
    MIP_RTK_GET_STATUS_FLAGS_COMMAND_STATUS_FLAGS_DEVICE_OUT_OF_RANGE     = 0x8000000,
    MIP_RTK_GET_STATUS_FLAGS_COMMAND_STATUS_FLAGS_CORRECTIONS_UNAVAILABLE = 0x10000000,
    MIP_RTK_GET_STATUS_FLAGS_COMMAND_STATUS_FLAGS_RESERVED                = 0x20000000,
    MIP_RTK_GET_STATUS_FLAGS_COMMAND_STATUS_FLAGS_VERSION                 = 0xC0000000,
};
size_t insert_mip_rtk_get_status_flags_command_status_flags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_rtk_get_status_flags_command_status_flags self);
size_t extract_mip_rtk_get_status_flags_command_status_flags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_rtk_get_status_flags_command_status_flags* self);

struct mip_rtk_get_status_flags_response
{
    enum mip_rtk_get_status_flags_command_status_flags flags;
};
size_t insert_mip_rtk_get_status_flags_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_rtk_get_status_flags_response* self);
size_t extract_mip_rtk_get_status_flags_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_rtk_get_status_flags_response* self);

mip_cmd_result mip_rtk_get_status_flags(struct mip_interface* device, enum mip_rtk_get_status_flags_command_status_flags* flags);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup get_imei  Get RTK Device IMEI (International Mobile Equipment Identifier)
///
///@{

struct mip_rtk_get_imei_command
{
};
size_t insert_mip_rtk_get_imei_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_rtk_get_imei_command* self);
size_t extract_mip_rtk_get_imei_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_rtk_get_imei_command* self);

struct mip_rtk_get_imei_response
{
    char                                              IMEI[32];
};
size_t insert_mip_rtk_get_imei_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_rtk_get_imei_response* self);
size_t extract_mip_rtk_get_imei_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_rtk_get_imei_response* self);

mip_cmd_result mip_rtk_get_imei(struct mip_interface* device, char* IMEI);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup get_imsi  Get RTK Device IMSI (International Mobile Subscriber Identifier)
///
///@{

struct mip_rtk_get_imsi_command
{
};
size_t insert_mip_rtk_get_imsi_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_rtk_get_imsi_command* self);
size_t extract_mip_rtk_get_imsi_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_rtk_get_imsi_command* self);

struct mip_rtk_get_imsi_response
{
    char                                              IMSI[32];
};
size_t insert_mip_rtk_get_imsi_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_rtk_get_imsi_response* self);
size_t extract_mip_rtk_get_imsi_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_rtk_get_imsi_response* self);

mip_cmd_result mip_rtk_get_imsi(struct mip_interface* device, char* IMSI);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup get_iccid  Get RTK Device ICCID (Integrated Circuit Card Identification [SIM Number])
///
///@{

struct mip_rtk_get_iccid_command
{
};
size_t insert_mip_rtk_get_iccid_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_rtk_get_iccid_command* self);
size_t extract_mip_rtk_get_iccid_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_rtk_get_iccid_command* self);

struct mip_rtk_get_iccid_response
{
    char                                              ICCID[32];
};
size_t insert_mip_rtk_get_iccid_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_rtk_get_iccid_response* self);
size_t extract_mip_rtk_get_iccid_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_rtk_get_iccid_response* self);

mip_cmd_result mip_rtk_get_iccid(struct mip_interface* device, char* ICCID);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup connected_device_type  Configure or read the type of the connected device
///
///@{

struct mip_rtk_connected_device_type_command
{
    enum mip_function_selector                        function;
    enum mip_connected_device_type                    devType;
};
size_t insert_mip_rtk_connected_device_type_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_rtk_connected_device_type_command* self);
size_t extract_mip_rtk_connected_device_type_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_rtk_connected_device_type_command* self);

struct mip_rtk_connected_device_type_response
{
    enum mip_connected_device_type                    devType;
};
size_t insert_mip_rtk_connected_device_type_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_rtk_connected_device_type_response* self);
size_t extract_mip_rtk_connected_device_type_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_rtk_connected_device_type_response* self);

mip_cmd_result write_mip_rtk_connected_device_type(struct mip_interface* device, enum mip_connected_device_type devType);
mip_cmd_result read_mip_rtk_connected_device_type(struct mip_interface* device, enum mip_connected_device_type* devType);
mip_cmd_result save_mip_rtk_connected_device_type(struct mip_interface* device);
mip_cmd_result load_mip_rtk_connected_device_type(struct mip_interface* device);
mip_cmd_result default_mip_rtk_connected_device_type(struct mip_interface* device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup get_act_code  Get RTK Device Activation Code
///
///@{

struct mip_rtk_get_act_code_command
{
};
size_t insert_mip_rtk_get_act_code_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_rtk_get_act_code_command* self);
size_t extract_mip_rtk_get_act_code_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_rtk_get_act_code_command* self);

struct mip_rtk_get_act_code_response
{
    char                                              ActivationCode[32];
};
size_t insert_mip_rtk_get_act_code_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_rtk_get_act_code_response* self);
size_t extract_mip_rtk_get_act_code_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_rtk_get_act_code_response* self);

mip_cmd_result mip_rtk_get_act_code(struct mip_interface* device, char* ActivationCode);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup get_modem_firmware_version  Get RTK Device's Cell Modem Firmware version number
///
///@{

struct mip_rtk_get_modem_firmware_version_command
{
};
size_t insert_mip_rtk_get_modem_firmware_version_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_rtk_get_modem_firmware_version_command* self);
size_t extract_mip_rtk_get_modem_firmware_version_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_rtk_get_modem_firmware_version_command* self);

struct mip_rtk_get_modem_firmware_version_response
{
    char                                              ModemFirmwareVersion[32];
};
size_t insert_mip_rtk_get_modem_firmware_version_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_rtk_get_modem_firmware_version_response* self);
size_t extract_mip_rtk_get_modem_firmware_version_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_rtk_get_modem_firmware_version_response* self);

mip_cmd_result mip_rtk_get_modem_firmware_version(struct mip_interface* device, char* ModemFirmwareVersion);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup get_rssi  None
/// Get the RSSI and connected/disconnected status of modem
///
///@{

struct mip_rtk_get_rssi_command
{
};
size_t insert_mip_rtk_get_rssi_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_rtk_get_rssi_command* self);
size_t extract_mip_rtk_get_rssi_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_rtk_get_rssi_command* self);

struct mip_rtk_get_rssi_response
{
    bool                                              valid;
    int32_t                                           rssi;
    int32_t                                           signalQuality;
};
size_t insert_mip_rtk_get_rssi_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_rtk_get_rssi_response* self);
size_t extract_mip_rtk_get_rssi_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_rtk_get_rssi_response* self);

mip_cmd_result mip_rtk_get_rssi(struct mip_interface* device, bool* valid, int32_t* rssi, int32_t* signalQuality);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup service_status  None
/// The 3DMRTK will send this message to the server to indicate that the connection should remain open. The Server will respond with information and status.
///
///@{

struct mip_rtk_service_status_command
{
    uint32_t                                          reserved1;
    uint32_t                                          reserved2;
};
size_t insert_mip_rtk_service_status_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_rtk_service_status_command* self);
size_t extract_mip_rtk_service_status_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_rtk_service_status_command* self);

enum mip_rtk_service_status_command_service_flags
{
    MIP_RTK_SERVICE_STATUS_COMMAND_SERVICE_FLAGS_THROTTLE                = 0x01,
    MIP_RTK_SERVICE_STATUS_COMMAND_SERVICE_FLAGS_CORRECTIONS_UNAVAILABLE = 0x02,
    MIP_RTK_SERVICE_STATUS_COMMAND_SERVICE_FLAGS_RESERVED                = 0xFC,
};
size_t insert_mip_rtk_service_status_command_service_flags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_rtk_service_status_command_service_flags self);
size_t extract_mip_rtk_service_status_command_service_flags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_rtk_service_status_command_service_flags* self);

struct mip_rtk_service_status_response
{
    enum mip_rtk_service_status_command_service_flags flags;
    uint32_t                                          recievedBytes;
    uint32_t                                          lastBytes;
    uint64_t                                          lastBytesTime;
};
size_t insert_mip_rtk_service_status_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_rtk_service_status_response* self);
size_t extract_mip_rtk_service_status_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_rtk_service_status_response* self);

mip_cmd_result mip_rtk_service_status(struct mip_interface* device, uint32_t reserved1, uint32_t reserved2, enum mip_rtk_service_status_command_service_flags* flags, uint32_t* recievedBytes, uint32_t* lastBytes, uint64_t* lastBytesTime);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup prod_erase_storage  None
/// This command will erase the selected media to a raw and unitialized state. ALL DATA WILL BE LOST.
/// This command is only available in calibration mode.
///
///@{

struct mip_rtk_prod_erase_storage_command
{
    enum mip_media_selector                           media;
};
size_t insert_mip_rtk_prod_erase_storage_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_rtk_prod_erase_storage_command* self);
size_t extract_mip_rtk_prod_erase_storage_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_rtk_prod_erase_storage_command* self);

mip_cmd_result mip_rtk_prod_erase_storage(struct mip_interface* device, enum mip_media_selector media);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup led_control  None
/// This command allows direct control of the LED on the 3DM RTK. This command is only available in calibration mode or Production Test Mode.
///
///@{

struct mip_rtk_led_control_command
{
    uint8_t                                           primaryColor[3];
    uint8_t                                           altColor[3];
    enum mip_led_action                               act;
    uint32_t                                          period;
};
size_t insert_mip_rtk_led_control_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_rtk_led_control_command* self);
size_t extract_mip_rtk_led_control_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_rtk_led_control_command* self);

mip_cmd_result mip_rtk_led_control(struct mip_interface* device, const uint8_t* primaryColor, const uint8_t* altColor, enum mip_led_action act, uint32_t period);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup modem_hard_reset  None
/// This command will clear the modem flash.  THIS MUST NOT BE DONE OFTEN AS IT CAN DAMAGE THE FLASH!
/// This command is only available in calibration mode.
///
///@{

struct mip_rtk_modem_hard_reset_command
{
};
size_t insert_mip_rtk_modem_hard_reset_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_rtk_modem_hard_reset_command* self);
size_t extract_mip_rtk_modem_hard_reset_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_rtk_modem_hard_reset_command* self);

mip_cmd_result mip_rtk_modem_hard_reset(struct mip_interface* device);
///@}
///

///@}
///@}
///
////////////////////////////////////////////////////////////////////////////////

#ifdef __cplusplus
} // extern "C"
} // namespace C
namespace RtkCommands {


struct GetStatusFlags : C::mip_rtk_get_status_flags_command
{
    static const uint8_t descriptorSet = MIP_RTK_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_RTK_GET_STATUS_FLAGS;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::insert_mip_rtk_get_status_flags_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_rtk_get_status_flags_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = false;
    
    struct Response : C::mip_rtk_get_status_flags_response
    {
        static const uint8_t descriptorSet = MIP_RTK_CMD_DESC_SET;
        static const uint8_t fieldDescriptor = MIP_REPLY_DESC_RTK_GET_STATUS_FLAGS;
        
        size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::insert_mip_rtk_get_status_flags_response(buffer, bufferSize, offset, this);
        }
        size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::extract_mip_rtk_get_status_flags_response(buffer, bufferSize, offset, this);
        }
    };
    
};
MipCmdResult getStatusFlags(C::mip_interface& device, enum C::mip_rtk_get_status_flags_command_status_flags& flags)
{
    return C::mip_rtk_get_status_flags(&device, &flags);
}



struct GetImei : C::mip_rtk_get_imei_command
{
    static const uint8_t descriptorSet = MIP_RTK_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_RTK_GET_IMEI;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::insert_mip_rtk_get_imei_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_rtk_get_imei_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = false;
    
    struct Response : C::mip_rtk_get_imei_response
    {
        static const uint8_t descriptorSet = MIP_RTK_CMD_DESC_SET;
        static const uint8_t fieldDescriptor = MIP_REPLY_DESC_RTK_GET_IMEI;
        
        size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::insert_mip_rtk_get_imei_response(buffer, bufferSize, offset, this);
        }
        size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::extract_mip_rtk_get_imei_response(buffer, bufferSize, offset, this);
        }
    };
    
};
MipCmdResult getImei(C::mip_interface& device, char* IMEI)
{
    return C::mip_rtk_get_imei(&device, IMEI);
}



struct GetImsi : C::mip_rtk_get_imsi_command
{
    static const uint8_t descriptorSet = MIP_RTK_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_RTK_GET_IMSI;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::insert_mip_rtk_get_imsi_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_rtk_get_imsi_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = false;
    
    struct Response : C::mip_rtk_get_imsi_response
    {
        static const uint8_t descriptorSet = MIP_RTK_CMD_DESC_SET;
        static const uint8_t fieldDescriptor = MIP_REPLY_DESC_RTK_GET_IMSI;
        
        size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::insert_mip_rtk_get_imsi_response(buffer, bufferSize, offset, this);
        }
        size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::extract_mip_rtk_get_imsi_response(buffer, bufferSize, offset, this);
        }
    };
    
};
MipCmdResult getImsi(C::mip_interface& device, char* IMSI)
{
    return C::mip_rtk_get_imsi(&device, IMSI);
}



struct GetIccid : C::mip_rtk_get_iccid_command
{
    static const uint8_t descriptorSet = MIP_RTK_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_RTK_GET_ICCID;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::insert_mip_rtk_get_iccid_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_rtk_get_iccid_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = false;
    
    struct Response : C::mip_rtk_get_iccid_response
    {
        static const uint8_t descriptorSet = MIP_RTK_CMD_DESC_SET;
        static const uint8_t fieldDescriptor = MIP_REPLY_DESC_RTK_GET_ICCID;
        
        size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::insert_mip_rtk_get_iccid_response(buffer, bufferSize, offset, this);
        }
        size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::extract_mip_rtk_get_iccid_response(buffer, bufferSize, offset, this);
        }
    };
    
};
MipCmdResult getIccid(C::mip_interface& device, char* ICCID)
{
    return C::mip_rtk_get_iccid(&device, ICCID);
}



struct ConnectedDeviceType : C::mip_rtk_connected_device_type_command
{
    static const uint8_t descriptorSet = MIP_RTK_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_RTK_CONNECTED_DEVICE_TYPE;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::insert_mip_rtk_connected_device_type_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_rtk_connected_device_type_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = true;
    static const bool canWrite = true;
    static const bool canRead = true;
    static const bool canSave = true;
    static const bool canLoad = true;
    static const bool canReset = true;
    
    struct Response : C::mip_rtk_connected_device_type_response
    {
        static const uint8_t descriptorSet = MIP_RTK_CMD_DESC_SET;
        static const uint8_t fieldDescriptor = MIP_REPLY_DESC_RTK_CONNECTED_DEVICE_TYPE;
        
        size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::insert_mip_rtk_connected_device_type_response(buffer, bufferSize, offset, this);
        }
        size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::extract_mip_rtk_connected_device_type_response(buffer, bufferSize, offset, this);
        }
    };
    
};
MipCmdResult writeConnectedDeviceType(C::mip_interface& device, C::mip_connected_device_type devType)
{
    return C::write_mip_rtk_connected_device_type(&device, devType);
}
MipCmdResult readConnectedDeviceType(C::mip_interface& device, enum C::mip_connected_device_type& devType)
{
    return C::read_mip_rtk_connected_device_type(&device, &devType);
}
MipCmdResult saveConnectedDeviceType(C::mip_interface& device)
{
    return C::save_mip_rtk_connected_device_type(&device);
}
MipCmdResult loadConnectedDeviceType(C::mip_interface& device)
{
    return C::load_mip_rtk_connected_device_type(&device);
}
MipCmdResult defaultConnectedDeviceType(C::mip_interface& device)
{
    return C::default_mip_rtk_connected_device_type(&device);
}



struct GetActCode : C::mip_rtk_get_act_code_command
{
    static const uint8_t descriptorSet = MIP_RTK_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_RTK_GET_ACT_CODE;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::insert_mip_rtk_get_act_code_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_rtk_get_act_code_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = false;
    
    struct Response : C::mip_rtk_get_act_code_response
    {
        static const uint8_t descriptorSet = MIP_RTK_CMD_DESC_SET;
        static const uint8_t fieldDescriptor = MIP_REPLY_DESC_RTK_GET_ACT_CODE;
        
        size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::insert_mip_rtk_get_act_code_response(buffer, bufferSize, offset, this);
        }
        size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::extract_mip_rtk_get_act_code_response(buffer, bufferSize, offset, this);
        }
    };
    
};
MipCmdResult getActCode(C::mip_interface& device, char* ActivationCode)
{
    return C::mip_rtk_get_act_code(&device, ActivationCode);
}



struct GetModemFirmwareVersion : C::mip_rtk_get_modem_firmware_version_command
{
    static const uint8_t descriptorSet = MIP_RTK_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_RTK_GET_MODEM_FIRMWARE_VERSION;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::insert_mip_rtk_get_modem_firmware_version_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_rtk_get_modem_firmware_version_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = false;
    
    struct Response : C::mip_rtk_get_modem_firmware_version_response
    {
        static const uint8_t descriptorSet = MIP_RTK_CMD_DESC_SET;
        static const uint8_t fieldDescriptor = MIP_REPLY_DESC_RTK_GET_MODEM_FIRMWARE_VERSION;
        
        size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::insert_mip_rtk_get_modem_firmware_version_response(buffer, bufferSize, offset, this);
        }
        size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::extract_mip_rtk_get_modem_firmware_version_response(buffer, bufferSize, offset, this);
        }
    };
    
};
MipCmdResult getModemFirmwareVersion(C::mip_interface& device, char* ModemFirmwareVersion)
{
    return C::mip_rtk_get_modem_firmware_version(&device, ModemFirmwareVersion);
}



struct GetRssi : C::mip_rtk_get_rssi_command
{
    static const uint8_t descriptorSet = MIP_RTK_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_RTK_GET_RSSI;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::insert_mip_rtk_get_rssi_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_rtk_get_rssi_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = false;
    
    struct Response : C::mip_rtk_get_rssi_response
    {
        static const uint8_t descriptorSet = MIP_RTK_CMD_DESC_SET;
        static const uint8_t fieldDescriptor = MIP_REPLY_DESC_RTK_GET_RSSI;
        
        size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::insert_mip_rtk_get_rssi_response(buffer, bufferSize, offset, this);
        }
        size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::extract_mip_rtk_get_rssi_response(buffer, bufferSize, offset, this);
        }
    };
    
};
MipCmdResult getRssi(C::mip_interface& device, bool& valid, int32_t& rssi, int32_t& signalQuality)
{
    return C::mip_rtk_get_rssi(&device, &valid, &rssi, &signalQuality);
}



struct ServiceStatus : C::mip_rtk_service_status_command
{
    static const uint8_t descriptorSet = MIP_RTK_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_RTK_SERVICE_STATUS;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::insert_mip_rtk_service_status_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_rtk_service_status_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = false;
    
    struct Response : C::mip_rtk_service_status_response
    {
        static const uint8_t descriptorSet = MIP_RTK_CMD_DESC_SET;
        static const uint8_t fieldDescriptor = MIP_REPLY_DESC_RTK_SERVICE_STATUS;
        
        size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::insert_mip_rtk_service_status_response(buffer, bufferSize, offset, this);
        }
        size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
        {
            return C::extract_mip_rtk_service_status_response(buffer, bufferSize, offset, this);
        }
    };
    
};
MipCmdResult serviceStatus(C::mip_interface& device, uint32_t reserved1, uint32_t reserved2, enum C::mip_rtk_service_status_command_service_flags& flags, uint32_t& recievedBytes, uint32_t& lastBytes, uint64_t& lastBytesTime)
{
    return C::mip_rtk_service_status(&device, reserved1, reserved2, &flags, &recievedBytes, &lastBytes, &lastBytesTime);
}



struct ProdEraseStorage : C::mip_rtk_prod_erase_storage_command
{
    static const uint8_t descriptorSet = MIP_RTK_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_RTK_PROD_ERASE_STORAGE;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::insert_mip_rtk_prod_erase_storage_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_rtk_prod_erase_storage_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = false;
    
};
MipCmdResult prodEraseStorage(C::mip_interface& device, C::mip_media_selector media)
{
    return C::mip_rtk_prod_erase_storage(&device, media);
}



struct LedControl : C::mip_rtk_led_control_command
{
    static const uint8_t descriptorSet = MIP_RTK_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_LED_CONTROL;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::insert_mip_rtk_led_control_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_rtk_led_control_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = false;
    
};
MipCmdResult ledControl(C::mip_interface& device, const uint8_t* primaryColor, const uint8_t* altColor, C::mip_led_action act, uint32_t period)
{
    return C::mip_rtk_led_control(&device, primaryColor, altColor, act, period);
}



struct ModemHardReset : C::mip_rtk_modem_hard_reset_command
{
    static const uint8_t descriptorSet = MIP_RTK_CMD_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_RTK_MODEM_HARD_RESET;
    
    size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::insert_mip_rtk_modem_hard_reset_command(buffer, bufferSize, offset, this);
    }
    size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset=0)
    {
        return C::extract_mip_rtk_modem_hard_reset_command(buffer, bufferSize, offset, this);
    }
    
    static const bool hasFunctionSelector = false;
    
};
MipCmdResult modemHardReset(C::mip_interface& device)
{
    return C::mip_rtk_modem_hard_reset(&device);
}



} // namespace RtkCommands
} // namespace mscl
#endif // __cplusplus
