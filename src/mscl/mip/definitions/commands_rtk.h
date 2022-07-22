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
///@defgroup RTK_COMMAND  RTK COMMAND
///
///@{

////////////////////////////////////////////////////////////////////////////////
// Descriptors
////////////////////////////////////////////////////////////////////////////////

enum mip_rtk_command_descriptors
{
    MIP_RTK_COMMAND_DESC_SET                      = 0x0F,
    
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
///@defgroup mip_rtk_get_status_flags  Get RTK Device Status Flags
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

mip_cmd_result get_rtk_device_status_flags(struct mip_interface* device, enum mip_rtk_get_status_flags_command_status_flags* flags);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_rtk_get_imei  Get RTK Device IMEI (International Mobile Equipment Identifier)
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

mip_cmd_result get_rtk_device_imei_international_mobile_equipment_identifier(struct mip_interface* device, char* IMEI);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_rtk_get_imsi  Get RTK Device IMSI (International Mobile Subscriber Identifier)
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

mip_cmd_result get_rtk_device_imsi_international_mobile_subscriber_identifier(struct mip_interface* device, char* IMSI);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_rtk_get_iccid  Get RTK Device ICCID (Integrated Circuit Card Identification [SIM Number])
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

mip_cmd_result get_rtk_device_iccid_integrated_circuit_card_identification_sim_number(struct mip_interface* device, char* ICCID);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_rtk_connected_device_type  Configure or read the type of the connected device
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

mip_cmd_result write_configure_or_read_the_type_of_the_connected_device(struct mip_interface* device, enum mip_connected_device_type devType);
mip_cmd_result read_configure_or_read_the_type_of_the_connected_device(struct mip_interface* device, enum mip_connected_device_type* devType);
mip_cmd_result save_configure_or_read_the_type_of_the_connected_device(struct mip_interface* device);
mip_cmd_result load_configure_or_read_the_type_of_the_connected_device(struct mip_interface* device);
mip_cmd_result default_configure_or_read_the_type_of_the_connected_device(struct mip_interface* device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_rtk_get_act_code  Get RTK Device Activation Code
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

mip_cmd_result get_rtk_device_activation_code(struct mip_interface* device, char* ActivationCode);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_rtk_get_modem_firmware_version  Get RTK Device's Cell Modem Firmware version number
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

mip_cmd_result get_rtk_devices_cell_modem_firmware_version_number(struct mip_interface* device, char* ModemFirmwareVersion);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_rtk_get_rssi  None
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
///@defgroup mip_rtk_service_status  None
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
///@defgroup mip_rtk_prod_erase_storage  None
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
///@defgroup mip_rtk_led_control  None
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
///@defgroup mip_rtk_modem_hard_reset  None
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


template<>
struct MipFieldInfo<C::mip_rtk_get_status_flags_command>
{
    static const uint8_t descriptorSet = MIP_RTK_COMMAND_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_RTK_GET_STATUS_FLAGS;
    static const uint8_t responseDescriptor = MIP_REPLY_DESC_RTK_GET_STATUS_FLAGS;
    typedef C::mip_rtk_get_status_flags_response Response;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_rtk_get_status_flags_command& self)
    {
        return C::insert_mip_rtk_get_status_flags_command(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_rtk_get_status_flags_command& self)
    {
        return C::extract_mip_rtk_get_status_flags_command(buffer, bufferSize, offset, &self);
    }
    static inline size_t insert_response(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_rtk_get_status_flags_response& self)
    {
        return C::insert_mip_rtk_get_status_flags_response(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract_response(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_rtk_get_status_flags_response& self)
    {
        return C::extract_mip_rtk_get_status_flags_response(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_rtk_get_imei_command>
{
    static const uint8_t descriptorSet = MIP_RTK_COMMAND_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_RTK_GET_IMEI;
    static const uint8_t responseDescriptor = MIP_REPLY_DESC_RTK_GET_IMEI;
    typedef C::mip_rtk_get_imei_response Response;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_rtk_get_imei_command& self)
    {
        return C::insert_mip_rtk_get_imei_command(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_rtk_get_imei_command& self)
    {
        return C::extract_mip_rtk_get_imei_command(buffer, bufferSize, offset, &self);
    }
    static inline size_t insert_response(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_rtk_get_imei_response& self)
    {
        return C::insert_mip_rtk_get_imei_response(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract_response(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_rtk_get_imei_response& self)
    {
        return C::extract_mip_rtk_get_imei_response(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_rtk_get_imsi_command>
{
    static const uint8_t descriptorSet = MIP_RTK_COMMAND_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_RTK_GET_IMSI;
    static const uint8_t responseDescriptor = MIP_REPLY_DESC_RTK_GET_IMSI;
    typedef C::mip_rtk_get_imsi_response Response;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_rtk_get_imsi_command& self)
    {
        return C::insert_mip_rtk_get_imsi_command(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_rtk_get_imsi_command& self)
    {
        return C::extract_mip_rtk_get_imsi_command(buffer, bufferSize, offset, &self);
    }
    static inline size_t insert_response(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_rtk_get_imsi_response& self)
    {
        return C::insert_mip_rtk_get_imsi_response(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract_response(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_rtk_get_imsi_response& self)
    {
        return C::extract_mip_rtk_get_imsi_response(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_rtk_get_iccid_command>
{
    static const uint8_t descriptorSet = MIP_RTK_COMMAND_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_RTK_GET_ICCID;
    static const uint8_t responseDescriptor = MIP_REPLY_DESC_RTK_GET_ICCID;
    typedef C::mip_rtk_get_iccid_response Response;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_rtk_get_iccid_command& self)
    {
        return C::insert_mip_rtk_get_iccid_command(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_rtk_get_iccid_command& self)
    {
        return C::extract_mip_rtk_get_iccid_command(buffer, bufferSize, offset, &self);
    }
    static inline size_t insert_response(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_rtk_get_iccid_response& self)
    {
        return C::insert_mip_rtk_get_iccid_response(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract_response(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_rtk_get_iccid_response& self)
    {
        return C::extract_mip_rtk_get_iccid_response(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_rtk_connected_device_type_command>
{
    static const uint8_t descriptorSet = MIP_RTK_COMMAND_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_RTK_CONNECTED_DEVICE_TYPE;
    static const uint8_t responseDescriptor = MIP_REPLY_DESC_RTK_CONNECTED_DEVICE_TYPE;
    typedef C::mip_rtk_connected_device_type_response Response;
    
    static const bool hasFunctionSelector = true;
    static const bool canWrite = true;
    static const bool canRead = true;
    static const bool canSave = true;
    static const bool canLoad = true;
    static const bool canReset = true;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_rtk_connected_device_type_command& self)
    {
        return C::insert_mip_rtk_connected_device_type_command(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_rtk_connected_device_type_command& self)
    {
        return C::extract_mip_rtk_connected_device_type_command(buffer, bufferSize, offset, &self);
    }
    static inline size_t insert_response(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_rtk_connected_device_type_response& self)
    {
        return C::insert_mip_rtk_connected_device_type_response(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract_response(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_rtk_connected_device_type_response& self)
    {
        return C::extract_mip_rtk_connected_device_type_response(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_rtk_get_act_code_command>
{
    static const uint8_t descriptorSet = MIP_RTK_COMMAND_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_RTK_GET_ACT_CODE;
    static const uint8_t responseDescriptor = MIP_REPLY_DESC_RTK_GET_ACT_CODE;
    typedef C::mip_rtk_get_act_code_response Response;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_rtk_get_act_code_command& self)
    {
        return C::insert_mip_rtk_get_act_code_command(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_rtk_get_act_code_command& self)
    {
        return C::extract_mip_rtk_get_act_code_command(buffer, bufferSize, offset, &self);
    }
    static inline size_t insert_response(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_rtk_get_act_code_response& self)
    {
        return C::insert_mip_rtk_get_act_code_response(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract_response(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_rtk_get_act_code_response& self)
    {
        return C::extract_mip_rtk_get_act_code_response(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_rtk_get_modem_firmware_version_command>
{
    static const uint8_t descriptorSet = MIP_RTK_COMMAND_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_RTK_GET_MODEM_FIRMWARE_VERSION;
    static const uint8_t responseDescriptor = MIP_REPLY_DESC_RTK_GET_MODEM_FIRMWARE_VERSION;
    typedef C::mip_rtk_get_modem_firmware_version_response Response;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_rtk_get_modem_firmware_version_command& self)
    {
        return C::insert_mip_rtk_get_modem_firmware_version_command(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_rtk_get_modem_firmware_version_command& self)
    {
        return C::extract_mip_rtk_get_modem_firmware_version_command(buffer, bufferSize, offset, &self);
    }
    static inline size_t insert_response(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_rtk_get_modem_firmware_version_response& self)
    {
        return C::insert_mip_rtk_get_modem_firmware_version_response(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract_response(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_rtk_get_modem_firmware_version_response& self)
    {
        return C::extract_mip_rtk_get_modem_firmware_version_response(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_rtk_get_rssi_command>
{
    static const uint8_t descriptorSet = MIP_RTK_COMMAND_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_RTK_GET_RSSI;
    static const uint8_t responseDescriptor = MIP_REPLY_DESC_RTK_GET_RSSI;
    typedef C::mip_rtk_get_rssi_response Response;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_rtk_get_rssi_command& self)
    {
        return C::insert_mip_rtk_get_rssi_command(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_rtk_get_rssi_command& self)
    {
        return C::extract_mip_rtk_get_rssi_command(buffer, bufferSize, offset, &self);
    }
    static inline size_t insert_response(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_rtk_get_rssi_response& self)
    {
        return C::insert_mip_rtk_get_rssi_response(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract_response(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_rtk_get_rssi_response& self)
    {
        return C::extract_mip_rtk_get_rssi_response(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_rtk_service_status_command>
{
    static const uint8_t descriptorSet = MIP_RTK_COMMAND_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_RTK_SERVICE_STATUS;
    static const uint8_t responseDescriptor = MIP_REPLY_DESC_RTK_SERVICE_STATUS;
    typedef C::mip_rtk_service_status_response Response;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_rtk_service_status_command& self)
    {
        return C::insert_mip_rtk_service_status_command(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_rtk_service_status_command& self)
    {
        return C::extract_mip_rtk_service_status_command(buffer, bufferSize, offset, &self);
    }
    static inline size_t insert_response(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_rtk_service_status_response& self)
    {
        return C::insert_mip_rtk_service_status_response(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract_response(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_rtk_service_status_response& self)
    {
        return C::extract_mip_rtk_service_status_response(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_rtk_prod_erase_storage_command>
{
    static const uint8_t descriptorSet = MIP_RTK_COMMAND_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_RTK_PROD_ERASE_STORAGE;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_rtk_prod_erase_storage_command& self)
    {
        return C::insert_mip_rtk_prod_erase_storage_command(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_rtk_prod_erase_storage_command& self)
    {
        return C::extract_mip_rtk_prod_erase_storage_command(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_rtk_led_control_command>
{
    static const uint8_t descriptorSet = MIP_RTK_COMMAND_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_LED_CONTROL;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_rtk_led_control_command& self)
    {
        return C::insert_mip_rtk_led_control_command(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_rtk_led_control_command& self)
    {
        return C::extract_mip_rtk_led_control_command(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_rtk_modem_hard_reset_command>
{
    static const uint8_t descriptorSet = MIP_RTK_COMMAND_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_RTK_MODEM_HARD_RESET;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_rtk_modem_hard_reset_command& self)
    {
        return C::insert_mip_rtk_modem_hard_reset_command(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_rtk_modem_hard_reset_command& self)
    {
        return C::extract_mip_rtk_modem_hard_reset_command(buffer, bufferSize, offset, &self);
    }
};



} // namespace mscl
#endif // __cplusplus
