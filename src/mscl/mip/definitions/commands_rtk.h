#pragma once

#include "descriptors.h"
#include "../mip_result.h"

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
namespace mscl {
extern "C" {
#endif // __cplusplus

struct MipInterfaceState;

////////////////////////////////////////////////////////////////////////////////
///@addtogroup MipCommands
///@{
///@defgroup RTK_COMMAND  RTK COMMAND
///
///@{

////////////////////////////////////////////////////////////////////////////////
// Descriptors
////////////////////////////////////////////////////////////////////////////////

enum MipRtkCommandDescriptors
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

////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////

enum MipConnectedDeviceType
{
    MIPCONNECTEDDEVICETYPE_GENERIC = 0,  ///<  
    MIPCONNECTEDDEVICETYPE_GQ7     = 1,  ///<  
};
size_t insert_MipConnectedDeviceType(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipConnectedDeviceType self);
size_t extract_MipConnectedDeviceType(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipConnectedDeviceType* self);

enum MipMediaSelector
{
    MIPMEDIASELECTOR_MEDIA_EXTERNALFLASH = 0,  ///<  
    MIPMEDIASELECTOR_MEDIA_SD            = 1,  ///<  
};
size_t insert_MipMediaSelector(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipMediaSelector self);
size_t extract_MipMediaSelector(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipMediaSelector* self);

enum MipLedAction
{
    MIPLEDACTION_LED_NONE    = 0,  ///<  
    MIPLEDACTION_LED_FLASH   = 1,  ///<  
    MIPLEDACTION_LED_PULSATE = 2,  ///<  
};
size_t insert_MipLedAction(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipLedAction self);
size_t extract_MipLedAction(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipLedAction* self);


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_cmd_rtk_get_status_flags  Rtk Get Status Flags
///
///@{

struct MipCmd_Rtk_GetStatusFlags
{
};
size_t insert_MipCmd_Rtk_GetStatusFlags(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Rtk_GetStatusFlags* self);
size_t extract_MipCmd_Rtk_GetStatusFlags(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Rtk_GetStatusFlags* self);

enum MipCmd_Rtk_GetStatusFlags_Statusflags
{
    MIPCMD_RTK_GETSTATUSFLAGS_STATUSFLAGS_MODEM_STATE             = 0x0F,
    MIPCMD_RTK_GETSTATUSFLAGS_STATUSFLAGS_CONNECTION_TYPE         = 0xF0,
    MIPCMD_RTK_GETSTATUSFLAGS_STATUSFLAGS_RSSI                    = 0xFF00,
    MIPCMD_RTK_GETSTATUSFLAGS_STATUSFLAGS_SIGNAL_QUALITY          = 0xF0000,
    MIPCMD_RTK_GETSTATUSFLAGS_STATUSFLAGS_TOWER_CHANGE_INDICATOR  = 0xF00000,
    MIPCMD_RTK_GETSTATUSFLAGS_STATUSFLAGS_NMEA_TIMEOUT            = 0x1000000,
    MIPCMD_RTK_GETSTATUSFLAGS_STATUSFLAGS_SERVER_TIMEOUT          = 0x2000000,
    MIPCMD_RTK_GETSTATUSFLAGS_STATUSFLAGS_RTCM_TIMEOUT            = 0x4000000,
    MIPCMD_RTK_GETSTATUSFLAGS_STATUSFLAGS_DEVICE_OUT_OF_RANGE     = 0x8000000,
    MIPCMD_RTK_GETSTATUSFLAGS_STATUSFLAGS_CORRECTIONS_UNAVAILABLE = 0x10000000,
    MIPCMD_RTK_GETSTATUSFLAGS_STATUSFLAGS_RESERVED                = 0x20000000,
    MIPCMD_RTK_GETSTATUSFLAGS_STATUSFLAGS_VERSION                 = 0xC0000000,
};
size_t insert_MipCmd_Rtk_GetStatusFlags_Statusflags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipCmd_Rtk_GetStatusFlags_Statusflags self);
size_t extract_MipCmd_Rtk_GetStatusFlags_Statusflags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipCmd_Rtk_GetStatusFlags_Statusflags* self);

struct MipCmd_Rtk_GetStatusFlags_Response
{
    enum MipCmd_Rtk_GetStatusFlags_Statusflags        flags;
};
size_t insert_MipCmd_Rtk_GetStatusFlags_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Rtk_GetStatusFlags_Response* self);
size_t extract_MipCmd_Rtk_GetStatusFlags_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Rtk_GetStatusFlags_Response* self);

MipCmdResult get_rtk_device_status_flags(struct MipInterfaceState* device, enum MipCmd_Rtk_GetStatusFlags_Statusflags* flags);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_cmd_rtk_get_imei  Rtk Get Imei
///
///@{

struct MipCmd_Rtk_GetImei
{
};
size_t insert_MipCmd_Rtk_GetImei(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Rtk_GetImei* self);
size_t extract_MipCmd_Rtk_GetImei(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Rtk_GetImei* self);

struct MipCmd_Rtk_GetImei_Response
{
    char                                              IMEI[32];
};
size_t insert_MipCmd_Rtk_GetImei_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Rtk_GetImei_Response* self);
size_t extract_MipCmd_Rtk_GetImei_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Rtk_GetImei_Response* self);

MipCmdResult get_rtk_device_imei_international_mobile_equipment_identifier(struct MipInterfaceState* device, char* IMEI);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_cmd_rtk_get_imsi  Rtk Get Imsi
///
///@{

struct MipCmd_Rtk_GetImsi
{
};
size_t insert_MipCmd_Rtk_GetImsi(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Rtk_GetImsi* self);
size_t extract_MipCmd_Rtk_GetImsi(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Rtk_GetImsi* self);

struct MipCmd_Rtk_GetImsi_Response
{
    char                                              IMSI[32];
};
size_t insert_MipCmd_Rtk_GetImsi_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Rtk_GetImsi_Response* self);
size_t extract_MipCmd_Rtk_GetImsi_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Rtk_GetImsi_Response* self);

MipCmdResult get_rtk_device_imsi_international_mobile_subscriber_identifier(struct MipInterfaceState* device, char* IMSI);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_cmd_rtk_get_iccid  Rtk Get Iccid
///
///@{

struct MipCmd_Rtk_GetIccid
{
};
size_t insert_MipCmd_Rtk_GetIccid(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Rtk_GetIccid* self);
size_t extract_MipCmd_Rtk_GetIccid(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Rtk_GetIccid* self);

struct MipCmd_Rtk_GetIccid_Response
{
    char                                              ICCID[32];
};
size_t insert_MipCmd_Rtk_GetIccid_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Rtk_GetIccid_Response* self);
size_t extract_MipCmd_Rtk_GetIccid_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Rtk_GetIccid_Response* self);

MipCmdResult get_rtk_device_iccid_integrated_circuit_card_identification_sim_number(struct MipInterfaceState* device, char* ICCID);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_cmd_rtk_connected_device_type  Rtk Connected Device Type
///
///@{

struct MipCmd_Rtk_ConnectedDeviceType
{
    enum MipFunctionSelector                          function;
    enum MipConnectedDeviceType                       devType;
};
size_t insert_MipCmd_Rtk_ConnectedDeviceType(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Rtk_ConnectedDeviceType* self);
size_t extract_MipCmd_Rtk_ConnectedDeviceType(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Rtk_ConnectedDeviceType* self);

struct MipCmd_Rtk_ConnectedDeviceType_Response
{
    enum MipConnectedDeviceType                       devType;
};
size_t insert_MipCmd_Rtk_ConnectedDeviceType_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Rtk_ConnectedDeviceType_Response* self);
size_t extract_MipCmd_Rtk_ConnectedDeviceType_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Rtk_ConnectedDeviceType_Response* self);

MipCmdResult write_configure_or_read_the_type_of_the_connected_device(struct MipInterfaceState* device, enum MipConnectedDeviceType devType);
MipCmdResult read_configure_or_read_the_type_of_the_connected_device(struct MipInterfaceState* device, enum MipConnectedDeviceType* devType);
MipCmdResult save_configure_or_read_the_type_of_the_connected_device(struct MipInterfaceState* device);
MipCmdResult load_configure_or_read_the_type_of_the_connected_device(struct MipInterfaceState* device);
MipCmdResult default_configure_or_read_the_type_of_the_connected_device(struct MipInterfaceState* device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_cmd_rtk_get_act_code  Rtk Get Act Code
///
///@{

struct MipCmd_Rtk_GetActCode
{
};
size_t insert_MipCmd_Rtk_GetActCode(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Rtk_GetActCode* self);
size_t extract_MipCmd_Rtk_GetActCode(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Rtk_GetActCode* self);

struct MipCmd_Rtk_GetActCode_Response
{
    char                                              ActivationCode[32];
};
size_t insert_MipCmd_Rtk_GetActCode_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Rtk_GetActCode_Response* self);
size_t extract_MipCmd_Rtk_GetActCode_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Rtk_GetActCode_Response* self);

MipCmdResult get_rtk_device_activation_code(struct MipInterfaceState* device, char* ActivationCode);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_cmd_rtk_get_modem_firmware_version  Rtk Get Modem Firmware Version
///
///@{

struct MipCmd_Rtk_GetModemFirmwareVersion
{
};
size_t insert_MipCmd_Rtk_GetModemFirmwareVersion(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Rtk_GetModemFirmwareVersion* self);
size_t extract_MipCmd_Rtk_GetModemFirmwareVersion(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Rtk_GetModemFirmwareVersion* self);

struct MipCmd_Rtk_GetModemFirmwareVersion_Response
{
    char                                              ModemFirmwareVersion[32];
};
size_t insert_MipCmd_Rtk_GetModemFirmwareVersion_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Rtk_GetModemFirmwareVersion_Response* self);
size_t extract_MipCmd_Rtk_GetModemFirmwareVersion_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Rtk_GetModemFirmwareVersion_Response* self);

MipCmdResult get_rtk_devices_cell_modem_firmware_version_number(struct MipInterfaceState* device, char* ModemFirmwareVersion);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_cmd_rtk_get_rssi  Rtk Get Rssi
/// Get the RSSI and connected/disconnected status of modem
///
///@{

struct MipCmd_Rtk_GetRssi
{
};
size_t insert_MipCmd_Rtk_GetRssi(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Rtk_GetRssi* self);
size_t extract_MipCmd_Rtk_GetRssi(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Rtk_GetRssi* self);

struct MipCmd_Rtk_GetRssi_Response
{
    bool                                              valid;
    int32_t                                           rssi;
    int32_t                                           signalQuality;
};
size_t insert_MipCmd_Rtk_GetRssi_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Rtk_GetRssi_Response* self);
size_t extract_MipCmd_Rtk_GetRssi_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Rtk_GetRssi_Response* self);

MipCmdResult mip_cmd_rtk_get_rssi(struct MipInterfaceState* device, bool* valid, int32_t* rssi, int32_t* signalQuality);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_cmd_rtk_service_status  Rtk Service Status
/// The 3DMRTK will send this message to the server to indicate that the connection should remain open. The Server will respond with information and status.
///
///@{

struct MipCmd_Rtk_ServiceStatus
{
    uint32_t                                          reserved1;
    uint32_t                                          reserved2;
};
size_t insert_MipCmd_Rtk_ServiceStatus(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Rtk_ServiceStatus* self);
size_t extract_MipCmd_Rtk_ServiceStatus(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Rtk_ServiceStatus* self);

enum MipCmd_Rtk_ServiceStatus_Serviceflags
{
    MIPCMD_RTK_SERVICESTATUS_SERVICEFLAGS_THROTTLE                = 0x01,
    MIPCMD_RTK_SERVICESTATUS_SERVICEFLAGS_CORRECTIONS_UNAVAILABLE = 0x02,
    MIPCMD_RTK_SERVICESTATUS_SERVICEFLAGS_RESERVED                = 0xFC,
};
size_t insert_MipCmd_Rtk_ServiceStatus_Serviceflags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipCmd_Rtk_ServiceStatus_Serviceflags self);
size_t extract_MipCmd_Rtk_ServiceStatus_Serviceflags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipCmd_Rtk_ServiceStatus_Serviceflags* self);

struct MipCmd_Rtk_ServiceStatus_Response
{
    enum MipCmd_Rtk_ServiceStatus_Serviceflags        flags;
    uint32_t                                          recievedBytes;
    uint32_t                                          lastBytes;
    uint64_t                                          lastBytesTime;
};
size_t insert_MipCmd_Rtk_ServiceStatus_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Rtk_ServiceStatus_Response* self);
size_t extract_MipCmd_Rtk_ServiceStatus_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Rtk_ServiceStatus_Response* self);

MipCmdResult mip_cmd_rtk_service_status(struct MipInterfaceState* device, uint32_t reserved1, uint32_t reserved2, enum MipCmd_Rtk_ServiceStatus_Serviceflags* flags, uint32_t* recievedBytes, uint32_t* lastBytes, uint64_t* lastBytesTime);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_cmd_rtk_prod_erase_storage  Rtk Prod Erase Storage
/// This command will erase the selected media to a raw and unitialized state. ALL DATA WILL BE LOST.
/// This command is only available in calibration mode.
///
///@{

struct MipCmd_Rtk_ProdEraseStorage
{
    enum MipMediaSelector                             media;
};
size_t insert_MipCmd_Rtk_ProdEraseStorage(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Rtk_ProdEraseStorage* self);
size_t extract_MipCmd_Rtk_ProdEraseStorage(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Rtk_ProdEraseStorage* self);

MipCmdResult mip_cmd_rtk_prod_erase_storage(struct MipInterfaceState* device, enum MipMediaSelector media);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_cmd_rtk_led_control  Rtk Led Control
/// This command allows direct control of the LED on the 3DM RTK. This command is only available in calibration mode or Production Test Mode.
///
///@{

struct MipCmd_Rtk_LedControl
{
    uint8_t                                           primaryColor[3];
    uint8_t                                           altColor[3];
    enum MipLedAction                                 act;
    uint32_t                                          period;
};
size_t insert_MipCmd_Rtk_LedControl(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Rtk_LedControl* self);
size_t extract_MipCmd_Rtk_LedControl(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Rtk_LedControl* self);

MipCmdResult mip_cmd_rtk_led_control(struct MipInterfaceState* device, const uint8_t* primaryColor, const uint8_t* altColor, enum MipLedAction act, uint32_t period);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_cmd_rtk_modem_hard_reset  Rtk Modem Hard Reset
/// This command will clear the modem flash.  THIS MUST NOT BE DONE OFTEN AS IT CAN DAMAGE THE FLASH!
/// This command is only available in calibration mode.
///
///@{

struct MipCmd_Rtk_ModemHardReset
{
};
size_t insert_MipCmd_Rtk_ModemHardReset(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Rtk_ModemHardReset* self);
size_t extract_MipCmd_Rtk_ModemHardReset(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Rtk_ModemHardReset* self);

MipCmdResult mip_cmd_rtk_modem_hard_reset(struct MipInterfaceState* device);
///@}
///

///@}
///@}
///
////////////////////////////////////////////////////////////////////////////////

#ifdef __cplusplus
} // extern "C"


template<>
struct MipFieldInfo<MipCmd_Rtk_GetStatusFlags>
{
    static const uint8_t descriptorSet = MIP_RTK_COMMAND_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_RTK_GET_STATUS_FLAGS;
    static const uint8_t responseDescriptor = MIP_REPLY_DESC_RTK_GET_STATUS_FLAGS;
    typedef MipCmd_Rtk_GetStatusFlags_Response Response;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipCmd_Rtk_GetStatusFlags& self)
    {
        return insert_MipCmd_Rtk_GetStatusFlags(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipCmd_Rtk_GetStatusFlags& self)
    {
        return extract_MipCmd_Rtk_GetStatusFlags(buffer, bufferSize, offset, &self);
    }
    static inline size_t insert_response(uint8_t* buffer, size_t bufferSize, size_t offset, const MipCmd_Rtk_GetStatusFlags_Response& self)
    {
        return insert_MipCmd_Rtk_GetStatusFlags_Response(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract_response(const uint8_t* buffer, size_t bufferSize, size_t offset, MipCmd_Rtk_GetStatusFlags_Response& self)
    {
        return extract_MipCmd_Rtk_GetStatusFlags_Response(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipCmd_Rtk_GetImei>
{
    static const uint8_t descriptorSet = MIP_RTK_COMMAND_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_RTK_GET_IMEI;
    static const uint8_t responseDescriptor = MIP_REPLY_DESC_RTK_GET_IMEI;
    typedef MipCmd_Rtk_GetImei_Response Response;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipCmd_Rtk_GetImei& self)
    {
        return insert_MipCmd_Rtk_GetImei(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipCmd_Rtk_GetImei& self)
    {
        return extract_MipCmd_Rtk_GetImei(buffer, bufferSize, offset, &self);
    }
    static inline size_t insert_response(uint8_t* buffer, size_t bufferSize, size_t offset, const MipCmd_Rtk_GetImei_Response& self)
    {
        return insert_MipCmd_Rtk_GetImei_Response(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract_response(const uint8_t* buffer, size_t bufferSize, size_t offset, MipCmd_Rtk_GetImei_Response& self)
    {
        return extract_MipCmd_Rtk_GetImei_Response(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipCmd_Rtk_GetImsi>
{
    static const uint8_t descriptorSet = MIP_RTK_COMMAND_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_RTK_GET_IMSI;
    static const uint8_t responseDescriptor = MIP_REPLY_DESC_RTK_GET_IMSI;
    typedef MipCmd_Rtk_GetImsi_Response Response;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipCmd_Rtk_GetImsi& self)
    {
        return insert_MipCmd_Rtk_GetImsi(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipCmd_Rtk_GetImsi& self)
    {
        return extract_MipCmd_Rtk_GetImsi(buffer, bufferSize, offset, &self);
    }
    static inline size_t insert_response(uint8_t* buffer, size_t bufferSize, size_t offset, const MipCmd_Rtk_GetImsi_Response& self)
    {
        return insert_MipCmd_Rtk_GetImsi_Response(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract_response(const uint8_t* buffer, size_t bufferSize, size_t offset, MipCmd_Rtk_GetImsi_Response& self)
    {
        return extract_MipCmd_Rtk_GetImsi_Response(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipCmd_Rtk_GetIccid>
{
    static const uint8_t descriptorSet = MIP_RTK_COMMAND_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_RTK_GET_ICCID;
    static const uint8_t responseDescriptor = MIP_REPLY_DESC_RTK_GET_ICCID;
    typedef MipCmd_Rtk_GetIccid_Response Response;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipCmd_Rtk_GetIccid& self)
    {
        return insert_MipCmd_Rtk_GetIccid(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipCmd_Rtk_GetIccid& self)
    {
        return extract_MipCmd_Rtk_GetIccid(buffer, bufferSize, offset, &self);
    }
    static inline size_t insert_response(uint8_t* buffer, size_t bufferSize, size_t offset, const MipCmd_Rtk_GetIccid_Response& self)
    {
        return insert_MipCmd_Rtk_GetIccid_Response(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract_response(const uint8_t* buffer, size_t bufferSize, size_t offset, MipCmd_Rtk_GetIccid_Response& self)
    {
        return extract_MipCmd_Rtk_GetIccid_Response(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipCmd_Rtk_ConnectedDeviceType>
{
    static const uint8_t descriptorSet = MIP_RTK_COMMAND_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_RTK_CONNECTED_DEVICE_TYPE;
    static const uint8_t responseDescriptor = MIP_REPLY_DESC_RTK_CONNECTED_DEVICE_TYPE;
    typedef MipCmd_Rtk_ConnectedDeviceType_Response Response;
    
    static const bool hasFunctionSelector = true;
    static const bool canWrite = true;
    static const bool canRead = true;
    static const bool canSave = true;
    static const bool canLoad = true;
    static const bool canReset = true;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipCmd_Rtk_ConnectedDeviceType& self)
    {
        return insert_MipCmd_Rtk_ConnectedDeviceType(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipCmd_Rtk_ConnectedDeviceType& self)
    {
        return extract_MipCmd_Rtk_ConnectedDeviceType(buffer, bufferSize, offset, &self);
    }
    static inline size_t insert_response(uint8_t* buffer, size_t bufferSize, size_t offset, const MipCmd_Rtk_ConnectedDeviceType_Response& self)
    {
        return insert_MipCmd_Rtk_ConnectedDeviceType_Response(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract_response(const uint8_t* buffer, size_t bufferSize, size_t offset, MipCmd_Rtk_ConnectedDeviceType_Response& self)
    {
        return extract_MipCmd_Rtk_ConnectedDeviceType_Response(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipCmd_Rtk_GetActCode>
{
    static const uint8_t descriptorSet = MIP_RTK_COMMAND_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_RTK_GET_ACT_CODE;
    static const uint8_t responseDescriptor = MIP_REPLY_DESC_RTK_GET_ACT_CODE;
    typedef MipCmd_Rtk_GetActCode_Response Response;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipCmd_Rtk_GetActCode& self)
    {
        return insert_MipCmd_Rtk_GetActCode(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipCmd_Rtk_GetActCode& self)
    {
        return extract_MipCmd_Rtk_GetActCode(buffer, bufferSize, offset, &self);
    }
    static inline size_t insert_response(uint8_t* buffer, size_t bufferSize, size_t offset, const MipCmd_Rtk_GetActCode_Response& self)
    {
        return insert_MipCmd_Rtk_GetActCode_Response(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract_response(const uint8_t* buffer, size_t bufferSize, size_t offset, MipCmd_Rtk_GetActCode_Response& self)
    {
        return extract_MipCmd_Rtk_GetActCode_Response(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipCmd_Rtk_GetModemFirmwareVersion>
{
    static const uint8_t descriptorSet = MIP_RTK_COMMAND_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_RTK_GET_MODEM_FIRMWARE_VERSION;
    static const uint8_t responseDescriptor = MIP_REPLY_DESC_RTK_GET_MODEM_FIRMWARE_VERSION;
    typedef MipCmd_Rtk_GetModemFirmwareVersion_Response Response;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipCmd_Rtk_GetModemFirmwareVersion& self)
    {
        return insert_MipCmd_Rtk_GetModemFirmwareVersion(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipCmd_Rtk_GetModemFirmwareVersion& self)
    {
        return extract_MipCmd_Rtk_GetModemFirmwareVersion(buffer, bufferSize, offset, &self);
    }
    static inline size_t insert_response(uint8_t* buffer, size_t bufferSize, size_t offset, const MipCmd_Rtk_GetModemFirmwareVersion_Response& self)
    {
        return insert_MipCmd_Rtk_GetModemFirmwareVersion_Response(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract_response(const uint8_t* buffer, size_t bufferSize, size_t offset, MipCmd_Rtk_GetModemFirmwareVersion_Response& self)
    {
        return extract_MipCmd_Rtk_GetModemFirmwareVersion_Response(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipCmd_Rtk_GetRssi>
{
    static const uint8_t descriptorSet = MIP_RTK_COMMAND_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_RTK_GET_RSSI;
    static const uint8_t responseDescriptor = MIP_REPLY_DESC_RTK_GET_RSSI;
    typedef MipCmd_Rtk_GetRssi_Response Response;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipCmd_Rtk_GetRssi& self)
    {
        return insert_MipCmd_Rtk_GetRssi(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipCmd_Rtk_GetRssi& self)
    {
        return extract_MipCmd_Rtk_GetRssi(buffer, bufferSize, offset, &self);
    }
    static inline size_t insert_response(uint8_t* buffer, size_t bufferSize, size_t offset, const MipCmd_Rtk_GetRssi_Response& self)
    {
        return insert_MipCmd_Rtk_GetRssi_Response(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract_response(const uint8_t* buffer, size_t bufferSize, size_t offset, MipCmd_Rtk_GetRssi_Response& self)
    {
        return extract_MipCmd_Rtk_GetRssi_Response(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipCmd_Rtk_ServiceStatus>
{
    static const uint8_t descriptorSet = MIP_RTK_COMMAND_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_RTK_SERVICE_STATUS;
    static const uint8_t responseDescriptor = MIP_REPLY_DESC_RTK_SERVICE_STATUS;
    typedef MipCmd_Rtk_ServiceStatus_Response Response;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipCmd_Rtk_ServiceStatus& self)
    {
        return insert_MipCmd_Rtk_ServiceStatus(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipCmd_Rtk_ServiceStatus& self)
    {
        return extract_MipCmd_Rtk_ServiceStatus(buffer, bufferSize, offset, &self);
    }
    static inline size_t insert_response(uint8_t* buffer, size_t bufferSize, size_t offset, const MipCmd_Rtk_ServiceStatus_Response& self)
    {
        return insert_MipCmd_Rtk_ServiceStatus_Response(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract_response(const uint8_t* buffer, size_t bufferSize, size_t offset, MipCmd_Rtk_ServiceStatus_Response& self)
    {
        return extract_MipCmd_Rtk_ServiceStatus_Response(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipCmd_Rtk_ProdEraseStorage>
{
    static const uint8_t descriptorSet = MIP_RTK_COMMAND_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_RTK_PROD_ERASE_STORAGE;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipCmd_Rtk_ProdEraseStorage& self)
    {
        return insert_MipCmd_Rtk_ProdEraseStorage(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipCmd_Rtk_ProdEraseStorage& self)
    {
        return extract_MipCmd_Rtk_ProdEraseStorage(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipCmd_Rtk_LedControl>
{
    static const uint8_t descriptorSet = MIP_RTK_COMMAND_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_LED_CONTROL;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipCmd_Rtk_LedControl& self)
    {
        return insert_MipCmd_Rtk_LedControl(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipCmd_Rtk_LedControl& self)
    {
        return extract_MipCmd_Rtk_LedControl(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipCmd_Rtk_ModemHardReset>
{
    static const uint8_t descriptorSet = MIP_RTK_COMMAND_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_RTK_MODEM_HARD_RESET;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipCmd_Rtk_ModemHardReset& self)
    {
        return insert_MipCmd_Rtk_ModemHardReset(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipCmd_Rtk_ModemHardReset& self)
    {
        return extract_MipCmd_Rtk_ModemHardReset(buffer, bufferSize, offset, &self);
    }
};



} // namespace mscl
#endif // __cplusplus
