#pragma once

#include "descriptors.h"
#include "../mip_result.h"

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

namespace mip {
class Serializer;

namespace C {
struct mip_interface;
} // namespace C

namespace commands_rtk {

////////////////////////////////////////////////////////////////////////////////
///@addtogroup MipCommands_cpp  MIP Commands [CPP]
///@{
///@defgroup rtk_commands_cpp  Rtk Commands [CPP]
///
///@{

////////////////////////////////////////////////////////////////////////////////
// Descriptors
////////////////////////////////////////////////////////////////////////////////

enum 
{
    DESCRIPTOR_SET                   = 0x0F,
    
    CMD_GET_STATUS_FLAGS             = 0x01,
    CMD_GET_IMEI                     = 0x02,
    CMD_GET_IMSI                     = 0x03,
    CMD_GET_ICCID                    = 0x04,
    CMD_GET_RSSI                     = 0x05,
    CMD_CONNECTED_DEVICE_TYPE        = 0x06,
    CMD_GET_ACT_CODE                 = 0x07,
    CMD_GET_MODEM_FIRMWARE_VERSION   = 0x08,
    CMD_SERVICE_STATUS               = 0x0A,
    CMD_PROD_ERASE_STORAGE           = 0x20,
    CMD_CONTROL                      = 0x21,
    CMD_MODEM_HARD_RESET             = 0x22,
    
    REPLY_GET_STATUS_FLAGS           = 0x81,
    REPLY_GET_IMEI                   = 0x82,
    REPLY_GET_IMSI                   = 0x83,
    REPLY_GET_ICCID                  = 0x84,
    REPLY_CONNECTED_DEVICE_TYPE      = 0x86,
    REPLY_GET_ACT_CODE               = 0x87,
    REPLY_GET_MODEM_FIRMWARE_VERSION = 0x88,
    REPLY_GET_RSSI                   = 0x85,
    REPLY_SERVICE_STATUS             = 0x8A,
};

////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////

enum class MediaSelector : uint8_t
{
    MEDIA_EXTERNALFLASH = 0,  ///<  
    MEDIA_SD            = 1,  ///<  
};

enum class LedAction : uint8_t
{
    LED_NONE    = 0,  ///<  
    LED_FLASH   = 1,  ///<  
    LED_PULSATE = 2,  ///<  
};


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_rtk_get_status_flags  (0x0F,0x01) Get Status Flags [CPP]
///
///@{

struct GetStatusFlags
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_rtk::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_rtk::CMD_GET_STATUS_FLAGS;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    struct StatusFlagsLegacy : Bitfield<StatusFlagsLegacy>
    {
        enum _enumType : uint32_t
        {
            NONE                 = 0x00000000,
            CONTROLLERSTATE      = 0x00000007,  ///<  
            PLATFORMSTATE        = 0x000000F8,  ///<  
            CONTROLLERSTATUSCODE = 0x00000700,  ///<  
            PLATFORMSTATUSCODE   = 0x00003800,  ///<  
            RESETCODE            = 0x0000C000,  ///<  
            SIGNALQUALITY        = 0x000F0000,  ///<  
            RESERVED             = 0xFFF00000,  ///<  
            RSSI                 = 0x03F00000,  ///<  
            RSRP                 = 0x0C000000,  ///<  
            RSRQ                 = 0x30000000,  ///<  
            SINR                 = 0xC0000000,  ///<  
        };
        uint32_t value = NONE;
        
        StatusFlagsLegacy() : value(NONE) {}
        StatusFlagsLegacy(int val) : value((uint32_t)val) {}
        operator uint32_t() const { return value; }
        StatusFlagsLegacy& operator=(uint32_t val) { value = val; return *this; }
        StatusFlagsLegacy& operator=(int val) { value = val; return *this; }
        StatusFlagsLegacy& operator|=(uint32_t val) { return *this = value | val; }
        StatusFlagsLegacy& operator&=(uint32_t val) { return *this = value & val; }
    };
    
    struct StatusFlags : Bitfield<StatusFlags>
    {
        enum _enumType : uint32_t
        {
            NONE                    = 0x00000000,
            MODEM_STATE             = 0x0000000F,  ///<  
            CONNECTION_TYPE         = 0x000000F0,  ///<  
            RSSI                    = 0x0000FF00,  ///<  
            SIGNAL_QUALITY          = 0x000F0000,  ///<  
            TOWER_CHANGE_INDICATOR  = 0x00F00000,  ///<  
            NMEA_TIMEOUT            = 0x01000000,  ///<  
            SERVER_TIMEOUT          = 0x02000000,  ///<  
            RTCM_TIMEOUT            = 0x04000000,  ///<  
            DEVICE_OUT_OF_RANGE     = 0x08000000,  ///<  
            CORRECTIONS_UNAVAILABLE = 0x10000000,  ///<  
            RESERVED                = 0x20000000,  ///<  
            VERSION                 = 0xC0000000,  ///<  
        };
        uint32_t value = NONE;
        
        StatusFlags() : value(NONE) {}
        StatusFlags(int val) : value((uint32_t)val) {}
        operator uint32_t() const { return value; }
        StatusFlags& operator=(uint32_t val) { value = val; return *this; }
        StatusFlags& operator=(int val) { value = val; return *this; }
        StatusFlags& operator|=(uint32_t val) { return *this = value | val; }
        StatusFlags& operator&=(uint32_t val) { return *this = value & val; }
    };
    
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_rtk::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_rtk::REPLY_GET_STATUS_FLAGS;
        
        StatusFlags flags; ///< Model number dependent. See above structures.
        
    };
};
void insert(Serializer& serializer, const GetStatusFlags& self);
void extract(Serializer& serializer, GetStatusFlags& self);

void insert(Serializer& serializer, const GetStatusFlags::Response& self);
void extract(Serializer& serializer, GetStatusFlags::Response& self);

CmdResult getStatusFlags(C::mip_interface& device, GetStatusFlags::StatusFlags* flagsOut);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_rtk_get_imei  (0x0F,0x02) Get Imei [CPP]
///
///@{

struct GetImei
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_rtk::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_rtk::CMD_GET_IMEI;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_rtk::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_rtk::REPLY_GET_IMEI;
        
        char IMEI[32] = {0};
        
    };
};
void insert(Serializer& serializer, const GetImei& self);
void extract(Serializer& serializer, GetImei& self);

void insert(Serializer& serializer, const GetImei::Response& self);
void extract(Serializer& serializer, GetImei::Response& self);

CmdResult getImei(C::mip_interface& device, char* imeiOut);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_rtk_get_imsi  (0x0F,0x03) Get Imsi [CPP]
///
///@{

struct GetImsi
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_rtk::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_rtk::CMD_GET_IMSI;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_rtk::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_rtk::REPLY_GET_IMSI;
        
        char IMSI[32] = {0};
        
    };
};
void insert(Serializer& serializer, const GetImsi& self);
void extract(Serializer& serializer, GetImsi& self);

void insert(Serializer& serializer, const GetImsi::Response& self);
void extract(Serializer& serializer, GetImsi::Response& self);

CmdResult getImsi(C::mip_interface& device, char* imsiOut);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_rtk_get_iccid  (0x0F,0x04) Get Iccid [CPP]
///
///@{

struct GetIccid
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_rtk::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_rtk::CMD_GET_ICCID;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_rtk::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_rtk::REPLY_GET_ICCID;
        
        char ICCID[32] = {0};
        
    };
};
void insert(Serializer& serializer, const GetIccid& self);
void extract(Serializer& serializer, GetIccid& self);

void insert(Serializer& serializer, const GetIccid::Response& self);
void extract(Serializer& serializer, GetIccid::Response& self);

CmdResult getIccid(C::mip_interface& device, char* iccidOut);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_rtk_connected_device_type  (0x0F,0x06) Connected Device Type [CPP]
///
///@{

struct ConnectedDeviceType
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_rtk::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_rtk::CMD_CONNECTED_DEVICE_TYPE;
    
    static const bool HAS_WRITE_FUNCTION = true;
    static const bool HAS_READ_FUNCTION = true;
    static const bool HAS_SAVE_FUNCTION = true;
    static const bool HAS_LOAD_FUNCTION = true;
    static const bool HAS_RESET_FUNCTION = true;
    
    enum class Type : uint8_t
    {
        GENERIC = 0,  ///<  
        GQ7     = 1,  ///<  
    };
    
    FunctionSelector function = static_cast<FunctionSelector>(0);
    Type devType = static_cast<Type>(0);
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_rtk::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_rtk::REPLY_CONNECTED_DEVICE_TYPE;
        
        Type devType = static_cast<Type>(0);
        
    };
};
void insert(Serializer& serializer, const ConnectedDeviceType& self);
void extract(Serializer& serializer, ConnectedDeviceType& self);

void insert(Serializer& serializer, const ConnectedDeviceType::Response& self);
void extract(Serializer& serializer, ConnectedDeviceType::Response& self);

CmdResult writeConnectedDeviceType(C::mip_interface& device, ConnectedDeviceType::Type devtype);
CmdResult readConnectedDeviceType(C::mip_interface& device, ConnectedDeviceType::Type* devtypeOut);
CmdResult saveConnectedDeviceType(C::mip_interface& device);
CmdResult loadConnectedDeviceType(C::mip_interface& device);
CmdResult defaultConnectedDeviceType(C::mip_interface& device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_rtk_get_act_code  (0x0F,0x07) Get Act Code [CPP]
///
///@{

struct GetActCode
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_rtk::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_rtk::CMD_GET_ACT_CODE;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_rtk::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_rtk::REPLY_GET_ACT_CODE;
        
        char ActivationCode[32] = {0};
        
    };
};
void insert(Serializer& serializer, const GetActCode& self);
void extract(Serializer& serializer, GetActCode& self);

void insert(Serializer& serializer, const GetActCode::Response& self);
void extract(Serializer& serializer, GetActCode::Response& self);

CmdResult getActCode(C::mip_interface& device, char* activationcodeOut);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_rtk_get_modem_firmware_version  (0x0F,0x08) Get Modem Firmware Version [CPP]
///
///@{

struct GetModemFirmwareVersion
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_rtk::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_rtk::CMD_GET_MODEM_FIRMWARE_VERSION;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_rtk::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_rtk::REPLY_GET_MODEM_FIRMWARE_VERSION;
        
        char ModemFirmwareVersion[32] = {0};
        
    };
};
void insert(Serializer& serializer, const GetModemFirmwareVersion& self);
void extract(Serializer& serializer, GetModemFirmwareVersion& self);

void insert(Serializer& serializer, const GetModemFirmwareVersion::Response& self);
void extract(Serializer& serializer, GetModemFirmwareVersion::Response& self);

CmdResult getModemFirmwareVersion(C::mip_interface& device, char* modemfirmwareversionOut);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_rtk_get_rssi  (0x0F,0x05) Get Rssi [CPP]
/// Get the RSSI and connected/disconnected status of modem
///
///@{

struct GetRssi
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_rtk::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_rtk::CMD_GET_RSSI;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_rtk::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_rtk::REPLY_GET_RSSI;
        
        bool valid = 0;
        int32_t rssi = 0;
        int32_t signalQuality = 0;
        
    };
};
void insert(Serializer& serializer, const GetRssi& self);
void extract(Serializer& serializer, GetRssi& self);

void insert(Serializer& serializer, const GetRssi::Response& self);
void extract(Serializer& serializer, GetRssi::Response& self);

CmdResult getRssi(C::mip_interface& device, bool* validOut, int32_t* rssiOut, int32_t* signalqualityOut);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_rtk_service_status  (0x0F,0x0A) Service Status [CPP]
/// The 3DMRTK will send this message to the server to indicate that the connection should remain open. The Server will respond with information and status.
///
///@{

struct ServiceStatus
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_rtk::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_rtk::CMD_SERVICE_STATUS;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    struct ServiceFlags : Bitfield<ServiceFlags>
    {
        enum _enumType : uint8_t
        {
            NONE                    = 0x00,
            THROTTLE                = 0x01,  ///<  
            CORRECTIONS_UNAVAILABLE = 0x02,  ///<  
            RESERVED                = 0xFC,  ///<  
        };
        uint8_t value = NONE;
        
        ServiceFlags() : value(NONE) {}
        ServiceFlags(int val) : value((uint8_t)val) {}
        operator uint8_t() const { return value; }
        ServiceFlags& operator=(uint8_t val) { value = val; return *this; }
        ServiceFlags& operator=(int val) { value = val; return *this; }
        ServiceFlags& operator|=(uint8_t val) { return *this = value | val; }
        ServiceFlags& operator&=(uint8_t val) { return *this = value & val; }
    };
    
    uint32_t reserved1 = 0;
    uint32_t reserved2 = 0;
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_rtk::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_rtk::REPLY_SERVICE_STATUS;
        
        ServiceFlags flags;
        uint32_t recievedBytes = 0;
        uint32_t lastBytes = 0;
        uint64_t lastBytesTime = 0;
        
    };
};
void insert(Serializer& serializer, const ServiceStatus& self);
void extract(Serializer& serializer, ServiceStatus& self);

void insert(Serializer& serializer, const ServiceStatus::Response& self);
void extract(Serializer& serializer, ServiceStatus::Response& self);

CmdResult serviceStatus(C::mip_interface& device, uint32_t reserved1, uint32_t reserved2, ServiceStatus::ServiceFlags* flagsOut, uint32_t* recievedbytesOut, uint32_t* lastbytesOut, uint64_t* lastbytestimeOut);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_rtk_prod_erase_storage  (0x0F,0x20) Prod Erase Storage [CPP]
/// This command will erase the selected media to a raw and unitialized state. ALL DATA WILL BE LOST.
/// This command is only available in calibration mode.
///
///@{

struct ProdEraseStorage
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_rtk::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_rtk::CMD_PROD_ERASE_STORAGE;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    MediaSelector media = static_cast<MediaSelector>(0);
    
};
void insert(Serializer& serializer, const ProdEraseStorage& self);
void extract(Serializer& serializer, ProdEraseStorage& self);

CmdResult prodEraseStorage(C::mip_interface& device, MediaSelector media);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_rtk_led_control  (0x0F,0x21) Led Control [CPP]
/// This command allows direct control of the LED on the 3DM RTK. This command is only available in calibration mode or Production Test Mode.
///
///@{

struct LedControl
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_rtk::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_rtk::CMD_CONTROL;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    uint8_t primaryColor[3] = {0};
    uint8_t altColor[3] = {0};
    LedAction act = static_cast<LedAction>(0);
    uint32_t period = 0;
    
};
void insert(Serializer& serializer, const LedControl& self);
void extract(Serializer& serializer, LedControl& self);

CmdResult ledControl(C::mip_interface& device, const uint8_t* primarycolor, const uint8_t* altcolor, LedAction act, uint32_t period);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_rtk_modem_hard_reset  (0x0F,0x22) Modem Hard Reset [CPP]
/// This command will clear the modem flash.  THIS MUST NOT BE DONE OFTEN AS IT CAN DAMAGE THE FLASH!
/// This command is only available in calibration mode.
///
///@{

struct ModemHardReset
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_rtk::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_rtk::CMD_MODEM_HARD_RESET;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    
};
void insert(Serializer& serializer, const ModemHardReset& self);
void extract(Serializer& serializer, ModemHardReset& self);

CmdResult modemHardReset(C::mip_interface& device);
///@}
///

///@}
///@}
///
////////////////////////////////////////////////////////////////////////////////
} // namespace commands_rtk
} // namespace mip

