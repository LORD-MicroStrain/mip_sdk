#pragma once

#include <mip/definitions/common.hpp>
#include <mip/mip_descriptors.hpp>
#include <mip/mip_result.hpp>
#include <mip/mip_interface.hpp>

#include <stdint.h>
#include <stddef.h>

namespace mip {
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
    struct StatusFlagsLegacy : Bitfield<StatusFlagsLegacy>
    {
        typedef uint32_t Type;
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
            ALL                  = 0xFFFFFFFF,
        };
        uint32_t value = NONE;
        
        StatusFlagsLegacy() : value(NONE) {}
        StatusFlagsLegacy(int val) : value((uint32_t)val) {}
        operator uint32_t() const { return value; }
        StatusFlagsLegacy& operator=(uint32_t val) { value = val; return *this; }
        StatusFlagsLegacy& operator=(int val) { value = uint32_t(val); return *this; }
        StatusFlagsLegacy& operator|=(uint32_t val) { return *this = value | val; }
        StatusFlagsLegacy& operator&=(uint32_t val) { return *this = value & val; }
        
        bool allSet() const { return value == ALL; }
        void setAll() { value |= ALL; }
    };
    struct StatusFlags : Bitfield<StatusFlags>
    {
        typedef uint32_t Type;
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
            CORRECTIONS_TIMEOUT     = 0x04000000,  ///<  
            DEVICE_OUT_OF_RANGE     = 0x08000000,  ///<  
            CORRECTIONS_UNAVAILABLE = 0x10000000,  ///<  
            RESERVED                = 0x20000000,  ///<  
            VERSION                 = 0xC0000000,  ///<  
            ALL                     = 0xFFFFFFFF,
        };
        uint32_t value = NONE;
        
        StatusFlags() : value(NONE) {}
        StatusFlags(int val) : value((uint32_t)val) {}
        operator uint32_t() const { return value; }
        StatusFlags& operator=(uint32_t val) { value = val; return *this; }
        StatusFlags& operator=(int val) { value = uint32_t(val); return *this; }
        StatusFlags& operator|=(uint32_t val) { return *this = value | val; }
        StatusFlags& operator&=(uint32_t val) { return *this = value & val; }
        
        bool allSet() const { return value == ALL; }
        void setAll() { value |= ALL; }
    };
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_rtk::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_rtk::CMD_GET_STATUS_FLAGS;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "GetStatusFlags";
    static constexpr const char* DOC_NAME = "Get RTK Device Status Flags";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple();
    }
    
    auto asTuple()
    {
        return std::make_tuple();
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
    struct Response
    {
        /// Parameters
        StatusFlags flags; ///< Model number dependent. See above structures.
        
        /// Descriptors
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_rtk::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_rtk::REPLY_GET_STATUS_FLAGS;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "GetStatusFlags::Response";
        static constexpr const char* DOC_NAME = "Get RTK Device Status Flags Response";
        static constexpr const bool HAS_FUNCTION_SELECTOR = false;
        
        auto asTuple() const
        {
            return std::make_tuple(flags);
        }
        
        auto asTuple()
        {
            return std::make_tuple(std::ref(flags));
        }
        
        /// Serialization
        void insert(Serializer& serializer) const;
        void extract(Serializer& serializer);
        
    };
};
TypedResult<GetStatusFlags> getStatusFlags(C::mip_interface& device, GetStatusFlags::StatusFlags* flagsOut);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_rtk_get_imei  (0x0F,0x02) Get Imei [CPP]
///
///@{

struct GetImei
{
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_rtk::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_rtk::CMD_GET_IMEI;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "GetImei";
    static constexpr const char* DOC_NAME = "Get RTK Device IMEI (International Mobile Equipment Identifier)";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple();
    }
    
    auto asTuple()
    {
        return std::make_tuple();
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
    struct Response
    {
        /// Parameters
        char IMEI[32] = {0};
        
        /// Descriptors
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_rtk::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_rtk::REPLY_GET_IMEI;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "GetImei::Response";
        static constexpr const char* DOC_NAME = "Get RTK Device IMEI (International Mobile Equipment Identifier) Response";
        static constexpr const bool HAS_FUNCTION_SELECTOR = false;
        
        auto asTuple() const
        {
            return std::make_tuple(IMEI);
        }
        
        auto asTuple()
        {
            return std::make_tuple(std::ref(IMEI));
        }
        
        /// Serialization
        void insert(Serializer& serializer) const;
        void extract(Serializer& serializer);
        
    };
};
TypedResult<GetImei> getImei(C::mip_interface& device, char* imeiOut);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_rtk_get_imsi  (0x0F,0x03) Get Imsi [CPP]
///
///@{

struct GetImsi
{
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_rtk::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_rtk::CMD_GET_IMSI;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "GetImsi";
    static constexpr const char* DOC_NAME = "Get RTK Device IMSI (International Mobile Subscriber Identifier)";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple();
    }
    
    auto asTuple()
    {
        return std::make_tuple();
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
    struct Response
    {
        /// Parameters
        char IMSI[32] = {0};
        
        /// Descriptors
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_rtk::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_rtk::REPLY_GET_IMSI;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "GetImsi::Response";
        static constexpr const char* DOC_NAME = "Get RTK Device IMSI (International Mobile Subscriber Identifier) Response";
        static constexpr const bool HAS_FUNCTION_SELECTOR = false;
        
        auto asTuple() const
        {
            return std::make_tuple(IMSI);
        }
        
        auto asTuple()
        {
            return std::make_tuple(std::ref(IMSI));
        }
        
        /// Serialization
        void insert(Serializer& serializer) const;
        void extract(Serializer& serializer);
        
    };
};
TypedResult<GetImsi> getImsi(C::mip_interface& device, char* imsiOut);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_rtk_get_iccid  (0x0F,0x04) Get Iccid [CPP]
///
///@{

struct GetIccid
{
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_rtk::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_rtk::CMD_GET_ICCID;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "GetIccid";
    static constexpr const char* DOC_NAME = "Get RTK Device ICCID (Integrated Circuit Card Identification [SIM Number])";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple();
    }
    
    auto asTuple()
    {
        return std::make_tuple();
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
    struct Response
    {
        /// Parameters
        char ICCID[32] = {0};
        
        /// Descriptors
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_rtk::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_rtk::REPLY_GET_ICCID;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "GetIccid::Response";
        static constexpr const char* DOC_NAME = "Get RTK Device ICCID (Integrated Circuit Card Identification [SIM Number]) Response";
        static constexpr const bool HAS_FUNCTION_SELECTOR = false;
        
        auto asTuple() const
        {
            return std::make_tuple(ICCID);
        }
        
        auto asTuple()
        {
            return std::make_tuple(std::ref(ICCID));
        }
        
        /// Serialization
        void insert(Serializer& serializer) const;
        void extract(Serializer& serializer);
        
    };
};
TypedResult<GetIccid> getIccid(C::mip_interface& device, char* iccidOut);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_rtk_connected_device_type  (0x0F,0x06) Connected Device Type [CPP]
///
///@{

struct ConnectedDeviceType
{
    enum class Type : uint8_t
    {
        GENERIC = 0,  ///<  
        GQ7     = 1,  ///<  
    };
    
    /// Parameters
    FunctionSelector function = static_cast<FunctionSelector>(0);
    Type devType = static_cast<Type>(0);
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_rtk::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_rtk::CMD_CONNECTED_DEVICE_TYPE;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "ConnectedDeviceType";
    static constexpr const char* DOC_NAME = "Configure or read the type of the connected device";
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    
    auto asTuple() const
    {
        return std::make_tuple(devType);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(devType));
    }
    
    static ConnectedDeviceType create_sld_all(::mip::FunctionSelector function)
    {
        ConnectedDeviceType cmd;
        cmd.function = function;
        return cmd;
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
    struct Response
    {
        /// Parameters
        Type devType = static_cast<Type>(0);
        
        /// Descriptors
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_rtk::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_rtk::REPLY_CONNECTED_DEVICE_TYPE;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "ConnectedDeviceType::Response";
        static constexpr const char* DOC_NAME = "Configure or read the type of the connected device Response";
        static constexpr const bool HAS_FUNCTION_SELECTOR = false;
        
        auto asTuple() const
        {
            return std::make_tuple(devType);
        }
        
        auto asTuple()
        {
            return std::make_tuple(std::ref(devType));
        }
        
        /// Serialization
        void insert(Serializer& serializer) const;
        void extract(Serializer& serializer);
        
    };
};
TypedResult<ConnectedDeviceType> writeConnectedDeviceType(C::mip_interface& device, ConnectedDeviceType::Type devtype);
TypedResult<ConnectedDeviceType> readConnectedDeviceType(C::mip_interface& device, ConnectedDeviceType::Type* devtypeOut);
TypedResult<ConnectedDeviceType> saveConnectedDeviceType(C::mip_interface& device);
TypedResult<ConnectedDeviceType> loadConnectedDeviceType(C::mip_interface& device);
TypedResult<ConnectedDeviceType> defaultConnectedDeviceType(C::mip_interface& device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_rtk_get_act_code  (0x0F,0x07) Get Act Code [CPP]
///
///@{

struct GetActCode
{
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_rtk::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_rtk::CMD_GET_ACT_CODE;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "GetActCode";
    static constexpr const char* DOC_NAME = "Get RTK Device Activation Code";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple();
    }
    
    auto asTuple()
    {
        return std::make_tuple();
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
    struct Response
    {
        /// Parameters
        char ActivationCode[32] = {0};
        
        /// Descriptors
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_rtk::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_rtk::REPLY_GET_ACT_CODE;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "GetActCode::Response";
        static constexpr const char* DOC_NAME = "Get RTK Device Activation Code Response";
        static constexpr const bool HAS_FUNCTION_SELECTOR = false;
        
        auto asTuple() const
        {
            return std::make_tuple(ActivationCode);
        }
        
        auto asTuple()
        {
            return std::make_tuple(std::ref(ActivationCode));
        }
        
        /// Serialization
        void insert(Serializer& serializer) const;
        void extract(Serializer& serializer);
        
    };
};
TypedResult<GetActCode> getActCode(C::mip_interface& device, char* activationcodeOut);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_rtk_get_modem_firmware_version  (0x0F,0x08) Get Modem Firmware Version [CPP]
///
///@{

struct GetModemFirmwareVersion
{
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_rtk::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_rtk::CMD_GET_MODEM_FIRMWARE_VERSION;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "GetModemFirmwareVersion";
    static constexpr const char* DOC_NAME = "Get RTK Device's Cell Modem Firmware version number";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple();
    }
    
    auto asTuple()
    {
        return std::make_tuple();
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
    struct Response
    {
        /// Parameters
        char ModemFirmwareVersion[32] = {0};
        
        /// Descriptors
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_rtk::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_rtk::REPLY_GET_MODEM_FIRMWARE_VERSION;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "GetModemFirmwareVersion::Response";
        static constexpr const char* DOC_NAME = "Get RTK Device's Cell Modem Firmware version number Response";
        static constexpr const bool HAS_FUNCTION_SELECTOR = false;
        
        auto asTuple() const
        {
            return std::make_tuple(ModemFirmwareVersion);
        }
        
        auto asTuple()
        {
            return std::make_tuple(std::ref(ModemFirmwareVersion));
        }
        
        /// Serialization
        void insert(Serializer& serializer) const;
        void extract(Serializer& serializer);
        
    };
};
TypedResult<GetModemFirmwareVersion> getModemFirmwareVersion(C::mip_interface& device, char* modemfirmwareversionOut);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_rtk_get_rssi  (0x0F,0x05) Get Rssi [CPP]
/// Get the RSSI and connected/disconnected status of modem
///
///@{

struct GetRssi
{
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_rtk::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_rtk::CMD_GET_RSSI;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "GetRssi";
    static constexpr const char* DOC_NAME = "GetRssi";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple();
    }
    
    auto asTuple()
    {
        return std::make_tuple();
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
    struct Response
    {
        /// Parameters
        bool valid = 0;
        int32_t rssi = 0;
        int32_t signalQuality = 0;
        
        /// Descriptors
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_rtk::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_rtk::REPLY_GET_RSSI;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "GetRssi::Response";
        static constexpr const char* DOC_NAME = "GetRssi Response";
        static constexpr const bool HAS_FUNCTION_SELECTOR = false;
        
        auto asTuple() const
        {
            return std::make_tuple(valid,rssi,signalQuality);
        }
        
        auto asTuple()
        {
            return std::make_tuple(std::ref(valid),std::ref(rssi),std::ref(signalQuality));
        }
        
        /// Serialization
        void insert(Serializer& serializer) const;
        void extract(Serializer& serializer);
        
    };
};
TypedResult<GetRssi> getRssi(C::mip_interface& device, bool* validOut, int32_t* rssiOut, int32_t* signalqualityOut);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_rtk_service_status  (0x0F,0x0A) Service Status [CPP]
/// The 3DMRTK will send this message to the server to indicate that the connection should remain open. The Server will respond with information and status.
///
///@{

struct ServiceStatus
{
    struct ServiceFlags : Bitfield<ServiceFlags>
    {
        typedef uint8_t Type;
        enum _enumType : uint8_t
        {
            NONE                    = 0x00,
            THROTTLE                = 0x01,  ///<  
            CORRECTIONS_UNAVAILABLE = 0x02,  ///<  
            RESERVED                = 0xFC,  ///<  
            ALL                     = 0xFF,
        };
        uint8_t value = NONE;
        
        ServiceFlags() : value(NONE) {}
        ServiceFlags(int val) : value((uint8_t)val) {}
        operator uint8_t() const { return value; }
        ServiceFlags& operator=(uint8_t val) { value = val; return *this; }
        ServiceFlags& operator=(int val) { value = uint8_t(val); return *this; }
        ServiceFlags& operator|=(uint8_t val) { return *this = value | val; }
        ServiceFlags& operator&=(uint8_t val) { return *this = value & val; }
        
        bool allSet() const { return value == ALL; }
        void setAll() { value |= ALL; }
    };
    /// Parameters
    uint32_t reserved1 = 0;
    uint32_t reserved2 = 0;
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_rtk::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_rtk::CMD_SERVICE_STATUS;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "ServiceStatus";
    static constexpr const char* DOC_NAME = "ServiceStatus";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(reserved1,reserved2);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(reserved1),std::ref(reserved2));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
    struct Response
    {
        /// Parameters
        ServiceFlags flags;
        uint32_t receivedBytes = 0;
        uint32_t lastBytes = 0;
        uint64_t lastBytesTime = 0;
        
        /// Descriptors
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_rtk::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_rtk::REPLY_SERVICE_STATUS;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "ServiceStatus::Response";
        static constexpr const char* DOC_NAME = "ServiceStatus Response";
        static constexpr const bool HAS_FUNCTION_SELECTOR = false;
        
        auto asTuple() const
        {
            return std::make_tuple(flags,receivedBytes,lastBytes,lastBytesTime);
        }
        
        auto asTuple()
        {
            return std::make_tuple(std::ref(flags),std::ref(receivedBytes),std::ref(lastBytes),std::ref(lastBytesTime));
        }
        
        /// Serialization
        void insert(Serializer& serializer) const;
        void extract(Serializer& serializer);
        
    };
};
TypedResult<ServiceStatus> serviceStatus(C::mip_interface& device, uint32_t reserved1, uint32_t reserved2, ServiceStatus::ServiceFlags* flagsOut, uint32_t* receivedbytesOut, uint32_t* lastbytesOut, uint64_t* lastbytestimeOut);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_rtk_prod_erase_storage  (0x0F,0x20) Prod Erase Storage [CPP]
/// This command will erase the selected media to a raw and uninitialized state. ALL DATA WILL BE LOST.
/// This command is only available in calibration mode.
///
///@{

struct ProdEraseStorage
{
    /// Parameters
    MediaSelector media = static_cast<MediaSelector>(0);
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_rtk::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_rtk::CMD_PROD_ERASE_STORAGE;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "ProdEraseStorage";
    static constexpr const char* DOC_NAME = "ProdEraseStorage";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(media);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(media));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
    typedef void Response;
};
TypedResult<ProdEraseStorage> prodEraseStorage(C::mip_interface& device, MediaSelector media);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_rtk_led_control  (0x0F,0x21) Led Control [CPP]
/// This command allows direct control of the LED on the 3DM RTK. This command is only available in calibration mode or Production Test Mode.
///
///@{

struct LedControl
{
    /// Parameters
    uint8_t primaryColor[3] = {0};
    uint8_t altColor[3] = {0};
    LedAction act = static_cast<LedAction>(0);
    uint32_t period = 0;
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_rtk::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_rtk::CMD_CONTROL;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "LedControl";
    static constexpr const char* DOC_NAME = "LedControl";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple(primaryColor,altColor,act,period);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(primaryColor),std::ref(altColor),std::ref(act),std::ref(period));
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
    typedef void Response;
};
TypedResult<LedControl> ledControl(C::mip_interface& device, const uint8_t* primarycolor, const uint8_t* altcolor, LedAction act, uint32_t period);

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
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_rtk::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_rtk::CMD_MODEM_HARD_RESET;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "ModemHardReset";
    static constexpr const char* DOC_NAME = "ModemHardReset";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple();
    }
    
    auto asTuple()
    {
        return std::make_tuple();
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
    typedef void Response;
};
TypedResult<ModemHardReset> modemHardReset(C::mip_interface& device);

///@}
///

///@}
///@}
///
////////////////////////////////////////////////////////////////////////////////
} // namespace commands_rtk
} // namespace mip

