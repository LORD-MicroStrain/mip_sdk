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

namespace commands_gnss {

////////////////////////////////////////////////////////////////////////////////
///@addtogroup MipCommands_cpp
///@{
///@defgroup gnss_commands_cpp_cpp  Gnss Commands_cpp [CPP]
///
///@{

////////////////////////////////////////////////////////////////////////////////
// Descriptors
////////////////////////////////////////////////////////////////////////////////

enum 
{
    DESCRIPTOR_SET                 = 0x0E,
    
    CMD_LIST_RECEIVERS             = 0x01,
    CMD_SIGNAL_CONFIGURATION       = 0x02,
    CMD_RTK_DONGLE_CONFIGURATION   = 0x10,
    CMD_RECEIVER_SAFE_MODE         = 0x60,
    
    REPLY_LIST_RECEIVERS           = 0x81,
    REPLY_SIGNAL_CONFIGURATION     = 0x82,
    REPLY_RTK_DONGLE_CONFIGURATION = 0x90,
};

////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////

static const uint16_t GNSS_GPS_ENABLE_L1CA = 0x0001;
static const uint16_t GNSS_GPS_ENABLE_L2C = 0x0002;
static const uint16_t GNSS_GLONASS_ENABLE_L1OF = 0x0001;
static const uint16_t GNSS_GLONASS_ENABLE_L2OF = 0x0002;
static const uint16_t GNSS_GALILEO_ENABLE_E1 = 0x0001;
static const uint16_t GNSS_GALILEO_ENABLE_E5B = 0x0002;
static const uint16_t GNSS_BEIDOU_ENABLE_B1 = 0x0001;
static const uint16_t GNSS_BEIDOU_ENABLE_B2 = 0x0002;

////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_gnss_receiver_info  Receiver Info
/// Return information about the GNSS receivers in the device.
/// 
///
///@{

struct ReceiverInfo
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_gnss::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_gnss::CMD_LIST_RECEIVERS;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    struct Info
    {
        uint8_t receiver_id = 0; ///< Receiver id: e.g. 1, 2, etc.
        uint8_t mip_data_descriptor_set = 0; ///< MIP descriptor set associated with this receiver
        char description[32] = {0}; ///< Ascii description of receiver
        
    };
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_gnss::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_gnss::REPLY_LIST_RECEIVERS;
        
        uint8_t num_receivers = 0; ///< Number of physical receivers in the device
        Info* receiver_info = {nullptr};
        
    };
};
void insert(Serializer& serializer, const ReceiverInfo& self);
void extract(Serializer& serializer, ReceiverInfo& self);

void insert(Serializer& serializer, const ReceiverInfo::Info& self);
void extract(Serializer& serializer, ReceiverInfo::Info& self);

void insert(Serializer& serializer, const ReceiverInfo::Response& self);
void extract(Serializer& serializer, ReceiverInfo::Response& self);

CmdResult receiverInfo(C::mip_interface& device, uint8_t* numReceiversOut, uint8_t numReceiversOutMax, ReceiverInfo::Info* receiverInfoOut);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_gnss_signal_configuration  Signal Configuration
/// Configure the GNSS signals used by the device.
/// 
///
///@{

struct SignalConfiguration
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_gnss::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_gnss::CMD_SIGNAL_CONFIGURATION;
    
    static const bool HAS_WRITE_FUNCTION = true;
    static const bool HAS_READ_FUNCTION = true;
    static const bool HAS_SAVE_FUNCTION = true;
    static const bool HAS_LOAD_FUNCTION = true;
    static const bool HAS_RESET_FUNCTION = true;
    
    FunctionSelector function = static_cast<FunctionSelector>(0);
    uint8_t gps_enable = 0; ///< Bitfield 0: Enable L1CA, 1: Enable L2C
    uint8_t glonass_enable = 0; ///< Bitfield 0: Enable L1OF, 1: Enable L2OF
    uint8_t galileo_enable = 0; ///< Bitfield 0: Enable E1,   1: Enable E5B
    uint8_t beidou_enable = 0; ///< Bitfield 0: Enable B1,   1: Enable B2
    uint8_t reserved[4] = {0};
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_gnss::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_gnss::REPLY_SIGNAL_CONFIGURATION;
        
        uint8_t gps_enable = 0; ///< Bitfield 0: Enable L1CA, 1: Enable L2C
        uint8_t glonass_enable = 0; ///< Bitfield 0: Enable L1OF, 1: Enable L2OF
        uint8_t galileo_enable = 0; ///< Bitfield 0: Enable E1,   1: Enable E5B
        uint8_t beidou_enable = 0; ///< Bitfield 0: Enable B1,   1: Enable B2
        uint8_t reserved[4] = {0};
        
    };
};
void insert(Serializer& serializer, const SignalConfiguration& self);
void extract(Serializer& serializer, SignalConfiguration& self);

void insert(Serializer& serializer, const SignalConfiguration::Response& self);
void extract(Serializer& serializer, SignalConfiguration::Response& self);

CmdResult writeSignalConfiguration(C::mip_interface& device, uint8_t gpsEnable, uint8_t glonassEnable, uint8_t galileoEnable, uint8_t beidouEnable, const uint8_t* reserved);
CmdResult readSignalConfiguration(C::mip_interface& device, uint8_t* gpsEnableOut, uint8_t* glonassEnableOut, uint8_t* galileoEnableOut, uint8_t* beidouEnableOut, uint8_t* reservedOut);
CmdResult saveSignalConfiguration(C::mip_interface& device);
CmdResult loadSignalConfiguration(C::mip_interface& device);
CmdResult defaultSignalConfiguration(C::mip_interface& device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_gnss_rtk_dongle_configuration  Rtk Dongle Configuration
/// Configure the communications with the RTK Dongle connected to the device.
/// 
///
///@{

struct RtkDongleConfiguration
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_gnss::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_gnss::CMD_RTK_DONGLE_CONFIGURATION;
    
    static const bool HAS_WRITE_FUNCTION = true;
    static const bool HAS_READ_FUNCTION = true;
    static const bool HAS_SAVE_FUNCTION = true;
    static const bool HAS_LOAD_FUNCTION = true;
    static const bool HAS_RESET_FUNCTION = true;
    
    FunctionSelector function = static_cast<FunctionSelector>(0);
    uint8_t enable = 0; ///< 0 - Disabled, 1- Enabled
    uint8_t reserved[3] = {0};
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_gnss::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_gnss::REPLY_RTK_DONGLE_CONFIGURATION;
        
        uint8_t enable = 0;
        uint8_t reserved[3] = {0};
        
    };
};
void insert(Serializer& serializer, const RtkDongleConfiguration& self);
void extract(Serializer& serializer, RtkDongleConfiguration& self);

void insert(Serializer& serializer, const RtkDongleConfiguration::Response& self);
void extract(Serializer& serializer, RtkDongleConfiguration::Response& self);

CmdResult writeRtkDongleConfiguration(C::mip_interface& device, uint8_t enable, const uint8_t* reserved);
CmdResult readRtkDongleConfiguration(C::mip_interface& device, uint8_t* enableOut, uint8_t* reservedOut);
CmdResult saveRtkDongleConfiguration(C::mip_interface& device);
CmdResult loadRtkDongleConfiguration(C::mip_interface& device);
CmdResult defaultRtkDongleConfiguration(C::mip_interface& device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_gnss_receiver_safe_mode  Receiver Safe Mode
/// Enable/disable safe mode for the provided receiver ID.
/// Note: Receivers in safe mode will not output valid GNSS data.
/// 
///
///@{

struct ReceiverSafeMode
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_gnss::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_gnss::CMD_RECEIVER_SAFE_MODE;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    uint8_t receiver_id = 0; ///< Receiver id: e.g. 1, 2, etc.
    uint8_t enable = 0; ///< 0 - Disabled, 1- Enabled
    
};
void insert(Serializer& serializer, const ReceiverSafeMode& self);
void extract(Serializer& serializer, ReceiverSafeMode& self);

CmdResult receiverSafeMode(C::mip_interface& device, uint8_t receiverId, uint8_t enable);
///@}
///

///@}
///@}
///
////////////////////////////////////////////////////////////////////////////////
} // namespace commands_gnss
} // namespace mip

