#pragma once

#include "descriptors.h"
#include "../mip_result.h"

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

namespace mscl {
class MipSerializer;

namespace C {
struct mip_interface;
} // namespace C

namespace commands_gnss {

////////////////////////////////////////////////////////////////////////////////
///@addtogroup MipCommands
///@{
///@defgroup gnss_commands_cpp  GNSSCommands
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
///@defgroup cpp_receiver_info  None
/// Return information about the GNSS receivers in the device.
/// 
///
///@{

struct ReceiverInfo
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::commands_gnss::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::commands_gnss::CMD_LIST_RECEIVERS;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    struct Info
    {
        uint8_t receiver_id;
        uint8_t mip_data_descriptor_set;
        char description[32];
        
    };
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mscl::commands_gnss::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mscl::commands_gnss::REPLY_LIST_RECEIVERS;
        
        uint8_t num_receivers;
        Info receiver_info[5];
        
    };
};
void insert(MipSerializer& serializer, const ReceiverInfo& self);
void extract(MipSerializer& serializer, ReceiverInfo& self);

void insert(MipSerializer& serializer, const ReceiverInfo::Info& self);
void extract(MipSerializer& serializer, ReceiverInfo::Info& self);

void insert(MipSerializer& serializer, const ReceiverInfo::Response& self);
void extract(MipSerializer& serializer, ReceiverInfo::Response& self);

MipCmdResult receiverInfo(C::mip_interface& device, uint8_t& num_receivers, ReceiverInfo::Info* receiver_info);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_signal_configuration  None
/// Configure the GNSS signals used by the device.
/// 
///
///@{

struct SignalConfiguration
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::commands_gnss::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::commands_gnss::CMD_SIGNAL_CONFIGURATION;
    
    static const bool HAS_WRITE_FUNCTION = true;
    static const bool HAS_READ_FUNCTION = true;
    static const bool HAS_SAVE_FUNCTION = true;
    static const bool HAS_LOAD_FUNCTION = true;
    static const bool HAS_RESET_FUNCTION = true;
    
    MipFunctionSelector function;
    uint8_t gps_enable;
    uint8_t glonass_enable;
    uint8_t galileo_enable;
    uint8_t beidou_enable;
    uint8_t reserved[4];
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mscl::commands_gnss::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mscl::commands_gnss::REPLY_SIGNAL_CONFIGURATION;
        
        uint8_t gps_enable;
        uint8_t glonass_enable;
        uint8_t galileo_enable;
        uint8_t beidou_enable;
        uint8_t reserved[4];
        
    };
};
void insert(MipSerializer& serializer, const SignalConfiguration& self);
void extract(MipSerializer& serializer, SignalConfiguration& self);

void insert(MipSerializer& serializer, const SignalConfiguration::Response& self);
void extract(MipSerializer& serializer, SignalConfiguration::Response& self);

MipCmdResult writeSignalConfiguration(C::mip_interface& device, uint8_t gps_enable, uint8_t glonass_enable, uint8_t galileo_enable, uint8_t beidou_enable, const uint8_t* reserved);
MipCmdResult readSignalConfiguration(C::mip_interface& device, uint8_t& gps_enable, uint8_t& glonass_enable, uint8_t& galileo_enable, uint8_t& beidou_enable, uint8_t* reserved);
MipCmdResult saveSignalConfiguration(C::mip_interface& device);
MipCmdResult loadSignalConfiguration(C::mip_interface& device);
MipCmdResult defaultSignalConfiguration(C::mip_interface& device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_rtk_dongle_configuration  None
/// Configure the communications with the RTK Dongle connected to the device.
/// 
///
///@{

struct RtkDongleConfiguration
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::commands_gnss::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::commands_gnss::CMD_RTK_DONGLE_CONFIGURATION;
    
    static const bool HAS_WRITE_FUNCTION = true;
    static const bool HAS_READ_FUNCTION = true;
    static const bool HAS_SAVE_FUNCTION = true;
    static const bool HAS_LOAD_FUNCTION = true;
    static const bool HAS_RESET_FUNCTION = true;
    
    MipFunctionSelector function;
    uint8_t enable;
    uint8_t reserved[3];
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mscl::commands_gnss::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mscl::commands_gnss::REPLY_RTK_DONGLE_CONFIGURATION;
        
        uint8_t enable;
        uint8_t reserved[3];
        
    };
};
void insert(MipSerializer& serializer, const RtkDongleConfiguration& self);
void extract(MipSerializer& serializer, RtkDongleConfiguration& self);

void insert(MipSerializer& serializer, const RtkDongleConfiguration::Response& self);
void extract(MipSerializer& serializer, RtkDongleConfiguration::Response& self);

MipCmdResult writeRtkDongleConfiguration(C::mip_interface& device, uint8_t enable, const uint8_t* reserved);
MipCmdResult readRtkDongleConfiguration(C::mip_interface& device, uint8_t& enable, uint8_t* reserved);
MipCmdResult saveRtkDongleConfiguration(C::mip_interface& device);
MipCmdResult loadRtkDongleConfiguration(C::mip_interface& device);
MipCmdResult defaultRtkDongleConfiguration(C::mip_interface& device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_receiver_safe_mode  GNSS Receiver Safe Mode
/// Enable/disable safe mode for the provided receiver ID.
/// Note: Receivers in safe mode will not output valid GNSS data.
/// 
///
///@{

struct ReceiverSafeMode
{
    static const uint8_t DESCRIPTOR_SET = ::mscl::commands_gnss::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mscl::commands_gnss::CMD_RECEIVER_SAFE_MODE;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    uint8_t receiver_id;
    uint8_t enable;
    
};
void insert(MipSerializer& serializer, const ReceiverSafeMode& self);
void extract(MipSerializer& serializer, ReceiverSafeMode& self);

MipCmdResult receiverSafeMode(C::mip_interface& device, uint8_t receiver_id, uint8_t enable);
///@}
///

///@}
///@}
///
////////////////////////////////////////////////////////////////////////////////
} // namespace commands_gnss
} // namespace mscl

