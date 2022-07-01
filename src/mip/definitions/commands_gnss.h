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
///@defgroup GNSS_COMMAND  GNSS COMMAND
///
///@{

////////////////////////////////////////////////////////////////////////////////
// Descriptors
////////////////////////////////////////////////////////////////////////////////

enum MipGnssCommandDescriptors
{
    MIP_GNSS_COMMAND_DESC_SET                    = 0x0E,
    
    MIP_CMD_DESC_GNSS_LIST_RECEIVERS             = 0x01,
    MIP_CMD_DESC_GNSS_SIGNAL_CONFIGURATION       = 0x02,
    MIP_CMD_DESC_GNSS_RTK_DONGLE_CONFIGURATION   = 0x10,
    MIP_CMD_DESC_GNSS_RECEIVER_SAFE_MODE         = 0x60,
    
    MIP_REPLY_DESC_GNSS_LIST_RECEIVERS           = 0x81,
    MIP_REPLY_DESC_GNSS_SIGNAL_CONFIGURATION     = 0x82,
    MIP_REPLY_DESC_GNSS_RTK_DONGLE_CONFIGURATION = 0x90,
};

////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////

#define MIPGNSSGPSENABLEL1CA_GNSS_GPS_ENABLE_L1CA 0x0001
#define MIPGNSSGPSENABLEL2C_GNSS_GPS_ENABLE_L2C 0x0002
#define MIPGNSSGLONASSENABLEL1OF_GNSS_GLONASS_ENABLE_L1OF 0x0001
#define MIPGNSSGLONASSENABLEL2OF_GNSS_GLONASS_ENABLE_L2OF 0x0002
#define MIPGNSSGALILEOENABLEE1_GNSS_GALILEO_ENABLE_E1 0x0001
#define MIPGNSSGALILEOENABLEE5B_GNSS_GALILEO_ENABLE_E5B 0x0002
#define MIPGNSSBEIDOUENABLEB1_GNSS_BEIDOU_ENABLE_B1 0x0001
#define MIPGNSSBEIDOUENABLEB2_GNSS_BEIDOU_ENABLE_B2 0x0002

////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_gnss_receiver_info  Receiver Info
/// Return information about the GNSS receivers in the device.
/// 
///
///@{

struct MipCmd_Gnss_ReceiverInfo
{
};
size_t insert_MipCmd_Gnss_ReceiverInfo(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Gnss_ReceiverInfo* self);
size_t extract_MipCmd_Gnss_ReceiverInfo(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Gnss_ReceiverInfo* self);

struct MipCmd_Gnss_ReceiverInfo_Receiverinfo
{
    uint8_t                                           receiver_id;
    uint8_t                                           mip_data_descriptor_set;
    char                                              description[32];
};
size_t insert_MipCmd_Gnss_ReceiverInfo_Receiverinfo(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Gnss_ReceiverInfo_Receiverinfo* self);
size_t extract_MipCmd_Gnss_ReceiverInfo_Receiverinfo(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Gnss_ReceiverInfo_Receiverinfo* self);

struct MipCmd_Gnss_ReceiverInfo_Response
{
    uint8_t                                           num_receivers;
    struct MipCmd_Gnss_ReceiverInfo_Receiverinfo*     receiver_info;
};
size_t insert_MipCmd_Gnss_ReceiverInfo_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Gnss_ReceiverInfo_Response* self);
size_t extract_MipCmd_Gnss_ReceiverInfo_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Gnss_ReceiverInfo_Response* self);

MipCmdResult mip_gnss_receiver_info(struct MipInterfaceState* device, uint8_t* num_receivers, struct MipCmd_Gnss_ReceiverInfo_Receiverinfo* receiver_info);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_gnss_signal_configuration  Signal Configuration
/// Configure the GNSS signals used by the device.
/// 
///
///@{

struct MipCmd_Gnss_SignalConfiguration
{
    enum MipFunctionSelector                          function;
    uint8_t                                           gps_enable;
    uint8_t                                           glonass_enable;
    uint8_t                                           galileo_enable;
    uint8_t                                           beidou_enable;
    uint8_t                                           reserved[4];
};
size_t insert_MipCmd_Gnss_SignalConfiguration(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Gnss_SignalConfiguration* self);
size_t extract_MipCmd_Gnss_SignalConfiguration(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Gnss_SignalConfiguration* self);

struct MipCmd_Gnss_SignalConfiguration_Response
{
    uint8_t                                           gps_enable;
    uint8_t                                           glonass_enable;
    uint8_t                                           galileo_enable;
    uint8_t                                           beidou_enable;
    uint8_t                                           reserved[4];
};
size_t insert_MipCmd_Gnss_SignalConfiguration_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Gnss_SignalConfiguration_Response* self);
size_t extract_MipCmd_Gnss_SignalConfiguration_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Gnss_SignalConfiguration_Response* self);

MipCmdResult write_mip_gnss_signal_configuration(struct MipInterfaceState* device, uint8_t gps_enable, uint8_t glonass_enable, uint8_t galileo_enable, uint8_t beidou_enable, const uint8_t* reserved);
MipCmdResult read_mip_gnss_signal_configuration(struct MipInterfaceState* device, uint8_t* gps_enable, uint8_t* glonass_enable, uint8_t* galileo_enable, uint8_t* beidou_enable, uint8_t* reserved);
MipCmdResult save_mip_gnss_signal_configuration(struct MipInterfaceState* device);
MipCmdResult load_mip_gnss_signal_configuration(struct MipInterfaceState* device);
MipCmdResult default_mip_gnss_signal_configuration(struct MipInterfaceState* device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_gnss_rtk_dongle_configuration  Rtk Dongle Configuration
/// Configure the communications with the RTK Dongle connected to the device.
/// 
///
///@{

struct MipCmd_Gnss_RtkDongleConfiguration
{
    enum MipFunctionSelector                          function;
    uint8_t                                           enable;
    uint8_t                                           reserved[3];
};
size_t insert_MipCmd_Gnss_RtkDongleConfiguration(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Gnss_RtkDongleConfiguration* self);
size_t extract_MipCmd_Gnss_RtkDongleConfiguration(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Gnss_RtkDongleConfiguration* self);

struct MipCmd_Gnss_RtkDongleConfiguration_Response
{
    uint8_t                                           enable;
    uint8_t                                           reserved[3];
};
size_t insert_MipCmd_Gnss_RtkDongleConfiguration_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Gnss_RtkDongleConfiguration_Response* self);
size_t extract_MipCmd_Gnss_RtkDongleConfiguration_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Gnss_RtkDongleConfiguration_Response* self);

MipCmdResult write_mip_gnss_rtk_dongle_configuration(struct MipInterfaceState* device, uint8_t enable, const uint8_t* reserved);
MipCmdResult read_mip_gnss_rtk_dongle_configuration(struct MipInterfaceState* device, uint8_t* enable, uint8_t* reserved);
MipCmdResult save_mip_gnss_rtk_dongle_configuration(struct MipInterfaceState* device);
MipCmdResult load_mip_gnss_rtk_dongle_configuration(struct MipInterfaceState* device);
MipCmdResult default_mip_gnss_rtk_dongle_configuration(struct MipInterfaceState* device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_gnss_receiver_safe_mode  Receiver Safe Mode
/// Enable/disable safe mode for the provided receiver ID.
/// Note: Receivers in safe mode will not output valid GNSS data.
/// 
///
///@{

struct MipCmd_Gnss_ReceiverSafeMode
{
    uint8_t                                           receiver_id;
    uint8_t                                           enable;
};
size_t insert_MipCmd_Gnss_ReceiverSafeMode(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Gnss_ReceiverSafeMode* self);
size_t extract_MipCmd_Gnss_ReceiverSafeMode(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Gnss_ReceiverSafeMode* self);

MipCmdResult gnss_receiver_safe_mode(struct MipInterfaceState* device, uint8_t receiver_id, uint8_t enable);
///@}
///

///@}
///@}
///
////////////////////////////////////////////////////////////////////////////////

#ifdef __cplusplus
} // extern "C"


template<>
struct MipFieldInfo<MipCmd_Gnss_ReceiverInfo>
{
    static const uint8_t descriptorSet = MIP_GNSS_COMMAND_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_GNSS_LIST_RECEIVERS;
    static const uint8_t responseDescriptor = MIP_REPLY_DESC_GNSS_LIST_RECEIVERS;
    typedef MipCmd_Gnss_ReceiverInfo_Response Response;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipCmd_Gnss_ReceiverInfo& self)
    {
        return insert_MipCmd_Gnss_ReceiverInfo(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipCmd_Gnss_ReceiverInfo& self)
    {
        return extract_MipCmd_Gnss_ReceiverInfo(buffer, bufferSize, offset, &self);
    }
    static inline size_t insert_response(uint8_t* buffer, size_t bufferSize, size_t offset, const MipCmd_Gnss_ReceiverInfo_Response& self)
    {
        return insert_MipCmd_Gnss_ReceiverInfo_Response(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract_response(const uint8_t* buffer, size_t bufferSize, size_t offset, MipCmd_Gnss_ReceiverInfo_Response& self)
    {
        return extract_MipCmd_Gnss_ReceiverInfo_Response(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipCmd_Gnss_SignalConfiguration>
{
    static const uint8_t descriptorSet = MIP_GNSS_COMMAND_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_GNSS_SIGNAL_CONFIGURATION;
    static const uint8_t responseDescriptor = MIP_REPLY_DESC_GNSS_SIGNAL_CONFIGURATION;
    typedef MipCmd_Gnss_SignalConfiguration_Response Response;
    
    static const bool hasFunctionSelector = true;
    static const bool canWrite = true;
    static const bool canRead = true;
    static const bool canSave = true;
    static const bool canLoad = true;
    static const bool canReset = true;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipCmd_Gnss_SignalConfiguration& self)
    {
        return insert_MipCmd_Gnss_SignalConfiguration(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipCmd_Gnss_SignalConfiguration& self)
    {
        return extract_MipCmd_Gnss_SignalConfiguration(buffer, bufferSize, offset, &self);
    }
    static inline size_t insert_response(uint8_t* buffer, size_t bufferSize, size_t offset, const MipCmd_Gnss_SignalConfiguration_Response& self)
    {
        return insert_MipCmd_Gnss_SignalConfiguration_Response(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract_response(const uint8_t* buffer, size_t bufferSize, size_t offset, MipCmd_Gnss_SignalConfiguration_Response& self)
    {
        return extract_MipCmd_Gnss_SignalConfiguration_Response(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipCmd_Gnss_RtkDongleConfiguration>
{
    static const uint8_t descriptorSet = MIP_GNSS_COMMAND_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_GNSS_RTK_DONGLE_CONFIGURATION;
    static const uint8_t responseDescriptor = MIP_REPLY_DESC_GNSS_RTK_DONGLE_CONFIGURATION;
    typedef MipCmd_Gnss_RtkDongleConfiguration_Response Response;
    
    static const bool hasFunctionSelector = true;
    static const bool canWrite = true;
    static const bool canRead = true;
    static const bool canSave = true;
    static const bool canLoad = true;
    static const bool canReset = true;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipCmd_Gnss_RtkDongleConfiguration& self)
    {
        return insert_MipCmd_Gnss_RtkDongleConfiguration(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipCmd_Gnss_RtkDongleConfiguration& self)
    {
        return extract_MipCmd_Gnss_RtkDongleConfiguration(buffer, bufferSize, offset, &self);
    }
    static inline size_t insert_response(uint8_t* buffer, size_t bufferSize, size_t offset, const MipCmd_Gnss_RtkDongleConfiguration_Response& self)
    {
        return insert_MipCmd_Gnss_RtkDongleConfiguration_Response(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract_response(const uint8_t* buffer, size_t bufferSize, size_t offset, MipCmd_Gnss_RtkDongleConfiguration_Response& self)
    {
        return extract_MipCmd_Gnss_RtkDongleConfiguration_Response(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<MipCmd_Gnss_ReceiverSafeMode>
{
    static const uint8_t descriptorSet = MIP_GNSS_COMMAND_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_GNSS_RECEIVER_SAFE_MODE;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const MipCmd_Gnss_ReceiverSafeMode& self)
    {
        return insert_MipCmd_Gnss_ReceiverSafeMode(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, MipCmd_Gnss_ReceiverSafeMode& self)
    {
        return extract_MipCmd_Gnss_ReceiverSafeMode(buffer, bufferSize, offset, &self);
    }
};



} // namespace mscl
#endif // __cplusplus
