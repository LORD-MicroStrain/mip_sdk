#pragma once

#include "descriptors.h"

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

////////////////////////////////////////////////////////////////////////////////
///@addtogroup MipCommands
///@{
///@defgroup GNSS_COMMAND  GNSS COMMAND
///
///@{

////////////////////////////////////////////////////////////////////////////////
// Descriptors
////////////////////////////////////////////////////////////////////////////////

enum MipGnssCommand_Descriptors
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

#define MIPGNSS_GPS_ENABLE_L1CA_GNSS_GPS_ENABLE_L1CA 0x0001
#define MIPGNSS_GPS_ENABLE_L2C_GNSS_GPS_ENABLE_L2C 0x0002
#define MIPGNSS_GLONASS_ENABLE_L1OF_GNSS_GLONASS_ENABLE_L1OF 0x0001
#define MIPGNSS_GLONASS_ENABLE_L2OF_GNSS_GLONASS_ENABLE_L2OF 0x0002
#define MIPGNSS_GALILEO_ENABLE_E1_GNSS_GALILEO_ENABLE_E1 0x0001
#define MIPGNSS_GALILEO_ENABLE_E5B_GNSS_GALILEO_ENABLE_E5B 0x0002
#define MIPGNSS_BEIDOU_ENABLE_B1_GNSS_BEIDOU_ENABLE_B1 0x0001
#define MIPGNSS_BEIDOU_ENABLE_B2_GNSS_BEIDOU_ENABLE_B2 0x0002

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

///@}
///

///@}
///@}
///
////////////////////////////////////////////////////////////////////////////////
