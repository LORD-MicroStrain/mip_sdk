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
///@defgroup GNSS_COMMAND  GNSS COMMAND
///
///@{

////////////////////////////////////////////////////////////////////////////////
// Descriptors
////////////////////////////////////////////////////////////////////////////////

enum mip_gnss_command_descriptors
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
#ifdef __cplusplus
namespace C {
extern "C" {
#endif // __cplusplus


////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////

#define MIP_GNSS_GPS_ENABLE_L1CA_GNSS_GPS_ENABLE_L1CA 0x0001
#define MIP_GNSS_GPS_ENABLE_L2C_GNSS_GPS_ENABLE_L2C 0x0002
#define MIP_GNSS_GLONASS_ENABLE_L1OF_GNSS_GLONASS_ENABLE_L1OF 0x0001
#define MIP_GNSS_GLONASS_ENABLE_L2OF_GNSS_GLONASS_ENABLE_L2OF 0x0002
#define MIP_GNSS_GALILEO_ENABLE_E1_GNSS_GALILEO_ENABLE_E1 0x0001
#define MIP_GNSS_GALILEO_ENABLE_E5B_GNSS_GALILEO_ENABLE_E5B 0x0002
#define MIP_GNSS_BEIDOU_ENABLE_B1_GNSS_BEIDOU_ENABLE_B1 0x0001
#define MIP_GNSS_BEIDOU_ENABLE_B2_GNSS_BEIDOU_ENABLE_B2 0x0002

////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_gnss_receiver_info  None
/// Return information about the GNSS receivers in the device.
/// 
///
///@{

struct mip_gnss_receiver_info_command
{
};
size_t insert_mip_gnss_receiver_info_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_gnss_receiver_info_command* self);
size_t extract_mip_gnss_receiver_info_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_gnss_receiver_info_command* self);

struct mip_gnss_receiver_info_command_receiver_info
{
    uint8_t                                           receiver_id;
    uint8_t                                           mip_data_descriptor_set;
    char                                              description[32];
};
size_t insert_mip_gnss_receiver_info_command_receiver_info(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_gnss_receiver_info_command_receiver_info* self);
size_t extract_mip_gnss_receiver_info_command_receiver_info(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_gnss_receiver_info_command_receiver_info* self);

struct mip_gnss_receiver_info_response
{
    uint8_t                                           num_receivers;
    struct mip_gnss_receiver_info_command_receiver_info* receiver_info;
};
size_t insert_mip_gnss_receiver_info_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_gnss_receiver_info_response* self);
size_t extract_mip_gnss_receiver_info_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_gnss_receiver_info_response* self);

mip_cmd_result mip_gnss_receiver_info(struct mip_interface* device, uint8_t* num_receivers, struct mip_gnss_receiver_info_command_receiver_info* receiver_info);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_gnss_signal_configuration  None
/// Configure the GNSS signals used by the device.
/// 
///
///@{

struct mip_gnss_signal_configuration_command
{
    enum mip_function_selector                        function;
    uint8_t                                           gps_enable;
    uint8_t                                           glonass_enable;
    uint8_t                                           galileo_enable;
    uint8_t                                           beidou_enable;
    uint8_t                                           reserved[4];
};
size_t insert_mip_gnss_signal_configuration_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_gnss_signal_configuration_command* self);
size_t extract_mip_gnss_signal_configuration_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_gnss_signal_configuration_command* self);

struct mip_gnss_signal_configuration_response
{
    uint8_t                                           gps_enable;
    uint8_t                                           glonass_enable;
    uint8_t                                           galileo_enable;
    uint8_t                                           beidou_enable;
    uint8_t                                           reserved[4];
};
size_t insert_mip_gnss_signal_configuration_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_gnss_signal_configuration_response* self);
size_t extract_mip_gnss_signal_configuration_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_gnss_signal_configuration_response* self);

mip_cmd_result write_mip_gnss_signal_configuration(struct mip_interface* device, uint8_t gps_enable, uint8_t glonass_enable, uint8_t galileo_enable, uint8_t beidou_enable, const uint8_t* reserved);
mip_cmd_result read_mip_gnss_signal_configuration(struct mip_interface* device, uint8_t* gps_enable, uint8_t* glonass_enable, uint8_t* galileo_enable, uint8_t* beidou_enable, uint8_t* reserved);
mip_cmd_result save_mip_gnss_signal_configuration(struct mip_interface* device);
mip_cmd_result load_mip_gnss_signal_configuration(struct mip_interface* device);
mip_cmd_result default_mip_gnss_signal_configuration(struct mip_interface* device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_gnss_rtk_dongle_configuration  None
/// Configure the communications with the RTK Dongle connected to the device.
/// 
///
///@{

struct mip_gnss_rtk_dongle_configuration_command
{
    enum mip_function_selector                        function;
    uint8_t                                           enable;
    uint8_t                                           reserved[3];
};
size_t insert_mip_gnss_rtk_dongle_configuration_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_gnss_rtk_dongle_configuration_command* self);
size_t extract_mip_gnss_rtk_dongle_configuration_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_gnss_rtk_dongle_configuration_command* self);

struct mip_gnss_rtk_dongle_configuration_response
{
    uint8_t                                           enable;
    uint8_t                                           reserved[3];
};
size_t insert_mip_gnss_rtk_dongle_configuration_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_gnss_rtk_dongle_configuration_response* self);
size_t extract_mip_gnss_rtk_dongle_configuration_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_gnss_rtk_dongle_configuration_response* self);

mip_cmd_result write_mip_gnss_rtk_dongle_configuration(struct mip_interface* device, uint8_t enable, const uint8_t* reserved);
mip_cmd_result read_mip_gnss_rtk_dongle_configuration(struct mip_interface* device, uint8_t* enable, uint8_t* reserved);
mip_cmd_result save_mip_gnss_rtk_dongle_configuration(struct mip_interface* device);
mip_cmd_result load_mip_gnss_rtk_dongle_configuration(struct mip_interface* device);
mip_cmd_result default_mip_gnss_rtk_dongle_configuration(struct mip_interface* device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_gnss_receiver_safe_mode  GNSS Receiver Safe Mode
/// Enable/disable safe mode for the provided receiver ID.
/// Note: Receivers in safe mode will not output valid GNSS data.
/// 
///
///@{

struct mip_gnss_receiver_safe_mode_command
{
    uint8_t                                           receiver_id;
    uint8_t                                           enable;
};
size_t insert_mip_gnss_receiver_safe_mode_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_gnss_receiver_safe_mode_command* self);
size_t extract_mip_gnss_receiver_safe_mode_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_gnss_receiver_safe_mode_command* self);

mip_cmd_result gnss_receiver_safe_mode(struct mip_interface* device, uint8_t receiver_id, uint8_t enable);
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
struct MipFieldInfo<C::mip_gnss_receiver_info_command>
{
    static const uint8_t descriptorSet = MIP_GNSS_COMMAND_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_GNSS_LIST_RECEIVERS;
    static const uint8_t responseDescriptor = MIP_REPLY_DESC_GNSS_LIST_RECEIVERS;
    typedef C::mip_gnss_receiver_info_response Response;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_gnss_receiver_info_command& self)
    {
        return C::insert_mip_gnss_receiver_info_command(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_gnss_receiver_info_command& self)
    {
        return C::extract_mip_gnss_receiver_info_command(buffer, bufferSize, offset, &self);
    }
    static inline size_t insert_response(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_gnss_receiver_info_response& self)
    {
        return C::insert_mip_gnss_receiver_info_response(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract_response(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_gnss_receiver_info_response& self)
    {
        return C::extract_mip_gnss_receiver_info_response(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_gnss_signal_configuration_command>
{
    static const uint8_t descriptorSet = MIP_GNSS_COMMAND_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_GNSS_SIGNAL_CONFIGURATION;
    static const uint8_t responseDescriptor = MIP_REPLY_DESC_GNSS_SIGNAL_CONFIGURATION;
    typedef C::mip_gnss_signal_configuration_response Response;
    
    static const bool hasFunctionSelector = true;
    static const bool canWrite = true;
    static const bool canRead = true;
    static const bool canSave = true;
    static const bool canLoad = true;
    static const bool canReset = true;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_gnss_signal_configuration_command& self)
    {
        return C::insert_mip_gnss_signal_configuration_command(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_gnss_signal_configuration_command& self)
    {
        return C::extract_mip_gnss_signal_configuration_command(buffer, bufferSize, offset, &self);
    }
    static inline size_t insert_response(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_gnss_signal_configuration_response& self)
    {
        return C::insert_mip_gnss_signal_configuration_response(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract_response(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_gnss_signal_configuration_response& self)
    {
        return C::extract_mip_gnss_signal_configuration_response(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_gnss_rtk_dongle_configuration_command>
{
    static const uint8_t descriptorSet = MIP_GNSS_COMMAND_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_GNSS_RTK_DONGLE_CONFIGURATION;
    static const uint8_t responseDescriptor = MIP_REPLY_DESC_GNSS_RTK_DONGLE_CONFIGURATION;
    typedef C::mip_gnss_rtk_dongle_configuration_response Response;
    
    static const bool hasFunctionSelector = true;
    static const bool canWrite = true;
    static const bool canRead = true;
    static const bool canSave = true;
    static const bool canLoad = true;
    static const bool canReset = true;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_gnss_rtk_dongle_configuration_command& self)
    {
        return C::insert_mip_gnss_rtk_dongle_configuration_command(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_gnss_rtk_dongle_configuration_command& self)
    {
        return C::extract_mip_gnss_rtk_dongle_configuration_command(buffer, bufferSize, offset, &self);
    }
    static inline size_t insert_response(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_gnss_rtk_dongle_configuration_response& self)
    {
        return C::insert_mip_gnss_rtk_dongle_configuration_response(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract_response(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_gnss_rtk_dongle_configuration_response& self)
    {
        return C::extract_mip_gnss_rtk_dongle_configuration_response(buffer, bufferSize, offset, &self);
    }
};



template<>
struct MipFieldInfo<C::mip_gnss_receiver_safe_mode_command>
{
    static const uint8_t descriptorSet = MIP_GNSS_COMMAND_DESC_SET;
    static const uint8_t fieldDescriptor = MIP_CMD_DESC_GNSS_RECEIVER_SAFE_MODE;
    static const uint8_t responseDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    
    static const bool hasFunctionSelector = false;
    
    static inline size_t insert(uint8_t* buffer, size_t bufferSize, size_t offset, const C::mip_gnss_receiver_safe_mode_command& self)
    {
        return C::insert_mip_gnss_receiver_safe_mode_command(buffer, bufferSize, offset, &self);
    }
    static inline size_t extract(const uint8_t* buffer, size_t bufferSize, size_t offset, C::mip_gnss_receiver_safe_mode_command& self)
    {
        return C::extract_mip_gnss_receiver_safe_mode_command(buffer, bufferSize, offset, &self);
    }
};



} // namespace mscl
#endif // __cplusplus
