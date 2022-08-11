
#include "commands_gnss.hpp"

#include "../../utils/serialization.h"
#include "../mip_interface.h"

#include <assert.h>


namespace mip {
class Serializer;

namespace C {
struct mip_interface;
} // namespace C

namespace commands_gnss {

using ::mip::insert;
using ::mip::extract;
using namespace ::mip::C;

////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

void insert(Serializer& serializer, const ReceiverInfo& self)
{
    (void)serializer;
    (void)self;
}

void extract(Serializer& serializer, ReceiverInfo& self)
{
    (void)serializer;
    (void)self;
}

void insert(Serializer& serializer, const ReceiverInfo::Info& self)
{
    insert(serializer, self.receiver_id);
    insert(serializer, self.mip_data_descriptor_set);
    for(unsigned int i=0; i < 32; i++)
        insert(serializer, self.description[i]);
}

void extract(Serializer& serializer, ReceiverInfo::Info& self)
{
    extract(serializer, self.receiver_id);
    extract(serializer, self.mip_data_descriptor_set);
    for(unsigned int i=0; i < 32; i++)
        extract(serializer, self.description[i]);
}

/// @brief Return information about the GNSS receivers in the device.
/// 
/// @param[out] num_receivers Number of physical receivers in the device
/// @param[out] receiver_info 
/// 
/// @returns CmdResult
/// 
CmdResult receiverInfo(C::mip_interface& device, uint8_t& num_receivers, ReceiverInfo::Info* receiver_info)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    mip_cmd_result result_local = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_LIST_RECEIVERS, NULL, 0, REPLY_LIST_RECEIVERS, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        Serializer serializer(buffer, sizeof(buffer));
        
        ::mip::C::extract_count(&serializer, &num_receivers, num_receivers);
        for(unsigned int i=0; i < num_receivers; i++)
            extract(serializer, receiver_info[i]);
        
        if( !!!serializer )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

void insert(Serializer& serializer, const SignalConfiguration& self)
{
    insert(serializer, self.function);
    insert(serializer, self.gps_enable);
    insert(serializer, self.glonass_enable);
    insert(serializer, self.galileo_enable);
    insert(serializer, self.beidou_enable);
    for(unsigned int i=0; i < 4; i++)
        insert(serializer, self.reserved[i]);
}

void extract(Serializer& serializer, SignalConfiguration& self)
{
    extract(serializer, self.function);
    extract(serializer, self.gps_enable);
    extract(serializer, self.glonass_enable);
    extract(serializer, self.galileo_enable);
    extract(serializer, self.beidou_enable);
    for(unsigned int i=0; i < 4; i++)
        extract(serializer, self.reserved[i]);
}

/// @brief Configure the GNSS signals used by the device.
/// 
/// @param gps_enable Bitfield 0: Enable L1CA, 1: Enable L2C
/// @param glonass_enable Bitfield 0: Enable L1OF, 1: Enable L2OF
/// @param galileo_enable Bitfield 0: Enable E1,   1: Enable E5B
/// @param beidou_enable Bitfield 0: Enable B1,   1: Enable B2
/// @param reserved 
/// 
/// @returns CmdResult
/// 
CmdResult writeSignalConfiguration(C::mip_interface& device, uint8_t gps_enable, uint8_t glonass_enable, uint8_t galileo_enable, uint8_t beidou_enable, const uint8_t* reserved)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, gps_enable);
    insert(serializer, glonass_enable);
    insert(serializer, galileo_enable);
    insert(serializer, beidou_enable);
    for(unsigned int i=0; i < 4; i++)
        insert(serializer, reserved[i]);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SIGNAL_CONFIGURATION, buffer, serializer.offset);
}

/// @brief Configure the GNSS signals used by the device.
/// 
/// @param[out] gps_enable Bitfield 0: Enable L1CA, 1: Enable L2C
/// @param[out] glonass_enable Bitfield 0: Enable L1OF, 1: Enable L2OF
/// @param[out] galileo_enable Bitfield 0: Enable E1,   1: Enable E5B
/// @param[out] beidou_enable Bitfield 0: Enable B1,   1: Enable B2
/// @param[out] reserved 
/// 
/// @returns CmdResult
/// 
CmdResult readSignalConfiguration(C::mip_interface& device, uint8_t& gps_enable, uint8_t& glonass_enable, uint8_t& galileo_enable, uint8_t& beidou_enable, uint8_t* reserved)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(!!serializer);
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_SIGNAL_CONFIGURATION, buffer, serializer.offset, REPLY_SIGNAL_CONFIGURATION, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        Serializer serializer(buffer, sizeof(buffer));
        
        extract(serializer, gps_enable);
        extract(serializer, glonass_enable);
        extract(serializer, galileo_enable);
        extract(serializer, beidou_enable);
        for(unsigned int i=0; i < 4; i++)
            extract(serializer, reserved[i]);
        
        if( !!!serializer )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Configure the GNSS signals used by the device.
/// 
/// 
/// @returns CmdResult
/// 
CmdResult saveSignalConfiguration(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SIGNAL_CONFIGURATION, buffer, serializer.offset);
}

/// @brief Configure the GNSS signals used by the device.
/// 
/// 
/// @returns CmdResult
/// 
CmdResult loadSignalConfiguration(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SIGNAL_CONFIGURATION, buffer, serializer.offset);
}

/// @brief Configure the GNSS signals used by the device.
/// 
/// 
/// @returns CmdResult
/// 
CmdResult defaultSignalConfiguration(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SIGNAL_CONFIGURATION, buffer, serializer.offset);
}

void insert(Serializer& serializer, const RtkDongleConfiguration& self)
{
    insert(serializer, self.function);
    insert(serializer, self.enable);
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.reserved[i]);
}

void extract(Serializer& serializer, RtkDongleConfiguration& self)
{
    extract(serializer, self.function);
    extract(serializer, self.enable);
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.reserved[i]);
}

/// @brief Configure the communications with the RTK Dongle connected to the device.
/// 
/// @param enable 0 - Disabled, 1- Enabled
/// @param reserved 
/// 
/// @returns CmdResult
/// 
CmdResult writeRtkDongleConfiguration(C::mip_interface& device, uint8_t enable, const uint8_t* reserved)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, enable);
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, reserved[i]);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_RTK_DONGLE_CONFIGURATION, buffer, serializer.offset);
}

/// @brief Configure the communications with the RTK Dongle connected to the device.
/// 
/// @param[out] enable 
/// @param[out] reserved 
/// 
/// @returns CmdResult
/// 
CmdResult readRtkDongleConfiguration(C::mip_interface& device, uint8_t& enable, uint8_t* reserved)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(!!serializer);
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_RTK_DONGLE_CONFIGURATION, buffer, serializer.offset, REPLY_RTK_DONGLE_CONFIGURATION, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        Serializer serializer(buffer, sizeof(buffer));
        
        extract(serializer, enable);
        for(unsigned int i=0; i < 3; i++)
            extract(serializer, reserved[i]);
        
        if( !!!serializer )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Configure the communications with the RTK Dongle connected to the device.
/// 
/// 
/// @returns CmdResult
/// 
CmdResult saveRtkDongleConfiguration(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_RTK_DONGLE_CONFIGURATION, buffer, serializer.offset);
}

/// @brief Configure the communications with the RTK Dongle connected to the device.
/// 
/// 
/// @returns CmdResult
/// 
CmdResult loadRtkDongleConfiguration(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_RTK_DONGLE_CONFIGURATION, buffer, serializer.offset);
}

/// @brief Configure the communications with the RTK Dongle connected to the device.
/// 
/// 
/// @returns CmdResult
/// 
CmdResult defaultRtkDongleConfiguration(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_RTK_DONGLE_CONFIGURATION, buffer, serializer.offset);
}

void insert(Serializer& serializer, const ReceiverSafeMode& self)
{
    insert(serializer, self.receiver_id);
    insert(serializer, self.enable);
}

void extract(Serializer& serializer, ReceiverSafeMode& self)
{
    extract(serializer, self.receiver_id);
    extract(serializer, self.enable);
}

/// @brief Enable/disable safe mode for the provided receiver ID.
/// Note: Receivers in safe mode will not output valid GNSS data.
/// 
/// @param receiver_id Receiver id: e.g. 1, 2, etc.
/// @param enable 0 - Disabled, 1- Enabled
/// 
/// @returns CmdResult
/// 
CmdResult receiverSafeMode(C::mip_interface& device, uint8_t receiver_id, uint8_t enable)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, receiver_id);
    insert(serializer, enable);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_RECEIVER_SAFE_MODE, buffer, serializer.offset);
}


} // namespace commands_gnss
} // namespace mip

