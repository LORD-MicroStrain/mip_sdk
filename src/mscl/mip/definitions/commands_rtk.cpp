
#include "commands_rtk.hpp"

#include "../../utils/serialization.h"
#include "../mip_interface.h"

#include <assert.h>


namespace mscl {
class MipSerializer;

namespace C {
struct mip_interface;
} // namespace C

namespace commands_rtk {

using ::mscl::insert;
using ::mscl::extract;
using namespace ::mscl::C;

////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

void insert(MipSerializer& serializer, const GetStatusFlags& self)
{
    (void)serializer;
    (void)self;
}

void extract(MipSerializer& serializer, GetStatusFlags& self)
{
    (void)serializer;
    (void)self;
}

MipCmdResult getStatusFlags(C::mip_interface& device, GetStatusFlags::StatusFlags& flags)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    mip_cmd_result result_local = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GET_STATUS_FLAGS, NULL, 0, REPLY_GET_STATUS_FLAGS, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        MipSerializer serializer(buffer, sizeof(buffer));
        
        extract(serializer, flags);
        
        if( !!!serializer )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

void insert(MipSerializer& serializer, const GetImei& self)
{
    (void)serializer;
    (void)self;
}

void extract(MipSerializer& serializer, GetImei& self)
{
    (void)serializer;
    (void)self;
}

MipCmdResult getImei(C::mip_interface& device, char* IMEI)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    mip_cmd_result result_local = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GET_IMEI, NULL, 0, REPLY_GET_IMEI, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        MipSerializer serializer(buffer, sizeof(buffer));
        
        for(unsigned int i=0; i < 32; i++)
            extract(serializer, IMEI[i]);
        
        if( !!!serializer )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

void insert(MipSerializer& serializer, const GetImsi& self)
{
    (void)serializer;
    (void)self;
}

void extract(MipSerializer& serializer, GetImsi& self)
{
    (void)serializer;
    (void)self;
}

MipCmdResult getImsi(C::mip_interface& device, char* IMSI)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    mip_cmd_result result_local = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GET_IMSI, NULL, 0, REPLY_GET_IMSI, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        MipSerializer serializer(buffer, sizeof(buffer));
        
        for(unsigned int i=0; i < 32; i++)
            extract(serializer, IMSI[i]);
        
        if( !!!serializer )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

void insert(MipSerializer& serializer, const GetIccid& self)
{
    (void)serializer;
    (void)self;
}

void extract(MipSerializer& serializer, GetIccid& self)
{
    (void)serializer;
    (void)self;
}

MipCmdResult getIccid(C::mip_interface& device, char* ICCID)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    mip_cmd_result result_local = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GET_ICCID, NULL, 0, REPLY_GET_ICCID, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        MipSerializer serializer(buffer, sizeof(buffer));
        
        for(unsigned int i=0; i < 32; i++)
            extract(serializer, ICCID[i]);
        
        if( !!!serializer )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

void insert(MipSerializer& serializer, const ConnectedDeviceType& self)
{
    insert(serializer, self.function);
    insert(serializer, self.devType);
}

void extract(MipSerializer& serializer, ConnectedDeviceType& self)
{
    extract(serializer, self.function);
    extract(serializer, self.devType);
}

MipCmdResult writeConnectedDeviceType(C::mip_interface& device, ConnectedDeviceType::Type devType)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::WRITE);
    insert(serializer, devType);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_CONNECTED_DEVICE_TYPE, buffer, serializer.offset);
}

MipCmdResult readConnectedDeviceType(C::mip_interface& device, ConnectedDeviceType::Type& devType)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::READ);
    assert(!!serializer);
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_CONNECTED_DEVICE_TYPE, buffer, serializer.offset, REPLY_CONNECTED_DEVICE_TYPE, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        MipSerializer serializer(buffer, sizeof(buffer));
        
        extract(serializer, devType);
        
        if( !!!serializer )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

MipCmdResult saveConnectedDeviceType(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::SAVE);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_CONNECTED_DEVICE_TYPE, buffer, serializer.offset);
}

MipCmdResult loadConnectedDeviceType(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::LOAD);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_CONNECTED_DEVICE_TYPE, buffer, serializer.offset);
}

MipCmdResult defaultConnectedDeviceType(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::RESET);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_CONNECTED_DEVICE_TYPE, buffer, serializer.offset);
}

void insert(MipSerializer& serializer, const GetActCode& self)
{
    (void)serializer;
    (void)self;
}

void extract(MipSerializer& serializer, GetActCode& self)
{
    (void)serializer;
    (void)self;
}

MipCmdResult getActCode(C::mip_interface& device, char* ActivationCode)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    mip_cmd_result result_local = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GET_ACT_CODE, NULL, 0, REPLY_GET_ACT_CODE, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        MipSerializer serializer(buffer, sizeof(buffer));
        
        for(unsigned int i=0; i < 32; i++)
            extract(serializer, ActivationCode[i]);
        
        if( !!!serializer )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

void insert(MipSerializer& serializer, const GetModemFirmwareVersion& self)
{
    (void)serializer;
    (void)self;
}

void extract(MipSerializer& serializer, GetModemFirmwareVersion& self)
{
    (void)serializer;
    (void)self;
}

MipCmdResult getModemFirmwareVersion(C::mip_interface& device, char* ModemFirmwareVersion)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    mip_cmd_result result_local = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GET_MODEM_FIRMWARE_VERSION, NULL, 0, REPLY_GET_MODEM_FIRMWARE_VERSION, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        MipSerializer serializer(buffer, sizeof(buffer));
        
        for(unsigned int i=0; i < 32; i++)
            extract(serializer, ModemFirmwareVersion[i]);
        
        if( !!!serializer )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

void insert(MipSerializer& serializer, const GetRssi& self)
{
    (void)serializer;
    (void)self;
}

void extract(MipSerializer& serializer, GetRssi& self)
{
    (void)serializer;
    (void)self;
}

/// @brief Get the RSSI and connected/disconnected status of modem
/// 
/// @param[out] valid 
/// @param[out] rssi 
/// @param[out] signalQuality 
/// 
/// @returns MipCmdResult
/// 
MipCmdResult getRssi(C::mip_interface& device, bool& valid, int32_t& rssi, int32_t& signalQuality)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    mip_cmd_result result_local = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GET_RSSI, NULL, 0, REPLY_GET_RSSI, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        MipSerializer serializer(buffer, sizeof(buffer));
        
        extract(serializer, valid);
        extract(serializer, rssi);
        extract(serializer, signalQuality);
        
        if( !!!serializer )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

void insert(MipSerializer& serializer, const ServiceStatus& self)
{
    insert(serializer, self.reserved1);
    insert(serializer, self.reserved2);
}

void extract(MipSerializer& serializer, ServiceStatus& self)
{
    extract(serializer, self.reserved1);
    extract(serializer, self.reserved2);
}

/// @brief The 3DMRTK will send this message to the server to indicate that the connection should remain open. The Server will respond with information and status.
/// 
/// @param reserved1 
/// @param reserved2 
/// @param[out] flags 
/// @param[out] recievedBytes 
/// @param[out] lastBytes 
/// @param[out] lastBytesTime 
/// 
/// @returns MipCmdResult
/// 
MipCmdResult serviceStatus(C::mip_interface& device, uint32_t reserved1, uint32_t reserved2, ServiceStatus::ServiceFlags& flags, uint32_t& recievedBytes, uint32_t& lastBytes, uint64_t& lastBytesTime)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, reserved1);
    insert(serializer, reserved2);
    assert(!!serializer);
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_SERVICE_STATUS, buffer, serializer.offset, REPLY_SERVICE_STATUS, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        MipSerializer serializer(buffer, sizeof(buffer));
        
        extract(serializer, flags);
        extract(serializer, recievedBytes);
        extract(serializer, lastBytes);
        extract(serializer, lastBytesTime);
        
        if( !!!serializer )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

void insert(MipSerializer& serializer, const ProdEraseStorage& self)
{
    insert(serializer, self.media);
}

void extract(MipSerializer& serializer, ProdEraseStorage& self)
{
    extract(serializer, self.media);
}

/// @brief This command will erase the selected media to a raw and unitialized state. ALL DATA WILL BE LOST.
/// This command is only available in calibration mode.
/// @param media 
/// 
/// @returns MipCmdResult
/// 
MipCmdResult prodEraseStorage(C::mip_interface& device, MediaSelector media)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, media);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_PROD_ERASE_STORAGE, buffer, serializer.offset);
}

void insert(MipSerializer& serializer, const LedControl& self)
{
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.primaryColor[i]);
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.altColor[i]);
    insert(serializer, self.act);
    insert(serializer, self.period);
}

void extract(MipSerializer& serializer, LedControl& self)
{
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.primaryColor[i]);
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.altColor[i]);
    extract(serializer, self.act);
    extract(serializer, self.period);
}

/// @brief This command allows direct control of the LED on the 3DM RTK. This command is only available in calibration mode or Production Test Mode.
/// 
/// @param primaryColor 
/// @param altColor 
/// @param act 
/// @param period 
/// 
/// @returns MipCmdResult
/// 
MipCmdResult ledControl(C::mip_interface& device, const uint8_t* primaryColor, const uint8_t* altColor, LedAction act, uint32_t period)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, primaryColor[i]);
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, altColor[i]);
    insert(serializer, act);
    insert(serializer, period);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_CONTROL, buffer, serializer.offset);
}

void insert(MipSerializer& serializer, const ModemHardReset& self)
{
    (void)serializer;
    (void)self;
}

void extract(MipSerializer& serializer, ModemHardReset& self)
{
    (void)serializer;
    (void)self;
}

/// @brief This command will clear the modem flash.  THIS MUST NOT BE DONE OFTEN AS IT CAN DAMAGE THE FLASH!
/// This command is only available in calibration mode.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult modemHardReset(C::mip_interface& device)
{
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_MODEM_HARD_RESET, NULL, 0);
}


} // namespace commands_rtk
} // namespace mscl

