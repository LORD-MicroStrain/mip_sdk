
#include "commands_rtk.hpp"

#include <mip/mip_serialization.hpp>
#include <mip/mip_interface.h>

#include <assert.h>


namespace mip {
namespace C {
struct mip_interface;
} // namespace C

namespace commands_rtk {

using namespace ::mip::C;

////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

void GetStatusFlags::insert(Serializer& serializer) const
{
    (void)serializer;
}
void GetStatusFlags::extract(Serializer& serializer)
{
    (void)serializer;
}

void GetStatusFlags::Response::insert(Serializer& serializer) const
{
    serializer.insert(flags);
    
}
void GetStatusFlags::Response::extract(Serializer& serializer)
{
    serializer.extract(flags);
    
}

TypedResult<GetStatusFlags> getStatusFlags(C::mip_interface& device, GetStatusFlags::StatusFlags* flagsOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    TypedResult<GetStatusFlags> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GET_STATUS_FLAGS, NULL, 0, REPLY_GET_STATUS_FLAGS, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(flagsOut);
        deserializer.extract(*flagsOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
void GetImei::insert(Serializer& serializer) const
{
    (void)serializer;
}
void GetImei::extract(Serializer& serializer)
{
    (void)serializer;
}

void GetImei::Response::insert(Serializer& serializer) const
{
    for(unsigned int i=0; i < 32; i++)
        serializer.insert(IMEI[i]);
    
}
void GetImei::Response::extract(Serializer& serializer)
{
    for(unsigned int i=0; i < 32; i++)
        serializer.extract(IMEI[i]);
    
}

TypedResult<GetImei> getImei(C::mip_interface& device, char* imeiOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    TypedResult<GetImei> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GET_IMEI, NULL, 0, REPLY_GET_IMEI, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(imeiOut);
        for(unsigned int i=0; i < 32; i++)
            deserializer.extract(imeiOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
void GetImsi::insert(Serializer& serializer) const
{
    (void)serializer;
}
void GetImsi::extract(Serializer& serializer)
{
    (void)serializer;
}

void GetImsi::Response::insert(Serializer& serializer) const
{
    for(unsigned int i=0; i < 32; i++)
        serializer.insert(IMSI[i]);
    
}
void GetImsi::Response::extract(Serializer& serializer)
{
    for(unsigned int i=0; i < 32; i++)
        serializer.extract(IMSI[i]);
    
}

TypedResult<GetImsi> getImsi(C::mip_interface& device, char* imsiOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    TypedResult<GetImsi> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GET_IMSI, NULL, 0, REPLY_GET_IMSI, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(imsiOut);
        for(unsigned int i=0; i < 32; i++)
            deserializer.extract(imsiOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
void GetIccid::insert(Serializer& serializer) const
{
    (void)serializer;
}
void GetIccid::extract(Serializer& serializer)
{
    (void)serializer;
}

void GetIccid::Response::insert(Serializer& serializer) const
{
    for(unsigned int i=0; i < 32; i++)
        serializer.insert(ICCID[i]);
    
}
void GetIccid::Response::extract(Serializer& serializer)
{
    for(unsigned int i=0; i < 32; i++)
        serializer.extract(ICCID[i]);
    
}

TypedResult<GetIccid> getIccid(C::mip_interface& device, char* iccidOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    TypedResult<GetIccid> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GET_ICCID, NULL, 0, REPLY_GET_ICCID, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(iccidOut);
        for(unsigned int i=0; i < 32; i++)
            deserializer.extract(iccidOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
void ConnectedDeviceType::insert(Serializer& serializer) const
{
    serializer.insert(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.insert(devType);
        
    }
}
void ConnectedDeviceType::extract(Serializer& serializer)
{
    serializer.extract(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.extract(devType);
        
    }
}

void ConnectedDeviceType::Response::insert(Serializer& serializer) const
{
    serializer.insert(devType);
    
}
void ConnectedDeviceType::Response::extract(Serializer& serializer)
{
    serializer.extract(devType);
    
}

TypedResult<ConnectedDeviceType> writeConnectedDeviceType(C::mip_interface& device, ConnectedDeviceType::Type devtype)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::WRITE);
    serializer.insert(devtype);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_CONNECTED_DEVICE_TYPE, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<ConnectedDeviceType> readConnectedDeviceType(C::mip_interface& device, ConnectedDeviceType::Type* devtypeOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<ConnectedDeviceType> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_CONNECTED_DEVICE_TYPE, buffer, (uint8_t)serializer.usedLength(), REPLY_CONNECTED_DEVICE_TYPE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(devtypeOut);
        deserializer.extract(*devtypeOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<ConnectedDeviceType> saveConnectedDeviceType(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_CONNECTED_DEVICE_TYPE, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<ConnectedDeviceType> loadConnectedDeviceType(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_CONNECTED_DEVICE_TYPE, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<ConnectedDeviceType> defaultConnectedDeviceType(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_CONNECTED_DEVICE_TYPE, buffer, (uint8_t)serializer.usedLength());
}
void GetActCode::insert(Serializer& serializer) const
{
    (void)serializer;
}
void GetActCode::extract(Serializer& serializer)
{
    (void)serializer;
}

void GetActCode::Response::insert(Serializer& serializer) const
{
    for(unsigned int i=0; i < 32; i++)
        serializer.insert(ActivationCode[i]);
    
}
void GetActCode::Response::extract(Serializer& serializer)
{
    for(unsigned int i=0; i < 32; i++)
        serializer.extract(ActivationCode[i]);
    
}

TypedResult<GetActCode> getActCode(C::mip_interface& device, char* activationcodeOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    TypedResult<GetActCode> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GET_ACT_CODE, NULL, 0, REPLY_GET_ACT_CODE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(activationcodeOut);
        for(unsigned int i=0; i < 32; i++)
            deserializer.extract(activationcodeOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
void GetModemFirmwareVersion::insert(Serializer& serializer) const
{
    (void)serializer;
}
void GetModemFirmwareVersion::extract(Serializer& serializer)
{
    (void)serializer;
}

void GetModemFirmwareVersion::Response::insert(Serializer& serializer) const
{
    for(unsigned int i=0; i < 32; i++)
        serializer.insert(ModemFirmwareVersion[i]);
    
}
void GetModemFirmwareVersion::Response::extract(Serializer& serializer)
{
    for(unsigned int i=0; i < 32; i++)
        serializer.extract(ModemFirmwareVersion[i]);
    
}

TypedResult<GetModemFirmwareVersion> getModemFirmwareVersion(C::mip_interface& device, char* modemfirmwareversionOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    TypedResult<GetModemFirmwareVersion> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GET_MODEM_FIRMWARE_VERSION, NULL, 0, REPLY_GET_MODEM_FIRMWARE_VERSION, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(modemfirmwareversionOut);
        for(unsigned int i=0; i < 32; i++)
            deserializer.extract(modemfirmwareversionOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
void GetRssi::insert(Serializer& serializer) const
{
    (void)serializer;
}
void GetRssi::extract(Serializer& serializer)
{
    (void)serializer;
}

void GetRssi::Response::insert(Serializer& serializer) const
{
    serializer.insert(valid);
    
    serializer.insert(rssi);
    
    serializer.insert(signalQuality);
    
}
void GetRssi::Response::extract(Serializer& serializer)
{
    serializer.extract(valid);
    
    serializer.extract(rssi);
    
    serializer.extract(signalQuality);
    
}

TypedResult<GetRssi> getRssi(C::mip_interface& device, bool* validOut, int32_t* rssiOut, int32_t* signalqualityOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    TypedResult<GetRssi> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GET_RSSI, NULL, 0, REPLY_GET_RSSI, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(validOut);
        deserializer.extract(*validOut);
        
        assert(rssiOut);
        deserializer.extract(*rssiOut);
        
        assert(signalqualityOut);
        deserializer.extract(*signalqualityOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
void ServiceStatus::insert(Serializer& serializer) const
{
    serializer.insert(reserved1);
    
    serializer.insert(reserved2);
    
}
void ServiceStatus::extract(Serializer& serializer)
{
    serializer.extract(reserved1);
    
    serializer.extract(reserved2);
    
}

void ServiceStatus::Response::insert(Serializer& serializer) const
{
    serializer.insert(flags);
    
    serializer.insert(receivedBytes);
    
    serializer.insert(lastBytes);
    
    serializer.insert(lastBytesTime);
    
}
void ServiceStatus::Response::extract(Serializer& serializer)
{
    serializer.extract(flags);
    
    serializer.extract(receivedBytes);
    
    serializer.extract(lastBytes);
    
    serializer.extract(lastBytesTime);
    
}

TypedResult<ServiceStatus> serviceStatus(C::mip_interface& device, uint32_t reserved1, uint32_t reserved2, ServiceStatus::ServiceFlags* flagsOut, uint32_t* receivedbytesOut, uint32_t* lastbytesOut, uint64_t* lastbytestimeOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(reserved1);
    
    serializer.insert(reserved2);
    
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<ServiceStatus> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_SERVICE_STATUS, buffer, (uint8_t)serializer.usedLength(), REPLY_SERVICE_STATUS, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(flagsOut);
        deserializer.extract(*flagsOut);
        
        assert(receivedbytesOut);
        deserializer.extract(*receivedbytesOut);
        
        assert(lastbytesOut);
        deserializer.extract(*lastbytesOut);
        
        assert(lastbytestimeOut);
        deserializer.extract(*lastbytestimeOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
void ProdEraseStorage::insert(Serializer& serializer) const
{
    serializer.insert(media);
    
}
void ProdEraseStorage::extract(Serializer& serializer)
{
    serializer.extract(media);
    
}

TypedResult<ProdEraseStorage> prodEraseStorage(C::mip_interface& device, MediaSelector media)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(media);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_PROD_ERASE_STORAGE, buffer, (uint8_t)serializer.usedLength());
}
void LedControl::insert(Serializer& serializer) const
{
    for(unsigned int i=0; i < 3; i++)
        serializer.insert(primaryColor[i]);
    
    for(unsigned int i=0; i < 3; i++)
        serializer.insert(altColor[i]);
    
    serializer.insert(act);
    
    serializer.insert(period);
    
}
void LedControl::extract(Serializer& serializer)
{
    for(unsigned int i=0; i < 3; i++)
        serializer.extract(primaryColor[i]);
    
    for(unsigned int i=0; i < 3; i++)
        serializer.extract(altColor[i]);
    
    serializer.extract(act);
    
    serializer.extract(period);
    
}

TypedResult<LedControl> ledControl(C::mip_interface& device, const uint8_t* primarycolor, const uint8_t* altcolor, LedAction act, uint32_t period)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    assert(primarycolor);
    for(unsigned int i=0; i < 3; i++)
        serializer.insert(primarycolor[i]);
    
    assert(altcolor);
    for(unsigned int i=0; i < 3; i++)
        serializer.insert(altcolor[i]);
    
    serializer.insert(act);
    
    serializer.insert(period);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_CONTROL, buffer, (uint8_t)serializer.usedLength());
}
void ModemHardReset::insert(Serializer& serializer) const
{
    (void)serializer;
}
void ModemHardReset::extract(Serializer& serializer)
{
    (void)serializer;
}

TypedResult<ModemHardReset> modemHardReset(C::mip_interface& device)
{
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_MODEM_HARD_RESET, NULL, 0);
}

} // namespace commands_rtk
} // namespace mip

