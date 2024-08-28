
#include "commands_base.hpp"

#include <mip/mip_serialization.hpp>
#include <mip/mip_interface.h>

#include <assert.h>


namespace mip {
namespace C {
struct mip_interface;
} // namespace C

namespace commands_base {

using namespace ::mip::C;

////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

void BaseDeviceInfo::insert(Serializer& serializer) const
{
    serializer.insert(firmware_version);
    
    for(unsigned int i=0; i < 16; i++)
        serializer.insert(model_name[i]);
    
    for(unsigned int i=0; i < 16; i++)
        serializer.insert(model_number[i]);
    
    for(unsigned int i=0; i < 16; i++)
        serializer.insert(serial_number[i]);
    
    for(unsigned int i=0; i < 16; i++)
        serializer.insert(lot_number[i]);
    
    for(unsigned int i=0; i < 16; i++)
        serializer.insert(device_options[i]);
    
}
void BaseDeviceInfo::extract(Serializer& serializer)
{
    serializer.extract(firmware_version);
    
    for(unsigned int i=0; i < 16; i++)
        serializer.extract(model_name[i]);
    
    for(unsigned int i=0; i < 16; i++)
        serializer.extract(model_number[i]);
    
    for(unsigned int i=0; i < 16; i++)
        serializer.extract(serial_number[i]);
    
    for(unsigned int i=0; i < 16; i++)
        serializer.extract(lot_number[i]);
    
    for(unsigned int i=0; i < 16; i++)
        serializer.extract(device_options[i]);
    
}

void Ping::insert(Serializer& serializer) const
{
    (void)serializer;
}
void Ping::extract(Serializer& serializer)
{
    (void)serializer;
}

TypedResult<Ping> ping(C::mip_interface& device)
{
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_PING, NULL, 0);
}
void SetIdle::insert(Serializer& serializer) const
{
    (void)serializer;
}
void SetIdle::extract(Serializer& serializer)
{
    (void)serializer;
}

TypedResult<SetIdle> setIdle(C::mip_interface& device)
{
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SET_TO_IDLE, NULL, 0);
}
void GetDeviceInfo::insert(Serializer& serializer) const
{
    (void)serializer;
}
void GetDeviceInfo::extract(Serializer& serializer)
{
    (void)serializer;
}

void GetDeviceInfo::Response::insert(Serializer& serializer) const
{
    serializer.insert(device_info);
    
}
void GetDeviceInfo::Response::extract(Serializer& serializer)
{
    serializer.extract(device_info);
    
}

TypedResult<GetDeviceInfo> getDeviceInfo(C::mip_interface& device, BaseDeviceInfo* deviceInfoOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    TypedResult<GetDeviceInfo> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GET_DEVICE_INFO, NULL, 0, REPLY_DEVICE_INFO, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(deviceInfoOut);
        deserializer.extract(*deviceInfoOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
void GetDeviceDescriptors::insert(Serializer& serializer) const
{
    (void)serializer;
}
void GetDeviceDescriptors::extract(Serializer& serializer)
{
    (void)serializer;
}

void GetDeviceDescriptors::Response::insert(Serializer& serializer) const
{
    for(unsigned int i=0; i < descriptors_count; i++)
        serializer.insert(descriptors[i]);
    
}
void GetDeviceDescriptors::Response::extract(Serializer& serializer)
{
    for(descriptors_count = 0; (descriptors_count < sizeof(descriptors)/sizeof(descriptors[0])) && (serializer.remaining() > 0); (descriptors_count)++)
        serializer.extract(descriptors[descriptors_count]);
    
}

TypedResult<GetDeviceDescriptors> getDeviceDescriptors(C::mip_interface& device, uint16_t* descriptorsOut, size_t descriptorsOutMax, uint8_t* descriptorsOutCount)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    TypedResult<GetDeviceDescriptors> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GET_DEVICE_DESCRIPTORS, NULL, 0, REPLY_DEVICE_DESCRIPTORS, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        for(*descriptorsOutCount = 0; (*descriptorsOutCount < descriptorsOutMax) && (deserializer.remaining() > 0); (*descriptorsOutCount)++)
            deserializer.extract(descriptorsOut[*descriptorsOutCount]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
void BuiltInTest::insert(Serializer& serializer) const
{
    (void)serializer;
}
void BuiltInTest::extract(Serializer& serializer)
{
    (void)serializer;
}

void BuiltInTest::Response::insert(Serializer& serializer) const
{
    serializer.insert(result);
    
}
void BuiltInTest::Response::extract(Serializer& serializer)
{
    serializer.extract(result);
    
}

TypedResult<BuiltInTest> builtInTest(C::mip_interface& device, uint32_t* resultOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    TypedResult<BuiltInTest> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_BUILT_IN_TEST, NULL, 0, REPLY_BUILT_IN_TEST, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(resultOut);
        deserializer.extract(*resultOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
void Resume::insert(Serializer& serializer) const
{
    (void)serializer;
}
void Resume::extract(Serializer& serializer)
{
    (void)serializer;
}

TypedResult<Resume> resume(C::mip_interface& device)
{
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_RESUME, NULL, 0);
}
void GetExtendedDescriptors::insert(Serializer& serializer) const
{
    (void)serializer;
}
void GetExtendedDescriptors::extract(Serializer& serializer)
{
    (void)serializer;
}

void GetExtendedDescriptors::Response::insert(Serializer& serializer) const
{
    for(unsigned int i=0; i < descriptors_count; i++)
        serializer.insert(descriptors[i]);
    
}
void GetExtendedDescriptors::Response::extract(Serializer& serializer)
{
    for(descriptors_count = 0; (descriptors_count < sizeof(descriptors)/sizeof(descriptors[0])) && (serializer.remaining() > 0); (descriptors_count)++)
        serializer.extract(descriptors[descriptors_count]);
    
}

TypedResult<GetExtendedDescriptors> getExtendedDescriptors(C::mip_interface& device, uint16_t* descriptorsOut, size_t descriptorsOutMax, uint8_t* descriptorsOutCount)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    TypedResult<GetExtendedDescriptors> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GET_EXTENDED_DESCRIPTORS, NULL, 0, REPLY_GET_EXTENDED_DESCRIPTORS, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        for(*descriptorsOutCount = 0; (*descriptorsOutCount < descriptorsOutMax) && (deserializer.remaining() > 0); (*descriptorsOutCount)++)
            deserializer.extract(descriptorsOut[*descriptorsOutCount]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
void ContinuousBit::insert(Serializer& serializer) const
{
    (void)serializer;
}
void ContinuousBit::extract(Serializer& serializer)
{
    (void)serializer;
}

void ContinuousBit::Response::insert(Serializer& serializer) const
{
    for(unsigned int i=0; i < 16; i++)
        serializer.insert(result[i]);
    
}
void ContinuousBit::Response::extract(Serializer& serializer)
{
    for(unsigned int i=0; i < 16; i++)
        serializer.extract(result[i]);
    
}

TypedResult<ContinuousBit> continuousBit(C::mip_interface& device, uint8_t* resultOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    TypedResult<ContinuousBit> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_CONTINUOUS_BIT, NULL, 0, REPLY_CONTINUOUS_BIT, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(resultOut);
        for(unsigned int i=0; i < 16; i++)
            deserializer.extract(resultOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
void CommSpeed::insert(Serializer& serializer) const
{
    serializer.insert(function);
    
    serializer.insert(port);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.insert(baud);
        
    }
}
void CommSpeed::extract(Serializer& serializer)
{
    serializer.extract(function);
    
    serializer.extract(port);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.extract(baud);
        
    }
}

void CommSpeed::Response::insert(Serializer& serializer) const
{
    serializer.insert(port);
    
    serializer.insert(baud);
    
}
void CommSpeed::Response::extract(Serializer& serializer)
{
    serializer.extract(port);
    
    serializer.extract(baud);
    
}

TypedResult<CommSpeed> writeCommSpeed(C::mip_interface& device, uint8_t port, uint32_t baud)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::WRITE);
    serializer.insert(port);
    
    serializer.insert(baud);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_COMM_SPEED, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<CommSpeed> readCommSpeed(C::mip_interface& device, uint8_t port, uint32_t* baudOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::READ);
    serializer.insert(port);
    
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<CommSpeed> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_COMM_SPEED, buffer, (uint8_t)serializer.usedLength(), REPLY_COMM_SPEED, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        deserializer.extract(port);
        
        assert(baudOut);
        deserializer.extract(*baudOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<CommSpeed> saveCommSpeed(C::mip_interface& device, uint8_t port)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::SAVE);
    serializer.insert(port);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_COMM_SPEED, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<CommSpeed> loadCommSpeed(C::mip_interface& device, uint8_t port)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::LOAD);
    serializer.insert(port);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_COMM_SPEED, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<CommSpeed> defaultCommSpeed(C::mip_interface& device, uint8_t port)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::RESET);
    serializer.insert(port);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_COMM_SPEED, buffer, (uint8_t)serializer.usedLength());
}
void GpsTimeUpdate::insert(Serializer& serializer) const
{
    serializer.insert(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.insert(field_id);
        
        serializer.insert(value);
        
    }
}
void GpsTimeUpdate::extract(Serializer& serializer)
{
    serializer.extract(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.extract(field_id);
        
        serializer.extract(value);
        
    }
}

TypedResult<GpsTimeUpdate> writeGpsTimeUpdate(C::mip_interface& device, GpsTimeUpdate::FieldId fieldId, uint32_t value)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::WRITE);
    serializer.insert(fieldId);
    
    serializer.insert(value);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GPS_TIME_UPDATE, buffer, (uint8_t)serializer.usedLength());
}
void SoftReset::insert(Serializer& serializer) const
{
    (void)serializer;
}
void SoftReset::extract(Serializer& serializer)
{
    (void)serializer;
}

TypedResult<SoftReset> softReset(C::mip_interface& device)
{
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SOFT_RESET, NULL, 0);
}

} // namespace commands_base
} // namespace mip

