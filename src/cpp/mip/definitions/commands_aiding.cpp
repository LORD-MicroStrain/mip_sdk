
#include "commands_aiding.hpp"

#include <mip/mip_serialization.hpp>
#include <mip/mip_interface.h>

#include <assert.h>


namespace mip {
namespace C {
struct mip_interface;
} // namespace C

namespace commands_aiding {

using namespace ::mip::C;

////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

void Time::insert(Serializer& serializer) const
{
    serializer.insert(timebase);
    
    serializer.insert(reserved);
    
    serializer.insert(nanoseconds);
    
}
void Time::extract(Serializer& serializer)
{
    serializer.extract(timebase);
    
    serializer.extract(reserved);
    
    serializer.extract(nanoseconds);
    
}

void FrameConfig::insert(Serializer& serializer) const
{
    serializer.insert(function);
    
    serializer.insert(frame_id);
    
    if( function == FunctionSelector::WRITE || function == FunctionSelector::READ )
    {
        serializer.insert(format);
        
    }
    if( function == FunctionSelector::WRITE )
    {
        serializer.insert(tracking_enabled);
        
        serializer.insert(translation);
        
        if( format == FrameConfig::Format::EULER )
        {
            serializer.insert(rotation.euler);
            
        }
        if( format == FrameConfig::Format::QUATERNION )
        {
            serializer.insert(rotation.quaternion);
            
        }
    }
}
void FrameConfig::extract(Serializer& serializer)
{
    serializer.extract(function);
    
    serializer.extract(frame_id);
    
    if( function == FunctionSelector::WRITE || function == FunctionSelector::READ )
    {
        serializer.extract(format);
        
    }
    if( function == FunctionSelector::WRITE )
    {
        serializer.extract(tracking_enabled);
        
        serializer.extract(translation);
        
        if( format == FrameConfig::Format::EULER )
        {
            serializer.extract(rotation.euler);
            
        }
        if( format == FrameConfig::Format::QUATERNION )
        {
            serializer.extract(rotation.quaternion);
            
        }
    }
}

void FrameConfig::Response::insert(Serializer& serializer) const
{
    serializer.insert(frame_id);
    
    serializer.insert(format);
    
    serializer.insert(tracking_enabled);
    
    serializer.insert(translation);
    
    if( format == FrameConfig::Format::EULER )
    {
        serializer.insert(rotation.euler);
        
    }
    if( format == FrameConfig::Format::QUATERNION )
    {
        serializer.insert(rotation.quaternion);
        
    }
}
void FrameConfig::Response::extract(Serializer& serializer)
{
    serializer.extract(frame_id);
    
    serializer.extract(format);
    
    serializer.extract(tracking_enabled);
    
    serializer.extract(translation);
    
    if( format == FrameConfig::Format::EULER )
    {
        serializer.extract(rotation.euler);
        
    }
    if( format == FrameConfig::Format::QUATERNION )
    {
        serializer.extract(rotation.quaternion);
        
    }
}

TypedResult<FrameConfig> writeFrameConfig(C::mip_interface& device, uint8_t frameId, FrameConfig::Format format, bool trackingEnabled, const float* translation, const FrameConfig::Rotation& rotation)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::WRITE);
    serializer.insert(frameId);
    
    serializer.insert(format);
    
    serializer.insert(trackingEnabled);
    
    assert(translation);
    for(unsigned int i=0; i < 3; i++)
        serializer.insert(translation[i]);
    
    if( format == FrameConfig::Format::EULER )
    {
        serializer.insert(rotation.euler);
        
    }
    if( format == FrameConfig::Format::QUATERNION )
    {
        serializer.insert(rotation.quaternion);
        
    }
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_FRAME_CONFIG, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<FrameConfig> readFrameConfig(C::mip_interface& device, uint8_t frameId, FrameConfig::Format format, bool* trackingEnabledOut, float* translationOut, FrameConfig::Rotation* rotationOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::READ);
    serializer.insert(frameId);
    
    serializer.insert(format);
    
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<FrameConfig> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_FRAME_CONFIG, buffer, (uint8_t)serializer.usedLength(), REPLY_FRAME_CONFIG, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        deserializer.extract(frameId);
        
        deserializer.extract(format);
        
        assert(trackingEnabledOut);
        deserializer.extract(*trackingEnabledOut);
        
        assert(translationOut);
        for(unsigned int i=0; i < 3; i++)
            deserializer.extract(translationOut[i]);
        
        if( format == FrameConfig::Format::EULER )
        {
            deserializer.extract(rotationOut->euler);
            
        }
        if( format == FrameConfig::Format::QUATERNION )
        {
            deserializer.extract(rotationOut->quaternion);
            
        }
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<FrameConfig> saveFrameConfig(C::mip_interface& device, uint8_t frameId)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::SAVE);
    serializer.insert(frameId);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_FRAME_CONFIG, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<FrameConfig> loadFrameConfig(C::mip_interface& device, uint8_t frameId)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::LOAD);
    serializer.insert(frameId);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_FRAME_CONFIG, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<FrameConfig> defaultFrameConfig(C::mip_interface& device, uint8_t frameId)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::RESET);
    serializer.insert(frameId);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_FRAME_CONFIG, buffer, (uint8_t)serializer.usedLength());
}
void EchoControl::insert(Serializer& serializer) const
{
    serializer.insert(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.insert(mode);
        
    }
}
void EchoControl::extract(Serializer& serializer)
{
    serializer.extract(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.extract(mode);
        
    }
}

void EchoControl::Response::insert(Serializer& serializer) const
{
    serializer.insert(mode);
    
}
void EchoControl::Response::extract(Serializer& serializer)
{
    serializer.extract(mode);
    
}

TypedResult<EchoControl> writeEchoControl(C::mip_interface& device, EchoControl::Mode mode)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::WRITE);
    serializer.insert(mode);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ECHO_CONTROL, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<EchoControl> readEchoControl(C::mip_interface& device, EchoControl::Mode* modeOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<EchoControl> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_ECHO_CONTROL, buffer, (uint8_t)serializer.usedLength(), REPLY_ECHO_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(modeOut);
        deserializer.extract(*modeOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<EchoControl> saveEchoControl(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ECHO_CONTROL, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<EchoControl> loadEchoControl(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ECHO_CONTROL, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<EchoControl> defaultEchoControl(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ECHO_CONTROL, buffer, (uint8_t)serializer.usedLength());
}
void PosEcef::insert(Serializer& serializer) const
{
    serializer.insert(time);
    
    serializer.insert(frame_id);
    
    serializer.insert(position);
    
    serializer.insert(uncertainty);
    
    serializer.insert(valid_flags);
    
}
void PosEcef::extract(Serializer& serializer)
{
    serializer.extract(time);
    
    serializer.extract(frame_id);
    
    serializer.extract(position);
    
    serializer.extract(uncertainty);
    
    serializer.extract(valid_flags);
    
}

TypedResult<PosEcef> posEcef(C::mip_interface& device, const Time& time, uint8_t frameId, const double* position, const float* uncertainty, PosEcef::ValidFlags validFlags)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(time);
    
    serializer.insert(frameId);
    
    assert(position);
    for(unsigned int i=0; i < 3; i++)
        serializer.insert(position[i]);
    
    assert(uncertainty);
    for(unsigned int i=0; i < 3; i++)
        serializer.insert(uncertainty[i]);
    
    serializer.insert(validFlags);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_POS_ECEF, buffer, (uint8_t)serializer.usedLength());
}
void PosLlh::insert(Serializer& serializer) const
{
    serializer.insert(time);
    
    serializer.insert(frame_id);
    
    serializer.insert(latitude);
    
    serializer.insert(longitude);
    
    serializer.insert(height);
    
    serializer.insert(uncertainty);
    
    serializer.insert(valid_flags);
    
}
void PosLlh::extract(Serializer& serializer)
{
    serializer.extract(time);
    
    serializer.extract(frame_id);
    
    serializer.extract(latitude);
    
    serializer.extract(longitude);
    
    serializer.extract(height);
    
    serializer.extract(uncertainty);
    
    serializer.extract(valid_flags);
    
}

TypedResult<PosLlh> posLlh(C::mip_interface& device, const Time& time, uint8_t frameId, double latitude, double longitude, double height, const float* uncertainty, PosLlh::ValidFlags validFlags)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(time);
    
    serializer.insert(frameId);
    
    serializer.insert(latitude);
    
    serializer.insert(longitude);
    
    serializer.insert(height);
    
    assert(uncertainty);
    for(unsigned int i=0; i < 3; i++)
        serializer.insert(uncertainty[i]);
    
    serializer.insert(validFlags);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_POS_LLH, buffer, (uint8_t)serializer.usedLength());
}
void HeightAboveEllipsoid::insert(Serializer& serializer) const
{
    serializer.insert(time);
    
    serializer.insert(frame_id);
    
    serializer.insert(height);
    
    serializer.insert(uncertainty);
    
    serializer.insert(valid_flags);
    
}
void HeightAboveEllipsoid::extract(Serializer& serializer)
{
    serializer.extract(time);
    
    serializer.extract(frame_id);
    
    serializer.extract(height);
    
    serializer.extract(uncertainty);
    
    serializer.extract(valid_flags);
    
}

TypedResult<HeightAboveEllipsoid> heightAboveEllipsoid(C::mip_interface& device, const Time& time, uint8_t frameId, float height, float uncertainty, uint16_t validFlags)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(time);
    
    serializer.insert(frameId);
    
    serializer.insert(height);
    
    serializer.insert(uncertainty);
    
    serializer.insert(validFlags);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_HEIGHT_ABOVE_ELLIPSOID, buffer, (uint8_t)serializer.usedLength());
}
void VelEcef::insert(Serializer& serializer) const
{
    serializer.insert(time);
    
    serializer.insert(frame_id);
    
    serializer.insert(velocity);
    
    serializer.insert(uncertainty);
    
    serializer.insert(valid_flags);
    
}
void VelEcef::extract(Serializer& serializer)
{
    serializer.extract(time);
    
    serializer.extract(frame_id);
    
    serializer.extract(velocity);
    
    serializer.extract(uncertainty);
    
    serializer.extract(valid_flags);
    
}

TypedResult<VelEcef> velEcef(C::mip_interface& device, const Time& time, uint8_t frameId, const float* velocity, const float* uncertainty, VelEcef::ValidFlags validFlags)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(time);
    
    serializer.insert(frameId);
    
    assert(velocity);
    for(unsigned int i=0; i < 3; i++)
        serializer.insert(velocity[i]);
    
    assert(uncertainty);
    for(unsigned int i=0; i < 3; i++)
        serializer.insert(uncertainty[i]);
    
    serializer.insert(validFlags);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_VEL_ECEF, buffer, (uint8_t)serializer.usedLength());
}
void VelNed::insert(Serializer& serializer) const
{
    serializer.insert(time);
    
    serializer.insert(frame_id);
    
    serializer.insert(velocity);
    
    serializer.insert(uncertainty);
    
    serializer.insert(valid_flags);
    
}
void VelNed::extract(Serializer& serializer)
{
    serializer.extract(time);
    
    serializer.extract(frame_id);
    
    serializer.extract(velocity);
    
    serializer.extract(uncertainty);
    
    serializer.extract(valid_flags);
    
}

TypedResult<VelNed> velNed(C::mip_interface& device, const Time& time, uint8_t frameId, const float* velocity, const float* uncertainty, VelNed::ValidFlags validFlags)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(time);
    
    serializer.insert(frameId);
    
    assert(velocity);
    for(unsigned int i=0; i < 3; i++)
        serializer.insert(velocity[i]);
    
    assert(uncertainty);
    for(unsigned int i=0; i < 3; i++)
        serializer.insert(uncertainty[i]);
    
    serializer.insert(validFlags);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_VEL_NED, buffer, (uint8_t)serializer.usedLength());
}
void VelBodyFrame::insert(Serializer& serializer) const
{
    serializer.insert(time);
    
    serializer.insert(frame_id);
    
    serializer.insert(velocity);
    
    serializer.insert(uncertainty);
    
    serializer.insert(valid_flags);
    
}
void VelBodyFrame::extract(Serializer& serializer)
{
    serializer.extract(time);
    
    serializer.extract(frame_id);
    
    serializer.extract(velocity);
    
    serializer.extract(uncertainty);
    
    serializer.extract(valid_flags);
    
}

TypedResult<VelBodyFrame> velBodyFrame(C::mip_interface& device, const Time& time, uint8_t frameId, const float* velocity, const float* uncertainty, VelBodyFrame::ValidFlags validFlags)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(time);
    
    serializer.insert(frameId);
    
    assert(velocity);
    for(unsigned int i=0; i < 3; i++)
        serializer.insert(velocity[i]);
    
    assert(uncertainty);
    for(unsigned int i=0; i < 3; i++)
        serializer.insert(uncertainty[i]);
    
    serializer.insert(validFlags);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_VEL_BODY_FRAME, buffer, (uint8_t)serializer.usedLength());
}
void HeadingTrue::insert(Serializer& serializer) const
{
    serializer.insert(time);
    
    serializer.insert(frame_id);
    
    serializer.insert(heading);
    
    serializer.insert(uncertainty);
    
    serializer.insert(valid_flags);
    
}
void HeadingTrue::extract(Serializer& serializer)
{
    serializer.extract(time);
    
    serializer.extract(frame_id);
    
    serializer.extract(heading);
    
    serializer.extract(uncertainty);
    
    serializer.extract(valid_flags);
    
}

TypedResult<HeadingTrue> headingTrue(C::mip_interface& device, const Time& time, uint8_t frameId, float heading, float uncertainty, uint16_t validFlags)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(time);
    
    serializer.insert(frameId);
    
    serializer.insert(heading);
    
    serializer.insert(uncertainty);
    
    serializer.insert(validFlags);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_HEADING_TRUE, buffer, (uint8_t)serializer.usedLength());
}
void MagneticField::insert(Serializer& serializer) const
{
    serializer.insert(time);
    
    serializer.insert(frame_id);
    
    serializer.insert(magnetic_field);
    
    serializer.insert(uncertainty);
    
    serializer.insert(valid_flags);
    
}
void MagneticField::extract(Serializer& serializer)
{
    serializer.extract(time);
    
    serializer.extract(frame_id);
    
    serializer.extract(magnetic_field);
    
    serializer.extract(uncertainty);
    
    serializer.extract(valid_flags);
    
}

TypedResult<MagneticField> magneticField(C::mip_interface& device, const Time& time, uint8_t frameId, const float* magneticField, const float* uncertainty, MagneticField::ValidFlags validFlags)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(time);
    
    serializer.insert(frameId);
    
    assert(magneticField);
    for(unsigned int i=0; i < 3; i++)
        serializer.insert(magneticField[i]);
    
    assert(uncertainty);
    for(unsigned int i=0; i < 3; i++)
        serializer.insert(uncertainty[i]);
    
    serializer.insert(validFlags);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_MAGNETIC_FIELD, buffer, (uint8_t)serializer.usedLength());
}
void Pressure::insert(Serializer& serializer) const
{
    serializer.insert(time);
    
    serializer.insert(frame_id);
    
    serializer.insert(pressure);
    
    serializer.insert(uncertainty);
    
    serializer.insert(valid_flags);
    
}
void Pressure::extract(Serializer& serializer)
{
    serializer.extract(time);
    
    serializer.extract(frame_id);
    
    serializer.extract(pressure);
    
    serializer.extract(uncertainty);
    
    serializer.extract(valid_flags);
    
}

TypedResult<Pressure> pressure(C::mip_interface& device, const Time& time, uint8_t frameId, float pressure, float uncertainty, uint16_t validFlags)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(time);
    
    serializer.insert(frameId);
    
    serializer.insert(pressure);
    
    serializer.insert(uncertainty);
    
    serializer.insert(validFlags);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_PRESSURE, buffer, (uint8_t)serializer.usedLength());
}

} // namespace commands_aiding
} // namespace mip

