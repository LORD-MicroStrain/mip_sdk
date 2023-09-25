
#include "commands_aiding.hpp"

#include "../utils/serialization.h"
#include "../mip_interface.h"

#include <assert.h>


namespace mip {
class Serializer;

namespace C {
struct mip_interface;
} // namespace C

namespace commands_aiding {

using ::mip::insert;
using ::mip::extract;
using namespace ::mip::C;

////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////

void insert(Serializer& serializer, const Time& self)
{
    insert(serializer, self.timebase);
    
    insert(serializer, self.reserved);
    
    insert(serializer, self.nanoseconds);
    
}
void extract(Serializer& serializer, Time& self)
{
    extract(serializer, self.timebase);
    
    extract(serializer, self.reserved);
    
    extract(serializer, self.nanoseconds);
    
}


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

void insert(Serializer& serializer, const SensorFrameMapping& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.sensor_id);
        
        insert(serializer, self.frame_id);
        
    }
}
void extract(Serializer& serializer, SensorFrameMapping& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        extract(serializer, self.sensor_id);
        
        extract(serializer, self.frame_id);
        
    }
}

void insert(Serializer& serializer, const SensorFrameMapping::Response& self)
{
    insert(serializer, self.sensor_id);
    
    insert(serializer, self.frame_id);
    
}
void extract(Serializer& serializer, SensorFrameMapping::Response& self)
{
    extract(serializer, self.sensor_id);
    
    extract(serializer, self.frame_id);
    
}

CmdResult writeSensorFrameMapping(C::mip_interface& device, uint8_t sensorId, uint8_t frameId)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, sensorId);
    
    insert(serializer, frameId);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR_FRAME_MAP, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult readSensorFrameMapping(C::mip_interface& device, uint8_t* sensorIdOut, uint8_t* frameIdOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_SENSOR_FRAME_MAP, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_SENSOR_FRAME_MAP, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(sensorIdOut);
        extract(deserializer, *sensorIdOut);
        
        assert(frameIdOut);
        extract(deserializer, *frameIdOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
CmdResult saveSensorFrameMapping(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR_FRAME_MAP, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult loadSensorFrameMapping(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR_FRAME_MAP, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult defaultSensorFrameMapping(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR_FRAME_MAP, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const ReferenceFrame& self)
{
    insert(serializer, self.function);
    
    insert(serializer, self.frame_id);
    
    if( self.function == FunctionSelector::WRITE || self.function == FunctionSelector::READ )
    {
        insert(serializer, self.format);
        
    }
    if( self.function == FunctionSelector::WRITE )
    {
        for(unsigned int i=0; i < 3; i++)
            insert(serializer, self.translation[i]);
        
        if( self.format == ReferenceFrame::Format::EULER )
        {
            insert(serializer, self.rotation.euler);
            
        }
        if( self.format == ReferenceFrame::Format::QUATERNION )
        {
            insert(serializer, self.rotation.quaternion);
            
        }
    }
}
void extract(Serializer& serializer, ReferenceFrame& self)
{
    extract(serializer, self.function);
    
    extract(serializer, self.frame_id);
    
    if( self.function == FunctionSelector::WRITE || self.function == FunctionSelector::READ )
    {
        extract(serializer, self.format);
        
    }
    if( self.function == FunctionSelector::WRITE )
    {
        for(unsigned int i=0; i < 3; i++)
            extract(serializer, self.translation[i]);
        
        if( self.format == ReferenceFrame::Format::EULER )
        {
            extract(serializer, self.rotation.euler);
            
        }
        if( self.format == ReferenceFrame::Format::QUATERNION )
        {
            extract(serializer, self.rotation.quaternion);
            
        }
    }
}

void insert(Serializer& serializer, const ReferenceFrame::Response& self)
{
    insert(serializer, self.frame_id);
    
    insert(serializer, self.format);
    
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.translation[i]);
    
    if( self.format == ReferenceFrame::Format::EULER )
    {
        insert(serializer, self.rotation.euler);
        
    }
    if( self.format == ReferenceFrame::Format::QUATERNION )
    {
        insert(serializer, self.rotation.quaternion);
        
    }
}
void extract(Serializer& serializer, ReferenceFrame::Response& self)
{
    extract(serializer, self.frame_id);
    
    extract(serializer, self.format);
    
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.translation[i]);
    
    if( self.format == ReferenceFrame::Format::EULER )
    {
        extract(serializer, self.rotation.euler);
        
    }
    if( self.format == ReferenceFrame::Format::QUATERNION )
    {
        extract(serializer, self.rotation.quaternion);
        
    }
}

CmdResult writeReferenceFrame(C::mip_interface& device, uint8_t frameId, ReferenceFrame::Format format, const float* translation, const ReferenceFrame::Rotation& rotation)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, frameId);
    
    insert(serializer, format);
    
    assert(translation || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, translation[i]);
    
    if( format == ReferenceFrame::Format::EULER )
    {
        insert(serializer, rotation.euler);
        
    }
    if( format == ReferenceFrame::Format::QUATERNION )
    {
        insert(serializer, rotation.quaternion);
        
    }
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_FRAME_CONFIG, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult readReferenceFrame(C::mip_interface& device, uint8_t frameId, ReferenceFrame::Format format, float* translationOut, ReferenceFrame::Rotation* rotationOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    insert(serializer, frameId);
    
    insert(serializer, format);
    
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_FRAME_CONFIG, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_FRAME_CONFIG, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        extract(deserializer, frameId);
        
        extract(deserializer, format);
        
        assert(translationOut || (3 == 0));
        for(unsigned int i=0; i < 3; i++)
            extract(deserializer, translationOut[i]);
        
        if( format == ReferenceFrame::Format::EULER )
        {
            extract(deserializer, rotationOut->euler);
            
        }
        if( format == ReferenceFrame::Format::QUATERNION )
        {
            extract(deserializer, rotationOut->quaternion);
            
        }
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
CmdResult saveReferenceFrame(C::mip_interface& device, uint8_t frameId)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    insert(serializer, frameId);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_FRAME_CONFIG, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult loadReferenceFrame(C::mip_interface& device, uint8_t frameId)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    insert(serializer, frameId);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_FRAME_CONFIG, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult defaultReferenceFrame(C::mip_interface& device, uint8_t frameId)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    insert(serializer, frameId);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_FRAME_CONFIG, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const AidingEchoControl& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.mode);
        
    }
}
void extract(Serializer& serializer, AidingEchoControl& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        extract(serializer, self.mode);
        
    }
}

void insert(Serializer& serializer, const AidingEchoControl::Response& self)
{
    insert(serializer, self.mode);
    
}
void extract(Serializer& serializer, AidingEchoControl::Response& self)
{
    extract(serializer, self.mode);
    
}

CmdResult writeAidingEchoControl(C::mip_interface& device, AidingEchoControl::Mode mode)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, mode);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ECHO_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult readAidingEchoControl(C::mip_interface& device, AidingEchoControl::Mode* modeOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_ECHO_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_ECHO_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(modeOut);
        extract(deserializer, *modeOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
CmdResult saveAidingEchoControl(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ECHO_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult loadAidingEchoControl(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ECHO_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult defaultAidingEchoControl(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ECHO_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const EcefPos& self)
{
    insert(serializer, self.time);
    
    insert(serializer, self.sensor_id);
    
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.position[i]);
    
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.uncertainty[i]);
    
    insert(serializer, self.valid_flags);
    
}
void extract(Serializer& serializer, EcefPos& self)
{
    extract(serializer, self.time);
    
    extract(serializer, self.sensor_id);
    
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.position[i]);
    
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.uncertainty[i]);
    
    extract(serializer, self.valid_flags);
    
}

CmdResult ecefPos(C::mip_interface& device, const Time& time, uint8_t sensorId, const double* position, const float* uncertainty, EcefPos::ValidFlags validFlags)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, time);
    
    insert(serializer, sensorId);
    
    assert(position || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, position[i]);
    
    assert(uncertainty || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, uncertainty[i]);
    
    insert(serializer, validFlags);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_POS_ECEF, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const LlhPos& self)
{
    insert(serializer, self.time);
    
    insert(serializer, self.sensor_id);
    
    insert(serializer, self.latitude);
    
    insert(serializer, self.longitude);
    
    insert(serializer, self.height);
    
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.uncertainty[i]);
    
    insert(serializer, self.valid_flags);
    
}
void extract(Serializer& serializer, LlhPos& self)
{
    extract(serializer, self.time);
    
    extract(serializer, self.sensor_id);
    
    extract(serializer, self.latitude);
    
    extract(serializer, self.longitude);
    
    extract(serializer, self.height);
    
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.uncertainty[i]);
    
    extract(serializer, self.valid_flags);
    
}

CmdResult llhPos(C::mip_interface& device, const Time& time, uint8_t sensorId, double latitude, double longitude, double height, const float* uncertainty, LlhPos::ValidFlags validFlags)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, time);
    
    insert(serializer, sensorId);
    
    insert(serializer, latitude);
    
    insert(serializer, longitude);
    
    insert(serializer, height);
    
    assert(uncertainty || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, uncertainty[i]);
    
    insert(serializer, validFlags);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_POS_LLH, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const EcefVel& self)
{
    insert(serializer, self.time);
    
    insert(serializer, self.sensor_id);
    
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.velocity[i]);
    
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.uncertainty[i]);
    
    insert(serializer, self.valid_flags);
    
}
void extract(Serializer& serializer, EcefVel& self)
{
    extract(serializer, self.time);
    
    extract(serializer, self.sensor_id);
    
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.velocity[i]);
    
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.uncertainty[i]);
    
    extract(serializer, self.valid_flags);
    
}

CmdResult ecefVel(C::mip_interface& device, const Time& time, uint8_t sensorId, const float* velocity, const float* uncertainty, EcefVel::ValidFlags validFlags)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, time);
    
    insert(serializer, sensorId);
    
    assert(velocity || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, velocity[i]);
    
    assert(uncertainty || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, uncertainty[i]);
    
    insert(serializer, validFlags);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_VEL_ECEF, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const NedVel& self)
{
    insert(serializer, self.time);
    
    insert(serializer, self.sensor_id);
    
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.velocity[i]);
    
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.uncertainty[i]);
    
    insert(serializer, self.valid_flags);
    
}
void extract(Serializer& serializer, NedVel& self)
{
    extract(serializer, self.time);
    
    extract(serializer, self.sensor_id);
    
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.velocity[i]);
    
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.uncertainty[i]);
    
    extract(serializer, self.valid_flags);
    
}

CmdResult nedVel(C::mip_interface& device, const Time& time, uint8_t sensorId, const float* velocity, const float* uncertainty, NedVel::ValidFlags validFlags)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, time);
    
    insert(serializer, sensorId);
    
    assert(velocity || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, velocity[i]);
    
    assert(uncertainty || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, uncertainty[i]);
    
    insert(serializer, validFlags);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_VEL_NED, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const VehicleFixedFrameVelocity& self)
{
    insert(serializer, self.time);
    
    insert(serializer, self.sensor_id);
    
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.velocity[i]);
    
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.uncertainty[i]);
    
    insert(serializer, self.valid_flags);
    
}
void extract(Serializer& serializer, VehicleFixedFrameVelocity& self)
{
    extract(serializer, self.time);
    
    extract(serializer, self.sensor_id);
    
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.velocity[i]);
    
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.uncertainty[i]);
    
    extract(serializer, self.valid_flags);
    
}

CmdResult vehicleFixedFrameVelocity(C::mip_interface& device, const Time& time, uint8_t sensorId, const float* velocity, const float* uncertainty, VehicleFixedFrameVelocity::ValidFlags validFlags)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, time);
    
    insert(serializer, sensorId);
    
    assert(velocity || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, velocity[i]);
    
    assert(uncertainty || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, uncertainty[i]);
    
    insert(serializer, validFlags);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_VEL_ODOM, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const TrueHeading& self)
{
    insert(serializer, self.time);
    
    insert(serializer, self.sensor_id);
    
    insert(serializer, self.heading);
    
    insert(serializer, self.uncertainty);
    
    insert(serializer, self.valid_flags);
    
}
void extract(Serializer& serializer, TrueHeading& self)
{
    extract(serializer, self.time);
    
    extract(serializer, self.sensor_id);
    
    extract(serializer, self.heading);
    
    extract(serializer, self.uncertainty);
    
    extract(serializer, self.valid_flags);
    
}

CmdResult trueHeading(C::mip_interface& device, const Time& time, uint8_t sensorId, float heading, float uncertainty, uint16_t validFlags)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, time);
    
    insert(serializer, sensorId);
    
    insert(serializer, heading);
    
    insert(serializer, uncertainty);
    
    insert(serializer, validFlags);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_HEADING_TRUE, buffer, (uint8_t)mip_serializer_length(&serializer));
}

} // namespace commands_aiding
} // namespace mip

