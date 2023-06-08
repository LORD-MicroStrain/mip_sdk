
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

void insert(Serializer& serializer, const EcefPos::Response& self)
{
    insert(serializer, self.time);
    
    insert(serializer, self.sensor_id);
    
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.position[i]);
    
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.uncertainty[i]);
    
    insert(serializer, self.valid_flags);
    
}
void extract(Serializer& serializer, EcefPos::Response& self)
{
    extract(serializer, self.time);
    
    extract(serializer, self.sensor_id);
    
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.position[i]);
    
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.uncertainty[i]);
    
    extract(serializer, self.valid_flags);
    
}

CmdResult ecefPos(C::mip_interface& device, const Time& time, uint8_t sensorId, const double* position, const float* uncertainty, EcefPos::ValidFlags validFlags, Time* timeOut, uint8_t* sensorIdOut, double* positionOut, float* uncertaintyOut, EcefPos::ValidFlags* validFlagsOut)
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
    
    uint8_t responseLength = sizeof(buffer);
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_ECEF_POS, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_ECEF_POS, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(timeOut);
        extract(deserializer, *timeOut);
        
        assert(sensorIdOut);
        extract(deserializer, *sensorIdOut);
        
        assert(positionOut || (3 == 0));
        for(unsigned int i=0; i < 3; i++)
            extract(deserializer, positionOut[i]);
        
        assert(uncertaintyOut || (3 == 0));
        for(unsigned int i=0; i < 3; i++)
            extract(deserializer, uncertaintyOut[i]);
        
        assert(validFlagsOut);
        extract(deserializer, *validFlagsOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
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

void insert(Serializer& serializer, const LlhPos::Response& self)
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
void extract(Serializer& serializer, LlhPos::Response& self)
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

CmdResult llhPos(C::mip_interface& device, const Time& time, uint8_t sensorId, double latitude, double longitude, double height, const float* uncertainty, LlhPos::ValidFlags validFlags, Time* timeOut, uint8_t* sensorIdOut, double* latitudeOut, double* longitudeOut, double* heightOut, float* uncertaintyOut, LlhPos::ValidFlags* validFlagsOut)
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
    
    uint8_t responseLength = sizeof(buffer);
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_LLH_POS, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_LLH_POS, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(timeOut);
        extract(deserializer, *timeOut);
        
        assert(sensorIdOut);
        extract(deserializer, *sensorIdOut);
        
        assert(latitudeOut);
        extract(deserializer, *latitudeOut);
        
        assert(longitudeOut);
        extract(deserializer, *longitudeOut);
        
        assert(heightOut);
        extract(deserializer, *heightOut);
        
        assert(uncertaintyOut || (3 == 0));
        for(unsigned int i=0; i < 3; i++)
            extract(deserializer, uncertaintyOut[i]);
        
        assert(validFlagsOut);
        extract(deserializer, *validFlagsOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
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

void insert(Serializer& serializer, const VehicleFixedFrameVelocity::Response& self)
{
    insert(serializer, self.time);
    
    insert(serializer, self.sensor_id);
    
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.velocity[i]);
    
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.uncertainty[i]);
    
    insert(serializer, self.valid_flags);
    
}
void extract(Serializer& serializer, VehicleFixedFrameVelocity::Response& self)
{
    extract(serializer, self.time);
    
    extract(serializer, self.sensor_id);
    
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.velocity[i]);
    
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.uncertainty[i]);
    
    extract(serializer, self.valid_flags);
    
}

CmdResult vehicleFixedFrameVelocity(C::mip_interface& device, const Time& time, uint8_t sensorId, const float* velocity, const float* uncertainty, VehicleFixedFrameVelocity::ValidFlags validFlags, Time* timeOut, uint8_t* sensorIdOut, float* velocityOut, float* uncertaintyOut, VehicleFixedFrameVelocity::ValidFlags* validFlagsOut)
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
    
    uint8_t responseLength = sizeof(buffer);
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_ODOM_VEL, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_ODOM_VEL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(timeOut);
        extract(deserializer, *timeOut);
        
        assert(sensorIdOut);
        extract(deserializer, *sensorIdOut);
        
        assert(velocityOut || (3 == 0));
        for(unsigned int i=0; i < 3; i++)
            extract(deserializer, velocityOut[i]);
        
        assert(uncertaintyOut || (3 == 0));
        for(unsigned int i=0; i < 3; i++)
            extract(deserializer, uncertaintyOut[i]);
        
        assert(validFlagsOut);
        extract(deserializer, *validFlagsOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
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

void insert(Serializer& serializer, const TrueHeading::Response& self)
{
    insert(serializer, self.time);
    
    insert(serializer, self.sensor_id);
    
    insert(serializer, self.heading);
    
    insert(serializer, self.uncertainty);
    
    insert(serializer, self.valid_flags);
    
}
void extract(Serializer& serializer, TrueHeading::Response& self)
{
    extract(serializer, self.time);
    
    extract(serializer, self.sensor_id);
    
    extract(serializer, self.heading);
    
    extract(serializer, self.uncertainty);
    
    extract(serializer, self.valid_flags);
    
}

CmdResult trueHeading(C::mip_interface& device, const Time& time, uint8_t sensorId, float heading, float uncertainty, uint16_t validFlags, Time* timeOut, uint8_t* sensorIdOut, float* headingOut, float* uncertaintyOut, uint16_t* validFlagsOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, time);
    
    insert(serializer, sensorId);
    
    insert(serializer, heading);
    
    insert(serializer, uncertainty);
    
    insert(serializer, validFlags);
    
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_HEADING_TRUE, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_HEADING_TRUE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(timeOut);
        extract(deserializer, *timeOut);
        
        assert(sensorIdOut);
        extract(deserializer, *sensorIdOut);
        
        assert(headingOut);
        extract(deserializer, *headingOut);
        
        assert(uncertaintyOut);
        extract(deserializer, *uncertaintyOut);
        
        assert(validFlagsOut);
        extract(deserializer, *validFlagsOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
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

} // namespace commands_aiding
} // namespace mip

