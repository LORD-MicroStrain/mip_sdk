
#include "commands_filter.hpp"

#include "../../utils/serialization.h"
#include "../mip_interface.h"

#include <assert.h>


namespace mip {
class Serializer;

namespace C {
struct mip_interface;
} // namespace C

namespace commands_filter {

using ::mip::insert;
using ::mip::extract;
using namespace ::mip::C;

////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

void insert(Serializer& serializer, const Reset& self)
{
    (void)serializer;
    (void)self;
}
void extract(Serializer& serializer, Reset& self)
{
    (void)serializer;
    (void)self;
}

CmdResult reset(C::mip_interface& device)
{
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_RESET_FILTER, NULL, 0);
}
void insert(Serializer& serializer, const SetInitialAttitude& self)
{
    insert(serializer, self.roll);
    
    insert(serializer, self.pitch);
    
    insert(serializer, self.heading);
    
}
void extract(Serializer& serializer, SetInitialAttitude& self)
{
    extract(serializer, self.roll);
    
    extract(serializer, self.pitch);
    
    extract(serializer, self.heading);
    
}

CmdResult setInitialAttitude(C::mip_interface& device, float roll, float pitch, float heading)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, roll);
    
    insert(serializer, pitch);
    
    insert(serializer, heading);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SET_INITIAL_ATTITUDE, buffer, serializer.offset);
}
void insert(Serializer& serializer, const EstimationControl& self)
{
    insert(serializer, self.enable);
    
}
void extract(Serializer& serializer, EstimationControl& self)
{
    extract(serializer, self.enable);
    
}

CmdResult writeEstimationControl(C::mip_interface& device, EstimationControl::EnableFlags enable)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, enable);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ESTIMATION_CONTROL_FLAGS, buffer, serializer.offset);
}
CmdResult readEstimationControl(C::mip_interface& device, EstimationControl::EnableFlags* enableOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength;
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_ESTIMATION_CONTROL_FLAGS, buffer, serializer.offset, REPLY_ESTIMATION_CONTROL_FLAGS, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, sizeof(buffer));
        
        assert(enableOut);
        extract(deserializer, *enableOut);
        
        if( !deserializer.isOk() )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
CmdResult saveEstimationControl(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ESTIMATION_CONTROL_FLAGS, buffer, serializer.offset);
}
CmdResult loadEstimationControl(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ESTIMATION_CONTROL_FLAGS, buffer, serializer.offset);
}
CmdResult defaultEstimationControl(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ESTIMATION_CONTROL_FLAGS, buffer, serializer.offset);
}
void insert(Serializer& serializer, const ExternalGnssUpdate& self)
{
    insert(serializer, self.gps_time);
    
    insert(serializer, self.gps_week);
    
    insert(serializer, self.latitude);
    
    insert(serializer, self.longitude);
    
    insert(serializer, self.height);
    
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.velocity[i]);
    
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.pos_uncertainty[i]);
    
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.vel_uncertainty[i]);
    
}
void extract(Serializer& serializer, ExternalGnssUpdate& self)
{
    extract(serializer, self.gps_time);
    
    extract(serializer, self.gps_week);
    
    extract(serializer, self.latitude);
    
    extract(serializer, self.longitude);
    
    extract(serializer, self.height);
    
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.velocity[i]);
    
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.pos_uncertainty[i]);
    
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.vel_uncertainty[i]);
    
}

CmdResult externalGnssUpdate(C::mip_interface& device, double gpsTime, uint16_t gpsWeek, double latitude, double longitude, double height, const float* velocity, const float* posUncertainty, const float* velUncertainty)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, gpsTime);
    
    insert(serializer, gpsWeek);
    
    insert(serializer, latitude);
    
    insert(serializer, longitude);
    
    insert(serializer, height);
    
    assert(velocity);
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, velocity[i]);
    
    assert(posUncertainty);
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, posUncertainty[i]);
    
    assert(velUncertainty);
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, velUncertainty[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_EXTERNAL_GNSS_UPDATE, buffer, serializer.offset);
}
void insert(Serializer& serializer, const ExternalHeadingUpdate& self)
{
    insert(serializer, self.heading);
    
    insert(serializer, self.heading_uncertainty);
    
    insert(serializer, self.type);
    
}
void extract(Serializer& serializer, ExternalHeadingUpdate& self)
{
    extract(serializer, self.heading);
    
    extract(serializer, self.heading_uncertainty);
    
    extract(serializer, self.type);
    
}

CmdResult externalHeadingUpdate(C::mip_interface& device, float heading, float headingUncertainty, uint8_t type)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, heading);
    
    insert(serializer, headingUncertainty);
    
    insert(serializer, type);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_EXTERNAL_HEADING_UPDATE, buffer, serializer.offset);
}
void insert(Serializer& serializer, const ExternalHeadingUpdateWithTime& self)
{
    insert(serializer, self.gps_time);
    
    insert(serializer, self.gps_week);
    
    insert(serializer, self.heading);
    
    insert(serializer, self.heading_uncertainty);
    
    insert(serializer, self.type);
    
}
void extract(Serializer& serializer, ExternalHeadingUpdateWithTime& self)
{
    extract(serializer, self.gps_time);
    
    extract(serializer, self.gps_week);
    
    extract(serializer, self.heading);
    
    extract(serializer, self.heading_uncertainty);
    
    extract(serializer, self.type);
    
}

CmdResult externalHeadingUpdateWithTime(C::mip_interface& device, double gpsTime, uint16_t gpsWeek, float heading, float headingUncertainty, uint8_t type)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, gpsTime);
    
    insert(serializer, gpsWeek);
    
    insert(serializer, heading);
    
    insert(serializer, headingUncertainty);
    
    insert(serializer, type);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_EXTERNAL_HEADING_UPDATE_WITH_TIME, buffer, serializer.offset);
}
void insert(Serializer& serializer, const TareOrientation& self)
{
    insert(serializer, self.axes);
    
}
void extract(Serializer& serializer, TareOrientation& self)
{
    extract(serializer, self.axes);
    
}

CmdResult writeTareOrientation(C::mip_interface& device, TareOrientation::MipTareAxes axes)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, axes);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_TARE_ORIENTATION, buffer, serializer.offset);
}
CmdResult readTareOrientation(C::mip_interface& device, TareOrientation::MipTareAxes* axesOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength;
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_TARE_ORIENTATION, buffer, serializer.offset, REPLY_TARE_ORIENTATION, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, sizeof(buffer));
        
        assert(axesOut);
        extract(deserializer, *axesOut);
        
        if( !deserializer.isOk() )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
CmdResult saveTareOrientation(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_TARE_ORIENTATION, buffer, serializer.offset);
}
CmdResult loadTareOrientation(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_TARE_ORIENTATION, buffer, serializer.offset);
}
CmdResult defaultTareOrientation(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_TARE_ORIENTATION, buffer, serializer.offset);
}
void insert(Serializer& serializer, const SensorToVehicleRotationEuler& self)
{
    insert(serializer, self.roll);
    
    insert(serializer, self.pitch);
    
    insert(serializer, self.yaw);
    
}
void extract(Serializer& serializer, SensorToVehicleRotationEuler& self)
{
    extract(serializer, self.roll);
    
    extract(serializer, self.pitch);
    
    extract(serializer, self.yaw);
    
}

CmdResult writeSensorToVehicleRotationEuler(C::mip_interface& device, float roll, float pitch, float yaw)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, roll);
    
    insert(serializer, pitch);
    
    insert(serializer, yaw);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_ROTATION_EULER, buffer, serializer.offset);
}
CmdResult readSensorToVehicleRotationEuler(C::mip_interface& device, float* rollOut, float* pitchOut, float* yawOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength;
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_ROTATION_EULER, buffer, serializer.offset, REPLY_SENSOR2VEHICLE_ROTATION_EULER, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, sizeof(buffer));
        
        assert(rollOut);
        extract(deserializer, *rollOut);
        
        assert(pitchOut);
        extract(deserializer, *pitchOut);
        
        assert(yawOut);
        extract(deserializer, *yawOut);
        
        if( !deserializer.isOk() )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
CmdResult saveSensorToVehicleRotationEuler(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_ROTATION_EULER, buffer, serializer.offset);
}
CmdResult loadSensorToVehicleRotationEuler(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_ROTATION_EULER, buffer, serializer.offset);
}
CmdResult defaultSensorToVehicleRotationEuler(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_ROTATION_EULER, buffer, serializer.offset);
}
void insert(Serializer& serializer, const SensorToVehicleRotationDcm& self)
{
    for(unsigned int i=0; i < 9; i++)
        insert(serializer, self.dcm[i]);
    
}
void extract(Serializer& serializer, SensorToVehicleRotationDcm& self)
{
    for(unsigned int i=0; i < 9; i++)
        extract(serializer, self.dcm[i]);
    
}

CmdResult writeSensorToVehicleRotationDcm(C::mip_interface& device, const float* dcm)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    assert(dcm);
    for(unsigned int i=0; i < 9; i++)
        insert(serializer, dcm[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_ROTATION_DCM, buffer, serializer.offset);
}
CmdResult readSensorToVehicleRotationDcm(C::mip_interface& device, float* dcmOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength;
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_ROTATION_DCM, buffer, serializer.offset, REPLY_SENSOR2VEHICLE_ROTATION_DCM, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, sizeof(buffer));
        
        assert(dcmOut);
        for(unsigned int i=0; i < 9; i++)
            extract(deserializer, dcmOut[i]);
        
        if( !deserializer.isOk() )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
CmdResult saveSensorToVehicleRotationDcm(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_ROTATION_DCM, buffer, serializer.offset);
}
CmdResult loadSensorToVehicleRotationDcm(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_ROTATION_DCM, buffer, serializer.offset);
}
CmdResult defaultSensorToVehicleRotationDcm(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_ROTATION_DCM, buffer, serializer.offset);
}
void insert(Serializer& serializer, const SensorToVehicleRotationQuaternion& self)
{
    for(unsigned int i=0; i < 4; i++)
        insert(serializer, self.quat[i]);
    
}
void extract(Serializer& serializer, SensorToVehicleRotationQuaternion& self)
{
    for(unsigned int i=0; i < 4; i++)
        extract(serializer, self.quat[i]);
    
}

CmdResult writeSensorToVehicleRotationQuaternion(C::mip_interface& device, const float* quat)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    assert(quat);
    for(unsigned int i=0; i < 4; i++)
        insert(serializer, quat[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_ROTATION_QUATERNION, buffer, serializer.offset);
}
CmdResult readSensorToVehicleRotationQuaternion(C::mip_interface& device, float* quatOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength;
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_ROTATION_QUATERNION, buffer, serializer.offset, REPLY_SENSOR2VEHICLE_ROTATION_QUATERNION, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, sizeof(buffer));
        
        assert(quatOut);
        for(unsigned int i=0; i < 4; i++)
            extract(deserializer, quatOut[i]);
        
        if( !deserializer.isOk() )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
CmdResult saveSensorToVehicleRotationQuaternion(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_ROTATION_QUATERNION, buffer, serializer.offset);
}
CmdResult loadSensorToVehicleRotationQuaternion(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_ROTATION_QUATERNION, buffer, serializer.offset);
}
CmdResult defaultSensorToVehicleRotationQuaternion(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_ROTATION_QUATERNION, buffer, serializer.offset);
}
void insert(Serializer& serializer, const SensorToVehicleOffset& self)
{
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.offset[i]);
    
}
void extract(Serializer& serializer, SensorToVehicleOffset& self)
{
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.offset[i]);
    
}

CmdResult writeSensorToVehicleOffset(C::mip_interface& device, const float* offset)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    assert(offset);
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, offset[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_OFFSET, buffer, serializer.offset);
}
CmdResult readSensorToVehicleOffset(C::mip_interface& device, float* offsetOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength;
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_OFFSET, buffer, serializer.offset, REPLY_SENSOR2VEHICLE_OFFSET, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, sizeof(buffer));
        
        assert(offsetOut);
        for(unsigned int i=0; i < 3; i++)
            extract(deserializer, offsetOut[i]);
        
        if( !deserializer.isOk() )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
CmdResult saveSensorToVehicleOffset(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_OFFSET, buffer, serializer.offset);
}
CmdResult loadSensorToVehicleOffset(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_OFFSET, buffer, serializer.offset);
}
CmdResult defaultSensorToVehicleOffset(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_OFFSET, buffer, serializer.offset);
}
void insert(Serializer& serializer, const AntennaOffset& self)
{
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.offset[i]);
    
}
void extract(Serializer& serializer, AntennaOffset& self)
{
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.offset[i]);
    
}

CmdResult writeAntennaOffset(C::mip_interface& device, const float* offset)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    assert(offset);
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, offset[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ANTENNA_OFFSET, buffer, serializer.offset);
}
CmdResult readAntennaOffset(C::mip_interface& device, float* offsetOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength;
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_ANTENNA_OFFSET, buffer, serializer.offset, REPLY_ANTENNA_OFFSET, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, sizeof(buffer));
        
        assert(offsetOut);
        for(unsigned int i=0; i < 3; i++)
            extract(deserializer, offsetOut[i]);
        
        if( !deserializer.isOk() )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
CmdResult saveAntennaOffset(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ANTENNA_OFFSET, buffer, serializer.offset);
}
CmdResult loadAntennaOffset(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ANTENNA_OFFSET, buffer, serializer.offset);
}
CmdResult defaultAntennaOffset(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ANTENNA_OFFSET, buffer, serializer.offset);
}
void insert(Serializer& serializer, const GnssSource& self)
{
    insert(serializer, self.source);
    
}
void extract(Serializer& serializer, GnssSource& self)
{
    extract(serializer, self.source);
    
}

CmdResult writeGnssSource(C::mip_interface& device, GnssSource::Source source)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, source);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GNSS_SOURCE_CONTROL, buffer, serializer.offset);
}
CmdResult readGnssSource(C::mip_interface& device, GnssSource::Source* sourceOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength;
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GNSS_SOURCE_CONTROL, buffer, serializer.offset, REPLY_GNSS_SOURCE_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, sizeof(buffer));
        
        assert(sourceOut);
        extract(deserializer, *sourceOut);
        
        if( !deserializer.isOk() )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
CmdResult saveGnssSource(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GNSS_SOURCE_CONTROL, buffer, serializer.offset);
}
CmdResult loadGnssSource(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GNSS_SOURCE_CONTROL, buffer, serializer.offset);
}
CmdResult defaultGnssSource(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GNSS_SOURCE_CONTROL, buffer, serializer.offset);
}
void insert(Serializer& serializer, const HeadingSource& self)
{
    insert(serializer, self.source);
    
}
void extract(Serializer& serializer, HeadingSource& self)
{
    extract(serializer, self.source);
    
}

CmdResult writeHeadingSource(C::mip_interface& device, HeadingSource::Source source)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, source);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_HEADING_UPDATE_CONTROL, buffer, serializer.offset);
}
CmdResult readHeadingSource(C::mip_interface& device, HeadingSource::Source* sourceOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength;
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_HEADING_UPDATE_CONTROL, buffer, serializer.offset, REPLY_HEADING_UPDATE_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, sizeof(buffer));
        
        assert(sourceOut);
        extract(deserializer, *sourceOut);
        
        if( !deserializer.isOk() )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
CmdResult saveHeadingSource(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_HEADING_UPDATE_CONTROL, buffer, serializer.offset);
}
CmdResult loadHeadingSource(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_HEADING_UPDATE_CONTROL, buffer, serializer.offset);
}
CmdResult defaultHeadingSource(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_HEADING_UPDATE_CONTROL, buffer, serializer.offset);
}
void insert(Serializer& serializer, const AltitudeAiding& self)
{
    insert(serializer, self.aiding_selector);
    
}
void extract(Serializer& serializer, AltitudeAiding& self)
{
    extract(serializer, self.aiding_selector);
    
}

CmdResult writeAltitudeAiding(C::mip_interface& device, uint8_t aidingSelector)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, aidingSelector);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ALTITUDE_AIDING_CONTROL, buffer, serializer.offset);
}
CmdResult readAltitudeAiding(C::mip_interface& device, uint8_t* aidingSelectorOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength;
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_ALTITUDE_AIDING_CONTROL, buffer, serializer.offset, REPLY_ALTITUDE_AIDING_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, sizeof(buffer));
        
        assert(aidingSelectorOut);
        extract(deserializer, *aidingSelectorOut);
        
        if( !deserializer.isOk() )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
CmdResult saveAltitudeAiding(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ALTITUDE_AIDING_CONTROL, buffer, serializer.offset);
}
CmdResult loadAltitudeAiding(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ALTITUDE_AIDING_CONTROL, buffer, serializer.offset);
}
CmdResult defaultAltitudeAiding(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ALTITUDE_AIDING_CONTROL, buffer, serializer.offset);
}
void insert(Serializer& serializer, const AutoZupt& self)
{
    insert(serializer, self.enable);
    
    insert(serializer, self.threshold);
    
}
void extract(Serializer& serializer, AutoZupt& self)
{
    extract(serializer, self.enable);
    
    extract(serializer, self.threshold);
    
}

CmdResult writeAutoZupt(C::mip_interface& device, uint8_t enable, float threshold)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, enable);
    
    insert(serializer, threshold);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ZUPT_CONTROL, buffer, serializer.offset);
}
CmdResult readAutoZupt(C::mip_interface& device, uint8_t* enableOut, float* thresholdOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength;
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_ZUPT_CONTROL, buffer, serializer.offset, REPLY_ZUPT_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, sizeof(buffer));
        
        assert(enableOut);
        extract(deserializer, *enableOut);
        
        assert(thresholdOut);
        extract(deserializer, *thresholdOut);
        
        if( !deserializer.isOk() )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
CmdResult saveAutoZupt(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ZUPT_CONTROL, buffer, serializer.offset);
}
CmdResult loadAutoZupt(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ZUPT_CONTROL, buffer, serializer.offset);
}
CmdResult defaultAutoZupt(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ZUPT_CONTROL, buffer, serializer.offset);
}
void insert(Serializer& serializer, const AutoAngularZupt& self)
{
    insert(serializer, self.enable);
    
    insert(serializer, self.threshold);
    
}
void extract(Serializer& serializer, AutoAngularZupt& self)
{
    extract(serializer, self.enable);
    
    extract(serializer, self.threshold);
    
}

CmdResult writeAutoAngularZupt(C::mip_interface& device, uint8_t enable, float threshold)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, enable);
    
    insert(serializer, threshold);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ANGULAR_ZUPT_CONTROL, buffer, serializer.offset);
}
CmdResult readAutoAngularZupt(C::mip_interface& device, uint8_t* enableOut, float* thresholdOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength;
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_ANGULAR_ZUPT_CONTROL, buffer, serializer.offset, REPLY_ANGULAR_ZUPT_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, sizeof(buffer));
        
        assert(enableOut);
        extract(deserializer, *enableOut);
        
        assert(thresholdOut);
        extract(deserializer, *thresholdOut);
        
        if( !deserializer.isOk() )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
CmdResult saveAutoAngularZupt(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ANGULAR_ZUPT_CONTROL, buffer, serializer.offset);
}
CmdResult loadAutoAngularZupt(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ANGULAR_ZUPT_CONTROL, buffer, serializer.offset);
}
CmdResult defaultAutoAngularZupt(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ANGULAR_ZUPT_CONTROL, buffer, serializer.offset);
}
void insert(Serializer& serializer, const CommandedZupt& self)
{
    (void)serializer;
    (void)self;
}
void extract(Serializer& serializer, CommandedZupt& self)
{
    (void)serializer;
    (void)self;
}

CmdResult commandedZupt(C::mip_interface& device)
{
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_COMMANDED_ZUPT, NULL, 0);
}
void insert(Serializer& serializer, const CommandedAngularZupt& self)
{
    (void)serializer;
    (void)self;
}
void extract(Serializer& serializer, CommandedAngularZupt& self)
{
    (void)serializer;
    (void)self;
}

CmdResult commandedAngularZupt(C::mip_interface& device)
{
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_COMMANDED_ANGULAR_ZUPT, NULL, 0);
}
void insert(Serializer& serializer, const AidingMeasurementEnable& self)
{
    insert(serializer, self.aiding_source);
    
    insert(serializer, self.enable);
    
}
void extract(Serializer& serializer, AidingMeasurementEnable& self)
{
    extract(serializer, self.aiding_source);
    
    extract(serializer, self.enable);
    
}

CmdResult writeAidingMeasurementEnable(C::mip_interface& device, AidingMeasurementEnable::AidingSource aidingSource, bool enable)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, aidingSource);
    
    insert(serializer, enable);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_AIDING_MEASUREMENT_ENABLE, buffer, serializer.offset);
}
CmdResult readAidingMeasurementEnable(C::mip_interface& device, AidingMeasurementEnable::AidingSource aidingSource, bool* enableOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    insert(serializer, aidingSource);
    
    assert(serializer.isOk());
    
    uint8_t responseLength;
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_AIDING_MEASUREMENT_ENABLE, buffer, serializer.offset, REPLY_AIDING_MEASUREMENT_ENABLE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, sizeof(buffer));
        
        extract(deserializer, aidingSource);
        
        assert(enableOut);
        extract(deserializer, *enableOut);
        
        if( !deserializer.isOk() )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
CmdResult saveAidingMeasurementEnable(C::mip_interface& device, AidingMeasurementEnable::AidingSource aidingSource)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    insert(serializer, aidingSource);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_AIDING_MEASUREMENT_ENABLE, buffer, serializer.offset);
}
CmdResult loadAidingMeasurementEnable(C::mip_interface& device, AidingMeasurementEnable::AidingSource aidingSource)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    insert(serializer, aidingSource);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_AIDING_MEASUREMENT_ENABLE, buffer, serializer.offset);
}
CmdResult defaultAidingMeasurementEnable(C::mip_interface& device, AidingMeasurementEnable::AidingSource aidingSource)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    insert(serializer, aidingSource);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_AIDING_MEASUREMENT_ENABLE, buffer, serializer.offset);
}
void insert(Serializer& serializer, const Run& self)
{
    (void)serializer;
    (void)self;
}
void extract(Serializer& serializer, Run& self)
{
    (void)serializer;
    (void)self;
}

CmdResult run(C::mip_interface& device)
{
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_RUN, NULL, 0);
}
void insert(Serializer& serializer, const KinematicConstraint& self)
{
    insert(serializer, self.acceleration_constraint_selection);
    
    insert(serializer, self.velocity_constraint_selection);
    
    insert(serializer, self.angular_constraint_selection);
    
}
void extract(Serializer& serializer, KinematicConstraint& self)
{
    extract(serializer, self.acceleration_constraint_selection);
    
    extract(serializer, self.velocity_constraint_selection);
    
    extract(serializer, self.angular_constraint_selection);
    
}

CmdResult writeKinematicConstraint(C::mip_interface& device, uint8_t accelerationConstraintSelection, uint8_t velocityConstraintSelection, uint8_t angularConstraintSelection)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, accelerationConstraintSelection);
    
    insert(serializer, velocityConstraintSelection);
    
    insert(serializer, angularConstraintSelection);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_KINEMATIC_CONSTRAINT, buffer, serializer.offset);
}
CmdResult readKinematicConstraint(C::mip_interface& device, uint8_t* accelerationConstraintSelectionOut, uint8_t* velocityConstraintSelectionOut, uint8_t* angularConstraintSelectionOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength;
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_KINEMATIC_CONSTRAINT, buffer, serializer.offset, REPLY_KINEMATIC_CONSTRAINT, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, sizeof(buffer));
        
        assert(accelerationConstraintSelectionOut);
        extract(deserializer, *accelerationConstraintSelectionOut);
        
        assert(velocityConstraintSelectionOut);
        extract(deserializer, *velocityConstraintSelectionOut);
        
        assert(angularConstraintSelectionOut);
        extract(deserializer, *angularConstraintSelectionOut);
        
        if( !deserializer.isOk() )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
CmdResult saveKinematicConstraint(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_KINEMATIC_CONSTRAINT, buffer, serializer.offset);
}
CmdResult loadKinematicConstraint(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_KINEMATIC_CONSTRAINT, buffer, serializer.offset);
}
CmdResult defaultKinematicConstraint(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_KINEMATIC_CONSTRAINT, buffer, serializer.offset);
}
void insert(Serializer& serializer, const InitializationConfiguration& self)
{
    insert(serializer, self.wait_for_run_command);
    
    insert(serializer, self.initial_cond_src);
    
    insert(serializer, self.auto_heading_alignment_selector);
    
    insert(serializer, self.initial_heading);
    
    insert(serializer, self.initial_pitch);
    
    insert(serializer, self.initial_roll);
    
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.initial_position[i]);
    
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.initial_velocity[i]);
    
    insert(serializer, self.reference_frame_selector);
    
}
void extract(Serializer& serializer, InitializationConfiguration& self)
{
    extract(serializer, self.wait_for_run_command);
    
    extract(serializer, self.initial_cond_src);
    
    extract(serializer, self.auto_heading_alignment_selector);
    
    extract(serializer, self.initial_heading);
    
    extract(serializer, self.initial_pitch);
    
    extract(serializer, self.initial_roll);
    
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.initial_position[i]);
    
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.initial_velocity[i]);
    
    extract(serializer, self.reference_frame_selector);
    
}

CmdResult writeInitializationConfiguration(C::mip_interface& device, uint8_t waitForRunCommand, InitializationConfiguration::InitialConditionSource initialCondSrc, InitializationConfiguration::AlignmentSelector autoHeadingAlignmentSelector, float initialHeading, float initialPitch, float initialRoll, const float* initialPosition, const float* initialVelocity, FilterReferenceFrame referenceFrameSelector)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, waitForRunCommand);
    
    insert(serializer, initialCondSrc);
    
    insert(serializer, autoHeadingAlignmentSelector);
    
    insert(serializer, initialHeading);
    
    insert(serializer, initialPitch);
    
    insert(serializer, initialRoll);
    
    assert(initialPosition);
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, initialPosition[i]);
    
    assert(initialVelocity);
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, initialVelocity[i]);
    
    insert(serializer, referenceFrameSelector);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_INITIALIZATION_CONFIGURATION, buffer, serializer.offset);
}
CmdResult readInitializationConfiguration(C::mip_interface& device, uint8_t* waitForRunCommandOut, InitializationConfiguration::InitialConditionSource* initialCondSrcOut, InitializationConfiguration::AlignmentSelector* autoHeadingAlignmentSelectorOut, float* initialHeadingOut, float* initialPitchOut, float* initialRollOut, float* initialPositionOut, float* initialVelocityOut, FilterReferenceFrame* referenceFrameSelectorOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength;
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_INITIALIZATION_CONFIGURATION, buffer, serializer.offset, REPLY_INITIALIZATION_CONFIGURATION, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, sizeof(buffer));
        
        assert(waitForRunCommandOut);
        extract(deserializer, *waitForRunCommandOut);
        
        assert(initialCondSrcOut);
        extract(deserializer, *initialCondSrcOut);
        
        assert(autoHeadingAlignmentSelectorOut);
        extract(deserializer, *autoHeadingAlignmentSelectorOut);
        
        assert(initialHeadingOut);
        extract(deserializer, *initialHeadingOut);
        
        assert(initialPitchOut);
        extract(deserializer, *initialPitchOut);
        
        assert(initialRollOut);
        extract(deserializer, *initialRollOut);
        
        assert(initialPositionOut);
        for(unsigned int i=0; i < 3; i++)
            extract(deserializer, initialPositionOut[i]);
        
        assert(initialVelocityOut);
        for(unsigned int i=0; i < 3; i++)
            extract(deserializer, initialVelocityOut[i]);
        
        assert(referenceFrameSelectorOut);
        extract(deserializer, *referenceFrameSelectorOut);
        
        if( !deserializer.isOk() )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
CmdResult saveInitializationConfiguration(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_INITIALIZATION_CONFIGURATION, buffer, serializer.offset);
}
CmdResult loadInitializationConfiguration(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_INITIALIZATION_CONFIGURATION, buffer, serializer.offset);
}
CmdResult defaultInitializationConfiguration(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_INITIALIZATION_CONFIGURATION, buffer, serializer.offset);
}
void insert(Serializer& serializer, const AdaptiveFilterOptions& self)
{
    insert(serializer, self.level);
    
    insert(serializer, self.time_limit);
    
}
void extract(Serializer& serializer, AdaptiveFilterOptions& self)
{
    extract(serializer, self.level);
    
    extract(serializer, self.time_limit);
    
}

CmdResult writeAdaptiveFilterOptions(C::mip_interface& device, uint8_t level, uint16_t timeLimit)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, level);
    
    insert(serializer, timeLimit);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ADAPTIVE_FILTER_OPTIONS, buffer, serializer.offset);
}
CmdResult readAdaptiveFilterOptions(C::mip_interface& device, uint8_t* levelOut, uint16_t* timeLimitOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength;
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_ADAPTIVE_FILTER_OPTIONS, buffer, serializer.offset, REPLY_ADAPTIVE_FILTER_OPTIONS, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, sizeof(buffer));
        
        assert(levelOut);
        extract(deserializer, *levelOut);
        
        assert(timeLimitOut);
        extract(deserializer, *timeLimitOut);
        
        if( !deserializer.isOk() )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
CmdResult saveAdaptiveFilterOptions(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ADAPTIVE_FILTER_OPTIONS, buffer, serializer.offset);
}
CmdResult loadAdaptiveFilterOptions(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ADAPTIVE_FILTER_OPTIONS, buffer, serializer.offset);
}
CmdResult defaultAdaptiveFilterOptions(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ADAPTIVE_FILTER_OPTIONS, buffer, serializer.offset);
}
void insert(Serializer& serializer, const MultiAntennaOffset& self)
{
    insert(serializer, self.receiver_id);
    
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.antenna_offset[i]);
    
}
void extract(Serializer& serializer, MultiAntennaOffset& self)
{
    extract(serializer, self.receiver_id);
    
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.antenna_offset[i]);
    
}

CmdResult writeMultiAntennaOffset(C::mip_interface& device, uint8_t receiverId, const float* antennaOffset)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, receiverId);
    
    assert(antennaOffset);
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, antennaOffset[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_MULTI_ANTENNA_OFFSET, buffer, serializer.offset);
}
CmdResult readMultiAntennaOffset(C::mip_interface& device, uint8_t receiverId, float* antennaOffsetOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    insert(serializer, receiverId);
    
    assert(serializer.isOk());
    
    uint8_t responseLength;
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_MULTI_ANTENNA_OFFSET, buffer, serializer.offset, REPLY_MULTI_ANTENNA_OFFSET, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, sizeof(buffer));
        
        extract(deserializer, receiverId);
        
        assert(antennaOffsetOut);
        for(unsigned int i=0; i < 3; i++)
            extract(deserializer, antennaOffsetOut[i]);
        
        if( !deserializer.isOk() )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
CmdResult saveMultiAntennaOffset(C::mip_interface& device, uint8_t receiverId)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    insert(serializer, receiverId);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_MULTI_ANTENNA_OFFSET, buffer, serializer.offset);
}
CmdResult loadMultiAntennaOffset(C::mip_interface& device, uint8_t receiverId)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    insert(serializer, receiverId);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_MULTI_ANTENNA_OFFSET, buffer, serializer.offset);
}
CmdResult defaultMultiAntennaOffset(C::mip_interface& device, uint8_t receiverId)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    insert(serializer, receiverId);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_MULTI_ANTENNA_OFFSET, buffer, serializer.offset);
}
void insert(Serializer& serializer, const RelPosConfiguration& self)
{
    insert(serializer, self.source);
    
    insert(serializer, self.reference_frame_selector);
    
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.reference_coordinates[i]);
    
}
void extract(Serializer& serializer, RelPosConfiguration& self)
{
    extract(serializer, self.source);
    
    extract(serializer, self.reference_frame_selector);
    
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.reference_coordinates[i]);
    
}

CmdResult writeRelPosConfiguration(C::mip_interface& device, uint8_t source, FilterReferenceFrame referenceFrameSelector, const double* referenceCoordinates)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, source);
    
    insert(serializer, referenceFrameSelector);
    
    assert(referenceCoordinates);
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, referenceCoordinates[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_REL_POS_CONFIGURATION, buffer, serializer.offset);
}
CmdResult readRelPosConfiguration(C::mip_interface& device, uint8_t* sourceOut, FilterReferenceFrame* referenceFrameSelectorOut, double* referenceCoordinatesOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength;
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_REL_POS_CONFIGURATION, buffer, serializer.offset, REPLY_REL_POS_CONFIGURATION, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, sizeof(buffer));
        
        assert(sourceOut);
        extract(deserializer, *sourceOut);
        
        assert(referenceFrameSelectorOut);
        extract(deserializer, *referenceFrameSelectorOut);
        
        assert(referenceCoordinatesOut);
        for(unsigned int i=0; i < 3; i++)
            extract(deserializer, referenceCoordinatesOut[i]);
        
        if( !deserializer.isOk() )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
CmdResult saveRelPosConfiguration(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_REL_POS_CONFIGURATION, buffer, serializer.offset);
}
CmdResult loadRelPosConfiguration(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_REL_POS_CONFIGURATION, buffer, serializer.offset);
}
CmdResult defaultRelPosConfiguration(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_REL_POS_CONFIGURATION, buffer, serializer.offset);
}
void insert(Serializer& serializer, const RefPointLeverArm& self)
{
    insert(serializer, self.ref_point_sel);
    
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.lever_arm_offset[i]);
    
}
void extract(Serializer& serializer, RefPointLeverArm& self)
{
    extract(serializer, self.ref_point_sel);
    
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.lever_arm_offset[i]);
    
}

CmdResult writeRefPointLeverArm(C::mip_interface& device, RefPointLeverArm::ReferencePointSelector refPointSel, const float* leverArmOffset)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, refPointSel);
    
    assert(leverArmOffset);
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, leverArmOffset[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_REF_POINT_LEVER_ARM, buffer, serializer.offset);
}
CmdResult readRefPointLeverArm(C::mip_interface& device, RefPointLeverArm::ReferencePointSelector* refPointSelOut, float* leverArmOffsetOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength;
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_REF_POINT_LEVER_ARM, buffer, serializer.offset, REPLY_REF_POINT_LEVER_ARM, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, sizeof(buffer));
        
        assert(refPointSelOut);
        extract(deserializer, *refPointSelOut);
        
        assert(leverArmOffsetOut);
        for(unsigned int i=0; i < 3; i++)
            extract(deserializer, leverArmOffsetOut[i]);
        
        if( !deserializer.isOk() )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
CmdResult saveRefPointLeverArm(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_REF_POINT_LEVER_ARM, buffer, serializer.offset);
}
CmdResult loadRefPointLeverArm(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_REF_POINT_LEVER_ARM, buffer, serializer.offset);
}
CmdResult defaultRefPointLeverArm(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_REF_POINT_LEVER_ARM, buffer, serializer.offset);
}
void insert(Serializer& serializer, const SpeedMeasurement& self)
{
    insert(serializer, self.source);
    
    insert(serializer, self.time_of_week);
    
    insert(serializer, self.speed);
    
    insert(serializer, self.speed_uncertainty);
    
}
void extract(Serializer& serializer, SpeedMeasurement& self)
{
    extract(serializer, self.source);
    
    extract(serializer, self.time_of_week);
    
    extract(serializer, self.speed);
    
    extract(serializer, self.speed_uncertainty);
    
}

CmdResult speedMeasurement(C::mip_interface& device, uint8_t source, float timeOfWeek, float speed, float speedUncertainty)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, source);
    
    insert(serializer, timeOfWeek);
    
    insert(serializer, speed);
    
    insert(serializer, speedUncertainty);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SPEED_MEASUREMENT, buffer, serializer.offset);
}
void insert(Serializer& serializer, const SpeedLeverArm& self)
{
    insert(serializer, self.source);
    
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.lever_arm_offset[i]);
    
}
void extract(Serializer& serializer, SpeedLeverArm& self)
{
    extract(serializer, self.source);
    
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.lever_arm_offset[i]);
    
}

CmdResult writeSpeedLeverArm(C::mip_interface& device, uint8_t source, const float* leverArmOffset)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, source);
    
    assert(leverArmOffset);
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, leverArmOffset[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SPEED_LEVER_ARM, buffer, serializer.offset);
}
CmdResult readSpeedLeverArm(C::mip_interface& device, uint8_t source, float* leverArmOffsetOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    insert(serializer, source);
    
    assert(serializer.isOk());
    
    uint8_t responseLength;
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_SPEED_LEVER_ARM, buffer, serializer.offset, REPLY_SPEED_LEVER_ARM, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, sizeof(buffer));
        
        extract(deserializer, source);
        
        assert(leverArmOffsetOut);
        for(unsigned int i=0; i < 3; i++)
            extract(deserializer, leverArmOffsetOut[i]);
        
        if( !deserializer.isOk() )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
CmdResult saveSpeedLeverArm(C::mip_interface& device, uint8_t source)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    insert(serializer, source);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SPEED_LEVER_ARM, buffer, serializer.offset);
}
CmdResult loadSpeedLeverArm(C::mip_interface& device, uint8_t source)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    insert(serializer, source);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SPEED_LEVER_ARM, buffer, serializer.offset);
}
CmdResult defaultSpeedLeverArm(C::mip_interface& device, uint8_t source)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    insert(serializer, source);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SPEED_LEVER_ARM, buffer, serializer.offset);
}
void insert(Serializer& serializer, const WheeledVehicleConstraintControl& self)
{
    insert(serializer, self.enable);
    
}
void extract(Serializer& serializer, WheeledVehicleConstraintControl& self)
{
    extract(serializer, self.enable);
    
}

CmdResult writeWheeledVehicleConstraintControl(C::mip_interface& device, uint8_t enable)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, enable);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_VEHICLE_CONSTRAINT_CONTROL, buffer, serializer.offset);
}
CmdResult readWheeledVehicleConstraintControl(C::mip_interface& device, uint8_t* enableOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength;
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_VEHICLE_CONSTRAINT_CONTROL, buffer, serializer.offset, REPLY_VEHICLE_CONSTRAINT_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, sizeof(buffer));
        
        assert(enableOut);
        extract(deserializer, *enableOut);
        
        if( !deserializer.isOk() )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
CmdResult saveWheeledVehicleConstraintControl(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_VEHICLE_CONSTRAINT_CONTROL, buffer, serializer.offset);
}
CmdResult loadWheeledVehicleConstraintControl(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_VEHICLE_CONSTRAINT_CONTROL, buffer, serializer.offset);
}
CmdResult defaultWheeledVehicleConstraintControl(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_VEHICLE_CONSTRAINT_CONTROL, buffer, serializer.offset);
}
void insert(Serializer& serializer, const VerticalGyroConstraintControl& self)
{
    insert(serializer, self.enable);
    
}
void extract(Serializer& serializer, VerticalGyroConstraintControl& self)
{
    extract(serializer, self.enable);
    
}

CmdResult writeVerticalGyroConstraintControl(C::mip_interface& device, uint8_t enable)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, enable);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GYRO_CONSTRAINT_CONTROL, buffer, serializer.offset);
}
CmdResult readVerticalGyroConstraintControl(C::mip_interface& device, uint8_t* enableOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength;
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GYRO_CONSTRAINT_CONTROL, buffer, serializer.offset, REPLY_GYRO_CONSTRAINT_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, sizeof(buffer));
        
        assert(enableOut);
        extract(deserializer, *enableOut);
        
        if( !deserializer.isOk() )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
CmdResult saveVerticalGyroConstraintControl(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GYRO_CONSTRAINT_CONTROL, buffer, serializer.offset);
}
CmdResult loadVerticalGyroConstraintControl(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GYRO_CONSTRAINT_CONTROL, buffer, serializer.offset);
}
CmdResult defaultVerticalGyroConstraintControl(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GYRO_CONSTRAINT_CONTROL, buffer, serializer.offset);
}
void insert(Serializer& serializer, const GnssAntennaCalControl& self)
{
    insert(serializer, self.enable);
    
    insert(serializer, self.max_offset);
    
}
void extract(Serializer& serializer, GnssAntennaCalControl& self)
{
    extract(serializer, self.enable);
    
    extract(serializer, self.max_offset);
    
}

CmdResult writeGnssAntennaCalControl(C::mip_interface& device, uint8_t enable, float maxOffset)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, enable);
    
    insert(serializer, maxOffset);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ANTENNA_CALIBRATION_CONTROL, buffer, serializer.offset);
}
CmdResult readGnssAntennaCalControl(C::mip_interface& device, uint8_t* enableOut, float* maxOffsetOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength;
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_ANTENNA_CALIBRATION_CONTROL, buffer, serializer.offset, REPLY_ANTENNA_CALIBRATION_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, sizeof(buffer));
        
        assert(enableOut);
        extract(deserializer, *enableOut);
        
        assert(maxOffsetOut);
        extract(deserializer, *maxOffsetOut);
        
        if( !deserializer.isOk() )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
CmdResult saveGnssAntennaCalControl(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ANTENNA_CALIBRATION_CONTROL, buffer, serializer.offset);
}
CmdResult loadGnssAntennaCalControl(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ANTENNA_CALIBRATION_CONTROL, buffer, serializer.offset);
}
CmdResult defaultGnssAntennaCalControl(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ANTENNA_CALIBRATION_CONTROL, buffer, serializer.offset);
}
void insert(Serializer& serializer, const MagneticDeclinationSource& self)
{
    insert(serializer, self.source);
    
    insert(serializer, self.declination);
    
}
void extract(Serializer& serializer, MagneticDeclinationSource& self)
{
    extract(serializer, self.source);
    
    extract(serializer, self.declination);
    
}

CmdResult writeMagneticDeclinationSource(C::mip_interface& device, FilterMagDeclinationSource source, float declination)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, source);
    
    insert(serializer, declination);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_DECLINATION_SOURCE, buffer, serializer.offset);
}
CmdResult readMagneticDeclinationSource(C::mip_interface& device, FilterMagDeclinationSource* sourceOut, float* declinationOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength;
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_DECLINATION_SOURCE, buffer, serializer.offset, REPLY_DECLINATION_SOURCE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, sizeof(buffer));
        
        assert(sourceOut);
        extract(deserializer, *sourceOut);
        
        assert(declinationOut);
        extract(deserializer, *declinationOut);
        
        if( !deserializer.isOk() )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
CmdResult saveMagneticDeclinationSource(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_DECLINATION_SOURCE, buffer, serializer.offset);
}
CmdResult loadMagneticDeclinationSource(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_DECLINATION_SOURCE, buffer, serializer.offset);
}
CmdResult defaultMagneticDeclinationSource(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_DECLINATION_SOURCE, buffer, serializer.offset);
}
void insert(Serializer& serializer, const SetInitialHeading& self)
{
    insert(serializer, self.heading);
    
}
void extract(Serializer& serializer, SetInitialHeading& self)
{
    extract(serializer, self.heading);
    
}

CmdResult setInitialHeading(C::mip_interface& device, float heading)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, heading);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SET_INITIAL_HEADING, buffer, serializer.offset);
}

} // namespace commands_filter
} // namespace mip

