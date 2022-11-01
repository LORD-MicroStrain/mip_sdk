
#include "commands_filter.hpp"

#include "../utils/serialization.h"
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
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SET_INITIAL_ATTITUDE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const EstimationControl& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.enable);
        
    }
}
void extract(Serializer& serializer, EstimationControl& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        extract(serializer, self.enable);
        
    }
}

void insert(Serializer& serializer, const EstimationControl::Response& self)
{
    insert(serializer, self.enable);
    
}
void extract(Serializer& serializer, EstimationControl::Response& self)
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
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ESTIMATION_CONTROL_FLAGS, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult readEstimationControl(C::mip_interface& device, EstimationControl::EnableFlags* enableOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_ESTIMATION_CONTROL_FLAGS, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_ESTIMATION_CONTROL_FLAGS, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(enableOut);
        extract(deserializer, *enableOut);
        
        if( deserializer.remaining() != 0 )
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
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ESTIMATION_CONTROL_FLAGS, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult loadEstimationControl(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ESTIMATION_CONTROL_FLAGS, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult defaultEstimationControl(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ESTIMATION_CONTROL_FLAGS, buffer, (uint8_t)mip_serializer_length(&serializer));
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
    
    assert(velocity || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, velocity[i]);
    
    assert(posUncertainty || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, posUncertainty[i]);
    
    assert(velUncertainty || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, velUncertainty[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_EXTERNAL_GNSS_UPDATE, buffer, (uint8_t)mip_serializer_length(&serializer));
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
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_EXTERNAL_HEADING_UPDATE, buffer, (uint8_t)mip_serializer_length(&serializer));
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
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_EXTERNAL_HEADING_UPDATE_WITH_TIME, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const TareOrientation& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.axes);
        
    }
}
void extract(Serializer& serializer, TareOrientation& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        extract(serializer, self.axes);
        
    }
}

void insert(Serializer& serializer, const TareOrientation::Response& self)
{
    insert(serializer, self.axes);
    
}
void extract(Serializer& serializer, TareOrientation::Response& self)
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
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_TARE_ORIENTATION, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult readTareOrientation(C::mip_interface& device, TareOrientation::MipTareAxes* axesOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_TARE_ORIENTATION, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_TARE_ORIENTATION, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(axesOut);
        extract(deserializer, *axesOut);
        
        if( deserializer.remaining() != 0 )
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
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_TARE_ORIENTATION, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult loadTareOrientation(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_TARE_ORIENTATION, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult defaultTareOrientation(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_TARE_ORIENTATION, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const VehicleDynamicsMode& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.mode);
        
    }
}
void extract(Serializer& serializer, VehicleDynamicsMode& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        extract(serializer, self.mode);
        
    }
}

void insert(Serializer& serializer, const VehicleDynamicsMode::Response& self)
{
    insert(serializer, self.mode);
    
}
void extract(Serializer& serializer, VehicleDynamicsMode::Response& self)
{
    extract(serializer, self.mode);
    
}

CmdResult writeVehicleDynamicsMode(C::mip_interface& device, VehicleDynamicsMode::DynamicsMode mode)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, mode);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_VEHICLE_DYNAMICS_MODE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult readVehicleDynamicsMode(C::mip_interface& device, VehicleDynamicsMode::DynamicsMode* modeOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_VEHICLE_DYNAMICS_MODE, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_VEHICLE_DYNAMICS_MODE, buffer, &responseLength);
    
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
CmdResult saveVehicleDynamicsMode(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_VEHICLE_DYNAMICS_MODE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult loadVehicleDynamicsMode(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_VEHICLE_DYNAMICS_MODE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult defaultVehicleDynamicsMode(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_VEHICLE_DYNAMICS_MODE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const SensorToVehicleRotationEuler& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.roll);
        
        insert(serializer, self.pitch);
        
        insert(serializer, self.yaw);
        
    }
}
void extract(Serializer& serializer, SensorToVehicleRotationEuler& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        extract(serializer, self.roll);
        
        extract(serializer, self.pitch);
        
        extract(serializer, self.yaw);
        
    }
}

void insert(Serializer& serializer, const SensorToVehicleRotationEuler::Response& self)
{
    insert(serializer, self.roll);
    
    insert(serializer, self.pitch);
    
    insert(serializer, self.yaw);
    
}
void extract(Serializer& serializer, SensorToVehicleRotationEuler::Response& self)
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
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_ROTATION_EULER, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult readSensorToVehicleRotationEuler(C::mip_interface& device, float* rollOut, float* pitchOut, float* yawOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_ROTATION_EULER, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_SENSOR2VEHICLE_ROTATION_EULER, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(rollOut);
        extract(deserializer, *rollOut);
        
        assert(pitchOut);
        extract(deserializer, *pitchOut);
        
        assert(yawOut);
        extract(deserializer, *yawOut);
        
        if( deserializer.remaining() != 0 )
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
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_ROTATION_EULER, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult loadSensorToVehicleRotationEuler(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_ROTATION_EULER, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult defaultSensorToVehicleRotationEuler(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_ROTATION_EULER, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const SensorToVehicleRotationDcm& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        for(unsigned int i=0; i < 9; i++)
            insert(serializer, self.dcm[i]);
        
    }
}
void extract(Serializer& serializer, SensorToVehicleRotationDcm& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        for(unsigned int i=0; i < 9; i++)
            extract(serializer, self.dcm[i]);
        
    }
}

void insert(Serializer& serializer, const SensorToVehicleRotationDcm::Response& self)
{
    for(unsigned int i=0; i < 9; i++)
        insert(serializer, self.dcm[i]);
    
}
void extract(Serializer& serializer, SensorToVehicleRotationDcm::Response& self)
{
    for(unsigned int i=0; i < 9; i++)
        extract(serializer, self.dcm[i]);
    
}

CmdResult writeSensorToVehicleRotationDcm(C::mip_interface& device, const float* dcm)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    assert(dcm || (9 == 0));
    for(unsigned int i=0; i < 9; i++)
        insert(serializer, dcm[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_ROTATION_DCM, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult readSensorToVehicleRotationDcm(C::mip_interface& device, float* dcmOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_ROTATION_DCM, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_SENSOR2VEHICLE_ROTATION_DCM, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(dcmOut || (9 == 0));
        for(unsigned int i=0; i < 9; i++)
            extract(deserializer, dcmOut[i]);
        
        if( deserializer.remaining() != 0 )
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
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_ROTATION_DCM, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult loadSensorToVehicleRotationDcm(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_ROTATION_DCM, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult defaultSensorToVehicleRotationDcm(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_ROTATION_DCM, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const SensorToVehicleRotationQuaternion& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        for(unsigned int i=0; i < 4; i++)
            insert(serializer, self.quat[i]);
        
    }
}
void extract(Serializer& serializer, SensorToVehicleRotationQuaternion& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        for(unsigned int i=0; i < 4; i++)
            extract(serializer, self.quat[i]);
        
    }
}

void insert(Serializer& serializer, const SensorToVehicleRotationQuaternion::Response& self)
{
    for(unsigned int i=0; i < 4; i++)
        insert(serializer, self.quat[i]);
    
}
void extract(Serializer& serializer, SensorToVehicleRotationQuaternion::Response& self)
{
    for(unsigned int i=0; i < 4; i++)
        extract(serializer, self.quat[i]);
    
}

CmdResult writeSensorToVehicleRotationQuaternion(C::mip_interface& device, const float* quat)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    assert(quat || (4 == 0));
    for(unsigned int i=0; i < 4; i++)
        insert(serializer, quat[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_ROTATION_QUATERNION, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult readSensorToVehicleRotationQuaternion(C::mip_interface& device, float* quatOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_ROTATION_QUATERNION, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_SENSOR2VEHICLE_ROTATION_QUATERNION, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(quatOut || (4 == 0));
        for(unsigned int i=0; i < 4; i++)
            extract(deserializer, quatOut[i]);
        
        if( deserializer.remaining() != 0 )
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
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_ROTATION_QUATERNION, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult loadSensorToVehicleRotationQuaternion(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_ROTATION_QUATERNION, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult defaultSensorToVehicleRotationQuaternion(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_ROTATION_QUATERNION, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const SensorToVehicleOffset& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        for(unsigned int i=0; i < 3; i++)
            insert(serializer, self.offset[i]);
        
    }
}
void extract(Serializer& serializer, SensorToVehicleOffset& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        for(unsigned int i=0; i < 3; i++)
            extract(serializer, self.offset[i]);
        
    }
}

void insert(Serializer& serializer, const SensorToVehicleOffset::Response& self)
{
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.offset[i]);
    
}
void extract(Serializer& serializer, SensorToVehicleOffset::Response& self)
{
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.offset[i]);
    
}

CmdResult writeSensorToVehicleOffset(C::mip_interface& device, const float* offset)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    assert(offset || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, offset[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_OFFSET, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult readSensorToVehicleOffset(C::mip_interface& device, float* offsetOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_OFFSET, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_SENSOR2VEHICLE_OFFSET, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(offsetOut || (3 == 0));
        for(unsigned int i=0; i < 3; i++)
            extract(deserializer, offsetOut[i]);
        
        if( deserializer.remaining() != 0 )
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
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_OFFSET, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult loadSensorToVehicleOffset(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_OFFSET, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult defaultSensorToVehicleOffset(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_OFFSET, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const AntennaOffset& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        for(unsigned int i=0; i < 3; i++)
            insert(serializer, self.offset[i]);
        
    }
}
void extract(Serializer& serializer, AntennaOffset& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        for(unsigned int i=0; i < 3; i++)
            extract(serializer, self.offset[i]);
        
    }
}

void insert(Serializer& serializer, const AntennaOffset::Response& self)
{
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.offset[i]);
    
}
void extract(Serializer& serializer, AntennaOffset::Response& self)
{
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.offset[i]);
    
}

CmdResult writeAntennaOffset(C::mip_interface& device, const float* offset)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    assert(offset || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, offset[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ANTENNA_OFFSET, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult readAntennaOffset(C::mip_interface& device, float* offsetOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_ANTENNA_OFFSET, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_ANTENNA_OFFSET, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(offsetOut || (3 == 0));
        for(unsigned int i=0; i < 3; i++)
            extract(deserializer, offsetOut[i]);
        
        if( deserializer.remaining() != 0 )
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
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ANTENNA_OFFSET, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult loadAntennaOffset(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ANTENNA_OFFSET, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult defaultAntennaOffset(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ANTENNA_OFFSET, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const GnssSource& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.source);
        
    }
}
void extract(Serializer& serializer, GnssSource& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        extract(serializer, self.source);
        
    }
}

void insert(Serializer& serializer, const GnssSource::Response& self)
{
    insert(serializer, self.source);
    
}
void extract(Serializer& serializer, GnssSource::Response& self)
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
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GNSS_SOURCE_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult readGnssSource(C::mip_interface& device, GnssSource::Source* sourceOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GNSS_SOURCE_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_GNSS_SOURCE_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(sourceOut);
        extract(deserializer, *sourceOut);
        
        if( deserializer.remaining() != 0 )
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
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GNSS_SOURCE_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult loadGnssSource(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GNSS_SOURCE_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult defaultGnssSource(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GNSS_SOURCE_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const HeadingSource& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.source);
        
    }
}
void extract(Serializer& serializer, HeadingSource& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        extract(serializer, self.source);
        
    }
}

void insert(Serializer& serializer, const HeadingSource::Response& self)
{
    insert(serializer, self.source);
    
}
void extract(Serializer& serializer, HeadingSource::Response& self)
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
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_HEADING_UPDATE_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult readHeadingSource(C::mip_interface& device, HeadingSource::Source* sourceOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_HEADING_UPDATE_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_HEADING_UPDATE_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(sourceOut);
        extract(deserializer, *sourceOut);
        
        if( deserializer.remaining() != 0 )
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
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_HEADING_UPDATE_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult loadHeadingSource(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_HEADING_UPDATE_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult defaultHeadingSource(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_HEADING_UPDATE_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const AutoInitControl& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.enable);
        
    }
}
void extract(Serializer& serializer, AutoInitControl& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        extract(serializer, self.enable);
        
    }
}

void insert(Serializer& serializer, const AutoInitControl::Response& self)
{
    insert(serializer, self.enable);
    
}
void extract(Serializer& serializer, AutoInitControl::Response& self)
{
    extract(serializer, self.enable);
    
}

CmdResult writeAutoInitControl(C::mip_interface& device, uint8_t enable)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, enable);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_AUTOINIT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult readAutoInitControl(C::mip_interface& device, uint8_t* enableOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_AUTOINIT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_AUTOINIT_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(enableOut);
        extract(deserializer, *enableOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
CmdResult saveAutoInitControl(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_AUTOINIT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult loadAutoInitControl(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_AUTOINIT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult defaultAutoInitControl(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_AUTOINIT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const AccelNoise& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.x);
        
        insert(serializer, self.y);
        
        insert(serializer, self.z);
        
    }
}
void extract(Serializer& serializer, AccelNoise& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        extract(serializer, self.x);
        
        extract(serializer, self.y);
        
        extract(serializer, self.z);
        
    }
}

void insert(Serializer& serializer, const AccelNoise::Response& self)
{
    insert(serializer, self.x);
    
    insert(serializer, self.y);
    
    insert(serializer, self.z);
    
}
void extract(Serializer& serializer, AccelNoise::Response& self)
{
    extract(serializer, self.x);
    
    extract(serializer, self.y);
    
    extract(serializer, self.z);
    
}

CmdResult writeAccelNoise(C::mip_interface& device, float x, float y, float z)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, x);
    
    insert(serializer, y);
    
    insert(serializer, z);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ACCEL_NOISE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult readAccelNoise(C::mip_interface& device, float* xOut, float* yOut, float* zOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_ACCEL_NOISE, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_ACCEL_NOISE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(xOut);
        extract(deserializer, *xOut);
        
        assert(yOut);
        extract(deserializer, *yOut);
        
        assert(zOut);
        extract(deserializer, *zOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
CmdResult saveAccelNoise(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ACCEL_NOISE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult loadAccelNoise(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ACCEL_NOISE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult defaultAccelNoise(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ACCEL_NOISE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const GyroNoise& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.x);
        
        insert(serializer, self.y);
        
        insert(serializer, self.z);
        
    }
}
void extract(Serializer& serializer, GyroNoise& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        extract(serializer, self.x);
        
        extract(serializer, self.y);
        
        extract(serializer, self.z);
        
    }
}

void insert(Serializer& serializer, const GyroNoise::Response& self)
{
    insert(serializer, self.x);
    
    insert(serializer, self.y);
    
    insert(serializer, self.z);
    
}
void extract(Serializer& serializer, GyroNoise::Response& self)
{
    extract(serializer, self.x);
    
    extract(serializer, self.y);
    
    extract(serializer, self.z);
    
}

CmdResult writeGyroNoise(C::mip_interface& device, float x, float y, float z)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, x);
    
    insert(serializer, y);
    
    insert(serializer, z);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GYRO_NOISE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult readGyroNoise(C::mip_interface& device, float* xOut, float* yOut, float* zOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GYRO_NOISE, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_GYRO_NOISE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(xOut);
        extract(deserializer, *xOut);
        
        assert(yOut);
        extract(deserializer, *yOut);
        
        assert(zOut);
        extract(deserializer, *zOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
CmdResult saveGyroNoise(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GYRO_NOISE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult loadGyroNoise(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GYRO_NOISE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult defaultGyroNoise(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GYRO_NOISE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const AccelBiasModel& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.x_beta);
        
        insert(serializer, self.y_beta);
        
        insert(serializer, self.z_beta);
        
        insert(serializer, self.x);
        
        insert(serializer, self.y);
        
        insert(serializer, self.z);
        
    }
}
void extract(Serializer& serializer, AccelBiasModel& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        extract(serializer, self.x_beta);
        
        extract(serializer, self.y_beta);
        
        extract(serializer, self.z_beta);
        
        extract(serializer, self.x);
        
        extract(serializer, self.y);
        
        extract(serializer, self.z);
        
    }
}

void insert(Serializer& serializer, const AccelBiasModel::Response& self)
{
    insert(serializer, self.x_beta);
    
    insert(serializer, self.y_beta);
    
    insert(serializer, self.z_beta);
    
    insert(serializer, self.x);
    
    insert(serializer, self.y);
    
    insert(serializer, self.z);
    
}
void extract(Serializer& serializer, AccelBiasModel::Response& self)
{
    extract(serializer, self.x_beta);
    
    extract(serializer, self.y_beta);
    
    extract(serializer, self.z_beta);
    
    extract(serializer, self.x);
    
    extract(serializer, self.y);
    
    extract(serializer, self.z);
    
}

CmdResult writeAccelBiasModel(C::mip_interface& device, float xBeta, float yBeta, float zBeta, float x, float y, float z)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, xBeta);
    
    insert(serializer, yBeta);
    
    insert(serializer, zBeta);
    
    insert(serializer, x);
    
    insert(serializer, y);
    
    insert(serializer, z);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ACCEL_BIAS_MODEL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult readAccelBiasModel(C::mip_interface& device, float* xBetaOut, float* yBetaOut, float* zBetaOut, float* xOut, float* yOut, float* zOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_ACCEL_BIAS_MODEL, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_ACCEL_BIAS_MODEL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(xBetaOut);
        extract(deserializer, *xBetaOut);
        
        assert(yBetaOut);
        extract(deserializer, *yBetaOut);
        
        assert(zBetaOut);
        extract(deserializer, *zBetaOut);
        
        assert(xOut);
        extract(deserializer, *xOut);
        
        assert(yOut);
        extract(deserializer, *yOut);
        
        assert(zOut);
        extract(deserializer, *zOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
CmdResult saveAccelBiasModel(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ACCEL_BIAS_MODEL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult loadAccelBiasModel(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ACCEL_BIAS_MODEL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult defaultAccelBiasModel(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ACCEL_BIAS_MODEL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const GyroBiasModel& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.x_beta);
        
        insert(serializer, self.y_beta);
        
        insert(serializer, self.z_beta);
        
        insert(serializer, self.x);
        
        insert(serializer, self.y);
        
        insert(serializer, self.z);
        
    }
}
void extract(Serializer& serializer, GyroBiasModel& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        extract(serializer, self.x_beta);
        
        extract(serializer, self.y_beta);
        
        extract(serializer, self.z_beta);
        
        extract(serializer, self.x);
        
        extract(serializer, self.y);
        
        extract(serializer, self.z);
        
    }
}

void insert(Serializer& serializer, const GyroBiasModel::Response& self)
{
    insert(serializer, self.x_beta);
    
    insert(serializer, self.y_beta);
    
    insert(serializer, self.z_beta);
    
    insert(serializer, self.x);
    
    insert(serializer, self.y);
    
    insert(serializer, self.z);
    
}
void extract(Serializer& serializer, GyroBiasModel::Response& self)
{
    extract(serializer, self.x_beta);
    
    extract(serializer, self.y_beta);
    
    extract(serializer, self.z_beta);
    
    extract(serializer, self.x);
    
    extract(serializer, self.y);
    
    extract(serializer, self.z);
    
}

CmdResult writeGyroBiasModel(C::mip_interface& device, float xBeta, float yBeta, float zBeta, float x, float y, float z)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, xBeta);
    
    insert(serializer, yBeta);
    
    insert(serializer, zBeta);
    
    insert(serializer, x);
    
    insert(serializer, y);
    
    insert(serializer, z);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GYRO_BIAS_MODEL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult readGyroBiasModel(C::mip_interface& device, float* xBetaOut, float* yBetaOut, float* zBetaOut, float* xOut, float* yOut, float* zOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GYRO_BIAS_MODEL, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_GYRO_BIAS_MODEL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(xBetaOut);
        extract(deserializer, *xBetaOut);
        
        assert(yBetaOut);
        extract(deserializer, *yBetaOut);
        
        assert(zBetaOut);
        extract(deserializer, *zBetaOut);
        
        assert(xOut);
        extract(deserializer, *xOut);
        
        assert(yOut);
        extract(deserializer, *yOut);
        
        assert(zOut);
        extract(deserializer, *zOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
CmdResult saveGyroBiasModel(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GYRO_BIAS_MODEL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult loadGyroBiasModel(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GYRO_BIAS_MODEL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult defaultGyroBiasModel(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GYRO_BIAS_MODEL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const AltitudeAiding& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.aiding_selector);
        
    }
}
void extract(Serializer& serializer, AltitudeAiding& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        extract(serializer, self.aiding_selector);
        
    }
}

void insert(Serializer& serializer, const AltitudeAiding::Response& self)
{
    insert(serializer, self.aiding_selector);
    
}
void extract(Serializer& serializer, AltitudeAiding::Response& self)
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
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ALTITUDE_AIDING_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult readAltitudeAiding(C::mip_interface& device, uint8_t* aidingSelectorOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_ALTITUDE_AIDING_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_ALTITUDE_AIDING_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(aidingSelectorOut);
        extract(deserializer, *aidingSelectorOut);
        
        if( deserializer.remaining() != 0 )
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
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ALTITUDE_AIDING_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult loadAltitudeAiding(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ALTITUDE_AIDING_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult defaultAltitudeAiding(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ALTITUDE_AIDING_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const AutoZupt& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.enable);
        
        insert(serializer, self.threshold);
        
    }
}
void extract(Serializer& serializer, AutoZupt& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        extract(serializer, self.enable);
        
        extract(serializer, self.threshold);
        
    }
}

void insert(Serializer& serializer, const AutoZupt::Response& self)
{
    insert(serializer, self.enable);
    
    insert(serializer, self.threshold);
    
}
void extract(Serializer& serializer, AutoZupt::Response& self)
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
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ZUPT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult readAutoZupt(C::mip_interface& device, uint8_t* enableOut, float* thresholdOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_ZUPT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_ZUPT_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(enableOut);
        extract(deserializer, *enableOut);
        
        assert(thresholdOut);
        extract(deserializer, *thresholdOut);
        
        if( deserializer.remaining() != 0 )
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
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ZUPT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult loadAutoZupt(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ZUPT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult defaultAutoZupt(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ZUPT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const AutoAngularZupt& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.enable);
        
        insert(serializer, self.threshold);
        
    }
}
void extract(Serializer& serializer, AutoAngularZupt& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        extract(serializer, self.enable);
        
        extract(serializer, self.threshold);
        
    }
}

void insert(Serializer& serializer, const AutoAngularZupt::Response& self)
{
    insert(serializer, self.enable);
    
    insert(serializer, self.threshold);
    
}
void extract(Serializer& serializer, AutoAngularZupt::Response& self)
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
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ANGULAR_ZUPT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult readAutoAngularZupt(C::mip_interface& device, uint8_t* enableOut, float* thresholdOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_ANGULAR_ZUPT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_ANGULAR_ZUPT_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(enableOut);
        extract(deserializer, *enableOut);
        
        assert(thresholdOut);
        extract(deserializer, *thresholdOut);
        
        if( deserializer.remaining() != 0 )
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
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ANGULAR_ZUPT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult loadAutoAngularZupt(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ANGULAR_ZUPT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult defaultAutoAngularZupt(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ANGULAR_ZUPT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
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
void insert(Serializer& serializer, const ReferencePosition& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.enable);
        
        insert(serializer, self.latitude);
        
        insert(serializer, self.longitude);
        
        insert(serializer, self.altitude);
        
    }
}
void extract(Serializer& serializer, ReferencePosition& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        extract(serializer, self.enable);
        
        extract(serializer, self.latitude);
        
        extract(serializer, self.longitude);
        
        extract(serializer, self.altitude);
        
    }
}

void insert(Serializer& serializer, const ReferencePosition::Response& self)
{
    insert(serializer, self.enable);
    
    insert(serializer, self.latitude);
    
    insert(serializer, self.longitude);
    
    insert(serializer, self.altitude);
    
}
void extract(Serializer& serializer, ReferencePosition::Response& self)
{
    extract(serializer, self.enable);
    
    extract(serializer, self.latitude);
    
    extract(serializer, self.longitude);
    
    extract(serializer, self.altitude);
    
}

CmdResult writeReferencePosition(C::mip_interface& device, bool enable, double latitude, double longitude, double altitude)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, enable);
    
    insert(serializer, latitude);
    
    insert(serializer, longitude);
    
    insert(serializer, altitude);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_REFERENCE_POSITION, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult readReferencePosition(C::mip_interface& device, bool* enableOut, double* latitudeOut, double* longitudeOut, double* altitudeOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_REFERENCE_POSITION, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_REFERENCE_POSITION, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(enableOut);
        extract(deserializer, *enableOut);
        
        assert(latitudeOut);
        extract(deserializer, *latitudeOut);
        
        assert(longitudeOut);
        extract(deserializer, *longitudeOut);
        
        assert(altitudeOut);
        extract(deserializer, *altitudeOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
CmdResult saveReferencePosition(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_REFERENCE_POSITION, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult loadReferencePosition(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_REFERENCE_POSITION, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult defaultReferencePosition(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_REFERENCE_POSITION, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const AccelMagnitudeErrorAdaptiveMeasurement& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.adaptive_measurement);
        
        insert(serializer, self.frequency);
        
        insert(serializer, self.low_limit);
        
        insert(serializer, self.high_limit);
        
        insert(serializer, self.low_limit_uncertainty);
        
        insert(serializer, self.high_limit_uncertainty);
        
        insert(serializer, self.minimum_uncertainty);
        
    }
}
void extract(Serializer& serializer, AccelMagnitudeErrorAdaptiveMeasurement& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        extract(serializer, self.adaptive_measurement);
        
        extract(serializer, self.frequency);
        
        extract(serializer, self.low_limit);
        
        extract(serializer, self.high_limit);
        
        extract(serializer, self.low_limit_uncertainty);
        
        extract(serializer, self.high_limit_uncertainty);
        
        extract(serializer, self.minimum_uncertainty);
        
    }
}

void insert(Serializer& serializer, const AccelMagnitudeErrorAdaptiveMeasurement::Response& self)
{
    insert(serializer, self.adaptive_measurement);
    
    insert(serializer, self.frequency);
    
    insert(serializer, self.low_limit);
    
    insert(serializer, self.high_limit);
    
    insert(serializer, self.low_limit_uncertainty);
    
    insert(serializer, self.high_limit_uncertainty);
    
    insert(serializer, self.minimum_uncertainty);
    
}
void extract(Serializer& serializer, AccelMagnitudeErrorAdaptiveMeasurement::Response& self)
{
    extract(serializer, self.adaptive_measurement);
    
    extract(serializer, self.frequency);
    
    extract(serializer, self.low_limit);
    
    extract(serializer, self.high_limit);
    
    extract(serializer, self.low_limit_uncertainty);
    
    extract(serializer, self.high_limit_uncertainty);
    
    extract(serializer, self.minimum_uncertainty);
    
}

CmdResult writeAccelMagnitudeErrorAdaptiveMeasurement(C::mip_interface& device, FilterAdaptiveMeasurement adaptiveMeasurement, float frequency, float lowLimit, float highLimit, float lowLimitUncertainty, float highLimitUncertainty, float minimumUncertainty)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, adaptiveMeasurement);
    
    insert(serializer, frequency);
    
    insert(serializer, lowLimit);
    
    insert(serializer, highLimit);
    
    insert(serializer, lowLimitUncertainty);
    
    insert(serializer, highLimitUncertainty);
    
    insert(serializer, minimumUncertainty);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ACCEL_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult readAccelMagnitudeErrorAdaptiveMeasurement(C::mip_interface& device, FilterAdaptiveMeasurement* adaptiveMeasurementOut, float* frequencyOut, float* lowLimitOut, float* highLimitOut, float* lowLimitUncertaintyOut, float* highLimitUncertaintyOut, float* minimumUncertaintyOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_ACCEL_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_ACCEL_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(adaptiveMeasurementOut);
        extract(deserializer, *adaptiveMeasurementOut);
        
        assert(frequencyOut);
        extract(deserializer, *frequencyOut);
        
        assert(lowLimitOut);
        extract(deserializer, *lowLimitOut);
        
        assert(highLimitOut);
        extract(deserializer, *highLimitOut);
        
        assert(lowLimitUncertaintyOut);
        extract(deserializer, *lowLimitUncertaintyOut);
        
        assert(highLimitUncertaintyOut);
        extract(deserializer, *highLimitUncertaintyOut);
        
        assert(minimumUncertaintyOut);
        extract(deserializer, *minimumUncertaintyOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
CmdResult saveAccelMagnitudeErrorAdaptiveMeasurement(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ACCEL_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult loadAccelMagnitudeErrorAdaptiveMeasurement(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ACCEL_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult defaultAccelMagnitudeErrorAdaptiveMeasurement(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ACCEL_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const MagMagnitudeErrorAdaptiveMeasurement& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.adaptive_measurement);
        
        insert(serializer, self.frequency);
        
        insert(serializer, self.low_limit);
        
        insert(serializer, self.high_limit);
        
        insert(serializer, self.low_limit_uncertainty);
        
        insert(serializer, self.high_limit_uncertainty);
        
        insert(serializer, self.minimum_uncertainty);
        
    }
}
void extract(Serializer& serializer, MagMagnitudeErrorAdaptiveMeasurement& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        extract(serializer, self.adaptive_measurement);
        
        extract(serializer, self.frequency);
        
        extract(serializer, self.low_limit);
        
        extract(serializer, self.high_limit);
        
        extract(serializer, self.low_limit_uncertainty);
        
        extract(serializer, self.high_limit_uncertainty);
        
        extract(serializer, self.minimum_uncertainty);
        
    }
}

void insert(Serializer& serializer, const MagMagnitudeErrorAdaptiveMeasurement::Response& self)
{
    insert(serializer, self.adaptive_measurement);
    
    insert(serializer, self.frequency);
    
    insert(serializer, self.low_limit);
    
    insert(serializer, self.high_limit);
    
    insert(serializer, self.low_limit_uncertainty);
    
    insert(serializer, self.high_limit_uncertainty);
    
    insert(serializer, self.minimum_uncertainty);
    
}
void extract(Serializer& serializer, MagMagnitudeErrorAdaptiveMeasurement::Response& self)
{
    extract(serializer, self.adaptive_measurement);
    
    extract(serializer, self.frequency);
    
    extract(serializer, self.low_limit);
    
    extract(serializer, self.high_limit);
    
    extract(serializer, self.low_limit_uncertainty);
    
    extract(serializer, self.high_limit_uncertainty);
    
    extract(serializer, self.minimum_uncertainty);
    
}

CmdResult writeMagMagnitudeErrorAdaptiveMeasurement(C::mip_interface& device, FilterAdaptiveMeasurement adaptiveMeasurement, float frequency, float lowLimit, float highLimit, float lowLimitUncertainty, float highLimitUncertainty, float minimumUncertainty)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, adaptiveMeasurement);
    
    insert(serializer, frequency);
    
    insert(serializer, lowLimit);
    
    insert(serializer, highLimit);
    
    insert(serializer, lowLimitUncertainty);
    
    insert(serializer, highLimitUncertainty);
    
    insert(serializer, minimumUncertainty);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_MAG_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult readMagMagnitudeErrorAdaptiveMeasurement(C::mip_interface& device, FilterAdaptiveMeasurement* adaptiveMeasurementOut, float* frequencyOut, float* lowLimitOut, float* highLimitOut, float* lowLimitUncertaintyOut, float* highLimitUncertaintyOut, float* minimumUncertaintyOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_MAG_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_MAG_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(adaptiveMeasurementOut);
        extract(deserializer, *adaptiveMeasurementOut);
        
        assert(frequencyOut);
        extract(deserializer, *frequencyOut);
        
        assert(lowLimitOut);
        extract(deserializer, *lowLimitOut);
        
        assert(highLimitOut);
        extract(deserializer, *highLimitOut);
        
        assert(lowLimitUncertaintyOut);
        extract(deserializer, *lowLimitUncertaintyOut);
        
        assert(highLimitUncertaintyOut);
        extract(deserializer, *highLimitUncertaintyOut);
        
        assert(minimumUncertaintyOut);
        extract(deserializer, *minimumUncertaintyOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
CmdResult saveMagMagnitudeErrorAdaptiveMeasurement(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_MAG_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult loadMagMagnitudeErrorAdaptiveMeasurement(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_MAG_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult defaultMagMagnitudeErrorAdaptiveMeasurement(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_MAG_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const MagDipAngleErrorAdaptiveMeasurement& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.enable);
        
        insert(serializer, self.frequency);
        
        insert(serializer, self.high_limit);
        
        insert(serializer, self.high_limit_uncertainty);
        
        insert(serializer, self.minimum_uncertainty);
        
    }
}
void extract(Serializer& serializer, MagDipAngleErrorAdaptiveMeasurement& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        extract(serializer, self.enable);
        
        extract(serializer, self.frequency);
        
        extract(serializer, self.high_limit);
        
        extract(serializer, self.high_limit_uncertainty);
        
        extract(serializer, self.minimum_uncertainty);
        
    }
}

void insert(Serializer& serializer, const MagDipAngleErrorAdaptiveMeasurement::Response& self)
{
    insert(serializer, self.enable);
    
    insert(serializer, self.frequency);
    
    insert(serializer, self.high_limit);
    
    insert(serializer, self.high_limit_uncertainty);
    
    insert(serializer, self.minimum_uncertainty);
    
}
void extract(Serializer& serializer, MagDipAngleErrorAdaptiveMeasurement::Response& self)
{
    extract(serializer, self.enable);
    
    extract(serializer, self.frequency);
    
    extract(serializer, self.high_limit);
    
    extract(serializer, self.high_limit_uncertainty);
    
    extract(serializer, self.minimum_uncertainty);
    
}

CmdResult writeMagDipAngleErrorAdaptiveMeasurement(C::mip_interface& device, bool enable, float frequency, float highLimit, float highLimitUncertainty, float minimumUncertainty)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, enable);
    
    insert(serializer, frequency);
    
    insert(serializer, highLimit);
    
    insert(serializer, highLimitUncertainty);
    
    insert(serializer, minimumUncertainty);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_MAG_DIP_ANGLE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult readMagDipAngleErrorAdaptiveMeasurement(C::mip_interface& device, bool* enableOut, float* frequencyOut, float* highLimitOut, float* highLimitUncertaintyOut, float* minimumUncertaintyOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_MAG_DIP_ANGLE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_MAG_DIP_ANGLE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(enableOut);
        extract(deserializer, *enableOut);
        
        assert(frequencyOut);
        extract(deserializer, *frequencyOut);
        
        assert(highLimitOut);
        extract(deserializer, *highLimitOut);
        
        assert(highLimitUncertaintyOut);
        extract(deserializer, *highLimitUncertaintyOut);
        
        assert(minimumUncertaintyOut);
        extract(deserializer, *minimumUncertaintyOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
CmdResult saveMagDipAngleErrorAdaptiveMeasurement(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_MAG_DIP_ANGLE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult loadMagDipAngleErrorAdaptiveMeasurement(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_MAG_DIP_ANGLE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult defaultMagDipAngleErrorAdaptiveMeasurement(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_MAG_DIP_ANGLE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const AidingMeasurementEnable& self)
{
    insert(serializer, self.function);
    
    insert(serializer, self.aiding_source);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.enable);
        
    }
}
void extract(Serializer& serializer, AidingMeasurementEnable& self)
{
    extract(serializer, self.function);
    
    extract(serializer, self.aiding_source);
    
    if( self.function == FunctionSelector::WRITE )
    {
        extract(serializer, self.enable);
        
    }
}

void insert(Serializer& serializer, const AidingMeasurementEnable::Response& self)
{
    insert(serializer, self.aiding_source);
    
    insert(serializer, self.enable);
    
}
void extract(Serializer& serializer, AidingMeasurementEnable::Response& self)
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
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_AIDING_MEASUREMENT_ENABLE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult readAidingMeasurementEnable(C::mip_interface& device, AidingMeasurementEnable::AidingSource aidingSource, bool* enableOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    insert(serializer, aidingSource);
    
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_AIDING_MEASUREMENT_ENABLE, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_AIDING_MEASUREMENT_ENABLE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        extract(deserializer, aidingSource);
        
        assert(enableOut);
        extract(deserializer, *enableOut);
        
        if( deserializer.remaining() != 0 )
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
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_AIDING_MEASUREMENT_ENABLE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult loadAidingMeasurementEnable(C::mip_interface& device, AidingMeasurementEnable::AidingSource aidingSource)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    insert(serializer, aidingSource);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_AIDING_MEASUREMENT_ENABLE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult defaultAidingMeasurementEnable(C::mip_interface& device, AidingMeasurementEnable::AidingSource aidingSource)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    insert(serializer, aidingSource);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_AIDING_MEASUREMENT_ENABLE, buffer, (uint8_t)mip_serializer_length(&serializer));
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
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.acceleration_constraint_selection);
        
        insert(serializer, self.velocity_constraint_selection);
        
        insert(serializer, self.angular_constraint_selection);
        
    }
}
void extract(Serializer& serializer, KinematicConstraint& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        extract(serializer, self.acceleration_constraint_selection);
        
        extract(serializer, self.velocity_constraint_selection);
        
        extract(serializer, self.angular_constraint_selection);
        
    }
}

void insert(Serializer& serializer, const KinematicConstraint::Response& self)
{
    insert(serializer, self.acceleration_constraint_selection);
    
    insert(serializer, self.velocity_constraint_selection);
    
    insert(serializer, self.angular_constraint_selection);
    
}
void extract(Serializer& serializer, KinematicConstraint::Response& self)
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
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_KINEMATIC_CONSTRAINT, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult readKinematicConstraint(C::mip_interface& device, uint8_t* accelerationConstraintSelectionOut, uint8_t* velocityConstraintSelectionOut, uint8_t* angularConstraintSelectionOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_KINEMATIC_CONSTRAINT, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_KINEMATIC_CONSTRAINT, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(accelerationConstraintSelectionOut);
        extract(deserializer, *accelerationConstraintSelectionOut);
        
        assert(velocityConstraintSelectionOut);
        extract(deserializer, *velocityConstraintSelectionOut);
        
        assert(angularConstraintSelectionOut);
        extract(deserializer, *angularConstraintSelectionOut);
        
        if( deserializer.remaining() != 0 )
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
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_KINEMATIC_CONSTRAINT, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult loadKinematicConstraint(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_KINEMATIC_CONSTRAINT, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult defaultKinematicConstraint(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_KINEMATIC_CONSTRAINT, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const InitializationConfiguration& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
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
}
void extract(Serializer& serializer, InitializationConfiguration& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
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
}

void insert(Serializer& serializer, const InitializationConfiguration::Response& self)
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
void extract(Serializer& serializer, InitializationConfiguration::Response& self)
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
    
    assert(initialPosition || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, initialPosition[i]);
    
    assert(initialVelocity || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, initialVelocity[i]);
    
    insert(serializer, referenceFrameSelector);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_INITIALIZATION_CONFIGURATION, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult readInitializationConfiguration(C::mip_interface& device, uint8_t* waitForRunCommandOut, InitializationConfiguration::InitialConditionSource* initialCondSrcOut, InitializationConfiguration::AlignmentSelector* autoHeadingAlignmentSelectorOut, float* initialHeadingOut, float* initialPitchOut, float* initialRollOut, float* initialPositionOut, float* initialVelocityOut, FilterReferenceFrame* referenceFrameSelectorOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_INITIALIZATION_CONFIGURATION, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_INITIALIZATION_CONFIGURATION, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
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
        
        assert(initialPositionOut || (3 == 0));
        for(unsigned int i=0; i < 3; i++)
            extract(deserializer, initialPositionOut[i]);
        
        assert(initialVelocityOut || (3 == 0));
        for(unsigned int i=0; i < 3; i++)
            extract(deserializer, initialVelocityOut[i]);
        
        assert(referenceFrameSelectorOut);
        extract(deserializer, *referenceFrameSelectorOut);
        
        if( deserializer.remaining() != 0 )
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
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_INITIALIZATION_CONFIGURATION, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult loadInitializationConfiguration(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_INITIALIZATION_CONFIGURATION, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult defaultInitializationConfiguration(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_INITIALIZATION_CONFIGURATION, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const AdaptiveFilterOptions& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.level);
        
        insert(serializer, self.time_limit);
        
    }
}
void extract(Serializer& serializer, AdaptiveFilterOptions& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        extract(serializer, self.level);
        
        extract(serializer, self.time_limit);
        
    }
}

void insert(Serializer& serializer, const AdaptiveFilterOptions::Response& self)
{
    insert(serializer, self.level);
    
    insert(serializer, self.time_limit);
    
}
void extract(Serializer& serializer, AdaptiveFilterOptions::Response& self)
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
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ADAPTIVE_FILTER_OPTIONS, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult readAdaptiveFilterOptions(C::mip_interface& device, uint8_t* levelOut, uint16_t* timeLimitOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_ADAPTIVE_FILTER_OPTIONS, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_ADAPTIVE_FILTER_OPTIONS, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(levelOut);
        extract(deserializer, *levelOut);
        
        assert(timeLimitOut);
        extract(deserializer, *timeLimitOut);
        
        if( deserializer.remaining() != 0 )
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
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ADAPTIVE_FILTER_OPTIONS, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult loadAdaptiveFilterOptions(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ADAPTIVE_FILTER_OPTIONS, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult defaultAdaptiveFilterOptions(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ADAPTIVE_FILTER_OPTIONS, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const MultiAntennaOffset& self)
{
    insert(serializer, self.function);
    
    insert(serializer, self.receiver_id);
    
    if( self.function == FunctionSelector::WRITE )
    {
        for(unsigned int i=0; i < 3; i++)
            insert(serializer, self.antenna_offset[i]);
        
    }
}
void extract(Serializer& serializer, MultiAntennaOffset& self)
{
    extract(serializer, self.function);
    
    extract(serializer, self.receiver_id);
    
    if( self.function == FunctionSelector::WRITE )
    {
        for(unsigned int i=0; i < 3; i++)
            extract(serializer, self.antenna_offset[i]);
        
    }
}

void insert(Serializer& serializer, const MultiAntennaOffset::Response& self)
{
    insert(serializer, self.receiver_id);
    
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.antenna_offset[i]);
    
}
void extract(Serializer& serializer, MultiAntennaOffset::Response& self)
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
    
    assert(antennaOffset || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, antennaOffset[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_MULTI_ANTENNA_OFFSET, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult readMultiAntennaOffset(C::mip_interface& device, uint8_t receiverId, float* antennaOffsetOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    insert(serializer, receiverId);
    
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_MULTI_ANTENNA_OFFSET, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_MULTI_ANTENNA_OFFSET, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        extract(deserializer, receiverId);
        
        assert(antennaOffsetOut || (3 == 0));
        for(unsigned int i=0; i < 3; i++)
            extract(deserializer, antennaOffsetOut[i]);
        
        if( deserializer.remaining() != 0 )
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
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_MULTI_ANTENNA_OFFSET, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult loadMultiAntennaOffset(C::mip_interface& device, uint8_t receiverId)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    insert(serializer, receiverId);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_MULTI_ANTENNA_OFFSET, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult defaultMultiAntennaOffset(C::mip_interface& device, uint8_t receiverId)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    insert(serializer, receiverId);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_MULTI_ANTENNA_OFFSET, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const RelPosConfiguration& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.source);
        
        insert(serializer, self.reference_frame_selector);
        
        for(unsigned int i=0; i < 3; i++)
            insert(serializer, self.reference_coordinates[i]);
        
    }
}
void extract(Serializer& serializer, RelPosConfiguration& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        extract(serializer, self.source);
        
        extract(serializer, self.reference_frame_selector);
        
        for(unsigned int i=0; i < 3; i++)
            extract(serializer, self.reference_coordinates[i]);
        
    }
}

void insert(Serializer& serializer, const RelPosConfiguration::Response& self)
{
    insert(serializer, self.source);
    
    insert(serializer, self.reference_frame_selector);
    
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.reference_coordinates[i]);
    
}
void extract(Serializer& serializer, RelPosConfiguration::Response& self)
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
    
    assert(referenceCoordinates || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, referenceCoordinates[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_REL_POS_CONFIGURATION, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult readRelPosConfiguration(C::mip_interface& device, uint8_t* sourceOut, FilterReferenceFrame* referenceFrameSelectorOut, double* referenceCoordinatesOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_REL_POS_CONFIGURATION, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_REL_POS_CONFIGURATION, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(sourceOut);
        extract(deserializer, *sourceOut);
        
        assert(referenceFrameSelectorOut);
        extract(deserializer, *referenceFrameSelectorOut);
        
        assert(referenceCoordinatesOut || (3 == 0));
        for(unsigned int i=0; i < 3; i++)
            extract(deserializer, referenceCoordinatesOut[i]);
        
        if( deserializer.remaining() != 0 )
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
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_REL_POS_CONFIGURATION, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult loadRelPosConfiguration(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_REL_POS_CONFIGURATION, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult defaultRelPosConfiguration(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_REL_POS_CONFIGURATION, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const RefPointLeverArm& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.ref_point_sel);
        
        for(unsigned int i=0; i < 3; i++)
            insert(serializer, self.lever_arm_offset[i]);
        
    }
}
void extract(Serializer& serializer, RefPointLeverArm& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        extract(serializer, self.ref_point_sel);
        
        for(unsigned int i=0; i < 3; i++)
            extract(serializer, self.lever_arm_offset[i]);
        
    }
}

void insert(Serializer& serializer, const RefPointLeverArm::Response& self)
{
    insert(serializer, self.ref_point_sel);
    
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.lever_arm_offset[i]);
    
}
void extract(Serializer& serializer, RefPointLeverArm::Response& self)
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
    
    assert(leverArmOffset || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, leverArmOffset[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_REF_POINT_LEVER_ARM, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult readRefPointLeverArm(C::mip_interface& device, RefPointLeverArm::ReferencePointSelector* refPointSelOut, float* leverArmOffsetOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_REF_POINT_LEVER_ARM, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_REF_POINT_LEVER_ARM, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(refPointSelOut);
        extract(deserializer, *refPointSelOut);
        
        assert(leverArmOffsetOut || (3 == 0));
        for(unsigned int i=0; i < 3; i++)
            extract(deserializer, leverArmOffsetOut[i]);
        
        if( deserializer.remaining() != 0 )
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
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_REF_POINT_LEVER_ARM, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult loadRefPointLeverArm(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_REF_POINT_LEVER_ARM, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult defaultRefPointLeverArm(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_REF_POINT_LEVER_ARM, buffer, (uint8_t)mip_serializer_length(&serializer));
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
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SPEED_MEASUREMENT, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const SpeedLeverArm& self)
{
    insert(serializer, self.function);
    
    insert(serializer, self.source);
    
    if( self.function == FunctionSelector::WRITE )
    {
        for(unsigned int i=0; i < 3; i++)
            insert(serializer, self.lever_arm_offset[i]);
        
    }
}
void extract(Serializer& serializer, SpeedLeverArm& self)
{
    extract(serializer, self.function);
    
    extract(serializer, self.source);
    
    if( self.function == FunctionSelector::WRITE )
    {
        for(unsigned int i=0; i < 3; i++)
            extract(serializer, self.lever_arm_offset[i]);
        
    }
}

void insert(Serializer& serializer, const SpeedLeverArm::Response& self)
{
    insert(serializer, self.source);
    
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.lever_arm_offset[i]);
    
}
void extract(Serializer& serializer, SpeedLeverArm::Response& self)
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
    
    assert(leverArmOffset || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, leverArmOffset[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SPEED_LEVER_ARM, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult readSpeedLeverArm(C::mip_interface& device, uint8_t source, float* leverArmOffsetOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    insert(serializer, source);
    
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_SPEED_LEVER_ARM, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_SPEED_LEVER_ARM, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        extract(deserializer, source);
        
        assert(leverArmOffsetOut || (3 == 0));
        for(unsigned int i=0; i < 3; i++)
            extract(deserializer, leverArmOffsetOut[i]);
        
        if( deserializer.remaining() != 0 )
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
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SPEED_LEVER_ARM, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult loadSpeedLeverArm(C::mip_interface& device, uint8_t source)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    insert(serializer, source);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SPEED_LEVER_ARM, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult defaultSpeedLeverArm(C::mip_interface& device, uint8_t source)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    insert(serializer, source);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SPEED_LEVER_ARM, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const WheeledVehicleConstraintControl& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.enable);
        
    }
}
void extract(Serializer& serializer, WheeledVehicleConstraintControl& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        extract(serializer, self.enable);
        
    }
}

void insert(Serializer& serializer, const WheeledVehicleConstraintControl::Response& self)
{
    insert(serializer, self.enable);
    
}
void extract(Serializer& serializer, WheeledVehicleConstraintControl::Response& self)
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
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_VEHICLE_CONSTRAINT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult readWheeledVehicleConstraintControl(C::mip_interface& device, uint8_t* enableOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_VEHICLE_CONSTRAINT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_VEHICLE_CONSTRAINT_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(enableOut);
        extract(deserializer, *enableOut);
        
        if( deserializer.remaining() != 0 )
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
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_VEHICLE_CONSTRAINT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult loadWheeledVehicleConstraintControl(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_VEHICLE_CONSTRAINT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult defaultWheeledVehicleConstraintControl(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_VEHICLE_CONSTRAINT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const VerticalGyroConstraintControl& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.enable);
        
    }
}
void extract(Serializer& serializer, VerticalGyroConstraintControl& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        extract(serializer, self.enable);
        
    }
}

void insert(Serializer& serializer, const VerticalGyroConstraintControl::Response& self)
{
    insert(serializer, self.enable);
    
}
void extract(Serializer& serializer, VerticalGyroConstraintControl::Response& self)
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
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GYRO_CONSTRAINT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult readVerticalGyroConstraintControl(C::mip_interface& device, uint8_t* enableOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GYRO_CONSTRAINT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_GYRO_CONSTRAINT_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(enableOut);
        extract(deserializer, *enableOut);
        
        if( deserializer.remaining() != 0 )
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
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GYRO_CONSTRAINT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult loadVerticalGyroConstraintControl(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GYRO_CONSTRAINT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult defaultVerticalGyroConstraintControl(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GYRO_CONSTRAINT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const GnssAntennaCalControl& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.enable);
        
        insert(serializer, self.max_offset);
        
    }
}
void extract(Serializer& serializer, GnssAntennaCalControl& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        extract(serializer, self.enable);
        
        extract(serializer, self.max_offset);
        
    }
}

void insert(Serializer& serializer, const GnssAntennaCalControl::Response& self)
{
    insert(serializer, self.enable);
    
    insert(serializer, self.max_offset);
    
}
void extract(Serializer& serializer, GnssAntennaCalControl::Response& self)
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
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ANTENNA_CALIBRATION_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult readGnssAntennaCalControl(C::mip_interface& device, uint8_t* enableOut, float* maxOffsetOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_ANTENNA_CALIBRATION_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_ANTENNA_CALIBRATION_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(enableOut);
        extract(deserializer, *enableOut);
        
        assert(maxOffsetOut);
        extract(deserializer, *maxOffsetOut);
        
        if( deserializer.remaining() != 0 )
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
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ANTENNA_CALIBRATION_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult loadGnssAntennaCalControl(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ANTENNA_CALIBRATION_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult defaultGnssAntennaCalControl(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ANTENNA_CALIBRATION_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const HardIronOffsetNoise& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.x);
        
        insert(serializer, self.y);
        
        insert(serializer, self.z);
        
    }
}
void extract(Serializer& serializer, HardIronOffsetNoise& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        extract(serializer, self.x);
        
        extract(serializer, self.y);
        
        extract(serializer, self.z);
        
    }
}

void insert(Serializer& serializer, const HardIronOffsetNoise::Response& self)
{
    insert(serializer, self.x);
    
    insert(serializer, self.y);
    
    insert(serializer, self.z);
    
}
void extract(Serializer& serializer, HardIronOffsetNoise::Response& self)
{
    extract(serializer, self.x);
    
    extract(serializer, self.y);
    
    extract(serializer, self.z);
    
}

CmdResult writeHardIronOffsetNoise(C::mip_interface& device, float x, float y, float z)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, x);
    
    insert(serializer, y);
    
    insert(serializer, z);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_HARD_IRON_OFFSET_NOISE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult readHardIronOffsetNoise(C::mip_interface& device, float* xOut, float* yOut, float* zOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_HARD_IRON_OFFSET_NOISE, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_HARD_IRON_OFFSET_NOISE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(xOut);
        extract(deserializer, *xOut);
        
        assert(yOut);
        extract(deserializer, *yOut);
        
        assert(zOut);
        extract(deserializer, *zOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
CmdResult saveHardIronOffsetNoise(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_HARD_IRON_OFFSET_NOISE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult loadHardIronOffsetNoise(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_HARD_IRON_OFFSET_NOISE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult defaultHardIronOffsetNoise(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_HARD_IRON_OFFSET_NOISE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const MagneticDeclinationSource& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.source);
        
        insert(serializer, self.declination);
        
    }
}
void extract(Serializer& serializer, MagneticDeclinationSource& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        extract(serializer, self.source);
        
        extract(serializer, self.declination);
        
    }
}

void insert(Serializer& serializer, const MagneticDeclinationSource::Response& self)
{
    insert(serializer, self.source);
    
    insert(serializer, self.declination);
    
}
void extract(Serializer& serializer, MagneticDeclinationSource::Response& self)
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
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_DECLINATION_SOURCE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult readMagneticDeclinationSource(C::mip_interface& device, FilterMagDeclinationSource* sourceOut, float* declinationOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_DECLINATION_SOURCE, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_DECLINATION_SOURCE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(sourceOut);
        extract(deserializer, *sourceOut);
        
        assert(declinationOut);
        extract(deserializer, *declinationOut);
        
        if( deserializer.remaining() != 0 )
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
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_DECLINATION_SOURCE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult loadMagneticDeclinationSource(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_DECLINATION_SOURCE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
CmdResult defaultMagneticDeclinationSource(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_DECLINATION_SOURCE, buffer, (uint8_t)mip_serializer_length(&serializer));
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
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SET_INITIAL_HEADING, buffer, (uint8_t)mip_serializer_length(&serializer));
}

} // namespace commands_filter
} // namespace mip

