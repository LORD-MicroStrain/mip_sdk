
#include "commands_filter.hpp"

#include <mip/mip_serialization.hpp>
#include <mip/mip_interface.h>

#include <assert.h>


namespace mip {
namespace C {
struct mip_interface;
} // namespace C

namespace commands_filter {

using namespace ::mip::C;

////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

void Reset::insert(Serializer& serializer) const
{
    (void)serializer;
}
void Reset::extract(Serializer& serializer)
{
    (void)serializer;
}

TypedResult<Reset> reset(C::mip_interface& device)
{
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_RESET_FILTER, NULL, 0);
}
void SetInitialAttitude::insert(Serializer& serializer) const
{
    serializer.insert(roll);
    
    serializer.insert(pitch);
    
    serializer.insert(heading);
    
}
void SetInitialAttitude::extract(Serializer& serializer)
{
    serializer.extract(roll);
    
    serializer.extract(pitch);
    
    serializer.extract(heading);
    
}

TypedResult<SetInitialAttitude> setInitialAttitude(C::mip_interface& device, float roll, float pitch, float heading)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(roll);
    
    serializer.insert(pitch);
    
    serializer.insert(heading);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SET_INITIAL_ATTITUDE, buffer, (uint8_t)serializer.usedLength());
}
void EstimationControl::insert(Serializer& serializer) const
{
    serializer.insert(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.insert(enable);
        
    }
}
void EstimationControl::extract(Serializer& serializer)
{
    serializer.extract(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.extract(enable);
        
    }
}

void EstimationControl::Response::insert(Serializer& serializer) const
{
    serializer.insert(enable);
    
}
void EstimationControl::Response::extract(Serializer& serializer)
{
    serializer.extract(enable);
    
}

TypedResult<EstimationControl> writeEstimationControl(C::mip_interface& device, EstimationControl::EnableFlags enable)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::WRITE);
    serializer.insert(enable);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ESTIMATION_CONTROL_FLAGS, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<EstimationControl> readEstimationControl(C::mip_interface& device, EstimationControl::EnableFlags* enableOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<EstimationControl> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_ESTIMATION_CONTROL_FLAGS, buffer, (uint8_t)serializer.usedLength(), REPLY_ESTIMATION_CONTROL_FLAGS, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(enableOut);
        deserializer.extract(*enableOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<EstimationControl> saveEstimationControl(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ESTIMATION_CONTROL_FLAGS, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<EstimationControl> loadEstimationControl(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ESTIMATION_CONTROL_FLAGS, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<EstimationControl> defaultEstimationControl(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ESTIMATION_CONTROL_FLAGS, buffer, (uint8_t)serializer.usedLength());
}
void ExternalGnssUpdate::insert(Serializer& serializer) const
{
    serializer.insert(gps_time);
    
    serializer.insert(gps_week);
    
    serializer.insert(latitude);
    
    serializer.insert(longitude);
    
    serializer.insert(height);
    
    serializer.insert(velocity);
    
    serializer.insert(pos_uncertainty);
    
    serializer.insert(vel_uncertainty);
    
}
void ExternalGnssUpdate::extract(Serializer& serializer)
{
    serializer.extract(gps_time);
    
    serializer.extract(gps_week);
    
    serializer.extract(latitude);
    
    serializer.extract(longitude);
    
    serializer.extract(height);
    
    serializer.extract(velocity);
    
    serializer.extract(pos_uncertainty);
    
    serializer.extract(vel_uncertainty);
    
}

TypedResult<ExternalGnssUpdate> externalGnssUpdate(C::mip_interface& device, double gpsTime, uint16_t gpsWeek, double latitude, double longitude, double height, const float* velocity, const float* posUncertainty, const float* velUncertainty)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(gpsTime);
    
    serializer.insert(gpsWeek);
    
    serializer.insert(latitude);
    
    serializer.insert(longitude);
    
    serializer.insert(height);
    
    assert(velocity);
    for(unsigned int i=0; i < 3; i++)
        serializer.insert(velocity[i]);
    
    assert(posUncertainty);
    for(unsigned int i=0; i < 3; i++)
        serializer.insert(posUncertainty[i]);
    
    assert(velUncertainty);
    for(unsigned int i=0; i < 3; i++)
        serializer.insert(velUncertainty[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_EXTERNAL_GNSS_UPDATE, buffer, (uint8_t)serializer.usedLength());
}
void ExternalHeadingUpdate::insert(Serializer& serializer) const
{
    serializer.insert(heading);
    
    serializer.insert(heading_uncertainty);
    
    serializer.insert(type);
    
}
void ExternalHeadingUpdate::extract(Serializer& serializer)
{
    serializer.extract(heading);
    
    serializer.extract(heading_uncertainty);
    
    serializer.extract(type);
    
}

TypedResult<ExternalHeadingUpdate> externalHeadingUpdate(C::mip_interface& device, float heading, float headingUncertainty, uint8_t type)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(heading);
    
    serializer.insert(headingUncertainty);
    
    serializer.insert(type);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_EXTERNAL_HEADING_UPDATE, buffer, (uint8_t)serializer.usedLength());
}
void ExternalHeadingUpdateWithTime::insert(Serializer& serializer) const
{
    serializer.insert(gps_time);
    
    serializer.insert(gps_week);
    
    serializer.insert(heading);
    
    serializer.insert(heading_uncertainty);
    
    serializer.insert(type);
    
}
void ExternalHeadingUpdateWithTime::extract(Serializer& serializer)
{
    serializer.extract(gps_time);
    
    serializer.extract(gps_week);
    
    serializer.extract(heading);
    
    serializer.extract(heading_uncertainty);
    
    serializer.extract(type);
    
}

TypedResult<ExternalHeadingUpdateWithTime> externalHeadingUpdateWithTime(C::mip_interface& device, double gpsTime, uint16_t gpsWeek, float heading, float headingUncertainty, uint8_t type)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(gpsTime);
    
    serializer.insert(gpsWeek);
    
    serializer.insert(heading);
    
    serializer.insert(headingUncertainty);
    
    serializer.insert(type);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_EXTERNAL_HEADING_UPDATE_WITH_TIME, buffer, (uint8_t)serializer.usedLength());
}
void TareOrientation::insert(Serializer& serializer) const
{
    serializer.insert(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.insert(axes);
        
    }
}
void TareOrientation::extract(Serializer& serializer)
{
    serializer.extract(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.extract(axes);
        
    }
}

void TareOrientation::Response::insert(Serializer& serializer) const
{
    serializer.insert(axes);
    
}
void TareOrientation::Response::extract(Serializer& serializer)
{
    serializer.extract(axes);
    
}

TypedResult<TareOrientation> writeTareOrientation(C::mip_interface& device, TareOrientation::MipTareAxes axes)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::WRITE);
    serializer.insert(axes);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_TARE_ORIENTATION, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<TareOrientation> readTareOrientation(C::mip_interface& device, TareOrientation::MipTareAxes* axesOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<TareOrientation> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_TARE_ORIENTATION, buffer, (uint8_t)serializer.usedLength(), REPLY_TARE_ORIENTATION, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(axesOut);
        deserializer.extract(*axesOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<TareOrientation> saveTareOrientation(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_TARE_ORIENTATION, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<TareOrientation> loadTareOrientation(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_TARE_ORIENTATION, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<TareOrientation> defaultTareOrientation(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_TARE_ORIENTATION, buffer, (uint8_t)serializer.usedLength());
}
void VehicleDynamicsMode::insert(Serializer& serializer) const
{
    serializer.insert(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.insert(mode);
        
    }
}
void VehicleDynamicsMode::extract(Serializer& serializer)
{
    serializer.extract(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.extract(mode);
        
    }
}

void VehicleDynamicsMode::Response::insert(Serializer& serializer) const
{
    serializer.insert(mode);
    
}
void VehicleDynamicsMode::Response::extract(Serializer& serializer)
{
    serializer.extract(mode);
    
}

TypedResult<VehicleDynamicsMode> writeVehicleDynamicsMode(C::mip_interface& device, VehicleDynamicsMode::DynamicsMode mode)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::WRITE);
    serializer.insert(mode);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_VEHICLE_DYNAMICS_MODE, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<VehicleDynamicsMode> readVehicleDynamicsMode(C::mip_interface& device, VehicleDynamicsMode::DynamicsMode* modeOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<VehicleDynamicsMode> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_VEHICLE_DYNAMICS_MODE, buffer, (uint8_t)serializer.usedLength(), REPLY_VEHICLE_DYNAMICS_MODE, buffer, &responseLength);
    
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
TypedResult<VehicleDynamicsMode> saveVehicleDynamicsMode(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_VEHICLE_DYNAMICS_MODE, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<VehicleDynamicsMode> loadVehicleDynamicsMode(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_VEHICLE_DYNAMICS_MODE, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<VehicleDynamicsMode> defaultVehicleDynamicsMode(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_VEHICLE_DYNAMICS_MODE, buffer, (uint8_t)serializer.usedLength());
}
void SensorToVehicleRotationEuler::insert(Serializer& serializer) const
{
    serializer.insert(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.insert(roll);
        
        serializer.insert(pitch);
        
        serializer.insert(yaw);
        
    }
}
void SensorToVehicleRotationEuler::extract(Serializer& serializer)
{
    serializer.extract(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.extract(roll);
        
        serializer.extract(pitch);
        
        serializer.extract(yaw);
        
    }
}

void SensorToVehicleRotationEuler::Response::insert(Serializer& serializer) const
{
    serializer.insert(roll);
    
    serializer.insert(pitch);
    
    serializer.insert(yaw);
    
}
void SensorToVehicleRotationEuler::Response::extract(Serializer& serializer)
{
    serializer.extract(roll);
    
    serializer.extract(pitch);
    
    serializer.extract(yaw);
    
}

TypedResult<SensorToVehicleRotationEuler> writeSensorToVehicleRotationEuler(C::mip_interface& device, float roll, float pitch, float yaw)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::WRITE);
    serializer.insert(roll);
    
    serializer.insert(pitch);
    
    serializer.insert(yaw);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_ROTATION_EULER, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<SensorToVehicleRotationEuler> readSensorToVehicleRotationEuler(C::mip_interface& device, float* rollOut, float* pitchOut, float* yawOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<SensorToVehicleRotationEuler> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_ROTATION_EULER, buffer, (uint8_t)serializer.usedLength(), REPLY_SENSOR2VEHICLE_ROTATION_EULER, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(rollOut);
        deserializer.extract(*rollOut);
        
        assert(pitchOut);
        deserializer.extract(*pitchOut);
        
        assert(yawOut);
        deserializer.extract(*yawOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<SensorToVehicleRotationEuler> saveSensorToVehicleRotationEuler(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_ROTATION_EULER, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<SensorToVehicleRotationEuler> loadSensorToVehicleRotationEuler(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_ROTATION_EULER, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<SensorToVehicleRotationEuler> defaultSensorToVehicleRotationEuler(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_ROTATION_EULER, buffer, (uint8_t)serializer.usedLength());
}
void SensorToVehicleRotationDcm::insert(Serializer& serializer) const
{
    serializer.insert(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.insert(dcm);
        
    }
}
void SensorToVehicleRotationDcm::extract(Serializer& serializer)
{
    serializer.extract(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.extract(dcm);
        
    }
}

void SensorToVehicleRotationDcm::Response::insert(Serializer& serializer) const
{
    serializer.insert(dcm);
    
}
void SensorToVehicleRotationDcm::Response::extract(Serializer& serializer)
{
    serializer.extract(dcm);
    
}

TypedResult<SensorToVehicleRotationDcm> writeSensorToVehicleRotationDcm(C::mip_interface& device, const float* dcm)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::WRITE);
    assert(dcm);
    for(unsigned int i=0; i < 9; i++)
        serializer.insert(dcm[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_ROTATION_DCM, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<SensorToVehicleRotationDcm> readSensorToVehicleRotationDcm(C::mip_interface& device, float* dcmOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<SensorToVehicleRotationDcm> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_ROTATION_DCM, buffer, (uint8_t)serializer.usedLength(), REPLY_SENSOR2VEHICLE_ROTATION_DCM, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(dcmOut);
        for(unsigned int i=0; i < 9; i++)
            deserializer.extract(dcmOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<SensorToVehicleRotationDcm> saveSensorToVehicleRotationDcm(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_ROTATION_DCM, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<SensorToVehicleRotationDcm> loadSensorToVehicleRotationDcm(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_ROTATION_DCM, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<SensorToVehicleRotationDcm> defaultSensorToVehicleRotationDcm(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_ROTATION_DCM, buffer, (uint8_t)serializer.usedLength());
}
void SensorToVehicleRotationQuaternion::insert(Serializer& serializer) const
{
    serializer.insert(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.insert(quat);
        
    }
}
void SensorToVehicleRotationQuaternion::extract(Serializer& serializer)
{
    serializer.extract(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.extract(quat);
        
    }
}

void SensorToVehicleRotationQuaternion::Response::insert(Serializer& serializer) const
{
    serializer.insert(quat);
    
}
void SensorToVehicleRotationQuaternion::Response::extract(Serializer& serializer)
{
    serializer.extract(quat);
    
}

TypedResult<SensorToVehicleRotationQuaternion> writeSensorToVehicleRotationQuaternion(C::mip_interface& device, const float* quat)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::WRITE);
    assert(quat);
    for(unsigned int i=0; i < 4; i++)
        serializer.insert(quat[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_ROTATION_QUATERNION, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<SensorToVehicleRotationQuaternion> readSensorToVehicleRotationQuaternion(C::mip_interface& device, float* quatOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<SensorToVehicleRotationQuaternion> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_ROTATION_QUATERNION, buffer, (uint8_t)serializer.usedLength(), REPLY_SENSOR2VEHICLE_ROTATION_QUATERNION, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(quatOut);
        for(unsigned int i=0; i < 4; i++)
            deserializer.extract(quatOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<SensorToVehicleRotationQuaternion> saveSensorToVehicleRotationQuaternion(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_ROTATION_QUATERNION, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<SensorToVehicleRotationQuaternion> loadSensorToVehicleRotationQuaternion(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_ROTATION_QUATERNION, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<SensorToVehicleRotationQuaternion> defaultSensorToVehicleRotationQuaternion(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_ROTATION_QUATERNION, buffer, (uint8_t)serializer.usedLength());
}
void SensorToVehicleOffset::insert(Serializer& serializer) const
{
    serializer.insert(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.insert(offset);
        
    }
}
void SensorToVehicleOffset::extract(Serializer& serializer)
{
    serializer.extract(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.extract(offset);
        
    }
}

void SensorToVehicleOffset::Response::insert(Serializer& serializer) const
{
    serializer.insert(offset);
    
}
void SensorToVehicleOffset::Response::extract(Serializer& serializer)
{
    serializer.extract(offset);
    
}

TypedResult<SensorToVehicleOffset> writeSensorToVehicleOffset(C::mip_interface& device, const float* offset)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::WRITE);
    assert(offset);
    for(unsigned int i=0; i < 3; i++)
        serializer.insert(offset[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_OFFSET, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<SensorToVehicleOffset> readSensorToVehicleOffset(C::mip_interface& device, float* offsetOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<SensorToVehicleOffset> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_OFFSET, buffer, (uint8_t)serializer.usedLength(), REPLY_SENSOR2VEHICLE_OFFSET, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(offsetOut);
        for(unsigned int i=0; i < 3; i++)
            deserializer.extract(offsetOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<SensorToVehicleOffset> saveSensorToVehicleOffset(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_OFFSET, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<SensorToVehicleOffset> loadSensorToVehicleOffset(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_OFFSET, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<SensorToVehicleOffset> defaultSensorToVehicleOffset(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_OFFSET, buffer, (uint8_t)serializer.usedLength());
}
void AntennaOffset::insert(Serializer& serializer) const
{
    serializer.insert(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.insert(offset);
        
    }
}
void AntennaOffset::extract(Serializer& serializer)
{
    serializer.extract(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.extract(offset);
        
    }
}

void AntennaOffset::Response::insert(Serializer& serializer) const
{
    serializer.insert(offset);
    
}
void AntennaOffset::Response::extract(Serializer& serializer)
{
    serializer.extract(offset);
    
}

TypedResult<AntennaOffset> writeAntennaOffset(C::mip_interface& device, const float* offset)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::WRITE);
    assert(offset);
    for(unsigned int i=0; i < 3; i++)
        serializer.insert(offset[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ANTENNA_OFFSET, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<AntennaOffset> readAntennaOffset(C::mip_interface& device, float* offsetOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<AntennaOffset> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_ANTENNA_OFFSET, buffer, (uint8_t)serializer.usedLength(), REPLY_ANTENNA_OFFSET, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(offsetOut);
        for(unsigned int i=0; i < 3; i++)
            deserializer.extract(offsetOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<AntennaOffset> saveAntennaOffset(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ANTENNA_OFFSET, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<AntennaOffset> loadAntennaOffset(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ANTENNA_OFFSET, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<AntennaOffset> defaultAntennaOffset(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ANTENNA_OFFSET, buffer, (uint8_t)serializer.usedLength());
}
void GnssSource::insert(Serializer& serializer) const
{
    serializer.insert(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.insert(source);
        
    }
}
void GnssSource::extract(Serializer& serializer)
{
    serializer.extract(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.extract(source);
        
    }
}

void GnssSource::Response::insert(Serializer& serializer) const
{
    serializer.insert(source);
    
}
void GnssSource::Response::extract(Serializer& serializer)
{
    serializer.extract(source);
    
}

TypedResult<GnssSource> writeGnssSource(C::mip_interface& device, GnssSource::Source source)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::WRITE);
    serializer.insert(source);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GNSS_SOURCE_CONTROL, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<GnssSource> readGnssSource(C::mip_interface& device, GnssSource::Source* sourceOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<GnssSource> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GNSS_SOURCE_CONTROL, buffer, (uint8_t)serializer.usedLength(), REPLY_GNSS_SOURCE_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(sourceOut);
        deserializer.extract(*sourceOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<GnssSource> saveGnssSource(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GNSS_SOURCE_CONTROL, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<GnssSource> loadGnssSource(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GNSS_SOURCE_CONTROL, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<GnssSource> defaultGnssSource(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GNSS_SOURCE_CONTROL, buffer, (uint8_t)serializer.usedLength());
}
void HeadingSource::insert(Serializer& serializer) const
{
    serializer.insert(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.insert(source);
        
    }
}
void HeadingSource::extract(Serializer& serializer)
{
    serializer.extract(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.extract(source);
        
    }
}

void HeadingSource::Response::insert(Serializer& serializer) const
{
    serializer.insert(source);
    
}
void HeadingSource::Response::extract(Serializer& serializer)
{
    serializer.extract(source);
    
}

TypedResult<HeadingSource> writeHeadingSource(C::mip_interface& device, HeadingSource::Source source)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::WRITE);
    serializer.insert(source);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_HEADING_UPDATE_CONTROL, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<HeadingSource> readHeadingSource(C::mip_interface& device, HeadingSource::Source* sourceOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<HeadingSource> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_HEADING_UPDATE_CONTROL, buffer, (uint8_t)serializer.usedLength(), REPLY_HEADING_UPDATE_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(sourceOut);
        deserializer.extract(*sourceOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<HeadingSource> saveHeadingSource(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_HEADING_UPDATE_CONTROL, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<HeadingSource> loadHeadingSource(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_HEADING_UPDATE_CONTROL, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<HeadingSource> defaultHeadingSource(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_HEADING_UPDATE_CONTROL, buffer, (uint8_t)serializer.usedLength());
}
void AutoInitControl::insert(Serializer& serializer) const
{
    serializer.insert(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.insert(enable);
        
    }
}
void AutoInitControl::extract(Serializer& serializer)
{
    serializer.extract(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.extract(enable);
        
    }
}

void AutoInitControl::Response::insert(Serializer& serializer) const
{
    serializer.insert(enable);
    
}
void AutoInitControl::Response::extract(Serializer& serializer)
{
    serializer.extract(enable);
    
}

TypedResult<AutoInitControl> writeAutoInitControl(C::mip_interface& device, uint8_t enable)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::WRITE);
    serializer.insert(enable);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_AUTOINIT_CONTROL, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<AutoInitControl> readAutoInitControl(C::mip_interface& device, uint8_t* enableOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<AutoInitControl> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_AUTOINIT_CONTROL, buffer, (uint8_t)serializer.usedLength(), REPLY_AUTOINIT_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(enableOut);
        deserializer.extract(*enableOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<AutoInitControl> saveAutoInitControl(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_AUTOINIT_CONTROL, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<AutoInitControl> loadAutoInitControl(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_AUTOINIT_CONTROL, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<AutoInitControl> defaultAutoInitControl(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_AUTOINIT_CONTROL, buffer, (uint8_t)serializer.usedLength());
}
void AccelNoise::insert(Serializer& serializer) const
{
    serializer.insert(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.insert(noise);
        
    }
}
void AccelNoise::extract(Serializer& serializer)
{
    serializer.extract(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.extract(noise);
        
    }
}

void AccelNoise::Response::insert(Serializer& serializer) const
{
    serializer.insert(noise);
    
}
void AccelNoise::Response::extract(Serializer& serializer)
{
    serializer.extract(noise);
    
}

TypedResult<AccelNoise> writeAccelNoise(C::mip_interface& device, const float* noise)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::WRITE);
    assert(noise);
    for(unsigned int i=0; i < 3; i++)
        serializer.insert(noise[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ACCEL_NOISE, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<AccelNoise> readAccelNoise(C::mip_interface& device, float* noiseOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<AccelNoise> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_ACCEL_NOISE, buffer, (uint8_t)serializer.usedLength(), REPLY_ACCEL_NOISE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(noiseOut);
        for(unsigned int i=0; i < 3; i++)
            deserializer.extract(noiseOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<AccelNoise> saveAccelNoise(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ACCEL_NOISE, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<AccelNoise> loadAccelNoise(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ACCEL_NOISE, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<AccelNoise> defaultAccelNoise(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ACCEL_NOISE, buffer, (uint8_t)serializer.usedLength());
}
void GyroNoise::insert(Serializer& serializer) const
{
    serializer.insert(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.insert(noise);
        
    }
}
void GyroNoise::extract(Serializer& serializer)
{
    serializer.extract(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.extract(noise);
        
    }
}

void GyroNoise::Response::insert(Serializer& serializer) const
{
    serializer.insert(noise);
    
}
void GyroNoise::Response::extract(Serializer& serializer)
{
    serializer.extract(noise);
    
}

TypedResult<GyroNoise> writeGyroNoise(C::mip_interface& device, const float* noise)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::WRITE);
    assert(noise);
    for(unsigned int i=0; i < 3; i++)
        serializer.insert(noise[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GYRO_NOISE, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<GyroNoise> readGyroNoise(C::mip_interface& device, float* noiseOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<GyroNoise> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GYRO_NOISE, buffer, (uint8_t)serializer.usedLength(), REPLY_GYRO_NOISE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(noiseOut);
        for(unsigned int i=0; i < 3; i++)
            deserializer.extract(noiseOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<GyroNoise> saveGyroNoise(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GYRO_NOISE, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<GyroNoise> loadGyroNoise(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GYRO_NOISE, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<GyroNoise> defaultGyroNoise(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GYRO_NOISE, buffer, (uint8_t)serializer.usedLength());
}
void AccelBiasModel::insert(Serializer& serializer) const
{
    serializer.insert(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.insert(beta);
        
        serializer.insert(noise);
        
    }
}
void AccelBiasModel::extract(Serializer& serializer)
{
    serializer.extract(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.extract(beta);
        
        serializer.extract(noise);
        
    }
}

void AccelBiasModel::Response::insert(Serializer& serializer) const
{
    serializer.insert(beta);
    
    serializer.insert(noise);
    
}
void AccelBiasModel::Response::extract(Serializer& serializer)
{
    serializer.extract(beta);
    
    serializer.extract(noise);
    
}

TypedResult<AccelBiasModel> writeAccelBiasModel(C::mip_interface& device, const float* beta, const float* noise)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::WRITE);
    assert(beta);
    for(unsigned int i=0; i < 3; i++)
        serializer.insert(beta[i]);
    
    assert(noise);
    for(unsigned int i=0; i < 3; i++)
        serializer.insert(noise[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ACCEL_BIAS_MODEL, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<AccelBiasModel> readAccelBiasModel(C::mip_interface& device, float* betaOut, float* noiseOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<AccelBiasModel> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_ACCEL_BIAS_MODEL, buffer, (uint8_t)serializer.usedLength(), REPLY_ACCEL_BIAS_MODEL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(betaOut);
        for(unsigned int i=0; i < 3; i++)
            deserializer.extract(betaOut[i]);
        
        assert(noiseOut);
        for(unsigned int i=0; i < 3; i++)
            deserializer.extract(noiseOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<AccelBiasModel> saveAccelBiasModel(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ACCEL_BIAS_MODEL, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<AccelBiasModel> loadAccelBiasModel(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ACCEL_BIAS_MODEL, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<AccelBiasModel> defaultAccelBiasModel(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ACCEL_BIAS_MODEL, buffer, (uint8_t)serializer.usedLength());
}
void GyroBiasModel::insert(Serializer& serializer) const
{
    serializer.insert(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.insert(beta);
        
        serializer.insert(noise);
        
    }
}
void GyroBiasModel::extract(Serializer& serializer)
{
    serializer.extract(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.extract(beta);
        
        serializer.extract(noise);
        
    }
}

void GyroBiasModel::Response::insert(Serializer& serializer) const
{
    serializer.insert(beta);
    
    serializer.insert(noise);
    
}
void GyroBiasModel::Response::extract(Serializer& serializer)
{
    serializer.extract(beta);
    
    serializer.extract(noise);
    
}

TypedResult<GyroBiasModel> writeGyroBiasModel(C::mip_interface& device, const float* beta, const float* noise)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::WRITE);
    assert(beta);
    for(unsigned int i=0; i < 3; i++)
        serializer.insert(beta[i]);
    
    assert(noise);
    for(unsigned int i=0; i < 3; i++)
        serializer.insert(noise[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GYRO_BIAS_MODEL, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<GyroBiasModel> readGyroBiasModel(C::mip_interface& device, float* betaOut, float* noiseOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<GyroBiasModel> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GYRO_BIAS_MODEL, buffer, (uint8_t)serializer.usedLength(), REPLY_GYRO_BIAS_MODEL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(betaOut);
        for(unsigned int i=0; i < 3; i++)
            deserializer.extract(betaOut[i]);
        
        assert(noiseOut);
        for(unsigned int i=0; i < 3; i++)
            deserializer.extract(noiseOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<GyroBiasModel> saveGyroBiasModel(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GYRO_BIAS_MODEL, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<GyroBiasModel> loadGyroBiasModel(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GYRO_BIAS_MODEL, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<GyroBiasModel> defaultGyroBiasModel(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GYRO_BIAS_MODEL, buffer, (uint8_t)serializer.usedLength());
}
void AltitudeAiding::insert(Serializer& serializer) const
{
    serializer.insert(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.insert(selector);
        
    }
}
void AltitudeAiding::extract(Serializer& serializer)
{
    serializer.extract(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.extract(selector);
        
    }
}

void AltitudeAiding::Response::insert(Serializer& serializer) const
{
    serializer.insert(selector);
    
}
void AltitudeAiding::Response::extract(Serializer& serializer)
{
    serializer.extract(selector);
    
}

TypedResult<AltitudeAiding> writeAltitudeAiding(C::mip_interface& device, AltitudeAiding::AidingSelector selector)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::WRITE);
    serializer.insert(selector);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ALTITUDE_AIDING_CONTROL, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<AltitudeAiding> readAltitudeAiding(C::mip_interface& device, AltitudeAiding::AidingSelector* selectorOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<AltitudeAiding> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_ALTITUDE_AIDING_CONTROL, buffer, (uint8_t)serializer.usedLength(), REPLY_ALTITUDE_AIDING_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(selectorOut);
        deserializer.extract(*selectorOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<AltitudeAiding> saveAltitudeAiding(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ALTITUDE_AIDING_CONTROL, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<AltitudeAiding> loadAltitudeAiding(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ALTITUDE_AIDING_CONTROL, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<AltitudeAiding> defaultAltitudeAiding(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ALTITUDE_AIDING_CONTROL, buffer, (uint8_t)serializer.usedLength());
}
void PitchRollAiding::insert(Serializer& serializer) const
{
    serializer.insert(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.insert(source);
        
    }
}
void PitchRollAiding::extract(Serializer& serializer)
{
    serializer.extract(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.extract(source);
        
    }
}

void PitchRollAiding::Response::insert(Serializer& serializer) const
{
    serializer.insert(source);
    
}
void PitchRollAiding::Response::extract(Serializer& serializer)
{
    serializer.extract(source);
    
}

TypedResult<PitchRollAiding> writePitchRollAiding(C::mip_interface& device, PitchRollAiding::AidingSource source)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::WRITE);
    serializer.insert(source);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SECONDARY_PITCH_ROLL_AIDING_CONTROL, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<PitchRollAiding> readPitchRollAiding(C::mip_interface& device, PitchRollAiding::AidingSource* sourceOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<PitchRollAiding> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_SECONDARY_PITCH_ROLL_AIDING_CONTROL, buffer, (uint8_t)serializer.usedLength(), REPLY_SECONDARY_PITCH_ROLL_AIDING_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(sourceOut);
        deserializer.extract(*sourceOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<PitchRollAiding> savePitchRollAiding(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SECONDARY_PITCH_ROLL_AIDING_CONTROL, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<PitchRollAiding> loadPitchRollAiding(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SECONDARY_PITCH_ROLL_AIDING_CONTROL, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<PitchRollAiding> defaultPitchRollAiding(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SECONDARY_PITCH_ROLL_AIDING_CONTROL, buffer, (uint8_t)serializer.usedLength());
}
void AutoZupt::insert(Serializer& serializer) const
{
    serializer.insert(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.insert(enable);
        
        serializer.insert(threshold);
        
    }
}
void AutoZupt::extract(Serializer& serializer)
{
    serializer.extract(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.extract(enable);
        
        serializer.extract(threshold);
        
    }
}

void AutoZupt::Response::insert(Serializer& serializer) const
{
    serializer.insert(enable);
    
    serializer.insert(threshold);
    
}
void AutoZupt::Response::extract(Serializer& serializer)
{
    serializer.extract(enable);
    
    serializer.extract(threshold);
    
}

TypedResult<AutoZupt> writeAutoZupt(C::mip_interface& device, uint8_t enable, float threshold)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::WRITE);
    serializer.insert(enable);
    
    serializer.insert(threshold);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ZUPT_CONTROL, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<AutoZupt> readAutoZupt(C::mip_interface& device, uint8_t* enableOut, float* thresholdOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<AutoZupt> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_ZUPT_CONTROL, buffer, (uint8_t)serializer.usedLength(), REPLY_ZUPT_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(enableOut);
        deserializer.extract(*enableOut);
        
        assert(thresholdOut);
        deserializer.extract(*thresholdOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<AutoZupt> saveAutoZupt(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ZUPT_CONTROL, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<AutoZupt> loadAutoZupt(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ZUPT_CONTROL, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<AutoZupt> defaultAutoZupt(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ZUPT_CONTROL, buffer, (uint8_t)serializer.usedLength());
}
void AutoAngularZupt::insert(Serializer& serializer) const
{
    serializer.insert(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.insert(enable);
        
        serializer.insert(threshold);
        
    }
}
void AutoAngularZupt::extract(Serializer& serializer)
{
    serializer.extract(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.extract(enable);
        
        serializer.extract(threshold);
        
    }
}

void AutoAngularZupt::Response::insert(Serializer& serializer) const
{
    serializer.insert(enable);
    
    serializer.insert(threshold);
    
}
void AutoAngularZupt::Response::extract(Serializer& serializer)
{
    serializer.extract(enable);
    
    serializer.extract(threshold);
    
}

TypedResult<AutoAngularZupt> writeAutoAngularZupt(C::mip_interface& device, uint8_t enable, float threshold)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::WRITE);
    serializer.insert(enable);
    
    serializer.insert(threshold);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ANGULAR_ZUPT_CONTROL, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<AutoAngularZupt> readAutoAngularZupt(C::mip_interface& device, uint8_t* enableOut, float* thresholdOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<AutoAngularZupt> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_ANGULAR_ZUPT_CONTROL, buffer, (uint8_t)serializer.usedLength(), REPLY_ANGULAR_ZUPT_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(enableOut);
        deserializer.extract(*enableOut);
        
        assert(thresholdOut);
        deserializer.extract(*thresholdOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<AutoAngularZupt> saveAutoAngularZupt(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ANGULAR_ZUPT_CONTROL, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<AutoAngularZupt> loadAutoAngularZupt(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ANGULAR_ZUPT_CONTROL, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<AutoAngularZupt> defaultAutoAngularZupt(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ANGULAR_ZUPT_CONTROL, buffer, (uint8_t)serializer.usedLength());
}
void CommandedZupt::insert(Serializer& serializer) const
{
    (void)serializer;
}
void CommandedZupt::extract(Serializer& serializer)
{
    (void)serializer;
}

TypedResult<CommandedZupt> commandedZupt(C::mip_interface& device)
{
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_COMMANDED_ZUPT, NULL, 0);
}
void CommandedAngularZupt::insert(Serializer& serializer) const
{
    (void)serializer;
}
void CommandedAngularZupt::extract(Serializer& serializer)
{
    (void)serializer;
}

TypedResult<CommandedAngularZupt> commandedAngularZupt(C::mip_interface& device)
{
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_COMMANDED_ANGULAR_ZUPT, NULL, 0);
}
void MagCaptureAutoCal::insert(Serializer& serializer) const
{
    serializer.insert(function);
    
}
void MagCaptureAutoCal::extract(Serializer& serializer)
{
    serializer.extract(function);
    
}

TypedResult<MagCaptureAutoCal> writeMagCaptureAutoCal(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::WRITE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_MAG_CAPTURE_AUTO_CALIBRATION, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<MagCaptureAutoCal> saveMagCaptureAutoCal(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_MAG_CAPTURE_AUTO_CALIBRATION, buffer, (uint8_t)serializer.usedLength());
}
void GravityNoise::insert(Serializer& serializer) const
{
    serializer.insert(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.insert(noise);
        
    }
}
void GravityNoise::extract(Serializer& serializer)
{
    serializer.extract(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.extract(noise);
        
    }
}

void GravityNoise::Response::insert(Serializer& serializer) const
{
    serializer.insert(noise);
    
}
void GravityNoise::Response::extract(Serializer& serializer)
{
    serializer.extract(noise);
    
}

TypedResult<GravityNoise> writeGravityNoise(C::mip_interface& device, const float* noise)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::WRITE);
    assert(noise);
    for(unsigned int i=0; i < 3; i++)
        serializer.insert(noise[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GRAVITY_NOISE, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<GravityNoise> readGravityNoise(C::mip_interface& device, float* noiseOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<GravityNoise> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GRAVITY_NOISE, buffer, (uint8_t)serializer.usedLength(), REPLY_GRAVITY_NOISE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(noiseOut);
        for(unsigned int i=0; i < 3; i++)
            deserializer.extract(noiseOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<GravityNoise> saveGravityNoise(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GRAVITY_NOISE, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<GravityNoise> loadGravityNoise(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GRAVITY_NOISE, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<GravityNoise> defaultGravityNoise(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GRAVITY_NOISE, buffer, (uint8_t)serializer.usedLength());
}
void PressureAltitudeNoise::insert(Serializer& serializer) const
{
    serializer.insert(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.insert(noise);
        
    }
}
void PressureAltitudeNoise::extract(Serializer& serializer)
{
    serializer.extract(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.extract(noise);
        
    }
}

void PressureAltitudeNoise::Response::insert(Serializer& serializer) const
{
    serializer.insert(noise);
    
}
void PressureAltitudeNoise::Response::extract(Serializer& serializer)
{
    serializer.extract(noise);
    
}

TypedResult<PressureAltitudeNoise> writePressureAltitudeNoise(C::mip_interface& device, float noise)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::WRITE);
    serializer.insert(noise);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_PRESSURE_NOISE, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<PressureAltitudeNoise> readPressureAltitudeNoise(C::mip_interface& device, float* noiseOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<PressureAltitudeNoise> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_PRESSURE_NOISE, buffer, (uint8_t)serializer.usedLength(), REPLY_PRESSURE_NOISE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(noiseOut);
        deserializer.extract(*noiseOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<PressureAltitudeNoise> savePressureAltitudeNoise(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_PRESSURE_NOISE, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<PressureAltitudeNoise> loadPressureAltitudeNoise(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_PRESSURE_NOISE, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<PressureAltitudeNoise> defaultPressureAltitudeNoise(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_PRESSURE_NOISE, buffer, (uint8_t)serializer.usedLength());
}
void HardIronOffsetNoise::insert(Serializer& serializer) const
{
    serializer.insert(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.insert(noise);
        
    }
}
void HardIronOffsetNoise::extract(Serializer& serializer)
{
    serializer.extract(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.extract(noise);
        
    }
}

void HardIronOffsetNoise::Response::insert(Serializer& serializer) const
{
    serializer.insert(noise);
    
}
void HardIronOffsetNoise::Response::extract(Serializer& serializer)
{
    serializer.extract(noise);
    
}

TypedResult<HardIronOffsetNoise> writeHardIronOffsetNoise(C::mip_interface& device, const float* noise)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::WRITE);
    assert(noise);
    for(unsigned int i=0; i < 3; i++)
        serializer.insert(noise[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_HARD_IRON_OFFSET_NOISE, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<HardIronOffsetNoise> readHardIronOffsetNoise(C::mip_interface& device, float* noiseOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<HardIronOffsetNoise> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_HARD_IRON_OFFSET_NOISE, buffer, (uint8_t)serializer.usedLength(), REPLY_HARD_IRON_OFFSET_NOISE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(noiseOut);
        for(unsigned int i=0; i < 3; i++)
            deserializer.extract(noiseOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<HardIronOffsetNoise> saveHardIronOffsetNoise(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_HARD_IRON_OFFSET_NOISE, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<HardIronOffsetNoise> loadHardIronOffsetNoise(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_HARD_IRON_OFFSET_NOISE, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<HardIronOffsetNoise> defaultHardIronOffsetNoise(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_HARD_IRON_OFFSET_NOISE, buffer, (uint8_t)serializer.usedLength());
}
void SoftIronMatrixNoise::insert(Serializer& serializer) const
{
    serializer.insert(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.insert(noise);
        
    }
}
void SoftIronMatrixNoise::extract(Serializer& serializer)
{
    serializer.extract(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.extract(noise);
        
    }
}

void SoftIronMatrixNoise::Response::insert(Serializer& serializer) const
{
    serializer.insert(noise);
    
}
void SoftIronMatrixNoise::Response::extract(Serializer& serializer)
{
    serializer.extract(noise);
    
}

TypedResult<SoftIronMatrixNoise> writeSoftIronMatrixNoise(C::mip_interface& device, const float* noise)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::WRITE);
    assert(noise);
    for(unsigned int i=0; i < 9; i++)
        serializer.insert(noise[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SOFT_IRON_MATRIX_NOISE, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<SoftIronMatrixNoise> readSoftIronMatrixNoise(C::mip_interface& device, float* noiseOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<SoftIronMatrixNoise> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_SOFT_IRON_MATRIX_NOISE, buffer, (uint8_t)serializer.usedLength(), REPLY_SOFT_IRON_MATRIX_NOISE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(noiseOut);
        for(unsigned int i=0; i < 9; i++)
            deserializer.extract(noiseOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<SoftIronMatrixNoise> saveSoftIronMatrixNoise(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SOFT_IRON_MATRIX_NOISE, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<SoftIronMatrixNoise> loadSoftIronMatrixNoise(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SOFT_IRON_MATRIX_NOISE, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<SoftIronMatrixNoise> defaultSoftIronMatrixNoise(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SOFT_IRON_MATRIX_NOISE, buffer, (uint8_t)serializer.usedLength());
}
void MagNoise::insert(Serializer& serializer) const
{
    serializer.insert(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.insert(noise);
        
    }
}
void MagNoise::extract(Serializer& serializer)
{
    serializer.extract(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.extract(noise);
        
    }
}

void MagNoise::Response::insert(Serializer& serializer) const
{
    serializer.insert(noise);
    
}
void MagNoise::Response::extract(Serializer& serializer)
{
    serializer.extract(noise);
    
}

TypedResult<MagNoise> writeMagNoise(C::mip_interface& device, const float* noise)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::WRITE);
    assert(noise);
    for(unsigned int i=0; i < 3; i++)
        serializer.insert(noise[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_MAG_NOISE, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<MagNoise> readMagNoise(C::mip_interface& device, float* noiseOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<MagNoise> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_MAG_NOISE, buffer, (uint8_t)serializer.usedLength(), REPLY_MAG_NOISE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(noiseOut);
        for(unsigned int i=0; i < 3; i++)
            deserializer.extract(noiseOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<MagNoise> saveMagNoise(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_MAG_NOISE, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<MagNoise> loadMagNoise(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_MAG_NOISE, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<MagNoise> defaultMagNoise(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_MAG_NOISE, buffer, (uint8_t)serializer.usedLength());
}
void InclinationSource::insert(Serializer& serializer) const
{
    serializer.insert(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.insert(source);
        
        serializer.insert(inclination);
        
    }
}
void InclinationSource::extract(Serializer& serializer)
{
    serializer.extract(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.extract(source);
        
        serializer.extract(inclination);
        
    }
}

void InclinationSource::Response::insert(Serializer& serializer) const
{
    serializer.insert(source);
    
    serializer.insert(inclination);
    
}
void InclinationSource::Response::extract(Serializer& serializer)
{
    serializer.extract(source);
    
    serializer.extract(inclination);
    
}

TypedResult<InclinationSource> writeInclinationSource(C::mip_interface& device, FilterMagParamSource source, float inclination)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::WRITE);
    serializer.insert(source);
    
    serializer.insert(inclination);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_INCLINATION_SOURCE, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<InclinationSource> readInclinationSource(C::mip_interface& device, FilterMagParamSource* sourceOut, float* inclinationOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<InclinationSource> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_INCLINATION_SOURCE, buffer, (uint8_t)serializer.usedLength(), REPLY_INCLINATION_SOURCE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(sourceOut);
        deserializer.extract(*sourceOut);
        
        assert(inclinationOut);
        deserializer.extract(*inclinationOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<InclinationSource> saveInclinationSource(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_INCLINATION_SOURCE, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<InclinationSource> loadInclinationSource(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_INCLINATION_SOURCE, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<InclinationSource> defaultInclinationSource(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_INCLINATION_SOURCE, buffer, (uint8_t)serializer.usedLength());
}
void MagneticDeclinationSource::insert(Serializer& serializer) const
{
    serializer.insert(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.insert(source);
        
        serializer.insert(declination);
        
    }
}
void MagneticDeclinationSource::extract(Serializer& serializer)
{
    serializer.extract(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.extract(source);
        
        serializer.extract(declination);
        
    }
}

void MagneticDeclinationSource::Response::insert(Serializer& serializer) const
{
    serializer.insert(source);
    
    serializer.insert(declination);
    
}
void MagneticDeclinationSource::Response::extract(Serializer& serializer)
{
    serializer.extract(source);
    
    serializer.extract(declination);
    
}

TypedResult<MagneticDeclinationSource> writeMagneticDeclinationSource(C::mip_interface& device, FilterMagParamSource source, float declination)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::WRITE);
    serializer.insert(source);
    
    serializer.insert(declination);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_DECLINATION_SOURCE, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<MagneticDeclinationSource> readMagneticDeclinationSource(C::mip_interface& device, FilterMagParamSource* sourceOut, float* declinationOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<MagneticDeclinationSource> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_DECLINATION_SOURCE, buffer, (uint8_t)serializer.usedLength(), REPLY_DECLINATION_SOURCE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(sourceOut);
        deserializer.extract(*sourceOut);
        
        assert(declinationOut);
        deserializer.extract(*declinationOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<MagneticDeclinationSource> saveMagneticDeclinationSource(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_DECLINATION_SOURCE, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<MagneticDeclinationSource> loadMagneticDeclinationSource(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_DECLINATION_SOURCE, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<MagneticDeclinationSource> defaultMagneticDeclinationSource(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_DECLINATION_SOURCE, buffer, (uint8_t)serializer.usedLength());
}
void MagFieldMagnitudeSource::insert(Serializer& serializer) const
{
    serializer.insert(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.insert(source);
        
        serializer.insert(magnitude);
        
    }
}
void MagFieldMagnitudeSource::extract(Serializer& serializer)
{
    serializer.extract(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.extract(source);
        
        serializer.extract(magnitude);
        
    }
}

void MagFieldMagnitudeSource::Response::insert(Serializer& serializer) const
{
    serializer.insert(source);
    
    serializer.insert(magnitude);
    
}
void MagFieldMagnitudeSource::Response::extract(Serializer& serializer)
{
    serializer.extract(source);
    
    serializer.extract(magnitude);
    
}

TypedResult<MagFieldMagnitudeSource> writeMagFieldMagnitudeSource(C::mip_interface& device, FilterMagParamSource source, float magnitude)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::WRITE);
    serializer.insert(source);
    
    serializer.insert(magnitude);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_MAGNETIC_MAGNITUDE_SOURCE, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<MagFieldMagnitudeSource> readMagFieldMagnitudeSource(C::mip_interface& device, FilterMagParamSource* sourceOut, float* magnitudeOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<MagFieldMagnitudeSource> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_MAGNETIC_MAGNITUDE_SOURCE, buffer, (uint8_t)serializer.usedLength(), REPLY_MAGNETIC_MAGNITUDE_SOURCE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(sourceOut);
        deserializer.extract(*sourceOut);
        
        assert(magnitudeOut);
        deserializer.extract(*magnitudeOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<MagFieldMagnitudeSource> saveMagFieldMagnitudeSource(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_MAGNETIC_MAGNITUDE_SOURCE, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<MagFieldMagnitudeSource> loadMagFieldMagnitudeSource(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_MAGNETIC_MAGNITUDE_SOURCE, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<MagFieldMagnitudeSource> defaultMagFieldMagnitudeSource(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_MAGNETIC_MAGNITUDE_SOURCE, buffer, (uint8_t)serializer.usedLength());
}
void ReferencePosition::insert(Serializer& serializer) const
{
    serializer.insert(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.insert(enable);
        
        serializer.insert(latitude);
        
        serializer.insert(longitude);
        
        serializer.insert(altitude);
        
    }
}
void ReferencePosition::extract(Serializer& serializer)
{
    serializer.extract(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.extract(enable);
        
        serializer.extract(latitude);
        
        serializer.extract(longitude);
        
        serializer.extract(altitude);
        
    }
}

void ReferencePosition::Response::insert(Serializer& serializer) const
{
    serializer.insert(enable);
    
    serializer.insert(latitude);
    
    serializer.insert(longitude);
    
    serializer.insert(altitude);
    
}
void ReferencePosition::Response::extract(Serializer& serializer)
{
    serializer.extract(enable);
    
    serializer.extract(latitude);
    
    serializer.extract(longitude);
    
    serializer.extract(altitude);
    
}

TypedResult<ReferencePosition> writeReferencePosition(C::mip_interface& device, bool enable, double latitude, double longitude, double altitude)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::WRITE);
    serializer.insert(enable);
    
    serializer.insert(latitude);
    
    serializer.insert(longitude);
    
    serializer.insert(altitude);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_REFERENCE_POSITION, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<ReferencePosition> readReferencePosition(C::mip_interface& device, bool* enableOut, double* latitudeOut, double* longitudeOut, double* altitudeOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<ReferencePosition> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_REFERENCE_POSITION, buffer, (uint8_t)serializer.usedLength(), REPLY_REFERENCE_POSITION, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(enableOut);
        deserializer.extract(*enableOut);
        
        assert(latitudeOut);
        deserializer.extract(*latitudeOut);
        
        assert(longitudeOut);
        deserializer.extract(*longitudeOut);
        
        assert(altitudeOut);
        deserializer.extract(*altitudeOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<ReferencePosition> saveReferencePosition(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_REFERENCE_POSITION, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<ReferencePosition> loadReferencePosition(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_REFERENCE_POSITION, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<ReferencePosition> defaultReferencePosition(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_REFERENCE_POSITION, buffer, (uint8_t)serializer.usedLength());
}
void AccelMagnitudeErrorAdaptiveMeasurement::insert(Serializer& serializer) const
{
    serializer.insert(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.insert(adaptive_measurement);
        
        serializer.insert(frequency);
        
        serializer.insert(low_limit);
        
        serializer.insert(high_limit);
        
        serializer.insert(low_limit_uncertainty);
        
        serializer.insert(high_limit_uncertainty);
        
        serializer.insert(minimum_uncertainty);
        
    }
}
void AccelMagnitudeErrorAdaptiveMeasurement::extract(Serializer& serializer)
{
    serializer.extract(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.extract(adaptive_measurement);
        
        serializer.extract(frequency);
        
        serializer.extract(low_limit);
        
        serializer.extract(high_limit);
        
        serializer.extract(low_limit_uncertainty);
        
        serializer.extract(high_limit_uncertainty);
        
        serializer.extract(minimum_uncertainty);
        
    }
}

void AccelMagnitudeErrorAdaptiveMeasurement::Response::insert(Serializer& serializer) const
{
    serializer.insert(adaptive_measurement);
    
    serializer.insert(frequency);
    
    serializer.insert(low_limit);
    
    serializer.insert(high_limit);
    
    serializer.insert(low_limit_uncertainty);
    
    serializer.insert(high_limit_uncertainty);
    
    serializer.insert(minimum_uncertainty);
    
}
void AccelMagnitudeErrorAdaptiveMeasurement::Response::extract(Serializer& serializer)
{
    serializer.extract(adaptive_measurement);
    
    serializer.extract(frequency);
    
    serializer.extract(low_limit);
    
    serializer.extract(high_limit);
    
    serializer.extract(low_limit_uncertainty);
    
    serializer.extract(high_limit_uncertainty);
    
    serializer.extract(minimum_uncertainty);
    
}

TypedResult<AccelMagnitudeErrorAdaptiveMeasurement> writeAccelMagnitudeErrorAdaptiveMeasurement(C::mip_interface& device, FilterAdaptiveMeasurement adaptiveMeasurement, float frequency, float lowLimit, float highLimit, float lowLimitUncertainty, float highLimitUncertainty, float minimumUncertainty)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::WRITE);
    serializer.insert(adaptiveMeasurement);
    
    serializer.insert(frequency);
    
    serializer.insert(lowLimit);
    
    serializer.insert(highLimit);
    
    serializer.insert(lowLimitUncertainty);
    
    serializer.insert(highLimitUncertainty);
    
    serializer.insert(minimumUncertainty);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ACCEL_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<AccelMagnitudeErrorAdaptiveMeasurement> readAccelMagnitudeErrorAdaptiveMeasurement(C::mip_interface& device, FilterAdaptiveMeasurement* adaptiveMeasurementOut, float* frequencyOut, float* lowLimitOut, float* highLimitOut, float* lowLimitUncertaintyOut, float* highLimitUncertaintyOut, float* minimumUncertaintyOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<AccelMagnitudeErrorAdaptiveMeasurement> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_ACCEL_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, (uint8_t)serializer.usedLength(), REPLY_ACCEL_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(adaptiveMeasurementOut);
        deserializer.extract(*adaptiveMeasurementOut);
        
        assert(frequencyOut);
        deserializer.extract(*frequencyOut);
        
        assert(lowLimitOut);
        deserializer.extract(*lowLimitOut);
        
        assert(highLimitOut);
        deserializer.extract(*highLimitOut);
        
        assert(lowLimitUncertaintyOut);
        deserializer.extract(*lowLimitUncertaintyOut);
        
        assert(highLimitUncertaintyOut);
        deserializer.extract(*highLimitUncertaintyOut);
        
        assert(minimumUncertaintyOut);
        deserializer.extract(*minimumUncertaintyOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<AccelMagnitudeErrorAdaptiveMeasurement> saveAccelMagnitudeErrorAdaptiveMeasurement(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ACCEL_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<AccelMagnitudeErrorAdaptiveMeasurement> loadAccelMagnitudeErrorAdaptiveMeasurement(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ACCEL_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<AccelMagnitudeErrorAdaptiveMeasurement> defaultAccelMagnitudeErrorAdaptiveMeasurement(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ACCEL_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, (uint8_t)serializer.usedLength());
}
void MagMagnitudeErrorAdaptiveMeasurement::insert(Serializer& serializer) const
{
    serializer.insert(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.insert(adaptive_measurement);
        
        serializer.insert(frequency);
        
        serializer.insert(low_limit);
        
        serializer.insert(high_limit);
        
        serializer.insert(low_limit_uncertainty);
        
        serializer.insert(high_limit_uncertainty);
        
        serializer.insert(minimum_uncertainty);
        
    }
}
void MagMagnitudeErrorAdaptiveMeasurement::extract(Serializer& serializer)
{
    serializer.extract(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.extract(adaptive_measurement);
        
        serializer.extract(frequency);
        
        serializer.extract(low_limit);
        
        serializer.extract(high_limit);
        
        serializer.extract(low_limit_uncertainty);
        
        serializer.extract(high_limit_uncertainty);
        
        serializer.extract(minimum_uncertainty);
        
    }
}

void MagMagnitudeErrorAdaptiveMeasurement::Response::insert(Serializer& serializer) const
{
    serializer.insert(adaptive_measurement);
    
    serializer.insert(frequency);
    
    serializer.insert(low_limit);
    
    serializer.insert(high_limit);
    
    serializer.insert(low_limit_uncertainty);
    
    serializer.insert(high_limit_uncertainty);
    
    serializer.insert(minimum_uncertainty);
    
}
void MagMagnitudeErrorAdaptiveMeasurement::Response::extract(Serializer& serializer)
{
    serializer.extract(adaptive_measurement);
    
    serializer.extract(frequency);
    
    serializer.extract(low_limit);
    
    serializer.extract(high_limit);
    
    serializer.extract(low_limit_uncertainty);
    
    serializer.extract(high_limit_uncertainty);
    
    serializer.extract(minimum_uncertainty);
    
}

TypedResult<MagMagnitudeErrorAdaptiveMeasurement> writeMagMagnitudeErrorAdaptiveMeasurement(C::mip_interface& device, FilterAdaptiveMeasurement adaptiveMeasurement, float frequency, float lowLimit, float highLimit, float lowLimitUncertainty, float highLimitUncertainty, float minimumUncertainty)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::WRITE);
    serializer.insert(adaptiveMeasurement);
    
    serializer.insert(frequency);
    
    serializer.insert(lowLimit);
    
    serializer.insert(highLimit);
    
    serializer.insert(lowLimitUncertainty);
    
    serializer.insert(highLimitUncertainty);
    
    serializer.insert(minimumUncertainty);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_MAG_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<MagMagnitudeErrorAdaptiveMeasurement> readMagMagnitudeErrorAdaptiveMeasurement(C::mip_interface& device, FilterAdaptiveMeasurement* adaptiveMeasurementOut, float* frequencyOut, float* lowLimitOut, float* highLimitOut, float* lowLimitUncertaintyOut, float* highLimitUncertaintyOut, float* minimumUncertaintyOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<MagMagnitudeErrorAdaptiveMeasurement> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_MAG_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, (uint8_t)serializer.usedLength(), REPLY_MAG_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(adaptiveMeasurementOut);
        deserializer.extract(*adaptiveMeasurementOut);
        
        assert(frequencyOut);
        deserializer.extract(*frequencyOut);
        
        assert(lowLimitOut);
        deserializer.extract(*lowLimitOut);
        
        assert(highLimitOut);
        deserializer.extract(*highLimitOut);
        
        assert(lowLimitUncertaintyOut);
        deserializer.extract(*lowLimitUncertaintyOut);
        
        assert(highLimitUncertaintyOut);
        deserializer.extract(*highLimitUncertaintyOut);
        
        assert(minimumUncertaintyOut);
        deserializer.extract(*minimumUncertaintyOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<MagMagnitudeErrorAdaptiveMeasurement> saveMagMagnitudeErrorAdaptiveMeasurement(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_MAG_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<MagMagnitudeErrorAdaptiveMeasurement> loadMagMagnitudeErrorAdaptiveMeasurement(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_MAG_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<MagMagnitudeErrorAdaptiveMeasurement> defaultMagMagnitudeErrorAdaptiveMeasurement(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_MAG_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, (uint8_t)serializer.usedLength());
}
void MagDipAngleErrorAdaptiveMeasurement::insert(Serializer& serializer) const
{
    serializer.insert(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.insert(enable);
        
        serializer.insert(frequency);
        
        serializer.insert(high_limit);
        
        serializer.insert(high_limit_uncertainty);
        
        serializer.insert(minimum_uncertainty);
        
    }
}
void MagDipAngleErrorAdaptiveMeasurement::extract(Serializer& serializer)
{
    serializer.extract(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.extract(enable);
        
        serializer.extract(frequency);
        
        serializer.extract(high_limit);
        
        serializer.extract(high_limit_uncertainty);
        
        serializer.extract(minimum_uncertainty);
        
    }
}

void MagDipAngleErrorAdaptiveMeasurement::Response::insert(Serializer& serializer) const
{
    serializer.insert(enable);
    
    serializer.insert(frequency);
    
    serializer.insert(high_limit);
    
    serializer.insert(high_limit_uncertainty);
    
    serializer.insert(minimum_uncertainty);
    
}
void MagDipAngleErrorAdaptiveMeasurement::Response::extract(Serializer& serializer)
{
    serializer.extract(enable);
    
    serializer.extract(frequency);
    
    serializer.extract(high_limit);
    
    serializer.extract(high_limit_uncertainty);
    
    serializer.extract(minimum_uncertainty);
    
}

TypedResult<MagDipAngleErrorAdaptiveMeasurement> writeMagDipAngleErrorAdaptiveMeasurement(C::mip_interface& device, bool enable, float frequency, float highLimit, float highLimitUncertainty, float minimumUncertainty)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::WRITE);
    serializer.insert(enable);
    
    serializer.insert(frequency);
    
    serializer.insert(highLimit);
    
    serializer.insert(highLimitUncertainty);
    
    serializer.insert(minimumUncertainty);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_MAG_DIP_ANGLE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<MagDipAngleErrorAdaptiveMeasurement> readMagDipAngleErrorAdaptiveMeasurement(C::mip_interface& device, bool* enableOut, float* frequencyOut, float* highLimitOut, float* highLimitUncertaintyOut, float* minimumUncertaintyOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<MagDipAngleErrorAdaptiveMeasurement> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_MAG_DIP_ANGLE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, (uint8_t)serializer.usedLength(), REPLY_MAG_DIP_ANGLE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(enableOut);
        deserializer.extract(*enableOut);
        
        assert(frequencyOut);
        deserializer.extract(*frequencyOut);
        
        assert(highLimitOut);
        deserializer.extract(*highLimitOut);
        
        assert(highLimitUncertaintyOut);
        deserializer.extract(*highLimitUncertaintyOut);
        
        assert(minimumUncertaintyOut);
        deserializer.extract(*minimumUncertaintyOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<MagDipAngleErrorAdaptiveMeasurement> saveMagDipAngleErrorAdaptiveMeasurement(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_MAG_DIP_ANGLE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<MagDipAngleErrorAdaptiveMeasurement> loadMagDipAngleErrorAdaptiveMeasurement(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_MAG_DIP_ANGLE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<MagDipAngleErrorAdaptiveMeasurement> defaultMagDipAngleErrorAdaptiveMeasurement(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_MAG_DIP_ANGLE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, (uint8_t)serializer.usedLength());
}
void AidingMeasurementEnable::insert(Serializer& serializer) const
{
    serializer.insert(function);
    
    serializer.insert(aiding_source);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.insert(enable);
        
    }
}
void AidingMeasurementEnable::extract(Serializer& serializer)
{
    serializer.extract(function);
    
    serializer.extract(aiding_source);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.extract(enable);
        
    }
}

void AidingMeasurementEnable::Response::insert(Serializer& serializer) const
{
    serializer.insert(aiding_source);
    
    serializer.insert(enable);
    
}
void AidingMeasurementEnable::Response::extract(Serializer& serializer)
{
    serializer.extract(aiding_source);
    
    serializer.extract(enable);
    
}

TypedResult<AidingMeasurementEnable> writeAidingMeasurementEnable(C::mip_interface& device, AidingMeasurementEnable::AidingSource aidingSource, bool enable)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::WRITE);
    serializer.insert(aidingSource);
    
    serializer.insert(enable);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_AIDING_MEASUREMENT_ENABLE, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<AidingMeasurementEnable> readAidingMeasurementEnable(C::mip_interface& device, AidingMeasurementEnable::AidingSource aidingSource, bool* enableOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::READ);
    serializer.insert(aidingSource);
    
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<AidingMeasurementEnable> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_AIDING_MEASUREMENT_ENABLE, buffer, (uint8_t)serializer.usedLength(), REPLY_AIDING_MEASUREMENT_ENABLE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        deserializer.extract(aidingSource);
        
        assert(enableOut);
        deserializer.extract(*enableOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<AidingMeasurementEnable> saveAidingMeasurementEnable(C::mip_interface& device, AidingMeasurementEnable::AidingSource aidingSource)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::SAVE);
    serializer.insert(aidingSource);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_AIDING_MEASUREMENT_ENABLE, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<AidingMeasurementEnable> loadAidingMeasurementEnable(C::mip_interface& device, AidingMeasurementEnable::AidingSource aidingSource)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::LOAD);
    serializer.insert(aidingSource);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_AIDING_MEASUREMENT_ENABLE, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<AidingMeasurementEnable> defaultAidingMeasurementEnable(C::mip_interface& device, AidingMeasurementEnable::AidingSource aidingSource)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::RESET);
    serializer.insert(aidingSource);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_AIDING_MEASUREMENT_ENABLE, buffer, (uint8_t)serializer.usedLength());
}
void Run::insert(Serializer& serializer) const
{
    (void)serializer;
}
void Run::extract(Serializer& serializer)
{
    (void)serializer;
}

TypedResult<Run> run(C::mip_interface& device)
{
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_RUN, NULL, 0);
}
void KinematicConstraint::insert(Serializer& serializer) const
{
    serializer.insert(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.insert(acceleration_constraint_selection);
        
        serializer.insert(velocity_constraint_selection);
        
        serializer.insert(angular_constraint_selection);
        
    }
}
void KinematicConstraint::extract(Serializer& serializer)
{
    serializer.extract(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.extract(acceleration_constraint_selection);
        
        serializer.extract(velocity_constraint_selection);
        
        serializer.extract(angular_constraint_selection);
        
    }
}

void KinematicConstraint::Response::insert(Serializer& serializer) const
{
    serializer.insert(acceleration_constraint_selection);
    
    serializer.insert(velocity_constraint_selection);
    
    serializer.insert(angular_constraint_selection);
    
}
void KinematicConstraint::Response::extract(Serializer& serializer)
{
    serializer.extract(acceleration_constraint_selection);
    
    serializer.extract(velocity_constraint_selection);
    
    serializer.extract(angular_constraint_selection);
    
}

TypedResult<KinematicConstraint> writeKinematicConstraint(C::mip_interface& device, uint8_t accelerationConstraintSelection, uint8_t velocityConstraintSelection, uint8_t angularConstraintSelection)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::WRITE);
    serializer.insert(accelerationConstraintSelection);
    
    serializer.insert(velocityConstraintSelection);
    
    serializer.insert(angularConstraintSelection);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_KINEMATIC_CONSTRAINT, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<KinematicConstraint> readKinematicConstraint(C::mip_interface& device, uint8_t* accelerationConstraintSelectionOut, uint8_t* velocityConstraintSelectionOut, uint8_t* angularConstraintSelectionOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<KinematicConstraint> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_KINEMATIC_CONSTRAINT, buffer, (uint8_t)serializer.usedLength(), REPLY_KINEMATIC_CONSTRAINT, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(accelerationConstraintSelectionOut);
        deserializer.extract(*accelerationConstraintSelectionOut);
        
        assert(velocityConstraintSelectionOut);
        deserializer.extract(*velocityConstraintSelectionOut);
        
        assert(angularConstraintSelectionOut);
        deserializer.extract(*angularConstraintSelectionOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<KinematicConstraint> saveKinematicConstraint(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_KINEMATIC_CONSTRAINT, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<KinematicConstraint> loadKinematicConstraint(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_KINEMATIC_CONSTRAINT, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<KinematicConstraint> defaultKinematicConstraint(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_KINEMATIC_CONSTRAINT, buffer, (uint8_t)serializer.usedLength());
}
void InitializationConfiguration::insert(Serializer& serializer) const
{
    serializer.insert(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.insert(wait_for_run_command);
        
        serializer.insert(initial_cond_src);
        
        serializer.insert(auto_heading_alignment_selector);
        
        serializer.insert(initial_heading);
        
        serializer.insert(initial_pitch);
        
        serializer.insert(initial_roll);
        
        serializer.insert(initial_position);
        
        serializer.insert(initial_velocity);
        
        serializer.insert(reference_frame_selector);
        
    }
}
void InitializationConfiguration::extract(Serializer& serializer)
{
    serializer.extract(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.extract(wait_for_run_command);
        
        serializer.extract(initial_cond_src);
        
        serializer.extract(auto_heading_alignment_selector);
        
        serializer.extract(initial_heading);
        
        serializer.extract(initial_pitch);
        
        serializer.extract(initial_roll);
        
        serializer.extract(initial_position);
        
        serializer.extract(initial_velocity);
        
        serializer.extract(reference_frame_selector);
        
    }
}

void InitializationConfiguration::Response::insert(Serializer& serializer) const
{
    serializer.insert(wait_for_run_command);
    
    serializer.insert(initial_cond_src);
    
    serializer.insert(auto_heading_alignment_selector);
    
    serializer.insert(initial_heading);
    
    serializer.insert(initial_pitch);
    
    serializer.insert(initial_roll);
    
    serializer.insert(initial_position);
    
    serializer.insert(initial_velocity);
    
    serializer.insert(reference_frame_selector);
    
}
void InitializationConfiguration::Response::extract(Serializer& serializer)
{
    serializer.extract(wait_for_run_command);
    
    serializer.extract(initial_cond_src);
    
    serializer.extract(auto_heading_alignment_selector);
    
    serializer.extract(initial_heading);
    
    serializer.extract(initial_pitch);
    
    serializer.extract(initial_roll);
    
    serializer.extract(initial_position);
    
    serializer.extract(initial_velocity);
    
    serializer.extract(reference_frame_selector);
    
}

TypedResult<InitializationConfiguration> writeInitializationConfiguration(C::mip_interface& device, uint8_t waitForRunCommand, InitializationConfiguration::InitialConditionSource initialCondSrc, InitializationConfiguration::AlignmentSelector autoHeadingAlignmentSelector, float initialHeading, float initialPitch, float initialRoll, const float* initialPosition, const float* initialVelocity, FilterReferenceFrame referenceFrameSelector)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::WRITE);
    serializer.insert(waitForRunCommand);
    
    serializer.insert(initialCondSrc);
    
    serializer.insert(autoHeadingAlignmentSelector);
    
    serializer.insert(initialHeading);
    
    serializer.insert(initialPitch);
    
    serializer.insert(initialRoll);
    
    assert(initialPosition);
    for(unsigned int i=0; i < 3; i++)
        serializer.insert(initialPosition[i]);
    
    assert(initialVelocity);
    for(unsigned int i=0; i < 3; i++)
        serializer.insert(initialVelocity[i]);
    
    serializer.insert(referenceFrameSelector);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_INITIALIZATION_CONFIGURATION, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<InitializationConfiguration> readInitializationConfiguration(C::mip_interface& device, uint8_t* waitForRunCommandOut, InitializationConfiguration::InitialConditionSource* initialCondSrcOut, InitializationConfiguration::AlignmentSelector* autoHeadingAlignmentSelectorOut, float* initialHeadingOut, float* initialPitchOut, float* initialRollOut, float* initialPositionOut, float* initialVelocityOut, FilterReferenceFrame* referenceFrameSelectorOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<InitializationConfiguration> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_INITIALIZATION_CONFIGURATION, buffer, (uint8_t)serializer.usedLength(), REPLY_INITIALIZATION_CONFIGURATION, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(waitForRunCommandOut);
        deserializer.extract(*waitForRunCommandOut);
        
        assert(initialCondSrcOut);
        deserializer.extract(*initialCondSrcOut);
        
        assert(autoHeadingAlignmentSelectorOut);
        deserializer.extract(*autoHeadingAlignmentSelectorOut);
        
        assert(initialHeadingOut);
        deserializer.extract(*initialHeadingOut);
        
        assert(initialPitchOut);
        deserializer.extract(*initialPitchOut);
        
        assert(initialRollOut);
        deserializer.extract(*initialRollOut);
        
        assert(initialPositionOut);
        for(unsigned int i=0; i < 3; i++)
            deserializer.extract(initialPositionOut[i]);
        
        assert(initialVelocityOut);
        for(unsigned int i=0; i < 3; i++)
            deserializer.extract(initialVelocityOut[i]);
        
        assert(referenceFrameSelectorOut);
        deserializer.extract(*referenceFrameSelectorOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<InitializationConfiguration> saveInitializationConfiguration(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_INITIALIZATION_CONFIGURATION, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<InitializationConfiguration> loadInitializationConfiguration(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_INITIALIZATION_CONFIGURATION, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<InitializationConfiguration> defaultInitializationConfiguration(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_INITIALIZATION_CONFIGURATION, buffer, (uint8_t)serializer.usedLength());
}
void AdaptiveFilterOptions::insert(Serializer& serializer) const
{
    serializer.insert(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.insert(level);
        
        serializer.insert(time_limit);
        
    }
}
void AdaptiveFilterOptions::extract(Serializer& serializer)
{
    serializer.extract(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.extract(level);
        
        serializer.extract(time_limit);
        
    }
}

void AdaptiveFilterOptions::Response::insert(Serializer& serializer) const
{
    serializer.insert(level);
    
    serializer.insert(time_limit);
    
}
void AdaptiveFilterOptions::Response::extract(Serializer& serializer)
{
    serializer.extract(level);
    
    serializer.extract(time_limit);
    
}

TypedResult<AdaptiveFilterOptions> writeAdaptiveFilterOptions(C::mip_interface& device, uint8_t level, uint16_t timeLimit)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::WRITE);
    serializer.insert(level);
    
    serializer.insert(timeLimit);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ADAPTIVE_FILTER_OPTIONS, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<AdaptiveFilterOptions> readAdaptiveFilterOptions(C::mip_interface& device, uint8_t* levelOut, uint16_t* timeLimitOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<AdaptiveFilterOptions> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_ADAPTIVE_FILTER_OPTIONS, buffer, (uint8_t)serializer.usedLength(), REPLY_ADAPTIVE_FILTER_OPTIONS, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(levelOut);
        deserializer.extract(*levelOut);
        
        assert(timeLimitOut);
        deserializer.extract(*timeLimitOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<AdaptiveFilterOptions> saveAdaptiveFilterOptions(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ADAPTIVE_FILTER_OPTIONS, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<AdaptiveFilterOptions> loadAdaptiveFilterOptions(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ADAPTIVE_FILTER_OPTIONS, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<AdaptiveFilterOptions> defaultAdaptiveFilterOptions(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ADAPTIVE_FILTER_OPTIONS, buffer, (uint8_t)serializer.usedLength());
}
void MultiAntennaOffset::insert(Serializer& serializer) const
{
    serializer.insert(function);
    
    serializer.insert(receiver_id);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.insert(antenna_offset);
        
    }
}
void MultiAntennaOffset::extract(Serializer& serializer)
{
    serializer.extract(function);
    
    serializer.extract(receiver_id);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.extract(antenna_offset);
        
    }
}

void MultiAntennaOffset::Response::insert(Serializer& serializer) const
{
    serializer.insert(receiver_id);
    
    serializer.insert(antenna_offset);
    
}
void MultiAntennaOffset::Response::extract(Serializer& serializer)
{
    serializer.extract(receiver_id);
    
    serializer.extract(antenna_offset);
    
}

TypedResult<MultiAntennaOffset> writeMultiAntennaOffset(C::mip_interface& device, uint8_t receiverId, const float* antennaOffset)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::WRITE);
    serializer.insert(receiverId);
    
    assert(antennaOffset);
    for(unsigned int i=0; i < 3; i++)
        serializer.insert(antennaOffset[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_MULTI_ANTENNA_OFFSET, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<MultiAntennaOffset> readMultiAntennaOffset(C::mip_interface& device, uint8_t receiverId, float* antennaOffsetOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::READ);
    serializer.insert(receiverId);
    
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<MultiAntennaOffset> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_MULTI_ANTENNA_OFFSET, buffer, (uint8_t)serializer.usedLength(), REPLY_MULTI_ANTENNA_OFFSET, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        deserializer.extract(receiverId);
        
        assert(antennaOffsetOut);
        for(unsigned int i=0; i < 3; i++)
            deserializer.extract(antennaOffsetOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<MultiAntennaOffset> saveMultiAntennaOffset(C::mip_interface& device, uint8_t receiverId)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::SAVE);
    serializer.insert(receiverId);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_MULTI_ANTENNA_OFFSET, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<MultiAntennaOffset> loadMultiAntennaOffset(C::mip_interface& device, uint8_t receiverId)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::LOAD);
    serializer.insert(receiverId);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_MULTI_ANTENNA_OFFSET, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<MultiAntennaOffset> defaultMultiAntennaOffset(C::mip_interface& device, uint8_t receiverId)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::RESET);
    serializer.insert(receiverId);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_MULTI_ANTENNA_OFFSET, buffer, (uint8_t)serializer.usedLength());
}
void RelPosConfiguration::insert(Serializer& serializer) const
{
    serializer.insert(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.insert(source);
        
        serializer.insert(reference_frame_selector);
        
        serializer.insert(reference_coordinates);
        
    }
}
void RelPosConfiguration::extract(Serializer& serializer)
{
    serializer.extract(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.extract(source);
        
        serializer.extract(reference_frame_selector);
        
        serializer.extract(reference_coordinates);
        
    }
}

void RelPosConfiguration::Response::insert(Serializer& serializer) const
{
    serializer.insert(source);
    
    serializer.insert(reference_frame_selector);
    
    serializer.insert(reference_coordinates);
    
}
void RelPosConfiguration::Response::extract(Serializer& serializer)
{
    serializer.extract(source);
    
    serializer.extract(reference_frame_selector);
    
    serializer.extract(reference_coordinates);
    
}

TypedResult<RelPosConfiguration> writeRelPosConfiguration(C::mip_interface& device, uint8_t source, FilterReferenceFrame referenceFrameSelector, const double* referenceCoordinates)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::WRITE);
    serializer.insert(source);
    
    serializer.insert(referenceFrameSelector);
    
    assert(referenceCoordinates);
    for(unsigned int i=0; i < 3; i++)
        serializer.insert(referenceCoordinates[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_REL_POS_CONFIGURATION, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<RelPosConfiguration> readRelPosConfiguration(C::mip_interface& device, uint8_t* sourceOut, FilterReferenceFrame* referenceFrameSelectorOut, double* referenceCoordinatesOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<RelPosConfiguration> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_REL_POS_CONFIGURATION, buffer, (uint8_t)serializer.usedLength(), REPLY_REL_POS_CONFIGURATION, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(sourceOut);
        deserializer.extract(*sourceOut);
        
        assert(referenceFrameSelectorOut);
        deserializer.extract(*referenceFrameSelectorOut);
        
        assert(referenceCoordinatesOut);
        for(unsigned int i=0; i < 3; i++)
            deserializer.extract(referenceCoordinatesOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<RelPosConfiguration> saveRelPosConfiguration(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_REL_POS_CONFIGURATION, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<RelPosConfiguration> loadRelPosConfiguration(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_REL_POS_CONFIGURATION, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<RelPosConfiguration> defaultRelPosConfiguration(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_REL_POS_CONFIGURATION, buffer, (uint8_t)serializer.usedLength());
}
void RefPointLeverArm::insert(Serializer& serializer) const
{
    serializer.insert(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.insert(ref_point_sel);
        
        serializer.insert(lever_arm_offset);
        
    }
}
void RefPointLeverArm::extract(Serializer& serializer)
{
    serializer.extract(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.extract(ref_point_sel);
        
        serializer.extract(lever_arm_offset);
        
    }
}

void RefPointLeverArm::Response::insert(Serializer& serializer) const
{
    serializer.insert(ref_point_sel);
    
    serializer.insert(lever_arm_offset);
    
}
void RefPointLeverArm::Response::extract(Serializer& serializer)
{
    serializer.extract(ref_point_sel);
    
    serializer.extract(lever_arm_offset);
    
}

TypedResult<RefPointLeverArm> writeRefPointLeverArm(C::mip_interface& device, RefPointLeverArm::ReferencePointSelector refPointSel, const float* leverArmOffset)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::WRITE);
    serializer.insert(refPointSel);
    
    assert(leverArmOffset);
    for(unsigned int i=0; i < 3; i++)
        serializer.insert(leverArmOffset[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_REF_POINT_LEVER_ARM, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<RefPointLeverArm> readRefPointLeverArm(C::mip_interface& device, RefPointLeverArm::ReferencePointSelector* refPointSelOut, float* leverArmOffsetOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<RefPointLeverArm> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_REF_POINT_LEVER_ARM, buffer, (uint8_t)serializer.usedLength(), REPLY_REF_POINT_LEVER_ARM, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(refPointSelOut);
        deserializer.extract(*refPointSelOut);
        
        assert(leverArmOffsetOut);
        for(unsigned int i=0; i < 3; i++)
            deserializer.extract(leverArmOffsetOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<RefPointLeverArm> saveRefPointLeverArm(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_REF_POINT_LEVER_ARM, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<RefPointLeverArm> loadRefPointLeverArm(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_REF_POINT_LEVER_ARM, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<RefPointLeverArm> defaultRefPointLeverArm(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_REF_POINT_LEVER_ARM, buffer, (uint8_t)serializer.usedLength());
}
void SpeedMeasurement::insert(Serializer& serializer) const
{
    serializer.insert(source);
    
    serializer.insert(time_of_week);
    
    serializer.insert(speed);
    
    serializer.insert(speed_uncertainty);
    
}
void SpeedMeasurement::extract(Serializer& serializer)
{
    serializer.extract(source);
    
    serializer.extract(time_of_week);
    
    serializer.extract(speed);
    
    serializer.extract(speed_uncertainty);
    
}

TypedResult<SpeedMeasurement> speedMeasurement(C::mip_interface& device, uint8_t source, float timeOfWeek, float speed, float speedUncertainty)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(source);
    
    serializer.insert(timeOfWeek);
    
    serializer.insert(speed);
    
    serializer.insert(speedUncertainty);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SPEED_MEASUREMENT, buffer, (uint8_t)serializer.usedLength());
}
void SpeedLeverArm::insert(Serializer& serializer) const
{
    serializer.insert(function);
    
    serializer.insert(source);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.insert(lever_arm_offset);
        
    }
}
void SpeedLeverArm::extract(Serializer& serializer)
{
    serializer.extract(function);
    
    serializer.extract(source);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.extract(lever_arm_offset);
        
    }
}

void SpeedLeverArm::Response::insert(Serializer& serializer) const
{
    serializer.insert(source);
    
    serializer.insert(lever_arm_offset);
    
}
void SpeedLeverArm::Response::extract(Serializer& serializer)
{
    serializer.extract(source);
    
    serializer.extract(lever_arm_offset);
    
}

TypedResult<SpeedLeverArm> writeSpeedLeverArm(C::mip_interface& device, uint8_t source, const float* leverArmOffset)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::WRITE);
    serializer.insert(source);
    
    assert(leverArmOffset);
    for(unsigned int i=0; i < 3; i++)
        serializer.insert(leverArmOffset[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SPEED_LEVER_ARM, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<SpeedLeverArm> readSpeedLeverArm(C::mip_interface& device, uint8_t source, float* leverArmOffsetOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::READ);
    serializer.insert(source);
    
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<SpeedLeverArm> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_SPEED_LEVER_ARM, buffer, (uint8_t)serializer.usedLength(), REPLY_SPEED_LEVER_ARM, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        deserializer.extract(source);
        
        assert(leverArmOffsetOut);
        for(unsigned int i=0; i < 3; i++)
            deserializer.extract(leverArmOffsetOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<SpeedLeverArm> saveSpeedLeverArm(C::mip_interface& device, uint8_t source)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::SAVE);
    serializer.insert(source);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SPEED_LEVER_ARM, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<SpeedLeverArm> loadSpeedLeverArm(C::mip_interface& device, uint8_t source)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::LOAD);
    serializer.insert(source);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SPEED_LEVER_ARM, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<SpeedLeverArm> defaultSpeedLeverArm(C::mip_interface& device, uint8_t source)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::RESET);
    serializer.insert(source);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SPEED_LEVER_ARM, buffer, (uint8_t)serializer.usedLength());
}
void WheeledVehicleConstraintControl::insert(Serializer& serializer) const
{
    serializer.insert(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.insert(enable);
        
    }
}
void WheeledVehicleConstraintControl::extract(Serializer& serializer)
{
    serializer.extract(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.extract(enable);
        
    }
}

void WheeledVehicleConstraintControl::Response::insert(Serializer& serializer) const
{
    serializer.insert(enable);
    
}
void WheeledVehicleConstraintControl::Response::extract(Serializer& serializer)
{
    serializer.extract(enable);
    
}

TypedResult<WheeledVehicleConstraintControl> writeWheeledVehicleConstraintControl(C::mip_interface& device, uint8_t enable)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::WRITE);
    serializer.insert(enable);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_VEHICLE_CONSTRAINT_CONTROL, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<WheeledVehicleConstraintControl> readWheeledVehicleConstraintControl(C::mip_interface& device, uint8_t* enableOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<WheeledVehicleConstraintControl> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_VEHICLE_CONSTRAINT_CONTROL, buffer, (uint8_t)serializer.usedLength(), REPLY_VEHICLE_CONSTRAINT_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(enableOut);
        deserializer.extract(*enableOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<WheeledVehicleConstraintControl> saveWheeledVehicleConstraintControl(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_VEHICLE_CONSTRAINT_CONTROL, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<WheeledVehicleConstraintControl> loadWheeledVehicleConstraintControl(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_VEHICLE_CONSTRAINT_CONTROL, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<WheeledVehicleConstraintControl> defaultWheeledVehicleConstraintControl(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_VEHICLE_CONSTRAINT_CONTROL, buffer, (uint8_t)serializer.usedLength());
}
void VerticalGyroConstraintControl::insert(Serializer& serializer) const
{
    serializer.insert(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.insert(enable);
        
    }
}
void VerticalGyroConstraintControl::extract(Serializer& serializer)
{
    serializer.extract(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.extract(enable);
        
    }
}

void VerticalGyroConstraintControl::Response::insert(Serializer& serializer) const
{
    serializer.insert(enable);
    
}
void VerticalGyroConstraintControl::Response::extract(Serializer& serializer)
{
    serializer.extract(enable);
    
}

TypedResult<VerticalGyroConstraintControl> writeVerticalGyroConstraintControl(C::mip_interface& device, uint8_t enable)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::WRITE);
    serializer.insert(enable);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GYRO_CONSTRAINT_CONTROL, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<VerticalGyroConstraintControl> readVerticalGyroConstraintControl(C::mip_interface& device, uint8_t* enableOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<VerticalGyroConstraintControl> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GYRO_CONSTRAINT_CONTROL, buffer, (uint8_t)serializer.usedLength(), REPLY_GYRO_CONSTRAINT_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(enableOut);
        deserializer.extract(*enableOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<VerticalGyroConstraintControl> saveVerticalGyroConstraintControl(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GYRO_CONSTRAINT_CONTROL, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<VerticalGyroConstraintControl> loadVerticalGyroConstraintControl(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GYRO_CONSTRAINT_CONTROL, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<VerticalGyroConstraintControl> defaultVerticalGyroConstraintControl(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GYRO_CONSTRAINT_CONTROL, buffer, (uint8_t)serializer.usedLength());
}
void GnssAntennaCalControl::insert(Serializer& serializer) const
{
    serializer.insert(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.insert(enable);
        
        serializer.insert(max_offset);
        
    }
}
void GnssAntennaCalControl::extract(Serializer& serializer)
{
    serializer.extract(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.extract(enable);
        
        serializer.extract(max_offset);
        
    }
}

void GnssAntennaCalControl::Response::insert(Serializer& serializer) const
{
    serializer.insert(enable);
    
    serializer.insert(max_offset);
    
}
void GnssAntennaCalControl::Response::extract(Serializer& serializer)
{
    serializer.extract(enable);
    
    serializer.extract(max_offset);
    
}

TypedResult<GnssAntennaCalControl> writeGnssAntennaCalControl(C::mip_interface& device, uint8_t enable, float maxOffset)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::WRITE);
    serializer.insert(enable);
    
    serializer.insert(maxOffset);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ANTENNA_CALIBRATION_CONTROL, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<GnssAntennaCalControl> readGnssAntennaCalControl(C::mip_interface& device, uint8_t* enableOut, float* maxOffsetOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<GnssAntennaCalControl> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_ANTENNA_CALIBRATION_CONTROL, buffer, (uint8_t)serializer.usedLength(), REPLY_ANTENNA_CALIBRATION_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(enableOut);
        deserializer.extract(*enableOut);
        
        assert(maxOffsetOut);
        deserializer.extract(*maxOffsetOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<GnssAntennaCalControl> saveGnssAntennaCalControl(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ANTENNA_CALIBRATION_CONTROL, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<GnssAntennaCalControl> loadGnssAntennaCalControl(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ANTENNA_CALIBRATION_CONTROL, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<GnssAntennaCalControl> defaultGnssAntennaCalControl(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ANTENNA_CALIBRATION_CONTROL, buffer, (uint8_t)serializer.usedLength());
}
void SetInitialHeading::insert(Serializer& serializer) const
{
    serializer.insert(heading);
    
}
void SetInitialHeading::extract(Serializer& serializer)
{
    serializer.extract(heading);
    
}

TypedResult<SetInitialHeading> setInitialHeading(C::mip_interface& device, float heading)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(heading);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SET_INITIAL_HEADING, buffer, (uint8_t)serializer.usedLength());
}

} // namespace commands_filter
} // namespace mip

