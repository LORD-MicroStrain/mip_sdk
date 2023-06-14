
#include "commands_aiding.h"

#include "../utils/serialization.h"
#include "../mip_interface.h"

#include <assert.h>


#ifdef __cplusplus
namespace mip {
namespace C {
extern "C" {

#endif // __cplusplus
struct mip_interface;
struct mip_serializer;
struct mip_field;


////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////

void insert_mip_time(mip_serializer* serializer, const mip_time* self)
{
    insert_mip_time_timebase(serializer, self->timebase);
    
    insert_u8(serializer, self->reserved);
    
    insert_u64(serializer, self->nanoseconds);
    
}
void extract_mip_time(mip_serializer* serializer, mip_time* self)
{
    extract_mip_time_timebase(serializer, &self->timebase);
    
    extract_u8(serializer, &self->reserved);
    
    extract_u64(serializer, &self->nanoseconds);
    
}

void insert_mip_time_timebase(struct mip_serializer* serializer, const mip_time_timebase self)
{
    insert_u8(serializer, (uint8_t)(self));
}
void extract_mip_time_timebase(struct mip_serializer* serializer, mip_time_timebase* self)
{
    uint8_t tmp = 0;
    extract_u8(serializer, &tmp);
    *self = tmp;
}


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

void insert_mip_aiding_ecef_pos_command(mip_serializer* serializer, const mip_aiding_ecef_pos_command* self)
{
    insert_mip_time(serializer, &self->time);
    
    insert_u8(serializer, self->sensor_id);
    
    for(unsigned int i=0; i < 3; i++)
        insert_double(serializer, self->position[i]);
    
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->uncertainty[i]);
    
    insert_mip_aiding_ecef_pos_command_valid_flags(serializer, self->valid_flags);
    
}
void extract_mip_aiding_ecef_pos_command(mip_serializer* serializer, mip_aiding_ecef_pos_command* self)
{
    extract_mip_time(serializer, &self->time);
    
    extract_u8(serializer, &self->sensor_id);
    
    for(unsigned int i=0; i < 3; i++)
        extract_double(serializer, &self->position[i]);
    
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->uncertainty[i]);
    
    extract_mip_aiding_ecef_pos_command_valid_flags(serializer, &self->valid_flags);
    
}

void insert_mip_aiding_ecef_pos_response(mip_serializer* serializer, const mip_aiding_ecef_pos_response* self)
{
    insert_mip_time(serializer, &self->time);
    
    insert_u8(serializer, self->sensor_id);
    
    for(unsigned int i=0; i < 3; i++)
        insert_double(serializer, self->position[i]);
    
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->uncertainty[i]);
    
    insert_mip_aiding_ecef_pos_command_valid_flags(serializer, self->valid_flags);
    
}
void extract_mip_aiding_ecef_pos_response(mip_serializer* serializer, mip_aiding_ecef_pos_response* self)
{
    extract_mip_time(serializer, &self->time);
    
    extract_u8(serializer, &self->sensor_id);
    
    for(unsigned int i=0; i < 3; i++)
        extract_double(serializer, &self->position[i]);
    
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->uncertainty[i]);
    
    extract_mip_aiding_ecef_pos_command_valid_flags(serializer, &self->valid_flags);
    
}

void insert_mip_aiding_ecef_pos_command_valid_flags(struct mip_serializer* serializer, const mip_aiding_ecef_pos_command_valid_flags self)
{
    insert_u16(serializer, (uint16_t)(self));
}
void extract_mip_aiding_ecef_pos_command_valid_flags(struct mip_serializer* serializer, mip_aiding_ecef_pos_command_valid_flags* self)
{
    uint16_t tmp = 0;
    extract_u16(serializer, &tmp);
    *self = tmp;
}

mip_cmd_result mip_aiding_ecef_pos(struct mip_interface* device, const mip_time* time, uint8_t sensor_id, const double* position, const float* uncertainty, mip_aiding_ecef_pos_command_valid_flags valid_flags, mip_time* time_out, uint8_t* sensor_id_out, double* position_out, float* uncertainty_out, mip_aiding_ecef_pos_command_valid_flags* valid_flags_out)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    assert(time);
    insert_mip_time(&serializer, time);
    
    insert_u8(&serializer, sensor_id);
    
    assert(position || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert_double(&serializer, position[i]);
    
    assert(uncertainty || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert_float(&serializer, uncertainty[i]);
    
    insert_mip_aiding_ecef_pos_command_valid_flags(&serializer, valid_flags);
    
    assert(mip_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_AIDING_CMD_DESC_SET, MIP_CMD_DESC_AIDING_ECEF_POS, buffer, (uint8_t)mip_serializer_length(&serializer), MIP_REPLY_DESC_AIDING_ECEF_POS, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(time_out);
        extract_mip_time(&deserializer, time_out);
        
        assert(sensor_id_out);
        extract_u8(&deserializer, sensor_id_out);
        
        assert(position_out || (3 == 0));
        for(unsigned int i=0; i < 3; i++)
            extract_double(&deserializer, &position_out[i]);
        
        assert(uncertainty_out || (3 == 0));
        for(unsigned int i=0; i < 3; i++)
            extract_float(&deserializer, &uncertainty_out[i]);
        
        assert(valid_flags_out);
        extract_mip_aiding_ecef_pos_command_valid_flags(&deserializer, valid_flags_out);
        
        if( mip_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
void insert_mip_aiding_llh_pos_command(mip_serializer* serializer, const mip_aiding_llh_pos_command* self)
{
    insert_mip_time(serializer, &self->time);
    
    insert_u8(serializer, self->sensor_id);
    
    insert_double(serializer, self->latitude);
    
    insert_double(serializer, self->longitude);
    
    insert_double(serializer, self->height);
    
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->uncertainty[i]);
    
    insert_mip_aiding_llh_pos_command_valid_flags(serializer, self->valid_flags);
    
}
void extract_mip_aiding_llh_pos_command(mip_serializer* serializer, mip_aiding_llh_pos_command* self)
{
    extract_mip_time(serializer, &self->time);
    
    extract_u8(serializer, &self->sensor_id);
    
    extract_double(serializer, &self->latitude);
    
    extract_double(serializer, &self->longitude);
    
    extract_double(serializer, &self->height);
    
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->uncertainty[i]);
    
    extract_mip_aiding_llh_pos_command_valid_flags(serializer, &self->valid_flags);
    
}

void insert_mip_aiding_llh_pos_response(mip_serializer* serializer, const mip_aiding_llh_pos_response* self)
{
    insert_mip_time(serializer, &self->time);
    
    insert_u8(serializer, self->sensor_id);
    
    insert_double(serializer, self->latitude);
    
    insert_double(serializer, self->longitude);
    
    insert_double(serializer, self->height);
    
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->uncertainty[i]);
    
    insert_mip_aiding_llh_pos_command_valid_flags(serializer, self->valid_flags);
    
}
void extract_mip_aiding_llh_pos_response(mip_serializer* serializer, mip_aiding_llh_pos_response* self)
{
    extract_mip_time(serializer, &self->time);
    
    extract_u8(serializer, &self->sensor_id);
    
    extract_double(serializer, &self->latitude);
    
    extract_double(serializer, &self->longitude);
    
    extract_double(serializer, &self->height);
    
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->uncertainty[i]);
    
    extract_mip_aiding_llh_pos_command_valid_flags(serializer, &self->valid_flags);
    
}

void insert_mip_aiding_llh_pos_command_valid_flags(struct mip_serializer* serializer, const mip_aiding_llh_pos_command_valid_flags self)
{
    insert_u16(serializer, (uint16_t)(self));
}
void extract_mip_aiding_llh_pos_command_valid_flags(struct mip_serializer* serializer, mip_aiding_llh_pos_command_valid_flags* self)
{
    uint16_t tmp = 0;
    extract_u16(serializer, &tmp);
    *self = tmp;
}

mip_cmd_result mip_aiding_llh_pos(struct mip_interface* device, const mip_time* time, uint8_t sensor_id, double latitude, double longitude, double height, const float* uncertainty, mip_aiding_llh_pos_command_valid_flags valid_flags, mip_time* time_out, uint8_t* sensor_id_out, double* latitude_out, double* longitude_out, double* height_out, float* uncertainty_out, mip_aiding_llh_pos_command_valid_flags* valid_flags_out)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    assert(time);
    insert_mip_time(&serializer, time);
    
    insert_u8(&serializer, sensor_id);
    
    insert_double(&serializer, latitude);
    
    insert_double(&serializer, longitude);
    
    insert_double(&serializer, height);
    
    assert(uncertainty || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert_float(&serializer, uncertainty[i]);
    
    insert_mip_aiding_llh_pos_command_valid_flags(&serializer, valid_flags);
    
    assert(mip_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_AIDING_CMD_DESC_SET, MIP_CMD_DESC_AIDING_LLH_POS, buffer, (uint8_t)mip_serializer_length(&serializer), MIP_REPLY_DESC_AIDING_LLH_POS, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(time_out);
        extract_mip_time(&deserializer, time_out);
        
        assert(sensor_id_out);
        extract_u8(&deserializer, sensor_id_out);
        
        assert(latitude_out);
        extract_double(&deserializer, latitude_out);
        
        assert(longitude_out);
        extract_double(&deserializer, longitude_out);
        
        assert(height_out);
        extract_double(&deserializer, height_out);
        
        assert(uncertainty_out || (3 == 0));
        for(unsigned int i=0; i < 3; i++)
            extract_float(&deserializer, &uncertainty_out[i]);
        
        assert(valid_flags_out);
        extract_mip_aiding_llh_pos_command_valid_flags(&deserializer, valid_flags_out);
        
        if( mip_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
void insert_mip_aiding_ecef_vel_command(mip_serializer* serializer, const mip_aiding_ecef_vel_command* self)
{
    insert_mip_time(serializer, &self->time);
    
    insert_u8(serializer, self->sensor_id);
    
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->velocity[i]);
    
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->uncertainty[i]);
    
    insert_mip_aiding_ecef_vel_command_valid_flags(serializer, self->valid_flags);
    
}
void extract_mip_aiding_ecef_vel_command(mip_serializer* serializer, mip_aiding_ecef_vel_command* self)
{
    extract_mip_time(serializer, &self->time);
    
    extract_u8(serializer, &self->sensor_id);
    
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->velocity[i]);
    
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->uncertainty[i]);
    
    extract_mip_aiding_ecef_vel_command_valid_flags(serializer, &self->valid_flags);
    
}

void insert_mip_aiding_ecef_vel_response(mip_serializer* serializer, const mip_aiding_ecef_vel_response* self)
{
    insert_mip_time(serializer, &self->time);
    
    insert_u8(serializer, self->sensor_id);
    
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->velocity[i]);
    
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->uncertainty[i]);
    
    insert_mip_aiding_ecef_vel_command_valid_flags(serializer, self->valid_flags);
    
}
void extract_mip_aiding_ecef_vel_response(mip_serializer* serializer, mip_aiding_ecef_vel_response* self)
{
    extract_mip_time(serializer, &self->time);
    
    extract_u8(serializer, &self->sensor_id);
    
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->velocity[i]);
    
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->uncertainty[i]);
    
    extract_mip_aiding_ecef_vel_command_valid_flags(serializer, &self->valid_flags);
    
}

void insert_mip_aiding_ecef_vel_command_valid_flags(struct mip_serializer* serializer, const mip_aiding_ecef_vel_command_valid_flags self)
{
    insert_u16(serializer, (uint16_t)(self));
}
void extract_mip_aiding_ecef_vel_command_valid_flags(struct mip_serializer* serializer, mip_aiding_ecef_vel_command_valid_flags* self)
{
    uint16_t tmp = 0;
    extract_u16(serializer, &tmp);
    *self = tmp;
}

mip_cmd_result mip_aiding_ecef_vel(struct mip_interface* device, const mip_time* time, uint8_t sensor_id, const float* velocity, const float* uncertainty, mip_aiding_ecef_vel_command_valid_flags valid_flags, mip_time* time_out, uint8_t* sensor_id_out, float* velocity_out, float* uncertainty_out, mip_aiding_ecef_vel_command_valid_flags* valid_flags_out)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    assert(time);
    insert_mip_time(&serializer, time);
    
    insert_u8(&serializer, sensor_id);
    
    assert(velocity || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert_float(&serializer, velocity[i]);
    
    assert(uncertainty || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert_float(&serializer, uncertainty[i]);
    
    insert_mip_aiding_ecef_vel_command_valid_flags(&serializer, valid_flags);
    
    assert(mip_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_AIDING_CMD_DESC_SET, MIP_CMD_DESC_AIDING_ECEF_VEL, buffer, (uint8_t)mip_serializer_length(&serializer), MIP_REPLY_DESC_AIDING_ECEF_VEL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(time_out);
        extract_mip_time(&deserializer, time_out);
        
        assert(sensor_id_out);
        extract_u8(&deserializer, sensor_id_out);
        
        assert(velocity_out || (3 == 0));
        for(unsigned int i=0; i < 3; i++)
            extract_float(&deserializer, &velocity_out[i]);
        
        assert(uncertainty_out || (3 == 0));
        for(unsigned int i=0; i < 3; i++)
            extract_float(&deserializer, &uncertainty_out[i]);
        
        assert(valid_flags_out);
        extract_mip_aiding_ecef_vel_command_valid_flags(&deserializer, valid_flags_out);
        
        if( mip_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
void insert_mip_aiding_ned_vel_command(mip_serializer* serializer, const mip_aiding_ned_vel_command* self)
{
    insert_mip_time(serializer, &self->time);
    
    insert_u8(serializer, self->sensor_id);
    
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->velocity[i]);
    
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->uncertainty[i]);
    
    insert_mip_aiding_ned_vel_command_valid_flags(serializer, self->valid_flags);
    
}
void extract_mip_aiding_ned_vel_command(mip_serializer* serializer, mip_aiding_ned_vel_command* self)
{
    extract_mip_time(serializer, &self->time);
    
    extract_u8(serializer, &self->sensor_id);
    
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->velocity[i]);
    
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->uncertainty[i]);
    
    extract_mip_aiding_ned_vel_command_valid_flags(serializer, &self->valid_flags);
    
}

void insert_mip_aiding_ned_vel_response(mip_serializer* serializer, const mip_aiding_ned_vel_response* self)
{
    insert_mip_time(serializer, &self->time);
    
    insert_u8(serializer, self->sensor_id);
    
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->velocity[i]);
    
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->uncertainty[i]);
    
    insert_mip_aiding_ned_vel_command_valid_flags(serializer, self->valid_flags);
    
}
void extract_mip_aiding_ned_vel_response(mip_serializer* serializer, mip_aiding_ned_vel_response* self)
{
    extract_mip_time(serializer, &self->time);
    
    extract_u8(serializer, &self->sensor_id);
    
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->velocity[i]);
    
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->uncertainty[i]);
    
    extract_mip_aiding_ned_vel_command_valid_flags(serializer, &self->valid_flags);
    
}

void insert_mip_aiding_ned_vel_command_valid_flags(struct mip_serializer* serializer, const mip_aiding_ned_vel_command_valid_flags self)
{
    insert_u16(serializer, (uint16_t)(self));
}
void extract_mip_aiding_ned_vel_command_valid_flags(struct mip_serializer* serializer, mip_aiding_ned_vel_command_valid_flags* self)
{
    uint16_t tmp = 0;
    extract_u16(serializer, &tmp);
    *self = tmp;
}

mip_cmd_result mip_aiding_ned_vel(struct mip_interface* device, const mip_time* time, uint8_t sensor_id, const float* velocity, const float* uncertainty, mip_aiding_ned_vel_command_valid_flags valid_flags, mip_time* time_out, uint8_t* sensor_id_out, float* velocity_out, float* uncertainty_out, mip_aiding_ned_vel_command_valid_flags* valid_flags_out)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    assert(time);
    insert_mip_time(&serializer, time);
    
    insert_u8(&serializer, sensor_id);
    
    assert(velocity || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert_float(&serializer, velocity[i]);
    
    assert(uncertainty || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert_float(&serializer, uncertainty[i]);
    
    insert_mip_aiding_ned_vel_command_valid_flags(&serializer, valid_flags);
    
    assert(mip_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_AIDING_CMD_DESC_SET, MIP_CMD_DESC_AIDING_NED_VEL, buffer, (uint8_t)mip_serializer_length(&serializer), MIP_REPLY_DESC_AIDING_NED_VEL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(time_out);
        extract_mip_time(&deserializer, time_out);
        
        assert(sensor_id_out);
        extract_u8(&deserializer, sensor_id_out);
        
        assert(velocity_out || (3 == 0));
        for(unsigned int i=0; i < 3; i++)
            extract_float(&deserializer, &velocity_out[i]);
        
        assert(uncertainty_out || (3 == 0));
        for(unsigned int i=0; i < 3; i++)
            extract_float(&deserializer, &uncertainty_out[i]);
        
        assert(valid_flags_out);
        extract_mip_aiding_ned_vel_command_valid_flags(&deserializer, valid_flags_out);
        
        if( mip_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
void insert_mip_aiding_vehicle_fixed_frame_velocity_command(mip_serializer* serializer, const mip_aiding_vehicle_fixed_frame_velocity_command* self)
{
    insert_mip_time(serializer, &self->time);
    
    insert_u8(serializer, self->sensor_id);
    
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->velocity[i]);
    
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->uncertainty[i]);
    
    insert_mip_aiding_vehicle_fixed_frame_velocity_command_valid_flags(serializer, self->valid_flags);
    
}
void extract_mip_aiding_vehicle_fixed_frame_velocity_command(mip_serializer* serializer, mip_aiding_vehicle_fixed_frame_velocity_command* self)
{
    extract_mip_time(serializer, &self->time);
    
    extract_u8(serializer, &self->sensor_id);
    
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->velocity[i]);
    
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->uncertainty[i]);
    
    extract_mip_aiding_vehicle_fixed_frame_velocity_command_valid_flags(serializer, &self->valid_flags);
    
}

void insert_mip_aiding_vehicle_fixed_frame_velocity_response(mip_serializer* serializer, const mip_aiding_vehicle_fixed_frame_velocity_response* self)
{
    insert_mip_time(serializer, &self->time);
    
    insert_u8(serializer, self->sensor_id);
    
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->velocity[i]);
    
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->uncertainty[i]);
    
    insert_mip_aiding_vehicle_fixed_frame_velocity_command_valid_flags(serializer, self->valid_flags);
    
}
void extract_mip_aiding_vehicle_fixed_frame_velocity_response(mip_serializer* serializer, mip_aiding_vehicle_fixed_frame_velocity_response* self)
{
    extract_mip_time(serializer, &self->time);
    
    extract_u8(serializer, &self->sensor_id);
    
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->velocity[i]);
    
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->uncertainty[i]);
    
    extract_mip_aiding_vehicle_fixed_frame_velocity_command_valid_flags(serializer, &self->valid_flags);
    
}

void insert_mip_aiding_vehicle_fixed_frame_velocity_command_valid_flags(struct mip_serializer* serializer, const mip_aiding_vehicle_fixed_frame_velocity_command_valid_flags self)
{
    insert_u16(serializer, (uint16_t)(self));
}
void extract_mip_aiding_vehicle_fixed_frame_velocity_command_valid_flags(struct mip_serializer* serializer, mip_aiding_vehicle_fixed_frame_velocity_command_valid_flags* self)
{
    uint16_t tmp = 0;
    extract_u16(serializer, &tmp);
    *self = tmp;
}

mip_cmd_result mip_aiding_vehicle_fixed_frame_velocity(struct mip_interface* device, const mip_time* time, uint8_t sensor_id, const float* velocity, const float* uncertainty, mip_aiding_vehicle_fixed_frame_velocity_command_valid_flags valid_flags, mip_time* time_out, uint8_t* sensor_id_out, float* velocity_out, float* uncertainty_out, mip_aiding_vehicle_fixed_frame_velocity_command_valid_flags* valid_flags_out)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    assert(time);
    insert_mip_time(&serializer, time);
    
    insert_u8(&serializer, sensor_id);
    
    assert(velocity || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert_float(&serializer, velocity[i]);
    
    assert(uncertainty || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert_float(&serializer, uncertainty[i]);
    
    insert_mip_aiding_vehicle_fixed_frame_velocity_command_valid_flags(&serializer, valid_flags);
    
    assert(mip_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_AIDING_CMD_DESC_SET, MIP_CMD_DESC_AIDING_ODOM_VEL, buffer, (uint8_t)mip_serializer_length(&serializer), MIP_REPLY_DESC_AIDING_ODOM_VEL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(time_out);
        extract_mip_time(&deserializer, time_out);
        
        assert(sensor_id_out);
        extract_u8(&deserializer, sensor_id_out);
        
        assert(velocity_out || (3 == 0));
        for(unsigned int i=0; i < 3; i++)
            extract_float(&deserializer, &velocity_out[i]);
        
        assert(uncertainty_out || (3 == 0));
        for(unsigned int i=0; i < 3; i++)
            extract_float(&deserializer, &uncertainty_out[i]);
        
        assert(valid_flags_out);
        extract_mip_aiding_vehicle_fixed_frame_velocity_command_valid_flags(&deserializer, valid_flags_out);
        
        if( mip_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
void insert_mip_aiding_true_heading_command(mip_serializer* serializer, const mip_aiding_true_heading_command* self)
{
    insert_mip_time(serializer, &self->time);
    
    insert_u8(serializer, self->sensor_id);
    
    insert_float(serializer, self->heading);
    
    insert_float(serializer, self->uncertainty);
    
    insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_aiding_true_heading_command(mip_serializer* serializer, mip_aiding_true_heading_command* self)
{
    extract_mip_time(serializer, &self->time);
    
    extract_u8(serializer, &self->sensor_id);
    
    extract_float(serializer, &self->heading);
    
    extract_float(serializer, &self->uncertainty);
    
    extract_u16(serializer, &self->valid_flags);
    
}

void insert_mip_aiding_true_heading_response(mip_serializer* serializer, const mip_aiding_true_heading_response* self)
{
    insert_mip_time(serializer, &self->time);
    
    insert_u8(serializer, self->sensor_id);
    
    insert_float(serializer, self->heading);
    
    insert_float(serializer, self->uncertainty);
    
    insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_aiding_true_heading_response(mip_serializer* serializer, mip_aiding_true_heading_response* self)
{
    extract_mip_time(serializer, &self->time);
    
    extract_u8(serializer, &self->sensor_id);
    
    extract_float(serializer, &self->heading);
    
    extract_float(serializer, &self->uncertainty);
    
    extract_u16(serializer, &self->valid_flags);
    
}

mip_cmd_result mip_aiding_true_heading(struct mip_interface* device, const mip_time* time, uint8_t sensor_id, float heading, float uncertainty, uint16_t valid_flags, mip_time* time_out, uint8_t* sensor_id_out, float* heading_out, float* uncertainty_out, uint16_t* valid_flags_out)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    assert(time);
    insert_mip_time(&serializer, time);
    
    insert_u8(&serializer, sensor_id);
    
    insert_float(&serializer, heading);
    
    insert_float(&serializer, uncertainty);
    
    insert_u16(&serializer, valid_flags);
    
    assert(mip_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_AIDING_CMD_DESC_SET, MIP_CMD_DESC_AIDING_HEADING_TRUE, buffer, (uint8_t)mip_serializer_length(&serializer), MIP_REPLY_DESC_AIDING_HEADING_TRUE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(time_out);
        extract_mip_time(&deserializer, time_out);
        
        assert(sensor_id_out);
        extract_u8(&deserializer, sensor_id_out);
        
        assert(heading_out);
        extract_float(&deserializer, heading_out);
        
        assert(uncertainty_out);
        extract_float(&deserializer, uncertainty_out);
        
        assert(valid_flags_out);
        extract_u16(&deserializer, valid_flags_out);
        
        if( mip_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
void insert_mip_aiding_aiding_echo_control_command(mip_serializer* serializer, const mip_aiding_aiding_echo_control_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        insert_mip_aiding_aiding_echo_control_command_mode(serializer, self->mode);
        
    }
}
void extract_mip_aiding_aiding_echo_control_command(mip_serializer* serializer, mip_aiding_aiding_echo_control_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        extract_mip_aiding_aiding_echo_control_command_mode(serializer, &self->mode);
        
    }
}

void insert_mip_aiding_aiding_echo_control_response(mip_serializer* serializer, const mip_aiding_aiding_echo_control_response* self)
{
    insert_mip_aiding_aiding_echo_control_command_mode(serializer, self->mode);
    
}
void extract_mip_aiding_aiding_echo_control_response(mip_serializer* serializer, mip_aiding_aiding_echo_control_response* self)
{
    extract_mip_aiding_aiding_echo_control_command_mode(serializer, &self->mode);
    
}

void insert_mip_aiding_aiding_echo_control_command_mode(struct mip_serializer* serializer, const mip_aiding_aiding_echo_control_command_mode self)
{
    insert_u8(serializer, (uint8_t)(self));
}
void extract_mip_aiding_aiding_echo_control_command_mode(struct mip_serializer* serializer, mip_aiding_aiding_echo_control_command_mode* self)
{
    uint8_t tmp = 0;
    extract_u8(serializer, &tmp);
    *self = tmp;
}

mip_cmd_result mip_aiding_write_aiding_echo_control(struct mip_interface* device, mip_aiding_aiding_echo_control_command_mode mode)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    insert_mip_aiding_aiding_echo_control_command_mode(&serializer, mode);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_AIDING_CMD_DESC_SET, MIP_CMD_DESC_AIDING_ECHO_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_aiding_read_aiding_echo_control(struct mip_interface* device, mip_aiding_aiding_echo_control_command_mode* mode_out)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(mip_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_AIDING_CMD_DESC_SET, MIP_CMD_DESC_AIDING_ECHO_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer), MIP_REPLY_DESC_AIDING_ECHO_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(mode_out);
        extract_mip_aiding_aiding_echo_control_command_mode(&deserializer, mode_out);
        
        if( mip_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_aiding_save_aiding_echo_control(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_AIDING_CMD_DESC_SET, MIP_CMD_DESC_AIDING_ECHO_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_aiding_load_aiding_echo_control(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_AIDING_CMD_DESC_SET, MIP_CMD_DESC_AIDING_ECHO_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_aiding_default_aiding_echo_control(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_AIDING_CMD_DESC_SET, MIP_CMD_DESC_AIDING_ECHO_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}

#ifdef __cplusplus
} // namespace C
} // namespace mip
} // extern "C"
#endif // __cplusplus

