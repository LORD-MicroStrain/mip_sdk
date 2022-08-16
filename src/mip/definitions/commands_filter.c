
#include "commands_filter.h"

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


////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////

void insert_mip_filter_reference_frame(struct mip_serializer* serializer, const enum mip_filter_reference_frame self)
{
    return insert_u8(serializer, (uint8_t)(self));
}
void extract_mip_filter_reference_frame(struct mip_serializer* serializer, enum mip_filter_reference_frame* self)
{
    uint8_t tmp = 0;
    extract_u8(serializer, &tmp);
    *self = tmp;
}

void insert_mip_filter_mag_declination_source(struct mip_serializer* serializer, const enum mip_filter_mag_declination_source self)
{
    return insert_u8(serializer, (uint8_t)(self));
}
void extract_mip_filter_mag_declination_source(struct mip_serializer* serializer, enum mip_filter_mag_declination_source* self)
{
    uint8_t tmp = 0;
    extract_u8(serializer, &tmp);
    *self = tmp;
}


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

enum mip_cmd_result mip_filter_reset(struct mip_interface* device)
{
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_RESET_FILTER, NULL, 0);
}
void insert_mip_filter_set_initial_attitude_command(struct mip_serializer* serializer, const struct mip_filter_set_initial_attitude_command* self)
{
    insert_float(serializer, self->roll);
    
    insert_float(serializer, self->pitch);
    
    insert_float(serializer, self->heading);
    
}
void extract_mip_filter_set_initial_attitude_command(struct mip_serializer* serializer, struct mip_filter_set_initial_attitude_command* self)
{
    extract_float(serializer, &self->roll);
    
    extract_float(serializer, &self->pitch);
    
    extract_float(serializer, &self->heading);
    
}

enum mip_cmd_result mip_filter_set_initial_attitude(struct mip_interface* device, float roll, float pitch, float heading)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_float(&serializer, roll);
    
    insert_float(&serializer, pitch);
    
    insert_float(&serializer, heading);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SET_INITIAL_ATTITUDE, buffer, serializer.offset);
}
void insert_mip_filter_estimation_control_command(struct mip_serializer* serializer, const struct mip_filter_estimation_control_command* self)
{
    insert_mip_filter_estimation_control_command_enable_flags(serializer, self->enable);
    
}
void extract_mip_filter_estimation_control_command(struct mip_serializer* serializer, struct mip_filter_estimation_control_command* self)
{
    extract_mip_filter_estimation_control_command_enable_flags(serializer, &self->enable);
    
}

void insert_mip_filter_estimation_control_command_enable_flags(struct mip_serializer* serializer, const enum mip_filter_estimation_control_command_enable_flags self)
{
    return insert_u16(serializer, (uint16_t)(self));
}
void extract_mip_filter_estimation_control_command_enable_flags(struct mip_serializer* serializer, enum mip_filter_estimation_control_command_enable_flags* self)
{
    uint16_t tmp = 0;
    extract_u16(serializer, &tmp);
    *self = tmp;
}

enum mip_cmd_result mip_filter_write_estimation_control(struct mip_interface* device, enum mip_filter_estimation_control_command_enable_flags enable)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    insert_mip_filter_estimation_control_command_enable_flags(&serializer, enable);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ESTIMATION_CONTROL_FLAGS, buffer, serializer.offset);
}
enum mip_cmd_result mip_filter_read_estimation_control(struct mip_interface* device, enum mip_filter_estimation_control_command_enable_flags* enable_out)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    enum mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ESTIMATION_CONTROL_FLAGS, buffer, serializer.offset, MIP_REPLY_DESC_FILTER_ESTIMATION_CONTROL_FLAGS, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        struct mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(enable_out);
        extract_mip_filter_estimation_control_command_enable_flags(&deserializer, enable_out);
        
        if( !mip_serializer_ok(&deserializer) )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
enum mip_cmd_result mip_filter_save_estimation_control(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ESTIMATION_CONTROL_FLAGS, buffer, serializer.offset);
}
enum mip_cmd_result mip_filter_load_estimation_control(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ESTIMATION_CONTROL_FLAGS, buffer, serializer.offset);
}
enum mip_cmd_result mip_filter_default_estimation_control(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ESTIMATION_CONTROL_FLAGS, buffer, serializer.offset);
}
void insert_mip_filter_external_gnss_update_command(struct mip_serializer* serializer, const struct mip_filter_external_gnss_update_command* self)
{
    insert_double(serializer, self->gps_time);
    
    insert_u16(serializer, self->gps_week);
    
    insert_double(serializer, self->latitude);
    
    insert_double(serializer, self->longitude);
    
    insert_double(serializer, self->height);
    
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->velocity[i]);
    
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->pos_uncertainty[i]);
    
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->vel_uncertainty[i]);
    
}
void extract_mip_filter_external_gnss_update_command(struct mip_serializer* serializer, struct mip_filter_external_gnss_update_command* self)
{
    extract_double(serializer, &self->gps_time);
    
    extract_u16(serializer, &self->gps_week);
    
    extract_double(serializer, &self->latitude);
    
    extract_double(serializer, &self->longitude);
    
    extract_double(serializer, &self->height);
    
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->velocity[i]);
    
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->pos_uncertainty[i]);
    
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->vel_uncertainty[i]);
    
}

enum mip_cmd_result mip_filter_external_gnss_update(struct mip_interface* device, double gps_time, uint16_t gps_week, double latitude, double longitude, double height, const float* velocity, const float* pos_uncertainty, const float* vel_uncertainty)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_double(&serializer, gps_time);
    
    insert_u16(&serializer, gps_week);
    
    insert_double(&serializer, latitude);
    
    insert_double(&serializer, longitude);
    
    insert_double(&serializer, height);
    
    assert(velocity);
    for(unsigned int i=0; i < 3; i++)
        insert_float(&serializer, velocity[i]);
    
    assert(pos_uncertainty);
    for(unsigned int i=0; i < 3; i++)
        insert_float(&serializer, pos_uncertainty[i]);
    
    assert(vel_uncertainty);
    for(unsigned int i=0; i < 3; i++)
        insert_float(&serializer, vel_uncertainty[i]);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_EXTERNAL_GNSS_UPDATE, buffer, serializer.offset);
}
void insert_mip_filter_external_heading_update_command(struct mip_serializer* serializer, const struct mip_filter_external_heading_update_command* self)
{
    insert_float(serializer, self->heading);
    
    insert_float(serializer, self->heading_uncertainty);
    
    insert_u8(serializer, self->type);
    
}
void extract_mip_filter_external_heading_update_command(struct mip_serializer* serializer, struct mip_filter_external_heading_update_command* self)
{
    extract_float(serializer, &self->heading);
    
    extract_float(serializer, &self->heading_uncertainty);
    
    extract_u8(serializer, &self->type);
    
}

enum mip_cmd_result mip_filter_external_heading_update(struct mip_interface* device, float heading, float heading_uncertainty, uint8_t type)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_float(&serializer, heading);
    
    insert_float(&serializer, heading_uncertainty);
    
    insert_u8(&serializer, type);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_EXTERNAL_HEADING_UPDATE, buffer, serializer.offset);
}
void insert_mip_filter_external_heading_update_with_time_command(struct mip_serializer* serializer, const struct mip_filter_external_heading_update_with_time_command* self)
{
    insert_double(serializer, self->gps_time);
    
    insert_u16(serializer, self->gps_week);
    
    insert_float(serializer, self->heading);
    
    insert_float(serializer, self->heading_uncertainty);
    
    insert_u8(serializer, self->type);
    
}
void extract_mip_filter_external_heading_update_with_time_command(struct mip_serializer* serializer, struct mip_filter_external_heading_update_with_time_command* self)
{
    extract_double(serializer, &self->gps_time);
    
    extract_u16(serializer, &self->gps_week);
    
    extract_float(serializer, &self->heading);
    
    extract_float(serializer, &self->heading_uncertainty);
    
    extract_u8(serializer, &self->type);
    
}

enum mip_cmd_result mip_filter_external_heading_update_with_time(struct mip_interface* device, double gps_time, uint16_t gps_week, float heading, float heading_uncertainty, uint8_t type)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_double(&serializer, gps_time);
    
    insert_u16(&serializer, gps_week);
    
    insert_float(&serializer, heading);
    
    insert_float(&serializer, heading_uncertainty);
    
    insert_u8(&serializer, type);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_EXTERNAL_HEADING_UPDATE_WITH_TIME, buffer, serializer.offset);
}
void insert_mip_filter_tare_orientation_command(struct mip_serializer* serializer, const struct mip_filter_tare_orientation_command* self)
{
    insert_mip_filter_tare_orientation_command_mip_tare_axes(serializer, self->axes);
    
}
void extract_mip_filter_tare_orientation_command(struct mip_serializer* serializer, struct mip_filter_tare_orientation_command* self)
{
    extract_mip_filter_tare_orientation_command_mip_tare_axes(serializer, &self->axes);
    
}

void insert_mip_filter_tare_orientation_command_mip_tare_axes(struct mip_serializer* serializer, const enum mip_filter_tare_orientation_command_mip_tare_axes self)
{
    return insert_u8(serializer, (uint8_t)(self));
}
void extract_mip_filter_tare_orientation_command_mip_tare_axes(struct mip_serializer* serializer, enum mip_filter_tare_orientation_command_mip_tare_axes* self)
{
    uint8_t tmp = 0;
    extract_u8(serializer, &tmp);
    *self = tmp;
}

enum mip_cmd_result mip_filter_write_tare_orientation(struct mip_interface* device, enum mip_filter_tare_orientation_command_mip_tare_axes axes)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    insert_mip_filter_tare_orientation_command_mip_tare_axes(&serializer, axes);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_TARE_ORIENTATION, buffer, serializer.offset);
}
enum mip_cmd_result mip_filter_read_tare_orientation(struct mip_interface* device, enum mip_filter_tare_orientation_command_mip_tare_axes* axes_out)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    enum mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_TARE_ORIENTATION, buffer, serializer.offset, MIP_REPLY_DESC_FILTER_TARE_ORIENTATION, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        struct mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(axes_out);
        extract_mip_filter_tare_orientation_command_mip_tare_axes(&deserializer, axes_out);
        
        if( !mip_serializer_ok(&deserializer) )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
enum mip_cmd_result mip_filter_save_tare_orientation(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_TARE_ORIENTATION, buffer, serializer.offset);
}
enum mip_cmd_result mip_filter_load_tare_orientation(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_TARE_ORIENTATION, buffer, serializer.offset);
}
enum mip_cmd_result mip_filter_default_tare_orientation(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_TARE_ORIENTATION, buffer, serializer.offset);
}
void insert_mip_filter_sensor_to_vehicle_rotation_euler_command(struct mip_serializer* serializer, const struct mip_filter_sensor_to_vehicle_rotation_euler_command* self)
{
    insert_float(serializer, self->roll);
    
    insert_float(serializer, self->pitch);
    
    insert_float(serializer, self->yaw);
    
}
void extract_mip_filter_sensor_to_vehicle_rotation_euler_command(struct mip_serializer* serializer, struct mip_filter_sensor_to_vehicle_rotation_euler_command* self)
{
    extract_float(serializer, &self->roll);
    
    extract_float(serializer, &self->pitch);
    
    extract_float(serializer, &self->yaw);
    
}

enum mip_cmd_result mip_filter_write_sensor_to_vehicle_rotation_euler(struct mip_interface* device, float roll, float pitch, float yaw)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    insert_float(&serializer, roll);
    
    insert_float(&serializer, pitch);
    
    insert_float(&serializer, yaw);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_ROTATION_EULER, buffer, serializer.offset);
}
enum mip_cmd_result mip_filter_read_sensor_to_vehicle_rotation_euler(struct mip_interface* device, float* roll_out, float* pitch_out, float* yaw_out)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    enum mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_ROTATION_EULER, buffer, serializer.offset, MIP_REPLY_DESC_FILTER_SENSOR2VEHICLE_ROTATION_EULER, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        struct mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(roll_out);
        extract_float(&deserializer, roll_out);
        
        assert(pitch_out);
        extract_float(&deserializer, pitch_out);
        
        assert(yaw_out);
        extract_float(&deserializer, yaw_out);
        
        if( !mip_serializer_ok(&deserializer) )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
enum mip_cmd_result mip_filter_save_sensor_to_vehicle_rotation_euler(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_ROTATION_EULER, buffer, serializer.offset);
}
enum mip_cmd_result mip_filter_load_sensor_to_vehicle_rotation_euler(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_ROTATION_EULER, buffer, serializer.offset);
}
enum mip_cmd_result mip_filter_default_sensor_to_vehicle_rotation_euler(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_ROTATION_EULER, buffer, serializer.offset);
}
void insert_mip_filter_sensor_to_vehicle_rotation_dcm_command(struct mip_serializer* serializer, const struct mip_filter_sensor_to_vehicle_rotation_dcm_command* self)
{
    for(unsigned int i=0; i < 9; i++)
        insert_float(serializer, self->dcm[i]);
    
}
void extract_mip_filter_sensor_to_vehicle_rotation_dcm_command(struct mip_serializer* serializer, struct mip_filter_sensor_to_vehicle_rotation_dcm_command* self)
{
    for(unsigned int i=0; i < 9; i++)
        extract_float(serializer, &self->dcm[i]);
    
}

enum mip_cmd_result mip_filter_write_sensor_to_vehicle_rotation_dcm(struct mip_interface* device, const float* dcm)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    assert(dcm);
    for(unsigned int i=0; i < 9; i++)
        insert_float(&serializer, dcm[i]);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_ROTATION_DCM, buffer, serializer.offset);
}
enum mip_cmd_result mip_filter_read_sensor_to_vehicle_rotation_dcm(struct mip_interface* device, float* dcm_out)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    enum mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_ROTATION_DCM, buffer, serializer.offset, MIP_REPLY_DESC_FILTER_SENSOR2VEHICLE_ROTATION_DCM, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        struct mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(dcm_out);
        for(unsigned int i=0; i < 9; i++)
            extract_float(&deserializer, &dcm_out[i]);
        
        if( !mip_serializer_ok(&deserializer) )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
enum mip_cmd_result mip_filter_save_sensor_to_vehicle_rotation_dcm(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_ROTATION_DCM, buffer, serializer.offset);
}
enum mip_cmd_result mip_filter_load_sensor_to_vehicle_rotation_dcm(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_ROTATION_DCM, buffer, serializer.offset);
}
enum mip_cmd_result mip_filter_default_sensor_to_vehicle_rotation_dcm(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_ROTATION_DCM, buffer, serializer.offset);
}
void insert_mip_filter_sensor_to_vehicle_rotation_quaternion_command(struct mip_serializer* serializer, const struct mip_filter_sensor_to_vehicle_rotation_quaternion_command* self)
{
    for(unsigned int i=0; i < 4; i++)
        insert_float(serializer, self->quat[i]);
    
}
void extract_mip_filter_sensor_to_vehicle_rotation_quaternion_command(struct mip_serializer* serializer, struct mip_filter_sensor_to_vehicle_rotation_quaternion_command* self)
{
    for(unsigned int i=0; i < 4; i++)
        extract_float(serializer, &self->quat[i]);
    
}

enum mip_cmd_result mip_filter_write_sensor_to_vehicle_rotation_quaternion(struct mip_interface* device, const float* quat)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    assert(quat);
    for(unsigned int i=0; i < 4; i++)
        insert_float(&serializer, quat[i]);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_ROTATION_QUATERNION, buffer, serializer.offset);
}
enum mip_cmd_result mip_filter_read_sensor_to_vehicle_rotation_quaternion(struct mip_interface* device, float* quat_out)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    enum mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_ROTATION_QUATERNION, buffer, serializer.offset, MIP_REPLY_DESC_FILTER_SENSOR2VEHICLE_ROTATION_QUATERNION, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        struct mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(quat_out);
        for(unsigned int i=0; i < 4; i++)
            extract_float(&deserializer, &quat_out[i]);
        
        if( !mip_serializer_ok(&deserializer) )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
enum mip_cmd_result mip_filter_save_sensor_to_vehicle_rotation_quaternion(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_ROTATION_QUATERNION, buffer, serializer.offset);
}
enum mip_cmd_result mip_filter_load_sensor_to_vehicle_rotation_quaternion(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_ROTATION_QUATERNION, buffer, serializer.offset);
}
enum mip_cmd_result mip_filter_default_sensor_to_vehicle_rotation_quaternion(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_ROTATION_QUATERNION, buffer, serializer.offset);
}
void insert_mip_filter_sensor_to_vehicle_offset_command(struct mip_serializer* serializer, const struct mip_filter_sensor_to_vehicle_offset_command* self)
{
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->offset[i]);
    
}
void extract_mip_filter_sensor_to_vehicle_offset_command(struct mip_serializer* serializer, struct mip_filter_sensor_to_vehicle_offset_command* self)
{
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->offset[i]);
    
}

enum mip_cmd_result mip_filter_write_sensor_to_vehicle_offset(struct mip_interface* device, const float* offset)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    assert(offset);
    for(unsigned int i=0; i < 3; i++)
        insert_float(&serializer, offset[i]);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_OFFSET, buffer, serializer.offset);
}
enum mip_cmd_result mip_filter_read_sensor_to_vehicle_offset(struct mip_interface* device, float* offset_out)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    enum mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_OFFSET, buffer, serializer.offset, MIP_REPLY_DESC_FILTER_SENSOR2VEHICLE_OFFSET, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        struct mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(offset_out);
        for(unsigned int i=0; i < 3; i++)
            extract_float(&deserializer, &offset_out[i]);
        
        if( !mip_serializer_ok(&deserializer) )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
enum mip_cmd_result mip_filter_save_sensor_to_vehicle_offset(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_OFFSET, buffer, serializer.offset);
}
enum mip_cmd_result mip_filter_load_sensor_to_vehicle_offset(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_OFFSET, buffer, serializer.offset);
}
enum mip_cmd_result mip_filter_default_sensor_to_vehicle_offset(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_OFFSET, buffer, serializer.offset);
}
void insert_mip_filter_antenna_offset_command(struct mip_serializer* serializer, const struct mip_filter_antenna_offset_command* self)
{
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->offset[i]);
    
}
void extract_mip_filter_antenna_offset_command(struct mip_serializer* serializer, struct mip_filter_antenna_offset_command* self)
{
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->offset[i]);
    
}

enum mip_cmd_result mip_filter_write_antenna_offset(struct mip_interface* device, const float* offset)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    assert(offset);
    for(unsigned int i=0; i < 3; i++)
        insert_float(&serializer, offset[i]);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ANTENNA_OFFSET, buffer, serializer.offset);
}
enum mip_cmd_result mip_filter_read_antenna_offset(struct mip_interface* device, float* offset_out)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    enum mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ANTENNA_OFFSET, buffer, serializer.offset, MIP_REPLY_DESC_FILTER_ANTENNA_OFFSET, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        struct mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(offset_out);
        for(unsigned int i=0; i < 3; i++)
            extract_float(&deserializer, &offset_out[i]);
        
        if( !mip_serializer_ok(&deserializer) )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
enum mip_cmd_result mip_filter_save_antenna_offset(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ANTENNA_OFFSET, buffer, serializer.offset);
}
enum mip_cmd_result mip_filter_load_antenna_offset(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ANTENNA_OFFSET, buffer, serializer.offset);
}
enum mip_cmd_result mip_filter_default_antenna_offset(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ANTENNA_OFFSET, buffer, serializer.offset);
}
void insert_mip_filter_gnss_source_command(struct mip_serializer* serializer, const struct mip_filter_gnss_source_command* self)
{
    insert_mip_filter_gnss_source_command_source(serializer, self->source);
    
}
void extract_mip_filter_gnss_source_command(struct mip_serializer* serializer, struct mip_filter_gnss_source_command* self)
{
    extract_mip_filter_gnss_source_command_source(serializer, &self->source);
    
}

void insert_mip_filter_gnss_source_command_source(struct mip_serializer* serializer, const enum mip_filter_gnss_source_command_source self)
{
    return insert_u8(serializer, (uint8_t)(self));
}
void extract_mip_filter_gnss_source_command_source(struct mip_serializer* serializer, enum mip_filter_gnss_source_command_source* self)
{
    uint8_t tmp = 0;
    extract_u8(serializer, &tmp);
    *self = tmp;
}

enum mip_cmd_result mip_filter_write_gnss_source(struct mip_interface* device, enum mip_filter_gnss_source_command_source source)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    insert_mip_filter_gnss_source_command_source(&serializer, source);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_GNSS_SOURCE_CONTROL, buffer, serializer.offset);
}
enum mip_cmd_result mip_filter_read_gnss_source(struct mip_interface* device, enum mip_filter_gnss_source_command_source* source_out)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    enum mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_GNSS_SOURCE_CONTROL, buffer, serializer.offset, MIP_REPLY_DESC_FILTER_GNSS_SOURCE_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        struct mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(source_out);
        extract_mip_filter_gnss_source_command_source(&deserializer, source_out);
        
        if( !mip_serializer_ok(&deserializer) )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
enum mip_cmd_result mip_filter_save_gnss_source(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_GNSS_SOURCE_CONTROL, buffer, serializer.offset);
}
enum mip_cmd_result mip_filter_load_gnss_source(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_GNSS_SOURCE_CONTROL, buffer, serializer.offset);
}
enum mip_cmd_result mip_filter_default_gnss_source(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_GNSS_SOURCE_CONTROL, buffer, serializer.offset);
}
void insert_mip_filter_heading_source_command(struct mip_serializer* serializer, const struct mip_filter_heading_source_command* self)
{
    insert_mip_filter_heading_source_command_source(serializer, self->source);
    
}
void extract_mip_filter_heading_source_command(struct mip_serializer* serializer, struct mip_filter_heading_source_command* self)
{
    extract_mip_filter_heading_source_command_source(serializer, &self->source);
    
}

void insert_mip_filter_heading_source_command_source(struct mip_serializer* serializer, const enum mip_filter_heading_source_command_source self)
{
    return insert_u8(serializer, (uint8_t)(self));
}
void extract_mip_filter_heading_source_command_source(struct mip_serializer* serializer, enum mip_filter_heading_source_command_source* self)
{
    uint8_t tmp = 0;
    extract_u8(serializer, &tmp);
    *self = tmp;
}

enum mip_cmd_result mip_filter_write_heading_source(struct mip_interface* device, enum mip_filter_heading_source_command_source source)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    insert_mip_filter_heading_source_command_source(&serializer, source);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_HEADING_UPDATE_CONTROL, buffer, serializer.offset);
}
enum mip_cmd_result mip_filter_read_heading_source(struct mip_interface* device, enum mip_filter_heading_source_command_source* source_out)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    enum mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_HEADING_UPDATE_CONTROL, buffer, serializer.offset, MIP_REPLY_DESC_FILTER_HEADING_UPDATE_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        struct mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(source_out);
        extract_mip_filter_heading_source_command_source(&deserializer, source_out);
        
        if( !mip_serializer_ok(&deserializer) )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
enum mip_cmd_result mip_filter_save_heading_source(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_HEADING_UPDATE_CONTROL, buffer, serializer.offset);
}
enum mip_cmd_result mip_filter_load_heading_source(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_HEADING_UPDATE_CONTROL, buffer, serializer.offset);
}
enum mip_cmd_result mip_filter_default_heading_source(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_HEADING_UPDATE_CONTROL, buffer, serializer.offset);
}
void insert_mip_filter_altitude_aiding_command(struct mip_serializer* serializer, const struct mip_filter_altitude_aiding_command* self)
{
    insert_u8(serializer, self->aiding_selector);
    
}
void extract_mip_filter_altitude_aiding_command(struct mip_serializer* serializer, struct mip_filter_altitude_aiding_command* self)
{
    extract_u8(serializer, &self->aiding_selector);
    
}

enum mip_cmd_result mip_filter_write_altitude_aiding(struct mip_interface* device, uint8_t aiding_selector)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    insert_u8(&serializer, aiding_selector);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ALTITUDE_AIDING_CONTROL, buffer, serializer.offset);
}
enum mip_cmd_result mip_filter_read_altitude_aiding(struct mip_interface* device, uint8_t* aiding_selector_out)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    enum mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ALTITUDE_AIDING_CONTROL, buffer, serializer.offset, MIP_REPLY_DESC_FILTER_ALTITUDE_AIDING_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        struct mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(aiding_selector_out);
        extract_u8(&deserializer, aiding_selector_out);
        
        if( !mip_serializer_ok(&deserializer) )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
enum mip_cmd_result mip_filter_save_altitude_aiding(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ALTITUDE_AIDING_CONTROL, buffer, serializer.offset);
}
enum mip_cmd_result mip_filter_load_altitude_aiding(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ALTITUDE_AIDING_CONTROL, buffer, serializer.offset);
}
enum mip_cmd_result mip_filter_default_altitude_aiding(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ALTITUDE_AIDING_CONTROL, buffer, serializer.offset);
}
void insert_mip_filter_auto_zupt_command(struct mip_serializer* serializer, const struct mip_filter_auto_zupt_command* self)
{
    insert_u8(serializer, self->enable);
    
    insert_float(serializer, self->threshold);
    
}
void extract_mip_filter_auto_zupt_command(struct mip_serializer* serializer, struct mip_filter_auto_zupt_command* self)
{
    extract_u8(serializer, &self->enable);
    
    extract_float(serializer, &self->threshold);
    
}

enum mip_cmd_result mip_filter_write_auto_zupt(struct mip_interface* device, uint8_t enable, float threshold)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    insert_u8(&serializer, enable);
    
    insert_float(&serializer, threshold);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ZUPT_CONTROL, buffer, serializer.offset);
}
enum mip_cmd_result mip_filter_read_auto_zupt(struct mip_interface* device, uint8_t* enable_out, float* threshold_out)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    enum mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ZUPT_CONTROL, buffer, serializer.offset, MIP_REPLY_DESC_FILTER_ZUPT_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        struct mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(enable_out);
        extract_u8(&deserializer, enable_out);
        
        assert(threshold_out);
        extract_float(&deserializer, threshold_out);
        
        if( !mip_serializer_ok(&deserializer) )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
enum mip_cmd_result mip_filter_save_auto_zupt(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ZUPT_CONTROL, buffer, serializer.offset);
}
enum mip_cmd_result mip_filter_load_auto_zupt(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ZUPT_CONTROL, buffer, serializer.offset);
}
enum mip_cmd_result mip_filter_default_auto_zupt(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ZUPT_CONTROL, buffer, serializer.offset);
}
void insert_mip_filter_auto_angular_zupt_command(struct mip_serializer* serializer, const struct mip_filter_auto_angular_zupt_command* self)
{
    insert_u8(serializer, self->enable);
    
    insert_float(serializer, self->threshold);
    
}
void extract_mip_filter_auto_angular_zupt_command(struct mip_serializer* serializer, struct mip_filter_auto_angular_zupt_command* self)
{
    extract_u8(serializer, &self->enable);
    
    extract_float(serializer, &self->threshold);
    
}

enum mip_cmd_result mip_filter_write_auto_angular_zupt(struct mip_interface* device, uint8_t enable, float threshold)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    insert_u8(&serializer, enable);
    
    insert_float(&serializer, threshold);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ANGULAR_ZUPT_CONTROL, buffer, serializer.offset);
}
enum mip_cmd_result mip_filter_read_auto_angular_zupt(struct mip_interface* device, uint8_t* enable_out, float* threshold_out)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    enum mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ANGULAR_ZUPT_CONTROL, buffer, serializer.offset, MIP_REPLY_DESC_FILTER_ANGULAR_ZUPT_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        struct mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(enable_out);
        extract_u8(&deserializer, enable_out);
        
        assert(threshold_out);
        extract_float(&deserializer, threshold_out);
        
        if( !mip_serializer_ok(&deserializer) )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
enum mip_cmd_result mip_filter_save_auto_angular_zupt(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ANGULAR_ZUPT_CONTROL, buffer, serializer.offset);
}
enum mip_cmd_result mip_filter_load_auto_angular_zupt(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ANGULAR_ZUPT_CONTROL, buffer, serializer.offset);
}
enum mip_cmd_result mip_filter_default_auto_angular_zupt(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ANGULAR_ZUPT_CONTROL, buffer, serializer.offset);
}
enum mip_cmd_result mip_filter_commanded_zupt(struct mip_interface* device)
{
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_COMMANDED_ZUPT, NULL, 0);
}
enum mip_cmd_result mip_filter_commanded_angular_zupt(struct mip_interface* device)
{
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_COMMANDED_ANGULAR_ZUPT, NULL, 0);
}
void insert_mip_filter_aiding_measurement_enable_command(struct mip_serializer* serializer, const struct mip_filter_aiding_measurement_enable_command* self)
{
    insert_mip_filter_aiding_measurement_enable_command_aiding_source(serializer, self->aiding_source);
    
    insert_bool(serializer, self->enable);
    
}
void extract_mip_filter_aiding_measurement_enable_command(struct mip_serializer* serializer, struct mip_filter_aiding_measurement_enable_command* self)
{
    extract_mip_filter_aiding_measurement_enable_command_aiding_source(serializer, &self->aiding_source);
    
    extract_bool(serializer, &self->enable);
    
}

void insert_mip_filter_aiding_measurement_enable_command_aiding_source(struct mip_serializer* serializer, const enum mip_filter_aiding_measurement_enable_command_aiding_source self)
{
    return insert_u16(serializer, (uint16_t)(self));
}
void extract_mip_filter_aiding_measurement_enable_command_aiding_source(struct mip_serializer* serializer, enum mip_filter_aiding_measurement_enable_command_aiding_source* self)
{
    uint16_t tmp = 0;
    extract_u16(serializer, &tmp);
    *self = tmp;
}

enum mip_cmd_result mip_filter_write_aiding_measurement_enable(struct mip_interface* device, enum mip_filter_aiding_measurement_enable_command_aiding_source aiding_source, bool enable)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    insert_mip_filter_aiding_measurement_enable_command_aiding_source(&serializer, aiding_source);
    
    insert_bool(&serializer, enable);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_AIDING_MEASUREMENT_ENABLE, buffer, serializer.offset);
}
enum mip_cmd_result mip_filter_read_aiding_measurement_enable(struct mip_interface* device, enum mip_filter_aiding_measurement_enable_command_aiding_source aiding_source, bool* enable_out)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    insert_mip_filter_aiding_measurement_enable_command_aiding_source(&serializer, aiding_source);
    
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    enum mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_AIDING_MEASUREMENT_ENABLE, buffer, serializer.offset, MIP_REPLY_DESC_FILTER_AIDING_MEASUREMENT_ENABLE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        struct mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        extract_mip_filter_aiding_measurement_enable_command_aiding_source(&deserializer, &aiding_source);
        
        assert(enable_out);
        extract_bool(&deserializer, enable_out);
        
        if( !mip_serializer_ok(&deserializer) )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
enum mip_cmd_result mip_filter_save_aiding_measurement_enable(struct mip_interface* device, enum mip_filter_aiding_measurement_enable_command_aiding_source aiding_source)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    insert_mip_filter_aiding_measurement_enable_command_aiding_source(&serializer, aiding_source);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_AIDING_MEASUREMENT_ENABLE, buffer, serializer.offset);
}
enum mip_cmd_result mip_filter_load_aiding_measurement_enable(struct mip_interface* device, enum mip_filter_aiding_measurement_enable_command_aiding_source aiding_source)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    insert_mip_filter_aiding_measurement_enable_command_aiding_source(&serializer, aiding_source);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_AIDING_MEASUREMENT_ENABLE, buffer, serializer.offset);
}
enum mip_cmd_result mip_filter_default_aiding_measurement_enable(struct mip_interface* device, enum mip_filter_aiding_measurement_enable_command_aiding_source aiding_source)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    insert_mip_filter_aiding_measurement_enable_command_aiding_source(&serializer, aiding_source);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_AIDING_MEASUREMENT_ENABLE, buffer, serializer.offset);
}
enum mip_cmd_result mip_filter_run(struct mip_interface* device)
{
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_RUN, NULL, 0);
}
void insert_mip_filter_kinematic_constraint_command(struct mip_serializer* serializer, const struct mip_filter_kinematic_constraint_command* self)
{
    insert_u8(serializer, self->acceleration_constraint_selection);
    
    insert_u8(serializer, self->velocity_constraint_selection);
    
    insert_u8(serializer, self->angular_constraint_selection);
    
}
void extract_mip_filter_kinematic_constraint_command(struct mip_serializer* serializer, struct mip_filter_kinematic_constraint_command* self)
{
    extract_u8(serializer, &self->acceleration_constraint_selection);
    
    extract_u8(serializer, &self->velocity_constraint_selection);
    
    extract_u8(serializer, &self->angular_constraint_selection);
    
}

enum mip_cmd_result mip_filter_write_kinematic_constraint(struct mip_interface* device, uint8_t acceleration_constraint_selection, uint8_t velocity_constraint_selection, uint8_t angular_constraint_selection)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    insert_u8(&serializer, acceleration_constraint_selection);
    
    insert_u8(&serializer, velocity_constraint_selection);
    
    insert_u8(&serializer, angular_constraint_selection);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_KINEMATIC_CONSTRAINT, buffer, serializer.offset);
}
enum mip_cmd_result mip_filter_read_kinematic_constraint(struct mip_interface* device, uint8_t* acceleration_constraint_selection_out, uint8_t* velocity_constraint_selection_out, uint8_t* angular_constraint_selection_out)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    enum mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_KINEMATIC_CONSTRAINT, buffer, serializer.offset, MIP_REPLY_DESC_FILTER_KINEMATIC_CONSTRAINT, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        struct mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(acceleration_constraint_selection_out);
        extract_u8(&deserializer, acceleration_constraint_selection_out);
        
        assert(velocity_constraint_selection_out);
        extract_u8(&deserializer, velocity_constraint_selection_out);
        
        assert(angular_constraint_selection_out);
        extract_u8(&deserializer, angular_constraint_selection_out);
        
        if( !mip_serializer_ok(&deserializer) )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
enum mip_cmd_result mip_filter_save_kinematic_constraint(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_KINEMATIC_CONSTRAINT, buffer, serializer.offset);
}
enum mip_cmd_result mip_filter_load_kinematic_constraint(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_KINEMATIC_CONSTRAINT, buffer, serializer.offset);
}
enum mip_cmd_result mip_filter_default_kinematic_constraint(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_KINEMATIC_CONSTRAINT, buffer, serializer.offset);
}
void insert_mip_filter_initialization_configuration_command(struct mip_serializer* serializer, const struct mip_filter_initialization_configuration_command* self)
{
    insert_u8(serializer, self->wait_for_run_command);
    
    insert_mip_filter_initialization_configuration_command_initial_condition_source(serializer, self->initial_cond_src);
    
    insert_mip_filter_initialization_configuration_command_alignment_selector(serializer, self->auto_heading_alignment_selector);
    
    insert_float(serializer, self->initial_heading);
    
    insert_float(serializer, self->initial_pitch);
    
    insert_float(serializer, self->initial_roll);
    
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->initial_position[i]);
    
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->initial_velocity[i]);
    
    insert_mip_filter_reference_frame(serializer, self->reference_frame_selector);
    
}
void extract_mip_filter_initialization_configuration_command(struct mip_serializer* serializer, struct mip_filter_initialization_configuration_command* self)
{
    extract_u8(serializer, &self->wait_for_run_command);
    
    extract_mip_filter_initialization_configuration_command_initial_condition_source(serializer, &self->initial_cond_src);
    
    extract_mip_filter_initialization_configuration_command_alignment_selector(serializer, &self->auto_heading_alignment_selector);
    
    extract_float(serializer, &self->initial_heading);
    
    extract_float(serializer, &self->initial_pitch);
    
    extract_float(serializer, &self->initial_roll);
    
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->initial_position[i]);
    
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->initial_velocity[i]);
    
    extract_mip_filter_reference_frame(serializer, &self->reference_frame_selector);
    
}

void insert_mip_filter_initialization_configuration_command_alignment_selector(struct mip_serializer* serializer, const enum mip_filter_initialization_configuration_command_alignment_selector self)
{
    return insert_u8(serializer, (uint8_t)(self));
}
void extract_mip_filter_initialization_configuration_command_alignment_selector(struct mip_serializer* serializer, enum mip_filter_initialization_configuration_command_alignment_selector* self)
{
    uint8_t tmp = 0;
    extract_u8(serializer, &tmp);
    *self = tmp;
}

void insert_mip_filter_initialization_configuration_command_initial_condition_source(struct mip_serializer* serializer, const enum mip_filter_initialization_configuration_command_initial_condition_source self)
{
    return insert_u8(serializer, (uint8_t)(self));
}
void extract_mip_filter_initialization_configuration_command_initial_condition_source(struct mip_serializer* serializer, enum mip_filter_initialization_configuration_command_initial_condition_source* self)
{
    uint8_t tmp = 0;
    extract_u8(serializer, &tmp);
    *self = tmp;
}

enum mip_cmd_result mip_filter_write_initialization_configuration(struct mip_interface* device, uint8_t wait_for_run_command, enum mip_filter_initialization_configuration_command_initial_condition_source initial_cond_src, enum mip_filter_initialization_configuration_command_alignment_selector auto_heading_alignment_selector, float initial_heading, float initial_pitch, float initial_roll, const float* initial_position, const float* initial_velocity, enum mip_filter_reference_frame reference_frame_selector)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    insert_u8(&serializer, wait_for_run_command);
    
    insert_mip_filter_initialization_configuration_command_initial_condition_source(&serializer, initial_cond_src);
    
    insert_mip_filter_initialization_configuration_command_alignment_selector(&serializer, auto_heading_alignment_selector);
    
    insert_float(&serializer, initial_heading);
    
    insert_float(&serializer, initial_pitch);
    
    insert_float(&serializer, initial_roll);
    
    assert(initial_position);
    for(unsigned int i=0; i < 3; i++)
        insert_float(&serializer, initial_position[i]);
    
    assert(initial_velocity);
    for(unsigned int i=0; i < 3; i++)
        insert_float(&serializer, initial_velocity[i]);
    
    insert_mip_filter_reference_frame(&serializer, reference_frame_selector);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_INITIALIZATION_CONFIGURATION, buffer, serializer.offset);
}
enum mip_cmd_result mip_filter_read_initialization_configuration(struct mip_interface* device, uint8_t* wait_for_run_command_out, enum mip_filter_initialization_configuration_command_initial_condition_source* initial_cond_src_out, enum mip_filter_initialization_configuration_command_alignment_selector* auto_heading_alignment_selector_out, float* initial_heading_out, float* initial_pitch_out, float* initial_roll_out, float* initial_position_out, float* initial_velocity_out, enum mip_filter_reference_frame* reference_frame_selector_out)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    enum mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_INITIALIZATION_CONFIGURATION, buffer, serializer.offset, MIP_REPLY_DESC_FILTER_INITIALIZATION_CONFIGURATION, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        struct mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(wait_for_run_command_out);
        extract_u8(&deserializer, wait_for_run_command_out);
        
        assert(initial_cond_src_out);
        extract_mip_filter_initialization_configuration_command_initial_condition_source(&deserializer, initial_cond_src_out);
        
        assert(auto_heading_alignment_selector_out);
        extract_mip_filter_initialization_configuration_command_alignment_selector(&deserializer, auto_heading_alignment_selector_out);
        
        assert(initial_heading_out);
        extract_float(&deserializer, initial_heading_out);
        
        assert(initial_pitch_out);
        extract_float(&deserializer, initial_pitch_out);
        
        assert(initial_roll_out);
        extract_float(&deserializer, initial_roll_out);
        
        assert(initial_position_out);
        for(unsigned int i=0; i < 3; i++)
            extract_float(&deserializer, &initial_position_out[i]);
        
        assert(initial_velocity_out);
        for(unsigned int i=0; i < 3; i++)
            extract_float(&deserializer, &initial_velocity_out[i]);
        
        assert(reference_frame_selector_out);
        extract_mip_filter_reference_frame(&deserializer, reference_frame_selector_out);
        
        if( !mip_serializer_ok(&deserializer) )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
enum mip_cmd_result mip_filter_save_initialization_configuration(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_INITIALIZATION_CONFIGURATION, buffer, serializer.offset);
}
enum mip_cmd_result mip_filter_load_initialization_configuration(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_INITIALIZATION_CONFIGURATION, buffer, serializer.offset);
}
enum mip_cmd_result mip_filter_default_initialization_configuration(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_INITIALIZATION_CONFIGURATION, buffer, serializer.offset);
}
void insert_mip_filter_adaptive_filter_options_command(struct mip_serializer* serializer, const struct mip_filter_adaptive_filter_options_command* self)
{
    insert_u8(serializer, self->level);
    
    insert_u16(serializer, self->time_limit);
    
}
void extract_mip_filter_adaptive_filter_options_command(struct mip_serializer* serializer, struct mip_filter_adaptive_filter_options_command* self)
{
    extract_u8(serializer, &self->level);
    
    extract_u16(serializer, &self->time_limit);
    
}

enum mip_cmd_result mip_filter_write_adaptive_filter_options(struct mip_interface* device, uint8_t level, uint16_t time_limit)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    insert_u8(&serializer, level);
    
    insert_u16(&serializer, time_limit);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ADAPTIVE_FILTER_OPTIONS, buffer, serializer.offset);
}
enum mip_cmd_result mip_filter_read_adaptive_filter_options(struct mip_interface* device, uint8_t* level_out, uint16_t* time_limit_out)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    enum mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ADAPTIVE_FILTER_OPTIONS, buffer, serializer.offset, MIP_REPLY_DESC_FILTER_ADAPTIVE_FILTER_OPTIONS, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        struct mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(level_out);
        extract_u8(&deserializer, level_out);
        
        assert(time_limit_out);
        extract_u16(&deserializer, time_limit_out);
        
        if( !mip_serializer_ok(&deserializer) )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
enum mip_cmd_result mip_filter_save_adaptive_filter_options(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ADAPTIVE_FILTER_OPTIONS, buffer, serializer.offset);
}
enum mip_cmd_result mip_filter_load_adaptive_filter_options(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ADAPTIVE_FILTER_OPTIONS, buffer, serializer.offset);
}
enum mip_cmd_result mip_filter_default_adaptive_filter_options(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ADAPTIVE_FILTER_OPTIONS, buffer, serializer.offset);
}
void insert_mip_filter_multi_antenna_offset_command(struct mip_serializer* serializer, const struct mip_filter_multi_antenna_offset_command* self)
{
    insert_u8(serializer, self->receiver_id);
    
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->antenna_offset[i]);
    
}
void extract_mip_filter_multi_antenna_offset_command(struct mip_serializer* serializer, struct mip_filter_multi_antenna_offset_command* self)
{
    extract_u8(serializer, &self->receiver_id);
    
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->antenna_offset[i]);
    
}

enum mip_cmd_result mip_filter_write_multi_antenna_offset(struct mip_interface* device, uint8_t receiver_id, const float* antenna_offset)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    insert_u8(&serializer, receiver_id);
    
    assert(antenna_offset);
    for(unsigned int i=0; i < 3; i++)
        insert_float(&serializer, antenna_offset[i]);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_MULTI_ANTENNA_OFFSET, buffer, serializer.offset);
}
enum mip_cmd_result mip_filter_read_multi_antenna_offset(struct mip_interface* device, uint8_t receiver_id, float* antenna_offset_out)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    insert_u8(&serializer, receiver_id);
    
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    enum mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_MULTI_ANTENNA_OFFSET, buffer, serializer.offset, MIP_REPLY_DESC_FILTER_MULTI_ANTENNA_OFFSET, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        struct mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        extract_u8(&deserializer, &receiver_id);
        
        assert(antenna_offset_out);
        for(unsigned int i=0; i < 3; i++)
            extract_float(&deserializer, &antenna_offset_out[i]);
        
        if( !mip_serializer_ok(&deserializer) )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
enum mip_cmd_result mip_filter_save_multi_antenna_offset(struct mip_interface* device, uint8_t receiver_id)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    insert_u8(&serializer, receiver_id);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_MULTI_ANTENNA_OFFSET, buffer, serializer.offset);
}
enum mip_cmd_result mip_filter_load_multi_antenna_offset(struct mip_interface* device, uint8_t receiver_id)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    insert_u8(&serializer, receiver_id);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_MULTI_ANTENNA_OFFSET, buffer, serializer.offset);
}
enum mip_cmd_result mip_filter_default_multi_antenna_offset(struct mip_interface* device, uint8_t receiver_id)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    insert_u8(&serializer, receiver_id);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_MULTI_ANTENNA_OFFSET, buffer, serializer.offset);
}
void insert_mip_filter_rel_pos_configuration_command(struct mip_serializer* serializer, const struct mip_filter_rel_pos_configuration_command* self)
{
    insert_u8(serializer, self->source);
    
    insert_mip_filter_reference_frame(serializer, self->reference_frame_selector);
    
    for(unsigned int i=0; i < 3; i++)
        insert_double(serializer, self->reference_coordinates[i]);
    
}
void extract_mip_filter_rel_pos_configuration_command(struct mip_serializer* serializer, struct mip_filter_rel_pos_configuration_command* self)
{
    extract_u8(serializer, &self->source);
    
    extract_mip_filter_reference_frame(serializer, &self->reference_frame_selector);
    
    for(unsigned int i=0; i < 3; i++)
        extract_double(serializer, &self->reference_coordinates[i]);
    
}

enum mip_cmd_result mip_filter_write_rel_pos_configuration(struct mip_interface* device, uint8_t source, enum mip_filter_reference_frame reference_frame_selector, const double* reference_coordinates)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    insert_u8(&serializer, source);
    
    insert_mip_filter_reference_frame(&serializer, reference_frame_selector);
    
    assert(reference_coordinates);
    for(unsigned int i=0; i < 3; i++)
        insert_double(&serializer, reference_coordinates[i]);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_REL_POS_CONFIGURATION, buffer, serializer.offset);
}
enum mip_cmd_result mip_filter_read_rel_pos_configuration(struct mip_interface* device, uint8_t* source_out, enum mip_filter_reference_frame* reference_frame_selector_out, double* reference_coordinates_out)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    enum mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_REL_POS_CONFIGURATION, buffer, serializer.offset, MIP_REPLY_DESC_FILTER_REL_POS_CONFIGURATION, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        struct mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(source_out);
        extract_u8(&deserializer, source_out);
        
        assert(reference_frame_selector_out);
        extract_mip_filter_reference_frame(&deserializer, reference_frame_selector_out);
        
        assert(reference_coordinates_out);
        for(unsigned int i=0; i < 3; i++)
            extract_double(&deserializer, &reference_coordinates_out[i]);
        
        if( !mip_serializer_ok(&deserializer) )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
enum mip_cmd_result mip_filter_save_rel_pos_configuration(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_REL_POS_CONFIGURATION, buffer, serializer.offset);
}
enum mip_cmd_result mip_filter_load_rel_pos_configuration(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_REL_POS_CONFIGURATION, buffer, serializer.offset);
}
enum mip_cmd_result mip_filter_default_rel_pos_configuration(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_REL_POS_CONFIGURATION, buffer, serializer.offset);
}
void insert_mip_filter_ref_point_lever_arm_command(struct mip_serializer* serializer, const struct mip_filter_ref_point_lever_arm_command* self)
{
    insert_mip_filter_ref_point_lever_arm_command_reference_point_selector(serializer, self->ref_point_sel);
    
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->lever_arm_offset[i]);
    
}
void extract_mip_filter_ref_point_lever_arm_command(struct mip_serializer* serializer, struct mip_filter_ref_point_lever_arm_command* self)
{
    extract_mip_filter_ref_point_lever_arm_command_reference_point_selector(serializer, &self->ref_point_sel);
    
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->lever_arm_offset[i]);
    
}

void insert_mip_filter_ref_point_lever_arm_command_reference_point_selector(struct mip_serializer* serializer, const enum mip_filter_ref_point_lever_arm_command_reference_point_selector self)
{
    return insert_u8(serializer, (uint8_t)(self));
}
void extract_mip_filter_ref_point_lever_arm_command_reference_point_selector(struct mip_serializer* serializer, enum mip_filter_ref_point_lever_arm_command_reference_point_selector* self)
{
    uint8_t tmp = 0;
    extract_u8(serializer, &tmp);
    *self = tmp;
}

enum mip_cmd_result mip_filter_write_ref_point_lever_arm(struct mip_interface* device, enum mip_filter_ref_point_lever_arm_command_reference_point_selector ref_point_sel, const float* lever_arm_offset)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    insert_mip_filter_ref_point_lever_arm_command_reference_point_selector(&serializer, ref_point_sel);
    
    assert(lever_arm_offset);
    for(unsigned int i=0; i < 3; i++)
        insert_float(&serializer, lever_arm_offset[i]);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_REF_POINT_LEVER_ARM, buffer, serializer.offset);
}
enum mip_cmd_result mip_filter_read_ref_point_lever_arm(struct mip_interface* device, enum mip_filter_ref_point_lever_arm_command_reference_point_selector* ref_point_sel_out, float* lever_arm_offset_out)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    enum mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_REF_POINT_LEVER_ARM, buffer, serializer.offset, MIP_REPLY_DESC_FILTER_REF_POINT_LEVER_ARM, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        struct mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(ref_point_sel_out);
        extract_mip_filter_ref_point_lever_arm_command_reference_point_selector(&deserializer, ref_point_sel_out);
        
        assert(lever_arm_offset_out);
        for(unsigned int i=0; i < 3; i++)
            extract_float(&deserializer, &lever_arm_offset_out[i]);
        
        if( !mip_serializer_ok(&deserializer) )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
enum mip_cmd_result mip_filter_save_ref_point_lever_arm(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_REF_POINT_LEVER_ARM, buffer, serializer.offset);
}
enum mip_cmd_result mip_filter_load_ref_point_lever_arm(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_REF_POINT_LEVER_ARM, buffer, serializer.offset);
}
enum mip_cmd_result mip_filter_default_ref_point_lever_arm(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_REF_POINT_LEVER_ARM, buffer, serializer.offset);
}
void insert_mip_filter_speed_measurement_command(struct mip_serializer* serializer, const struct mip_filter_speed_measurement_command* self)
{
    insert_u8(serializer, self->source);
    
    insert_float(serializer, self->time_of_week);
    
    insert_float(serializer, self->speed);
    
    insert_float(serializer, self->speed_uncertainty);
    
}
void extract_mip_filter_speed_measurement_command(struct mip_serializer* serializer, struct mip_filter_speed_measurement_command* self)
{
    extract_u8(serializer, &self->source);
    
    extract_float(serializer, &self->time_of_week);
    
    extract_float(serializer, &self->speed);
    
    extract_float(serializer, &self->speed_uncertainty);
    
}

enum mip_cmd_result mip_filter_speed_measurement(struct mip_interface* device, uint8_t source, float time_of_week, float speed, float speed_uncertainty)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_u8(&serializer, source);
    
    insert_float(&serializer, time_of_week);
    
    insert_float(&serializer, speed);
    
    insert_float(&serializer, speed_uncertainty);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SPEED_MEASUREMENT, buffer, serializer.offset);
}
void insert_mip_filter_speed_lever_arm_command(struct mip_serializer* serializer, const struct mip_filter_speed_lever_arm_command* self)
{
    insert_u8(serializer, self->source);
    
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->lever_arm_offset[i]);
    
}
void extract_mip_filter_speed_lever_arm_command(struct mip_serializer* serializer, struct mip_filter_speed_lever_arm_command* self)
{
    extract_u8(serializer, &self->source);
    
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->lever_arm_offset[i]);
    
}

enum mip_cmd_result mip_filter_write_speed_lever_arm(struct mip_interface* device, uint8_t source, const float* lever_arm_offset)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    insert_u8(&serializer, source);
    
    assert(lever_arm_offset);
    for(unsigned int i=0; i < 3; i++)
        insert_float(&serializer, lever_arm_offset[i]);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SPEED_LEVER_ARM, buffer, serializer.offset);
}
enum mip_cmd_result mip_filter_read_speed_lever_arm(struct mip_interface* device, uint8_t source, float* lever_arm_offset_out)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    insert_u8(&serializer, source);
    
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    enum mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SPEED_LEVER_ARM, buffer, serializer.offset, MIP_REPLY_DESC_FILTER_SPEED_LEVER_ARM, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        struct mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        extract_u8(&deserializer, &source);
        
        assert(lever_arm_offset_out);
        for(unsigned int i=0; i < 3; i++)
            extract_float(&deserializer, &lever_arm_offset_out[i]);
        
        if( !mip_serializer_ok(&deserializer) )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
enum mip_cmd_result mip_filter_save_speed_lever_arm(struct mip_interface* device, uint8_t source)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    insert_u8(&serializer, source);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SPEED_LEVER_ARM, buffer, serializer.offset);
}
enum mip_cmd_result mip_filter_load_speed_lever_arm(struct mip_interface* device, uint8_t source)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    insert_u8(&serializer, source);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SPEED_LEVER_ARM, buffer, serializer.offset);
}
enum mip_cmd_result mip_filter_default_speed_lever_arm(struct mip_interface* device, uint8_t source)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    insert_u8(&serializer, source);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SPEED_LEVER_ARM, buffer, serializer.offset);
}
void insert_mip_filter_wheeled_vehicle_constraint_control_command(struct mip_serializer* serializer, const struct mip_filter_wheeled_vehicle_constraint_control_command* self)
{
    insert_u8(serializer, self->enable);
    
}
void extract_mip_filter_wheeled_vehicle_constraint_control_command(struct mip_serializer* serializer, struct mip_filter_wheeled_vehicle_constraint_control_command* self)
{
    extract_u8(serializer, &self->enable);
    
}

enum mip_cmd_result mip_filter_write_wheeled_vehicle_constraint_control(struct mip_interface* device, uint8_t enable)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    insert_u8(&serializer, enable);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_WHEELED_VEHICLE_CONSTRAINT_CONTROL, buffer, serializer.offset);
}
enum mip_cmd_result mip_filter_read_wheeled_vehicle_constraint_control(struct mip_interface* device, uint8_t* enable_out)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    enum mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_WHEELED_VEHICLE_CONSTRAINT_CONTROL, buffer, serializer.offset, MIP_REPLY_DESC_WHEELED_VEHICLE_CONSTRAINT_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        struct mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(enable_out);
        extract_u8(&deserializer, enable_out);
        
        if( !mip_serializer_ok(&deserializer) )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
enum mip_cmd_result mip_filter_save_wheeled_vehicle_constraint_control(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_WHEELED_VEHICLE_CONSTRAINT_CONTROL, buffer, serializer.offset);
}
enum mip_cmd_result mip_filter_load_wheeled_vehicle_constraint_control(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_WHEELED_VEHICLE_CONSTRAINT_CONTROL, buffer, serializer.offset);
}
enum mip_cmd_result mip_filter_default_wheeled_vehicle_constraint_control(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_WHEELED_VEHICLE_CONSTRAINT_CONTROL, buffer, serializer.offset);
}
void insert_mip_filter_vertical_gyro_constraint_control_command(struct mip_serializer* serializer, const struct mip_filter_vertical_gyro_constraint_control_command* self)
{
    insert_u8(serializer, self->enable);
    
}
void extract_mip_filter_vertical_gyro_constraint_control_command(struct mip_serializer* serializer, struct mip_filter_vertical_gyro_constraint_control_command* self)
{
    extract_u8(serializer, &self->enable);
    
}

enum mip_cmd_result mip_filter_write_vertical_gyro_constraint_control(struct mip_interface* device, uint8_t enable)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    insert_u8(&serializer, enable);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_VERTICAL_GYRO_CONSTRAINT_CONTROL, buffer, serializer.offset);
}
enum mip_cmd_result mip_filter_read_vertical_gyro_constraint_control(struct mip_interface* device, uint8_t* enable_out)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    enum mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_VERTICAL_GYRO_CONSTRAINT_CONTROL, buffer, serializer.offset, MIP_REPLY_DESC_VERTICAL_GYRO_CONSTRAINT_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        struct mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(enable_out);
        extract_u8(&deserializer, enable_out);
        
        if( !mip_serializer_ok(&deserializer) )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
enum mip_cmd_result mip_filter_save_vertical_gyro_constraint_control(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_VERTICAL_GYRO_CONSTRAINT_CONTROL, buffer, serializer.offset);
}
enum mip_cmd_result mip_filter_load_vertical_gyro_constraint_control(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_VERTICAL_GYRO_CONSTRAINT_CONTROL, buffer, serializer.offset);
}
enum mip_cmd_result mip_filter_default_vertical_gyro_constraint_control(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_VERTICAL_GYRO_CONSTRAINT_CONTROL, buffer, serializer.offset);
}
void insert_mip_filter_gnss_antenna_cal_control_command(struct mip_serializer* serializer, const struct mip_filter_gnss_antenna_cal_control_command* self)
{
    insert_u8(serializer, self->enable);
    
    insert_float(serializer, self->max_offset);
    
}
void extract_mip_filter_gnss_antenna_cal_control_command(struct mip_serializer* serializer, struct mip_filter_gnss_antenna_cal_control_command* self)
{
    extract_u8(serializer, &self->enable);
    
    extract_float(serializer, &self->max_offset);
    
}

enum mip_cmd_result mip_filter_write_gnss_antenna_cal_control(struct mip_interface* device, uint8_t enable, float max_offset)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    insert_u8(&serializer, enable);
    
    insert_float(&serializer, max_offset);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_GNSS_ANTENNA_CALIBRATION_CONTROL, buffer, serializer.offset);
}
enum mip_cmd_result mip_filter_read_gnss_antenna_cal_control(struct mip_interface* device, uint8_t* enable_out, float* max_offset_out)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    enum mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_GNSS_ANTENNA_CALIBRATION_CONTROL, buffer, serializer.offset, MIP_REPLY_DESC_GNSS_ANTENNA_CALIBRATION_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        struct mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(enable_out);
        extract_u8(&deserializer, enable_out);
        
        assert(max_offset_out);
        extract_float(&deserializer, max_offset_out);
        
        if( !mip_serializer_ok(&deserializer) )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
enum mip_cmd_result mip_filter_save_gnss_antenna_cal_control(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_GNSS_ANTENNA_CALIBRATION_CONTROL, buffer, serializer.offset);
}
enum mip_cmd_result mip_filter_load_gnss_antenna_cal_control(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_GNSS_ANTENNA_CALIBRATION_CONTROL, buffer, serializer.offset);
}
enum mip_cmd_result mip_filter_default_gnss_antenna_cal_control(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_GNSS_ANTENNA_CALIBRATION_CONTROL, buffer, serializer.offset);
}
void insert_mip_filter_magnetic_declination_source_command(struct mip_serializer* serializer, const struct mip_filter_magnetic_declination_source_command* self)
{
    insert_mip_filter_mag_declination_source(serializer, self->source);
    
    insert_float(serializer, self->declination);
    
}
void extract_mip_filter_magnetic_declination_source_command(struct mip_serializer* serializer, struct mip_filter_magnetic_declination_source_command* self)
{
    extract_mip_filter_mag_declination_source(serializer, &self->source);
    
    extract_float(serializer, &self->declination);
    
}

enum mip_cmd_result mip_filter_write_magnetic_declination_source(struct mip_interface* device, enum mip_filter_mag_declination_source source, float declination)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    insert_mip_filter_mag_declination_source(&serializer, source);
    
    insert_float(&serializer, declination);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_DECLINATION_SOURCE, buffer, serializer.offset);
}
enum mip_cmd_result mip_filter_read_magnetic_declination_source(struct mip_interface* device, enum mip_filter_mag_declination_source* source_out, float* declination_out)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    enum mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_DECLINATION_SOURCE, buffer, serializer.offset, MIP_REPLY_DESC_FILTER_DECLINATION_SOURCE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        struct mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(source_out);
        extract_mip_filter_mag_declination_source(&deserializer, source_out);
        
        assert(declination_out);
        extract_float(&deserializer, declination_out);
        
        if( !mip_serializer_ok(&deserializer) )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
enum mip_cmd_result mip_filter_save_magnetic_declination_source(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_DECLINATION_SOURCE, buffer, serializer.offset);
}
enum mip_cmd_result mip_filter_load_magnetic_declination_source(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_DECLINATION_SOURCE, buffer, serializer.offset);
}
enum mip_cmd_result mip_filter_default_magnetic_declination_source(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_DECLINATION_SOURCE, buffer, serializer.offset);
}
void insert_mip_filter_set_initial_heading_command(struct mip_serializer* serializer, const struct mip_filter_set_initial_heading_command* self)
{
    insert_float(serializer, self->heading);
    
}
void extract_mip_filter_set_initial_heading_command(struct mip_serializer* serializer, struct mip_filter_set_initial_heading_command* self)
{
    extract_float(serializer, &self->heading);
    
}

enum mip_cmd_result mip_filter_set_initial_heading(struct mip_interface* device, float heading)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_float(&serializer, heading);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SET_INITIAL_HEADING, buffer, serializer.offset);
}

#ifdef __cplusplus
} // namespace C
} // namespace mip
} // extern "C"
#endif // __cplusplus

