
#include "commands_filter.h"

#include "../../utils/serialization.h"
#include "../mip_interface.h"

#include <assert.h>


#ifdef __cplusplus
namespace mscl {
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

void insert_mip_filter_estimation_control_command(struct mip_serializer* serializer, const struct mip_filter_estimation_control_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    insert_mip_filter_estimation_control_command_enable_flags(serializer, self->enable);
}

void extract_mip_filter_estimation_control_command(struct mip_serializer* serializer, struct mip_filter_estimation_control_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    extract_mip_filter_estimation_control_command_enable_flags(serializer, &self->enable);
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

void insert_mip_filter_tare_orientation_command(struct mip_serializer* serializer, const struct mip_filter_tare_orientation_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    insert_mip_filter_tare_orientation_command_mip_tare_axes(serializer, self->axes);
}

void extract_mip_filter_tare_orientation_command(struct mip_serializer* serializer, struct mip_filter_tare_orientation_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    extract_mip_filter_tare_orientation_command_mip_tare_axes(serializer, &self->axes);
}

void insert_mip_filter_sensor_to_vehicle_rotation_euler_command(struct mip_serializer* serializer, const struct mip_filter_sensor_to_vehicle_rotation_euler_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    insert_float(serializer, self->roll);
    insert_float(serializer, self->pitch);
    insert_float(serializer, self->yaw);
}

void extract_mip_filter_sensor_to_vehicle_rotation_euler_command(struct mip_serializer* serializer, struct mip_filter_sensor_to_vehicle_rotation_euler_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    extract_float(serializer, &self->roll);
    extract_float(serializer, &self->pitch);
    extract_float(serializer, &self->yaw);
}

void insert_mip_filter_sensor_to_vehicle_rotation_dcm_command(struct mip_serializer* serializer, const struct mip_filter_sensor_to_vehicle_rotation_dcm_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    for(unsigned int i=0; i < 9; i++)
        insert_float(serializer, self->dcm[i]);
}

void extract_mip_filter_sensor_to_vehicle_rotation_dcm_command(struct mip_serializer* serializer, struct mip_filter_sensor_to_vehicle_rotation_dcm_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    for(unsigned int i=0; i < 9; i++)
        extract_float(serializer, &self->dcm[i]);
}

void insert_mip_filter_sensor_to_vehicle_rotation_quaternion_command(struct mip_serializer* serializer, const struct mip_filter_sensor_to_vehicle_rotation_quaternion_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    for(unsigned int i=0; i < 4; i++)
        insert_float(serializer, self->quat[i]);
}

void extract_mip_filter_sensor_to_vehicle_rotation_quaternion_command(struct mip_serializer* serializer, struct mip_filter_sensor_to_vehicle_rotation_quaternion_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    for(unsigned int i=0; i < 4; i++)
        extract_float(serializer, &self->quat[i]);
}

void insert_mip_filter_sensor_to_vehicle_offset_command(struct mip_serializer* serializer, const struct mip_filter_sensor_to_vehicle_offset_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->offset[i]);
}

void extract_mip_filter_sensor_to_vehicle_offset_command(struct mip_serializer* serializer, struct mip_filter_sensor_to_vehicle_offset_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->offset[i]);
}

void insert_mip_filter_antenna_offset_command(struct mip_serializer* serializer, const struct mip_filter_antenna_offset_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->offset[i]);
}

void extract_mip_filter_antenna_offset_command(struct mip_serializer* serializer, struct mip_filter_antenna_offset_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->offset[i]);
}

void insert_mip_filter_gnss_source_command(struct mip_serializer* serializer, const struct mip_filter_gnss_source_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    insert_mip_filter_gnss_source_command_source(serializer, self->source);
}

void extract_mip_filter_gnss_source_command(struct mip_serializer* serializer, struct mip_filter_gnss_source_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    extract_mip_filter_gnss_source_command_source(serializer, &self->source);
}

void insert_mip_filter_heading_source_command(struct mip_serializer* serializer, const struct mip_filter_heading_source_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    insert_mip_filter_heading_source_command_source(serializer, self->source);
}

void extract_mip_filter_heading_source_command(struct mip_serializer* serializer, struct mip_filter_heading_source_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    extract_mip_filter_heading_source_command_source(serializer, &self->source);
}

void insert_mip_filter_altitude_aiding_command(struct mip_serializer* serializer, const struct mip_filter_altitude_aiding_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    insert_u8(serializer, self->aiding_selector);
}

void extract_mip_filter_altitude_aiding_command(struct mip_serializer* serializer, struct mip_filter_altitude_aiding_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    extract_u8(serializer, &self->aiding_selector);
}

void insert_mip_filter_auto_zupt_command(struct mip_serializer* serializer, const struct mip_filter_auto_zupt_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    insert_u8(serializer, self->enable);
    insert_float(serializer, self->threshold);
}

void extract_mip_filter_auto_zupt_command(struct mip_serializer* serializer, struct mip_filter_auto_zupt_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    extract_u8(serializer, &self->enable);
    extract_float(serializer, &self->threshold);
}

void insert_mip_filter_auto_angular_zupt_command(struct mip_serializer* serializer, const struct mip_filter_auto_angular_zupt_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    insert_u8(serializer, self->enable);
    insert_float(serializer, self->threshold);
}

void extract_mip_filter_auto_angular_zupt_command(struct mip_serializer* serializer, struct mip_filter_auto_angular_zupt_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    extract_u8(serializer, &self->enable);
    extract_float(serializer, &self->threshold);
}

void insert_mip_filter_aiding_measurement_enable_command(struct mip_serializer* serializer, const struct mip_filter_aiding_measurement_enable_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    insert_mip_filter_aiding_measurement_enable_command_aiding_source(serializer, self->aiding_source);
    insert_bool(serializer, self->enable);
}

void extract_mip_filter_aiding_measurement_enable_command(struct mip_serializer* serializer, struct mip_filter_aiding_measurement_enable_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    extract_mip_filter_aiding_measurement_enable_command_aiding_source(serializer, &self->aiding_source);
    extract_bool(serializer, &self->enable);
}

void insert_mip_filter_kinematic_constraint_command(struct mip_serializer* serializer, const struct mip_filter_kinematic_constraint_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    insert_u8(serializer, self->acceleration_constraint_selection);
    insert_u8(serializer, self->velocity_constraint_selection);
    insert_u8(serializer, self->angular_constraint_selection);
}

void extract_mip_filter_kinematic_constraint_command(struct mip_serializer* serializer, struct mip_filter_kinematic_constraint_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    extract_u8(serializer, &self->acceleration_constraint_selection);
    extract_u8(serializer, &self->velocity_constraint_selection);
    extract_u8(serializer, &self->angular_constraint_selection);
}

void insert_mip_filter_initialization_configuration_command(struct mip_serializer* serializer, const struct mip_filter_initialization_configuration_command* self)
{
    insert_mip_function_selector(serializer, self->function);
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
    extract_mip_function_selector(serializer, &self->function);
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

void insert_mip_filter_adaptive_filter_options_command(struct mip_serializer* serializer, const struct mip_filter_adaptive_filter_options_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    insert_u8(serializer, self->level);
    insert_u16(serializer, self->time_limit);
}

void extract_mip_filter_adaptive_filter_options_command(struct mip_serializer* serializer, struct mip_filter_adaptive_filter_options_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    extract_u8(serializer, &self->level);
    extract_u16(serializer, &self->time_limit);
}

void insert_mip_filter_multi_antenna_offset_command(struct mip_serializer* serializer, const struct mip_filter_multi_antenna_offset_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    insert_u8(serializer, self->receiver_id);
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->antenna_offset[i]);
}

void extract_mip_filter_multi_antenna_offset_command(struct mip_serializer* serializer, struct mip_filter_multi_antenna_offset_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    extract_u8(serializer, &self->receiver_id);
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->antenna_offset[i]);
}

void insert_mip_filter_rel_pos_configuration_command(struct mip_serializer* serializer, const struct mip_filter_rel_pos_configuration_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    insert_u8(serializer, self->source);
    insert_mip_filter_reference_frame(serializer, self->reference_frame_selector);
    for(unsigned int i=0; i < 3; i++)
        insert_double(serializer, self->reference_coordinates[i]);
}

void extract_mip_filter_rel_pos_configuration_command(struct mip_serializer* serializer, struct mip_filter_rel_pos_configuration_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    extract_u8(serializer, &self->source);
    extract_mip_filter_reference_frame(serializer, &self->reference_frame_selector);
    for(unsigned int i=0; i < 3; i++)
        extract_double(serializer, &self->reference_coordinates[i]);
}

void insert_mip_filter_ref_point_lever_arm_command(struct mip_serializer* serializer, const struct mip_filter_ref_point_lever_arm_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    insert_mip_filter_ref_point_lever_arm_command_reference_point_selector(serializer, self->ref_point_sel);
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->lever_arm_offset[i]);
}

void extract_mip_filter_ref_point_lever_arm_command(struct mip_serializer* serializer, struct mip_filter_ref_point_lever_arm_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    extract_mip_filter_ref_point_lever_arm_command_reference_point_selector(serializer, &self->ref_point_sel);
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->lever_arm_offset[i]);
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

void insert_mip_filter_speed_lever_arm_command(struct mip_serializer* serializer, const struct mip_filter_speed_lever_arm_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    insert_u8(serializer, self->source);
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->lever_arm_offset[i]);
}

void extract_mip_filter_speed_lever_arm_command(struct mip_serializer* serializer, struct mip_filter_speed_lever_arm_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    extract_u8(serializer, &self->source);
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->lever_arm_offset[i]);
}

void insert_mip_filter_wheeled_vehicle_constraint_control_command(struct mip_serializer* serializer, const struct mip_filter_wheeled_vehicle_constraint_control_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    insert_u8(serializer, self->enable);
}

void extract_mip_filter_wheeled_vehicle_constraint_control_command(struct mip_serializer* serializer, struct mip_filter_wheeled_vehicle_constraint_control_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    extract_u8(serializer, &self->enable);
}

void insert_mip_filter_vertical_gyro_constraint_control_command(struct mip_serializer* serializer, const struct mip_filter_vertical_gyro_constraint_control_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    insert_u8(serializer, self->enable);
}

void extract_mip_filter_vertical_gyro_constraint_control_command(struct mip_serializer* serializer, struct mip_filter_vertical_gyro_constraint_control_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    extract_u8(serializer, &self->enable);
}

void insert_mip_filter_gnss_antenna_cal_control_command(struct mip_serializer* serializer, const struct mip_filter_gnss_antenna_cal_control_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    insert_u8(serializer, self->enable);
    insert_float(serializer, self->max_offset);
}

void extract_mip_filter_gnss_antenna_cal_control_command(struct mip_serializer* serializer, struct mip_filter_gnss_antenna_cal_control_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    extract_u8(serializer, &self->enable);
    extract_float(serializer, &self->max_offset);
}

void insert_mip_filter_magnetic_declination_source_command(struct mip_serializer* serializer, const struct mip_filter_magnetic_declination_source_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    insert_mip_filter_mag_declination_source(serializer, self->source);
    insert_float(serializer, self->declination);
}

void extract_mip_filter_magnetic_declination_source_command(struct mip_serializer* serializer, struct mip_filter_magnetic_declination_source_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    extract_mip_filter_mag_declination_source(serializer, &self->source);
    extract_float(serializer, &self->declination);
}

void insert_mip_filter_set_initial_heading_command(struct mip_serializer* serializer, const struct mip_filter_set_initial_heading_command* self)
{
    insert_float(serializer, self->heading);
}

void extract_mip_filter_set_initial_heading_command(struct mip_serializer* serializer, struct mip_filter_set_initial_heading_command* self)
{
    extract_float(serializer, &self->heading);
}


#ifdef __cplusplus
} // namespace C
} // namespace mscl
} // extern "C"
#endif // __cplusplus

