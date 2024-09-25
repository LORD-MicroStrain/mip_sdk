
#include "data_sensor.h"

#include <mip/mip_serialization.h>
#include <mip/mip_interface.h>

#include <assert.h>


#ifdef __cplusplus
namespace mip {
namespace C {
extern "C" {

#endif // __cplusplus


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

void insert_mip_sensor_raw_accel_data(microstrain_serializer* serializer, const mip_sensor_raw_accel_data* self)
{
    insert_mip_vector3f(serializer, self->raw_accel);
    
}
void extract_mip_sensor_raw_accel_data(microstrain_serializer* serializer, mip_sensor_raw_accel_data* self)
{
    extract_mip_vector3f(serializer, self->raw_accel);
    
}
bool extract_mip_sensor_raw_accel_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_sensor_raw_accel_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_sensor_raw_accel_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_sensor_raw_gyro_data(microstrain_serializer* serializer, const mip_sensor_raw_gyro_data* self)
{
    insert_mip_vector3f(serializer, self->raw_gyro);
    
}
void extract_mip_sensor_raw_gyro_data(microstrain_serializer* serializer, mip_sensor_raw_gyro_data* self)
{
    extract_mip_vector3f(serializer, self->raw_gyro);
    
}
bool extract_mip_sensor_raw_gyro_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_sensor_raw_gyro_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_sensor_raw_gyro_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_sensor_raw_mag_data(microstrain_serializer* serializer, const mip_sensor_raw_mag_data* self)
{
    insert_mip_vector3f(serializer, self->raw_mag);
    
}
void extract_mip_sensor_raw_mag_data(microstrain_serializer* serializer, mip_sensor_raw_mag_data* self)
{
    extract_mip_vector3f(serializer, self->raw_mag);
    
}
bool extract_mip_sensor_raw_mag_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_sensor_raw_mag_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_sensor_raw_mag_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_sensor_raw_pressure_data(microstrain_serializer* serializer, const mip_sensor_raw_pressure_data* self)
{
    microstrain_insert_float(serializer, self->raw_pressure);
    
}
void extract_mip_sensor_raw_pressure_data(microstrain_serializer* serializer, mip_sensor_raw_pressure_data* self)
{
    microstrain_extract_float(serializer, &self->raw_pressure);
    
}
bool extract_mip_sensor_raw_pressure_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_sensor_raw_pressure_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_sensor_raw_pressure_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_sensor_scaled_accel_data(microstrain_serializer* serializer, const mip_sensor_scaled_accel_data* self)
{
    insert_mip_vector3f(serializer, self->scaled_accel);
    
}
void extract_mip_sensor_scaled_accel_data(microstrain_serializer* serializer, mip_sensor_scaled_accel_data* self)
{
    extract_mip_vector3f(serializer, self->scaled_accel);
    
}
bool extract_mip_sensor_scaled_accel_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_sensor_scaled_accel_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_sensor_scaled_accel_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_sensor_scaled_gyro_data(microstrain_serializer* serializer, const mip_sensor_scaled_gyro_data* self)
{
    insert_mip_vector3f(serializer, self->scaled_gyro);
    
}
void extract_mip_sensor_scaled_gyro_data(microstrain_serializer* serializer, mip_sensor_scaled_gyro_data* self)
{
    extract_mip_vector3f(serializer, self->scaled_gyro);
    
}
bool extract_mip_sensor_scaled_gyro_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_sensor_scaled_gyro_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_sensor_scaled_gyro_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_sensor_scaled_mag_data(microstrain_serializer* serializer, const mip_sensor_scaled_mag_data* self)
{
    insert_mip_vector3f(serializer, self->scaled_mag);
    
}
void extract_mip_sensor_scaled_mag_data(microstrain_serializer* serializer, mip_sensor_scaled_mag_data* self)
{
    extract_mip_vector3f(serializer, self->scaled_mag);
    
}
bool extract_mip_sensor_scaled_mag_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_sensor_scaled_mag_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_sensor_scaled_mag_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_sensor_scaled_pressure_data(microstrain_serializer* serializer, const mip_sensor_scaled_pressure_data* self)
{
    microstrain_insert_float(serializer, self->scaled_pressure);
    
}
void extract_mip_sensor_scaled_pressure_data(microstrain_serializer* serializer, mip_sensor_scaled_pressure_data* self)
{
    microstrain_extract_float(serializer, &self->scaled_pressure);
    
}
bool extract_mip_sensor_scaled_pressure_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_sensor_scaled_pressure_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_sensor_scaled_pressure_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_sensor_delta_theta_data(microstrain_serializer* serializer, const mip_sensor_delta_theta_data* self)
{
    insert_mip_vector3f(serializer, self->delta_theta);
    
}
void extract_mip_sensor_delta_theta_data(microstrain_serializer* serializer, mip_sensor_delta_theta_data* self)
{
    extract_mip_vector3f(serializer, self->delta_theta);
    
}
bool extract_mip_sensor_delta_theta_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_sensor_delta_theta_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_sensor_delta_theta_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_sensor_delta_velocity_data(microstrain_serializer* serializer, const mip_sensor_delta_velocity_data* self)
{
    insert_mip_vector3f(serializer, self->delta_velocity);
    
}
void extract_mip_sensor_delta_velocity_data(microstrain_serializer* serializer, mip_sensor_delta_velocity_data* self)
{
    extract_mip_vector3f(serializer, self->delta_velocity);
    
}
bool extract_mip_sensor_delta_velocity_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_sensor_delta_velocity_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_sensor_delta_velocity_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_sensor_comp_orientation_matrix_data(microstrain_serializer* serializer, const mip_sensor_comp_orientation_matrix_data* self)
{
    insert_mip_matrix3f(serializer, self->m);
    
}
void extract_mip_sensor_comp_orientation_matrix_data(microstrain_serializer* serializer, mip_sensor_comp_orientation_matrix_data* self)
{
    extract_mip_matrix3f(serializer, self->m);
    
}
bool extract_mip_sensor_comp_orientation_matrix_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_sensor_comp_orientation_matrix_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_sensor_comp_orientation_matrix_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_sensor_comp_quaternion_data(microstrain_serializer* serializer, const mip_sensor_comp_quaternion_data* self)
{
    insert_mip_quatf(serializer, self->q);
    
}
void extract_mip_sensor_comp_quaternion_data(microstrain_serializer* serializer, mip_sensor_comp_quaternion_data* self)
{
    extract_mip_quatf(serializer, self->q);
    
}
bool extract_mip_sensor_comp_quaternion_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_sensor_comp_quaternion_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_sensor_comp_quaternion_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_sensor_comp_euler_angles_data(microstrain_serializer* serializer, const mip_sensor_comp_euler_angles_data* self)
{
    microstrain_insert_float(serializer, self->roll);
    
    microstrain_insert_float(serializer, self->pitch);
    
    microstrain_insert_float(serializer, self->yaw);
    
}
void extract_mip_sensor_comp_euler_angles_data(microstrain_serializer* serializer, mip_sensor_comp_euler_angles_data* self)
{
    microstrain_extract_float(serializer, &self->roll);
    
    microstrain_extract_float(serializer, &self->pitch);
    
    microstrain_extract_float(serializer, &self->yaw);
    
}
bool extract_mip_sensor_comp_euler_angles_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_sensor_comp_euler_angles_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_sensor_comp_euler_angles_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_sensor_comp_orientation_update_matrix_data(microstrain_serializer* serializer, const mip_sensor_comp_orientation_update_matrix_data* self)
{
    insert_mip_matrix3f(serializer, self->m);
    
}
void extract_mip_sensor_comp_orientation_update_matrix_data(microstrain_serializer* serializer, mip_sensor_comp_orientation_update_matrix_data* self)
{
    extract_mip_matrix3f(serializer, self->m);
    
}
bool extract_mip_sensor_comp_orientation_update_matrix_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_sensor_comp_orientation_update_matrix_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_sensor_comp_orientation_update_matrix_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_sensor_orientation_raw_temp_data(microstrain_serializer* serializer, const mip_sensor_orientation_raw_temp_data* self)
{
    for(unsigned int i=0; i < 4; i++)
        microstrain_insert_u16(serializer, self->raw_temp[i]);
    
}
void extract_mip_sensor_orientation_raw_temp_data(microstrain_serializer* serializer, mip_sensor_orientation_raw_temp_data* self)
{
    for(unsigned int i=0; i < 4; i++)
        microstrain_extract_u16(serializer, &self->raw_temp[i]);
    
}
bool extract_mip_sensor_orientation_raw_temp_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_sensor_orientation_raw_temp_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_sensor_orientation_raw_temp_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_sensor_internal_timestamp_data(microstrain_serializer* serializer, const mip_sensor_internal_timestamp_data* self)
{
    microstrain_insert_u32(serializer, self->counts);
    
}
void extract_mip_sensor_internal_timestamp_data(microstrain_serializer* serializer, mip_sensor_internal_timestamp_data* self)
{
    microstrain_extract_u32(serializer, &self->counts);
    
}
bool extract_mip_sensor_internal_timestamp_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_sensor_internal_timestamp_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_sensor_internal_timestamp_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_sensor_pps_timestamp_data(microstrain_serializer* serializer, const mip_sensor_pps_timestamp_data* self)
{
    microstrain_insert_u32(serializer, self->seconds);
    
    microstrain_insert_u32(serializer, self->useconds);
    
}
void extract_mip_sensor_pps_timestamp_data(microstrain_serializer* serializer, mip_sensor_pps_timestamp_data* self)
{
    microstrain_extract_u32(serializer, &self->seconds);
    
    microstrain_extract_u32(serializer, &self->useconds);
    
}
bool extract_mip_sensor_pps_timestamp_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_sensor_pps_timestamp_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_sensor_pps_timestamp_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_sensor_gps_timestamp_data(microstrain_serializer* serializer, const mip_sensor_gps_timestamp_data* self)
{
    microstrain_insert_double(serializer, self->tow);
    
    microstrain_insert_u16(serializer, self->week_number);
    
    insert_mip_sensor_gps_timestamp_data_valid_flags(serializer, self->valid_flags);
    
}
void extract_mip_sensor_gps_timestamp_data(microstrain_serializer* serializer, mip_sensor_gps_timestamp_data* self)
{
    microstrain_extract_double(serializer, &self->tow);
    
    microstrain_extract_u16(serializer, &self->week_number);
    
    extract_mip_sensor_gps_timestamp_data_valid_flags(serializer, &self->valid_flags);
    
}
bool extract_mip_sensor_gps_timestamp_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_sensor_gps_timestamp_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_sensor_gps_timestamp_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_sensor_temperature_abs_data(microstrain_serializer* serializer, const mip_sensor_temperature_abs_data* self)
{
    microstrain_insert_float(serializer, self->min_temp);
    
    microstrain_insert_float(serializer, self->max_temp);
    
    microstrain_insert_float(serializer, self->mean_temp);
    
}
void extract_mip_sensor_temperature_abs_data(microstrain_serializer* serializer, mip_sensor_temperature_abs_data* self)
{
    microstrain_extract_float(serializer, &self->min_temp);
    
    microstrain_extract_float(serializer, &self->max_temp);
    
    microstrain_extract_float(serializer, &self->mean_temp);
    
}
bool extract_mip_sensor_temperature_abs_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_sensor_temperature_abs_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_sensor_temperature_abs_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_sensor_up_vector_data(microstrain_serializer* serializer, const mip_sensor_up_vector_data* self)
{
    insert_mip_vector3f(serializer, self->up);
    
}
void extract_mip_sensor_up_vector_data(microstrain_serializer* serializer, mip_sensor_up_vector_data* self)
{
    extract_mip_vector3f(serializer, self->up);
    
}
bool extract_mip_sensor_up_vector_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_sensor_up_vector_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_sensor_up_vector_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_sensor_north_vector_data(microstrain_serializer* serializer, const mip_sensor_north_vector_data* self)
{
    insert_mip_vector3f(serializer, self->north);
    
}
void extract_mip_sensor_north_vector_data(microstrain_serializer* serializer, mip_sensor_north_vector_data* self)
{
    extract_mip_vector3f(serializer, self->north);
    
}
bool extract_mip_sensor_north_vector_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_sensor_north_vector_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_sensor_north_vector_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_sensor_overrange_status_data(microstrain_serializer* serializer, const mip_sensor_overrange_status_data* self)
{
    insert_mip_sensor_overrange_status_data_status(serializer, self->status);
    
}
void extract_mip_sensor_overrange_status_data(microstrain_serializer* serializer, mip_sensor_overrange_status_data* self)
{
    extract_mip_sensor_overrange_status_data_status(serializer, &self->status);
    
}
bool extract_mip_sensor_overrange_status_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_sensor_overrange_status_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_sensor_overrange_status_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_sensor_odometer_data_data(microstrain_serializer* serializer, const mip_sensor_odometer_data_data* self)
{
    microstrain_insert_float(serializer, self->speed);
    
    microstrain_insert_float(serializer, self->uncertainty);
    
    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_sensor_odometer_data_data(microstrain_serializer* serializer, mip_sensor_odometer_data_data* self)
{
    microstrain_extract_float(serializer, &self->speed);
    
    microstrain_extract_float(serializer, &self->uncertainty);
    
    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_sensor_odometer_data_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_sensor_odometer_data_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_sensor_odometer_data_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}


#ifdef __cplusplus
} // extern "C"
} // namespace C
} // namespace mip
#endif // __cplusplus

