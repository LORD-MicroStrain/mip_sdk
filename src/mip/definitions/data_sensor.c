
#include "data_sensor.h"

#include "microstrain/common/serialization.h"
#include "../mip_interface.h"

#include <assert.h>


#ifdef __cplusplus
namespace mip {
namespace C {
extern "C" {

#endif // __cplusplus
struct mip_interface;
struct microstrain_serializer;
struct mip_field;


////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

void insert_mip_sensor_raw_accel_data(microstrain_serializer* serializer, const mip_sensor_raw_accel_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        microstrain_insert_float(serializer, self->raw_accel[i]);
    
}
void extract_mip_sensor_raw_accel_data(microstrain_serializer* serializer, mip_sensor_raw_accel_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        microstrain_extract_float(serializer, &self->raw_accel[i]);
    
}
bool extract_mip_sensor_raw_accel_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_sensor_raw_accel_data* self = ptr;
    struct microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_sensor_raw_accel_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_sensor_raw_gyro_data(microstrain_serializer* serializer, const mip_sensor_raw_gyro_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        microstrain_insert_float(serializer, self->raw_gyro[i]);
    
}
void extract_mip_sensor_raw_gyro_data(microstrain_serializer* serializer, mip_sensor_raw_gyro_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        microstrain_extract_float(serializer, &self->raw_gyro[i]);
    
}
bool extract_mip_sensor_raw_gyro_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_sensor_raw_gyro_data* self = ptr;
    struct microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_sensor_raw_gyro_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_sensor_raw_mag_data(microstrain_serializer* serializer, const mip_sensor_raw_mag_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        microstrain_insert_float(serializer, self->raw_mag[i]);
    
}
void extract_mip_sensor_raw_mag_data(microstrain_serializer* serializer, mip_sensor_raw_mag_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        microstrain_extract_float(serializer, &self->raw_mag[i]);
    
}
bool extract_mip_sensor_raw_mag_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_sensor_raw_mag_data* self = ptr;
    struct microstrain_serializer serializer;
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
bool extract_mip_sensor_raw_pressure_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_sensor_raw_pressure_data* self = ptr;
    struct microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_sensor_raw_pressure_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_sensor_scaled_accel_data(microstrain_serializer* serializer, const mip_sensor_scaled_accel_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        microstrain_insert_float(serializer, self->scaled_accel[i]);
    
}
void extract_mip_sensor_scaled_accel_data(microstrain_serializer* serializer, mip_sensor_scaled_accel_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        microstrain_extract_float(serializer, &self->scaled_accel[i]);
    
}
bool extract_mip_sensor_scaled_accel_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_sensor_scaled_accel_data* self = ptr;
    struct microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_sensor_scaled_accel_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_sensor_scaled_gyro_data(microstrain_serializer* serializer, const mip_sensor_scaled_gyro_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        microstrain_insert_float(serializer, self->scaled_gyro[i]);
    
}
void extract_mip_sensor_scaled_gyro_data(microstrain_serializer* serializer, mip_sensor_scaled_gyro_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        microstrain_extract_float(serializer, &self->scaled_gyro[i]);
    
}
bool extract_mip_sensor_scaled_gyro_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_sensor_scaled_gyro_data* self = ptr;
    struct microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_sensor_scaled_gyro_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_sensor_scaled_mag_data(microstrain_serializer* serializer, const mip_sensor_scaled_mag_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        microstrain_insert_float(serializer, self->scaled_mag[i]);
    
}
void extract_mip_sensor_scaled_mag_data(microstrain_serializer* serializer, mip_sensor_scaled_mag_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        microstrain_extract_float(serializer, &self->scaled_mag[i]);
    
}
bool extract_mip_sensor_scaled_mag_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_sensor_scaled_mag_data* self = ptr;
    struct microstrain_serializer serializer;
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
bool extract_mip_sensor_scaled_pressure_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_sensor_scaled_pressure_data* self = ptr;
    struct microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_sensor_scaled_pressure_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_sensor_delta_theta_data(microstrain_serializer* serializer, const mip_sensor_delta_theta_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        microstrain_insert_float(serializer, self->delta_theta[i]);
    
}
void extract_mip_sensor_delta_theta_data(microstrain_serializer* serializer, mip_sensor_delta_theta_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        microstrain_extract_float(serializer, &self->delta_theta[i]);
    
}
bool extract_mip_sensor_delta_theta_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_sensor_delta_theta_data* self = ptr;
    struct microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_sensor_delta_theta_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_sensor_delta_velocity_data(microstrain_serializer* serializer, const mip_sensor_delta_velocity_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        microstrain_insert_float(serializer, self->delta_velocity[i]);
    
}
void extract_mip_sensor_delta_velocity_data(microstrain_serializer* serializer, mip_sensor_delta_velocity_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        microstrain_extract_float(serializer, &self->delta_velocity[i]);
    
}
bool extract_mip_sensor_delta_velocity_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_sensor_delta_velocity_data* self = ptr;
    struct microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_sensor_delta_velocity_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_sensor_comp_orientation_matrix_data(microstrain_serializer* serializer, const mip_sensor_comp_orientation_matrix_data* self)
{
    for(unsigned int i=0; i < 9; i++)
        microstrain_insert_float(serializer, self->m[i]);
    
}
void extract_mip_sensor_comp_orientation_matrix_data(microstrain_serializer* serializer, mip_sensor_comp_orientation_matrix_data* self)
{
    for(unsigned int i=0; i < 9; i++)
        microstrain_extract_float(serializer, &self->m[i]);
    
}
bool extract_mip_sensor_comp_orientation_matrix_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_sensor_comp_orientation_matrix_data* self = ptr;
    struct microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_sensor_comp_orientation_matrix_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_sensor_comp_quaternion_data(microstrain_serializer* serializer, const mip_sensor_comp_quaternion_data* self)
{
    for(unsigned int i=0; i < 4; i++)
        microstrain_insert_float(serializer, self->q[i]);
    
}
void extract_mip_sensor_comp_quaternion_data(microstrain_serializer* serializer, mip_sensor_comp_quaternion_data* self)
{
    for(unsigned int i=0; i < 4; i++)
        microstrain_extract_float(serializer, &self->q[i]);
    
}
bool extract_mip_sensor_comp_quaternion_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_sensor_comp_quaternion_data* self = ptr;
    struct microstrain_serializer serializer;
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
bool extract_mip_sensor_comp_euler_angles_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_sensor_comp_euler_angles_data* self = ptr;
    struct microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_sensor_comp_euler_angles_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_sensor_comp_orientation_update_matrix_data(microstrain_serializer* serializer, const mip_sensor_comp_orientation_update_matrix_data* self)
{
    for(unsigned int i=0; i < 9; i++)
        microstrain_insert_float(serializer, self->m[i]);
    
}
void extract_mip_sensor_comp_orientation_update_matrix_data(microstrain_serializer* serializer, mip_sensor_comp_orientation_update_matrix_data* self)
{
    for(unsigned int i=0; i < 9; i++)
        microstrain_extract_float(serializer, &self->m[i]);
    
}
bool extract_mip_sensor_comp_orientation_update_matrix_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_sensor_comp_orientation_update_matrix_data* self = ptr;
    struct microstrain_serializer serializer;
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
bool extract_mip_sensor_orientation_raw_temp_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_sensor_orientation_raw_temp_data* self = ptr;
    struct microstrain_serializer serializer;
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
bool extract_mip_sensor_internal_timestamp_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_sensor_internal_timestamp_data* self = ptr;
    struct microstrain_serializer serializer;
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
bool extract_mip_sensor_pps_timestamp_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_sensor_pps_timestamp_data* self = ptr;
    struct microstrain_serializer serializer;
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
bool extract_mip_sensor_gps_timestamp_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_sensor_gps_timestamp_data* self = ptr;
    struct microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_sensor_gps_timestamp_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_sensor_gps_timestamp_data_valid_flags(microstrain_serializer* serializer, const mip_sensor_gps_timestamp_data_valid_flags self)
{
    microstrain_insert_u16(serializer, (uint16_t) (self));
}
void extract_mip_sensor_gps_timestamp_data_valid_flags(microstrain_serializer* serializer, mip_sensor_gps_timestamp_data_valid_flags* self)
{
    uint16_t tmp = 0;
    microstrain_extract_u16(serializer, &tmp);
    *self = tmp;
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
bool extract_mip_sensor_temperature_abs_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_sensor_temperature_abs_data* self = ptr;
    struct microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_sensor_temperature_abs_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_sensor_up_vector_data(microstrain_serializer* serializer, const mip_sensor_up_vector_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        microstrain_insert_float(serializer, self->up[i]);
    
}
void extract_mip_sensor_up_vector_data(microstrain_serializer* serializer, mip_sensor_up_vector_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        microstrain_extract_float(serializer, &self->up[i]);
    
}
bool extract_mip_sensor_up_vector_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_sensor_up_vector_data* self = ptr;
    struct microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_sensor_up_vector_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_sensor_north_vector_data(microstrain_serializer* serializer, const mip_sensor_north_vector_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        microstrain_insert_float(serializer, self->north[i]);
    
}
void extract_mip_sensor_north_vector_data(microstrain_serializer* serializer, mip_sensor_north_vector_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        microstrain_extract_float(serializer, &self->north[i]);
    
}
bool extract_mip_sensor_north_vector_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_sensor_north_vector_data* self = ptr;
    struct microstrain_serializer serializer;
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
bool extract_mip_sensor_overrange_status_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_sensor_overrange_status_data* self = ptr;
    struct microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_sensor_overrange_status_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_sensor_overrange_status_data_status(microstrain_serializer* serializer, const mip_sensor_overrange_status_data_status self)
{
    microstrain_insert_u16(serializer, (uint16_t) (self));
}
void extract_mip_sensor_overrange_status_data_status(microstrain_serializer* serializer, mip_sensor_overrange_status_data_status* self)
{
    uint16_t tmp = 0;
    microstrain_extract_u16(serializer, &tmp);
    *self = tmp;
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
bool extract_mip_sensor_odometer_data_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_sensor_odometer_data_data* self = ptr;
    struct microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_sensor_odometer_data_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}


#ifdef __cplusplus
} // namespace C
} // namespace mip
} // extern "C"
#endif // __cplusplus

