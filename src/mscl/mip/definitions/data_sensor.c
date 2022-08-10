
#include "data_sensor.h"

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


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

void insert_mip_sensor_raw_accel_data(struct mip_serializer* serializer, const struct mip_sensor_raw_accel_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->raw_accel[i]);
}

void extract_mip_sensor_raw_accel_data(struct mip_serializer* serializer, struct mip_sensor_raw_accel_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->raw_accel[i]);
}

void insert_mip_sensor_raw_gyro_data(struct mip_serializer* serializer, const struct mip_sensor_raw_gyro_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->raw_gyro[i]);
}

void extract_mip_sensor_raw_gyro_data(struct mip_serializer* serializer, struct mip_sensor_raw_gyro_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->raw_gyro[i]);
}

void insert_mip_sensor_raw_mag_data(struct mip_serializer* serializer, const struct mip_sensor_raw_mag_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->raw_mag[i]);
}

void extract_mip_sensor_raw_mag_data(struct mip_serializer* serializer, struct mip_sensor_raw_mag_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->raw_mag[i]);
}

void insert_mip_sensor_raw_pressure_data(struct mip_serializer* serializer, const struct mip_sensor_raw_pressure_data* self)
{
    insert_float(serializer, self->raw_pressure);
}

void extract_mip_sensor_raw_pressure_data(struct mip_serializer* serializer, struct mip_sensor_raw_pressure_data* self)
{
    extract_float(serializer, &self->raw_pressure);
}

void insert_mip_sensor_scaled_accel_data(struct mip_serializer* serializer, const struct mip_sensor_scaled_accel_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->scaled_accel[i]);
}

void extract_mip_sensor_scaled_accel_data(struct mip_serializer* serializer, struct mip_sensor_scaled_accel_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->scaled_accel[i]);
}

void insert_mip_sensor_scaled_gyro_data(struct mip_serializer* serializer, const struct mip_sensor_scaled_gyro_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->scaled_gyro[i]);
}

void extract_mip_sensor_scaled_gyro_data(struct mip_serializer* serializer, struct mip_sensor_scaled_gyro_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->scaled_gyro[i]);
}

void insert_mip_sensor_scaled_mag_data(struct mip_serializer* serializer, const struct mip_sensor_scaled_mag_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->scaled_mag[i]);
}

void extract_mip_sensor_scaled_mag_data(struct mip_serializer* serializer, struct mip_sensor_scaled_mag_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->scaled_mag[i]);
}

void insert_mip_sensor_scaled_pressure_data(struct mip_serializer* serializer, const struct mip_sensor_scaled_pressure_data* self)
{
    insert_float(serializer, self->scaled_pressure);
}

void extract_mip_sensor_scaled_pressure_data(struct mip_serializer* serializer, struct mip_sensor_scaled_pressure_data* self)
{
    extract_float(serializer, &self->scaled_pressure);
}

void insert_mip_sensor_delta_theta_data(struct mip_serializer* serializer, const struct mip_sensor_delta_theta_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->delta_theta[i]);
}

void extract_mip_sensor_delta_theta_data(struct mip_serializer* serializer, struct mip_sensor_delta_theta_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->delta_theta[i]);
}

void insert_mip_sensor_delta_velocity_data(struct mip_serializer* serializer, const struct mip_sensor_delta_velocity_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->delta_velocity[i]);
}

void extract_mip_sensor_delta_velocity_data(struct mip_serializer* serializer, struct mip_sensor_delta_velocity_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->delta_velocity[i]);
}

void insert_mip_sensor_comp_orientation_matrix_data(struct mip_serializer* serializer, const struct mip_sensor_comp_orientation_matrix_data* self)
{
    for(unsigned int i=0; i < 9; i++)
        insert_float(serializer, self->m[i]);
}

void extract_mip_sensor_comp_orientation_matrix_data(struct mip_serializer* serializer, struct mip_sensor_comp_orientation_matrix_data* self)
{
    for(unsigned int i=0; i < 9; i++)
        extract_float(serializer, &self->m[i]);
}

void insert_mip_sensor_comp_quaternion_data(struct mip_serializer* serializer, const struct mip_sensor_comp_quaternion_data* self)
{
    for(unsigned int i=0; i < 4; i++)
        insert_float(serializer, self->q[i]);
}

void extract_mip_sensor_comp_quaternion_data(struct mip_serializer* serializer, struct mip_sensor_comp_quaternion_data* self)
{
    for(unsigned int i=0; i < 4; i++)
        extract_float(serializer, &self->q[i]);
}

void insert_mip_sensor_comp_euler_angles_data(struct mip_serializer* serializer, const struct mip_sensor_comp_euler_angles_data* self)
{
    insert_float(serializer, self->roll);
    insert_float(serializer, self->pitch);
    insert_float(serializer, self->yaw);
}

void extract_mip_sensor_comp_euler_angles_data(struct mip_serializer* serializer, struct mip_sensor_comp_euler_angles_data* self)
{
    extract_float(serializer, &self->roll);
    extract_float(serializer, &self->pitch);
    extract_float(serializer, &self->yaw);
}

void insert_mip_sensor_comp_orientation_update_matrix_data(struct mip_serializer* serializer, const struct mip_sensor_comp_orientation_update_matrix_data* self)
{
    for(unsigned int i=0; i < 9; i++)
        insert_float(serializer, self->m[i]);
}

void extract_mip_sensor_comp_orientation_update_matrix_data(struct mip_serializer* serializer, struct mip_sensor_comp_orientation_update_matrix_data* self)
{
    for(unsigned int i=0; i < 9; i++)
        extract_float(serializer, &self->m[i]);
}

void insert_mip_sensor_orientation_raw_temp_data(struct mip_serializer* serializer, const struct mip_sensor_orientation_raw_temp_data* self)
{
    for(unsigned int i=0; i < 4; i++)
        insert_u16(serializer, self->raw_temp[i]);
}

void extract_mip_sensor_orientation_raw_temp_data(struct mip_serializer* serializer, struct mip_sensor_orientation_raw_temp_data* self)
{
    for(unsigned int i=0; i < 4; i++)
        extract_u16(serializer, &self->raw_temp[i]);
}

void insert_mip_sensor_internal_timestamp_data(struct mip_serializer* serializer, const struct mip_sensor_internal_timestamp_data* self)
{
    insert_u32(serializer, self->counts);
}

void extract_mip_sensor_internal_timestamp_data(struct mip_serializer* serializer, struct mip_sensor_internal_timestamp_data* self)
{
    extract_u32(serializer, &self->counts);
}

void insert_mip_sensor_pps_timestamp_data(struct mip_serializer* serializer, const struct mip_sensor_pps_timestamp_data* self)
{
    insert_u32(serializer, self->seconds);
    insert_u32(serializer, self->useconds);
}

void extract_mip_sensor_pps_timestamp_data(struct mip_serializer* serializer, struct mip_sensor_pps_timestamp_data* self)
{
    extract_u32(serializer, &self->seconds);
    extract_u32(serializer, &self->useconds);
}

void insert_mip_sensor_gps_timestamp_data(struct mip_serializer* serializer, const struct mip_sensor_gps_timestamp_data* self)
{
    insert_double(serializer, self->tow);
    insert_u16(serializer, self->week_number);
    insert_mip_sensor_gps_timestamp_data_valid_flags(serializer, self->valid_flags);
}

void extract_mip_sensor_gps_timestamp_data(struct mip_serializer* serializer, struct mip_sensor_gps_timestamp_data* self)
{
    extract_double(serializer, &self->tow);
    extract_u16(serializer, &self->week_number);
    extract_mip_sensor_gps_timestamp_data_valid_flags(serializer, &self->valid_flags);
}

void insert_mip_sensor_gps_timestamp_data_valid_flags(struct mip_serializer* serializer, const enum mip_sensor_gps_timestamp_data_valid_flags self)
{
    return insert_u16(serializer, (uint16_t)(self));
}
void extract_mip_sensor_gps_timestamp_data_valid_flags(struct mip_serializer* serializer, enum mip_sensor_gps_timestamp_data_valid_flags* self)
{
    uint16_t tmp = 0;
    extract_u16(serializer, &tmp);
    *self = tmp;
}

void insert_mip_sensor_temperature_abs_data(struct mip_serializer* serializer, const struct mip_sensor_temperature_abs_data* self)
{
    insert_float(serializer, self->min_temp);
    insert_float(serializer, self->max_temp);
    insert_float(serializer, self->mean_temp);
}

void extract_mip_sensor_temperature_abs_data(struct mip_serializer* serializer, struct mip_sensor_temperature_abs_data* self)
{
    extract_float(serializer, &self->min_temp);
    extract_float(serializer, &self->max_temp);
    extract_float(serializer, &self->mean_temp);
}

void insert_mip_sensor_up_vector_data(struct mip_serializer* serializer, const struct mip_sensor_up_vector_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->up[i]);
}

void extract_mip_sensor_up_vector_data(struct mip_serializer* serializer, struct mip_sensor_up_vector_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->up[i]);
}

void insert_mip_sensor_north_vector_data(struct mip_serializer* serializer, const struct mip_sensor_north_vector_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->north[i]);
}

void extract_mip_sensor_north_vector_data(struct mip_serializer* serializer, struct mip_sensor_north_vector_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->north[i]);
}

void insert_mip_sensor_overrange_status_data(struct mip_serializer* serializer, const struct mip_sensor_overrange_status_data* self)
{
    insert_mip_sensor_overrange_status_data_status(serializer, self->status);
}

void extract_mip_sensor_overrange_status_data(struct mip_serializer* serializer, struct mip_sensor_overrange_status_data* self)
{
    extract_mip_sensor_overrange_status_data_status(serializer, &self->status);
}

void insert_mip_sensor_overrange_status_data_status(struct mip_serializer* serializer, const enum mip_sensor_overrange_status_data_status self)
{
    return insert_u16(serializer, (uint16_t)(self));
}
void extract_mip_sensor_overrange_status_data_status(struct mip_serializer* serializer, enum mip_sensor_overrange_status_data_status* self)
{
    uint16_t tmp = 0;
    extract_u16(serializer, &tmp);
    *self = tmp;
}

void insert_mip_sensor_odometer_data_data(struct mip_serializer* serializer, const struct mip_sensor_odometer_data_data* self)
{
    insert_float(serializer, self->speed);
    insert_float(serializer, self->uncertainty);
    insert_u16(serializer, self->valid_flags);
}

void extract_mip_sensor_odometer_data_data(struct mip_serializer* serializer, struct mip_sensor_odometer_data_data* self)
{
    extract_float(serializer, &self->speed);
    extract_float(serializer, &self->uncertainty);
    extract_u16(serializer, &self->valid_flags);
}


#ifdef __cplusplus
} // namespace C
} // namespace mscl
} // extern "C"
#endif // __cplusplus

