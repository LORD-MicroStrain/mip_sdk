
#include "data_sensor.h"

#include "../../utils/serialization.h"
#include "../mip_interface.h"

#include <assert.h>


#ifdef __cplusplus
namespace mscl {
extern "C" {
#endif // __cplusplus


////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_sensor_raw_accel_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_sensor_raw_accel_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->raw_accel[i]);
    
    return offset;
}

size_t extract_mip_sensor_raw_accel_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_sensor_raw_accel_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->raw_accel[i]);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_sensor_raw_gyro_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_sensor_raw_gyro_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->raw_gyro[i]);
    
    return offset;
}

size_t extract_mip_sensor_raw_gyro_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_sensor_raw_gyro_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->raw_gyro[i]);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_sensor_raw_mag_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_sensor_raw_mag_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->raw_mag[i]);
    
    return offset;
}

size_t extract_mip_sensor_raw_mag_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_sensor_raw_mag_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->raw_mag[i]);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_sensor_raw_pressure_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_sensor_raw_pressure_data* self)
{
    offset = insert_float(buffer, bufferSize, offset, self->raw_pressure);
    
    return offset;
}

size_t extract_mip_sensor_raw_pressure_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_sensor_raw_pressure_data* self)
{
    offset = extract_float(buffer, bufferSize, offset, &self->raw_pressure);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_sensor_scaled_accel_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_sensor_scaled_accel_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->scaled_accel[i]);
    
    return offset;
}

size_t extract_mip_sensor_scaled_accel_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_sensor_scaled_accel_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->scaled_accel[i]);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_sensor_scaled_gyro_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_sensor_scaled_gyro_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->scaled_gyro[i]);
    
    return offset;
}

size_t extract_mip_sensor_scaled_gyro_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_sensor_scaled_gyro_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->scaled_gyro[i]);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_sensor_scaled_mag_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_sensor_scaled_mag_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->scaled_mag[i]);
    
    return offset;
}

size_t extract_mip_sensor_scaled_mag_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_sensor_scaled_mag_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->scaled_mag[i]);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_sensor_scaled_pressure_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_sensor_scaled_pressure_data* self)
{
    offset = insert_float(buffer, bufferSize, offset, self->scaled_pressure);
    
    return offset;
}

size_t extract_mip_sensor_scaled_pressure_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_sensor_scaled_pressure_data* self)
{
    offset = extract_float(buffer, bufferSize, offset, &self->scaled_pressure);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_sensor_delta_theta_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_sensor_delta_theta_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->delta_theta[i]);
    
    return offset;
}

size_t extract_mip_sensor_delta_theta_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_sensor_delta_theta_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->delta_theta[i]);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_sensor_delta_velocity_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_sensor_delta_velocity_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->delta_velocity[i]);
    
    return offset;
}

size_t extract_mip_sensor_delta_velocity_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_sensor_delta_velocity_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->delta_velocity[i]);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_sensor_comp_orientation_matrix_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_sensor_comp_orientation_matrix_data* self)
{
    for(unsigned int i=0; i < 9; i++)
        offset = insert_float(buffer, bufferSize, offset, self->m[i]);
    
    return offset;
}

size_t extract_mip_sensor_comp_orientation_matrix_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_sensor_comp_orientation_matrix_data* self)
{
    for(unsigned int i=0; i < 9; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->m[i]);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_sensor_comp_quaternion_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_sensor_comp_quaternion_data* self)
{
    for(unsigned int i=0; i < 4; i++)
        offset = insert_float(buffer, bufferSize, offset, self->q[i]);
    
    return offset;
}

size_t extract_mip_sensor_comp_quaternion_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_sensor_comp_quaternion_data* self)
{
    for(unsigned int i=0; i < 4; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->q[i]);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_sensor_comp_euler_angles_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_sensor_comp_euler_angles_data* self)
{
    offset = insert_float(buffer, bufferSize, offset, self->roll);
    offset = insert_float(buffer, bufferSize, offset, self->pitch);
    offset = insert_float(buffer, bufferSize, offset, self->yaw);
    
    return offset;
}

size_t extract_mip_sensor_comp_euler_angles_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_sensor_comp_euler_angles_data* self)
{
    offset = extract_float(buffer, bufferSize, offset, &self->roll);
    offset = extract_float(buffer, bufferSize, offset, &self->pitch);
    offset = extract_float(buffer, bufferSize, offset, &self->yaw);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_sensor_comp_orientation_update_matrix_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_sensor_comp_orientation_update_matrix_data* self)
{
    for(unsigned int i=0; i < 9; i++)
        offset = insert_float(buffer, bufferSize, offset, self->m[i]);
    
    return offset;
}

size_t extract_mip_sensor_comp_orientation_update_matrix_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_sensor_comp_orientation_update_matrix_data* self)
{
    for(unsigned int i=0; i < 9; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->m[i]);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_sensor_orientation_raw_temp_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_sensor_orientation_raw_temp_data* self)
{
    for(unsigned int i=0; i < 4; i++)
        offset = insert_u16(buffer, bufferSize, offset, self->raw_temp[i]);
    
    return offset;
}

size_t extract_mip_sensor_orientation_raw_temp_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_sensor_orientation_raw_temp_data* self)
{
    for(unsigned int i=0; i < 4; i++)
        offset = extract_u16(buffer, bufferSize, offset, &self->raw_temp[i]);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_sensor_internal_timestamp_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_sensor_internal_timestamp_data* self)
{
    offset = insert_u32(buffer, bufferSize, offset, self->counts);
    
    return offset;
}

size_t extract_mip_sensor_internal_timestamp_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_sensor_internal_timestamp_data* self)
{
    offset = extract_u32(buffer, bufferSize, offset, &self->counts);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_sensor_1pps_timestamp_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_sensor_1pps_timestamp_data* self)
{
    offset = insert_u32(buffer, bufferSize, offset, self->seconds);
    offset = insert_u32(buffer, bufferSize, offset, self->useconds);
    
    return offset;
}

size_t extract_mip_sensor_1pps_timestamp_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_sensor_1pps_timestamp_data* self)
{
    offset = extract_u32(buffer, bufferSize, offset, &self->seconds);
    offset = extract_u32(buffer, bufferSize, offset, &self->useconds);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_sensor_gps_timestamp_data_valid_flags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_sensor_gps_timestamp_data_valid_flags self)
{
    return insert_u16(buffer, bufferSize, offset, self);
}
size_t extract_mip_sensor_gps_timestamp_data_valid_flags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_sensor_gps_timestamp_data_valid_flags* self)
{
    uint16_t tmp;
    offset = extract_u16(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_mip_sensor_gps_timestamp_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_sensor_gps_timestamp_data* self)
{
    offset = insert_double(buffer, bufferSize, offset, self->tow);
    offset = insert_u16(buffer, bufferSize, offset, self->week_number);
    offset = insert_mip_sensor_gps_timestamp_data_valid_flags(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_mip_sensor_gps_timestamp_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_sensor_gps_timestamp_data* self)
{
    offset = extract_double(buffer, bufferSize, offset, &self->tow);
    offset = extract_u16(buffer, bufferSize, offset, &self->week_number);
    offset = extract_mip_sensor_gps_timestamp_data_valid_flags(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_sensor_temperature_abs_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_sensor_temperature_abs_data* self)
{
    offset = insert_float(buffer, bufferSize, offset, self->min_temp);
    offset = insert_float(buffer, bufferSize, offset, self->max_temp);
    offset = insert_float(buffer, bufferSize, offset, self->mean_temp);
    
    return offset;
}

size_t extract_mip_sensor_temperature_abs_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_sensor_temperature_abs_data* self)
{
    offset = extract_float(buffer, bufferSize, offset, &self->min_temp);
    offset = extract_float(buffer, bufferSize, offset, &self->max_temp);
    offset = extract_float(buffer, bufferSize, offset, &self->mean_temp);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_sensor_up_vector_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_sensor_up_vector_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->up[i]);
    
    return offset;
}

size_t extract_mip_sensor_up_vector_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_sensor_up_vector_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->up[i]);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_sensor_north_vector_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_sensor_north_vector_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->north[i]);
    
    return offset;
}

size_t extract_mip_sensor_north_vector_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_sensor_north_vector_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->north[i]);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_sensor_overrange_status_data_status(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_sensor_overrange_status_data_status self)
{
    return insert_u16(buffer, bufferSize, offset, self);
}
size_t extract_mip_sensor_overrange_status_data_status(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_sensor_overrange_status_data_status* self)
{
    uint16_t tmp;
    offset = extract_u16(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_mip_sensor_overrange_status_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_sensor_overrange_status_data* self)
{
    offset = insert_mip_sensor_overrange_status_data_status(buffer, bufferSize, offset, self->status);
    
    return offset;
}

size_t extract_mip_sensor_overrange_status_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_sensor_overrange_status_data* self)
{
    offset = extract_mip_sensor_overrange_status_data_status(buffer, bufferSize, offset, &self->status);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_sensor_odometer_data_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_sensor_odometer_data_data* self)
{
    offset = insert_float(buffer, bufferSize, offset, self->speed);
    offset = insert_float(buffer, bufferSize, offset, self->uncertainty);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_mip_sensor_odometer_data_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_sensor_odometer_data_data* self)
{
    offset = extract_float(buffer, bufferSize, offset, &self->speed);
    offset = extract_float(buffer, bufferSize, offset, &self->uncertainty);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}



#ifdef __cplusplus
} // extern "C"
} // namespace mscl
#endif // __cplusplus
