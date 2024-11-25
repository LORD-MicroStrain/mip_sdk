
#include "data_filter.h"

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

void insert_mip_filter_position_llh_data(microstrain_serializer* serializer, const mip_filter_position_llh_data* self)
{
    microstrain_insert_double(serializer, self->latitude);
    
    microstrain_insert_double(serializer, self->longitude);
    
    microstrain_insert_double(serializer, self->ellipsoid_height);
    
    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_position_llh_data(microstrain_serializer* serializer, mip_filter_position_llh_data* self)
{
    microstrain_extract_double(serializer, &self->latitude);
    
    microstrain_extract_double(serializer, &self->longitude);
    
    microstrain_extract_double(serializer, &self->ellipsoid_height);
    
    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_position_llh_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_filter_position_llh_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_filter_position_llh_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_velocity_ned_data(microstrain_serializer* serializer, const mip_filter_velocity_ned_data* self)
{
    microstrain_insert_float(serializer, self->north);
    
    microstrain_insert_float(serializer, self->east);
    
    microstrain_insert_float(serializer, self->down);
    
    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_velocity_ned_data(microstrain_serializer* serializer, mip_filter_velocity_ned_data* self)
{
    microstrain_extract_float(serializer, &self->north);
    
    microstrain_extract_float(serializer, &self->east);
    
    microstrain_extract_float(serializer, &self->down);
    
    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_velocity_ned_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_filter_velocity_ned_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_filter_velocity_ned_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_attitude_quaternion_data(microstrain_serializer* serializer, const mip_filter_attitude_quaternion_data* self)
{
    insert_mip_quatf(serializer, self->q);
    
    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_attitude_quaternion_data(microstrain_serializer* serializer, mip_filter_attitude_quaternion_data* self)
{
    extract_mip_quatf(serializer, self->q);
    
    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_attitude_quaternion_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_filter_attitude_quaternion_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_filter_attitude_quaternion_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_attitude_dcm_data(microstrain_serializer* serializer, const mip_filter_attitude_dcm_data* self)
{
    insert_mip_matrix3f(serializer, self->dcm);
    
    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_attitude_dcm_data(microstrain_serializer* serializer, mip_filter_attitude_dcm_data* self)
{
    extract_mip_matrix3f(serializer, self->dcm);
    
    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_attitude_dcm_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_filter_attitude_dcm_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_filter_attitude_dcm_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_euler_angles_data(microstrain_serializer* serializer, const mip_filter_euler_angles_data* self)
{
    microstrain_insert_float(serializer, self->roll);
    
    microstrain_insert_float(serializer, self->pitch);
    
    microstrain_insert_float(serializer, self->yaw);
    
    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_euler_angles_data(microstrain_serializer* serializer, mip_filter_euler_angles_data* self)
{
    microstrain_extract_float(serializer, &self->roll);
    
    microstrain_extract_float(serializer, &self->pitch);
    
    microstrain_extract_float(serializer, &self->yaw);
    
    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_euler_angles_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_filter_euler_angles_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_filter_euler_angles_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_gyro_bias_data(microstrain_serializer* serializer, const mip_filter_gyro_bias_data* self)
{
    insert_mip_vector3f(serializer, self->bias);
    
    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_gyro_bias_data(microstrain_serializer* serializer, mip_filter_gyro_bias_data* self)
{
    extract_mip_vector3f(serializer, self->bias);
    
    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_gyro_bias_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_filter_gyro_bias_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_filter_gyro_bias_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_accel_bias_data(microstrain_serializer* serializer, const mip_filter_accel_bias_data* self)
{
    insert_mip_vector3f(serializer, self->bias);
    
    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_accel_bias_data(microstrain_serializer* serializer, mip_filter_accel_bias_data* self)
{
    extract_mip_vector3f(serializer, self->bias);
    
    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_accel_bias_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_filter_accel_bias_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_filter_accel_bias_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_position_llh_uncertainty_data(microstrain_serializer* serializer, const mip_filter_position_llh_uncertainty_data* self)
{
    microstrain_insert_float(serializer, self->north);
    
    microstrain_insert_float(serializer, self->east);
    
    microstrain_insert_float(serializer, self->down);
    
    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_position_llh_uncertainty_data(microstrain_serializer* serializer, mip_filter_position_llh_uncertainty_data* self)
{
    microstrain_extract_float(serializer, &self->north);
    
    microstrain_extract_float(serializer, &self->east);
    
    microstrain_extract_float(serializer, &self->down);
    
    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_position_llh_uncertainty_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_filter_position_llh_uncertainty_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_filter_position_llh_uncertainty_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_velocity_ned_uncertainty_data(microstrain_serializer* serializer, const mip_filter_velocity_ned_uncertainty_data* self)
{
    microstrain_insert_float(serializer, self->north);
    
    microstrain_insert_float(serializer, self->east);
    
    microstrain_insert_float(serializer, self->down);
    
    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_velocity_ned_uncertainty_data(microstrain_serializer* serializer, mip_filter_velocity_ned_uncertainty_data* self)
{
    microstrain_extract_float(serializer, &self->north);
    
    microstrain_extract_float(serializer, &self->east);
    
    microstrain_extract_float(serializer, &self->down);
    
    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_velocity_ned_uncertainty_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_filter_velocity_ned_uncertainty_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_filter_velocity_ned_uncertainty_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_euler_angles_uncertainty_data(microstrain_serializer* serializer, const mip_filter_euler_angles_uncertainty_data* self)
{
    microstrain_insert_float(serializer, self->roll);
    
    microstrain_insert_float(serializer, self->pitch);
    
    microstrain_insert_float(serializer, self->yaw);
    
    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_euler_angles_uncertainty_data(microstrain_serializer* serializer, mip_filter_euler_angles_uncertainty_data* self)
{
    microstrain_extract_float(serializer, &self->roll);
    
    microstrain_extract_float(serializer, &self->pitch);
    
    microstrain_extract_float(serializer, &self->yaw);
    
    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_euler_angles_uncertainty_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_filter_euler_angles_uncertainty_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_filter_euler_angles_uncertainty_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_gyro_bias_uncertainty_data(microstrain_serializer* serializer, const mip_filter_gyro_bias_uncertainty_data* self)
{
    insert_mip_vector3f(serializer, self->bias_uncert);
    
    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_gyro_bias_uncertainty_data(microstrain_serializer* serializer, mip_filter_gyro_bias_uncertainty_data* self)
{
    extract_mip_vector3f(serializer, self->bias_uncert);
    
    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_gyro_bias_uncertainty_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_filter_gyro_bias_uncertainty_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_filter_gyro_bias_uncertainty_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_accel_bias_uncertainty_data(microstrain_serializer* serializer, const mip_filter_accel_bias_uncertainty_data* self)
{
    insert_mip_vector3f(serializer, self->bias_uncert);
    
    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_accel_bias_uncertainty_data(microstrain_serializer* serializer, mip_filter_accel_bias_uncertainty_data* self)
{
    extract_mip_vector3f(serializer, self->bias_uncert);
    
    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_accel_bias_uncertainty_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_filter_accel_bias_uncertainty_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_filter_accel_bias_uncertainty_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_timestamp_data(microstrain_serializer* serializer, const mip_filter_timestamp_data* self)
{
    microstrain_insert_double(serializer, self->tow);
    
    microstrain_insert_u16(serializer, self->week_number);
    
    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_timestamp_data(microstrain_serializer* serializer, mip_filter_timestamp_data* self)
{
    microstrain_extract_double(serializer, &self->tow);
    
    microstrain_extract_u16(serializer, &self->week_number);
    
    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_timestamp_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_filter_timestamp_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_filter_timestamp_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_status_data(microstrain_serializer* serializer, const mip_filter_status_data* self)
{
    insert_mip_filter_mode(serializer, self->filter_state);
    
    insert_mip_filter_dynamics_mode(serializer, self->dynamics_mode);
    
    insert_mip_filter_status_flags(serializer, self->status_flags);
    
}
void extract_mip_filter_status_data(microstrain_serializer* serializer, mip_filter_status_data* self)
{
    extract_mip_filter_mode(serializer, &self->filter_state);
    
    extract_mip_filter_dynamics_mode(serializer, &self->dynamics_mode);
    
    extract_mip_filter_status_flags(serializer, &self->status_flags);
    
}
bool extract_mip_filter_status_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_filter_status_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_filter_status_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_linear_accel_data(microstrain_serializer* serializer, const mip_filter_linear_accel_data* self)
{
    insert_mip_vector3f(serializer, self->accel);
    
    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_linear_accel_data(microstrain_serializer* serializer, mip_filter_linear_accel_data* self)
{
    extract_mip_vector3f(serializer, self->accel);
    
    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_linear_accel_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_filter_linear_accel_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_filter_linear_accel_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_gravity_vector_data(microstrain_serializer* serializer, const mip_filter_gravity_vector_data* self)
{
    insert_mip_vector3f(serializer, self->gravity);
    
    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_gravity_vector_data(microstrain_serializer* serializer, mip_filter_gravity_vector_data* self)
{
    extract_mip_vector3f(serializer, self->gravity);
    
    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_gravity_vector_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_filter_gravity_vector_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_filter_gravity_vector_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_comp_accel_data(microstrain_serializer* serializer, const mip_filter_comp_accel_data* self)
{
    insert_mip_vector3f(serializer, self->accel);
    
    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_comp_accel_data(microstrain_serializer* serializer, mip_filter_comp_accel_data* self)
{
    extract_mip_vector3f(serializer, self->accel);
    
    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_comp_accel_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_filter_comp_accel_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_filter_comp_accel_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_comp_angular_rate_data(microstrain_serializer* serializer, const mip_filter_comp_angular_rate_data* self)
{
    insert_mip_vector3f(serializer, self->gyro);
    
    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_comp_angular_rate_data(microstrain_serializer* serializer, mip_filter_comp_angular_rate_data* self)
{
    extract_mip_vector3f(serializer, self->gyro);
    
    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_comp_angular_rate_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_filter_comp_angular_rate_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_filter_comp_angular_rate_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_quaternion_attitude_uncertainty_data(microstrain_serializer* serializer, const mip_filter_quaternion_attitude_uncertainty_data* self)
{
    insert_mip_quatf(serializer, self->q);
    
    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_quaternion_attitude_uncertainty_data(microstrain_serializer* serializer, mip_filter_quaternion_attitude_uncertainty_data* self)
{
    extract_mip_quatf(serializer, self->q);
    
    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_quaternion_attitude_uncertainty_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_filter_quaternion_attitude_uncertainty_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_filter_quaternion_attitude_uncertainty_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_wgs84_gravity_mag_data(microstrain_serializer* serializer, const mip_filter_wgs84_gravity_mag_data* self)
{
    microstrain_insert_float(serializer, self->magnitude);
    
    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_wgs84_gravity_mag_data(microstrain_serializer* serializer, mip_filter_wgs84_gravity_mag_data* self)
{
    microstrain_extract_float(serializer, &self->magnitude);
    
    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_wgs84_gravity_mag_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_filter_wgs84_gravity_mag_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_filter_wgs84_gravity_mag_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_heading_update_state_data(microstrain_serializer* serializer, const mip_filter_heading_update_state_data* self)
{
    microstrain_insert_float(serializer, self->heading);
    
    microstrain_insert_float(serializer, self->heading_1sigma);
    
    insert_mip_filter_heading_update_state_data_heading_source(serializer, self->source);
    
    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_heading_update_state_data(microstrain_serializer* serializer, mip_filter_heading_update_state_data* self)
{
    microstrain_extract_float(serializer, &self->heading);
    
    microstrain_extract_float(serializer, &self->heading_1sigma);
    
    extract_mip_filter_heading_update_state_data_heading_source(serializer, &self->source);
    
    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_heading_update_state_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_filter_heading_update_state_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_filter_heading_update_state_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_magnetic_model_data(microstrain_serializer* serializer, const mip_filter_magnetic_model_data* self)
{
    microstrain_insert_float(serializer, self->intensity_north);
    
    microstrain_insert_float(serializer, self->intensity_east);
    
    microstrain_insert_float(serializer, self->intensity_down);
    
    microstrain_insert_float(serializer, self->inclination);
    
    microstrain_insert_float(serializer, self->declination);
    
    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_magnetic_model_data(microstrain_serializer* serializer, mip_filter_magnetic_model_data* self)
{
    microstrain_extract_float(serializer, &self->intensity_north);
    
    microstrain_extract_float(serializer, &self->intensity_east);
    
    microstrain_extract_float(serializer, &self->intensity_down);
    
    microstrain_extract_float(serializer, &self->inclination);
    
    microstrain_extract_float(serializer, &self->declination);
    
    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_magnetic_model_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_filter_magnetic_model_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_filter_magnetic_model_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_accel_scale_factor_data(microstrain_serializer* serializer, const mip_filter_accel_scale_factor_data* self)
{
    insert_mip_vector3f(serializer, self->scale_factor);
    
    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_accel_scale_factor_data(microstrain_serializer* serializer, mip_filter_accel_scale_factor_data* self)
{
    extract_mip_vector3f(serializer, self->scale_factor);
    
    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_accel_scale_factor_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_filter_accel_scale_factor_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_filter_accel_scale_factor_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_accel_scale_factor_uncertainty_data(microstrain_serializer* serializer, const mip_filter_accel_scale_factor_uncertainty_data* self)
{
    insert_mip_vector3f(serializer, self->scale_factor_uncert);
    
    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_accel_scale_factor_uncertainty_data(microstrain_serializer* serializer, mip_filter_accel_scale_factor_uncertainty_data* self)
{
    extract_mip_vector3f(serializer, self->scale_factor_uncert);
    
    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_accel_scale_factor_uncertainty_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_filter_accel_scale_factor_uncertainty_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_filter_accel_scale_factor_uncertainty_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_gyro_scale_factor_data(microstrain_serializer* serializer, const mip_filter_gyro_scale_factor_data* self)
{
    insert_mip_vector3f(serializer, self->scale_factor);
    
    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_gyro_scale_factor_data(microstrain_serializer* serializer, mip_filter_gyro_scale_factor_data* self)
{
    extract_mip_vector3f(serializer, self->scale_factor);
    
    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_gyro_scale_factor_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_filter_gyro_scale_factor_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_filter_gyro_scale_factor_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_gyro_scale_factor_uncertainty_data(microstrain_serializer* serializer, const mip_filter_gyro_scale_factor_uncertainty_data* self)
{
    insert_mip_vector3f(serializer, self->scale_factor_uncert);
    
    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_gyro_scale_factor_uncertainty_data(microstrain_serializer* serializer, mip_filter_gyro_scale_factor_uncertainty_data* self)
{
    extract_mip_vector3f(serializer, self->scale_factor_uncert);
    
    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_gyro_scale_factor_uncertainty_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_filter_gyro_scale_factor_uncertainty_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_filter_gyro_scale_factor_uncertainty_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_mag_bias_data(microstrain_serializer* serializer, const mip_filter_mag_bias_data* self)
{
    insert_mip_vector3f(serializer, self->bias);
    
    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_mag_bias_data(microstrain_serializer* serializer, mip_filter_mag_bias_data* self)
{
    extract_mip_vector3f(serializer, self->bias);
    
    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_mag_bias_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_filter_mag_bias_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_filter_mag_bias_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_mag_bias_uncertainty_data(microstrain_serializer* serializer, const mip_filter_mag_bias_uncertainty_data* self)
{
    insert_mip_vector3f(serializer, self->bias_uncert);
    
    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_mag_bias_uncertainty_data(microstrain_serializer* serializer, mip_filter_mag_bias_uncertainty_data* self)
{
    extract_mip_vector3f(serializer, self->bias_uncert);
    
    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_mag_bias_uncertainty_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_filter_mag_bias_uncertainty_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_filter_mag_bias_uncertainty_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_standard_atmosphere_data(microstrain_serializer* serializer, const mip_filter_standard_atmosphere_data* self)
{
    microstrain_insert_float(serializer, self->geometric_altitude);
    
    microstrain_insert_float(serializer, self->geopotential_altitude);
    
    microstrain_insert_float(serializer, self->standard_temperature);
    
    microstrain_insert_float(serializer, self->standard_pressure);
    
    microstrain_insert_float(serializer, self->standard_density);
    
    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_standard_atmosphere_data(microstrain_serializer* serializer, mip_filter_standard_atmosphere_data* self)
{
    microstrain_extract_float(serializer, &self->geometric_altitude);
    
    microstrain_extract_float(serializer, &self->geopotential_altitude);
    
    microstrain_extract_float(serializer, &self->standard_temperature);
    
    microstrain_extract_float(serializer, &self->standard_pressure);
    
    microstrain_extract_float(serializer, &self->standard_density);
    
    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_standard_atmosphere_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_filter_standard_atmosphere_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_filter_standard_atmosphere_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_pressure_altitude_data(microstrain_serializer* serializer, const mip_filter_pressure_altitude_data* self)
{
    microstrain_insert_float(serializer, self->pressure_altitude);
    
    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_pressure_altitude_data(microstrain_serializer* serializer, mip_filter_pressure_altitude_data* self)
{
    microstrain_extract_float(serializer, &self->pressure_altitude);
    
    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_pressure_altitude_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_filter_pressure_altitude_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_filter_pressure_altitude_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_density_altitude_data(microstrain_serializer* serializer, const mip_filter_density_altitude_data* self)
{
    microstrain_insert_float(serializer, self->density_altitude);
    
    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_density_altitude_data(microstrain_serializer* serializer, mip_filter_density_altitude_data* self)
{
    microstrain_extract_float(serializer, &self->density_altitude);
    
    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_density_altitude_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_filter_density_altitude_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_filter_density_altitude_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_antenna_offset_correction_data(microstrain_serializer* serializer, const mip_filter_antenna_offset_correction_data* self)
{
    insert_mip_vector3f(serializer, self->offset);
    
    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_antenna_offset_correction_data(microstrain_serializer* serializer, mip_filter_antenna_offset_correction_data* self)
{
    extract_mip_vector3f(serializer, self->offset);
    
    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_antenna_offset_correction_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_filter_antenna_offset_correction_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_filter_antenna_offset_correction_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_antenna_offset_correction_uncertainty_data(microstrain_serializer* serializer, const mip_filter_antenna_offset_correction_uncertainty_data* self)
{
    insert_mip_vector3f(serializer, self->offset_uncert);
    
    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_antenna_offset_correction_uncertainty_data(microstrain_serializer* serializer, mip_filter_antenna_offset_correction_uncertainty_data* self)
{
    extract_mip_vector3f(serializer, self->offset_uncert);
    
    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_antenna_offset_correction_uncertainty_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_filter_antenna_offset_correction_uncertainty_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_filter_antenna_offset_correction_uncertainty_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_multi_antenna_offset_correction_data(microstrain_serializer* serializer, const mip_filter_multi_antenna_offset_correction_data* self)
{
    microstrain_insert_u8(serializer, self->receiver_id);
    
    insert_mip_vector3f(serializer, self->offset);
    
    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_multi_antenna_offset_correction_data(microstrain_serializer* serializer, mip_filter_multi_antenna_offset_correction_data* self)
{
    microstrain_extract_u8(serializer, &self->receiver_id);
    
    extract_mip_vector3f(serializer, self->offset);
    
    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_multi_antenna_offset_correction_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_filter_multi_antenna_offset_correction_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_filter_multi_antenna_offset_correction_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_multi_antenna_offset_correction_uncertainty_data(microstrain_serializer* serializer, const mip_filter_multi_antenna_offset_correction_uncertainty_data* self)
{
    microstrain_insert_u8(serializer, self->receiver_id);
    
    insert_mip_vector3f(serializer, self->offset_uncert);
    
    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_multi_antenna_offset_correction_uncertainty_data(microstrain_serializer* serializer, mip_filter_multi_antenna_offset_correction_uncertainty_data* self)
{
    microstrain_extract_u8(serializer, &self->receiver_id);
    
    extract_mip_vector3f(serializer, self->offset_uncert);
    
    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_multi_antenna_offset_correction_uncertainty_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_filter_multi_antenna_offset_correction_uncertainty_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_filter_multi_antenna_offset_correction_uncertainty_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_magnetometer_offset_data(microstrain_serializer* serializer, const mip_filter_magnetometer_offset_data* self)
{
    insert_mip_vector3f(serializer, self->hard_iron);
    
    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_magnetometer_offset_data(microstrain_serializer* serializer, mip_filter_magnetometer_offset_data* self)
{
    extract_mip_vector3f(serializer, self->hard_iron);
    
    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_magnetometer_offset_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_filter_magnetometer_offset_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_filter_magnetometer_offset_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_magnetometer_matrix_data(microstrain_serializer* serializer, const mip_filter_magnetometer_matrix_data* self)
{
    insert_mip_matrix3f(serializer, self->soft_iron);
    
    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_magnetometer_matrix_data(microstrain_serializer* serializer, mip_filter_magnetometer_matrix_data* self)
{
    extract_mip_matrix3f(serializer, self->soft_iron);
    
    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_magnetometer_matrix_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_filter_magnetometer_matrix_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_filter_magnetometer_matrix_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_magnetometer_offset_uncertainty_data(microstrain_serializer* serializer, const mip_filter_magnetometer_offset_uncertainty_data* self)
{
    insert_mip_vector3f(serializer, self->hard_iron_uncertainty);
    
    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_magnetometer_offset_uncertainty_data(microstrain_serializer* serializer, mip_filter_magnetometer_offset_uncertainty_data* self)
{
    extract_mip_vector3f(serializer, self->hard_iron_uncertainty);
    
    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_magnetometer_offset_uncertainty_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_filter_magnetometer_offset_uncertainty_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_filter_magnetometer_offset_uncertainty_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_magnetometer_matrix_uncertainty_data(microstrain_serializer* serializer, const mip_filter_magnetometer_matrix_uncertainty_data* self)
{
    insert_mip_matrix3f(serializer, self->soft_iron_uncertainty);
    
    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_magnetometer_matrix_uncertainty_data(microstrain_serializer* serializer, mip_filter_magnetometer_matrix_uncertainty_data* self)
{
    extract_mip_matrix3f(serializer, self->soft_iron_uncertainty);
    
    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_magnetometer_matrix_uncertainty_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_filter_magnetometer_matrix_uncertainty_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_filter_magnetometer_matrix_uncertainty_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_magnetometer_covariance_matrix_data(microstrain_serializer* serializer, const mip_filter_magnetometer_covariance_matrix_data* self)
{
    insert_mip_matrix3f(serializer, self->covariance);
    
    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_magnetometer_covariance_matrix_data(microstrain_serializer* serializer, mip_filter_magnetometer_covariance_matrix_data* self)
{
    extract_mip_matrix3f(serializer, self->covariance);
    
    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_magnetometer_covariance_matrix_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_filter_magnetometer_covariance_matrix_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_filter_magnetometer_covariance_matrix_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_magnetometer_residual_vector_data(microstrain_serializer* serializer, const mip_filter_magnetometer_residual_vector_data* self)
{
    insert_mip_vector3f(serializer, self->residual);
    
    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_magnetometer_residual_vector_data(microstrain_serializer* serializer, mip_filter_magnetometer_residual_vector_data* self)
{
    extract_mip_vector3f(serializer, self->residual);
    
    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_magnetometer_residual_vector_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_filter_magnetometer_residual_vector_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_filter_magnetometer_residual_vector_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_clock_correction_data(microstrain_serializer* serializer, const mip_filter_clock_correction_data* self)
{
    microstrain_insert_u8(serializer, self->receiver_id);
    
    microstrain_insert_float(serializer, self->bias);
    
    microstrain_insert_float(serializer, self->bias_drift);
    
    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_clock_correction_data(microstrain_serializer* serializer, mip_filter_clock_correction_data* self)
{
    microstrain_extract_u8(serializer, &self->receiver_id);
    
    microstrain_extract_float(serializer, &self->bias);
    
    microstrain_extract_float(serializer, &self->bias_drift);
    
    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_clock_correction_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_filter_clock_correction_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_filter_clock_correction_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_clock_correction_uncertainty_data(microstrain_serializer* serializer, const mip_filter_clock_correction_uncertainty_data* self)
{
    microstrain_insert_u8(serializer, self->receiver_id);
    
    microstrain_insert_float(serializer, self->bias_uncertainty);
    
    microstrain_insert_float(serializer, self->bias_drift_uncertainty);
    
    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_clock_correction_uncertainty_data(microstrain_serializer* serializer, mip_filter_clock_correction_uncertainty_data* self)
{
    microstrain_extract_u8(serializer, &self->receiver_id);
    
    microstrain_extract_float(serializer, &self->bias_uncertainty);
    
    microstrain_extract_float(serializer, &self->bias_drift_uncertainty);
    
    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_clock_correction_uncertainty_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_filter_clock_correction_uncertainty_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_filter_clock_correction_uncertainty_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_gnss_pos_aid_status_data(microstrain_serializer* serializer, const mip_filter_gnss_pos_aid_status_data* self)
{
    microstrain_insert_u8(serializer, self->receiver_id);
    
    microstrain_insert_float(serializer, self->time_of_week);
    
    insert_mip_gnss_aid_status_flags(serializer, self->status);
    
    for(unsigned int i=0; i < 8; i++)
        microstrain_insert_u8(serializer, self->reserved[i]);
    
}
void extract_mip_filter_gnss_pos_aid_status_data(microstrain_serializer* serializer, mip_filter_gnss_pos_aid_status_data* self)
{
    microstrain_extract_u8(serializer, &self->receiver_id);
    
    microstrain_extract_float(serializer, &self->time_of_week);
    
    extract_mip_gnss_aid_status_flags(serializer, &self->status);
    
    for(unsigned int i=0; i < 8; i++)
        microstrain_extract_u8(serializer, &self->reserved[i]);
    
}
bool extract_mip_filter_gnss_pos_aid_status_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_filter_gnss_pos_aid_status_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_filter_gnss_pos_aid_status_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_gnss_att_aid_status_data(microstrain_serializer* serializer, const mip_filter_gnss_att_aid_status_data* self)
{
    microstrain_insert_float(serializer, self->time_of_week);
    
    insert_mip_gnss_aid_status_flags(serializer, self->status);
    
    for(unsigned int i=0; i < 8; i++)
        microstrain_insert_u8(serializer, self->reserved[i]);
    
}
void extract_mip_filter_gnss_att_aid_status_data(microstrain_serializer* serializer, mip_filter_gnss_att_aid_status_data* self)
{
    microstrain_extract_float(serializer, &self->time_of_week);
    
    extract_mip_gnss_aid_status_flags(serializer, &self->status);
    
    for(unsigned int i=0; i < 8; i++)
        microstrain_extract_u8(serializer, &self->reserved[i]);
    
}
bool extract_mip_filter_gnss_att_aid_status_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_filter_gnss_att_aid_status_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_filter_gnss_att_aid_status_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_head_aid_status_data(microstrain_serializer* serializer, const mip_filter_head_aid_status_data* self)
{
    microstrain_insert_float(serializer, self->time_of_week);
    
    insert_mip_filter_head_aid_status_data_heading_aid_type(serializer, self->type);
    
    for(unsigned int i=0; i < 2; i++)
        microstrain_insert_float(serializer, self->reserved[i]);
    
}
void extract_mip_filter_head_aid_status_data(microstrain_serializer* serializer, mip_filter_head_aid_status_data* self)
{
    microstrain_extract_float(serializer, &self->time_of_week);
    
    extract_mip_filter_head_aid_status_data_heading_aid_type(serializer, &self->type);
    
    for(unsigned int i=0; i < 2; i++)
        microstrain_extract_float(serializer, &self->reserved[i]);
    
}
bool extract_mip_filter_head_aid_status_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_filter_head_aid_status_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_filter_head_aid_status_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_rel_pos_ned_data(microstrain_serializer* serializer, const mip_filter_rel_pos_ned_data* self)
{
    insert_mip_vector3d(serializer, self->relative_position);
    
    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_rel_pos_ned_data(microstrain_serializer* serializer, mip_filter_rel_pos_ned_data* self)
{
    extract_mip_vector3d(serializer, self->relative_position);
    
    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_rel_pos_ned_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_filter_rel_pos_ned_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_filter_rel_pos_ned_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_ecef_pos_data(microstrain_serializer* serializer, const mip_filter_ecef_pos_data* self)
{
    insert_mip_vector3d(serializer, self->position_ecef);
    
    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_ecef_pos_data(microstrain_serializer* serializer, mip_filter_ecef_pos_data* self)
{
    extract_mip_vector3d(serializer, self->position_ecef);
    
    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_ecef_pos_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_filter_ecef_pos_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_filter_ecef_pos_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_ecef_vel_data(microstrain_serializer* serializer, const mip_filter_ecef_vel_data* self)
{
    insert_mip_vector3f(serializer, self->velocity_ecef);
    
    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_ecef_vel_data(microstrain_serializer* serializer, mip_filter_ecef_vel_data* self)
{
    extract_mip_vector3f(serializer, self->velocity_ecef);
    
    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_ecef_vel_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_filter_ecef_vel_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_filter_ecef_vel_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_ecef_pos_uncertainty_data(microstrain_serializer* serializer, const mip_filter_ecef_pos_uncertainty_data* self)
{
    insert_mip_vector3f(serializer, self->pos_uncertainty);
    
    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_ecef_pos_uncertainty_data(microstrain_serializer* serializer, mip_filter_ecef_pos_uncertainty_data* self)
{
    extract_mip_vector3f(serializer, self->pos_uncertainty);
    
    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_ecef_pos_uncertainty_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_filter_ecef_pos_uncertainty_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_filter_ecef_pos_uncertainty_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_ecef_vel_uncertainty_data(microstrain_serializer* serializer, const mip_filter_ecef_vel_uncertainty_data* self)
{
    insert_mip_vector3f(serializer, self->vel_uncertainty);
    
    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_ecef_vel_uncertainty_data(microstrain_serializer* serializer, mip_filter_ecef_vel_uncertainty_data* self)
{
    extract_mip_vector3f(serializer, self->vel_uncertainty);
    
    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_ecef_vel_uncertainty_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_filter_ecef_vel_uncertainty_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_filter_ecef_vel_uncertainty_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_aiding_measurement_summary_data(microstrain_serializer* serializer, const mip_filter_aiding_measurement_summary_data* self)
{
    microstrain_insert_float(serializer, self->time_of_week);
    
    microstrain_insert_u8(serializer, self->source);
    
    insert_mip_filter_aiding_measurement_type(serializer, self->type);
    
    insert_mip_filter_measurement_indicator(serializer, self->indicator);
    
}
void extract_mip_filter_aiding_measurement_summary_data(microstrain_serializer* serializer, mip_filter_aiding_measurement_summary_data* self)
{
    microstrain_extract_float(serializer, &self->time_of_week);
    
    microstrain_extract_u8(serializer, &self->source);
    
    extract_mip_filter_aiding_measurement_type(serializer, &self->type);
    
    extract_mip_filter_measurement_indicator(serializer, &self->indicator);
    
}
bool extract_mip_filter_aiding_measurement_summary_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_filter_aiding_measurement_summary_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_filter_aiding_measurement_summary_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_odometer_scale_factor_error_data(microstrain_serializer* serializer, const mip_filter_odometer_scale_factor_error_data* self)
{
    microstrain_insert_float(serializer, self->scale_factor_error);
    
    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_odometer_scale_factor_error_data(microstrain_serializer* serializer, mip_filter_odometer_scale_factor_error_data* self)
{
    microstrain_extract_float(serializer, &self->scale_factor_error);
    
    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_odometer_scale_factor_error_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_filter_odometer_scale_factor_error_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_filter_odometer_scale_factor_error_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_odometer_scale_factor_error_uncertainty_data(microstrain_serializer* serializer, const mip_filter_odometer_scale_factor_error_uncertainty_data* self)
{
    microstrain_insert_float(serializer, self->scale_factor_error_uncertainty);
    
    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_odometer_scale_factor_error_uncertainty_data(microstrain_serializer* serializer, mip_filter_odometer_scale_factor_error_uncertainty_data* self)
{
    microstrain_extract_float(serializer, &self->scale_factor_error_uncertainty);
    
    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_odometer_scale_factor_error_uncertainty_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_filter_odometer_scale_factor_error_uncertainty_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_filter_odometer_scale_factor_error_uncertainty_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_gnss_dual_antenna_status_data(microstrain_serializer* serializer, const mip_filter_gnss_dual_antenna_status_data* self)
{
    microstrain_insert_float(serializer, self->time_of_week);
    
    microstrain_insert_float(serializer, self->heading);
    
    microstrain_insert_float(serializer, self->heading_unc);
    
    insert_mip_filter_gnss_dual_antenna_status_data_fix_type(serializer, self->fix_type);
    
    insert_mip_filter_gnss_dual_antenna_status_data_dual_antenna_status_flags(serializer, self->status_flags);
    
    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_gnss_dual_antenna_status_data(microstrain_serializer* serializer, mip_filter_gnss_dual_antenna_status_data* self)
{
    microstrain_extract_float(serializer, &self->time_of_week);
    
    microstrain_extract_float(serializer, &self->heading);
    
    microstrain_extract_float(serializer, &self->heading_unc);
    
    extract_mip_filter_gnss_dual_antenna_status_data_fix_type(serializer, &self->fix_type);
    
    extract_mip_filter_gnss_dual_antenna_status_data_dual_antenna_status_flags(serializer, &self->status_flags);
    
    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_gnss_dual_antenna_status_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_filter_gnss_dual_antenna_status_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_filter_gnss_dual_antenna_status_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_aiding_frame_config_error_data(microstrain_serializer* serializer, const mip_filter_aiding_frame_config_error_data* self)
{
    microstrain_insert_u8(serializer, self->frame_id);
    
    insert_mip_vector3f(serializer, self->translation);
    
    insert_mip_quatf(serializer, self->attitude);
    
}
void extract_mip_filter_aiding_frame_config_error_data(microstrain_serializer* serializer, mip_filter_aiding_frame_config_error_data* self)
{
    microstrain_extract_u8(serializer, &self->frame_id);
    
    extract_mip_vector3f(serializer, self->translation);
    
    extract_mip_quatf(serializer, self->attitude);
    
}
bool extract_mip_filter_aiding_frame_config_error_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_filter_aiding_frame_config_error_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_filter_aiding_frame_config_error_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_aiding_frame_config_error_uncertainty_data(microstrain_serializer* serializer, const mip_filter_aiding_frame_config_error_uncertainty_data* self)
{
    microstrain_insert_u8(serializer, self->frame_id);
    
    insert_mip_vector3f(serializer, self->translation_unc);
    
    insert_mip_vector3f(serializer, self->attitude_unc);
    
}
void extract_mip_filter_aiding_frame_config_error_uncertainty_data(microstrain_serializer* serializer, mip_filter_aiding_frame_config_error_uncertainty_data* self)
{
    microstrain_extract_u8(serializer, &self->frame_id);
    
    extract_mip_vector3f(serializer, self->translation_unc);
    
    extract_mip_vector3f(serializer, self->attitude_unc);
    
}
bool extract_mip_filter_aiding_frame_config_error_uncertainty_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_filter_aiding_frame_config_error_uncertainty_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_filter_aiding_frame_config_error_uncertainty_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}


#ifdef __cplusplus
} // extern "C"
} // namespace C
} // namespace mip
#endif // __cplusplus

