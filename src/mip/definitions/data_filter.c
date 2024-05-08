
#include "data_filter.h"

#include "microstrain/common/serialization.h"
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

void insert_mip_filter_mode(struct mip_serializer* serializer, const mip_filter_mode self)
{
    microstrain_insert_u16(serializer, (uint16_t) (self));
}
void extract_mip_filter_mode(struct mip_serializer* serializer, mip_filter_mode* self)
{
    uint16_t tmp = 0;
    microstrain_extract_u16(serializer, &tmp);
    *self = tmp;
}

void insert_mip_filter_dynamics_mode(struct mip_serializer* serializer, const mip_filter_dynamics_mode self)
{
    microstrain_insert_u16(serializer, (uint16_t) (self));
}
void extract_mip_filter_dynamics_mode(struct mip_serializer* serializer, mip_filter_dynamics_mode* self)
{
    uint16_t tmp = 0;
    microstrain_extract_u16(serializer, &tmp);
    *self = tmp;
}

void insert_mip_filter_status_flags(struct mip_serializer* serializer, const mip_filter_status_flags self)
{
    microstrain_insert_u16(serializer, (uint16_t) (self));
}
void extract_mip_filter_status_flags(struct mip_serializer* serializer, mip_filter_status_flags* self)
{
    uint16_t tmp = 0;
    microstrain_extract_u16(serializer, &tmp);
    *self = tmp;
}

void insert_mip_filter_aiding_measurement_type(struct mip_serializer* serializer, const mip_filter_aiding_measurement_type self)
{
    microstrain_insert_u8(serializer, (uint8_t) (self));
}
void extract_mip_filter_aiding_measurement_type(struct mip_serializer* serializer, mip_filter_aiding_measurement_type* self)
{
    uint8_t tmp = 0;
    microstrain_extract_u8(serializer, &tmp);
    *self = tmp;
}

void insert_mip_filter_measurement_indicator(struct mip_serializer* serializer, const mip_filter_measurement_indicator self)
{
    microstrain_insert_u8(serializer, (uint8_t) (self));
}
void extract_mip_filter_measurement_indicator(struct mip_serializer* serializer, mip_filter_measurement_indicator* self)
{
    uint8_t tmp = 0;
    microstrain_extract_u8(serializer, &tmp);
    *self = tmp;
}

void insert_mip_gnss_aid_status_flags(struct mip_serializer* serializer, const mip_gnss_aid_status_flags self)
{
    microstrain_insert_u16(serializer, (uint16_t) (self));
}
void extract_mip_gnss_aid_status_flags(struct mip_serializer* serializer, mip_gnss_aid_status_flags* self)
{
    uint16_t tmp = 0;
    microstrain_extract_u16(serializer, &tmp);
    *self = tmp;
}


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

void insert_mip_filter_position_llh_data(mip_serializer* serializer, const mip_filter_position_llh_data* self)
{
    microstrain_insert_double(serializer, self->latitude);

    microstrain_insert_double(serializer, self->longitude);

    microstrain_insert_double(serializer, self->ellipsoid_height);

    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_position_llh_data(mip_serializer* serializer, mip_filter_position_llh_data* self)
{
    microstrain_extract_double(serializer, &self->latitude);

    microstrain_extract_double(serializer, &self->longitude);

    microstrain_extract_double(serializer, &self->ellipsoid_height);

    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_position_llh_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_filter_position_llh_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_filter_position_llh_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_velocity_ned_data(mip_serializer* serializer, const mip_filter_velocity_ned_data* self)
{
    microstrain_insert_float(serializer, self->north);

    microstrain_insert_float(serializer, self->east);

    microstrain_insert_float(serializer, self->down);

    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_velocity_ned_data(mip_serializer* serializer, mip_filter_velocity_ned_data* self)
{
    microstrain_extract_float(serializer, &self->north);

    microstrain_extract_float(serializer, &self->east);

    microstrain_extract_float(serializer, &self->down);

    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_velocity_ned_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_filter_velocity_ned_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_filter_velocity_ned_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_attitude_quaternion_data(mip_serializer* serializer, const mip_filter_attitude_quaternion_data* self)
{
    for(unsigned int i=0; i < 4; i++)
        microstrain_insert_float(serializer, self->q[i]);

    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_attitude_quaternion_data(mip_serializer* serializer, mip_filter_attitude_quaternion_data* self)
{
    for(unsigned int i=0; i < 4; i++)
        microstrain_extract_float(serializer, &self->q[i]);

    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_attitude_quaternion_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_filter_attitude_quaternion_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_filter_attitude_quaternion_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_attitude_dcm_data(mip_serializer* serializer, const mip_filter_attitude_dcm_data* self)
{
    for(unsigned int i=0; i < 9; i++)
        microstrain_insert_float(serializer, self->dcm[i]);

    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_attitude_dcm_data(mip_serializer* serializer, mip_filter_attitude_dcm_data* self)
{
    for(unsigned int i=0; i < 9; i++)
        microstrain_extract_float(serializer, &self->dcm[i]);

    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_attitude_dcm_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_filter_attitude_dcm_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_filter_attitude_dcm_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_euler_angles_data(mip_serializer* serializer, const mip_filter_euler_angles_data* self)
{
    microstrain_insert_float(serializer, self->roll);

    microstrain_insert_float(serializer, self->pitch);

    microstrain_insert_float(serializer, self->yaw);

    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_euler_angles_data(mip_serializer* serializer, mip_filter_euler_angles_data* self)
{
    microstrain_extract_float(serializer, &self->roll);

    microstrain_extract_float(serializer, &self->pitch);

    microstrain_extract_float(serializer, &self->yaw);

    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_euler_angles_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_filter_euler_angles_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_filter_euler_angles_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_gyro_bias_data(mip_serializer* serializer, const mip_filter_gyro_bias_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        microstrain_insert_float(serializer, self->bias[i]);

    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_gyro_bias_data(mip_serializer* serializer, mip_filter_gyro_bias_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        microstrain_extract_float(serializer, &self->bias[i]);

    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_gyro_bias_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_filter_gyro_bias_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_filter_gyro_bias_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_accel_bias_data(mip_serializer* serializer, const mip_filter_accel_bias_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        microstrain_insert_float(serializer, self->bias[i]);

    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_accel_bias_data(mip_serializer* serializer, mip_filter_accel_bias_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        microstrain_extract_float(serializer, &self->bias[i]);

    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_accel_bias_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_filter_accel_bias_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_filter_accel_bias_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_position_llh_uncertainty_data(mip_serializer* serializer, const mip_filter_position_llh_uncertainty_data* self)
{
    microstrain_insert_float(serializer, self->north);

    microstrain_insert_float(serializer, self->east);

    microstrain_insert_float(serializer, self->down);

    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_position_llh_uncertainty_data(mip_serializer* serializer, mip_filter_position_llh_uncertainty_data* self)
{
    microstrain_extract_float(serializer, &self->north);

    microstrain_extract_float(serializer, &self->east);

    microstrain_extract_float(serializer, &self->down);

    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_position_llh_uncertainty_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_filter_position_llh_uncertainty_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_filter_position_llh_uncertainty_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_velocity_ned_uncertainty_data(mip_serializer* serializer, const mip_filter_velocity_ned_uncertainty_data* self)
{
    microstrain_insert_float(serializer, self->north);

    microstrain_insert_float(serializer, self->east);

    microstrain_insert_float(serializer, self->down);

    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_velocity_ned_uncertainty_data(mip_serializer* serializer, mip_filter_velocity_ned_uncertainty_data* self)
{
    microstrain_extract_float(serializer, &self->north);

    microstrain_extract_float(serializer, &self->east);

    microstrain_extract_float(serializer, &self->down);

    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_velocity_ned_uncertainty_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_filter_velocity_ned_uncertainty_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_filter_velocity_ned_uncertainty_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_euler_angles_uncertainty_data(mip_serializer* serializer, const mip_filter_euler_angles_uncertainty_data* self)
{
    microstrain_insert_float(serializer, self->roll);

    microstrain_insert_float(serializer, self->pitch);

    microstrain_insert_float(serializer, self->yaw);

    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_euler_angles_uncertainty_data(mip_serializer* serializer, mip_filter_euler_angles_uncertainty_data* self)
{
    microstrain_extract_float(serializer, &self->roll);

    microstrain_extract_float(serializer, &self->pitch);

    microstrain_extract_float(serializer, &self->yaw);

    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_euler_angles_uncertainty_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_filter_euler_angles_uncertainty_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_filter_euler_angles_uncertainty_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_gyro_bias_uncertainty_data(mip_serializer* serializer, const mip_filter_gyro_bias_uncertainty_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        microstrain_insert_float(serializer, self->bias_uncert[i]);

    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_gyro_bias_uncertainty_data(mip_serializer* serializer, mip_filter_gyro_bias_uncertainty_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        microstrain_extract_float(serializer, &self->bias_uncert[i]);

    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_gyro_bias_uncertainty_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_filter_gyro_bias_uncertainty_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_filter_gyro_bias_uncertainty_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_accel_bias_uncertainty_data(mip_serializer* serializer, const mip_filter_accel_bias_uncertainty_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        microstrain_insert_float(serializer, self->bias_uncert[i]);

    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_accel_bias_uncertainty_data(mip_serializer* serializer, mip_filter_accel_bias_uncertainty_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        microstrain_extract_float(serializer, &self->bias_uncert[i]);

    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_accel_bias_uncertainty_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_filter_accel_bias_uncertainty_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_filter_accel_bias_uncertainty_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_timestamp_data(mip_serializer* serializer, const mip_filter_timestamp_data* self)
{
    microstrain_insert_double(serializer, self->tow);

    microstrain_insert_u16(serializer, self->week_number);

    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_timestamp_data(mip_serializer* serializer, mip_filter_timestamp_data* self)
{
    microstrain_extract_double(serializer, &self->tow);

    microstrain_extract_u16(serializer, &self->week_number);

    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_timestamp_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_filter_timestamp_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_filter_timestamp_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_status_data(mip_serializer* serializer, const mip_filter_status_data* self)
{
    insert_mip_filter_mode(serializer, self->filter_state);
    
    insert_mip_filter_dynamics_mode(serializer, self->dynamics_mode);
    
    insert_mip_filter_status_flags(serializer, self->status_flags);
    
}
void extract_mip_filter_status_data(mip_serializer* serializer, mip_filter_status_data* self)
{
    extract_mip_filter_mode(serializer, &self->filter_state);
    
    extract_mip_filter_dynamics_mode(serializer, &self->dynamics_mode);
    
    extract_mip_filter_status_flags(serializer, &self->status_flags);
    
}
bool extract_mip_filter_status_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_filter_status_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_filter_status_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_linear_accel_data(mip_serializer* serializer, const mip_filter_linear_accel_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        microstrain_insert_float(serializer, self->accel[i]);

    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_linear_accel_data(mip_serializer* serializer, mip_filter_linear_accel_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        microstrain_extract_float(serializer, &self->accel[i]);

    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_linear_accel_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_filter_linear_accel_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_filter_linear_accel_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_gravity_vector_data(mip_serializer* serializer, const mip_filter_gravity_vector_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        microstrain_insert_float(serializer, self->gravity[i]);

    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_gravity_vector_data(mip_serializer* serializer, mip_filter_gravity_vector_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        microstrain_extract_float(serializer, &self->gravity[i]);

    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_gravity_vector_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_filter_gravity_vector_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_filter_gravity_vector_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_comp_accel_data(mip_serializer* serializer, const mip_filter_comp_accel_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        microstrain_insert_float(serializer, self->accel[i]);

    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_comp_accel_data(mip_serializer* serializer, mip_filter_comp_accel_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        microstrain_extract_float(serializer, &self->accel[i]);

    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_comp_accel_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_filter_comp_accel_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_filter_comp_accel_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_comp_angular_rate_data(mip_serializer* serializer, const mip_filter_comp_angular_rate_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        microstrain_insert_float(serializer, self->gyro[i]);

    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_comp_angular_rate_data(mip_serializer* serializer, mip_filter_comp_angular_rate_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        microstrain_extract_float(serializer, &self->gyro[i]);

    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_comp_angular_rate_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_filter_comp_angular_rate_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_filter_comp_angular_rate_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_quaternion_attitude_uncertainty_data(mip_serializer* serializer, const mip_filter_quaternion_attitude_uncertainty_data* self)
{
    for(unsigned int i=0; i < 4; i++)
        microstrain_insert_float(serializer, self->q[i]);

    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_quaternion_attitude_uncertainty_data(mip_serializer* serializer, mip_filter_quaternion_attitude_uncertainty_data* self)
{
    for(unsigned int i=0; i < 4; i++)
        microstrain_extract_float(serializer, &self->q[i]);

    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_quaternion_attitude_uncertainty_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_filter_quaternion_attitude_uncertainty_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_filter_quaternion_attitude_uncertainty_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_wgs84_gravity_mag_data(mip_serializer* serializer, const mip_filter_wgs84_gravity_mag_data* self)
{
    microstrain_insert_float(serializer, self->magnitude);

    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_wgs84_gravity_mag_data(mip_serializer* serializer, mip_filter_wgs84_gravity_mag_data* self)
{
    microstrain_extract_float(serializer, &self->magnitude);

    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_wgs84_gravity_mag_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_filter_wgs84_gravity_mag_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_filter_wgs84_gravity_mag_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_heading_update_state_data(mip_serializer* serializer, const mip_filter_heading_update_state_data* self)
{
    microstrain_insert_float(serializer, self->heading);

    microstrain_insert_float(serializer, self->heading_1sigma);
    
    insert_mip_filter_heading_update_state_data_heading_source(serializer, self->source);

    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_heading_update_state_data(mip_serializer* serializer, mip_filter_heading_update_state_data* self)
{
    microstrain_extract_float(serializer, &self->heading);

    microstrain_extract_float(serializer, &self->heading_1sigma);
    
    extract_mip_filter_heading_update_state_data_heading_source(serializer, &self->source);

    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_heading_update_state_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_filter_heading_update_state_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_filter_heading_update_state_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_heading_update_state_data_heading_source(struct mip_serializer* serializer, const mip_filter_heading_update_state_data_heading_source self)
{
    microstrain_insert_u16(serializer, (uint16_t) (self));
}
void extract_mip_filter_heading_update_state_data_heading_source(struct mip_serializer* serializer, mip_filter_heading_update_state_data_heading_source* self)
{
    uint16_t tmp = 0;
    microstrain_extract_u16(serializer, &tmp);
    *self = tmp;
}

void insert_mip_filter_magnetic_model_data(mip_serializer* serializer, const mip_filter_magnetic_model_data* self)
{
    microstrain_insert_float(serializer, self->intensity_north);

    microstrain_insert_float(serializer, self->intensity_east);

    microstrain_insert_float(serializer, self->intensity_down);

    microstrain_insert_float(serializer, self->inclination);

    microstrain_insert_float(serializer, self->declination);

    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_magnetic_model_data(mip_serializer* serializer, mip_filter_magnetic_model_data* self)
{
    microstrain_extract_float(serializer, &self->intensity_north);

    microstrain_extract_float(serializer, &self->intensity_east);

    microstrain_extract_float(serializer, &self->intensity_down);

    microstrain_extract_float(serializer, &self->inclination);

    microstrain_extract_float(serializer, &self->declination);

    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_magnetic_model_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_filter_magnetic_model_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_filter_magnetic_model_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_accel_scale_factor_data(mip_serializer* serializer, const mip_filter_accel_scale_factor_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        microstrain_insert_float(serializer, self->scale_factor[i]);

    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_accel_scale_factor_data(mip_serializer* serializer, mip_filter_accel_scale_factor_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        microstrain_extract_float(serializer, &self->scale_factor[i]);

    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_accel_scale_factor_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_filter_accel_scale_factor_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_filter_accel_scale_factor_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_accel_scale_factor_uncertainty_data(mip_serializer* serializer, const mip_filter_accel_scale_factor_uncertainty_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        microstrain_insert_float(serializer, self->scale_factor_uncert[i]);

    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_accel_scale_factor_uncertainty_data(mip_serializer* serializer, mip_filter_accel_scale_factor_uncertainty_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        microstrain_extract_float(serializer, &self->scale_factor_uncert[i]);

    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_accel_scale_factor_uncertainty_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_filter_accel_scale_factor_uncertainty_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_filter_accel_scale_factor_uncertainty_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_gyro_scale_factor_data(mip_serializer* serializer, const mip_filter_gyro_scale_factor_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        microstrain_insert_float(serializer, self->scale_factor[i]);

    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_gyro_scale_factor_data(mip_serializer* serializer, mip_filter_gyro_scale_factor_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        microstrain_extract_float(serializer, &self->scale_factor[i]);

    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_gyro_scale_factor_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_filter_gyro_scale_factor_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_filter_gyro_scale_factor_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_gyro_scale_factor_uncertainty_data(mip_serializer* serializer, const mip_filter_gyro_scale_factor_uncertainty_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        microstrain_insert_float(serializer, self->scale_factor_uncert[i]);

    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_gyro_scale_factor_uncertainty_data(mip_serializer* serializer, mip_filter_gyro_scale_factor_uncertainty_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        microstrain_extract_float(serializer, &self->scale_factor_uncert[i]);

    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_gyro_scale_factor_uncertainty_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_filter_gyro_scale_factor_uncertainty_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_filter_gyro_scale_factor_uncertainty_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_mag_bias_data(mip_serializer* serializer, const mip_filter_mag_bias_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        microstrain_insert_float(serializer, self->bias[i]);

    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_mag_bias_data(mip_serializer* serializer, mip_filter_mag_bias_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        microstrain_extract_float(serializer, &self->bias[i]);

    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_mag_bias_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_filter_mag_bias_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_filter_mag_bias_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_mag_bias_uncertainty_data(mip_serializer* serializer, const mip_filter_mag_bias_uncertainty_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        microstrain_insert_float(serializer, self->bias_uncert[i]);

    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_mag_bias_uncertainty_data(mip_serializer* serializer, mip_filter_mag_bias_uncertainty_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        microstrain_extract_float(serializer, &self->bias_uncert[i]);

    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_mag_bias_uncertainty_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_filter_mag_bias_uncertainty_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_filter_mag_bias_uncertainty_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_standard_atmosphere_data(mip_serializer* serializer, const mip_filter_standard_atmosphere_data* self)
{
    microstrain_insert_float(serializer, self->geometric_altitude);

    microstrain_insert_float(serializer, self->geopotential_altitude);

    microstrain_insert_float(serializer, self->standard_temperature);

    microstrain_insert_float(serializer, self->standard_pressure);

    microstrain_insert_float(serializer, self->standard_density);

    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_standard_atmosphere_data(mip_serializer* serializer, mip_filter_standard_atmosphere_data* self)
{
    microstrain_extract_float(serializer, &self->geometric_altitude);

    microstrain_extract_float(serializer, &self->geopotential_altitude);

    microstrain_extract_float(serializer, &self->standard_temperature);

    microstrain_extract_float(serializer, &self->standard_pressure);

    microstrain_extract_float(serializer, &self->standard_density);

    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_standard_atmosphere_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_filter_standard_atmosphere_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_filter_standard_atmosphere_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_pressure_altitude_data(mip_serializer* serializer, const mip_filter_pressure_altitude_data* self)
{
    microstrain_insert_float(serializer, self->pressure_altitude);

    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_pressure_altitude_data(mip_serializer* serializer, mip_filter_pressure_altitude_data* self)
{
    microstrain_extract_float(serializer, &self->pressure_altitude);

    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_pressure_altitude_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_filter_pressure_altitude_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_filter_pressure_altitude_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_density_altitude_data(mip_serializer* serializer, const mip_filter_density_altitude_data* self)
{
    microstrain_insert_float(serializer, self->density_altitude);

    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_density_altitude_data(mip_serializer* serializer, mip_filter_density_altitude_data* self)
{
    microstrain_extract_float(serializer, &self->density_altitude);

    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_density_altitude_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_filter_density_altitude_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_filter_density_altitude_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_antenna_offset_correction_data(mip_serializer* serializer, const mip_filter_antenna_offset_correction_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        microstrain_insert_float(serializer, self->offset[i]);

    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_antenna_offset_correction_data(mip_serializer* serializer, mip_filter_antenna_offset_correction_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        microstrain_extract_float(serializer, &self->offset[i]);

    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_antenna_offset_correction_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_filter_antenna_offset_correction_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_filter_antenna_offset_correction_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_antenna_offset_correction_uncertainty_data(mip_serializer* serializer, const mip_filter_antenna_offset_correction_uncertainty_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        microstrain_insert_float(serializer, self->offset_uncert[i]);

    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_antenna_offset_correction_uncertainty_data(mip_serializer* serializer, mip_filter_antenna_offset_correction_uncertainty_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        microstrain_extract_float(serializer, &self->offset_uncert[i]);

    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_antenna_offset_correction_uncertainty_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_filter_antenna_offset_correction_uncertainty_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_filter_antenna_offset_correction_uncertainty_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_multi_antenna_offset_correction_data(mip_serializer* serializer, const mip_filter_multi_antenna_offset_correction_data* self)
{
    microstrain_insert_u8(serializer, self->receiver_id);
    
    for(unsigned int i=0; i < 3; i++)
        microstrain_insert_float(serializer, self->offset[i]);

    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_multi_antenna_offset_correction_data(mip_serializer* serializer, mip_filter_multi_antenna_offset_correction_data* self)
{
    microstrain_extract_u8(serializer, &self->receiver_id);
    
    for(unsigned int i=0; i < 3; i++)
        microstrain_extract_float(serializer, &self->offset[i]);

    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_multi_antenna_offset_correction_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_filter_multi_antenna_offset_correction_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_filter_multi_antenna_offset_correction_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_multi_antenna_offset_correction_uncertainty_data(mip_serializer* serializer, const mip_filter_multi_antenna_offset_correction_uncertainty_data* self)
{
    microstrain_insert_u8(serializer, self->receiver_id);
    
    for(unsigned int i=0; i < 3; i++)
        microstrain_insert_float(serializer, self->offset_uncert[i]);

    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_multi_antenna_offset_correction_uncertainty_data(mip_serializer* serializer, mip_filter_multi_antenna_offset_correction_uncertainty_data* self)
{
    microstrain_extract_u8(serializer, &self->receiver_id);
    
    for(unsigned int i=0; i < 3; i++)
        microstrain_extract_float(serializer, &self->offset_uncert[i]);

    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_multi_antenna_offset_correction_uncertainty_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_filter_multi_antenna_offset_correction_uncertainty_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_filter_multi_antenna_offset_correction_uncertainty_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_magnetometer_offset_data(mip_serializer* serializer, const mip_filter_magnetometer_offset_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        microstrain_insert_float(serializer, self->hard_iron[i]);

    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_magnetometer_offset_data(mip_serializer* serializer, mip_filter_magnetometer_offset_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        microstrain_extract_float(serializer, &self->hard_iron[i]);

    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_magnetometer_offset_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_filter_magnetometer_offset_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_filter_magnetometer_offset_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_magnetometer_matrix_data(mip_serializer* serializer, const mip_filter_magnetometer_matrix_data* self)
{
    for(unsigned int i=0; i < 9; i++)
        microstrain_insert_float(serializer, self->soft_iron[i]);

    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_magnetometer_matrix_data(mip_serializer* serializer, mip_filter_magnetometer_matrix_data* self)
{
    for(unsigned int i=0; i < 9; i++)
        microstrain_extract_float(serializer, &self->soft_iron[i]);

    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_magnetometer_matrix_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_filter_magnetometer_matrix_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_filter_magnetometer_matrix_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_magnetometer_offset_uncertainty_data(mip_serializer* serializer, const mip_filter_magnetometer_offset_uncertainty_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        microstrain_insert_float(serializer, self->hard_iron_uncertainty[i]);

    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_magnetometer_offset_uncertainty_data(mip_serializer* serializer, mip_filter_magnetometer_offset_uncertainty_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        microstrain_extract_float(serializer, &self->hard_iron_uncertainty[i]);

    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_magnetometer_offset_uncertainty_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_filter_magnetometer_offset_uncertainty_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_filter_magnetometer_offset_uncertainty_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_magnetometer_matrix_uncertainty_data(mip_serializer* serializer, const mip_filter_magnetometer_matrix_uncertainty_data* self)
{
    for(unsigned int i=0; i < 9; i++)
        microstrain_insert_float(serializer, self->soft_iron_uncertainty[i]);

    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_magnetometer_matrix_uncertainty_data(mip_serializer* serializer, mip_filter_magnetometer_matrix_uncertainty_data* self)
{
    for(unsigned int i=0; i < 9; i++)
        microstrain_extract_float(serializer, &self->soft_iron_uncertainty[i]);

    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_magnetometer_matrix_uncertainty_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_filter_magnetometer_matrix_uncertainty_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_filter_magnetometer_matrix_uncertainty_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_magnetometer_covariance_matrix_data(mip_serializer* serializer, const mip_filter_magnetometer_covariance_matrix_data* self)
{
    for(unsigned int i=0; i < 9; i++)
        microstrain_insert_float(serializer, self->covariance[i]);

    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_magnetometer_covariance_matrix_data(mip_serializer* serializer, mip_filter_magnetometer_covariance_matrix_data* self)
{
    for(unsigned int i=0; i < 9; i++)
        microstrain_extract_float(serializer, &self->covariance[i]);

    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_magnetometer_covariance_matrix_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_filter_magnetometer_covariance_matrix_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_filter_magnetometer_covariance_matrix_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_magnetometer_residual_vector_data(mip_serializer* serializer, const mip_filter_magnetometer_residual_vector_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        microstrain_insert_float(serializer, self->residual[i]);

    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_magnetometer_residual_vector_data(mip_serializer* serializer, mip_filter_magnetometer_residual_vector_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        microstrain_extract_float(serializer, &self->residual[i]);

    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_magnetometer_residual_vector_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_filter_magnetometer_residual_vector_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_filter_magnetometer_residual_vector_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_clock_correction_data(mip_serializer* serializer, const mip_filter_clock_correction_data* self)
{
    microstrain_insert_u8(serializer, self->receiver_id);

    microstrain_insert_float(serializer, self->bias);

    microstrain_insert_float(serializer, self->bias_drift);

    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_clock_correction_data(mip_serializer* serializer, mip_filter_clock_correction_data* self)
{
    microstrain_extract_u8(serializer, &self->receiver_id);

    microstrain_extract_float(serializer, &self->bias);

    microstrain_extract_float(serializer, &self->bias_drift);

    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_clock_correction_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_filter_clock_correction_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_filter_clock_correction_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_clock_correction_uncertainty_data(mip_serializer* serializer, const mip_filter_clock_correction_uncertainty_data* self)
{
    microstrain_insert_u8(serializer, self->receiver_id);

    microstrain_insert_float(serializer, self->bias_uncertainty);

    microstrain_insert_float(serializer, self->bias_drift_uncertainty);

    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_clock_correction_uncertainty_data(mip_serializer* serializer, mip_filter_clock_correction_uncertainty_data* self)
{
    microstrain_extract_u8(serializer, &self->receiver_id);

    microstrain_extract_float(serializer, &self->bias_uncertainty);

    microstrain_extract_float(serializer, &self->bias_drift_uncertainty);

    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_clock_correction_uncertainty_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_filter_clock_correction_uncertainty_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_filter_clock_correction_uncertainty_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_gnss_pos_aid_status_data(mip_serializer* serializer, const mip_filter_gnss_pos_aid_status_data* self)
{
    microstrain_insert_u8(serializer, self->receiver_id);

    microstrain_insert_float(serializer, self->time_of_week);
    
    insert_mip_gnss_aid_status_flags(serializer, self->status);
    
    for(unsigned int i=0; i < 8; i++)
        microstrain_insert_u8(serializer, self->reserved[i]);
    
}
void extract_mip_filter_gnss_pos_aid_status_data(mip_serializer* serializer, mip_filter_gnss_pos_aid_status_data* self)
{
    microstrain_extract_u8(serializer, &self->receiver_id);

    microstrain_extract_float(serializer, &self->time_of_week);
    
    extract_mip_gnss_aid_status_flags(serializer, &self->status);
    
    for(unsigned int i=0; i < 8; i++)
        microstrain_extract_u8(serializer, &self->reserved[i]);
    
}
bool extract_mip_filter_gnss_pos_aid_status_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_filter_gnss_pos_aid_status_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_filter_gnss_pos_aid_status_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_gnss_att_aid_status_data(mip_serializer* serializer, const mip_filter_gnss_att_aid_status_data* self)
{
    microstrain_insert_float(serializer, self->time_of_week);
    
    insert_mip_gnss_aid_status_flags(serializer, self->status);
    
    for(unsigned int i=0; i < 8; i++)
        microstrain_insert_u8(serializer, self->reserved[i]);
    
}
void extract_mip_filter_gnss_att_aid_status_data(mip_serializer* serializer, mip_filter_gnss_att_aid_status_data* self)
{
    microstrain_extract_float(serializer, &self->time_of_week);
    
    extract_mip_gnss_aid_status_flags(serializer, &self->status);
    
    for(unsigned int i=0; i < 8; i++)
        microstrain_extract_u8(serializer, &self->reserved[i]);
    
}
bool extract_mip_filter_gnss_att_aid_status_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_filter_gnss_att_aid_status_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_filter_gnss_att_aid_status_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_head_aid_status_data(mip_serializer* serializer, const mip_filter_head_aid_status_data* self)
{
    microstrain_insert_float(serializer, self->time_of_week);
    
    insert_mip_filter_head_aid_status_data_heading_aid_type(serializer, self->type);
    
    for(unsigned int i=0; i < 2; i++)
        microstrain_insert_float(serializer, self->reserved[i]);
    
}
void extract_mip_filter_head_aid_status_data(mip_serializer* serializer, mip_filter_head_aid_status_data* self)
{
    microstrain_extract_float(serializer, &self->time_of_week);
    
    extract_mip_filter_head_aid_status_data_heading_aid_type(serializer, &self->type);
    
    for(unsigned int i=0; i < 2; i++)
        microstrain_extract_float(serializer, &self->reserved[i]);
    
}
bool extract_mip_filter_head_aid_status_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_filter_head_aid_status_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_filter_head_aid_status_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_head_aid_status_data_heading_aid_type(struct mip_serializer* serializer, const mip_filter_head_aid_status_data_heading_aid_type self)
{
    microstrain_insert_u8(serializer, (uint8_t) (self));
}
void extract_mip_filter_head_aid_status_data_heading_aid_type(struct mip_serializer* serializer, mip_filter_head_aid_status_data_heading_aid_type* self)
{
    uint8_t tmp = 0;
    microstrain_extract_u8(serializer, &tmp);
    *self = tmp;
}

void insert_mip_filter_rel_pos_ned_data(mip_serializer* serializer, const mip_filter_rel_pos_ned_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        microstrain_insert_double(serializer, self->relative_position[i]);

    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_rel_pos_ned_data(mip_serializer* serializer, mip_filter_rel_pos_ned_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        microstrain_extract_double(serializer, &self->relative_position[i]);

    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_rel_pos_ned_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_filter_rel_pos_ned_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_filter_rel_pos_ned_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_ecef_pos_data(mip_serializer* serializer, const mip_filter_ecef_pos_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        microstrain_insert_double(serializer, self->position_ecef[i]);

    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_ecef_pos_data(mip_serializer* serializer, mip_filter_ecef_pos_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        microstrain_extract_double(serializer, &self->position_ecef[i]);

    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_ecef_pos_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_filter_ecef_pos_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_filter_ecef_pos_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_ecef_vel_data(mip_serializer* serializer, const mip_filter_ecef_vel_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        microstrain_insert_float(serializer, self->velocity_ecef[i]);

    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_ecef_vel_data(mip_serializer* serializer, mip_filter_ecef_vel_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        microstrain_extract_float(serializer, &self->velocity_ecef[i]);

    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_ecef_vel_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_filter_ecef_vel_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_filter_ecef_vel_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_ecef_pos_uncertainty_data(mip_serializer* serializer, const mip_filter_ecef_pos_uncertainty_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        microstrain_insert_float(serializer, self->pos_uncertainty[i]);

    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_ecef_pos_uncertainty_data(mip_serializer* serializer, mip_filter_ecef_pos_uncertainty_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        microstrain_extract_float(serializer, &self->pos_uncertainty[i]);

    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_ecef_pos_uncertainty_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_filter_ecef_pos_uncertainty_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_filter_ecef_pos_uncertainty_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_ecef_vel_uncertainty_data(mip_serializer* serializer, const mip_filter_ecef_vel_uncertainty_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        microstrain_insert_float(serializer, self->vel_uncertainty[i]);

    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_ecef_vel_uncertainty_data(mip_serializer* serializer, mip_filter_ecef_vel_uncertainty_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        microstrain_extract_float(serializer, &self->vel_uncertainty[i]);

    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_ecef_vel_uncertainty_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_filter_ecef_vel_uncertainty_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_filter_ecef_vel_uncertainty_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_aiding_measurement_summary_data(mip_serializer* serializer, const mip_filter_aiding_measurement_summary_data* self)
{
    microstrain_insert_float(serializer, self->time_of_week);

    microstrain_insert_u8(serializer, self->source);
    
    insert_mip_filter_aiding_measurement_type(serializer, self->type);
    
    insert_mip_filter_measurement_indicator(serializer, self->indicator);
    
}
void extract_mip_filter_aiding_measurement_summary_data(mip_serializer* serializer, mip_filter_aiding_measurement_summary_data* self)
{
    microstrain_extract_float(serializer, &self->time_of_week);

    microstrain_extract_u8(serializer, &self->source);
    
    extract_mip_filter_aiding_measurement_type(serializer, &self->type);
    
    extract_mip_filter_measurement_indicator(serializer, &self->indicator);
    
}
bool extract_mip_filter_aiding_measurement_summary_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_filter_aiding_measurement_summary_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_filter_aiding_measurement_summary_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_odometer_scale_factor_error_data(mip_serializer* serializer, const mip_filter_odometer_scale_factor_error_data* self)
{
    microstrain_insert_float(serializer, self->scale_factor_error);

    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_odometer_scale_factor_error_data(mip_serializer* serializer, mip_filter_odometer_scale_factor_error_data* self)
{
    microstrain_extract_float(serializer, &self->scale_factor_error);

    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_odometer_scale_factor_error_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_filter_odometer_scale_factor_error_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_filter_odometer_scale_factor_error_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_odometer_scale_factor_error_uncertainty_data(mip_serializer* serializer, const mip_filter_odometer_scale_factor_error_uncertainty_data* self)
{
    microstrain_insert_float(serializer, self->scale_factor_error_uncertainty);

    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_odometer_scale_factor_error_uncertainty_data(mip_serializer* serializer, mip_filter_odometer_scale_factor_error_uncertainty_data* self)
{
    microstrain_extract_float(serializer, &self->scale_factor_error_uncertainty);

    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_odometer_scale_factor_error_uncertainty_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_filter_odometer_scale_factor_error_uncertainty_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_filter_odometer_scale_factor_error_uncertainty_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_gnss_dual_antenna_status_data(mip_serializer* serializer, const mip_filter_gnss_dual_antenna_status_data* self)
{
    microstrain_insert_float(serializer, self->time_of_week);

    microstrain_insert_float(serializer, self->heading);

    microstrain_insert_float(serializer, self->heading_unc);
    
    insert_mip_filter_gnss_dual_antenna_status_data_fix_type(serializer, self->fix_type);
    
    insert_mip_filter_gnss_dual_antenna_status_data_dual_antenna_status_flags(serializer, self->status_flags);

    microstrain_insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_filter_gnss_dual_antenna_status_data(mip_serializer* serializer, mip_filter_gnss_dual_antenna_status_data* self)
{
    microstrain_extract_float(serializer, &self->time_of_week);

    microstrain_extract_float(serializer, &self->heading);

    microstrain_extract_float(serializer, &self->heading_unc);
    
    extract_mip_filter_gnss_dual_antenna_status_data_fix_type(serializer, &self->fix_type);
    
    extract_mip_filter_gnss_dual_antenna_status_data_dual_antenna_status_flags(serializer, &self->status_flags);

    microstrain_extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_filter_gnss_dual_antenna_status_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_filter_gnss_dual_antenna_status_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_filter_gnss_dual_antenna_status_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_gnss_dual_antenna_status_data_fix_type(struct mip_serializer* serializer, const mip_filter_gnss_dual_antenna_status_data_fix_type self)
{
    microstrain_insert_u8(serializer, (uint8_t) (self));
}
void extract_mip_filter_gnss_dual_antenna_status_data_fix_type(struct mip_serializer* serializer, mip_filter_gnss_dual_antenna_status_data_fix_type* self)
{
    uint8_t tmp = 0;
    microstrain_extract_u8(serializer, &tmp);
    *self = tmp;
}

void insert_mip_filter_gnss_dual_antenna_status_data_dual_antenna_status_flags(struct mip_serializer* serializer, const mip_filter_gnss_dual_antenna_status_data_dual_antenna_status_flags self)
{
    microstrain_insert_u16(serializer, (uint16_t) (self));
}
void extract_mip_filter_gnss_dual_antenna_status_data_dual_antenna_status_flags(struct mip_serializer* serializer, mip_filter_gnss_dual_antenna_status_data_dual_antenna_status_flags* self)
{
    uint16_t tmp = 0;
    microstrain_extract_u16(serializer, &tmp);
    *self = tmp;
}

void insert_mip_filter_aiding_frame_config_error_data(mip_serializer* serializer, const mip_filter_aiding_frame_config_error_data* self)
{
    microstrain_insert_u8(serializer, self->frame_id);
    
    for(unsigned int i=0; i < 3; i++)
        microstrain_insert_float(serializer, self->translation[i]);
    
    for(unsigned int i=0; i < 4; i++)
        microstrain_insert_float(serializer, self->attitude[i]);
    
}
void extract_mip_filter_aiding_frame_config_error_data(mip_serializer* serializer, mip_filter_aiding_frame_config_error_data* self)
{
    microstrain_extract_u8(serializer, &self->frame_id);
    
    for(unsigned int i=0; i < 3; i++)
        microstrain_extract_float(serializer, &self->translation[i]);
    
    for(unsigned int i=0; i < 4; i++)
        microstrain_extract_float(serializer, &self->attitude[i]);
    
}
bool extract_mip_filter_aiding_frame_config_error_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_filter_aiding_frame_config_error_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_filter_aiding_frame_config_error_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_filter_aiding_frame_config_error_uncertainty_data(mip_serializer* serializer, const mip_filter_aiding_frame_config_error_uncertainty_data* self)
{
    microstrain_insert_u8(serializer, self->frame_id);
    
    for(unsigned int i=0; i < 3; i++)
        microstrain_insert_float(serializer, self->translation_unc[i]);
    
    for(unsigned int i=0; i < 3; i++)
        microstrain_insert_float(serializer, self->attitude_unc[i]);
    
}
void extract_mip_filter_aiding_frame_config_error_uncertainty_data(mip_serializer* serializer, mip_filter_aiding_frame_config_error_uncertainty_data* self)
{
    microstrain_extract_u8(serializer, &self->frame_id);
    
    for(unsigned int i=0; i < 3; i++)
        microstrain_extract_float(serializer, &self->translation_unc[i]);
    
    for(unsigned int i=0; i < 3; i++)
        microstrain_extract_float(serializer, &self->attitude_unc[i]);
    
}
bool extract_mip_filter_aiding_frame_config_error_uncertainty_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_filter_aiding_frame_config_error_uncertainty_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_filter_aiding_frame_config_error_uncertainty_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}


#ifdef __cplusplus
} // namespace C
} // namespace mip
} // extern "C"
#endif // __cplusplus

