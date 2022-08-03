
#include "data_filter.h"

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

size_t insert_mip_filter_mode(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_filter_mode self)
{
    return insert_u16(buffer, bufferSize, offset, self);
}
size_t extract_mip_filter_mode(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_filter_mode* self)
{
    uint16_t tmp;
    offset = extract_u16(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}

size_t insert_mip_filter_dynamics_mode(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_filter_dynamics_mode self)
{
    return insert_u16(buffer, bufferSize, offset, self);
}
size_t extract_mip_filter_dynamics_mode(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_filter_dynamics_mode* self)
{
    uint16_t tmp;
    offset = extract_u16(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}

size_t insert_mip_filter_status_flags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_filter_status_flags self)
{
    return insert_u16(buffer, bufferSize, offset, self);
}
size_t extract_mip_filter_status_flags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_filter_status_flags* self)
{
    uint16_t tmp;
    offset = extract_u16(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_mip_filter_aiding_measurement_type(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_filter_aiding_measurement_type self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}
size_t extract_mip_filter_aiding_measurement_type(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_filter_aiding_measurement_type* self)
{
    uint8_t tmp;
    offset = extract_u8(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}

size_t insert_mip_filter_measurement_indicator(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_filter_measurement_indicator self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}
size_t extract_mip_filter_measurement_indicator(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_filter_measurement_indicator* self)
{
    uint8_t tmp;
    offset = extract_u8(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_mip_gnss_aid_status_flags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_gnss_aid_status_flags self)
{
    return insert_u16(buffer, bufferSize, offset, self);
}
size_t extract_mip_gnss_aid_status_flags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_gnss_aid_status_flags* self)
{
    uint16_t tmp;
    offset = extract_u16(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}



////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_position_llh_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_position_llh_data* self)
{
    offset = insert_double(buffer, bufferSize, offset, self->latitude);
    offset = insert_double(buffer, bufferSize, offset, self->longitude);
    offset = insert_double(buffer, bufferSize, offset, self->ellipsoid_height);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_mip_filter_position_llh_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_position_llh_data* self)
{
    offset = extract_double(buffer, bufferSize, offset, &self->latitude);
    offset = extract_double(buffer, bufferSize, offset, &self->longitude);
    offset = extract_double(buffer, bufferSize, offset, &self->ellipsoid_height);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_velocity_ned_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_velocity_ned_data* self)
{
    offset = insert_float(buffer, bufferSize, offset, self->north);
    offset = insert_float(buffer, bufferSize, offset, self->east);
    offset = insert_float(buffer, bufferSize, offset, self->down);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_mip_filter_velocity_ned_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_velocity_ned_data* self)
{
    offset = extract_float(buffer, bufferSize, offset, &self->north);
    offset = extract_float(buffer, bufferSize, offset, &self->east);
    offset = extract_float(buffer, bufferSize, offset, &self->down);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_attitude_quaternion_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_attitude_quaternion_data* self)
{
    for(unsigned int i=0; i < 4; i++)
        offset = insert_float(buffer, bufferSize, offset, self->q[i]);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_mip_filter_attitude_quaternion_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_attitude_quaternion_data* self)
{
    for(unsigned int i=0; i < 4; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->q[i]);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_attitude_dcm_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_attitude_dcm_data* self)
{
    for(unsigned int i=0; i < 9; i++)
        offset = insert_float(buffer, bufferSize, offset, self->dcm[i]);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_mip_filter_attitude_dcm_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_attitude_dcm_data* self)
{
    for(unsigned int i=0; i < 9; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->dcm[i]);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_euler_angles_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_euler_angles_data* self)
{
    offset = insert_float(buffer, bufferSize, offset, self->roll);
    offset = insert_float(buffer, bufferSize, offset, self->pitch);
    offset = insert_float(buffer, bufferSize, offset, self->yaw);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_mip_filter_euler_angles_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_euler_angles_data* self)
{
    offset = extract_float(buffer, bufferSize, offset, &self->roll);
    offset = extract_float(buffer, bufferSize, offset, &self->pitch);
    offset = extract_float(buffer, bufferSize, offset, &self->yaw);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_gyro_bias_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_gyro_bias_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->bias[i]);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_mip_filter_gyro_bias_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_gyro_bias_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->bias[i]);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_accel_bias_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_accel_bias_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->bias[i]);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_mip_filter_accel_bias_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_accel_bias_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->bias[i]);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_position_llh_uncertainty_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_position_llh_uncertainty_data* self)
{
    offset = insert_float(buffer, bufferSize, offset, self->north);
    offset = insert_float(buffer, bufferSize, offset, self->east);
    offset = insert_float(buffer, bufferSize, offset, self->down);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_mip_filter_position_llh_uncertainty_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_position_llh_uncertainty_data* self)
{
    offset = extract_float(buffer, bufferSize, offset, &self->north);
    offset = extract_float(buffer, bufferSize, offset, &self->east);
    offset = extract_float(buffer, bufferSize, offset, &self->down);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_velocity_ned_uncertainty_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_velocity_ned_uncertainty_data* self)
{
    offset = insert_float(buffer, bufferSize, offset, self->north);
    offset = insert_float(buffer, bufferSize, offset, self->east);
    offset = insert_float(buffer, bufferSize, offset, self->down);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_mip_filter_velocity_ned_uncertainty_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_velocity_ned_uncertainty_data* self)
{
    offset = extract_float(buffer, bufferSize, offset, &self->north);
    offset = extract_float(buffer, bufferSize, offset, &self->east);
    offset = extract_float(buffer, bufferSize, offset, &self->down);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_euler_angles_uncertainty_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_euler_angles_uncertainty_data* self)
{
    offset = insert_float(buffer, bufferSize, offset, self->roll);
    offset = insert_float(buffer, bufferSize, offset, self->pitch);
    offset = insert_float(buffer, bufferSize, offset, self->yaw);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_mip_filter_euler_angles_uncertainty_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_euler_angles_uncertainty_data* self)
{
    offset = extract_float(buffer, bufferSize, offset, &self->roll);
    offset = extract_float(buffer, bufferSize, offset, &self->pitch);
    offset = extract_float(buffer, bufferSize, offset, &self->yaw);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_gyro_bias_uncertainty_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_gyro_bias_uncertainty_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->bias_uncert[i]);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_mip_filter_gyro_bias_uncertainty_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_gyro_bias_uncertainty_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->bias_uncert[i]);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_accel_bias_uncertainty_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_accel_bias_uncertainty_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->bias_uncert[i]);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_mip_filter_accel_bias_uncertainty_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_accel_bias_uncertainty_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->bias_uncert[i]);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_timestamp_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_timestamp_data* self)
{
    offset = insert_double(buffer, bufferSize, offset, self->tow);
    offset = insert_u16(buffer, bufferSize, offset, self->week_number);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_mip_filter_timestamp_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_timestamp_data* self)
{
    offset = extract_double(buffer, bufferSize, offset, &self->tow);
    offset = extract_u16(buffer, bufferSize, offset, &self->week_number);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_status_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_status_data* self)
{
    offset = insert_mip_filter_mode(buffer, bufferSize, offset, self->filter_state);
    offset = insert_mip_filter_dynamics_mode(buffer, bufferSize, offset, self->dynamics_mode);
    offset = insert_mip_filter_status_flags(buffer, bufferSize, offset, self->status_flags);
    
    return offset;
}

size_t extract_mip_filter_status_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_status_data* self)
{
    offset = extract_mip_filter_mode(buffer, bufferSize, offset, &self->filter_state);
    offset = extract_mip_filter_dynamics_mode(buffer, bufferSize, offset, &self->dynamics_mode);
    offset = extract_mip_filter_status_flags(buffer, bufferSize, offset, &self->status_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_linear_accel_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_linear_accel_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->accel[i]);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_mip_filter_linear_accel_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_linear_accel_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->accel[i]);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_gravity_vector_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_gravity_vector_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->gravity[i]);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_mip_filter_gravity_vector_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_gravity_vector_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->gravity[i]);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_comp_accel_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_comp_accel_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->accel[i]);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_mip_filter_comp_accel_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_comp_accel_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->accel[i]);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_comp_angular_rate_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_comp_angular_rate_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->gyro[i]);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_mip_filter_comp_angular_rate_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_comp_angular_rate_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->gyro[i]);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_quaternion_attitude_uncertainty_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_quaternion_attitude_uncertainty_data* self)
{
    for(unsigned int i=0; i < 4; i++)
        offset = insert_float(buffer, bufferSize, offset, self->q[i]);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_mip_filter_quaternion_attitude_uncertainty_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_quaternion_attitude_uncertainty_data* self)
{
    for(unsigned int i=0; i < 4; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->q[i]);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_wgs84_gravity_mag_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_wgs84_gravity_mag_data* self)
{
    offset = insert_float(buffer, bufferSize, offset, self->magnitude);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_mip_filter_wgs84_gravity_mag_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_wgs84_gravity_mag_data* self)
{
    offset = extract_float(buffer, bufferSize, offset, &self->magnitude);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_heading_update_state_data_heading_source(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_filter_heading_update_state_data_heading_source self)
{
    return insert_u16(buffer, bufferSize, offset, self);
}
size_t extract_mip_filter_heading_update_state_data_heading_source(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_filter_heading_update_state_data_heading_source* self)
{
    uint16_t tmp;
    offset = extract_u16(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}

size_t insert_mip_filter_heading_update_state_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_heading_update_state_data* self)
{
    offset = insert_float(buffer, bufferSize, offset, self->heading);
    offset = insert_float(buffer, bufferSize, offset, self->heading_1sigma);
    offset = insert_mip_filter_heading_update_state_data_heading_source(buffer, bufferSize, offset, self->source);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_mip_filter_heading_update_state_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_heading_update_state_data* self)
{
    offset = extract_float(buffer, bufferSize, offset, &self->heading);
    offset = extract_float(buffer, bufferSize, offset, &self->heading_1sigma);
    offset = extract_mip_filter_heading_update_state_data_heading_source(buffer, bufferSize, offset, &self->source);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_magnetic_model_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_magnetic_model_data* self)
{
    offset = insert_float(buffer, bufferSize, offset, self->intensity_north);
    offset = insert_float(buffer, bufferSize, offset, self->intensity_east);
    offset = insert_float(buffer, bufferSize, offset, self->intensity_down);
    offset = insert_float(buffer, bufferSize, offset, self->inclination);
    offset = insert_float(buffer, bufferSize, offset, self->declination);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_mip_filter_magnetic_model_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_magnetic_model_data* self)
{
    offset = extract_float(buffer, bufferSize, offset, &self->intensity_north);
    offset = extract_float(buffer, bufferSize, offset, &self->intensity_east);
    offset = extract_float(buffer, bufferSize, offset, &self->intensity_down);
    offset = extract_float(buffer, bufferSize, offset, &self->inclination);
    offset = extract_float(buffer, bufferSize, offset, &self->declination);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_accel_scale_factor_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_accel_scale_factor_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->scale_factor[i]);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_mip_filter_accel_scale_factor_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_accel_scale_factor_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->scale_factor[i]);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_accel_scale_factor_uncertainty_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_accel_scale_factor_uncertainty_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->scale_factor_uncert[i]);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_mip_filter_accel_scale_factor_uncertainty_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_accel_scale_factor_uncertainty_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->scale_factor_uncert[i]);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_gyro_scale_factor_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_gyro_scale_factor_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->scale_factor[i]);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_mip_filter_gyro_scale_factor_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_gyro_scale_factor_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->scale_factor[i]);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_gyro_scale_factor_uncertainty_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_gyro_scale_factor_uncertainty_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->scale_factor_uncert[i]);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_mip_filter_gyro_scale_factor_uncertainty_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_gyro_scale_factor_uncertainty_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->scale_factor_uncert[i]);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_mag_bias_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_mag_bias_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->bias[i]);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_mip_filter_mag_bias_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_mag_bias_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->bias[i]);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_mag_bias_uncertainty_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_mag_bias_uncertainty_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->bias_uncert[i]);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_mip_filter_mag_bias_uncertainty_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_mag_bias_uncertainty_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->bias_uncert[i]);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_standard_atmosphere_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_standard_atmosphere_data* self)
{
    offset = insert_float(buffer, bufferSize, offset, self->geometric_altitude);
    offset = insert_float(buffer, bufferSize, offset, self->geopotential_altitude);
    offset = insert_float(buffer, bufferSize, offset, self->standard_temperature);
    offset = insert_float(buffer, bufferSize, offset, self->standard_pressure);
    offset = insert_float(buffer, bufferSize, offset, self->standard_density);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_mip_filter_standard_atmosphere_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_standard_atmosphere_data* self)
{
    offset = extract_float(buffer, bufferSize, offset, &self->geometric_altitude);
    offset = extract_float(buffer, bufferSize, offset, &self->geopotential_altitude);
    offset = extract_float(buffer, bufferSize, offset, &self->standard_temperature);
    offset = extract_float(buffer, bufferSize, offset, &self->standard_pressure);
    offset = extract_float(buffer, bufferSize, offset, &self->standard_density);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_pressure_altitude_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_pressure_altitude_data* self)
{
    offset = insert_float(buffer, bufferSize, offset, self->pressure_altitude);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_mip_filter_pressure_altitude_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_pressure_altitude_data* self)
{
    offset = extract_float(buffer, bufferSize, offset, &self->pressure_altitude);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_density_altitude_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_density_altitude_data* self)
{
    offset = insert_float(buffer, bufferSize, offset, self->density_altitude);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_mip_filter_density_altitude_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_density_altitude_data* self)
{
    offset = extract_float(buffer, bufferSize, offset, &self->density_altitude);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_antenna_offset_correction_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_antenna_offset_correction_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->offset[i]);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_mip_filter_antenna_offset_correction_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_antenna_offset_correction_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->offset[i]);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_antenna_offset_correction_uncertainty_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_antenna_offset_correction_uncertainty_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->offset_uncert[i]);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_mip_filter_antenna_offset_correction_uncertainty_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_antenna_offset_correction_uncertainty_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->offset_uncert[i]);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_multi_antenna_offset_correction_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_multi_antenna_offset_correction_data* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->receiver_id);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->offset[i]);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_mip_filter_multi_antenna_offset_correction_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_multi_antenna_offset_correction_data* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->receiver_id);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->offset[i]);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_multi_antenna_offset_correction_uncertainty_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_multi_antenna_offset_correction_uncertainty_data* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->receiver_id);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->offset_uncert[i]);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_mip_filter_multi_antenna_offset_correction_uncertainty_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_multi_antenna_offset_correction_uncertainty_data* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->receiver_id);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->offset_uncert[i]);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_magnetometer_offset_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_magnetometer_offset_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->hard_iron[i]);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_mip_filter_magnetometer_offset_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_magnetometer_offset_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->hard_iron[i]);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_magnetometer_matrix_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_magnetometer_matrix_data* self)
{
    for(unsigned int i=0; i < 9; i++)
        offset = insert_float(buffer, bufferSize, offset, self->soft_iron[i]);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_mip_filter_magnetometer_matrix_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_magnetometer_matrix_data* self)
{
    for(unsigned int i=0; i < 9; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->soft_iron[i]);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_magnetometer_offset_uncertainty_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_magnetometer_offset_uncertainty_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->hard_iron_uncertainty[i]);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_mip_filter_magnetometer_offset_uncertainty_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_magnetometer_offset_uncertainty_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->hard_iron_uncertainty[i]);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_magnetometer_matrix_uncertainty_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_magnetometer_matrix_uncertainty_data* self)
{
    for(unsigned int i=0; i < 9; i++)
        offset = insert_float(buffer, bufferSize, offset, self->soft_iron_uncertainty[i]);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_mip_filter_magnetometer_matrix_uncertainty_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_magnetometer_matrix_uncertainty_data* self)
{
    for(unsigned int i=0; i < 9; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->soft_iron_uncertainty[i]);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_magnetometer_covariance_matrix_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_magnetometer_covariance_matrix_data* self)
{
    for(unsigned int i=0; i < 9; i++)
        offset = insert_float(buffer, bufferSize, offset, self->covariance[i]);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_mip_filter_magnetometer_covariance_matrix_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_magnetometer_covariance_matrix_data* self)
{
    for(unsigned int i=0; i < 9; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->covariance[i]);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_magnetometer_residual_vector_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_magnetometer_residual_vector_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->residual[i]);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_mip_filter_magnetometer_residual_vector_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_magnetometer_residual_vector_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->residual[i]);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_clock_correction_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_clock_correction_data* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->receiver_id);
    offset = insert_float(buffer, bufferSize, offset, self->bias);
    offset = insert_float(buffer, bufferSize, offset, self->bias_drift);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_mip_filter_clock_correction_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_clock_correction_data* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->receiver_id);
    offset = extract_float(buffer, bufferSize, offset, &self->bias);
    offset = extract_float(buffer, bufferSize, offset, &self->bias_drift);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_clock_correction_uncertainty_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_clock_correction_uncertainty_data* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->receiver_id);
    offset = insert_float(buffer, bufferSize, offset, self->bias_uncertainty);
    offset = insert_float(buffer, bufferSize, offset, self->bias_drift_uncertainty);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_mip_filter_clock_correction_uncertainty_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_clock_correction_uncertainty_data* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->receiver_id);
    offset = extract_float(buffer, bufferSize, offset, &self->bias_uncertainty);
    offset = extract_float(buffer, bufferSize, offset, &self->bias_drift_uncertainty);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_gnss_pos_aid_status_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_gnss_pos_aid_status_data* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->receiver_id);
    offset = insert_float(buffer, bufferSize, offset, self->time_of_week);
    offset = insert_mip_gnss_aid_status_flags(buffer, bufferSize, offset, self->status);
    for(unsigned int i=0; i < 8; i++)
        offset = insert_u8(buffer, bufferSize, offset, self->reserved[i]);
    
    return offset;
}

size_t extract_mip_filter_gnss_pos_aid_status_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_gnss_pos_aid_status_data* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->receiver_id);
    offset = extract_float(buffer, bufferSize, offset, &self->time_of_week);
    offset = extract_mip_gnss_aid_status_flags(buffer, bufferSize, offset, &self->status);
    for(unsigned int i=0; i < 8; i++)
        offset = extract_u8(buffer, bufferSize, offset, &self->reserved[i]);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_gnss_att_aid_status_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_gnss_att_aid_status_data* self)
{
    offset = insert_float(buffer, bufferSize, offset, self->time_of_week);
    offset = insert_mip_gnss_aid_status_flags(buffer, bufferSize, offset, self->status);
    for(unsigned int i=0; i < 8; i++)
        offset = insert_u8(buffer, bufferSize, offset, self->reserved[i]);
    
    return offset;
}

size_t extract_mip_filter_gnss_att_aid_status_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_gnss_att_aid_status_data* self)
{
    offset = extract_float(buffer, bufferSize, offset, &self->time_of_week);
    offset = extract_mip_gnss_aid_status_flags(buffer, bufferSize, offset, &self->status);
    for(unsigned int i=0; i < 8; i++)
        offset = extract_u8(buffer, bufferSize, offset, &self->reserved[i]);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_head_aid_status_data_heading_aid_type(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_filter_head_aid_status_data_heading_aid_type self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}
size_t extract_mip_filter_head_aid_status_data_heading_aid_type(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_filter_head_aid_status_data_heading_aid_type* self)
{
    uint8_t tmp;
    offset = extract_u8(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}

size_t insert_mip_filter_head_aid_status_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_head_aid_status_data* self)
{
    offset = insert_float(buffer, bufferSize, offset, self->time_of_week);
    offset = insert_mip_filter_head_aid_status_data_heading_aid_type(buffer, bufferSize, offset, self->type);
    for(unsigned int i=0; i < 2; i++)
        offset = insert_float(buffer, bufferSize, offset, self->reserved[i]);
    
    return offset;
}

size_t extract_mip_filter_head_aid_status_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_head_aid_status_data* self)
{
    offset = extract_float(buffer, bufferSize, offset, &self->time_of_week);
    offset = extract_mip_filter_head_aid_status_data_heading_aid_type(buffer, bufferSize, offset, &self->type);
    for(unsigned int i=0; i < 2; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->reserved[i]);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_rel_pos_ned_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_rel_pos_ned_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = insert_double(buffer, bufferSize, offset, self->relative_position[i]);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_mip_filter_rel_pos_ned_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_rel_pos_ned_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = extract_double(buffer, bufferSize, offset, &self->relative_position[i]);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_ecef_pos_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_ecef_pos_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = insert_double(buffer, bufferSize, offset, self->position_ecef[i]);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_mip_filter_ecef_pos_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_ecef_pos_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = extract_double(buffer, bufferSize, offset, &self->position_ecef[i]);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_ecef_vel_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_ecef_vel_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->velocity_ecef[i]);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_mip_filter_ecef_vel_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_ecef_vel_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->velocity_ecef[i]);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_ecef_pos_uncertainty_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_ecef_pos_uncertainty_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->pos_uncertainty[i]);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_mip_filter_ecef_pos_uncertainty_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_ecef_pos_uncertainty_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->pos_uncertainty[i]);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_ecef_vel_uncertainty_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_ecef_vel_uncertainty_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->vel_uncertainty[i]);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_mip_filter_ecef_vel_uncertainty_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_ecef_vel_uncertainty_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->vel_uncertainty[i]);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_aiding_measurement_summary_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_aiding_measurement_summary_data* self)
{
    offset = insert_float(buffer, bufferSize, offset, self->time_of_week);
    offset = insert_u8(buffer, bufferSize, offset, self->source);
    offset = insert_mip_filter_aiding_measurement_type(buffer, bufferSize, offset, self->type);
    offset = insert_mip_filter_measurement_indicator(buffer, bufferSize, offset, self->indicator);
    
    return offset;
}

size_t extract_mip_filter_aiding_measurement_summary_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_aiding_measurement_summary_data* self)
{
    offset = extract_float(buffer, bufferSize, offset, &self->time_of_week);
    offset = extract_u8(buffer, bufferSize, offset, &self->source);
    offset = extract_mip_filter_aiding_measurement_type(buffer, bufferSize, offset, &self->type);
    offset = extract_mip_filter_measurement_indicator(buffer, bufferSize, offset, &self->indicator);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_odometer_scale_factor_error_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_odometer_scale_factor_error_data* self)
{
    offset = insert_float(buffer, bufferSize, offset, self->scale_factor_error);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_mip_filter_odometer_scale_factor_error_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_odometer_scale_factor_error_data* self)
{
    offset = extract_float(buffer, bufferSize, offset, &self->scale_factor_error);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_odometer_scale_factor_error_uncertainty_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_odometer_scale_factor_error_uncertainty_data* self)
{
    offset = insert_float(buffer, bufferSize, offset, self->scale_factor_error_uncertainty);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_mip_filter_odometer_scale_factor_error_uncertainty_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_odometer_scale_factor_error_uncertainty_data* self)
{
    offset = extract_float(buffer, bufferSize, offset, &self->scale_factor_error_uncertainty);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_filter_gnss_dual_antenna_status_data_fix_type(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_filter_gnss_dual_antenna_status_data_fix_type self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}
size_t extract_mip_filter_gnss_dual_antenna_status_data_fix_type(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_filter_gnss_dual_antenna_status_data_fix_type* self)
{
    uint8_t tmp;
    offset = extract_u8(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}

size_t insert_mip_filter_gnss_dual_antenna_status_data_dual_antenna_status_flags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_filter_gnss_dual_antenna_status_data_dual_antenna_status_flags self)
{
    return insert_u16(buffer, bufferSize, offset, self);
}
size_t extract_mip_filter_gnss_dual_antenna_status_data_dual_antenna_status_flags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_filter_gnss_dual_antenna_status_data_dual_antenna_status_flags* self)
{
    uint16_t tmp;
    offset = extract_u16(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_mip_filter_gnss_dual_antenna_status_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_filter_gnss_dual_antenna_status_data* self)
{
    offset = insert_float(buffer, bufferSize, offset, self->time_of_week);
    offset = insert_float(buffer, bufferSize, offset, self->heading);
    offset = insert_float(buffer, bufferSize, offset, self->heading_unc);
    offset = insert_mip_filter_gnss_dual_antenna_status_data_fix_type(buffer, bufferSize, offset, self->fix_type);
    offset = insert_mip_filter_gnss_dual_antenna_status_data_dual_antenna_status_flags(buffer, bufferSize, offset, self->status_flags);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_mip_filter_gnss_dual_antenna_status_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_filter_gnss_dual_antenna_status_data* self)
{
    offset = extract_float(buffer, bufferSize, offset, &self->time_of_week);
    offset = extract_float(buffer, bufferSize, offset, &self->heading);
    offset = extract_float(buffer, bufferSize, offset, &self->heading_unc);
    offset = extract_mip_filter_gnss_dual_antenna_status_data_fix_type(buffer, bufferSize, offset, &self->fix_type);
    offset = extract_mip_filter_gnss_dual_antenna_status_data_dual_antenna_status_flags(buffer, bufferSize, offset, &self->status_flags);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}



#ifdef __cplusplus
} // extern "C"
} // namespace mscl
#endif // __cplusplus
