
#include "data_gnss.h"

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

size_t insert_mip_gnss_constellation_id(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_gnss_constellation_id self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}
size_t extract_mip_gnss_constellation_id(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_gnss_constellation_id* self)
{
    uint8_t tmp;
    offset = extract_u8(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}

size_t insert_mip_gnss_signal_id(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_gnss_signal_id self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}
size_t extract_mip_gnss_signal_id(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_gnss_signal_id* self)
{
    uint8_t tmp;
    offset = extract_u8(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}

size_t insert_mip_sbas_system(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_sbas_system self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}
size_t extract_mip_sbas_system(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_sbas_system* self)
{
    uint8_t tmp;
    offset = extract_u8(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_gnss_llh_pos_data_valid_flags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_gnss_llh_pos_data_valid_flags self)
{
    return insert_u16(buffer, bufferSize, offset, self);
}
size_t extract_mip_gnss_llh_pos_data_valid_flags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_gnss_llh_pos_data_valid_flags* self)
{
    uint16_t tmp;
    offset = extract_u16(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_mip_gnss_llh_pos_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_gnss_llh_pos_data* self)
{
    offset = insert_double(buffer, bufferSize, offset, self->latitude);
    offset = insert_double(buffer, bufferSize, offset, self->longitude);
    offset = insert_double(buffer, bufferSize, offset, self->ellipsoid_height);
    offset = insert_double(buffer, bufferSize, offset, self->msl_height);
    offset = insert_float(buffer, bufferSize, offset, self->horizontal_accuracy);
    offset = insert_float(buffer, bufferSize, offset, self->vertical_accuracy);
    offset = insert_mip_gnss_llh_pos_data_valid_flags(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_mip_gnss_llh_pos_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_gnss_llh_pos_data* self)
{
    offset = extract_double(buffer, bufferSize, offset, &self->latitude);
    offset = extract_double(buffer, bufferSize, offset, &self->longitude);
    offset = extract_double(buffer, bufferSize, offset, &self->ellipsoid_height);
    offset = extract_double(buffer, bufferSize, offset, &self->msl_height);
    offset = extract_float(buffer, bufferSize, offset, &self->horizontal_accuracy);
    offset = extract_float(buffer, bufferSize, offset, &self->vertical_accuracy);
    offset = extract_mip_gnss_llh_pos_data_valid_flags(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_gnss_ecef_pos_data_valid_flags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_gnss_ecef_pos_data_valid_flags self)
{
    return insert_u16(buffer, bufferSize, offset, self);
}
size_t extract_mip_gnss_ecef_pos_data_valid_flags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_gnss_ecef_pos_data_valid_flags* self)
{
    uint16_t tmp;
    offset = extract_u16(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_mip_gnss_ecef_pos_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_gnss_ecef_pos_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = insert_double(buffer, bufferSize, offset, self->x[i]);
    offset = insert_float(buffer, bufferSize, offset, self->x_accuracy);
    offset = insert_mip_gnss_ecef_pos_data_valid_flags(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_mip_gnss_ecef_pos_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_gnss_ecef_pos_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = extract_double(buffer, bufferSize, offset, &self->x[i]);
    offset = extract_float(buffer, bufferSize, offset, &self->x_accuracy);
    offset = extract_mip_gnss_ecef_pos_data_valid_flags(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_gnss_ned_vel_data_valid_flags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_gnss_ned_vel_data_valid_flags self)
{
    return insert_u16(buffer, bufferSize, offset, self);
}
size_t extract_mip_gnss_ned_vel_data_valid_flags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_gnss_ned_vel_data_valid_flags* self)
{
    uint16_t tmp;
    offset = extract_u16(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_mip_gnss_ned_vel_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_gnss_ned_vel_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->v[i]);
    offset = insert_float(buffer, bufferSize, offset, self->speed);
    offset = insert_float(buffer, bufferSize, offset, self->ground_speed);
    offset = insert_float(buffer, bufferSize, offset, self->heading);
    offset = insert_float(buffer, bufferSize, offset, self->speed_accuracy);
    offset = insert_float(buffer, bufferSize, offset, self->heading_accuracy);
    offset = insert_mip_gnss_ned_vel_data_valid_flags(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_mip_gnss_ned_vel_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_gnss_ned_vel_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->v[i]);
    offset = extract_float(buffer, bufferSize, offset, &self->speed);
    offset = extract_float(buffer, bufferSize, offset, &self->ground_speed);
    offset = extract_float(buffer, bufferSize, offset, &self->heading);
    offset = extract_float(buffer, bufferSize, offset, &self->speed_accuracy);
    offset = extract_float(buffer, bufferSize, offset, &self->heading_accuracy);
    offset = extract_mip_gnss_ned_vel_data_valid_flags(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_gnss_ecef_vel_data_valid_flags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_gnss_ecef_vel_data_valid_flags self)
{
    return insert_u16(buffer, bufferSize, offset, self);
}
size_t extract_mip_gnss_ecef_vel_data_valid_flags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_gnss_ecef_vel_data_valid_flags* self)
{
    uint16_t tmp;
    offset = extract_u16(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_mip_gnss_ecef_vel_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_gnss_ecef_vel_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->v[i]);
    offset = insert_float(buffer, bufferSize, offset, self->v_accuracy);
    offset = insert_mip_gnss_ecef_vel_data_valid_flags(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_mip_gnss_ecef_vel_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_gnss_ecef_vel_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->v[i]);
    offset = extract_float(buffer, bufferSize, offset, &self->v_accuracy);
    offset = extract_mip_gnss_ecef_vel_data_valid_flags(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_gnss_dop_data_valid_flags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_gnss_dop_data_valid_flags self)
{
    return insert_u16(buffer, bufferSize, offset, self);
}
size_t extract_mip_gnss_dop_data_valid_flags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_gnss_dop_data_valid_flags* self)
{
    uint16_t tmp;
    offset = extract_u16(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_mip_gnss_dop_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_gnss_dop_data* self)
{
    offset = insert_float(buffer, bufferSize, offset, self->gdop);
    offset = insert_float(buffer, bufferSize, offset, self->pdop);
    offset = insert_float(buffer, bufferSize, offset, self->hdop);
    offset = insert_float(buffer, bufferSize, offset, self->vdop);
    offset = insert_float(buffer, bufferSize, offset, self->tdop);
    offset = insert_float(buffer, bufferSize, offset, self->ndop);
    offset = insert_float(buffer, bufferSize, offset, self->edop);
    offset = insert_mip_gnss_dop_data_valid_flags(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_mip_gnss_dop_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_gnss_dop_data* self)
{
    offset = extract_float(buffer, bufferSize, offset, &self->gdop);
    offset = extract_float(buffer, bufferSize, offset, &self->pdop);
    offset = extract_float(buffer, bufferSize, offset, &self->hdop);
    offset = extract_float(buffer, bufferSize, offset, &self->vdop);
    offset = extract_float(buffer, bufferSize, offset, &self->tdop);
    offset = extract_float(buffer, bufferSize, offset, &self->ndop);
    offset = extract_float(buffer, bufferSize, offset, &self->edop);
    offset = extract_mip_gnss_dop_data_valid_flags(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_gnss_utc_time_data_valid_flags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_gnss_utc_time_data_valid_flags self)
{
    return insert_u16(buffer, bufferSize, offset, self);
}
size_t extract_mip_gnss_utc_time_data_valid_flags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_gnss_utc_time_data_valid_flags* self)
{
    uint16_t tmp;
    offset = extract_u16(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_mip_gnss_utc_time_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_gnss_utc_time_data* self)
{
    offset = insert_u16(buffer, bufferSize, offset, self->year);
    offset = insert_u8(buffer, bufferSize, offset, self->month);
    offset = insert_u8(buffer, bufferSize, offset, self->day);
    offset = insert_u8(buffer, bufferSize, offset, self->hour);
    offset = insert_u8(buffer, bufferSize, offset, self->min);
    offset = insert_u8(buffer, bufferSize, offset, self->sec);
    offset = insert_u32(buffer, bufferSize, offset, self->msec);
    offset = insert_mip_gnss_utc_time_data_valid_flags(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_mip_gnss_utc_time_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_gnss_utc_time_data* self)
{
    offset = extract_u16(buffer, bufferSize, offset, &self->year);
    offset = extract_u8(buffer, bufferSize, offset, &self->month);
    offset = extract_u8(buffer, bufferSize, offset, &self->day);
    offset = extract_u8(buffer, bufferSize, offset, &self->hour);
    offset = extract_u8(buffer, bufferSize, offset, &self->min);
    offset = extract_u8(buffer, bufferSize, offset, &self->sec);
    offset = extract_u32(buffer, bufferSize, offset, &self->msec);
    offset = extract_mip_gnss_utc_time_data_valid_flags(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_gnss_gps_time_data_valid_flags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_gnss_gps_time_data_valid_flags self)
{
    return insert_u16(buffer, bufferSize, offset, self);
}
size_t extract_mip_gnss_gps_time_data_valid_flags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_gnss_gps_time_data_valid_flags* self)
{
    uint16_t tmp;
    offset = extract_u16(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_mip_gnss_gps_time_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_gnss_gps_time_data* self)
{
    offset = insert_double(buffer, bufferSize, offset, self->tow);
    offset = insert_u16(buffer, bufferSize, offset, self->week_number);
    offset = insert_mip_gnss_gps_time_data_valid_flags(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_mip_gnss_gps_time_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_gnss_gps_time_data* self)
{
    offset = extract_double(buffer, bufferSize, offset, &self->tow);
    offset = extract_u16(buffer, bufferSize, offset, &self->week_number);
    offset = extract_mip_gnss_gps_time_data_valid_flags(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_gnss_clock_info_data_valid_flags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_gnss_clock_info_data_valid_flags self)
{
    return insert_u16(buffer, bufferSize, offset, self);
}
size_t extract_mip_gnss_clock_info_data_valid_flags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_gnss_clock_info_data_valid_flags* self)
{
    uint16_t tmp;
    offset = extract_u16(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_mip_gnss_clock_info_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_gnss_clock_info_data* self)
{
    offset = insert_double(buffer, bufferSize, offset, self->bias);
    offset = insert_double(buffer, bufferSize, offset, self->drift);
    offset = insert_double(buffer, bufferSize, offset, self->accuracy_estimate);
    offset = insert_mip_gnss_clock_info_data_valid_flags(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_mip_gnss_clock_info_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_gnss_clock_info_data* self)
{
    offset = extract_double(buffer, bufferSize, offset, &self->bias);
    offset = extract_double(buffer, bufferSize, offset, &self->drift);
    offset = extract_double(buffer, bufferSize, offset, &self->accuracy_estimate);
    offset = extract_mip_gnss_clock_info_data_valid_flags(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_gnss_fix_info_data_fix_type(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_gnss_fix_info_data_fix_type self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}
size_t extract_mip_gnss_fix_info_data_fix_type(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_gnss_fix_info_data_fix_type* self)
{
    uint8_t tmp;
    offset = extract_u8(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}

size_t insert_mip_gnss_fix_info_data_fix_flags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_gnss_fix_info_data_fix_flags self)
{
    return insert_u16(buffer, bufferSize, offset, self);
}
size_t extract_mip_gnss_fix_info_data_fix_flags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_gnss_fix_info_data_fix_flags* self)
{
    uint16_t tmp;
    offset = extract_u16(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_mip_gnss_fix_info_data_valid_flags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_gnss_fix_info_data_valid_flags self)
{
    return insert_u16(buffer, bufferSize, offset, self);
}
size_t extract_mip_gnss_fix_info_data_valid_flags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_gnss_fix_info_data_valid_flags* self)
{
    uint16_t tmp;
    offset = extract_u16(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_mip_gnss_fix_info_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_gnss_fix_info_data* self)
{
    offset = insert_mip_gnss_fix_info_data_fix_type(buffer, bufferSize, offset, self->fix_type);
    offset = insert_u8(buffer, bufferSize, offset, self->num_sv);
    offset = insert_mip_gnss_fix_info_data_fix_flags(buffer, bufferSize, offset, self->fix_flags);
    offset = insert_mip_gnss_fix_info_data_valid_flags(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_mip_gnss_fix_info_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_gnss_fix_info_data* self)
{
    offset = extract_mip_gnss_fix_info_data_fix_type(buffer, bufferSize, offset, &self->fix_type);
    offset = extract_u8(buffer, bufferSize, offset, &self->num_sv);
    offset = extract_mip_gnss_fix_info_data_fix_flags(buffer, bufferSize, offset, &self->fix_flags);
    offset = extract_mip_gnss_fix_info_data_valid_flags(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_gnss_sv_info_data_svflags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_gnss_sv_info_data_svflags self)
{
    return insert_u16(buffer, bufferSize, offset, self);
}
size_t extract_mip_gnss_sv_info_data_svflags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_gnss_sv_info_data_svflags* self)
{
    uint16_t tmp;
    offset = extract_u16(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_mip_gnss_sv_info_data_valid_flags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_gnss_sv_info_data_valid_flags self)
{
    return insert_u16(buffer, bufferSize, offset, self);
}
size_t extract_mip_gnss_sv_info_data_valid_flags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_gnss_sv_info_data_valid_flags* self)
{
    uint16_t tmp;
    offset = extract_u16(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_mip_gnss_sv_info_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_gnss_sv_info_data* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->channel);
    offset = insert_u8(buffer, bufferSize, offset, self->sv_id);
    offset = insert_u16(buffer, bufferSize, offset, self->carrier_noise_ratio);
    offset = insert_s16(buffer, bufferSize, offset, self->azimuth);
    offset = insert_s16(buffer, bufferSize, offset, self->elevation);
    offset = insert_mip_gnss_sv_info_data_svflags(buffer, bufferSize, offset, self->sv_flags);
    offset = insert_mip_gnss_sv_info_data_valid_flags(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_mip_gnss_sv_info_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_gnss_sv_info_data* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->channel);
    offset = extract_u8(buffer, bufferSize, offset, &self->sv_id);
    offset = extract_u16(buffer, bufferSize, offset, &self->carrier_noise_ratio);
    offset = extract_s16(buffer, bufferSize, offset, &self->azimuth);
    offset = extract_s16(buffer, bufferSize, offset, &self->elevation);
    offset = extract_mip_gnss_sv_info_data_svflags(buffer, bufferSize, offset, &self->sv_flags);
    offset = extract_mip_gnss_sv_info_data_valid_flags(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_gnss_hw_status_data_receiver_state(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_gnss_hw_status_data_receiver_state self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}
size_t extract_mip_gnss_hw_status_data_receiver_state(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_gnss_hw_status_data_receiver_state* self)
{
    uint8_t tmp;
    offset = extract_u8(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}

size_t insert_mip_gnss_hw_status_data_antenna_state(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_gnss_hw_status_data_antenna_state self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}
size_t extract_mip_gnss_hw_status_data_antenna_state(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_gnss_hw_status_data_antenna_state* self)
{
    uint8_t tmp;
    offset = extract_u8(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}

size_t insert_mip_gnss_hw_status_data_antenna_power(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_gnss_hw_status_data_antenna_power self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}
size_t extract_mip_gnss_hw_status_data_antenna_power(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_gnss_hw_status_data_antenna_power* self)
{
    uint8_t tmp;
    offset = extract_u8(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}

size_t insert_mip_gnss_hw_status_data_valid_flags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_gnss_hw_status_data_valid_flags self)
{
    return insert_u16(buffer, bufferSize, offset, self);
}
size_t extract_mip_gnss_hw_status_data_valid_flags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_gnss_hw_status_data_valid_flags* self)
{
    uint16_t tmp;
    offset = extract_u16(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_mip_gnss_hw_status_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_gnss_hw_status_data* self)
{
    offset = insert_mip_gnss_hw_status_data_receiver_state(buffer, bufferSize, offset, self->receiver_state);
    offset = insert_mip_gnss_hw_status_data_antenna_state(buffer, bufferSize, offset, self->antenna_state);
    offset = insert_mip_gnss_hw_status_data_antenna_power(buffer, bufferSize, offset, self->antenna_power);
    offset = insert_mip_gnss_hw_status_data_valid_flags(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_mip_gnss_hw_status_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_gnss_hw_status_data* self)
{
    offset = extract_mip_gnss_hw_status_data_receiver_state(buffer, bufferSize, offset, &self->receiver_state);
    offset = extract_mip_gnss_hw_status_data_antenna_state(buffer, bufferSize, offset, &self->antenna_state);
    offset = extract_mip_gnss_hw_status_data_antenna_power(buffer, bufferSize, offset, &self->antenna_power);
    offset = extract_mip_gnss_hw_status_data_valid_flags(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_gnss_dgps_info_data_valid_flags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_gnss_dgps_info_data_valid_flags self)
{
    return insert_u16(buffer, bufferSize, offset, self);
}
size_t extract_mip_gnss_dgps_info_data_valid_flags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_gnss_dgps_info_data_valid_flags* self)
{
    uint16_t tmp;
    offset = extract_u16(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_mip_gnss_dgps_info_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_gnss_dgps_info_data* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->sv_id);
    offset = insert_float(buffer, bufferSize, offset, self->age);
    offset = insert_float(buffer, bufferSize, offset, self->range_correction);
    offset = insert_float(buffer, bufferSize, offset, self->range_rate_correction);
    offset = insert_mip_gnss_dgps_info_data_valid_flags(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_mip_gnss_dgps_info_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_gnss_dgps_info_data* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->sv_id);
    offset = extract_float(buffer, bufferSize, offset, &self->age);
    offset = extract_float(buffer, bufferSize, offset, &self->range_correction);
    offset = extract_float(buffer, bufferSize, offset, &self->range_rate_correction);
    offset = extract_mip_gnss_dgps_info_data_valid_flags(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_gnss_dgps_channel_data_valid_flags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_gnss_dgps_channel_data_valid_flags self)
{
    return insert_u16(buffer, bufferSize, offset, self);
}
size_t extract_mip_gnss_dgps_channel_data_valid_flags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_gnss_dgps_channel_data_valid_flags* self)
{
    uint16_t tmp;
    offset = extract_u16(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_mip_gnss_dgps_channel_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_gnss_dgps_channel_data* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->sv_id);
    offset = insert_float(buffer, bufferSize, offset, self->age);
    offset = insert_float(buffer, bufferSize, offset, self->range_correction);
    offset = insert_float(buffer, bufferSize, offset, self->range_rate_correction);
    offset = insert_mip_gnss_dgps_channel_data_valid_flags(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_mip_gnss_dgps_channel_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_gnss_dgps_channel_data* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->sv_id);
    offset = extract_float(buffer, bufferSize, offset, &self->age);
    offset = extract_float(buffer, bufferSize, offset, &self->range_correction);
    offset = extract_float(buffer, bufferSize, offset, &self->range_rate_correction);
    offset = extract_mip_gnss_dgps_channel_data_valid_flags(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_gnss_clock_info_2_data_valid_flags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_gnss_clock_info_2_data_valid_flags self)
{
    return insert_u16(buffer, bufferSize, offset, self);
}
size_t extract_mip_gnss_clock_info_2_data_valid_flags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_gnss_clock_info_2_data_valid_flags* self)
{
    uint16_t tmp;
    offset = extract_u16(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_mip_gnss_clock_info_2_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_gnss_clock_info_2_data* self)
{
    offset = insert_double(buffer, bufferSize, offset, self->bias);
    offset = insert_double(buffer, bufferSize, offset, self->drift);
    offset = insert_double(buffer, bufferSize, offset, self->bias_accuracy_estimate);
    offset = insert_double(buffer, bufferSize, offset, self->drift_accuracy_estimate);
    offset = insert_mip_gnss_clock_info_2_data_valid_flags(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_mip_gnss_clock_info_2_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_gnss_clock_info_2_data* self)
{
    offset = extract_double(buffer, bufferSize, offset, &self->bias);
    offset = extract_double(buffer, bufferSize, offset, &self->drift);
    offset = extract_double(buffer, bufferSize, offset, &self->bias_accuracy_estimate);
    offset = extract_double(buffer, bufferSize, offset, &self->drift_accuracy_estimate);
    offset = extract_mip_gnss_clock_info_2_data_valid_flags(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_gnss_gps_leap_seconds_data_valid_flags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_gnss_gps_leap_seconds_data_valid_flags self)
{
    return insert_u16(buffer, bufferSize, offset, self);
}
size_t extract_mip_gnss_gps_leap_seconds_data_valid_flags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_gnss_gps_leap_seconds_data_valid_flags* self)
{
    uint16_t tmp;
    offset = extract_u16(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_mip_gnss_gps_leap_seconds_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_gnss_gps_leap_seconds_data* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->leap_seconds);
    offset = insert_mip_gnss_gps_leap_seconds_data_valid_flags(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_mip_gnss_gps_leap_seconds_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_gnss_gps_leap_seconds_data* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->leap_seconds);
    offset = extract_mip_gnss_gps_leap_seconds_data_valid_flags(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_gnss_sbas_info_data_sbas_status(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_gnss_sbas_info_data_sbas_status self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}
size_t extract_mip_gnss_sbas_info_data_sbas_status(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_gnss_sbas_info_data_sbas_status* self)
{
    uint8_t tmp;
    offset = extract_u8(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_mip_gnss_sbas_info_data_valid_flags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_gnss_sbas_info_data_valid_flags self)
{
    return insert_u16(buffer, bufferSize, offset, self);
}
size_t extract_mip_gnss_sbas_info_data_valid_flags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_gnss_sbas_info_data_valid_flags* self)
{
    uint16_t tmp;
    offset = extract_u16(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_mip_gnss_sbas_info_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_gnss_sbas_info_data* self)
{
    offset = insert_double(buffer, bufferSize, offset, self->time_of_week);
    offset = insert_u16(buffer, bufferSize, offset, self->week_number);
    offset = insert_mip_sbas_system(buffer, bufferSize, offset, self->sbas_system);
    offset = insert_u8(buffer, bufferSize, offset, self->sbas_id);
    offset = insert_u8(buffer, bufferSize, offset, self->count);
    offset = insert_mip_gnss_sbas_info_data_sbas_status(buffer, bufferSize, offset, self->sbas_status);
    offset = insert_mip_gnss_sbas_info_data_valid_flags(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_mip_gnss_sbas_info_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_gnss_sbas_info_data* self)
{
    offset = extract_double(buffer, bufferSize, offset, &self->time_of_week);
    offset = extract_u16(buffer, bufferSize, offset, &self->week_number);
    offset = extract_mip_sbas_system(buffer, bufferSize, offset, &self->sbas_system);
    offset = extract_u8(buffer, bufferSize, offset, &self->sbas_id);
    offset = extract_u8(buffer, bufferSize, offset, &self->count);
    offset = extract_mip_gnss_sbas_info_data_sbas_status(buffer, bufferSize, offset, &self->sbas_status);
    offset = extract_mip_gnss_sbas_info_data_valid_flags(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_gnss_sbas_correction_data_valid_flags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_gnss_sbas_correction_data_valid_flags self)
{
    return insert_u16(buffer, bufferSize, offset, self);
}
size_t extract_mip_gnss_sbas_correction_data_valid_flags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_gnss_sbas_correction_data_valid_flags* self)
{
    uint16_t tmp;
    offset = extract_u16(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_mip_gnss_sbas_correction_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_gnss_sbas_correction_data* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->index);
    offset = insert_u8(buffer, bufferSize, offset, self->count);
    offset = insert_double(buffer, bufferSize, offset, self->time_of_week);
    offset = insert_u16(buffer, bufferSize, offset, self->week_number);
    offset = insert_mip_gnss_constellation_id(buffer, bufferSize, offset, self->gnss_id);
    offset = insert_u8(buffer, bufferSize, offset, self->sv_id);
    offset = insert_u8(buffer, bufferSize, offset, self->udrei);
    offset = insert_float(buffer, bufferSize, offset, self->pseudorange_correction);
    offset = insert_float(buffer, bufferSize, offset, self->iono_correction);
    offset = insert_mip_gnss_sbas_correction_data_valid_flags(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_mip_gnss_sbas_correction_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_gnss_sbas_correction_data* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->index);
    offset = extract_u8(buffer, bufferSize, offset, &self->count);
    offset = extract_double(buffer, bufferSize, offset, &self->time_of_week);
    offset = extract_u16(buffer, bufferSize, offset, &self->week_number);
    offset = extract_mip_gnss_constellation_id(buffer, bufferSize, offset, &self->gnss_id);
    offset = extract_u8(buffer, bufferSize, offset, &self->sv_id);
    offset = extract_u8(buffer, bufferSize, offset, &self->udrei);
    offset = extract_float(buffer, bufferSize, offset, &self->pseudorange_correction);
    offset = extract_float(buffer, bufferSize, offset, &self->iono_correction);
    offset = extract_mip_gnss_sbas_correction_data_valid_flags(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_gnss_rf_error_detection_data_rfband(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_gnss_rf_error_detection_data_rfband self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}
size_t extract_mip_gnss_rf_error_detection_data_rfband(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_gnss_rf_error_detection_data_rfband* self)
{
    uint8_t tmp;
    offset = extract_u8(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}

size_t insert_mip_gnss_rf_error_detection_data_jamming_state(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_gnss_rf_error_detection_data_jamming_state self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}
size_t extract_mip_gnss_rf_error_detection_data_jamming_state(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_gnss_rf_error_detection_data_jamming_state* self)
{
    uint8_t tmp;
    offset = extract_u8(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}

size_t insert_mip_gnss_rf_error_detection_data_spoofing_state(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_gnss_rf_error_detection_data_spoofing_state self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}
size_t extract_mip_gnss_rf_error_detection_data_spoofing_state(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_gnss_rf_error_detection_data_spoofing_state* self)
{
    uint8_t tmp;
    offset = extract_u8(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}

size_t insert_mip_gnss_rf_error_detection_data_valid_flags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_gnss_rf_error_detection_data_valid_flags self)
{
    return insert_u16(buffer, bufferSize, offset, self);
}
size_t extract_mip_gnss_rf_error_detection_data_valid_flags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_gnss_rf_error_detection_data_valid_flags* self)
{
    uint16_t tmp;
    offset = extract_u16(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_mip_gnss_rf_error_detection_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_gnss_rf_error_detection_data* self)
{
    offset = insert_mip_gnss_rf_error_detection_data_rfband(buffer, bufferSize, offset, self->rf_band);
    offset = insert_mip_gnss_rf_error_detection_data_jamming_state(buffer, bufferSize, offset, self->jamming_state);
    offset = insert_mip_gnss_rf_error_detection_data_spoofing_state(buffer, bufferSize, offset, self->spoofing_state);
    for(unsigned int i=0; i < 4; i++)
        offset = insert_u8(buffer, bufferSize, offset, self->reserved[i]);
    offset = insert_mip_gnss_rf_error_detection_data_valid_flags(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_mip_gnss_rf_error_detection_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_gnss_rf_error_detection_data* self)
{
    offset = extract_mip_gnss_rf_error_detection_data_rfband(buffer, bufferSize, offset, &self->rf_band);
    offset = extract_mip_gnss_rf_error_detection_data_jamming_state(buffer, bufferSize, offset, &self->jamming_state);
    offset = extract_mip_gnss_rf_error_detection_data_spoofing_state(buffer, bufferSize, offset, &self->spoofing_state);
    for(unsigned int i=0; i < 4; i++)
        offset = extract_u8(buffer, bufferSize, offset, &self->reserved[i]);
    offset = extract_mip_gnss_rf_error_detection_data_valid_flags(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_gnss_base_station_info_data_indicator_flags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_gnss_base_station_info_data_indicator_flags self)
{
    return insert_u16(buffer, bufferSize, offset, self);
}
size_t extract_mip_gnss_base_station_info_data_indicator_flags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_gnss_base_station_info_data_indicator_flags* self)
{
    uint16_t tmp;
    offset = extract_u16(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_mip_gnss_base_station_info_data_valid_flags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_gnss_base_station_info_data_valid_flags self)
{
    return insert_u16(buffer, bufferSize, offset, self);
}
size_t extract_mip_gnss_base_station_info_data_valid_flags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_gnss_base_station_info_data_valid_flags* self)
{
    uint16_t tmp;
    offset = extract_u16(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_mip_gnss_base_station_info_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_gnss_base_station_info_data* self)
{
    offset = insert_double(buffer, bufferSize, offset, self->time_of_week);
    offset = insert_u16(buffer, bufferSize, offset, self->week_number);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_double(buffer, bufferSize, offset, self->ecef_pos[i]);
    offset = insert_float(buffer, bufferSize, offset, self->height);
    offset = insert_u16(buffer, bufferSize, offset, self->station_id);
    offset = insert_mip_gnss_base_station_info_data_indicator_flags(buffer, bufferSize, offset, self->indicators);
    offset = insert_mip_gnss_base_station_info_data_valid_flags(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_mip_gnss_base_station_info_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_gnss_base_station_info_data* self)
{
    offset = extract_double(buffer, bufferSize, offset, &self->time_of_week);
    offset = extract_u16(buffer, bufferSize, offset, &self->week_number);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_double(buffer, bufferSize, offset, &self->ecef_pos[i]);
    offset = extract_float(buffer, bufferSize, offset, &self->height);
    offset = extract_u16(buffer, bufferSize, offset, &self->station_id);
    offset = extract_mip_gnss_base_station_info_data_indicator_flags(buffer, bufferSize, offset, &self->indicators);
    offset = extract_mip_gnss_base_station_info_data_valid_flags(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_gnss_rtk_corrections_status_data_epoch_status(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_gnss_rtk_corrections_status_data_epoch_status self)
{
    return insert_u16(buffer, bufferSize, offset, self);
}
size_t extract_mip_gnss_rtk_corrections_status_data_epoch_status(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_gnss_rtk_corrections_status_data_epoch_status* self)
{
    uint16_t tmp;
    offset = extract_u16(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_mip_gnss_rtk_corrections_status_data_valid_flags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_gnss_rtk_corrections_status_data_valid_flags self)
{
    return insert_u16(buffer, bufferSize, offset, self);
}
size_t extract_mip_gnss_rtk_corrections_status_data_valid_flags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_gnss_rtk_corrections_status_data_valid_flags* self)
{
    uint16_t tmp;
    offset = extract_u16(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_mip_gnss_rtk_corrections_status_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_gnss_rtk_corrections_status_data* self)
{
    offset = insert_double(buffer, bufferSize, offset, self->time_of_week);
    offset = insert_u16(buffer, bufferSize, offset, self->week_number);
    offset = insert_mip_gnss_rtk_corrections_status_data_epoch_status(buffer, bufferSize, offset, self->epoch_status);
    offset = insert_u32(buffer, bufferSize, offset, self->dongle_status);
    offset = insert_float(buffer, bufferSize, offset, self->gps_correction_latency);
    offset = insert_float(buffer, bufferSize, offset, self->glonass_correction_latency);
    offset = insert_float(buffer, bufferSize, offset, self->galileo_correction_latency);
    offset = insert_float(buffer, bufferSize, offset, self->beidou_correction_latency);
    for(unsigned int i=0; i < 4; i++)
        offset = insert_u32(buffer, bufferSize, offset, self->reserved[i]);
    offset = insert_mip_gnss_rtk_corrections_status_data_valid_flags(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_mip_gnss_rtk_corrections_status_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_gnss_rtk_corrections_status_data* self)
{
    offset = extract_double(buffer, bufferSize, offset, &self->time_of_week);
    offset = extract_u16(buffer, bufferSize, offset, &self->week_number);
    offset = extract_mip_gnss_rtk_corrections_status_data_epoch_status(buffer, bufferSize, offset, &self->epoch_status);
    offset = extract_u32(buffer, bufferSize, offset, &self->dongle_status);
    offset = extract_float(buffer, bufferSize, offset, &self->gps_correction_latency);
    offset = extract_float(buffer, bufferSize, offset, &self->glonass_correction_latency);
    offset = extract_float(buffer, bufferSize, offset, &self->galileo_correction_latency);
    offset = extract_float(buffer, bufferSize, offset, &self->beidou_correction_latency);
    for(unsigned int i=0; i < 4; i++)
        offset = extract_u32(buffer, bufferSize, offset, &self->reserved[i]);
    offset = extract_mip_gnss_rtk_corrections_status_data_valid_flags(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_gnss_satellite_status_data_valid_flags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_gnss_satellite_status_data_valid_flags self)
{
    return insert_u16(buffer, bufferSize, offset, self);
}
size_t extract_mip_gnss_satellite_status_data_valid_flags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_gnss_satellite_status_data_valid_flags* self)
{
    uint16_t tmp;
    offset = extract_u16(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_mip_gnss_satellite_status_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_gnss_satellite_status_data* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->index);
    offset = insert_u8(buffer, bufferSize, offset, self->count);
    offset = insert_double(buffer, bufferSize, offset, self->time_of_week);
    offset = insert_u16(buffer, bufferSize, offset, self->week_number);
    offset = insert_mip_gnss_constellation_id(buffer, bufferSize, offset, self->gnss_id);
    offset = insert_u8(buffer, bufferSize, offset, self->satellite_id);
    offset = insert_float(buffer, bufferSize, offset, self->elevation);
    offset = insert_float(buffer, bufferSize, offset, self->azimuth);
    offset = insert_bool(buffer, bufferSize, offset, self->health);
    offset = insert_mip_gnss_satellite_status_data_valid_flags(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_mip_gnss_satellite_status_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_gnss_satellite_status_data* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->index);
    offset = extract_u8(buffer, bufferSize, offset, &self->count);
    offset = extract_double(buffer, bufferSize, offset, &self->time_of_week);
    offset = extract_u16(buffer, bufferSize, offset, &self->week_number);
    offset = extract_mip_gnss_constellation_id(buffer, bufferSize, offset, &self->gnss_id);
    offset = extract_u8(buffer, bufferSize, offset, &self->satellite_id);
    offset = extract_float(buffer, bufferSize, offset, &self->elevation);
    offset = extract_float(buffer, bufferSize, offset, &self->azimuth);
    offset = extract_bool(buffer, bufferSize, offset, &self->health);
    offset = extract_mip_gnss_satellite_status_data_valid_flags(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_gnss_raw_data_gnss_signal_quality(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_gnss_raw_data_gnss_signal_quality self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}
size_t extract_mip_gnss_raw_data_gnss_signal_quality(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_gnss_raw_data_gnss_signal_quality* self)
{
    uint8_t tmp;
    offset = extract_u8(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}

size_t insert_mip_gnss_raw_data_valid_flags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_gnss_raw_data_valid_flags self)
{
    return insert_u16(buffer, bufferSize, offset, self);
}
size_t extract_mip_gnss_raw_data_valid_flags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_gnss_raw_data_valid_flags* self)
{
    uint16_t tmp;
    offset = extract_u16(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_mip_gnss_raw_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_gnss_raw_data* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->index);
    offset = insert_u8(buffer, bufferSize, offset, self->count);
    offset = insert_double(buffer, bufferSize, offset, self->time_of_week);
    offset = insert_u16(buffer, bufferSize, offset, self->week_number);
    offset = insert_u16(buffer, bufferSize, offset, self->receiver_id);
    offset = insert_u8(buffer, bufferSize, offset, self->tracking_channel);
    offset = insert_mip_gnss_constellation_id(buffer, bufferSize, offset, self->gnss_id);
    offset = insert_u8(buffer, bufferSize, offset, self->satellite_id);
    offset = insert_mip_gnss_signal_id(buffer, bufferSize, offset, self->signal_id);
    offset = insert_float(buffer, bufferSize, offset, self->signal_strength);
    offset = insert_mip_gnss_raw_data_gnss_signal_quality(buffer, bufferSize, offset, self->quality);
    offset = insert_double(buffer, bufferSize, offset, self->pseudorange);
    offset = insert_double(buffer, bufferSize, offset, self->carrier_phase);
    offset = insert_float(buffer, bufferSize, offset, self->doppler);
    offset = insert_float(buffer, bufferSize, offset, self->range_uncert);
    offset = insert_float(buffer, bufferSize, offset, self->phase_uncert);
    offset = insert_float(buffer, bufferSize, offset, self->doppler_uncert);
    offset = insert_float(buffer, bufferSize, offset, self->lock_time);
    offset = insert_mip_gnss_raw_data_valid_flags(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_mip_gnss_raw_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_gnss_raw_data* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->index);
    offset = extract_u8(buffer, bufferSize, offset, &self->count);
    offset = extract_double(buffer, bufferSize, offset, &self->time_of_week);
    offset = extract_u16(buffer, bufferSize, offset, &self->week_number);
    offset = extract_u16(buffer, bufferSize, offset, &self->receiver_id);
    offset = extract_u8(buffer, bufferSize, offset, &self->tracking_channel);
    offset = extract_mip_gnss_constellation_id(buffer, bufferSize, offset, &self->gnss_id);
    offset = extract_u8(buffer, bufferSize, offset, &self->satellite_id);
    offset = extract_mip_gnss_signal_id(buffer, bufferSize, offset, &self->signal_id);
    offset = extract_float(buffer, bufferSize, offset, &self->signal_strength);
    offset = extract_mip_gnss_raw_data_gnss_signal_quality(buffer, bufferSize, offset, &self->quality);
    offset = extract_double(buffer, bufferSize, offset, &self->pseudorange);
    offset = extract_double(buffer, bufferSize, offset, &self->carrier_phase);
    offset = extract_float(buffer, bufferSize, offset, &self->doppler);
    offset = extract_float(buffer, bufferSize, offset, &self->range_uncert);
    offset = extract_float(buffer, bufferSize, offset, &self->phase_uncert);
    offset = extract_float(buffer, bufferSize, offset, &self->doppler_uncert);
    offset = extract_float(buffer, bufferSize, offset, &self->lock_time);
    offset = extract_mip_gnss_raw_data_valid_flags(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_gnss_gps_ephemeris_data_valid_flags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_gnss_gps_ephemeris_data_valid_flags self)
{
    return insert_u16(buffer, bufferSize, offset, self);
}
size_t extract_mip_gnss_gps_ephemeris_data_valid_flags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_gnss_gps_ephemeris_data_valid_flags* self)
{
    uint16_t tmp;
    offset = extract_u16(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_mip_gnss_gps_ephemeris_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_gnss_gps_ephemeris_data* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->index);
    offset = insert_u8(buffer, bufferSize, offset, self->count);
    offset = insert_double(buffer, bufferSize, offset, self->time_of_week);
    offset = insert_u16(buffer, bufferSize, offset, self->week_number);
    offset = insert_u8(buffer, bufferSize, offset, self->satellite_id);
    offset = insert_u8(buffer, bufferSize, offset, self->health);
    offset = insert_u8(buffer, bufferSize, offset, self->iodc);
    offset = insert_u8(buffer, bufferSize, offset, self->iode);
    offset = insert_double(buffer, bufferSize, offset, self->t_oc);
    offset = insert_double(buffer, bufferSize, offset, self->af0);
    offset = insert_double(buffer, bufferSize, offset, self->af1);
    offset = insert_double(buffer, bufferSize, offset, self->af2);
    offset = insert_double(buffer, bufferSize, offset, self->t_gd);
    offset = insert_double(buffer, bufferSize, offset, self->ISC_L1CA);
    offset = insert_double(buffer, bufferSize, offset, self->ISC_L2C);
    offset = insert_double(buffer, bufferSize, offset, self->t_oe);
    offset = insert_double(buffer, bufferSize, offset, self->a);
    offset = insert_double(buffer, bufferSize, offset, self->a_dot);
    offset = insert_double(buffer, bufferSize, offset, self->mean_anomaly);
    offset = insert_double(buffer, bufferSize, offset, self->delta_mean_motion);
    offset = insert_double(buffer, bufferSize, offset, self->delta_mean_motion_dot);
    offset = insert_double(buffer, bufferSize, offset, self->eccentricity);
    offset = insert_double(buffer, bufferSize, offset, self->argument_of_perigee);
    offset = insert_double(buffer, bufferSize, offset, self->omega);
    offset = insert_double(buffer, bufferSize, offset, self->omega_dot);
    offset = insert_double(buffer, bufferSize, offset, self->inclination);
    offset = insert_double(buffer, bufferSize, offset, self->inclination_dot);
    offset = insert_double(buffer, bufferSize, offset, self->c_ic);
    offset = insert_double(buffer, bufferSize, offset, self->c_is);
    offset = insert_double(buffer, bufferSize, offset, self->c_uc);
    offset = insert_double(buffer, bufferSize, offset, self->c_us);
    offset = insert_double(buffer, bufferSize, offset, self->c_rc);
    offset = insert_double(buffer, bufferSize, offset, self->c_rs);
    offset = insert_mip_gnss_gps_ephemeris_data_valid_flags(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_mip_gnss_gps_ephemeris_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_gnss_gps_ephemeris_data* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->index);
    offset = extract_u8(buffer, bufferSize, offset, &self->count);
    offset = extract_double(buffer, bufferSize, offset, &self->time_of_week);
    offset = extract_u16(buffer, bufferSize, offset, &self->week_number);
    offset = extract_u8(buffer, bufferSize, offset, &self->satellite_id);
    offset = extract_u8(buffer, bufferSize, offset, &self->health);
    offset = extract_u8(buffer, bufferSize, offset, &self->iodc);
    offset = extract_u8(buffer, bufferSize, offset, &self->iode);
    offset = extract_double(buffer, bufferSize, offset, &self->t_oc);
    offset = extract_double(buffer, bufferSize, offset, &self->af0);
    offset = extract_double(buffer, bufferSize, offset, &self->af1);
    offset = extract_double(buffer, bufferSize, offset, &self->af2);
    offset = extract_double(buffer, bufferSize, offset, &self->t_gd);
    offset = extract_double(buffer, bufferSize, offset, &self->ISC_L1CA);
    offset = extract_double(buffer, bufferSize, offset, &self->ISC_L2C);
    offset = extract_double(buffer, bufferSize, offset, &self->t_oe);
    offset = extract_double(buffer, bufferSize, offset, &self->a);
    offset = extract_double(buffer, bufferSize, offset, &self->a_dot);
    offset = extract_double(buffer, bufferSize, offset, &self->mean_anomaly);
    offset = extract_double(buffer, bufferSize, offset, &self->delta_mean_motion);
    offset = extract_double(buffer, bufferSize, offset, &self->delta_mean_motion_dot);
    offset = extract_double(buffer, bufferSize, offset, &self->eccentricity);
    offset = extract_double(buffer, bufferSize, offset, &self->argument_of_perigee);
    offset = extract_double(buffer, bufferSize, offset, &self->omega);
    offset = extract_double(buffer, bufferSize, offset, &self->omega_dot);
    offset = extract_double(buffer, bufferSize, offset, &self->inclination);
    offset = extract_double(buffer, bufferSize, offset, &self->inclination_dot);
    offset = extract_double(buffer, bufferSize, offset, &self->c_ic);
    offset = extract_double(buffer, bufferSize, offset, &self->c_is);
    offset = extract_double(buffer, bufferSize, offset, &self->c_uc);
    offset = extract_double(buffer, bufferSize, offset, &self->c_us);
    offset = extract_double(buffer, bufferSize, offset, &self->c_rc);
    offset = extract_double(buffer, bufferSize, offset, &self->c_rs);
    offset = extract_mip_gnss_gps_ephemeris_data_valid_flags(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_gnss_glo_ephemeris_data_valid_flags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_gnss_glo_ephemeris_data_valid_flags self)
{
    return insert_u16(buffer, bufferSize, offset, self);
}
size_t extract_mip_gnss_glo_ephemeris_data_valid_flags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_gnss_glo_ephemeris_data_valid_flags* self)
{
    uint16_t tmp;
    offset = extract_u16(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_mip_gnss_glo_ephemeris_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_gnss_glo_ephemeris_data* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->index);
    offset = insert_u8(buffer, bufferSize, offset, self->count);
    offset = insert_double(buffer, bufferSize, offset, self->time_of_week);
    offset = insert_u16(buffer, bufferSize, offset, self->week_number);
    offset = insert_u8(buffer, bufferSize, offset, self->satellite_id);
    offset = insert_s8(buffer, bufferSize, offset, self->freq_number);
    offset = insert_u32(buffer, bufferSize, offset, self->tk);
    offset = insert_u32(buffer, bufferSize, offset, self->tb);
    offset = insert_u8(buffer, bufferSize, offset, self->sat_type);
    offset = insert_double(buffer, bufferSize, offset, self->gamma);
    offset = insert_double(buffer, bufferSize, offset, self->tau_n);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_double(buffer, bufferSize, offset, self->x[i]);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->v[i]);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->a[i]);
    offset = insert_u8(buffer, bufferSize, offset, self->health);
    offset = insert_u8(buffer, bufferSize, offset, self->P);
    offset = insert_u8(buffer, bufferSize, offset, self->NT);
    offset = insert_float(buffer, bufferSize, offset, self->delta_tau_n);
    offset = insert_u8(buffer, bufferSize, offset, self->Ft);
    offset = insert_u8(buffer, bufferSize, offset, self->En);
    offset = insert_u8(buffer, bufferSize, offset, self->P1);
    offset = insert_u8(buffer, bufferSize, offset, self->P2);
    offset = insert_u8(buffer, bufferSize, offset, self->P3);
    offset = insert_u8(buffer, bufferSize, offset, self->P4);
    offset = insert_mip_gnss_glo_ephemeris_data_valid_flags(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_mip_gnss_glo_ephemeris_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_gnss_glo_ephemeris_data* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->index);
    offset = extract_u8(buffer, bufferSize, offset, &self->count);
    offset = extract_double(buffer, bufferSize, offset, &self->time_of_week);
    offset = extract_u16(buffer, bufferSize, offset, &self->week_number);
    offset = extract_u8(buffer, bufferSize, offset, &self->satellite_id);
    offset = extract_s8(buffer, bufferSize, offset, &self->freq_number);
    offset = extract_u32(buffer, bufferSize, offset, &self->tk);
    offset = extract_u32(buffer, bufferSize, offset, &self->tb);
    offset = extract_u8(buffer, bufferSize, offset, &self->sat_type);
    offset = extract_double(buffer, bufferSize, offset, &self->gamma);
    offset = extract_double(buffer, bufferSize, offset, &self->tau_n);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_double(buffer, bufferSize, offset, &self->x[i]);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->v[i]);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->a[i]);
    offset = extract_u8(buffer, bufferSize, offset, &self->health);
    offset = extract_u8(buffer, bufferSize, offset, &self->P);
    offset = extract_u8(buffer, bufferSize, offset, &self->NT);
    offset = extract_float(buffer, bufferSize, offset, &self->delta_tau_n);
    offset = extract_u8(buffer, bufferSize, offset, &self->Ft);
    offset = extract_u8(buffer, bufferSize, offset, &self->En);
    offset = extract_u8(buffer, bufferSize, offset, &self->P1);
    offset = extract_u8(buffer, bufferSize, offset, &self->P2);
    offset = extract_u8(buffer, bufferSize, offset, &self->P3);
    offset = extract_u8(buffer, bufferSize, offset, &self->P4);
    offset = extract_mip_gnss_glo_ephemeris_data_valid_flags(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_gnss_gps_iono_corr_data_valid_flags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_gnss_gps_iono_corr_data_valid_flags self)
{
    return insert_u16(buffer, bufferSize, offset, self);
}
size_t extract_mip_gnss_gps_iono_corr_data_valid_flags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_gnss_gps_iono_corr_data_valid_flags* self)
{
    uint16_t tmp;
    offset = extract_u16(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_mip_gnss_gps_iono_corr_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_gnss_gps_iono_corr_data* self)
{
    offset = insert_double(buffer, bufferSize, offset, self->time_of_week);
    offset = insert_u16(buffer, bufferSize, offset, self->week_number);
    for(unsigned int i=0; i < 4; i++)
        offset = insert_double(buffer, bufferSize, offset, self->alpha[i]);
    for(unsigned int i=0; i < 4; i++)
        offset = insert_double(buffer, bufferSize, offset, self->beta[i]);
    offset = insert_mip_gnss_gps_iono_corr_data_valid_flags(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_mip_gnss_gps_iono_corr_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_gnss_gps_iono_corr_data* self)
{
    offset = extract_double(buffer, bufferSize, offset, &self->time_of_week);
    offset = extract_u16(buffer, bufferSize, offset, &self->week_number);
    for(unsigned int i=0; i < 4; i++)
        offset = extract_double(buffer, bufferSize, offset, &self->alpha[i]);
    for(unsigned int i=0; i < 4; i++)
        offset = extract_double(buffer, bufferSize, offset, &self->beta[i]);
    offset = extract_mip_gnss_gps_iono_corr_data_valid_flags(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_gnss_galileo_iono_corr_data_valid_flags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_gnss_galileo_iono_corr_data_valid_flags self)
{
    return insert_u16(buffer, bufferSize, offset, self);
}
size_t extract_mip_gnss_galileo_iono_corr_data_valid_flags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_gnss_galileo_iono_corr_data_valid_flags* self)
{
    uint16_t tmp;
    offset = extract_u16(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_mip_gnss_galileo_iono_corr_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_gnss_galileo_iono_corr_data* self)
{
    offset = insert_double(buffer, bufferSize, offset, self->time_of_week);
    offset = insert_u16(buffer, bufferSize, offset, self->week_number);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_double(buffer, bufferSize, offset, self->alpha[i]);
    offset = insert_u8(buffer, bufferSize, offset, self->disturbance_flags);
    offset = insert_mip_gnss_galileo_iono_corr_data_valid_flags(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_mip_gnss_galileo_iono_corr_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_gnss_galileo_iono_corr_data* self)
{
    offset = extract_double(buffer, bufferSize, offset, &self->time_of_week);
    offset = extract_u16(buffer, bufferSize, offset, &self->week_number);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_double(buffer, bufferSize, offset, &self->alpha[i]);
    offset = extract_u8(buffer, bufferSize, offset, &self->disturbance_flags);
    offset = extract_mip_gnss_galileo_iono_corr_data_valid_flags(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}



#ifdef __cplusplus
} // extern "C"
} // namespace mscl
#endif // __cplusplus
