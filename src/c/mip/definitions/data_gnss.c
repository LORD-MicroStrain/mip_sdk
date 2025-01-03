
#include "data_gnss.h"

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

void insert_mip_gnss_pos_llh_data(microstrain_serializer* serializer, const mip_gnss_pos_llh_data* self)
{
    microstrain_insert_double(serializer, self->latitude);
    
    microstrain_insert_double(serializer, self->longitude);
    
    microstrain_insert_double(serializer, self->ellipsoid_height);
    
    microstrain_insert_double(serializer, self->msl_height);
    
    microstrain_insert_float(serializer, self->horizontal_accuracy);
    
    microstrain_insert_float(serializer, self->vertical_accuracy);
    
    insert_mip_gnss_pos_llh_data_valid_flags(serializer, self->valid_flags);
    
}
void extract_mip_gnss_pos_llh_data(microstrain_serializer* serializer, mip_gnss_pos_llh_data* self)
{
    microstrain_extract_double(serializer, &self->latitude);
    
    microstrain_extract_double(serializer, &self->longitude);
    
    microstrain_extract_double(serializer, &self->ellipsoid_height);
    
    microstrain_extract_double(serializer, &self->msl_height);
    
    microstrain_extract_float(serializer, &self->horizontal_accuracy);
    
    microstrain_extract_float(serializer, &self->vertical_accuracy);
    
    extract_mip_gnss_pos_llh_data_valid_flags(serializer, &self->valid_flags);
    
}
bool extract_mip_gnss_pos_llh_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_gnss_pos_llh_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_gnss_pos_llh_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_gnss_pos_ecef_data(microstrain_serializer* serializer, const mip_gnss_pos_ecef_data* self)
{
    insert_mip_vector3d(serializer, self->x);
    
    microstrain_insert_float(serializer, self->x_accuracy);
    
    insert_mip_gnss_pos_ecef_data_valid_flags(serializer, self->valid_flags);
    
}
void extract_mip_gnss_pos_ecef_data(microstrain_serializer* serializer, mip_gnss_pos_ecef_data* self)
{
    extract_mip_vector3d(serializer, self->x);
    
    microstrain_extract_float(serializer, &self->x_accuracy);
    
    extract_mip_gnss_pos_ecef_data_valid_flags(serializer, &self->valid_flags);
    
}
bool extract_mip_gnss_pos_ecef_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_gnss_pos_ecef_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_gnss_pos_ecef_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_gnss_vel_ned_data(microstrain_serializer* serializer, const mip_gnss_vel_ned_data* self)
{
    insert_mip_vector3f(serializer, self->v);
    
    microstrain_insert_float(serializer, self->speed);
    
    microstrain_insert_float(serializer, self->ground_speed);
    
    microstrain_insert_float(serializer, self->heading);
    
    microstrain_insert_float(serializer, self->speed_accuracy);
    
    microstrain_insert_float(serializer, self->heading_accuracy);
    
    insert_mip_gnss_vel_ned_data_valid_flags(serializer, self->valid_flags);
    
}
void extract_mip_gnss_vel_ned_data(microstrain_serializer* serializer, mip_gnss_vel_ned_data* self)
{
    extract_mip_vector3f(serializer, self->v);
    
    microstrain_extract_float(serializer, &self->speed);
    
    microstrain_extract_float(serializer, &self->ground_speed);
    
    microstrain_extract_float(serializer, &self->heading);
    
    microstrain_extract_float(serializer, &self->speed_accuracy);
    
    microstrain_extract_float(serializer, &self->heading_accuracy);
    
    extract_mip_gnss_vel_ned_data_valid_flags(serializer, &self->valid_flags);
    
}
bool extract_mip_gnss_vel_ned_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_gnss_vel_ned_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_gnss_vel_ned_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_gnss_vel_ecef_data(microstrain_serializer* serializer, const mip_gnss_vel_ecef_data* self)
{
    insert_mip_vector3f(serializer, self->v);
    
    microstrain_insert_float(serializer, self->v_accuracy);
    
    insert_mip_gnss_vel_ecef_data_valid_flags(serializer, self->valid_flags);
    
}
void extract_mip_gnss_vel_ecef_data(microstrain_serializer* serializer, mip_gnss_vel_ecef_data* self)
{
    extract_mip_vector3f(serializer, self->v);
    
    microstrain_extract_float(serializer, &self->v_accuracy);
    
    extract_mip_gnss_vel_ecef_data_valid_flags(serializer, &self->valid_flags);
    
}
bool extract_mip_gnss_vel_ecef_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_gnss_vel_ecef_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_gnss_vel_ecef_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_gnss_dop_data(microstrain_serializer* serializer, const mip_gnss_dop_data* self)
{
    microstrain_insert_float(serializer, self->gdop);
    
    microstrain_insert_float(serializer, self->pdop);
    
    microstrain_insert_float(serializer, self->hdop);
    
    microstrain_insert_float(serializer, self->vdop);
    
    microstrain_insert_float(serializer, self->tdop);
    
    microstrain_insert_float(serializer, self->ndop);
    
    microstrain_insert_float(serializer, self->edop);
    
    insert_mip_gnss_dop_data_valid_flags(serializer, self->valid_flags);
    
}
void extract_mip_gnss_dop_data(microstrain_serializer* serializer, mip_gnss_dop_data* self)
{
    microstrain_extract_float(serializer, &self->gdop);
    
    microstrain_extract_float(serializer, &self->pdop);
    
    microstrain_extract_float(serializer, &self->hdop);
    
    microstrain_extract_float(serializer, &self->vdop);
    
    microstrain_extract_float(serializer, &self->tdop);
    
    microstrain_extract_float(serializer, &self->ndop);
    
    microstrain_extract_float(serializer, &self->edop);
    
    extract_mip_gnss_dop_data_valid_flags(serializer, &self->valid_flags);
    
}
bool extract_mip_gnss_dop_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_gnss_dop_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_gnss_dop_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_gnss_utc_time_data(microstrain_serializer* serializer, const mip_gnss_utc_time_data* self)
{
    microstrain_insert_u16(serializer, self->year);
    
    microstrain_insert_u8(serializer, self->month);
    
    microstrain_insert_u8(serializer, self->day);
    
    microstrain_insert_u8(serializer, self->hour);
    
    microstrain_insert_u8(serializer, self->min);
    
    microstrain_insert_u8(serializer, self->sec);
    
    microstrain_insert_u32(serializer, self->msec);
    
    insert_mip_gnss_utc_time_data_valid_flags(serializer, self->valid_flags);
    
}
void extract_mip_gnss_utc_time_data(microstrain_serializer* serializer, mip_gnss_utc_time_data* self)
{
    microstrain_extract_u16(serializer, &self->year);
    
    microstrain_extract_u8(serializer, &self->month);
    
    microstrain_extract_u8(serializer, &self->day);
    
    microstrain_extract_u8(serializer, &self->hour);
    
    microstrain_extract_u8(serializer, &self->min);
    
    microstrain_extract_u8(serializer, &self->sec);
    
    microstrain_extract_u32(serializer, &self->msec);
    
    extract_mip_gnss_utc_time_data_valid_flags(serializer, &self->valid_flags);
    
}
bool extract_mip_gnss_utc_time_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_gnss_utc_time_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_gnss_utc_time_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_gnss_gps_time_data(microstrain_serializer* serializer, const mip_gnss_gps_time_data* self)
{
    microstrain_insert_double(serializer, self->tow);
    
    microstrain_insert_u16(serializer, self->week_number);
    
    insert_mip_gnss_gps_time_data_valid_flags(serializer, self->valid_flags);
    
}
void extract_mip_gnss_gps_time_data(microstrain_serializer* serializer, mip_gnss_gps_time_data* self)
{
    microstrain_extract_double(serializer, &self->tow);
    
    microstrain_extract_u16(serializer, &self->week_number);
    
    extract_mip_gnss_gps_time_data_valid_flags(serializer, &self->valid_flags);
    
}
bool extract_mip_gnss_gps_time_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_gnss_gps_time_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_gnss_gps_time_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_gnss_clock_info_data(microstrain_serializer* serializer, const mip_gnss_clock_info_data* self)
{
    microstrain_insert_double(serializer, self->bias);
    
    microstrain_insert_double(serializer, self->drift);
    
    microstrain_insert_double(serializer, self->accuracy_estimate);
    
    insert_mip_gnss_clock_info_data_valid_flags(serializer, self->valid_flags);
    
}
void extract_mip_gnss_clock_info_data(microstrain_serializer* serializer, mip_gnss_clock_info_data* self)
{
    microstrain_extract_double(serializer, &self->bias);
    
    microstrain_extract_double(serializer, &self->drift);
    
    microstrain_extract_double(serializer, &self->accuracy_estimate);
    
    extract_mip_gnss_clock_info_data_valid_flags(serializer, &self->valid_flags);
    
}
bool extract_mip_gnss_clock_info_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_gnss_clock_info_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_gnss_clock_info_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_gnss_fix_info_data(microstrain_serializer* serializer, const mip_gnss_fix_info_data* self)
{
    insert_mip_gnss_fix_info_data_fix_type(serializer, self->fix_type);
    
    microstrain_insert_u8(serializer, self->num_sv);
    
    insert_mip_gnss_fix_info_data_fix_flags(serializer, self->fix_flags);
    
    insert_mip_gnss_fix_info_data_valid_flags(serializer, self->valid_flags);
    
}
void extract_mip_gnss_fix_info_data(microstrain_serializer* serializer, mip_gnss_fix_info_data* self)
{
    extract_mip_gnss_fix_info_data_fix_type(serializer, &self->fix_type);
    
    microstrain_extract_u8(serializer, &self->num_sv);
    
    extract_mip_gnss_fix_info_data_fix_flags(serializer, &self->fix_flags);
    
    extract_mip_gnss_fix_info_data_valid_flags(serializer, &self->valid_flags);
    
}
bool extract_mip_gnss_fix_info_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_gnss_fix_info_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_gnss_fix_info_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_gnss_sv_info_data(microstrain_serializer* serializer, const mip_gnss_sv_info_data* self)
{
    microstrain_insert_u8(serializer, self->channel);
    
    microstrain_insert_u8(serializer, self->sv_id);
    
    microstrain_insert_u16(serializer, self->carrier_noise_ratio);
    
    microstrain_insert_s16(serializer, self->azimuth);
    
    microstrain_insert_s16(serializer, self->elevation);
    
    insert_mip_gnss_sv_info_data_svflags(serializer, self->sv_flags);
    
    insert_mip_gnss_sv_info_data_valid_flags(serializer, self->valid_flags);
    
}
void extract_mip_gnss_sv_info_data(microstrain_serializer* serializer, mip_gnss_sv_info_data* self)
{
    microstrain_extract_u8(serializer, &self->channel);
    
    microstrain_extract_u8(serializer, &self->sv_id);
    
    microstrain_extract_u16(serializer, &self->carrier_noise_ratio);
    
    microstrain_extract_s16(serializer, &self->azimuth);
    
    microstrain_extract_s16(serializer, &self->elevation);
    
    extract_mip_gnss_sv_info_data_svflags(serializer, &self->sv_flags);
    
    extract_mip_gnss_sv_info_data_valid_flags(serializer, &self->valid_flags);
    
}
bool extract_mip_gnss_sv_info_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_gnss_sv_info_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_gnss_sv_info_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_gnss_hw_status_data(microstrain_serializer* serializer, const mip_gnss_hw_status_data* self)
{
    insert_mip_gnss_hw_status_data_receiver_state(serializer, self->receiver_state);
    
    insert_mip_gnss_hw_status_data_antenna_state(serializer, self->antenna_state);
    
    insert_mip_gnss_hw_status_data_antenna_power(serializer, self->antenna_power);
    
    insert_mip_gnss_hw_status_data_valid_flags(serializer, self->valid_flags);
    
}
void extract_mip_gnss_hw_status_data(microstrain_serializer* serializer, mip_gnss_hw_status_data* self)
{
    extract_mip_gnss_hw_status_data_receiver_state(serializer, &self->receiver_state);
    
    extract_mip_gnss_hw_status_data_antenna_state(serializer, &self->antenna_state);
    
    extract_mip_gnss_hw_status_data_antenna_power(serializer, &self->antenna_power);
    
    extract_mip_gnss_hw_status_data_valid_flags(serializer, &self->valid_flags);
    
}
bool extract_mip_gnss_hw_status_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_gnss_hw_status_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_gnss_hw_status_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_gnss_dgps_info_data(microstrain_serializer* serializer, const mip_gnss_dgps_info_data* self)
{
    microstrain_insert_u8(serializer, self->sv_id);
    
    microstrain_insert_float(serializer, self->age);
    
    microstrain_insert_float(serializer, self->range_correction);
    
    microstrain_insert_float(serializer, self->range_rate_correction);
    
    insert_mip_gnss_dgps_info_data_valid_flags(serializer, self->valid_flags);
    
}
void extract_mip_gnss_dgps_info_data(microstrain_serializer* serializer, mip_gnss_dgps_info_data* self)
{
    microstrain_extract_u8(serializer, &self->sv_id);
    
    microstrain_extract_float(serializer, &self->age);
    
    microstrain_extract_float(serializer, &self->range_correction);
    
    microstrain_extract_float(serializer, &self->range_rate_correction);
    
    extract_mip_gnss_dgps_info_data_valid_flags(serializer, &self->valid_flags);
    
}
bool extract_mip_gnss_dgps_info_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_gnss_dgps_info_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_gnss_dgps_info_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_gnss_dgps_channel_data(microstrain_serializer* serializer, const mip_gnss_dgps_channel_data* self)
{
    microstrain_insert_u8(serializer, self->sv_id);
    
    microstrain_insert_float(serializer, self->age);
    
    microstrain_insert_float(serializer, self->range_correction);
    
    microstrain_insert_float(serializer, self->range_rate_correction);
    
    insert_mip_gnss_dgps_channel_data_valid_flags(serializer, self->valid_flags);
    
}
void extract_mip_gnss_dgps_channel_data(microstrain_serializer* serializer, mip_gnss_dgps_channel_data* self)
{
    microstrain_extract_u8(serializer, &self->sv_id);
    
    microstrain_extract_float(serializer, &self->age);
    
    microstrain_extract_float(serializer, &self->range_correction);
    
    microstrain_extract_float(serializer, &self->range_rate_correction);
    
    extract_mip_gnss_dgps_channel_data_valid_flags(serializer, &self->valid_flags);
    
}
bool extract_mip_gnss_dgps_channel_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_gnss_dgps_channel_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_gnss_dgps_channel_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_gnss_clock_info_2_data(microstrain_serializer* serializer, const mip_gnss_clock_info_2_data* self)
{
    microstrain_insert_double(serializer, self->bias);
    
    microstrain_insert_double(serializer, self->drift);
    
    microstrain_insert_double(serializer, self->bias_accuracy_estimate);
    
    microstrain_insert_double(serializer, self->drift_accuracy_estimate);
    
    insert_mip_gnss_clock_info_2_data_valid_flags(serializer, self->valid_flags);
    
}
void extract_mip_gnss_clock_info_2_data(microstrain_serializer* serializer, mip_gnss_clock_info_2_data* self)
{
    microstrain_extract_double(serializer, &self->bias);
    
    microstrain_extract_double(serializer, &self->drift);
    
    microstrain_extract_double(serializer, &self->bias_accuracy_estimate);
    
    microstrain_extract_double(serializer, &self->drift_accuracy_estimate);
    
    extract_mip_gnss_clock_info_2_data_valid_flags(serializer, &self->valid_flags);
    
}
bool extract_mip_gnss_clock_info_2_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_gnss_clock_info_2_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_gnss_clock_info_2_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_gnss_gps_leap_seconds_data(microstrain_serializer* serializer, const mip_gnss_gps_leap_seconds_data* self)
{
    microstrain_insert_u8(serializer, self->leap_seconds);
    
    insert_mip_gnss_gps_leap_seconds_data_valid_flags(serializer, self->valid_flags);
    
}
void extract_mip_gnss_gps_leap_seconds_data(microstrain_serializer* serializer, mip_gnss_gps_leap_seconds_data* self)
{
    microstrain_extract_u8(serializer, &self->leap_seconds);
    
    extract_mip_gnss_gps_leap_seconds_data_valid_flags(serializer, &self->valid_flags);
    
}
bool extract_mip_gnss_gps_leap_seconds_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_gnss_gps_leap_seconds_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_gnss_gps_leap_seconds_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_gnss_sbas_info_data(microstrain_serializer* serializer, const mip_gnss_sbas_info_data* self)
{
    microstrain_insert_double(serializer, self->time_of_week);
    
    microstrain_insert_u16(serializer, self->week_number);
    
    insert_mip_sbas_system(serializer, self->sbas_system);
    
    microstrain_insert_u8(serializer, self->sbas_id);
    
    microstrain_insert_u8(serializer, self->count);
    
    insert_mip_gnss_sbas_info_data_sbas_status(serializer, self->sbas_status);
    
    insert_mip_gnss_sbas_info_data_valid_flags(serializer, self->valid_flags);
    
}
void extract_mip_gnss_sbas_info_data(microstrain_serializer* serializer, mip_gnss_sbas_info_data* self)
{
    microstrain_extract_double(serializer, &self->time_of_week);
    
    microstrain_extract_u16(serializer, &self->week_number);
    
    extract_mip_sbas_system(serializer, &self->sbas_system);
    
    microstrain_extract_u8(serializer, &self->sbas_id);
    
    microstrain_extract_u8(serializer, &self->count);
    
    extract_mip_gnss_sbas_info_data_sbas_status(serializer, &self->sbas_status);
    
    extract_mip_gnss_sbas_info_data_valid_flags(serializer, &self->valid_flags);
    
}
bool extract_mip_gnss_sbas_info_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_gnss_sbas_info_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_gnss_sbas_info_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_gnss_sbas_correction_data(microstrain_serializer* serializer, const mip_gnss_sbas_correction_data* self)
{
    microstrain_insert_u8(serializer, self->index);
    
    microstrain_insert_u8(serializer, self->count);
    
    microstrain_insert_double(serializer, self->time_of_week);
    
    microstrain_insert_u16(serializer, self->week_number);
    
    insert_mip_gnss_constellation_id(serializer, self->gnss_id);
    
    microstrain_insert_u8(serializer, self->sv_id);
    
    microstrain_insert_u8(serializer, self->udrei);
    
    microstrain_insert_float(serializer, self->pseudorange_correction);
    
    microstrain_insert_float(serializer, self->iono_correction);
    
    insert_mip_gnss_sbas_correction_data_valid_flags(serializer, self->valid_flags);
    
}
void extract_mip_gnss_sbas_correction_data(microstrain_serializer* serializer, mip_gnss_sbas_correction_data* self)
{
    microstrain_extract_u8(serializer, &self->index);
    
    microstrain_extract_u8(serializer, &self->count);
    
    microstrain_extract_double(serializer, &self->time_of_week);
    
    microstrain_extract_u16(serializer, &self->week_number);
    
    extract_mip_gnss_constellation_id(serializer, &self->gnss_id);
    
    microstrain_extract_u8(serializer, &self->sv_id);
    
    microstrain_extract_u8(serializer, &self->udrei);
    
    microstrain_extract_float(serializer, &self->pseudorange_correction);
    
    microstrain_extract_float(serializer, &self->iono_correction);
    
    extract_mip_gnss_sbas_correction_data_valid_flags(serializer, &self->valid_flags);
    
}
bool extract_mip_gnss_sbas_correction_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_gnss_sbas_correction_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_gnss_sbas_correction_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_gnss_rf_error_detection_data(microstrain_serializer* serializer, const mip_gnss_rf_error_detection_data* self)
{
    insert_mip_gnss_rf_error_detection_data_rfband(serializer, self->rf_band);
    
    insert_mip_gnss_rf_error_detection_data_jamming_state(serializer, self->jamming_state);
    
    insert_mip_gnss_rf_error_detection_data_spoofing_state(serializer, self->spoofing_state);
    
    for(unsigned int i=0; i < 4; i++)
        microstrain_insert_u8(serializer, self->reserved[i]);
    
    insert_mip_gnss_rf_error_detection_data_valid_flags(serializer, self->valid_flags);
    
}
void extract_mip_gnss_rf_error_detection_data(microstrain_serializer* serializer, mip_gnss_rf_error_detection_data* self)
{
    extract_mip_gnss_rf_error_detection_data_rfband(serializer, &self->rf_band);
    
    extract_mip_gnss_rf_error_detection_data_jamming_state(serializer, &self->jamming_state);
    
    extract_mip_gnss_rf_error_detection_data_spoofing_state(serializer, &self->spoofing_state);
    
    for(unsigned int i=0; i < 4; i++)
        microstrain_extract_u8(serializer, &self->reserved[i]);
    
    extract_mip_gnss_rf_error_detection_data_valid_flags(serializer, &self->valid_flags);
    
}
bool extract_mip_gnss_rf_error_detection_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_gnss_rf_error_detection_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_gnss_rf_error_detection_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_gnss_base_station_info_data(microstrain_serializer* serializer, const mip_gnss_base_station_info_data* self)
{
    microstrain_insert_double(serializer, self->time_of_week);
    
    microstrain_insert_u16(serializer, self->week_number);
    
    insert_mip_vector3d(serializer, self->ecef_pos);
    
    microstrain_insert_float(serializer, self->height);
    
    microstrain_insert_u16(serializer, self->station_id);
    
    insert_mip_gnss_base_station_info_data_indicator_flags(serializer, self->indicators);
    
    insert_mip_gnss_base_station_info_data_valid_flags(serializer, self->valid_flags);
    
}
void extract_mip_gnss_base_station_info_data(microstrain_serializer* serializer, mip_gnss_base_station_info_data* self)
{
    microstrain_extract_double(serializer, &self->time_of_week);
    
    microstrain_extract_u16(serializer, &self->week_number);
    
    extract_mip_vector3d(serializer, self->ecef_pos);
    
    microstrain_extract_float(serializer, &self->height);
    
    microstrain_extract_u16(serializer, &self->station_id);
    
    extract_mip_gnss_base_station_info_data_indicator_flags(serializer, &self->indicators);
    
    extract_mip_gnss_base_station_info_data_valid_flags(serializer, &self->valid_flags);
    
}
bool extract_mip_gnss_base_station_info_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_gnss_base_station_info_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_gnss_base_station_info_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_gnss_rtk_corrections_status_data(microstrain_serializer* serializer, const mip_gnss_rtk_corrections_status_data* self)
{
    microstrain_insert_double(serializer, self->time_of_week);
    
    microstrain_insert_u16(serializer, self->week_number);
    
    insert_mip_gnss_rtk_corrections_status_data_epoch_status(serializer, self->epoch_status);
    
    microstrain_insert_u32(serializer, self->dongle_status);
    
    microstrain_insert_float(serializer, self->gps_correction_latency);
    
    microstrain_insert_float(serializer, self->glonass_correction_latency);
    
    microstrain_insert_float(serializer, self->galileo_correction_latency);
    
    microstrain_insert_float(serializer, self->beidou_correction_latency);
    
    for(unsigned int i=0; i < 4; i++)
        microstrain_insert_u32(serializer, self->reserved[i]);
    
    insert_mip_gnss_rtk_corrections_status_data_valid_flags(serializer, self->valid_flags);
    
}
void extract_mip_gnss_rtk_corrections_status_data(microstrain_serializer* serializer, mip_gnss_rtk_corrections_status_data* self)
{
    microstrain_extract_double(serializer, &self->time_of_week);
    
    microstrain_extract_u16(serializer, &self->week_number);
    
    extract_mip_gnss_rtk_corrections_status_data_epoch_status(serializer, &self->epoch_status);
    
    microstrain_extract_u32(serializer, &self->dongle_status);
    
    microstrain_extract_float(serializer, &self->gps_correction_latency);
    
    microstrain_extract_float(serializer, &self->glonass_correction_latency);
    
    microstrain_extract_float(serializer, &self->galileo_correction_latency);
    
    microstrain_extract_float(serializer, &self->beidou_correction_latency);
    
    for(unsigned int i=0; i < 4; i++)
        microstrain_extract_u32(serializer, &self->reserved[i]);
    
    extract_mip_gnss_rtk_corrections_status_data_valid_flags(serializer, &self->valid_flags);
    
}
bool extract_mip_gnss_rtk_corrections_status_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_gnss_rtk_corrections_status_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_gnss_rtk_corrections_status_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_gnss_satellite_status_data(microstrain_serializer* serializer, const mip_gnss_satellite_status_data* self)
{
    microstrain_insert_u8(serializer, self->index);
    
    microstrain_insert_u8(serializer, self->count);
    
    microstrain_insert_double(serializer, self->time_of_week);
    
    microstrain_insert_u16(serializer, self->week_number);
    
    insert_mip_gnss_constellation_id(serializer, self->gnss_id);
    
    microstrain_insert_u8(serializer, self->satellite_id);
    
    microstrain_insert_float(serializer, self->elevation);
    
    microstrain_insert_float(serializer, self->azimuth);
    
    microstrain_insert_bool(serializer, self->health);
    
    insert_mip_gnss_satellite_status_data_valid_flags(serializer, self->valid_flags);
    
}
void extract_mip_gnss_satellite_status_data(microstrain_serializer* serializer, mip_gnss_satellite_status_data* self)
{
    microstrain_extract_u8(serializer, &self->index);
    
    microstrain_extract_u8(serializer, &self->count);
    
    microstrain_extract_double(serializer, &self->time_of_week);
    
    microstrain_extract_u16(serializer, &self->week_number);
    
    extract_mip_gnss_constellation_id(serializer, &self->gnss_id);
    
    microstrain_extract_u8(serializer, &self->satellite_id);
    
    microstrain_extract_float(serializer, &self->elevation);
    
    microstrain_extract_float(serializer, &self->azimuth);
    
    microstrain_extract_bool(serializer, &self->health);
    
    extract_mip_gnss_satellite_status_data_valid_flags(serializer, &self->valid_flags);
    
}
bool extract_mip_gnss_satellite_status_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_gnss_satellite_status_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_gnss_satellite_status_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_gnss_raw_data(microstrain_serializer* serializer, const mip_gnss_raw_data* self)
{
    microstrain_insert_u8(serializer, self->index);
    
    microstrain_insert_u8(serializer, self->count);
    
    microstrain_insert_double(serializer, self->time_of_week);
    
    microstrain_insert_u16(serializer, self->week_number);
    
    microstrain_insert_u16(serializer, self->receiver_id);
    
    microstrain_insert_u8(serializer, self->tracking_channel);
    
    insert_mip_gnss_constellation_id(serializer, self->gnss_id);
    
    microstrain_insert_u8(serializer, self->satellite_id);
    
    insert_mip_gnss_signal_id(serializer, self->signal_id);
    
    microstrain_insert_float(serializer, self->signal_strength);
    
    insert_mip_gnss_raw_data_gnss_signal_quality(serializer, self->quality);
    
    microstrain_insert_double(serializer, self->pseudorange);
    
    microstrain_insert_double(serializer, self->carrier_phase);
    
    microstrain_insert_float(serializer, self->doppler);
    
    microstrain_insert_float(serializer, self->range_uncert);
    
    microstrain_insert_float(serializer, self->phase_uncert);
    
    microstrain_insert_float(serializer, self->doppler_uncert);
    
    microstrain_insert_float(serializer, self->lock_time);
    
    insert_mip_gnss_raw_data_valid_flags(serializer, self->valid_flags);
    
}
void extract_mip_gnss_raw_data(microstrain_serializer* serializer, mip_gnss_raw_data* self)
{
    microstrain_extract_u8(serializer, &self->index);
    
    microstrain_extract_u8(serializer, &self->count);
    
    microstrain_extract_double(serializer, &self->time_of_week);
    
    microstrain_extract_u16(serializer, &self->week_number);
    
    microstrain_extract_u16(serializer, &self->receiver_id);
    
    microstrain_extract_u8(serializer, &self->tracking_channel);
    
    extract_mip_gnss_constellation_id(serializer, &self->gnss_id);
    
    microstrain_extract_u8(serializer, &self->satellite_id);
    
    extract_mip_gnss_signal_id(serializer, &self->signal_id);
    
    microstrain_extract_float(serializer, &self->signal_strength);
    
    extract_mip_gnss_raw_data_gnss_signal_quality(serializer, &self->quality);
    
    microstrain_extract_double(serializer, &self->pseudorange);
    
    microstrain_extract_double(serializer, &self->carrier_phase);
    
    microstrain_extract_float(serializer, &self->doppler);
    
    microstrain_extract_float(serializer, &self->range_uncert);
    
    microstrain_extract_float(serializer, &self->phase_uncert);
    
    microstrain_extract_float(serializer, &self->doppler_uncert);
    
    microstrain_extract_float(serializer, &self->lock_time);
    
    extract_mip_gnss_raw_data_valid_flags(serializer, &self->valid_flags);
    
}
bool extract_mip_gnss_raw_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_gnss_raw_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_gnss_raw_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_gnss_gps_ephemeris_data(microstrain_serializer* serializer, const mip_gnss_gps_ephemeris_data* self)
{
    microstrain_insert_u8(serializer, self->index);
    
    microstrain_insert_u8(serializer, self->count);
    
    microstrain_insert_double(serializer, self->time_of_week);
    
    microstrain_insert_u16(serializer, self->week_number);
    
    microstrain_insert_u8(serializer, self->satellite_id);
    
    microstrain_insert_u8(serializer, self->health);
    
    microstrain_insert_u8(serializer, self->iodc);
    
    microstrain_insert_u8(serializer, self->iode);
    
    microstrain_insert_double(serializer, self->t_oc);
    
    microstrain_insert_double(serializer, self->af0);
    
    microstrain_insert_double(serializer, self->af1);
    
    microstrain_insert_double(serializer, self->af2);
    
    microstrain_insert_double(serializer, self->t_gd);
    
    microstrain_insert_double(serializer, self->ISC_L1CA);
    
    microstrain_insert_double(serializer, self->ISC_L2C);
    
    microstrain_insert_double(serializer, self->t_oe);
    
    microstrain_insert_double(serializer, self->a);
    
    microstrain_insert_double(serializer, self->a_dot);
    
    microstrain_insert_double(serializer, self->mean_anomaly);
    
    microstrain_insert_double(serializer, self->delta_mean_motion);
    
    microstrain_insert_double(serializer, self->delta_mean_motion_dot);
    
    microstrain_insert_double(serializer, self->eccentricity);
    
    microstrain_insert_double(serializer, self->argument_of_perigee);
    
    microstrain_insert_double(serializer, self->omega);
    
    microstrain_insert_double(serializer, self->omega_dot);
    
    microstrain_insert_double(serializer, self->inclination);
    
    microstrain_insert_double(serializer, self->inclination_dot);
    
    microstrain_insert_double(serializer, self->c_ic);
    
    microstrain_insert_double(serializer, self->c_is);
    
    microstrain_insert_double(serializer, self->c_uc);
    
    microstrain_insert_double(serializer, self->c_us);
    
    microstrain_insert_double(serializer, self->c_rc);
    
    microstrain_insert_double(serializer, self->c_rs);
    
    insert_mip_gnss_gps_ephemeris_data_valid_flags(serializer, self->valid_flags);
    
}
void extract_mip_gnss_gps_ephemeris_data(microstrain_serializer* serializer, mip_gnss_gps_ephemeris_data* self)
{
    microstrain_extract_u8(serializer, &self->index);
    
    microstrain_extract_u8(serializer, &self->count);
    
    microstrain_extract_double(serializer, &self->time_of_week);
    
    microstrain_extract_u16(serializer, &self->week_number);
    
    microstrain_extract_u8(serializer, &self->satellite_id);
    
    microstrain_extract_u8(serializer, &self->health);
    
    microstrain_extract_u8(serializer, &self->iodc);
    
    microstrain_extract_u8(serializer, &self->iode);
    
    microstrain_extract_double(serializer, &self->t_oc);
    
    microstrain_extract_double(serializer, &self->af0);
    
    microstrain_extract_double(serializer, &self->af1);
    
    microstrain_extract_double(serializer, &self->af2);
    
    microstrain_extract_double(serializer, &self->t_gd);
    
    microstrain_extract_double(serializer, &self->ISC_L1CA);
    
    microstrain_extract_double(serializer, &self->ISC_L2C);
    
    microstrain_extract_double(serializer, &self->t_oe);
    
    microstrain_extract_double(serializer, &self->a);
    
    microstrain_extract_double(serializer, &self->a_dot);
    
    microstrain_extract_double(serializer, &self->mean_anomaly);
    
    microstrain_extract_double(serializer, &self->delta_mean_motion);
    
    microstrain_extract_double(serializer, &self->delta_mean_motion_dot);
    
    microstrain_extract_double(serializer, &self->eccentricity);
    
    microstrain_extract_double(serializer, &self->argument_of_perigee);
    
    microstrain_extract_double(serializer, &self->omega);
    
    microstrain_extract_double(serializer, &self->omega_dot);
    
    microstrain_extract_double(serializer, &self->inclination);
    
    microstrain_extract_double(serializer, &self->inclination_dot);
    
    microstrain_extract_double(serializer, &self->c_ic);
    
    microstrain_extract_double(serializer, &self->c_is);
    
    microstrain_extract_double(serializer, &self->c_uc);
    
    microstrain_extract_double(serializer, &self->c_us);
    
    microstrain_extract_double(serializer, &self->c_rc);
    
    microstrain_extract_double(serializer, &self->c_rs);
    
    extract_mip_gnss_gps_ephemeris_data_valid_flags(serializer, &self->valid_flags);
    
}
bool extract_mip_gnss_gps_ephemeris_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_gnss_gps_ephemeris_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_gnss_gps_ephemeris_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_gnss_galileo_ephemeris_data(microstrain_serializer* serializer, const mip_gnss_galileo_ephemeris_data* self)
{
    microstrain_insert_u8(serializer, self->index);
    
    microstrain_insert_u8(serializer, self->count);
    
    microstrain_insert_double(serializer, self->time_of_week);
    
    microstrain_insert_u16(serializer, self->week_number);
    
    microstrain_insert_u8(serializer, self->satellite_id);
    
    microstrain_insert_u8(serializer, self->health);
    
    microstrain_insert_u8(serializer, self->iodc);
    
    microstrain_insert_u8(serializer, self->iode);
    
    microstrain_insert_double(serializer, self->t_oc);
    
    microstrain_insert_double(serializer, self->af0);
    
    microstrain_insert_double(serializer, self->af1);
    
    microstrain_insert_double(serializer, self->af2);
    
    microstrain_insert_double(serializer, self->t_gd);
    
    microstrain_insert_double(serializer, self->ISC_L1CA);
    
    microstrain_insert_double(serializer, self->ISC_L2C);
    
    microstrain_insert_double(serializer, self->t_oe);
    
    microstrain_insert_double(serializer, self->a);
    
    microstrain_insert_double(serializer, self->a_dot);
    
    microstrain_insert_double(serializer, self->mean_anomaly);
    
    microstrain_insert_double(serializer, self->delta_mean_motion);
    
    microstrain_insert_double(serializer, self->delta_mean_motion_dot);
    
    microstrain_insert_double(serializer, self->eccentricity);
    
    microstrain_insert_double(serializer, self->argument_of_perigee);
    
    microstrain_insert_double(serializer, self->omega);
    
    microstrain_insert_double(serializer, self->omega_dot);
    
    microstrain_insert_double(serializer, self->inclination);
    
    microstrain_insert_double(serializer, self->inclination_dot);
    
    microstrain_insert_double(serializer, self->c_ic);
    
    microstrain_insert_double(serializer, self->c_is);
    
    microstrain_insert_double(serializer, self->c_uc);
    
    microstrain_insert_double(serializer, self->c_us);
    
    microstrain_insert_double(serializer, self->c_rc);
    
    microstrain_insert_double(serializer, self->c_rs);
    
    insert_mip_gnss_galileo_ephemeris_data_valid_flags(serializer, self->valid_flags);
    
}
void extract_mip_gnss_galileo_ephemeris_data(microstrain_serializer* serializer, mip_gnss_galileo_ephemeris_data* self)
{
    microstrain_extract_u8(serializer, &self->index);
    
    microstrain_extract_u8(serializer, &self->count);
    
    microstrain_extract_double(serializer, &self->time_of_week);
    
    microstrain_extract_u16(serializer, &self->week_number);
    
    microstrain_extract_u8(serializer, &self->satellite_id);
    
    microstrain_extract_u8(serializer, &self->health);
    
    microstrain_extract_u8(serializer, &self->iodc);
    
    microstrain_extract_u8(serializer, &self->iode);
    
    microstrain_extract_double(serializer, &self->t_oc);
    
    microstrain_extract_double(serializer, &self->af0);
    
    microstrain_extract_double(serializer, &self->af1);
    
    microstrain_extract_double(serializer, &self->af2);
    
    microstrain_extract_double(serializer, &self->t_gd);
    
    microstrain_extract_double(serializer, &self->ISC_L1CA);
    
    microstrain_extract_double(serializer, &self->ISC_L2C);
    
    microstrain_extract_double(serializer, &self->t_oe);
    
    microstrain_extract_double(serializer, &self->a);
    
    microstrain_extract_double(serializer, &self->a_dot);
    
    microstrain_extract_double(serializer, &self->mean_anomaly);
    
    microstrain_extract_double(serializer, &self->delta_mean_motion);
    
    microstrain_extract_double(serializer, &self->delta_mean_motion_dot);
    
    microstrain_extract_double(serializer, &self->eccentricity);
    
    microstrain_extract_double(serializer, &self->argument_of_perigee);
    
    microstrain_extract_double(serializer, &self->omega);
    
    microstrain_extract_double(serializer, &self->omega_dot);
    
    microstrain_extract_double(serializer, &self->inclination);
    
    microstrain_extract_double(serializer, &self->inclination_dot);
    
    microstrain_extract_double(serializer, &self->c_ic);
    
    microstrain_extract_double(serializer, &self->c_is);
    
    microstrain_extract_double(serializer, &self->c_uc);
    
    microstrain_extract_double(serializer, &self->c_us);
    
    microstrain_extract_double(serializer, &self->c_rc);
    
    microstrain_extract_double(serializer, &self->c_rs);
    
    extract_mip_gnss_galileo_ephemeris_data_valid_flags(serializer, &self->valid_flags);
    
}
bool extract_mip_gnss_galileo_ephemeris_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_gnss_galileo_ephemeris_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_gnss_galileo_ephemeris_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_gnss_glo_ephemeris_data(microstrain_serializer* serializer, const mip_gnss_glo_ephemeris_data* self)
{
    microstrain_insert_u8(serializer, self->index);
    
    microstrain_insert_u8(serializer, self->count);
    
    microstrain_insert_double(serializer, self->time_of_week);
    
    microstrain_insert_u16(serializer, self->week_number);
    
    microstrain_insert_u8(serializer, self->satellite_id);
    
    microstrain_insert_s8(serializer, self->freq_number);
    
    microstrain_insert_u32(serializer, self->tk);
    
    microstrain_insert_u32(serializer, self->tb);
    
    microstrain_insert_u8(serializer, self->sat_type);
    
    microstrain_insert_double(serializer, self->gamma);
    
    microstrain_insert_double(serializer, self->tau_n);
    
    insert_mip_vector3d(serializer, self->x);
    
    insert_mip_vector3f(serializer, self->v);
    
    insert_mip_vector3f(serializer, self->a);
    
    microstrain_insert_u8(serializer, self->health);
    
    microstrain_insert_u8(serializer, self->P);
    
    microstrain_insert_u8(serializer, self->NT);
    
    microstrain_insert_float(serializer, self->delta_tau_n);
    
    microstrain_insert_u8(serializer, self->Ft);
    
    microstrain_insert_u8(serializer, self->En);
    
    microstrain_insert_u8(serializer, self->P1);
    
    microstrain_insert_u8(serializer, self->P2);
    
    microstrain_insert_u8(serializer, self->P3);
    
    microstrain_insert_u8(serializer, self->P4);
    
    insert_mip_gnss_glo_ephemeris_data_valid_flags(serializer, self->valid_flags);
    
}
void extract_mip_gnss_glo_ephemeris_data(microstrain_serializer* serializer, mip_gnss_glo_ephemeris_data* self)
{
    microstrain_extract_u8(serializer, &self->index);
    
    microstrain_extract_u8(serializer, &self->count);
    
    microstrain_extract_double(serializer, &self->time_of_week);
    
    microstrain_extract_u16(serializer, &self->week_number);
    
    microstrain_extract_u8(serializer, &self->satellite_id);
    
    microstrain_extract_s8(serializer, &self->freq_number);
    
    microstrain_extract_u32(serializer, &self->tk);
    
    microstrain_extract_u32(serializer, &self->tb);
    
    microstrain_extract_u8(serializer, &self->sat_type);
    
    microstrain_extract_double(serializer, &self->gamma);
    
    microstrain_extract_double(serializer, &self->tau_n);
    
    extract_mip_vector3d(serializer, self->x);
    
    extract_mip_vector3f(serializer, self->v);
    
    extract_mip_vector3f(serializer, self->a);
    
    microstrain_extract_u8(serializer, &self->health);
    
    microstrain_extract_u8(serializer, &self->P);
    
    microstrain_extract_u8(serializer, &self->NT);
    
    microstrain_extract_float(serializer, &self->delta_tau_n);
    
    microstrain_extract_u8(serializer, &self->Ft);
    
    microstrain_extract_u8(serializer, &self->En);
    
    microstrain_extract_u8(serializer, &self->P1);
    
    microstrain_extract_u8(serializer, &self->P2);
    
    microstrain_extract_u8(serializer, &self->P3);
    
    microstrain_extract_u8(serializer, &self->P4);
    
    extract_mip_gnss_glo_ephemeris_data_valid_flags(serializer, &self->valid_flags);
    
}
bool extract_mip_gnss_glo_ephemeris_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_gnss_glo_ephemeris_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_gnss_glo_ephemeris_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_gnss_beidou_ephemeris_data(microstrain_serializer* serializer, const mip_gnss_beidou_ephemeris_data* self)
{
    microstrain_insert_u8(serializer, self->index);
    
    microstrain_insert_u8(serializer, self->count);
    
    microstrain_insert_double(serializer, self->time_of_week);
    
    microstrain_insert_u16(serializer, self->week_number);
    
    microstrain_insert_u8(serializer, self->satellite_id);
    
    microstrain_insert_u8(serializer, self->health);
    
    microstrain_insert_u8(serializer, self->iodc);
    
    microstrain_insert_u8(serializer, self->iode);
    
    microstrain_insert_double(serializer, self->t_oc);
    
    microstrain_insert_double(serializer, self->af0);
    
    microstrain_insert_double(serializer, self->af1);
    
    microstrain_insert_double(serializer, self->af2);
    
    microstrain_insert_double(serializer, self->t_gd);
    
    microstrain_insert_double(serializer, self->ISC_L1CA);
    
    microstrain_insert_double(serializer, self->ISC_L2C);
    
    microstrain_insert_double(serializer, self->t_oe);
    
    microstrain_insert_double(serializer, self->a);
    
    microstrain_insert_double(serializer, self->a_dot);
    
    microstrain_insert_double(serializer, self->mean_anomaly);
    
    microstrain_insert_double(serializer, self->delta_mean_motion);
    
    microstrain_insert_double(serializer, self->delta_mean_motion_dot);
    
    microstrain_insert_double(serializer, self->eccentricity);
    
    microstrain_insert_double(serializer, self->argument_of_perigee);
    
    microstrain_insert_double(serializer, self->omega);
    
    microstrain_insert_double(serializer, self->omega_dot);
    
    microstrain_insert_double(serializer, self->inclination);
    
    microstrain_insert_double(serializer, self->inclination_dot);
    
    microstrain_insert_double(serializer, self->c_ic);
    
    microstrain_insert_double(serializer, self->c_is);
    
    microstrain_insert_double(serializer, self->c_uc);
    
    microstrain_insert_double(serializer, self->c_us);
    
    microstrain_insert_double(serializer, self->c_rc);
    
    microstrain_insert_double(serializer, self->c_rs);
    
    insert_mip_gnss_beidou_ephemeris_data_valid_flags(serializer, self->valid_flags);
    
}
void extract_mip_gnss_beidou_ephemeris_data(microstrain_serializer* serializer, mip_gnss_beidou_ephemeris_data* self)
{
    microstrain_extract_u8(serializer, &self->index);
    
    microstrain_extract_u8(serializer, &self->count);
    
    microstrain_extract_double(serializer, &self->time_of_week);
    
    microstrain_extract_u16(serializer, &self->week_number);
    
    microstrain_extract_u8(serializer, &self->satellite_id);
    
    microstrain_extract_u8(serializer, &self->health);
    
    microstrain_extract_u8(serializer, &self->iodc);
    
    microstrain_extract_u8(serializer, &self->iode);
    
    microstrain_extract_double(serializer, &self->t_oc);
    
    microstrain_extract_double(serializer, &self->af0);
    
    microstrain_extract_double(serializer, &self->af1);
    
    microstrain_extract_double(serializer, &self->af2);
    
    microstrain_extract_double(serializer, &self->t_gd);
    
    microstrain_extract_double(serializer, &self->ISC_L1CA);
    
    microstrain_extract_double(serializer, &self->ISC_L2C);
    
    microstrain_extract_double(serializer, &self->t_oe);
    
    microstrain_extract_double(serializer, &self->a);
    
    microstrain_extract_double(serializer, &self->a_dot);
    
    microstrain_extract_double(serializer, &self->mean_anomaly);
    
    microstrain_extract_double(serializer, &self->delta_mean_motion);
    
    microstrain_extract_double(serializer, &self->delta_mean_motion_dot);
    
    microstrain_extract_double(serializer, &self->eccentricity);
    
    microstrain_extract_double(serializer, &self->argument_of_perigee);
    
    microstrain_extract_double(serializer, &self->omega);
    
    microstrain_extract_double(serializer, &self->omega_dot);
    
    microstrain_extract_double(serializer, &self->inclination);
    
    microstrain_extract_double(serializer, &self->inclination_dot);
    
    microstrain_extract_double(serializer, &self->c_ic);
    
    microstrain_extract_double(serializer, &self->c_is);
    
    microstrain_extract_double(serializer, &self->c_uc);
    
    microstrain_extract_double(serializer, &self->c_us);
    
    microstrain_extract_double(serializer, &self->c_rc);
    
    microstrain_extract_double(serializer, &self->c_rs);
    
    extract_mip_gnss_beidou_ephemeris_data_valid_flags(serializer, &self->valid_flags);
    
}
bool extract_mip_gnss_beidou_ephemeris_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_gnss_beidou_ephemeris_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_gnss_beidou_ephemeris_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_gnss_gps_iono_corr_data(microstrain_serializer* serializer, const mip_gnss_gps_iono_corr_data* self)
{
    microstrain_insert_double(serializer, self->time_of_week);
    
    microstrain_insert_u16(serializer, self->week_number);
    
    for(unsigned int i=0; i < 4; i++)
        microstrain_insert_double(serializer, self->alpha[i]);
    
    for(unsigned int i=0; i < 4; i++)
        microstrain_insert_double(serializer, self->beta[i]);
    
    insert_mip_gnss_gps_iono_corr_data_valid_flags(serializer, self->valid_flags);
    
}
void extract_mip_gnss_gps_iono_corr_data(microstrain_serializer* serializer, mip_gnss_gps_iono_corr_data* self)
{
    microstrain_extract_double(serializer, &self->time_of_week);
    
    microstrain_extract_u16(serializer, &self->week_number);
    
    for(unsigned int i=0; i < 4; i++)
        microstrain_extract_double(serializer, &self->alpha[i]);
    
    for(unsigned int i=0; i < 4; i++)
        microstrain_extract_double(serializer, &self->beta[i]);
    
    extract_mip_gnss_gps_iono_corr_data_valid_flags(serializer, &self->valid_flags);
    
}
bool extract_mip_gnss_gps_iono_corr_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_gnss_gps_iono_corr_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_gnss_gps_iono_corr_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_gnss_galileo_iono_corr_data(microstrain_serializer* serializer, const mip_gnss_galileo_iono_corr_data* self)
{
    microstrain_insert_double(serializer, self->time_of_week);
    
    microstrain_insert_u16(serializer, self->week_number);
    
    insert_mip_vector3d(serializer, self->alpha);
    
    microstrain_insert_u8(serializer, self->disturbance_flags);
    
    insert_mip_gnss_galileo_iono_corr_data_valid_flags(serializer, self->valid_flags);
    
}
void extract_mip_gnss_galileo_iono_corr_data(microstrain_serializer* serializer, mip_gnss_galileo_iono_corr_data* self)
{
    microstrain_extract_double(serializer, &self->time_of_week);
    
    microstrain_extract_u16(serializer, &self->week_number);
    
    extract_mip_vector3d(serializer, self->alpha);
    
    microstrain_extract_u8(serializer, &self->disturbance_flags);
    
    extract_mip_gnss_galileo_iono_corr_data_valid_flags(serializer, &self->valid_flags);
    
}
bool extract_mip_gnss_galileo_iono_corr_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_gnss_galileo_iono_corr_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_gnss_galileo_iono_corr_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_gnss_beidou_iono_corr_data(microstrain_serializer* serializer, const mip_gnss_beidou_iono_corr_data* self)
{
    microstrain_insert_double(serializer, self->time_of_week);
    
    microstrain_insert_u16(serializer, self->week_number);
    
    for(unsigned int i=0; i < 4; i++)
        microstrain_insert_double(serializer, self->alpha[i]);
    
    for(unsigned int i=0; i < 4; i++)
        microstrain_insert_double(serializer, self->beta[i]);
    
    for(unsigned int i=0; i < 9; i++)
        microstrain_insert_double(serializer, self->alpha_corr[i]);
    
    insert_mip_gnss_beidou_iono_corr_data_valid_flags(serializer, self->valid_flags);
    
}
void extract_mip_gnss_beidou_iono_corr_data(microstrain_serializer* serializer, mip_gnss_beidou_iono_corr_data* self)
{
    microstrain_extract_double(serializer, &self->time_of_week);
    
    microstrain_extract_u16(serializer, &self->week_number);
    
    for(unsigned int i=0; i < 4; i++)
        microstrain_extract_double(serializer, &self->alpha[i]);
    
    for(unsigned int i=0; i < 4; i++)
        microstrain_extract_double(serializer, &self->beta[i]);
    
    for(unsigned int i=0; i < 9; i++)
        microstrain_extract_double(serializer, &self->alpha_corr[i]);
    
    extract_mip_gnss_beidou_iono_corr_data_valid_flags(serializer, &self->valid_flags);
    
}
bool extract_mip_gnss_beidou_iono_corr_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_gnss_beidou_iono_corr_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_gnss_beidou_iono_corr_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}


#ifdef __cplusplus
} // extern "C"
} // namespace C
} // namespace mip
#endif // __cplusplus

