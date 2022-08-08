
#include "commands_3dm.h"

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

void insert_mip_nmeamessage_format(struct mip_serializer* serializer, const struct mip_nmeamessage_format* self)
{
    insert_mip_nmeamessage_format_message_id(serializer, self->message_id);
    insert_mip_nmeamessage_format_talker_id(serializer, self->talker_id);
    insert_mip_nmeamessage_format_source_id(serializer, self->source_id);
    insert_u16(serializer, self->decimation);
}

void extract_mip_nmeamessage_format(struct mip_serializer* serializer, struct mip_nmeamessage_format* self)
{
    extract_mip_nmeamessage_format_message_id(serializer, &self->message_id);
    extract_mip_nmeamessage_format_talker_id(serializer, &self->talker_id);
    extract_mip_nmeamessage_format_source_id(serializer, &self->source_id);
    extract_u16(serializer, &self->decimation);
}

void insert_mip_sensor_range_type(struct mip_serializer* serializer, const enum mip_sensor_range_type self)
{
    return insert_u8(serializer, (uint8_t)(self));
}
void extract_mip_sensor_range_type(struct mip_serializer* serializer, enum mip_sensor_range_type* self)
{
    uint8_t tmp = 0;
    extract_u8(serializer, &tmp);
    *self = tmp;
}


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

void insert_mip_3dm_poll_imu_message_command(struct mip_serializer* serializer, const struct mip_3dm_poll_imu_message_command* self)
{
    insert_bool(serializer, self->suppress_ack);
    insert_u8(serializer, self->num_descriptors);
    for(unsigned int i=0; i < self->num_descriptors; i++)
        insert_mip_descriptor_rate(serializer, &self->descriptors[i]);
}

void extract_mip_3dm_poll_imu_message_command(struct mip_serializer* serializer, struct mip_3dm_poll_imu_message_command* self)
{
    extract_bool(serializer, &self->suppress_ack);
    extract_count(serializer, &self->num_descriptors, self->num_descriptors);
    for(unsigned int i=0; i < self->num_descriptors; i++)
        extract_mip_descriptor_rate(serializer, &self->descriptors[i]);
}

void insert_mip_3dm_poll_gnss_message_command(struct mip_serializer* serializer, const struct mip_3dm_poll_gnss_message_command* self)
{
    insert_bool(serializer, self->suppress_ack);
    insert_u8(serializer, self->num_descriptors);
    for(unsigned int i=0; i < self->num_descriptors; i++)
        insert_mip_descriptor_rate(serializer, &self->descriptors[i]);
}

void extract_mip_3dm_poll_gnss_message_command(struct mip_serializer* serializer, struct mip_3dm_poll_gnss_message_command* self)
{
    extract_bool(serializer, &self->suppress_ack);
    extract_count(serializer, &self->num_descriptors, self->num_descriptors);
    for(unsigned int i=0; i < self->num_descriptors; i++)
        extract_mip_descriptor_rate(serializer, &self->descriptors[i]);
}

void insert_mip_3dm_poll_filter_message_command(struct mip_serializer* serializer, const struct mip_3dm_poll_filter_message_command* self)
{
    insert_bool(serializer, self->suppress_ack);
    insert_u8(serializer, self->num_descriptors);
    for(unsigned int i=0; i < self->num_descriptors; i++)
        insert_mip_descriptor_rate(serializer, &self->descriptors[i]);
}

void extract_mip_3dm_poll_filter_message_command(struct mip_serializer* serializer, struct mip_3dm_poll_filter_message_command* self)
{
    extract_bool(serializer, &self->suppress_ack);
    extract_count(serializer, &self->num_descriptors, self->num_descriptors);
    for(unsigned int i=0; i < self->num_descriptors; i++)
        extract_mip_descriptor_rate(serializer, &self->descriptors[i]);
}

void insert_mip_3dm_imu_message_format_command(struct mip_serializer* serializer, const struct mip_3dm_imu_message_format_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    insert_u8(serializer, self->num_descriptors);
    for(unsigned int i=0; i < self->num_descriptors; i++)
        insert_mip_descriptor_rate(serializer, &self->descriptors[i]);
}

void extract_mip_3dm_imu_message_format_command(struct mip_serializer* serializer, struct mip_3dm_imu_message_format_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    extract_count(serializer, &self->num_descriptors, self->num_descriptors);
    for(unsigned int i=0; i < self->num_descriptors; i++)
        extract_mip_descriptor_rate(serializer, &self->descriptors[i]);
}

void insert_mip_3dm_gps_message_format_command(struct mip_serializer* serializer, const struct mip_3dm_gps_message_format_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    insert_u8(serializer, self->num_descriptors);
    for(unsigned int i=0; i < self->num_descriptors; i++)
        insert_mip_descriptor_rate(serializer, &self->descriptors[i]);
}

void extract_mip_3dm_gps_message_format_command(struct mip_serializer* serializer, struct mip_3dm_gps_message_format_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    extract_count(serializer, &self->num_descriptors, self->num_descriptors);
    for(unsigned int i=0; i < self->num_descriptors; i++)
        extract_mip_descriptor_rate(serializer, &self->descriptors[i]);
}

void insert_mip_3dm_filter_message_format_command(struct mip_serializer* serializer, const struct mip_3dm_filter_message_format_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    insert_u8(serializer, self->num_descriptors);
    for(unsigned int i=0; i < self->num_descriptors; i++)
        insert_mip_descriptor_rate(serializer, &self->descriptors[i]);
}

void extract_mip_3dm_filter_message_format_command(struct mip_serializer* serializer, struct mip_3dm_filter_message_format_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    extract_count(serializer, &self->num_descriptors, self->num_descriptors);
    for(unsigned int i=0; i < self->num_descriptors; i++)
        extract_mip_descriptor_rate(serializer, &self->descriptors[i]);
}

void insert_mip_3dm_poll_data_command(struct mip_serializer* serializer, const struct mip_3dm_poll_data_command* self)
{
    insert_u8(serializer, self->desc_set);
    insert_bool(serializer, self->suppress_ack);
    insert_u8(serializer, self->num_descriptors);
    for(unsigned int i=0; i < self->num_descriptors; i++)
        insert_u8(serializer, self->descriptors[i]);
}

void extract_mip_3dm_poll_data_command(struct mip_serializer* serializer, struct mip_3dm_poll_data_command* self)
{
    extract_u8(serializer, &self->desc_set);
    extract_bool(serializer, &self->suppress_ack);
    extract_count(serializer, &self->num_descriptors, self->num_descriptors);
    for(unsigned int i=0; i < self->num_descriptors; i++)
        extract_u8(serializer, &self->descriptors[i]);
}

void insert_mip_3dm_get_base_rate_command(struct mip_serializer* serializer, const struct mip_3dm_get_base_rate_command* self)
{
    insert_u8(serializer, self->desc_set);
}

void extract_mip_3dm_get_base_rate_command(struct mip_serializer* serializer, struct mip_3dm_get_base_rate_command* self)
{
    extract_u8(serializer, &self->desc_set);
}

void insert_mip_3dm_message_format_command(struct mip_serializer* serializer, const struct mip_3dm_message_format_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    insert_u8(serializer, self->desc_set);
    insert_u8(serializer, self->num_descriptors);
    for(unsigned int i=0; i < self->num_descriptors; i++)
        insert_mip_descriptor_rate(serializer, &self->descriptors[i]);
}

void extract_mip_3dm_message_format_command(struct mip_serializer* serializer, struct mip_3dm_message_format_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    extract_u8(serializer, &self->desc_set);
    extract_count(serializer, &self->num_descriptors, self->num_descriptors);
    for(unsigned int i=0; i < self->num_descriptors; i++)
        extract_mip_descriptor_rate(serializer, &self->descriptors[i]);
}

void insert_mip_3dm_nmea_poll_data_command(struct mip_serializer* serializer, const struct mip_3dm_nmea_poll_data_command* self)
{
    insert_bool(serializer, self->suppress_ack);
    insert_u8(serializer, self->count);
    for(unsigned int i=0; i < self->count; i++)
        insert_mip_nmeamessage_format(serializer, &self->format_entries[i]);
}

void extract_mip_3dm_nmea_poll_data_command(struct mip_serializer* serializer, struct mip_3dm_nmea_poll_data_command* self)
{
    extract_bool(serializer, &self->suppress_ack);
    extract_count(serializer, &self->count, self->count);
    for(unsigned int i=0; i < self->count; i++)
        extract_mip_nmeamessage_format(serializer, &self->format_entries[i]);
}

void insert_mip_3dm_nmea_message_format_command(struct mip_serializer* serializer, const struct mip_3dm_nmea_message_format_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    insert_u8(serializer, self->count);
    for(unsigned int i=0; i < self->count; i++)
        insert_mip_nmeamessage_format(serializer, &self->format_entries[i]);
}

void extract_mip_3dm_nmea_message_format_command(struct mip_serializer* serializer, struct mip_3dm_nmea_message_format_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    extract_count(serializer, &self->count, self->count);
    for(unsigned int i=0; i < self->count; i++)
        extract_mip_nmeamessage_format(serializer, &self->format_entries[i]);
}

void insert_mip_3dm_device_settings_command(struct mip_serializer* serializer, const struct mip_3dm_device_settings_command* self)
{
    insert_mip_function_selector(serializer, self->function);
}

void extract_mip_3dm_device_settings_command(struct mip_serializer* serializer, struct mip_3dm_device_settings_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
}

void insert_mip_3dm_uart_baudrate_command(struct mip_serializer* serializer, const struct mip_3dm_uart_baudrate_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    insert_u32(serializer, self->baud);
}

void extract_mip_3dm_uart_baudrate_command(struct mip_serializer* serializer, struct mip_3dm_uart_baudrate_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    extract_u32(serializer, &self->baud);
}

void insert_mip_3dm_factory_streaming_command(struct mip_serializer* serializer, const struct mip_3dm_factory_streaming_command* self)
{
    insert_mip_3dm_factory_streaming_command_action(serializer, self->action);
    insert_u8(serializer, self->reserved);
}

void extract_mip_3dm_factory_streaming_command(struct mip_serializer* serializer, struct mip_3dm_factory_streaming_command* self)
{
    extract_mip_3dm_factory_streaming_command_action(serializer, &self->action);
    extract_u8(serializer, &self->reserved);
}

void insert_mip_3dm_datastream_control_command(struct mip_serializer* serializer, const struct mip_3dm_datastream_control_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    insert_u8(serializer, self->desc_set);
    insert_bool(serializer, self->enable);
}

void extract_mip_3dm_datastream_control_command(struct mip_serializer* serializer, struct mip_3dm_datastream_control_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    extract_u8(serializer, &self->desc_set);
    extract_bool(serializer, &self->enable);
}

void insert_mip_3dm_gnss_sbas_settings_command(struct mip_serializer* serializer, const struct mip_3dm_gnss_sbas_settings_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    insert_u8(serializer, self->enable_sbas);
    insert_mip_3dm_gnss_sbas_settings_command_sbasoptions(serializer, self->sbas_options);
    insert_u8(serializer, self->num_included_prns);
    for(unsigned int i=0; i < self->num_included_prns; i++)
        insert_u16(serializer, self->included_prns[i]);
}

void extract_mip_3dm_gnss_sbas_settings_command(struct mip_serializer* serializer, struct mip_3dm_gnss_sbas_settings_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    extract_u8(serializer, &self->enable_sbas);
    extract_mip_3dm_gnss_sbas_settings_command_sbasoptions(serializer, &self->sbas_options);
    extract_count(serializer, &self->num_included_prns, self->num_included_prns);
    for(unsigned int i=0; i < self->num_included_prns; i++)
        extract_u16(serializer, &self->included_prns[i]);
}

void insert_mip_3dm_gnss_time_assistance_command(struct mip_serializer* serializer, const struct mip_3dm_gnss_time_assistance_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    insert_double(serializer, self->tow);
    insert_u16(serializer, self->week_number);
    insert_float(serializer, self->accuracy);
}

void extract_mip_3dm_gnss_time_assistance_command(struct mip_serializer* serializer, struct mip_3dm_gnss_time_assistance_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    extract_double(serializer, &self->tow);
    extract_u16(serializer, &self->week_number);
    extract_float(serializer, &self->accuracy);
}

void insert_mip_3dm_adv_lowpass_filter_command(struct mip_serializer* serializer, const struct mip_3dm_adv_lowpass_filter_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    insert_u8(serializer, self->target_descriptor);
    insert_bool(serializer, self->enable);
    insert_bool(serializer, self->manual);
    insert_u16(serializer, self->frequency);
    insert_u8(serializer, self->reserved);
}

void extract_mip_3dm_adv_lowpass_filter_command(struct mip_serializer* serializer, struct mip_3dm_adv_lowpass_filter_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    extract_u8(serializer, &self->target_descriptor);
    extract_bool(serializer, &self->enable);
    extract_bool(serializer, &self->manual);
    extract_u16(serializer, &self->frequency);
    extract_u8(serializer, &self->reserved);
}

void insert_mip_3dm_pps_source_command(struct mip_serializer* serializer, const struct mip_3dm_pps_source_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    insert_mip_3dm_pps_source_command_source(serializer, self->source);
}

void extract_mip_3dm_pps_source_command(struct mip_serializer* serializer, struct mip_3dm_pps_source_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    extract_mip_3dm_pps_source_command_source(serializer, &self->source);
}

void insert_mip_3dm_gpio_config_command(struct mip_serializer* serializer, const struct mip_3dm_gpio_config_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    insert_u8(serializer, self->pin);
    insert_mip_3dm_gpio_config_command_feature(serializer, self->feature);
    insert_mip_3dm_gpio_config_command_behavior(serializer, self->behavior);
    insert_mip_3dm_gpio_config_command_pin_mode(serializer, self->pin_mode);
}

void extract_mip_3dm_gpio_config_command(struct mip_serializer* serializer, struct mip_3dm_gpio_config_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    extract_u8(serializer, &self->pin);
    extract_mip_3dm_gpio_config_command_feature(serializer, &self->feature);
    extract_mip_3dm_gpio_config_command_behavior(serializer, &self->behavior);
    extract_mip_3dm_gpio_config_command_pin_mode(serializer, &self->pin_mode);
}

void insert_mip_3dm_gpio_state_command(struct mip_serializer* serializer, const struct mip_3dm_gpio_state_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    insert_u8(serializer, self->pin);
    insert_bool(serializer, self->state);
}

void extract_mip_3dm_gpio_state_command(struct mip_serializer* serializer, struct mip_3dm_gpio_state_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    extract_u8(serializer, &self->pin);
    extract_bool(serializer, &self->state);
}

void insert_mip_3dm_odometer_command(struct mip_serializer* serializer, const struct mip_3dm_odometer_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    insert_mip_3dm_odometer_command_mode(serializer, self->mode);
    insert_float(serializer, self->scaling);
    insert_float(serializer, self->uncertainty);
}

void extract_mip_3dm_odometer_command(struct mip_serializer* serializer, struct mip_3dm_odometer_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    extract_mip_3dm_odometer_command_mode(serializer, &self->mode);
    extract_float(serializer, &self->scaling);
    extract_float(serializer, &self->uncertainty);
}

void insert_mip_3dm_get_event_support_command(struct mip_serializer* serializer, const struct mip_3dm_get_event_support_command* self)
{
    insert_mip_3dm_get_event_support_command_query(serializer, self->query);
}

void extract_mip_3dm_get_event_support_command(struct mip_serializer* serializer, struct mip_3dm_get_event_support_command* self)
{
    extract_mip_3dm_get_event_support_command_query(serializer, &self->query);
}

void insert_mip_3dm_event_control_command(struct mip_serializer* serializer, const struct mip_3dm_event_control_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    insert_u8(serializer, self->instance);
    insert_mip_3dm_event_control_command_mode(serializer, self->mode);
}

void extract_mip_3dm_event_control_command(struct mip_serializer* serializer, struct mip_3dm_event_control_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    extract_u8(serializer, &self->instance);
    extract_mip_3dm_event_control_command_mode(serializer, &self->mode);
}

void insert_mip_3dm_get_event_trigger_status_command(struct mip_serializer* serializer, const struct mip_3dm_get_event_trigger_status_command* self)
{
    insert_u8(serializer, self->requested_count);
    for(unsigned int i=0; i < self->requested_count; i++)
        insert_u8(serializer, self->requested_instances[i]);
}

void extract_mip_3dm_get_event_trigger_status_command(struct mip_serializer* serializer, struct mip_3dm_get_event_trigger_status_command* self)
{
    extract_count(serializer, &self->requested_count, self->requested_count);
    for(unsigned int i=0; i < self->requested_count; i++)
        extract_u8(serializer, &self->requested_instances[i]);
}

void insert_mip_3dm_get_event_action_status_command(struct mip_serializer* serializer, const struct mip_3dm_get_event_action_status_command* self)
{
    insert_u8(serializer, self->requested_count);
    for(unsigned int i=0; i < self->requested_count; i++)
        insert_u8(serializer, self->requested_instances[i]);
}

void extract_mip_3dm_get_event_action_status_command(struct mip_serializer* serializer, struct mip_3dm_get_event_action_status_command* self)
{
    extract_count(serializer, &self->requested_count, self->requested_count);
    for(unsigned int i=0; i < self->requested_count; i++)
        extract_u8(serializer, &self->requested_instances[i]);
}

void insert_mip_3dm_event_trigger_command(struct mip_serializer* serializer, const struct mip_3dm_event_trigger_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    insert_u8(serializer, self->instance);
    insert_mip_3dm_event_trigger_command_type(serializer, self->type);
    if( self->type == MIP_3DM_EVENT_TRIGGER_COMMAND_TYPE_GPIO )
        insert_mip_3dm_event_trigger_command_gpio_params(serializer, &self->gpio);
    if( self->type == MIP_3DM_EVENT_TRIGGER_COMMAND_TYPE_THRESHOLD )
        insert_mip_3dm_event_trigger_command_threshold_params(serializer, &self->threshold);
    if( self->type == MIP_3DM_EVENT_TRIGGER_COMMAND_TYPE_COMBINATION )
        insert_mip_3dm_event_trigger_command_combination_params(serializer, &self->combination);
}

void extract_mip_3dm_event_trigger_command(struct mip_serializer* serializer, struct mip_3dm_event_trigger_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    extract_u8(serializer, &self->instance);
    extract_mip_3dm_event_trigger_command_type(serializer, &self->type);
    if( self->type == MIP_3DM_EVENT_TRIGGER_COMMAND_TYPE_GPIO )
        extract_mip_3dm_event_trigger_command_gpio_params(serializer, &self->gpio);
    if( self->type == MIP_3DM_EVENT_TRIGGER_COMMAND_TYPE_THRESHOLD )
        extract_mip_3dm_event_trigger_command_threshold_params(serializer, &self->threshold);
    if( self->type == MIP_3DM_EVENT_TRIGGER_COMMAND_TYPE_COMBINATION )
        extract_mip_3dm_event_trigger_command_combination_params(serializer, &self->combination);
}

void insert_mip_3dm_event_action_command(struct mip_serializer* serializer, const struct mip_3dm_event_action_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    insert_u8(serializer, self->instance);
    insert_u8(serializer, self->trigger);
    insert_mip_3dm_event_action_command_type(serializer, self->type);
    if( self->type == MIP_3DM_EVENT_ACTION_COMMAND_TYPE_GPIO )
        insert_mip_3dm_event_action_command_gpio_params(serializer, &self->gpio);
    if( self->type == MIP_3DM_EVENT_ACTION_COMMAND_TYPE_MESSAGE )
        insert_mip_3dm_event_action_command_message_params(serializer, &self->message);
}

void extract_mip_3dm_event_action_command(struct mip_serializer* serializer, struct mip_3dm_event_action_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    extract_u8(serializer, &self->instance);
    extract_u8(serializer, &self->trigger);
    extract_mip_3dm_event_action_command_type(serializer, &self->type);
    if( self->type == MIP_3DM_EVENT_ACTION_COMMAND_TYPE_GPIO )
        extract_mip_3dm_event_action_command_gpio_params(serializer, &self->gpio);
    if( self->type == MIP_3DM_EVENT_ACTION_COMMAND_TYPE_MESSAGE )
        extract_mip_3dm_event_action_command_message_params(serializer, &self->message);
}

void insert_mip_3dm_accel_bias_command(struct mip_serializer* serializer, const struct mip_3dm_accel_bias_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->bias[i]);
}

void extract_mip_3dm_accel_bias_command(struct mip_serializer* serializer, struct mip_3dm_accel_bias_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->bias[i]);
}

void insert_mip_3dm_gyro_bias_command(struct mip_serializer* serializer, const struct mip_3dm_gyro_bias_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->bias[i]);
}

void extract_mip_3dm_gyro_bias_command(struct mip_serializer* serializer, struct mip_3dm_gyro_bias_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->bias[i]);
}

void insert_mip_3dm_capture_gyro_bias_command(struct mip_serializer* serializer, const struct mip_3dm_capture_gyro_bias_command* self)
{
    insert_u16(serializer, self->averaging_time_ms);
}

void extract_mip_3dm_capture_gyro_bias_command(struct mip_serializer* serializer, struct mip_3dm_capture_gyro_bias_command* self)
{
    extract_u16(serializer, &self->averaging_time_ms);
}

void insert_mip_3dm_mag_hard_iron_offset_command(struct mip_serializer* serializer, const struct mip_3dm_mag_hard_iron_offset_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->offset[i]);
}

void extract_mip_3dm_mag_hard_iron_offset_command(struct mip_serializer* serializer, struct mip_3dm_mag_hard_iron_offset_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->offset[i]);
}

void insert_mip_3dm_mag_soft_iron_matrix_command(struct mip_serializer* serializer, const struct mip_3dm_mag_soft_iron_matrix_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    for(unsigned int i=0; i < 9; i++)
        insert_float(serializer, self->offset[i]);
}

void extract_mip_3dm_mag_soft_iron_matrix_command(struct mip_serializer* serializer, struct mip_3dm_mag_soft_iron_matrix_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    for(unsigned int i=0; i < 9; i++)
        extract_float(serializer, &self->offset[i]);
}

void insert_mip_3dm_sensor_2_vehicle_transform_euler_command(struct mip_serializer* serializer, const struct mip_3dm_sensor_2_vehicle_transform_euler_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    insert_float(serializer, self->roll);
    insert_float(serializer, self->pitch);
    insert_float(serializer, self->yaw);
}

void extract_mip_3dm_sensor_2_vehicle_transform_euler_command(struct mip_serializer* serializer, struct mip_3dm_sensor_2_vehicle_transform_euler_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    extract_float(serializer, &self->roll);
    extract_float(serializer, &self->pitch);
    extract_float(serializer, &self->yaw);
}

void insert_mip_3dm_sensor_2_vehicle_transform_quaternion_command(struct mip_serializer* serializer, const struct mip_3dm_sensor_2_vehicle_transform_quaternion_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    for(unsigned int i=0; i < 4; i++)
        insert_float(serializer, self->q[i]);
}

void extract_mip_3dm_sensor_2_vehicle_transform_quaternion_command(struct mip_serializer* serializer, struct mip_3dm_sensor_2_vehicle_transform_quaternion_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    for(unsigned int i=0; i < 4; i++)
        extract_float(serializer, &self->q[i]);
}

void insert_mip_3dm_sensor_2_vehicle_transform_dcm_command(struct mip_serializer* serializer, const struct mip_3dm_sensor_2_vehicle_transform_dcm_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    for(unsigned int i=0; i < 9; i++)
        insert_float(serializer, self->dcm[i]);
}

void extract_mip_3dm_sensor_2_vehicle_transform_dcm_command(struct mip_serializer* serializer, struct mip_3dm_sensor_2_vehicle_transform_dcm_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    for(unsigned int i=0; i < 9; i++)
        extract_float(serializer, &self->dcm[i]);
}

void insert_mip_3dm_complementary_filter_command(struct mip_serializer* serializer, const struct mip_3dm_complementary_filter_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    insert_bool(serializer, self->pitch_roll_enable);
    insert_bool(serializer, self->heading_enable);
    insert_float(serializer, self->pitch_roll_time_constant);
    insert_float(serializer, self->heading_time_constant);
}

void extract_mip_3dm_complementary_filter_command(struct mip_serializer* serializer, struct mip_3dm_complementary_filter_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    extract_bool(serializer, &self->pitch_roll_enable);
    extract_bool(serializer, &self->heading_enable);
    extract_float(serializer, &self->pitch_roll_time_constant);
    extract_float(serializer, &self->heading_time_constant);
}

void insert_mip_3dm_sensor_range_command(struct mip_serializer* serializer, const struct mip_3dm_sensor_range_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    insert_mip_sensor_range_type(serializer, self->sensor);
    insert_u8(serializer, self->setting);
}

void extract_mip_3dm_sensor_range_command(struct mip_serializer* serializer, struct mip_3dm_sensor_range_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    extract_mip_sensor_range_type(serializer, &self->sensor);
    extract_u8(serializer, &self->setting);
}

void insert_mip_3dm_calibrated_sensor_ranges_command(struct mip_serializer* serializer, const struct mip_3dm_calibrated_sensor_ranges_command* self)
{
    insert_mip_sensor_range_type(serializer, self->sensor);
}

void extract_mip_3dm_calibrated_sensor_ranges_command(struct mip_serializer* serializer, struct mip_3dm_calibrated_sensor_ranges_command* self)
{
    extract_mip_sensor_range_type(serializer, &self->sensor);
}


#ifdef __cplusplus
} // namespace C
} // namespace mscl
} // extern "C"
#endif // __cplusplus

