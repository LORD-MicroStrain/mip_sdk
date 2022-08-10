
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

void insert_mip_nmeamessage_format_message_id(struct mip_serializer* serializer, const enum mip_nmeamessage_format_message_id self)
{
    return insert_u8(serializer, (uint8_t)(self));
}
void extract_mip_nmeamessage_format_message_id(struct mip_serializer* serializer, enum mip_nmeamessage_format_message_id* self)
{
    uint8_t tmp = 0;
    extract_u8(serializer, &tmp);
    *self = tmp;
}

void insert_mip_nmeamessage_format_talker_id(struct mip_serializer* serializer, const enum mip_nmeamessage_format_talker_id self)
{
    return insert_u8(serializer, (uint8_t)(self));
}
void extract_mip_nmeamessage_format_talker_id(struct mip_serializer* serializer, enum mip_nmeamessage_format_talker_id* self)
{
    uint8_t tmp = 0;
    extract_u8(serializer, &tmp);
    *self = tmp;
}

void insert_mip_nmeamessage_format_source_id(struct mip_serializer* serializer, const enum mip_nmeamessage_format_source_id self)
{
    return insert_u8(serializer, (uint8_t)(self));
}
void extract_mip_nmeamessage_format_source_id(struct mip_serializer* serializer, enum mip_nmeamessage_format_source_id* self)
{
    uint8_t tmp = 0;
    extract_u8(serializer, &tmp);
    *self = tmp;
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

/// @brief Poll the device for an IMU message with the specified format
/// 
/// This function polls for an IMU message using the provided format. The resulting message
/// will maintain the order of descriptors sent in the command and any unrecognized
/// descriptors are ignored. If the format is not provided, the device will attempt to use the
/// stored format (set with the Set IMU Message Format command.) If no format is provided
/// and there is no stored format, the device will respond with a NACK. The reply packet contains
/// an ACK/NACK field. The polled data packet is sent separately as an IMU Data packet.
/// @param suppress_ack Suppress the usual ACK/NACK reply.
/// @param num_descriptors Number of descriptors in the descriptor list.
/// @param descriptors Descriptor list.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_poll_imu_message(struct mip_interface* device, bool suppress_ack, uint8_t num_descriptors, const struct mip_descriptor_rate* descriptors)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_bool(&serializer, suppress_ack);
    insert_u8(&serializer, num_descriptors);
    for(unsigned int i=0; i < num_descriptors; i++)
        insert_mip_descriptor_rate(&serializer, &descriptors[i]);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_POLL_IMU_MESSAGE, buffer, serializer.offset);
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

/// @brief Poll the device for an GNSS message with the specified format
/// 
/// This function polls for a GNSS message using the provided format. The resulting message
/// will maintain the order of descriptors sent in the command and any unrecognized
/// descriptors are ignored. If the format is not provided, the device will attempt to use the
/// stored format (set with the Set GNSS Message Format command.) If no format is provided
/// and there is no stored format, the device will respond with a NACK. The reply packet contains
/// an ACK/NACK field. The polled data packet is sent separately as a GNSS Data packet.
/// @param suppress_ack Suppress the usual ACK/NACK reply.
/// @param num_descriptors Number of descriptors in the descriptor list.
/// @param descriptors Descriptor list.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_poll_gnss_message(struct mip_interface* device, bool suppress_ack, uint8_t num_descriptors, const struct mip_descriptor_rate* descriptors)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_bool(&serializer, suppress_ack);
    insert_u8(&serializer, num_descriptors);
    for(unsigned int i=0; i < num_descriptors; i++)
        insert_mip_descriptor_rate(&serializer, &descriptors[i]);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_POLL_GNSS_MESSAGE, buffer, serializer.offset);
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

/// @brief Poll the device for an Estimation Filter message with the specified format
/// 
/// This function polls for an Estimation Filter message using the provided format. The resulting message
/// will maintain the order of descriptors sent in the command and any unrecognized
/// descriptors are ignored. If the format is not provided, the device will attempt to use the
/// stored format (set with the Set Estimation Filter Message Format command.) If no format is provided
/// and there is no stored format, the device will respond with a NACK. The reply packet contains
/// an ACK/NACK field. The polled data packet is sent separately as an Estimation Filter Data packet.
/// @param suppress_ack Suppress the usual ACK/NACK reply.
/// @param num_descriptors Number of descriptors in the format list.
/// @param descriptors Descriptor format list.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_poll_filter_message(struct mip_interface* device, bool suppress_ack, uint8_t num_descriptors, const struct mip_descriptor_rate* descriptors)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_bool(&serializer, suppress_ack);
    insert_u8(&serializer, num_descriptors);
    for(unsigned int i=0; i < num_descriptors; i++)
        insert_mip_descriptor_rate(&serializer, &descriptors[i]);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_POLL_FILTER_MESSAGE, buffer, serializer.offset);
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

/// @brief Set, read, or save the format of the IMU data packet.
/// 
/// The resulting data messages will maintain the order of descriptors sent in the command.
/// @param num_descriptors Number of descriptors
/// @param descriptors Descriptor format list.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_write_imu_message_format(struct mip_interface* device, uint8_t num_descriptors, const struct mip_descriptor_rate* descriptors)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    insert_u8(&serializer, num_descriptors);
    for(unsigned int i=0; i < num_descriptors; i++)
        insert_mip_descriptor_rate(&serializer, &descriptors[i]);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_IMU_MESSAGE_FORMAT, buffer, serializer.offset);
}

/// @brief Set, read, or save the format of the IMU data packet.
/// 
/// The resulting data messages will maintain the order of descriptors sent in the command.
/// @param[out] num_descriptors Number of descriptors
/// @param[out] descriptors Descriptor format list.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_read_imu_message_format(struct mip_interface* device, uint8_t* num_descriptors, struct mip_descriptor_rate* descriptors)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_IMU_MESSAGE_FORMAT, buffer, serializer.offset, MIP_REPLY_DESC_3DM_IMU_MESSAGE_FORMAT, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        struct mip_serializer serializer;
        mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
        
        extract_count(&serializer, num_descriptors, *num_descriptors);
        for(unsigned int i=0; i < *num_descriptors; i++)
            extract_mip_descriptor_rate(&serializer, &descriptors[i]);
        
        if( !mip_serializer_ok(&serializer) )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Set, read, or save the format of the IMU data packet.
/// 
/// The resulting data messages will maintain the order of descriptors sent in the command.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_save_imu_message_format(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_IMU_MESSAGE_FORMAT, buffer, serializer.offset);
}

/// @brief Set, read, or save the format of the IMU data packet.
/// 
/// The resulting data messages will maintain the order of descriptors sent in the command.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_load_imu_message_format(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_IMU_MESSAGE_FORMAT, buffer, serializer.offset);
}

/// @brief Set, read, or save the format of the IMU data packet.
/// 
/// The resulting data messages will maintain the order of descriptors sent in the command.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_default_imu_message_format(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_IMU_MESSAGE_FORMAT, buffer, serializer.offset);
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

/// @brief Set, read, or save the format of the GNSS data packet.
/// 
/// The resulting data messages will maintain the order of descriptors sent in the command.
/// @param num_descriptors Number of descriptors
/// @param descriptors Descriptor format list.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_write_gps_message_format(struct mip_interface* device, uint8_t num_descriptors, const struct mip_descriptor_rate* descriptors)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    insert_u8(&serializer, num_descriptors);
    for(unsigned int i=0; i < num_descriptors; i++)
        insert_mip_descriptor_rate(&serializer, &descriptors[i]);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_GNSS_MESSAGE_FORMAT, buffer, serializer.offset);
}

/// @brief Set, read, or save the format of the GNSS data packet.
/// 
/// The resulting data messages will maintain the order of descriptors sent in the command.
/// @param[out] num_descriptors Number of descriptors
/// @param[out] descriptors Descriptor format list.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_read_gps_message_format(struct mip_interface* device, uint8_t* num_descriptors, struct mip_descriptor_rate* descriptors)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_GNSS_MESSAGE_FORMAT, buffer, serializer.offset, MIP_REPLY_DESC_3DM_GNSS_MESSAGE_FORMAT, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        struct mip_serializer serializer;
        mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
        
        extract_count(&serializer, num_descriptors, *num_descriptors);
        for(unsigned int i=0; i < *num_descriptors; i++)
            extract_mip_descriptor_rate(&serializer, &descriptors[i]);
        
        if( !mip_serializer_ok(&serializer) )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Set, read, or save the format of the GNSS data packet.
/// 
/// The resulting data messages will maintain the order of descriptors sent in the command.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_save_gps_message_format(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_GNSS_MESSAGE_FORMAT, buffer, serializer.offset);
}

/// @brief Set, read, or save the format of the GNSS data packet.
/// 
/// The resulting data messages will maintain the order of descriptors sent in the command.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_load_gps_message_format(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_GNSS_MESSAGE_FORMAT, buffer, serializer.offset);
}

/// @brief Set, read, or save the format of the GNSS data packet.
/// 
/// The resulting data messages will maintain the order of descriptors sent in the command.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_default_gps_message_format(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_GNSS_MESSAGE_FORMAT, buffer, serializer.offset);
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

/// @brief Set, read, or save the format of the Estimation Filter data packet.
/// 
/// The resulting data messages will maintain the order of descriptors sent in the command.
/// @param num_descriptors Number of descriptors (limited by payload size)
/// @param descriptors 
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_write_filter_message_format(struct mip_interface* device, uint8_t num_descriptors, const struct mip_descriptor_rate* descriptors)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    insert_u8(&serializer, num_descriptors);
    for(unsigned int i=0; i < num_descriptors; i++)
        insert_mip_descriptor_rate(&serializer, &descriptors[i]);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_FILTER_MESSAGE_FORMAT, buffer, serializer.offset);
}

/// @brief Set, read, or save the format of the Estimation Filter data packet.
/// 
/// The resulting data messages will maintain the order of descriptors sent in the command.
/// @param[out] num_descriptors Number of descriptors (limited by payload size)
/// @param[out] descriptors 
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_read_filter_message_format(struct mip_interface* device, uint8_t* num_descriptors, struct mip_descriptor_rate* descriptors)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_FILTER_MESSAGE_FORMAT, buffer, serializer.offset, MIP_REPLY_DESC_3DM_FILTER_MESSAGE_FORMAT, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        struct mip_serializer serializer;
        mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
        
        extract_count(&serializer, num_descriptors, *num_descriptors);
        for(unsigned int i=0; i < *num_descriptors; i++)
            extract_mip_descriptor_rate(&serializer, &descriptors[i]);
        
        if( !mip_serializer_ok(&serializer) )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Set, read, or save the format of the Estimation Filter data packet.
/// 
/// The resulting data messages will maintain the order of descriptors sent in the command.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_save_filter_message_format(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_FILTER_MESSAGE_FORMAT, buffer, serializer.offset);
}

/// @brief Set, read, or save the format of the Estimation Filter data packet.
/// 
/// The resulting data messages will maintain the order of descriptors sent in the command.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_load_filter_message_format(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_FILTER_MESSAGE_FORMAT, buffer, serializer.offset);
}

/// @brief Set, read, or save the format of the Estimation Filter data packet.
/// 
/// The resulting data messages will maintain the order of descriptors sent in the command.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_default_filter_message_format(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_FILTER_MESSAGE_FORMAT, buffer, serializer.offset);
}

/// @brief Get the base rate for the IMU data in Hz
/// 
/// This is the fastest rate for this type of data available on the device.
/// This is used in conjunction with the IMU Message Format Command to set streaming data at a specified rate.
/// @param[out] rate [hz]
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_imu_get_base_rate(struct mip_interface* device, uint16_t* rate)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_GET_IMU_BASE_RATE, NULL, 0, MIP_REPLY_DESC_3DM_IMU_BASE_RATE, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        struct mip_serializer serializer;
        mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
        
        extract_u16(&serializer, rate);
        
        if( !mip_serializer_ok(&serializer) )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Get the base rate for the GNSS data in Hz
/// 
/// This is the fastest rate for this type of data available on the device.
/// This is used in conjunction with the GNSS Message Format Command to set streaming data at a specified rate.
/// @param[out] rate [hz]
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_gps_get_base_rate(struct mip_interface* device, uint16_t* rate)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_GET_GNSS_BASE_RATE, NULL, 0, MIP_REPLY_DESC_3DM_GNSS_BASE_RATE, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        struct mip_serializer serializer;
        mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
        
        extract_u16(&serializer, rate);
        
        if( !mip_serializer_ok(&serializer) )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Get the base rate for the Estimation Filter data in Hz
/// 
/// This is the fastest rate for this type of data available on the device.
/// This is used in conjunction with the Estimation Filter Message Format Command to set streaming data at a specified rate.
/// @param[out] rate [hz]
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_filter_get_base_rate(struct mip_interface* device, uint16_t* rate)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_GET_FILTER_BASE_RATE, NULL, 0, MIP_REPLY_DESC_3DM_FILTER_BASE_RATE, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        struct mip_serializer serializer;
        mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
        
        extract_u16(&serializer, rate);
        
        if( !mip_serializer_ok(&serializer) )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
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

/// @brief Poll the device for a message with the specified descriptor set and format.
/// 
/// This function polls for a message using the provided format. The resulting message
/// will maintain the order of descriptors sent in the command and any unrecognized
/// descriptors are ignored. If the format is not provided, the device will attempt to use the
/// stored format (set with the Set Message Format command.) If no format is provided
/// and there is no stored format, the device will respond with a NACK. The reply packet contains
/// an ACK/NACK field. The polled data packet is sent separately as a normal Data packet.
/// @param desc_set Data descriptor set. Must be supported.
/// @param suppress_ack Suppress the usual ACK/NACK reply.
/// @param num_descriptors Number of descriptors in the format list.
/// @param descriptors Descriptor format list.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_poll_data(struct mip_interface* device, uint8_t desc_set, bool suppress_ack, uint8_t num_descriptors, const uint8_t* descriptors)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_u8(&serializer, desc_set);
    insert_bool(&serializer, suppress_ack);
    insert_u8(&serializer, num_descriptors);
    for(unsigned int i=0; i < num_descriptors; i++)
        insert_u8(&serializer, descriptors[i]);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_POLL_DATA, buffer, serializer.offset);
}

void insert_mip_3dm_get_base_rate_command(struct mip_serializer* serializer, const struct mip_3dm_get_base_rate_command* self)
{
    insert_u8(serializer, self->desc_set);
}

void extract_mip_3dm_get_base_rate_command(struct mip_serializer* serializer, struct mip_3dm_get_base_rate_command* self)
{
    extract_u8(serializer, &self->desc_set);
}

/// @brief Get the base rate for the specified descriptor set in Hz.
/// 
/// @param desc_set This is the data descriptor set. It must be a supported descriptor.
/// @param[out] desc_set Echoes the parameter in the command.
/// @param[out] rate Base rate in Hz (0 = variable, unknown, or user-defined rate.  Data will be sent when received).
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_get_base_rate(struct mip_interface* device, uint8_t desc_set, uint16_t* rate)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_u8(&serializer, desc_set);
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_GET_BASE_RATE, buffer, serializer.offset, MIP_REPLY_DESC_3DM_BASE_RATE, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        struct mip_serializer serializer;
        mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
        
        extract_u8(&serializer, &desc_set);
        extract_u16(&serializer, rate);
        
        if( !mip_serializer_ok(&serializer) )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
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

/// @brief Set, read, or save the format for a given data packet.
/// 
/// The resulting data messages will maintain the order of descriptors sent in the command.
/// @param desc_set Data descriptor set. Must be supported. When function is SAVE, LOAD, or DEFAULT, can be 0 to apply to all descriptor sets.
/// @param num_descriptors Number of descriptors (limited by payload size)
/// @param descriptors List of descriptors and decimations.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_write_message_format(struct mip_interface* device, uint8_t desc_set, uint8_t num_descriptors, const struct mip_descriptor_rate* descriptors)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    insert_u8(&serializer, desc_set);
    insert_u8(&serializer, num_descriptors);
    for(unsigned int i=0; i < num_descriptors; i++)
        insert_mip_descriptor_rate(&serializer, &descriptors[i]);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_MESSAGE_FORMAT, buffer, serializer.offset);
}

/// @brief Set, read, or save the format for a given data packet.
/// 
/// The resulting data messages will maintain the order of descriptors sent in the command.
/// @param desc_set Data descriptor set. Must be supported. When function is SAVE, LOAD, or DEFAULT, can be 0 to apply to all descriptor sets.
/// @param[out] desc_set Echoes the descriptor set from the command.
/// @param[out] num_descriptors Number of descriptors in the list.
/// @param[out] descriptors List of descriptors and decimations.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_read_message_format(struct mip_interface* device, uint8_t desc_set, uint8_t* num_descriptors, struct mip_descriptor_rate* descriptors)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    insert_u8(&serializer, desc_set);
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_MESSAGE_FORMAT, buffer, serializer.offset, MIP_REPLY_DESC_3DM_MESSAGE_FORMAT, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        struct mip_serializer serializer;
        mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
        
        extract_u8(&serializer, &desc_set);
        extract_count(&serializer, num_descriptors, *num_descriptors);
        for(unsigned int i=0; i < *num_descriptors; i++)
            extract_mip_descriptor_rate(&serializer, &descriptors[i]);
        
        if( !mip_serializer_ok(&serializer) )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Set, read, or save the format for a given data packet.
/// 
/// The resulting data messages will maintain the order of descriptors sent in the command.
/// @param desc_set Data descriptor set. Must be supported. When function is SAVE, LOAD, or DEFAULT, can be 0 to apply to all descriptor sets.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_save_message_format(struct mip_interface* device, uint8_t desc_set)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    insert_u8(&serializer, desc_set);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_MESSAGE_FORMAT, buffer, serializer.offset);
}

/// @brief Set, read, or save the format for a given data packet.
/// 
/// The resulting data messages will maintain the order of descriptors sent in the command.
/// @param desc_set Data descriptor set. Must be supported. When function is SAVE, LOAD, or DEFAULT, can be 0 to apply to all descriptor sets.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_load_message_format(struct mip_interface* device, uint8_t desc_set)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    insert_u8(&serializer, desc_set);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_MESSAGE_FORMAT, buffer, serializer.offset);
}

/// @brief Set, read, or save the format for a given data packet.
/// 
/// The resulting data messages will maintain the order of descriptors sent in the command.
/// @param desc_set Data descriptor set. Must be supported. When function is SAVE, LOAD, or DEFAULT, can be 0 to apply to all descriptor sets.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_default_message_format(struct mip_interface* device, uint8_t desc_set)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    insert_u8(&serializer, desc_set);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_MESSAGE_FORMAT, buffer, serializer.offset);
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

/// @brief Poll the device for a NMEA message with the specified format.
/// 
/// This function polls for a NMEA message using the provided format.
/// If the format is not provided, the device will attempt to use the
/// stored format (set with the Set NMEA Message Format command.) If no format is provided
/// and there is no stored format, the device will respond with a NACK. The reply packet contains
/// an ACK/NACK field. The polled data packet is sent separately as normal NMEA messages.
/// @param suppress_ack Suppress the usual ACK/NACK reply.
/// @param count Number of format entries (limited by payload size)
/// @param format_entries List of format entries.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_nmea_poll_data(struct mip_interface* device, bool suppress_ack, uint8_t count, const struct mip_nmeamessage_format* format_entries)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_bool(&serializer, suppress_ack);
    insert_u8(&serializer, count);
    for(unsigned int i=0; i < count; i++)
        insert_mip_nmeamessage_format(&serializer, &format_entries[i]);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_POLL_NMEA_MESSAGE, buffer, serializer.offset);
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

/// @brief Set, read, or save the NMEA message format.
/// 
/// @param count Number of format entries (limited by payload size)
/// @param format_entries List of format entries.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_write_nmea_message_format(struct mip_interface* device, uint8_t count, const struct mip_nmeamessage_format* format_entries)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    insert_u8(&serializer, count);
    for(unsigned int i=0; i < count; i++)
        insert_mip_nmeamessage_format(&serializer, &format_entries[i]);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_NMEA_MESSAGE_FORMAT, buffer, serializer.offset);
}

/// @brief Set, read, or save the NMEA message format.
/// 
/// @param[out] count Number of format entries (limited by payload size)
/// @param[out] format_entries List of format entries.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_read_nmea_message_format(struct mip_interface* device, uint8_t* count, struct mip_nmeamessage_format* format_entries)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_NMEA_MESSAGE_FORMAT, buffer, serializer.offset, MIP_REPLY_DESC_3DM_NMEA_MESSAGE_FORMAT, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        struct mip_serializer serializer;
        mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
        
        extract_count(&serializer, count, *count);
        for(unsigned int i=0; i < *count; i++)
            extract_mip_nmeamessage_format(&serializer, &format_entries[i]);
        
        if( !mip_serializer_ok(&serializer) )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Set, read, or save the NMEA message format.
/// 
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_save_nmea_message_format(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_NMEA_MESSAGE_FORMAT, buffer, serializer.offset);
}

/// @brief Set, read, or save the NMEA message format.
/// 
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_load_nmea_message_format(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_NMEA_MESSAGE_FORMAT, buffer, serializer.offset);
}

/// @brief Set, read, or save the NMEA message format.
/// 
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_default_nmea_message_format(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_NMEA_MESSAGE_FORMAT, buffer, serializer.offset);
}

void insert_mip_3dm_device_settings_command(struct mip_serializer* serializer, const struct mip_3dm_device_settings_command* self)
{
    insert_mip_function_selector(serializer, self->function);
}

void extract_mip_3dm_device_settings_command(struct mip_serializer* serializer, struct mip_3dm_device_settings_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
}

/// @brief Save, Load, or Reset to Default the values for all device settings.
/// 
/// When a save current settings command is issued, a brief data disturbance may occur while all settings are written to non-volatile memory.
/// 
/// This command should have a long timeout as it may take up to 1 second to complete.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_save_device_settings(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_DEVICE_STARTUP_SETTINGS, buffer, serializer.offset);
}

/// @brief Save, Load, or Reset to Default the values for all device settings.
/// 
/// When a save current settings command is issued, a brief data disturbance may occur while all settings are written to non-volatile memory.
/// 
/// This command should have a long timeout as it may take up to 1 second to complete.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_load_device_settings(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_DEVICE_STARTUP_SETTINGS, buffer, serializer.offset);
}

/// @brief Save, Load, or Reset to Default the values for all device settings.
/// 
/// When a save current settings command is issued, a brief data disturbance may occur while all settings are written to non-volatile memory.
/// 
/// This command should have a long timeout as it may take up to 1 second to complete.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_default_device_settings(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_DEVICE_STARTUP_SETTINGS, buffer, serializer.offset);
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

/// @brief Read, Save, Load, or Reset to Default the baud rate of the main communication channel.
/// 
/// For all functions except 0x01 (use new settings), the new baud rate value is ignored.
/// Please see the device user manual for supported baud rates.
/// 
/// The device will wait until all incoming and outgoing data has been sent, up
/// to a maximum of 250 ms, before applying any change.
/// 
/// No guarantee is provided as to what happens to commands issued during this
/// delay period; They may or may not be processed and any responses aren't
/// guaranteed to be at one rate or the other. The same applies to data packets.
/// 
/// It is highly recommended that the device be idle before issuing this command
/// and that it be issued in its own packet. Users should wait 250 ms after
/// sending this command before further interaction.
/// @param baud 
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_write_uart_baudrate(struct mip_interface* device, uint32_t baud)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    insert_u32(&serializer, baud);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_UART_BAUDRATE, buffer, serializer.offset);
}

/// @brief Read, Save, Load, or Reset to Default the baud rate of the main communication channel.
/// 
/// For all functions except 0x01 (use new settings), the new baud rate value is ignored.
/// Please see the device user manual for supported baud rates.
/// 
/// The device will wait until all incoming and outgoing data has been sent, up
/// to a maximum of 250 ms, before applying any change.
/// 
/// No guarantee is provided as to what happens to commands issued during this
/// delay period; They may or may not be processed and any responses aren't
/// guaranteed to be at one rate or the other. The same applies to data packets.
/// 
/// It is highly recommended that the device be idle before issuing this command
/// and that it be issued in its own packet. Users should wait 250 ms after
/// sending this command before further interaction.
/// @param[out] baud 
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_read_uart_baudrate(struct mip_interface* device, uint32_t* baud)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_UART_BAUDRATE, buffer, serializer.offset, MIP_REPLY_DESC_3DM_UART_BAUDRATE, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        struct mip_serializer serializer;
        mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
        
        extract_u32(&serializer, baud);
        
        if( !mip_serializer_ok(&serializer) )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Read, Save, Load, or Reset to Default the baud rate of the main communication channel.
/// 
/// For all functions except 0x01 (use new settings), the new baud rate value is ignored.
/// Please see the device user manual for supported baud rates.
/// 
/// The device will wait until all incoming and outgoing data has been sent, up
/// to a maximum of 250 ms, before applying any change.
/// 
/// No guarantee is provided as to what happens to commands issued during this
/// delay period; They may or may not be processed and any responses aren't
/// guaranteed to be at one rate or the other. The same applies to data packets.
/// 
/// It is highly recommended that the device be idle before issuing this command
/// and that it be issued in its own packet. Users should wait 250 ms after
/// sending this command before further interaction.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_save_uart_baudrate(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_UART_BAUDRATE, buffer, serializer.offset);
}

/// @brief Read, Save, Load, or Reset to Default the baud rate of the main communication channel.
/// 
/// For all functions except 0x01 (use new settings), the new baud rate value is ignored.
/// Please see the device user manual for supported baud rates.
/// 
/// The device will wait until all incoming and outgoing data has been sent, up
/// to a maximum of 250 ms, before applying any change.
/// 
/// No guarantee is provided as to what happens to commands issued during this
/// delay period; They may or may not be processed and any responses aren't
/// guaranteed to be at one rate or the other. The same applies to data packets.
/// 
/// It is highly recommended that the device be idle before issuing this command
/// and that it be issued in its own packet. Users should wait 250 ms after
/// sending this command before further interaction.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_load_uart_baudrate(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_UART_BAUDRATE, buffer, serializer.offset);
}

/// @brief Read, Save, Load, or Reset to Default the baud rate of the main communication channel.
/// 
/// For all functions except 0x01 (use new settings), the new baud rate value is ignored.
/// Please see the device user manual for supported baud rates.
/// 
/// The device will wait until all incoming and outgoing data has been sent, up
/// to a maximum of 250 ms, before applying any change.
/// 
/// No guarantee is provided as to what happens to commands issued during this
/// delay period; They may or may not be processed and any responses aren't
/// guaranteed to be at one rate or the other. The same applies to data packets.
/// 
/// It is highly recommended that the device be idle before issuing this command
/// and that it be issued in its own packet. Users should wait 250 ms after
/// sending this command before further interaction.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_default_uart_baudrate(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_UART_BAUDRATE, buffer, serializer.offset);
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

void insert_mip_3dm_factory_streaming_command_action(struct mip_serializer* serializer, const enum mip_3dm_factory_streaming_command_action self)
{
    return insert_u8(serializer, (uint8_t)(self));
}
void extract_mip_3dm_factory_streaming_command_action(struct mip_serializer* serializer, enum mip_3dm_factory_streaming_command_action* self)
{
    uint8_t tmp = 0;
    extract_u8(serializer, &tmp);
    *self = tmp;
}

/// @brief Configures the device for recording data for technical support.
/// 
/// This command will configure all available data streams to predefined
/// formats designed to be used with technical support.
/// @param action 
/// @param reserved Reserved. Set to 0x00.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_factory_streaming(struct mip_interface* device, enum mip_3dm_factory_streaming_command_action action, uint8_t reserved)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_3dm_factory_streaming_command_action(&serializer, action);
    insert_u8(&serializer, reserved);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_CONFIGURE_FACTORY_STREAMING, buffer, serializer.offset);
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

/// @brief Enable/disable the selected data stream.
/// 
/// Each data stream (descriptor set) can be enabled or disabled.
/// The default for the device is all streams enabled.
/// For all functions except 0x01 (use new setting),
/// the new enable flag value is ignored and can be omitted.
/// @param desc_set The descriptor set of the stream to control. When function is SAVE, LOAD, or DEFAULT, can be ALL_STREAMS(0) to apply to all descriptor sets. On Generation 5 products, this must be one of the above legacy constants.
/// @param enable True or false to enable or disable the stream.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_write_datastream_control(struct mip_interface* device, uint8_t desc_set, bool enable)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    insert_u8(&serializer, desc_set);
    insert_bool(&serializer, enable);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_CONTROL_DATA_STREAM, buffer, serializer.offset);
}

/// @brief Enable/disable the selected data stream.
/// 
/// Each data stream (descriptor set) can be enabled or disabled.
/// The default for the device is all streams enabled.
/// For all functions except 0x01 (use new setting),
/// the new enable flag value is ignored and can be omitted.
/// @param desc_set The descriptor set of the stream to control. When function is SAVE, LOAD, or DEFAULT, can be ALL_STREAMS(0) to apply to all descriptor sets. On Generation 5 products, this must be one of the above legacy constants.
/// @param[out] desc_set 
/// @param[out] enabled 
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_read_datastream_control(struct mip_interface* device, uint8_t desc_set, bool* enabled)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    insert_u8(&serializer, desc_set);
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_CONTROL_DATA_STREAM, buffer, serializer.offset, MIP_REPLY_DESC_3DM_DATASTREAM_ENABLE, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        struct mip_serializer serializer;
        mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
        
        extract_u8(&serializer, &desc_set);
        extract_bool(&serializer, enabled);
        
        if( !mip_serializer_ok(&serializer) )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Enable/disable the selected data stream.
/// 
/// Each data stream (descriptor set) can be enabled or disabled.
/// The default for the device is all streams enabled.
/// For all functions except 0x01 (use new setting),
/// the new enable flag value is ignored and can be omitted.
/// @param desc_set The descriptor set of the stream to control. When function is SAVE, LOAD, or DEFAULT, can be ALL_STREAMS(0) to apply to all descriptor sets. On Generation 5 products, this must be one of the above legacy constants.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_save_datastream_control(struct mip_interface* device, uint8_t desc_set)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    insert_u8(&serializer, desc_set);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_CONTROL_DATA_STREAM, buffer, serializer.offset);
}

/// @brief Enable/disable the selected data stream.
/// 
/// Each data stream (descriptor set) can be enabled or disabled.
/// The default for the device is all streams enabled.
/// For all functions except 0x01 (use new setting),
/// the new enable flag value is ignored and can be omitted.
/// @param desc_set The descriptor set of the stream to control. When function is SAVE, LOAD, or DEFAULT, can be ALL_STREAMS(0) to apply to all descriptor sets. On Generation 5 products, this must be one of the above legacy constants.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_load_datastream_control(struct mip_interface* device, uint8_t desc_set)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    insert_u8(&serializer, desc_set);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_CONTROL_DATA_STREAM, buffer, serializer.offset);
}

/// @brief Enable/disable the selected data stream.
/// 
/// Each data stream (descriptor set) can be enabled or disabled.
/// The default for the device is all streams enabled.
/// For all functions except 0x01 (use new setting),
/// the new enable flag value is ignored and can be omitted.
/// @param desc_set The descriptor set of the stream to control. When function is SAVE, LOAD, or DEFAULT, can be ALL_STREAMS(0) to apply to all descriptor sets. On Generation 5 products, this must be one of the above legacy constants.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_default_datastream_control(struct mip_interface* device, uint8_t desc_set)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    insert_u8(&serializer, desc_set);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_CONTROL_DATA_STREAM, buffer, serializer.offset);
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

void insert_mip_3dm_gnss_sbas_settings_command_sbasoptions(struct mip_serializer* serializer, const enum mip_3dm_gnss_sbas_settings_command_sbasoptions self)
{
    return insert_u16(serializer, (uint16_t)(self));
}
void extract_mip_3dm_gnss_sbas_settings_command_sbasoptions(struct mip_serializer* serializer, enum mip_3dm_gnss_sbas_settings_command_sbasoptions* self)
{
    uint16_t tmp = 0;
    extract_u16(serializer, &tmp);
    *self = tmp;
}

/// @brief Configure the SBAS subsystem
/// 
/// 
/// 
/// @param enable_sbas 0 - SBAS Disabled, 1 - SBAS enabled
/// @param sbas_options SBAS options, see definition
/// @param num_included_prns Number of SBAS PRNs to include in search (0 = include all)
/// @param included_prns List of specific SBAS PRNs to search for
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_write_gnss_sbas_settings(struct mip_interface* device, uint8_t enable_sbas, enum mip_3dm_gnss_sbas_settings_command_sbasoptions sbas_options, uint8_t num_included_prns, const uint16_t* included_prns)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    insert_u8(&serializer, enable_sbas);
    insert_mip_3dm_gnss_sbas_settings_command_sbasoptions(&serializer, sbas_options);
    insert_u8(&serializer, num_included_prns);
    for(unsigned int i=0; i < num_included_prns; i++)
        insert_u16(&serializer, included_prns[i]);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_GNSS_SBAS_SETTINGS, buffer, serializer.offset);
}

/// @brief Configure the SBAS subsystem
/// 
/// 
/// 
/// @param[out] enable_sbas 0 - SBAS Disabled, 1 - SBAS enabled
/// @param[out] sbas_options SBAS options, see definition
/// @param[out] num_included_prns Number of SBAS PRNs to include in search (0 = include all)
/// @param[out] included_prns List of specific SBAS PRNs to search for
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_read_gnss_sbas_settings(struct mip_interface* device, uint8_t* enable_sbas, enum mip_3dm_gnss_sbas_settings_command_sbasoptions* sbas_options, uint8_t* num_included_prns, uint16_t* included_prns)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_GNSS_SBAS_SETTINGS, buffer, serializer.offset, MIP_REPLY_DESC_3DM_GNSS_SBAS_SETTINGS, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        struct mip_serializer serializer;
        mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
        
        extract_u8(&serializer, enable_sbas);
        extract_mip_3dm_gnss_sbas_settings_command_sbasoptions(&serializer, sbas_options);
        extract_count(&serializer, num_included_prns, *num_included_prns);
        for(unsigned int i=0; i < *num_included_prns; i++)
            extract_u16(&serializer, &included_prns[i]);
        
        if( !mip_serializer_ok(&serializer) )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Configure the SBAS subsystem
/// 
/// 
/// 
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_save_gnss_sbas_settings(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_GNSS_SBAS_SETTINGS, buffer, serializer.offset);
}

/// @brief Configure the SBAS subsystem
/// 
/// 
/// 
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_load_gnss_sbas_settings(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_GNSS_SBAS_SETTINGS, buffer, serializer.offset);
}

/// @brief Configure the SBAS subsystem
/// 
/// 
/// 
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_default_gnss_sbas_settings(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_GNSS_SBAS_SETTINGS, buffer, serializer.offset);
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

/// @brief Provide the GNSS subsystem with initial time information.
/// 
/// This message is required immediately after power up if GNSS Assist was enabled when the device was powered off.
/// This will initialize the subsystem clock to help reduce the time to first fix (TTFF).
/// @param tow GPS Time of week [seconds]
/// @param week_number GPS Weeks since 1980 [weeks]
/// @param accuracy Accuracy of time information [seconds]
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_write_gnss_time_assistance(struct mip_interface* device, double tow, uint16_t week_number, float accuracy)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    insert_double(&serializer, tow);
    insert_u16(&serializer, week_number);
    insert_float(&serializer, accuracy);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_GNSS_TIME_ASSISTANCE, buffer, serializer.offset);
}

/// @brief Provide the GNSS subsystem with initial time information.
/// 
/// This message is required immediately after power up if GNSS Assist was enabled when the device was powered off.
/// This will initialize the subsystem clock to help reduce the time to first fix (TTFF).
/// @param[out] tow GPS Time of week [seconds]
/// @param[out] week_number GPS Weeks since 1980 [weeks]
/// @param[out] accuracy Accuracy of time information [seconds]
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_read_gnss_time_assistance(struct mip_interface* device, double* tow, uint16_t* week_number, float* accuracy)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_GNSS_TIME_ASSISTANCE, buffer, serializer.offset, MIP_REPLY_DESC_3DM_GNSS_TIME_ASSISTANCE, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        struct mip_serializer serializer;
        mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
        
        extract_double(&serializer, tow);
        extract_u16(&serializer, week_number);
        extract_float(&serializer, accuracy);
        
        if( !mip_serializer_ok(&serializer) )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
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

/// @brief Advanced configuration for the IMU data quantity low-pass filters.
/// 
/// The scaled data quantities are by default filtered through a single-pole IIR low-pass filter
/// which is configured with a -3dB cutoff frequency of half the reporting frequency (set by
/// decimation factor in the IMU Message Format command) to prevent aliasing on a per data
/// quantity basis. This advanced configuration command allows for the cutoff frequency to
/// be configured independently of the data reporting frequency as well as allowing for a
/// complete bypass of the digital low-pass filter.
/// 
/// Possible data descriptors:
/// 0x04 – Scaled accelerometer data
/// 0x05 – Scaled gyro data
/// 0x06 – Scaled magnetometer data (if applicable)
/// 0x17 – Scaled pressure data (if applicable)
/// @param target_descriptor Field descriptor of filtered quantity within the Sensor data set. Supported values are accel (0x04), gyro (0x05), mag (0x06), and pressure (0x17), provided the data is supported by the device. Except with the READ function selector, this can be 0 to apply to all of the above quantities.
/// @param enable The target data will be filtered if this is true.
/// @param manual If false, the cutoff frequency is set to half of the streaming rate as configured by the message format command. Otherwise, the cutoff frequency is set according to the following 'frequency' parameter.
/// @param frequency -3dB cutoff frequency in Hz. Will not affect filtering if 'manual' is false.
/// @param reserved Reserved, set to 0x00.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_write_adv_lowpass_filter(struct mip_interface* device, uint8_t target_descriptor, bool enable, bool manual, uint16_t frequency, uint8_t reserved)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    insert_u8(&serializer, target_descriptor);
    insert_bool(&serializer, enable);
    insert_bool(&serializer, manual);
    insert_u16(&serializer, frequency);
    insert_u8(&serializer, reserved);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_ADVANCED_DATA_FILTER, buffer, serializer.offset);
}

/// @brief Advanced configuration for the IMU data quantity low-pass filters.
/// 
/// The scaled data quantities are by default filtered through a single-pole IIR low-pass filter
/// which is configured with a -3dB cutoff frequency of half the reporting frequency (set by
/// decimation factor in the IMU Message Format command) to prevent aliasing on a per data
/// quantity basis. This advanced configuration command allows for the cutoff frequency to
/// be configured independently of the data reporting frequency as well as allowing for a
/// complete bypass of the digital low-pass filter.
/// 
/// Possible data descriptors:
/// 0x04 – Scaled accelerometer data
/// 0x05 – Scaled gyro data
/// 0x06 – Scaled magnetometer data (if applicable)
/// 0x17 – Scaled pressure data (if applicable)
/// @param target_descriptor Field descriptor of filtered quantity within the Sensor data set. Supported values are accel (0x04), gyro (0x05), mag (0x06), and pressure (0x17), provided the data is supported by the device. Except with the READ function selector, this can be 0 to apply to all of the above quantities.
/// @param[out] target_descriptor 
/// @param[out] enable True if the filter is currently enabled.
/// @param[out] manual True if the filter cutoff was manually configured.
/// @param[out] frequency The cutoff frequency of the filter. If the filter is in auto mode, this value is unspecified.
/// @param[out] reserved Reserved and must be ignored.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_read_adv_lowpass_filter(struct mip_interface* device, uint8_t target_descriptor, bool* enable, bool* manual, uint16_t* frequency, uint8_t* reserved)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    insert_u8(&serializer, target_descriptor);
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_ADVANCED_DATA_FILTER, buffer, serializer.offset, MIP_REPLY_DESC_3DM_ADVANCED_DATA_FILTER, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        struct mip_serializer serializer;
        mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
        
        extract_u8(&serializer, &target_descriptor);
        extract_bool(&serializer, enable);
        extract_bool(&serializer, manual);
        extract_u16(&serializer, frequency);
        extract_u8(&serializer, reserved);
        
        if( !mip_serializer_ok(&serializer) )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Advanced configuration for the IMU data quantity low-pass filters.
/// 
/// The scaled data quantities are by default filtered through a single-pole IIR low-pass filter
/// which is configured with a -3dB cutoff frequency of half the reporting frequency (set by
/// decimation factor in the IMU Message Format command) to prevent aliasing on a per data
/// quantity basis. This advanced configuration command allows for the cutoff frequency to
/// be configured independently of the data reporting frequency as well as allowing for a
/// complete bypass of the digital low-pass filter.
/// 
/// Possible data descriptors:
/// 0x04 – Scaled accelerometer data
/// 0x05 – Scaled gyro data
/// 0x06 – Scaled magnetometer data (if applicable)
/// 0x17 – Scaled pressure data (if applicable)
/// @param target_descriptor Field descriptor of filtered quantity within the Sensor data set. Supported values are accel (0x04), gyro (0x05), mag (0x06), and pressure (0x17), provided the data is supported by the device. Except with the READ function selector, this can be 0 to apply to all of the above quantities.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_save_adv_lowpass_filter(struct mip_interface* device, uint8_t target_descriptor)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    insert_u8(&serializer, target_descriptor);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_ADVANCED_DATA_FILTER, buffer, serializer.offset);
}

/// @brief Advanced configuration for the IMU data quantity low-pass filters.
/// 
/// The scaled data quantities are by default filtered through a single-pole IIR low-pass filter
/// which is configured with a -3dB cutoff frequency of half the reporting frequency (set by
/// decimation factor in the IMU Message Format command) to prevent aliasing on a per data
/// quantity basis. This advanced configuration command allows for the cutoff frequency to
/// be configured independently of the data reporting frequency as well as allowing for a
/// complete bypass of the digital low-pass filter.
/// 
/// Possible data descriptors:
/// 0x04 – Scaled accelerometer data
/// 0x05 – Scaled gyro data
/// 0x06 – Scaled magnetometer data (if applicable)
/// 0x17 – Scaled pressure data (if applicable)
/// @param target_descriptor Field descriptor of filtered quantity within the Sensor data set. Supported values are accel (0x04), gyro (0x05), mag (0x06), and pressure (0x17), provided the data is supported by the device. Except with the READ function selector, this can be 0 to apply to all of the above quantities.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_load_adv_lowpass_filter(struct mip_interface* device, uint8_t target_descriptor)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    insert_u8(&serializer, target_descriptor);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_ADVANCED_DATA_FILTER, buffer, serializer.offset);
}

/// @brief Advanced configuration for the IMU data quantity low-pass filters.
/// 
/// The scaled data quantities are by default filtered through a single-pole IIR low-pass filter
/// which is configured with a -3dB cutoff frequency of half the reporting frequency (set by
/// decimation factor in the IMU Message Format command) to prevent aliasing on a per data
/// quantity basis. This advanced configuration command allows for the cutoff frequency to
/// be configured independently of the data reporting frequency as well as allowing for a
/// complete bypass of the digital low-pass filter.
/// 
/// Possible data descriptors:
/// 0x04 – Scaled accelerometer data
/// 0x05 – Scaled gyro data
/// 0x06 – Scaled magnetometer data (if applicable)
/// 0x17 – Scaled pressure data (if applicable)
/// @param target_descriptor Field descriptor of filtered quantity within the Sensor data set. Supported values are accel (0x04), gyro (0x05), mag (0x06), and pressure (0x17), provided the data is supported by the device. Except with the READ function selector, this can be 0 to apply to all of the above quantities.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_default_adv_lowpass_filter(struct mip_interface* device, uint8_t target_descriptor)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    insert_u8(&serializer, target_descriptor);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_ADVANCED_DATA_FILTER, buffer, serializer.offset);
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

void insert_mip_3dm_pps_source_command_source(struct mip_serializer* serializer, const enum mip_3dm_pps_source_command_source self)
{
    return insert_u8(serializer, (uint8_t)(self));
}
void extract_mip_3dm_pps_source_command_source(struct mip_serializer* serializer, enum mip_3dm_pps_source_command_source* self)
{
    uint8_t tmp = 0;
    extract_u8(serializer, &tmp);
    *self = tmp;
}

/// @brief Controls the Pulse Per Second (PPS) source.
/// 
/// @param source 
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_write_pps_source(struct mip_interface* device, enum mip_3dm_pps_source_command_source source)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    insert_mip_3dm_pps_source_command_source(&serializer, source);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_PPS_SOURCE, buffer, serializer.offset);
}

/// @brief Controls the Pulse Per Second (PPS) source.
/// 
/// @param[out] source 
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_read_pps_source(struct mip_interface* device, enum mip_3dm_pps_source_command_source* source)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_PPS_SOURCE, buffer, serializer.offset, MIP_REPLY_DESC_3DM_PPS_SOURCE, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        struct mip_serializer serializer;
        mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
        
        extract_mip_3dm_pps_source_command_source(&serializer, source);
        
        if( !mip_serializer_ok(&serializer) )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Controls the Pulse Per Second (PPS) source.
/// 
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_save_pps_source(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_PPS_SOURCE, buffer, serializer.offset);
}

/// @brief Controls the Pulse Per Second (PPS) source.
/// 
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_load_pps_source(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_PPS_SOURCE, buffer, serializer.offset);
}

/// @brief Controls the Pulse Per Second (PPS) source.
/// 
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_default_pps_source(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_PPS_SOURCE, buffer, serializer.offset);
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

void insert_mip_3dm_gpio_config_command_feature(struct mip_serializer* serializer, const enum mip_3dm_gpio_config_command_feature self)
{
    return insert_u8(serializer, (uint8_t)(self));
}
void extract_mip_3dm_gpio_config_command_feature(struct mip_serializer* serializer, enum mip_3dm_gpio_config_command_feature* self)
{
    uint8_t tmp = 0;
    extract_u8(serializer, &tmp);
    *self = tmp;
}

void insert_mip_3dm_gpio_config_command_behavior(struct mip_serializer* serializer, const enum mip_3dm_gpio_config_command_behavior self)
{
    return insert_u8(serializer, (uint8_t)(self));
}
void extract_mip_3dm_gpio_config_command_behavior(struct mip_serializer* serializer, enum mip_3dm_gpio_config_command_behavior* self)
{
    uint8_t tmp = 0;
    extract_u8(serializer, &tmp);
    *self = tmp;
}

void insert_mip_3dm_gpio_config_command_pin_mode(struct mip_serializer* serializer, const enum mip_3dm_gpio_config_command_pin_mode self)
{
    return insert_u8(serializer, (uint8_t)(self));
}
void extract_mip_3dm_gpio_config_command_pin_mode(struct mip_serializer* serializer, enum mip_3dm_gpio_config_command_pin_mode* self)
{
    uint8_t tmp = 0;
    extract_u8(serializer, &tmp);
    *self = tmp;
}

/// @brief Configures the user GPIO pins on the connector for use with several built-in functions or for general input or output.
/// 
/// GPIO pins are device-dependent. Some features are only available on
/// certain pins. Some behaviors require specific configurations.
/// Consult the device user manual for restrictions and default settings.
/// 
/// To avoid glitches on GPIOs configured as an output in a mode other than
/// GPIO, always configure the relevant function before setting up the pin
/// with this command. Otherwise, the pin state will be undefined between
/// this command and the one to set up the feature. For input pins, use
/// this command first so the state is well-defined when the feature is
/// initialized.
/// 
/// Some configurations can only be active on one pin at a time. If such
/// configuration is applied to a second pin, the second one will take
/// precedence and the original pin's configuration will be reset.
/// 
/// @param pin GPIO pin number counting from 1. For save, load, and default function selectors, this can be 0 to select all pins.
/// @param feature Determines how the pin will be used.
/// @param behavior Select an appropriate value from the enumeration based on the selected feature (e.g. for PPS, select one of the values prefixed with PPS_.)
/// @param pin_mode GPIO configuration. May be restricted depending on device, pin, feature, and behavior. See device user manual.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_write_gpio_config(struct mip_interface* device, uint8_t pin, enum mip_3dm_gpio_config_command_feature feature, enum mip_3dm_gpio_config_command_behavior behavior, enum mip_3dm_gpio_config_command_pin_mode pin_mode)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    insert_u8(&serializer, pin);
    insert_mip_3dm_gpio_config_command_feature(&serializer, feature);
    insert_mip_3dm_gpio_config_command_behavior(&serializer, behavior);
    insert_mip_3dm_gpio_config_command_pin_mode(&serializer, pin_mode);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_GPIO_CONFIG, buffer, serializer.offset);
}

/// @brief Configures the user GPIO pins on the connector for use with several built-in functions or for general input or output.
/// 
/// GPIO pins are device-dependent. Some features are only available on
/// certain pins. Some behaviors require specific configurations.
/// Consult the device user manual for restrictions and default settings.
/// 
/// To avoid glitches on GPIOs configured as an output in a mode other than
/// GPIO, always configure the relevant function before setting up the pin
/// with this command. Otherwise, the pin state will be undefined between
/// this command and the one to set up the feature. For input pins, use
/// this command first so the state is well-defined when the feature is
/// initialized.
/// 
/// Some configurations can only be active on one pin at a time. If such
/// configuration is applied to a second pin, the second one will take
/// precedence and the original pin's configuration will be reset.
/// 
/// @param pin GPIO pin number counting from 1. For save, load, and default function selectors, this can be 0 to select all pins.
/// @param[out] pin GPIO pin number counting from 1. For save, load, and default function selectors, this can be 0 to select all pins.
/// @param[out] feature Determines how the pin will be used.
/// @param[out] behavior Select an appropriate value from the enumeration based on the selected feature (e.g. for PPS, select one of the values prefixed with PPS_.)
/// @param[out] pin_mode GPIO configuration. May be restricted depending on device, pin, feature, and behavior. See device user manual.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_read_gpio_config(struct mip_interface* device, uint8_t pin, enum mip_3dm_gpio_config_command_feature* feature, enum mip_3dm_gpio_config_command_behavior* behavior, enum mip_3dm_gpio_config_command_pin_mode* pin_mode)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    insert_u8(&serializer, pin);
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_GPIO_CONFIG, buffer, serializer.offset, MIP_REPLY_DESC_3DM_GPIO_CONFIG, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        struct mip_serializer serializer;
        mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
        
        extract_u8(&serializer, &pin);
        extract_mip_3dm_gpio_config_command_feature(&serializer, feature);
        extract_mip_3dm_gpio_config_command_behavior(&serializer, behavior);
        extract_mip_3dm_gpio_config_command_pin_mode(&serializer, pin_mode);
        
        if( !mip_serializer_ok(&serializer) )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Configures the user GPIO pins on the connector for use with several built-in functions or for general input or output.
/// 
/// GPIO pins are device-dependent. Some features are only available on
/// certain pins. Some behaviors require specific configurations.
/// Consult the device user manual for restrictions and default settings.
/// 
/// To avoid glitches on GPIOs configured as an output in a mode other than
/// GPIO, always configure the relevant function before setting up the pin
/// with this command. Otherwise, the pin state will be undefined between
/// this command and the one to set up the feature. For input pins, use
/// this command first so the state is well-defined when the feature is
/// initialized.
/// 
/// Some configurations can only be active on one pin at a time. If such
/// configuration is applied to a second pin, the second one will take
/// precedence and the original pin's configuration will be reset.
/// 
/// @param pin GPIO pin number counting from 1. For save, load, and default function selectors, this can be 0 to select all pins.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_save_gpio_config(struct mip_interface* device, uint8_t pin)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    insert_u8(&serializer, pin);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_GPIO_CONFIG, buffer, serializer.offset);
}

/// @brief Configures the user GPIO pins on the connector for use with several built-in functions or for general input or output.
/// 
/// GPIO pins are device-dependent. Some features are only available on
/// certain pins. Some behaviors require specific configurations.
/// Consult the device user manual for restrictions and default settings.
/// 
/// To avoid glitches on GPIOs configured as an output in a mode other than
/// GPIO, always configure the relevant function before setting up the pin
/// with this command. Otherwise, the pin state will be undefined between
/// this command and the one to set up the feature. For input pins, use
/// this command first so the state is well-defined when the feature is
/// initialized.
/// 
/// Some configurations can only be active on one pin at a time. If such
/// configuration is applied to a second pin, the second one will take
/// precedence and the original pin's configuration will be reset.
/// 
/// @param pin GPIO pin number counting from 1. For save, load, and default function selectors, this can be 0 to select all pins.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_load_gpio_config(struct mip_interface* device, uint8_t pin)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    insert_u8(&serializer, pin);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_GPIO_CONFIG, buffer, serializer.offset);
}

/// @brief Configures the user GPIO pins on the connector for use with several built-in functions or for general input or output.
/// 
/// GPIO pins are device-dependent. Some features are only available on
/// certain pins. Some behaviors require specific configurations.
/// Consult the device user manual for restrictions and default settings.
/// 
/// To avoid glitches on GPIOs configured as an output in a mode other than
/// GPIO, always configure the relevant function before setting up the pin
/// with this command. Otherwise, the pin state will be undefined between
/// this command and the one to set up the feature. For input pins, use
/// this command first so the state is well-defined when the feature is
/// initialized.
/// 
/// Some configurations can only be active on one pin at a time. If such
/// configuration is applied to a second pin, the second one will take
/// precedence and the original pin's configuration will be reset.
/// 
/// @param pin GPIO pin number counting from 1. For save, load, and default function selectors, this can be 0 to select all pins.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_default_gpio_config(struct mip_interface* device, uint8_t pin)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    insert_u8(&serializer, pin);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_GPIO_CONFIG, buffer, serializer.offset);
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

/// @brief Allows the state of the pin to be read or controlled.
/// 
/// This command serves two purposes: 1) To allow reading the state of a pin via command,
/// rather than polling a data quantity, and 2) to provide a way to set the output state
/// without also having to specify the operating mode.
/// 
/// The state read back from the pin is the physical state of the pin, rather than a
/// configuration value. The state can be read regardless of its configuration as long as
/// the device supports GPIO input on that pin. If the pin is set to an output, the read
/// value would match the output value.
/// 
/// While the state of a pin can always be set, it will only have an observable effect if
/// the pin is set to output mode.
/// 
/// This command does not support saving, loading, or reseting the state. Instead, use the
/// GPIO Configuration command, which allows the initial state to be configured.
/// @param pin GPIO pin number counting from 1. Cannot be 0.
/// @param state The pin state.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_write_gpio_state(struct mip_interface* device, uint8_t pin, bool state)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    insert_u8(&serializer, pin);
    insert_bool(&serializer, state);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_GPIO_STATE, buffer, serializer.offset);
}

/// @brief Allows the state of the pin to be read or controlled.
/// 
/// This command serves two purposes: 1) To allow reading the state of a pin via command,
/// rather than polling a data quantity, and 2) to provide a way to set the output state
/// without also having to specify the operating mode.
/// 
/// The state read back from the pin is the physical state of the pin, rather than a
/// configuration value. The state can be read regardless of its configuration as long as
/// the device supports GPIO input on that pin. If the pin is set to an output, the read
/// value would match the output value.
/// 
/// While the state of a pin can always be set, it will only have an observable effect if
/// the pin is set to output mode.
/// 
/// This command does not support saving, loading, or reseting the state. Instead, use the
/// GPIO Configuration command, which allows the initial state to be configured.
/// @param pin GPIO pin number counting from 1. Cannot be 0.
/// @param[out] pin GPIO pin number counting from 1. Cannot be 0.
/// @param[out] state The pin state.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_read_gpio_state(struct mip_interface* device, uint8_t pin, bool* state)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    insert_u8(&serializer, pin);
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_GPIO_STATE, buffer, serializer.offset, MIP_REPLY_DESC_3DM_GPIO_STATE, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        struct mip_serializer serializer;
        mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
        
        extract_u8(&serializer, &pin);
        extract_bool(&serializer, state);
        
        if( !mip_serializer_ok(&serializer) )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
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

void insert_mip_3dm_odometer_command_mode(struct mip_serializer* serializer, const enum mip_3dm_odometer_command_mode self)
{
    return insert_u8(serializer, (uint8_t)(self));
}
void extract_mip_3dm_odometer_command_mode(struct mip_serializer* serializer, enum mip_3dm_odometer_command_mode* self)
{
    uint8_t tmp = 0;
    extract_u8(serializer, &tmp);
    *self = tmp;
}

/// @brief Configures the hardware odometer interface.
/// 
/// @param mode Mode setting.
/// @param scaling Encoder pulses per meter of distance traveled [pulses/m]. Distance traveled is computed using the formula d = p / N * 2R * pi, where d is distance, p is the number of pulses received, N is the encoder resolution, and R is the wheel radius. By simplifying all of the parameters into one, the formula d = p / S is obtained, where s is the odometer scaling factor passed to this command. S is equivalent to N / (2R * pi) and has units of pulses / meter. N is in units of "A" pulses per revolution and R is in meters. Make this value negative if the odometer is mounted so that it rotates backwards.
/// @param uncertainty Uncertainty in encoder counts to distance translation (1-sigma value) [m/m].
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_write_odometer(struct mip_interface* device, enum mip_3dm_odometer_command_mode mode, float scaling, float uncertainty)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    insert_mip_3dm_odometer_command_mode(&serializer, mode);
    insert_float(&serializer, scaling);
    insert_float(&serializer, uncertainty);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_ODOMETER_CONFIG, buffer, serializer.offset);
}

/// @brief Configures the hardware odometer interface.
/// 
/// @param[out] mode Mode setting.
/// @param[out] scaling Encoder pulses per meter of distance traveled [pulses/m]. Distance traveled is computed using the formula d = p / N * 2R * pi, where d is distance, p is the number of pulses received, N is the encoder resolution, and R is the wheel radius. By simplifying all of the parameters into one, the formula d = p / S is obtained, where s is the odometer scaling factor passed to this command. S is equivalent to N / (2R * pi) and has units of pulses / meter. N is in units of "A" pulses per revolution and R is in meters. Make this value negative if the odometer is mounted so that it rotates backwards.
/// @param[out] uncertainty Uncertainty in encoder counts to distance translation (1-sigma value) [m/m].
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_read_odometer(struct mip_interface* device, enum mip_3dm_odometer_command_mode* mode, float* scaling, float* uncertainty)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_ODOMETER_CONFIG, buffer, serializer.offset, MIP_REPLY_DESC_3DM_ODOMETER_CONFIG, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        struct mip_serializer serializer;
        mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
        
        extract_mip_3dm_odometer_command_mode(&serializer, mode);
        extract_float(&serializer, scaling);
        extract_float(&serializer, uncertainty);
        
        if( !mip_serializer_ok(&serializer) )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Configures the hardware odometer interface.
/// 
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_save_odometer(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_ODOMETER_CONFIG, buffer, serializer.offset);
}

/// @brief Configures the hardware odometer interface.
/// 
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_load_odometer(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_ODOMETER_CONFIG, buffer, serializer.offset);
}

/// @brief Configures the hardware odometer interface.
/// 
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_default_odometer(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_ODOMETER_CONFIG, buffer, serializer.offset);
}

void insert_mip_3dm_get_event_support_command(struct mip_serializer* serializer, const struct mip_3dm_get_event_support_command* self)
{
    insert_mip_3dm_get_event_support_command_query(serializer, self->query);
}

void extract_mip_3dm_get_event_support_command(struct mip_serializer* serializer, struct mip_3dm_get_event_support_command* self)
{
    extract_mip_3dm_get_event_support_command_query(serializer, &self->query);
}

void insert_mip_3dm_get_event_support_command_query(struct mip_serializer* serializer, const enum mip_3dm_get_event_support_command_query self)
{
    return insert_u8(serializer, (uint8_t)(self));
}
void extract_mip_3dm_get_event_support_command_query(struct mip_serializer* serializer, enum mip_3dm_get_event_support_command_query* self)
{
    uint8_t tmp = 0;
    extract_u8(serializer, &tmp);
    *self = tmp;
}

void insert_mip_3dm_get_event_support_command_info(struct mip_serializer* serializer, const struct mip_3dm_get_event_support_command_info* self)
{
    insert_u8(serializer, self->type);
    insert_u8(serializer, self->count);
}

void extract_mip_3dm_get_event_support_command_info(struct mip_serializer* serializer, struct mip_3dm_get_event_support_command_info* self)
{
    extract_u8(serializer, &self->type);
    extract_u8(serializer, &self->count);
}

/// @brief Lists the available trigger or action types.
/// 
/// There are a limited number of trigger and action slots available
/// in the device. Up to M triggers and N actions can be configured at once
/// in slots 1..M and 1..N respectively. M and N are identified by the
/// max_instances field in the response with the appropriate query selector.
/// 
/// Each slot can be configured as one of a variety of different types of
/// triggers or actions. The supported types are enumerated in the response
/// to this command. Additionally, there is a limit on the number of a given
/// type. In other words, while the device may support M triggers in total,
/// only a few of them maybe usable as a given type. This limit helps optimize
/// device resources. The limit is identified in the count field.
/// 
/// All of the information in this command is available in the user manual.
/// This command provides a programmatic method for obtaining the information.
/// 
/// @param query What type of information to retrieve.
/// @param[out] query Query type specified in the command.
/// @param[out] max_instances Number of slots available. The 'instance' number for the configuration or control commands must be between 1 and this value.
/// @param[out] num_entries Number of supported types.
/// @param[out] entries List of supported types.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_get_event_support(struct mip_interface* device, enum mip_3dm_get_event_support_command_query query, uint8_t* max_instances, uint8_t* num_entries, struct mip_3dm_get_event_support_command_info* entries)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_3dm_get_event_support_command_query(&serializer, query);
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_EVENT_SUPPORT, buffer, serializer.offset, MIP_REPLY_DESC_3DM_EVENT_SUPPORT, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        struct mip_serializer serializer;
        mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
        
        extract_mip_3dm_get_event_support_command_query(&serializer, &query);
        extract_u8(&serializer, max_instances);
        extract_count(&serializer, num_entries, *num_entries);
        for(unsigned int i=0; i < *num_entries; i++)
            extract_mip_3dm_get_event_support_command_info(&serializer, &entries[i]);
        
        if( !mip_serializer_ok(&serializer) )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
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

void insert_mip_3dm_event_control_command_mode(struct mip_serializer* serializer, const enum mip_3dm_event_control_command_mode self)
{
    return insert_u8(serializer, (uint8_t)(self));
}
void extract_mip_3dm_event_control_command_mode(struct mip_serializer* serializer, enum mip_3dm_event_control_command_mode* self)
{
    uint8_t tmp = 0;
    extract_u8(serializer, &tmp);
    *self = tmp;
}

/// @brief Enables or disables event triggers.
/// 
/// Triggers can be disabled, enabled, and tested. While disabled, a trigger will
/// not evaluate its logic and effective behave like no trigger is configured.
/// A disabled trigger will not activate any actions. Triggers are disabled by default.
/// 
/// Use this command to enable (or disable) a trigger, or to place it into a test mode.
/// When in test mode, the trigger logic is disabled but the output is forced to
/// the active state, meaning that it will behave as if the trigger logic is satisfied
/// and any associated actions will execute.
/// @param instance Trigger instance to affect. 0 can be used to apply the mode to all configured triggers, except when the function selector is READ.
/// @param mode How to change the trigger state. Except when instance is 0, the corresponding trigger must be configured, i.e. not have type 0.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_write_event_control(struct mip_interface* device, uint8_t instance, enum mip_3dm_event_control_command_mode mode)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    insert_u8(&serializer, instance);
    insert_mip_3dm_event_control_command_mode(&serializer, mode);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_EVENT_CONTROL, buffer, serializer.offset);
}

/// @brief Enables or disables event triggers.
/// 
/// Triggers can be disabled, enabled, and tested. While disabled, a trigger will
/// not evaluate its logic and effective behave like no trigger is configured.
/// A disabled trigger will not activate any actions. Triggers are disabled by default.
/// 
/// Use this command to enable (or disable) a trigger, or to place it into a test mode.
/// When in test mode, the trigger logic is disabled but the output is forced to
/// the active state, meaning that it will behave as if the trigger logic is satisfied
/// and any associated actions will execute.
/// @param instance Trigger instance to affect. 0 can be used to apply the mode to all configured triggers, except when the function selector is READ.
/// @param[out] instance Trigger instance to affect. 0 can be used to apply the mode to all configured triggers, except when the function selector is READ.
/// @param[out] mode How to change the trigger state. Except when instance is 0, the corresponding trigger must be configured, i.e. not have type 0.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_read_event_control(struct mip_interface* device, uint8_t instance, enum mip_3dm_event_control_command_mode* mode)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    insert_u8(&serializer, instance);
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_EVENT_CONTROL, buffer, serializer.offset, MIP_REPLY_DESC_3DM_EVENT_CONTROL, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        struct mip_serializer serializer;
        mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
        
        extract_u8(&serializer, &instance);
        extract_mip_3dm_event_control_command_mode(&serializer, mode);
        
        if( !mip_serializer_ok(&serializer) )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Enables or disables event triggers.
/// 
/// Triggers can be disabled, enabled, and tested. While disabled, a trigger will
/// not evaluate its logic and effective behave like no trigger is configured.
/// A disabled trigger will not activate any actions. Triggers are disabled by default.
/// 
/// Use this command to enable (or disable) a trigger, or to place it into a test mode.
/// When in test mode, the trigger logic is disabled but the output is forced to
/// the active state, meaning that it will behave as if the trigger logic is satisfied
/// and any associated actions will execute.
/// @param instance Trigger instance to affect. 0 can be used to apply the mode to all configured triggers, except when the function selector is READ.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_save_event_control(struct mip_interface* device, uint8_t instance)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    insert_u8(&serializer, instance);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_EVENT_CONTROL, buffer, serializer.offset);
}

/// @brief Enables or disables event triggers.
/// 
/// Triggers can be disabled, enabled, and tested. While disabled, a trigger will
/// not evaluate its logic and effective behave like no trigger is configured.
/// A disabled trigger will not activate any actions. Triggers are disabled by default.
/// 
/// Use this command to enable (or disable) a trigger, or to place it into a test mode.
/// When in test mode, the trigger logic is disabled but the output is forced to
/// the active state, meaning that it will behave as if the trigger logic is satisfied
/// and any associated actions will execute.
/// @param instance Trigger instance to affect. 0 can be used to apply the mode to all configured triggers, except when the function selector is READ.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_load_event_control(struct mip_interface* device, uint8_t instance)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    insert_u8(&serializer, instance);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_EVENT_CONTROL, buffer, serializer.offset);
}

/// @brief Enables or disables event triggers.
/// 
/// Triggers can be disabled, enabled, and tested. While disabled, a trigger will
/// not evaluate its logic and effective behave like no trigger is configured.
/// A disabled trigger will not activate any actions. Triggers are disabled by default.
/// 
/// Use this command to enable (or disable) a trigger, or to place it into a test mode.
/// When in test mode, the trigger logic is disabled but the output is forced to
/// the active state, meaning that it will behave as if the trigger logic is satisfied
/// and any associated actions will execute.
/// @param instance Trigger instance to affect. 0 can be used to apply the mode to all configured triggers, except when the function selector is READ.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_default_event_control(struct mip_interface* device, uint8_t instance)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    insert_u8(&serializer, instance);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_EVENT_CONTROL, buffer, serializer.offset);
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

void insert_mip_3dm_get_event_trigger_status_command_status(struct mip_serializer* serializer, const enum mip_3dm_get_event_trigger_status_command_status self)
{
    return insert_u8(serializer, (uint8_t)(self));
}
void extract_mip_3dm_get_event_trigger_status_command_status(struct mip_serializer* serializer, enum mip_3dm_get_event_trigger_status_command_status* self)
{
    uint8_t tmp = 0;
    extract_u8(serializer, &tmp);
    *self = tmp;
}

void insert_mip_3dm_get_event_trigger_status_command_entry(struct mip_serializer* serializer, const struct mip_3dm_get_event_trigger_status_command_entry* self)
{
    insert_u8(serializer, self->type);
    insert_mip_3dm_get_event_trigger_status_command_status(serializer, self->status);
}

void extract_mip_3dm_get_event_trigger_status_command_entry(struct mip_serializer* serializer, struct mip_3dm_get_event_trigger_status_command_entry* self)
{
    extract_u8(serializer, &self->type);
    extract_mip_3dm_get_event_trigger_status_command_status(serializer, &self->status);
}

mip_cmd_result mip_3dm_get_event_trigger_status(struct mip_interface* device, uint8_t requested_count, const uint8_t* requested_instances, uint8_t* count, struct mip_3dm_get_event_trigger_status_command_entry* triggers)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_u8(&serializer, requested_count);
    for(unsigned int i=0; i < requested_count; i++)
        insert_u8(&serializer, requested_instances[i]);
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_EVENT_TRIGGER_STATUS, buffer, serializer.offset, MIP_REPLY_DESC_3DM_EVENT_TRIGGER_STATUS, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        struct mip_serializer serializer;
        mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
        
        extract_count(&serializer, count, *count);
        for(unsigned int i=0; i < *count; i++)
            extract_mip_3dm_get_event_trigger_status_command_entry(&serializer, &triggers[i]);
        
        if( !mip_serializer_ok(&serializer) )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
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

void insert_mip_3dm_get_event_action_status_command_entry(struct mip_serializer* serializer, const struct mip_3dm_get_event_action_status_command_entry* self)
{
    insert_u8(serializer, self->action_type);
    insert_u8(serializer, self->trigger_id);
}

void extract_mip_3dm_get_event_action_status_command_entry(struct mip_serializer* serializer, struct mip_3dm_get_event_action_status_command_entry* self)
{
    extract_u8(serializer, &self->action_type);
    extract_u8(serializer, &self->trigger_id);
}

mip_cmd_result mip_3dm_get_event_action_status(struct mip_interface* device, uint8_t requested_count, const uint8_t* requested_instances, uint8_t* count, struct mip_3dm_get_event_action_status_command_entry* actions)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_u8(&serializer, requested_count);
    for(unsigned int i=0; i < requested_count; i++)
        insert_u8(&serializer, requested_instances[i]);
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_EVENT_ACTION_STATUS, buffer, serializer.offset, MIP_REPLY_DESC_3DM_EVENT_ACTION_STATUS, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        struct mip_serializer serializer;
        mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
        
        extract_count(&serializer, count, *count);
        for(unsigned int i=0; i < *count; i++)
            extract_mip_3dm_get_event_action_status_command_entry(&serializer, &actions[i]);
        
        if( !mip_serializer_ok(&serializer) )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

void insert_mip_3dm_event_trigger_command(struct mip_serializer* serializer, const struct mip_3dm_event_trigger_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    insert_u8(serializer, self->instance);
    insert_mip_3dm_event_trigger_command_type(serializer, self->type);
    if( self->type == MIP_3DM_EVENT_TRIGGER_COMMAND_TYPE_GPIO )
        insert_mip_3dm_event_trigger_command_gpio_params(serializer, &self->parameters.gpio);
    if( self->type == MIP_3DM_EVENT_TRIGGER_COMMAND_TYPE_THRESHOLD )
        insert_mip_3dm_event_trigger_command_threshold_params(serializer, &self->parameters.threshold);
    if( self->type == MIP_3DM_EVENT_TRIGGER_COMMAND_TYPE_COMBINATION )
        insert_mip_3dm_event_trigger_command_combination_params(serializer, &self->parameters.combination);
}

void extract_mip_3dm_event_trigger_command(struct mip_serializer* serializer, struct mip_3dm_event_trigger_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    extract_u8(serializer, &self->instance);
    extract_mip_3dm_event_trigger_command_type(serializer, &self->type);
    if( self->type == MIP_3DM_EVENT_TRIGGER_COMMAND_TYPE_GPIO )
        extract_mip_3dm_event_trigger_command_gpio_params(serializer, &self->parameters.gpio);
    if( self->type == MIP_3DM_EVENT_TRIGGER_COMMAND_TYPE_THRESHOLD )
        extract_mip_3dm_event_trigger_command_threshold_params(serializer, &self->parameters.threshold);
    if( self->type == MIP_3DM_EVENT_TRIGGER_COMMAND_TYPE_COMBINATION )
        extract_mip_3dm_event_trigger_command_combination_params(serializer, &self->parameters.combination);
}

void insert_mip_3dm_event_trigger_command_gpio_params(struct mip_serializer* serializer, const struct mip_3dm_event_trigger_command_gpio_params* self)
{
    insert_u8(serializer, self->pin);
    insert_mip_3dm_event_trigger_command_gpio_params_mode(serializer, self->mode);
}

void extract_mip_3dm_event_trigger_command_gpio_params(struct mip_serializer* serializer, struct mip_3dm_event_trigger_command_gpio_params* self)
{
    extract_u8(serializer, &self->pin);
    extract_mip_3dm_event_trigger_command_gpio_params_mode(serializer, &self->mode);
}

void insert_mip_3dm_event_trigger_command_gpio_params_mode(struct mip_serializer* serializer, const enum mip_3dm_event_trigger_command_gpio_params_mode self)
{
    return insert_u8(serializer, (uint8_t)(self));
}
void extract_mip_3dm_event_trigger_command_gpio_params_mode(struct mip_serializer* serializer, enum mip_3dm_event_trigger_command_gpio_params_mode* self)
{
    uint8_t tmp = 0;
    extract_u8(serializer, &tmp);
    *self = tmp;
}

void insert_mip_3dm_event_trigger_command_threshold_params(struct mip_serializer* serializer, const struct mip_3dm_event_trigger_command_threshold_params* self)
{
    insert_u8(serializer, self->desc_set);
    insert_u8(serializer, self->field_desc);
    insert_u8(serializer, self->param_id);
    insert_mip_3dm_event_trigger_command_threshold_params_type(serializer, self->type);
    if( self->type == MIP_3DM_EVENT_TRIGGER_COMMAND_THRESHOLD_PARAMS_TYPE_WINDOW )
        insert_double(serializer, self->low_thres);
    if( self->type == MIP_3DM_EVENT_TRIGGER_COMMAND_THRESHOLD_PARAMS_TYPE_INTERVAL )
        insert_double(serializer, self->int_thres);
    if( self->type == MIP_3DM_EVENT_TRIGGER_COMMAND_THRESHOLD_PARAMS_TYPE_WINDOW )
        insert_double(serializer, self->high_thres);
    if( self->type == MIP_3DM_EVENT_TRIGGER_COMMAND_THRESHOLD_PARAMS_TYPE_INTERVAL )
        insert_double(serializer, self->interval);
}

void extract_mip_3dm_event_trigger_command_threshold_params(struct mip_serializer* serializer, struct mip_3dm_event_trigger_command_threshold_params* self)
{
    extract_u8(serializer, &self->desc_set);
    extract_u8(serializer, &self->field_desc);
    extract_u8(serializer, &self->param_id);
    extract_mip_3dm_event_trigger_command_threshold_params_type(serializer, &self->type);
    if( self->type == MIP_3DM_EVENT_TRIGGER_COMMAND_THRESHOLD_PARAMS_TYPE_WINDOW )
        extract_double(serializer, &self->low_thres);
    if( self->type == MIP_3DM_EVENT_TRIGGER_COMMAND_THRESHOLD_PARAMS_TYPE_INTERVAL )
        extract_double(serializer, &self->int_thres);
    if( self->type == MIP_3DM_EVENT_TRIGGER_COMMAND_THRESHOLD_PARAMS_TYPE_WINDOW )
        extract_double(serializer, &self->high_thres);
    if( self->type == MIP_3DM_EVENT_TRIGGER_COMMAND_THRESHOLD_PARAMS_TYPE_INTERVAL )
        extract_double(serializer, &self->interval);
}

void insert_mip_3dm_event_trigger_command_threshold_params_type(struct mip_serializer* serializer, const enum mip_3dm_event_trigger_command_threshold_params_type self)
{
    return insert_u8(serializer, (uint8_t)(self));
}
void extract_mip_3dm_event_trigger_command_threshold_params_type(struct mip_serializer* serializer, enum mip_3dm_event_trigger_command_threshold_params_type* self)
{
    uint8_t tmp = 0;
    extract_u8(serializer, &tmp);
    *self = tmp;
}

void insert_mip_3dm_event_trigger_command_combination_params(struct mip_serializer* serializer, const struct mip_3dm_event_trigger_command_combination_params* self)
{
    insert_u16(serializer, self->logic_table);
    for(unsigned int i=0; i < 4; i++)
        insert_u8(serializer, self->input_triggers[i]);
}

void extract_mip_3dm_event_trigger_command_combination_params(struct mip_serializer* serializer, struct mip_3dm_event_trigger_command_combination_params* self)
{
    extract_u16(serializer, &self->logic_table);
    for(unsigned int i=0; i < 4; i++)
        extract_u8(serializer, &self->input_triggers[i]);
}

void insert_mip_3dm_event_trigger_command_type(struct mip_serializer* serializer, const enum mip_3dm_event_trigger_command_type self)
{
    return insert_u8(serializer, (uint8_t)(self));
}
void extract_mip_3dm_event_trigger_command_type(struct mip_serializer* serializer, enum mip_3dm_event_trigger_command_type* self)
{
    uint8_t tmp = 0;
    extract_u8(serializer, &tmp);
    *self = tmp;
}

/// @brief Configures various types of event triggers.
/// 
/// @param instance Trigger number. When function is SAVE, LOAD, or DEFAULT, this can be 0 to apply to all instances.
/// @param type Type of trigger to configure.
/// @param parameters 
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_write_event_trigger(struct mip_interface* device, uint8_t instance, enum mip_3dm_event_trigger_command_type type, const union mip_3dm_event_trigger_command_parameters* parameters)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    insert_u8(&serializer, instance);
    insert_mip_3dm_event_trigger_command_type(&serializer, type);
    if( type == MIP_3DM_EVENT_TRIGGER_COMMAND_TYPE_GPIO )
        insert_mip_3dm_event_trigger_command_gpio_params(&serializer, &parameters->gpio);
    if( type == MIP_3DM_EVENT_TRIGGER_COMMAND_TYPE_THRESHOLD )
        insert_mip_3dm_event_trigger_command_threshold_params(&serializer, &parameters->threshold);
    if( type == MIP_3DM_EVENT_TRIGGER_COMMAND_TYPE_COMBINATION )
        insert_mip_3dm_event_trigger_command_combination_params(&serializer, &parameters->combination);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_EVENT_TRIGGER_CONFIG, buffer, serializer.offset);
}

/// @brief Configures various types of event triggers.
/// 
/// @param instance Trigger number. When function is SAVE, LOAD, or DEFAULT, this can be 0 to apply to all instances.
/// @param[out] instance Trigger number. When function is SAVE, LOAD, or DEFAULT, this can be 0 to apply to all instances.
/// @param[out] type Type of trigger to configure.
/// @param[out] parameters 
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_read_event_trigger(struct mip_interface* device, uint8_t instance, enum mip_3dm_event_trigger_command_type* type, union mip_3dm_event_trigger_command_parameters* parameters)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    insert_u8(&serializer, instance);
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_EVENT_TRIGGER_CONFIG, buffer, serializer.offset, MIP_REPLY_DESC_3DM_EVENT_TRIGGER_CONFIG, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        struct mip_serializer serializer;
        mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
        
        extract_u8(&serializer, &instance);
        extract_mip_3dm_event_trigger_command_type(&serializer, type);
        if( *type == MIP_3DM_EVENT_TRIGGER_COMMAND_TYPE_GPIO )
            extract_mip_3dm_event_trigger_command_gpio_params(&serializer, &parameters->gpio);
        if( *type == MIP_3DM_EVENT_TRIGGER_COMMAND_TYPE_THRESHOLD )
            extract_mip_3dm_event_trigger_command_threshold_params(&serializer, &parameters->threshold);
        if( *type == MIP_3DM_EVENT_TRIGGER_COMMAND_TYPE_COMBINATION )
            extract_mip_3dm_event_trigger_command_combination_params(&serializer, &parameters->combination);
        
        if( !mip_serializer_ok(&serializer) )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Configures various types of event triggers.
/// 
/// @param instance Trigger number. When function is SAVE, LOAD, or DEFAULT, this can be 0 to apply to all instances.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_save_event_trigger(struct mip_interface* device, uint8_t instance)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    insert_u8(&serializer, instance);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_EVENT_TRIGGER_CONFIG, buffer, serializer.offset);
}

/// @brief Configures various types of event triggers.
/// 
/// @param instance Trigger number. When function is SAVE, LOAD, or DEFAULT, this can be 0 to apply to all instances.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_load_event_trigger(struct mip_interface* device, uint8_t instance)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    insert_u8(&serializer, instance);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_EVENT_TRIGGER_CONFIG, buffer, serializer.offset);
}

/// @brief Configures various types of event triggers.
/// 
/// @param instance Trigger number. When function is SAVE, LOAD, or DEFAULT, this can be 0 to apply to all instances.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_default_event_trigger(struct mip_interface* device, uint8_t instance)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    insert_u8(&serializer, instance);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_EVENT_TRIGGER_CONFIG, buffer, serializer.offset);
}

void insert_mip_3dm_event_action_command(struct mip_serializer* serializer, const struct mip_3dm_event_action_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    insert_u8(serializer, self->instance);
    insert_u8(serializer, self->trigger);
    insert_mip_3dm_event_action_command_type(serializer, self->type);
    if( self->type == MIP_3DM_EVENT_ACTION_COMMAND_TYPE_GPIO )
        insert_mip_3dm_event_action_command_gpio_params(serializer, &self->parameters.gpio);
    if( self->type == MIP_3DM_EVENT_ACTION_COMMAND_TYPE_MESSAGE )
        insert_mip_3dm_event_action_command_message_params(serializer, &self->parameters.message);
}

void extract_mip_3dm_event_action_command(struct mip_serializer* serializer, struct mip_3dm_event_action_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    extract_u8(serializer, &self->instance);
    extract_u8(serializer, &self->trigger);
    extract_mip_3dm_event_action_command_type(serializer, &self->type);
    if( self->type == MIP_3DM_EVENT_ACTION_COMMAND_TYPE_GPIO )
        extract_mip_3dm_event_action_command_gpio_params(serializer, &self->parameters.gpio);
    if( self->type == MIP_3DM_EVENT_ACTION_COMMAND_TYPE_MESSAGE )
        extract_mip_3dm_event_action_command_message_params(serializer, &self->parameters.message);
}

void insert_mip_3dm_event_action_command_gpio_params(struct mip_serializer* serializer, const struct mip_3dm_event_action_command_gpio_params* self)
{
    insert_u8(serializer, self->pin);
    insert_mip_3dm_event_action_command_gpio_params_mode(serializer, self->mode);
}

void extract_mip_3dm_event_action_command_gpio_params(struct mip_serializer* serializer, struct mip_3dm_event_action_command_gpio_params* self)
{
    extract_u8(serializer, &self->pin);
    extract_mip_3dm_event_action_command_gpio_params_mode(serializer, &self->mode);
}

void insert_mip_3dm_event_action_command_gpio_params_mode(struct mip_serializer* serializer, const enum mip_3dm_event_action_command_gpio_params_mode self)
{
    return insert_u8(serializer, (uint8_t)(self));
}
void extract_mip_3dm_event_action_command_gpio_params_mode(struct mip_serializer* serializer, enum mip_3dm_event_action_command_gpio_params_mode* self)
{
    uint8_t tmp = 0;
    extract_u8(serializer, &tmp);
    *self = tmp;
}

void insert_mip_3dm_event_action_command_message_params(struct mip_serializer* serializer, const struct mip_3dm_event_action_command_message_params* self)
{
    insert_u8(serializer, self->desc_set);
    insert_u16(serializer, self->decimation);
    insert_u8(serializer, self->num_fields);
    for(unsigned int i=0; i < self->num_fields; i++)
        insert_u8(serializer, self->descriptors[i]);
}

void extract_mip_3dm_event_action_command_message_params(struct mip_serializer* serializer, struct mip_3dm_event_action_command_message_params* self)
{
    extract_u8(serializer, &self->desc_set);
    extract_u16(serializer, &self->decimation);
    extract_count(serializer, &self->num_fields, self->num_fields);
    for(unsigned int i=0; i < self->num_fields; i++)
        extract_u8(serializer, &self->descriptors[i]);
}

void insert_mip_3dm_event_action_command_type(struct mip_serializer* serializer, const enum mip_3dm_event_action_command_type self)
{
    return insert_u8(serializer, (uint8_t)(self));
}
void extract_mip_3dm_event_action_command_type(struct mip_serializer* serializer, enum mip_3dm_event_action_command_type* self)
{
    uint8_t tmp = 0;
    extract_u8(serializer, &tmp);
    *self = tmp;
}

/// @brief Configures various types of event actions.
/// 
/// @param instance Action number. When function is SAVE, LOAD, or DEFAULT, this can be 0 to apply to all instances.
/// @param trigger Trigger ID number.
/// @param type Type of action to configure.
/// @param parameters 
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_write_event_action(struct mip_interface* device, uint8_t instance, uint8_t trigger, enum mip_3dm_event_action_command_type type, const union mip_3dm_event_action_command_parameters* parameters)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    insert_u8(&serializer, instance);
    insert_u8(&serializer, trigger);
    insert_mip_3dm_event_action_command_type(&serializer, type);
    if( type == MIP_3DM_EVENT_ACTION_COMMAND_TYPE_GPIO )
        insert_mip_3dm_event_action_command_gpio_params(&serializer, &parameters->gpio);
    if( type == MIP_3DM_EVENT_ACTION_COMMAND_TYPE_MESSAGE )
        insert_mip_3dm_event_action_command_message_params(&serializer, &parameters->message);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_EVENT_ACTION_CONFIG, buffer, serializer.offset);
}

/// @brief Configures various types of event actions.
/// 
/// @param instance Action number. When function is SAVE, LOAD, or DEFAULT, this can be 0 to apply to all instances.
/// @param[out] instance Action number. When function is SAVE, LOAD, or DEFAULT, this can be 0 to apply to all instances.
/// @param[out] trigger Trigger ID number.
/// @param[out] type Type of action to configure.
/// @param[out] parameters 
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_read_event_action(struct mip_interface* device, uint8_t instance, uint8_t* trigger, enum mip_3dm_event_action_command_type* type, union mip_3dm_event_action_command_parameters* parameters)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    insert_u8(&serializer, instance);
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_EVENT_ACTION_CONFIG, buffer, serializer.offset, MIP_REPLY_DESC_3DM_EVENT_ACTION_CONFIG, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        struct mip_serializer serializer;
        mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
        
        extract_u8(&serializer, &instance);
        extract_u8(&serializer, trigger);
        extract_mip_3dm_event_action_command_type(&serializer, type);
        if( *type == MIP_3DM_EVENT_ACTION_COMMAND_TYPE_GPIO )
            extract_mip_3dm_event_action_command_gpio_params(&serializer, &parameters->gpio);
        if( *type == MIP_3DM_EVENT_ACTION_COMMAND_TYPE_MESSAGE )
            extract_mip_3dm_event_action_command_message_params(&serializer, &parameters->message);
        
        if( !mip_serializer_ok(&serializer) )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Configures various types of event actions.
/// 
/// @param instance Action number. When function is SAVE, LOAD, or DEFAULT, this can be 0 to apply to all instances.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_save_event_action(struct mip_interface* device, uint8_t instance)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    insert_u8(&serializer, instance);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_EVENT_ACTION_CONFIG, buffer, serializer.offset);
}

/// @brief Configures various types of event actions.
/// 
/// @param instance Action number. When function is SAVE, LOAD, or DEFAULT, this can be 0 to apply to all instances.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_load_event_action(struct mip_interface* device, uint8_t instance)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    insert_u8(&serializer, instance);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_EVENT_ACTION_CONFIG, buffer, serializer.offset);
}

/// @brief Configures various types of event actions.
/// 
/// @param instance Action number. When function is SAVE, LOAD, or DEFAULT, this can be 0 to apply to all instances.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_default_event_action(struct mip_interface* device, uint8_t instance)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    insert_u8(&serializer, instance);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_EVENT_ACTION_CONFIG, buffer, serializer.offset);
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

/// @brief Configures the user specified accelerometer bias
/// 
/// The user specified bias is subtracted from the calibrated accelerometer output.  Value is input in the sensor frame.
/// @param bias accelerometer bias in the sensor frame (x,y,z) [g]
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_write_accel_bias(struct mip_interface* device, const float* bias)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    for(unsigned int i=0; i < 3; i++)
        insert_float(&serializer, bias[i]);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_ACCEL_BIAS, buffer, serializer.offset);
}

/// @brief Configures the user specified accelerometer bias
/// 
/// The user specified bias is subtracted from the calibrated accelerometer output.  Value is input in the sensor frame.
/// @param[out] bias accelerometer bias in the sensor frame (x,y,z) [g]
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_read_accel_bias(struct mip_interface* device, float* bias)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_ACCEL_BIAS, buffer, serializer.offset, MIP_REPLY_DESC_3DM_ACCEL_BIAS_VECTOR, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        struct mip_serializer serializer;
        mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
        
        for(unsigned int i=0; i < 3; i++)
            extract_float(&serializer, &bias[i]);
        
        if( !mip_serializer_ok(&serializer) )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Configures the user specified accelerometer bias
/// 
/// The user specified bias is subtracted from the calibrated accelerometer output.  Value is input in the sensor frame.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_save_accel_bias(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_ACCEL_BIAS, buffer, serializer.offset);
}

/// @brief Configures the user specified accelerometer bias
/// 
/// The user specified bias is subtracted from the calibrated accelerometer output.  Value is input in the sensor frame.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_load_accel_bias(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_ACCEL_BIAS, buffer, serializer.offset);
}

/// @brief Configures the user specified accelerometer bias
/// 
/// The user specified bias is subtracted from the calibrated accelerometer output.  Value is input in the sensor frame.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_default_accel_bias(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_ACCEL_BIAS, buffer, serializer.offset);
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

/// @brief Configures the user specified gyroscope bias
/// 
/// The user specified bias is subtracted from the calibrated angular rate output.  Value is input in the sensor frame.
/// @param bias gyro bias in the sensor frame (x,y,z) [radians/second]
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_write_gyro_bias(struct mip_interface* device, const float* bias)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    for(unsigned int i=0; i < 3; i++)
        insert_float(&serializer, bias[i]);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_GYRO_BIAS, buffer, serializer.offset);
}

/// @brief Configures the user specified gyroscope bias
/// 
/// The user specified bias is subtracted from the calibrated angular rate output.  Value is input in the sensor frame.
/// @param[out] bias gyro bias in the sensor frame (x,y,z) [radians/second]
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_read_gyro_bias(struct mip_interface* device, float* bias)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_GYRO_BIAS, buffer, serializer.offset, MIP_REPLY_DESC_3DM_GYRO_BIAS_VECTOR, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        struct mip_serializer serializer;
        mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
        
        for(unsigned int i=0; i < 3; i++)
            extract_float(&serializer, &bias[i]);
        
        if( !mip_serializer_ok(&serializer) )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Configures the user specified gyroscope bias
/// 
/// The user specified bias is subtracted from the calibrated angular rate output.  Value is input in the sensor frame.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_save_gyro_bias(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_GYRO_BIAS, buffer, serializer.offset);
}

/// @brief Configures the user specified gyroscope bias
/// 
/// The user specified bias is subtracted from the calibrated angular rate output.  Value is input in the sensor frame.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_load_gyro_bias(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_GYRO_BIAS, buffer, serializer.offset);
}

/// @brief Configures the user specified gyroscope bias
/// 
/// The user specified bias is subtracted from the calibrated angular rate output.  Value is input in the sensor frame.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_default_gyro_bias(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_GYRO_BIAS, buffer, serializer.offset);
}

void insert_mip_3dm_capture_gyro_bias_command(struct mip_serializer* serializer, const struct mip_3dm_capture_gyro_bias_command* self)
{
    insert_u16(serializer, self->averaging_time_ms);
}

void extract_mip_3dm_capture_gyro_bias_command(struct mip_serializer* serializer, struct mip_3dm_capture_gyro_bias_command* self)
{
    extract_u16(serializer, &self->averaging_time_ms);
}

/// @brief Samples gyro for a specified time range and writes the averaged result to the Gyro Bias vector in RAM
/// 
/// The device will average the gyro output for the duration of "averaging_time_ms." To store the resulting vector
/// in non-volatile memory, use the Set Gyro Bias command.
/// IMPORTANT: The device must be stationary and experiencing minimum vibration for the duration of "averaging_time_ms"
/// Averaging Time range: 1000 to 30,000
/// @param averaging_time_ms Averaging time [milliseconds]
/// @param[out] bias gyro bias in the sensor frame (x,y,z) [radians/second]
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_capture_gyro_bias(struct mip_interface* device, uint16_t averaging_time_ms, float* bias)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_u16(&serializer, averaging_time_ms);
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_CAPTURE_GYRO_BIAS, buffer, serializer.offset, MIP_REPLY_DESC_3DM_GYRO_BIAS_VECTOR, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        struct mip_serializer serializer;
        mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
        
        for(unsigned int i=0; i < 3; i++)
            extract_float(&serializer, &bias[i]);
        
        if( !mip_serializer_ok(&serializer) )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
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

/// @brief Configure the user specified magnetometer hard iron offset vector
/// 
/// The values for this offset are determined empirically by external software algorithms
/// based on calibration data taken after the device is installed in its application. These values
/// can be obtained and set by using the LORD "MIP Iron Calibration" application.
/// Alternatively, on some systems, the auto-mag calibration feature may be used to capture these values in-run.
/// The offset is applied to the scaled magnetometer vector prior to output.
/// @param offset hard iron offset in the sensor frame (x,y,z) [Gauss]
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_write_mag_hard_iron_offset(struct mip_interface* device, const float* offset)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    for(unsigned int i=0; i < 3; i++)
        insert_float(&serializer, offset[i]);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_HARD_IRON_OFFSET, buffer, serializer.offset);
}

/// @brief Configure the user specified magnetometer hard iron offset vector
/// 
/// The values for this offset are determined empirically by external software algorithms
/// based on calibration data taken after the device is installed in its application. These values
/// can be obtained and set by using the LORD "MIP Iron Calibration" application.
/// Alternatively, on some systems, the auto-mag calibration feature may be used to capture these values in-run.
/// The offset is applied to the scaled magnetometer vector prior to output.
/// @param[out] offset hard iron offset in the sensor frame (x,y,z) [Gauss]
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_read_mag_hard_iron_offset(struct mip_interface* device, float* offset)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_HARD_IRON_OFFSET, buffer, serializer.offset, MIP_REPLY_DESC_3DM_HARD_IRON_OFFSET_VECTOR, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        struct mip_serializer serializer;
        mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
        
        for(unsigned int i=0; i < 3; i++)
            extract_float(&serializer, &offset[i]);
        
        if( !mip_serializer_ok(&serializer) )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Configure the user specified magnetometer hard iron offset vector
/// 
/// The values for this offset are determined empirically by external software algorithms
/// based on calibration data taken after the device is installed in its application. These values
/// can be obtained and set by using the LORD "MIP Iron Calibration" application.
/// Alternatively, on some systems, the auto-mag calibration feature may be used to capture these values in-run.
/// The offset is applied to the scaled magnetometer vector prior to output.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_save_mag_hard_iron_offset(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_HARD_IRON_OFFSET, buffer, serializer.offset);
}

/// @brief Configure the user specified magnetometer hard iron offset vector
/// 
/// The values for this offset are determined empirically by external software algorithms
/// based on calibration data taken after the device is installed in its application. These values
/// can be obtained and set by using the LORD "MIP Iron Calibration" application.
/// Alternatively, on some systems, the auto-mag calibration feature may be used to capture these values in-run.
/// The offset is applied to the scaled magnetometer vector prior to output.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_load_mag_hard_iron_offset(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_HARD_IRON_OFFSET, buffer, serializer.offset);
}

/// @brief Configure the user specified magnetometer hard iron offset vector
/// 
/// The values for this offset are determined empirically by external software algorithms
/// based on calibration data taken after the device is installed in its application. These values
/// can be obtained and set by using the LORD "MIP Iron Calibration" application.
/// Alternatively, on some systems, the auto-mag calibration feature may be used to capture these values in-run.
/// The offset is applied to the scaled magnetometer vector prior to output.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_default_mag_hard_iron_offset(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_HARD_IRON_OFFSET, buffer, serializer.offset);
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

/// @brief Configure the user specified magnetometer soft iron offset matrix
/// 
/// The values for this matrix are determined empirically by external software algorithms
/// based on calibration data taken after the device is installed in its application. These values
/// can be obtained and set by using the LORD "MIP Iron Calibration" application.
/// Alternatively, on some systems, the auto-mag calibration feature may be used to capture these values in-run.
/// The matrix is applied to the scaled magnetometer vector prior to output.
/// 
/// The matrix is in row major order:
/// EQSTART M = \begin{bmatrix} 0 &amp; 1 &amp; 2 \\ 3 &amp; 4 &amp; 5 \\ 6 &amp; 7 &amp; 8 \end{bmatrix} EQEND
/// 
/// @param offset soft iron matrix [dimensionless]
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_write_mag_soft_iron_matrix(struct mip_interface* device, const float* offset)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    for(unsigned int i=0; i < 9; i++)
        insert_float(&serializer, offset[i]);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_SOFT_IRON_MATRIX, buffer, serializer.offset);
}

/// @brief Configure the user specified magnetometer soft iron offset matrix
/// 
/// The values for this matrix are determined empirically by external software algorithms
/// based on calibration data taken after the device is installed in its application. These values
/// can be obtained and set by using the LORD "MIP Iron Calibration" application.
/// Alternatively, on some systems, the auto-mag calibration feature may be used to capture these values in-run.
/// The matrix is applied to the scaled magnetometer vector prior to output.
/// 
/// The matrix is in row major order:
/// EQSTART M = \begin{bmatrix} 0 &amp; 1 &amp; 2 \\ 3 &amp; 4 &amp; 5 \\ 6 &amp; 7 &amp; 8 \end{bmatrix} EQEND
/// 
/// @param[out] offset soft iron matrix [dimensionless]
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_read_mag_soft_iron_matrix(struct mip_interface* device, float* offset)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_SOFT_IRON_MATRIX, buffer, serializer.offset, MIP_REPLY_DESC_3DM_SOFT_IRON_COMP_MATRIX, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        struct mip_serializer serializer;
        mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
        
        for(unsigned int i=0; i < 9; i++)
            extract_float(&serializer, &offset[i]);
        
        if( !mip_serializer_ok(&serializer) )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Configure the user specified magnetometer soft iron offset matrix
/// 
/// The values for this matrix are determined empirically by external software algorithms
/// based on calibration data taken after the device is installed in its application. These values
/// can be obtained and set by using the LORD "MIP Iron Calibration" application.
/// Alternatively, on some systems, the auto-mag calibration feature may be used to capture these values in-run.
/// The matrix is applied to the scaled magnetometer vector prior to output.
/// 
/// The matrix is in row major order:
/// EQSTART M = \begin{bmatrix} 0 &amp; 1 &amp; 2 \\ 3 &amp; 4 &amp; 5 \\ 6 &amp; 7 &amp; 8 \end{bmatrix} EQEND
/// 
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_save_mag_soft_iron_matrix(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_SOFT_IRON_MATRIX, buffer, serializer.offset);
}

/// @brief Configure the user specified magnetometer soft iron offset matrix
/// 
/// The values for this matrix are determined empirically by external software algorithms
/// based on calibration data taken after the device is installed in its application. These values
/// can be obtained and set by using the LORD "MIP Iron Calibration" application.
/// Alternatively, on some systems, the auto-mag calibration feature may be used to capture these values in-run.
/// The matrix is applied to the scaled magnetometer vector prior to output.
/// 
/// The matrix is in row major order:
/// EQSTART M = \begin{bmatrix} 0 &amp; 1 &amp; 2 \\ 3 &amp; 4 &amp; 5 \\ 6 &amp; 7 &amp; 8 \end{bmatrix} EQEND
/// 
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_load_mag_soft_iron_matrix(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_SOFT_IRON_MATRIX, buffer, serializer.offset);
}

/// @brief Configure the user specified magnetometer soft iron offset matrix
/// 
/// The values for this matrix are determined empirically by external software algorithms
/// based on calibration data taken after the device is installed in its application. These values
/// can be obtained and set by using the LORD "MIP Iron Calibration" application.
/// Alternatively, on some systems, the auto-mag calibration feature may be used to capture these values in-run.
/// The matrix is applied to the scaled magnetometer vector prior to output.
/// 
/// The matrix is in row major order:
/// EQSTART M = \begin{bmatrix} 0 &amp; 1 &amp; 2 \\ 3 &amp; 4 &amp; 5 \\ 6 &amp; 7 &amp; 8 \end{bmatrix} EQEND
/// 
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_default_mag_soft_iron_matrix(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_SOFT_IRON_MATRIX, buffer, serializer.offset);
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

/// @brief Sets the sensor-to-vehicle frame transformation using Yaw, Pitch, Roll Euler angles.
/// These are the Yaw, Pitch, and Roll mounting angles of the sensor with respect to vehicle frame of reference,
/// and describe the transformation of vectors from the sensor body frame to the vehicle frame.<br/>
/// Note: This is the transformation, the inverse of the rotation defined in our legacy products.<br/>
/// The transformation may be stored in the device as a matrix or quaternion.  When Euler angles are read back from the device, they may not
/// be exactly equal to the Euler angles used to set the transformation, but they are functionally equivalent, such that they result in the same transformation.<br/>
/// <br/><br/>
/// This transformation to the vehicle frame will be applied to the following output quantities:<br/><br/>
/// IMU:<br/>
/// Scaled Acceleration<br/>
/// Scaled Gyro<br/>
/// Scaled Magnetometer<br/>
/// Delta Theta<br/>
/// Delta Velocity<br/>
/// Complementary Filter Orientation<br/>
/// <br/><br/>
/// Estimation Filter:<br/>
/// Estimated Orientation, Quaternion<br/>
/// Estimated Orientation, Matrix<br/>
/// Estimated Orientation, Euler Angles<br/>
/// Estimated Linear Acceleration<br/>
/// Estimated Angular Rate<br/>
/// Estimated Gravity Vector<br/>
/// <br/>
/// Changing this setting will force all low-pass filters, the complementary filter, and the estimation filter to reset.
/// @param roll [radians]
/// @param pitch [radians]
/// @param yaw [radians]
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_write_sensor_2_vehicle_transform_euler(struct mip_interface* device, float roll, float pitch, float yaw)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    insert_float(&serializer, roll);
    insert_float(&serializer, pitch);
    insert_float(&serializer, yaw);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_SENSOR2VEHICLE_TRANSFORM_EUL, buffer, serializer.offset);
}

/// @brief Sets the sensor-to-vehicle frame transformation using Yaw, Pitch, Roll Euler angles.
/// These are the Yaw, Pitch, and Roll mounting angles of the sensor with respect to vehicle frame of reference,
/// and describe the transformation of vectors from the sensor body frame to the vehicle frame.<br/>
/// Note: This is the transformation, the inverse of the rotation defined in our legacy products.<br/>
/// The transformation may be stored in the device as a matrix or quaternion.  When Euler angles are read back from the device, they may not
/// be exactly equal to the Euler angles used to set the transformation, but they are functionally equivalent, such that they result in the same transformation.<br/>
/// <br/><br/>
/// This transformation to the vehicle frame will be applied to the following output quantities:<br/><br/>
/// IMU:<br/>
/// Scaled Acceleration<br/>
/// Scaled Gyro<br/>
/// Scaled Magnetometer<br/>
/// Delta Theta<br/>
/// Delta Velocity<br/>
/// Complementary Filter Orientation<br/>
/// <br/><br/>
/// Estimation Filter:<br/>
/// Estimated Orientation, Quaternion<br/>
/// Estimated Orientation, Matrix<br/>
/// Estimated Orientation, Euler Angles<br/>
/// Estimated Linear Acceleration<br/>
/// Estimated Angular Rate<br/>
/// Estimated Gravity Vector<br/>
/// <br/>
/// Changing this setting will force all low-pass filters, the complementary filter, and the estimation filter to reset.
/// @param[out] roll [radians]
/// @param[out] pitch [radians]
/// @param[out] yaw [radians]
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_read_sensor_2_vehicle_transform_euler(struct mip_interface* device, float* roll, float* pitch, float* yaw)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_SENSOR2VEHICLE_TRANSFORM_EUL, buffer, serializer.offset, MIP_REPLY_DESC_3DM_SENSOR2VEHICLE_TRANSFORM_EUL, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        struct mip_serializer serializer;
        mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
        
        extract_float(&serializer, roll);
        extract_float(&serializer, pitch);
        extract_float(&serializer, yaw);
        
        if( !mip_serializer_ok(&serializer) )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Sets the sensor-to-vehicle frame transformation using Yaw, Pitch, Roll Euler angles.
/// These are the Yaw, Pitch, and Roll mounting angles of the sensor with respect to vehicle frame of reference,
/// and describe the transformation of vectors from the sensor body frame to the vehicle frame.<br/>
/// Note: This is the transformation, the inverse of the rotation defined in our legacy products.<br/>
/// The transformation may be stored in the device as a matrix or quaternion.  When Euler angles are read back from the device, they may not
/// be exactly equal to the Euler angles used to set the transformation, but they are functionally equivalent, such that they result in the same transformation.<br/>
/// <br/><br/>
/// This transformation to the vehicle frame will be applied to the following output quantities:<br/><br/>
/// IMU:<br/>
/// Scaled Acceleration<br/>
/// Scaled Gyro<br/>
/// Scaled Magnetometer<br/>
/// Delta Theta<br/>
/// Delta Velocity<br/>
/// Complementary Filter Orientation<br/>
/// <br/><br/>
/// Estimation Filter:<br/>
/// Estimated Orientation, Quaternion<br/>
/// Estimated Orientation, Matrix<br/>
/// Estimated Orientation, Euler Angles<br/>
/// Estimated Linear Acceleration<br/>
/// Estimated Angular Rate<br/>
/// Estimated Gravity Vector<br/>
/// <br/>
/// Changing this setting will force all low-pass filters, the complementary filter, and the estimation filter to reset.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_save_sensor_2_vehicle_transform_euler(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_SENSOR2VEHICLE_TRANSFORM_EUL, buffer, serializer.offset);
}

/// @brief Sets the sensor-to-vehicle frame transformation using Yaw, Pitch, Roll Euler angles.
/// These are the Yaw, Pitch, and Roll mounting angles of the sensor with respect to vehicle frame of reference,
/// and describe the transformation of vectors from the sensor body frame to the vehicle frame.<br/>
/// Note: This is the transformation, the inverse of the rotation defined in our legacy products.<br/>
/// The transformation may be stored in the device as a matrix or quaternion.  When Euler angles are read back from the device, they may not
/// be exactly equal to the Euler angles used to set the transformation, but they are functionally equivalent, such that they result in the same transformation.<br/>
/// <br/><br/>
/// This transformation to the vehicle frame will be applied to the following output quantities:<br/><br/>
/// IMU:<br/>
/// Scaled Acceleration<br/>
/// Scaled Gyro<br/>
/// Scaled Magnetometer<br/>
/// Delta Theta<br/>
/// Delta Velocity<br/>
/// Complementary Filter Orientation<br/>
/// <br/><br/>
/// Estimation Filter:<br/>
/// Estimated Orientation, Quaternion<br/>
/// Estimated Orientation, Matrix<br/>
/// Estimated Orientation, Euler Angles<br/>
/// Estimated Linear Acceleration<br/>
/// Estimated Angular Rate<br/>
/// Estimated Gravity Vector<br/>
/// <br/>
/// Changing this setting will force all low-pass filters, the complementary filter, and the estimation filter to reset.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_load_sensor_2_vehicle_transform_euler(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_SENSOR2VEHICLE_TRANSFORM_EUL, buffer, serializer.offset);
}

/// @brief Sets the sensor-to-vehicle frame transformation using Yaw, Pitch, Roll Euler angles.
/// These are the Yaw, Pitch, and Roll mounting angles of the sensor with respect to vehicle frame of reference,
/// and describe the transformation of vectors from the sensor body frame to the vehicle frame.<br/>
/// Note: This is the transformation, the inverse of the rotation defined in our legacy products.<br/>
/// The transformation may be stored in the device as a matrix or quaternion.  When Euler angles are read back from the device, they may not
/// be exactly equal to the Euler angles used to set the transformation, but they are functionally equivalent, such that they result in the same transformation.<br/>
/// <br/><br/>
/// This transformation to the vehicle frame will be applied to the following output quantities:<br/><br/>
/// IMU:<br/>
/// Scaled Acceleration<br/>
/// Scaled Gyro<br/>
/// Scaled Magnetometer<br/>
/// Delta Theta<br/>
/// Delta Velocity<br/>
/// Complementary Filter Orientation<br/>
/// <br/><br/>
/// Estimation Filter:<br/>
/// Estimated Orientation, Quaternion<br/>
/// Estimated Orientation, Matrix<br/>
/// Estimated Orientation, Euler Angles<br/>
/// Estimated Linear Acceleration<br/>
/// Estimated Angular Rate<br/>
/// Estimated Gravity Vector<br/>
/// <br/>
/// Changing this setting will force all low-pass filters, the complementary filter, and the estimation filter to reset.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_default_sensor_2_vehicle_transform_euler(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_SENSOR2VEHICLE_TRANSFORM_EUL, buffer, serializer.offset);
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

/// @brief Set the sensor to vehicle frame transformation using unit length quaternion.
/// 
/// Note: This is the transformation, the inverse of the rotation.
/// 
/// This quaternion describes the transformation of vectors from the sensor body frame to the vehicle frame of reference, and satisfies the following relationship:<br/>
/// 
/// EQSTART p^{veh} = q^{-1} p^{sen} q EQEND<br/>
/// 
/// Where:<br/>
/// EQSTART q = (q_w, q_x, q_y, q_z) EQEND is the quaternion desrcribing the transformation. <br/>
/// EQSTART p^{sen} = (0, v^{sen}_x, v^{sen}_y, v^{sen}_z) EQEND and EQSTART v^{sen} EQEND is a 3-element vector expressed in the sensor body frame.<br/>
/// EQSTART p^{veh} = (0, v^{veh}_x, v^{veh}_y, v^{veh}_z) EQEND and EQSTART v^{veh} EQEND is a 3-element vector expressed in the vehicle frame.<br/>
/// 
/// The transformation may be stored in the device as a matrix or a quaternion.  When the quaternion is read back from the device, it may not
/// be exactly equal to the quaternion used to set the transformation, but it is functionally equivalent.<br/>
/// <br/><br/>
/// This transformation affects the following output quantities:<br/><br/>
/// IMU:<br/>
/// Scaled Acceleration<br/>
/// Scaled Gyro<br/>
/// Scaled Magnetometer<br/>
/// Delta Theta<br/>
/// Delta Velocity<br/>
/// <br/><br/>
/// Estimation Filter:<br/>
/// Estimated Orientation, Quaternion<br/>
/// Estimated Orientation, Matrix<br/>
/// Estimated Orientation, Euler Angles<br/>
/// Estimated Linear Acceleration<br/>
/// Estimated Angular Rate<br/>
/// Estimated Gravity Vector<br/>
/// <br/>
/// Changing this setting will force all low-pass filters, the complementary filter, and the estimation filter to reset.
/// @param q Unit length quaternion representing transform [w, i, j, k]
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_write_sensor_2_vehicle_transform_quaternion(struct mip_interface* device, const float* q)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    for(unsigned int i=0; i < 4; i++)
        insert_float(&serializer, q[i]);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_SENSOR2VEHICLE_TRANSFORM_QUAT, buffer, serializer.offset);
}

/// @brief Set the sensor to vehicle frame transformation using unit length quaternion.
/// 
/// Note: This is the transformation, the inverse of the rotation.
/// 
/// This quaternion describes the transformation of vectors from the sensor body frame to the vehicle frame of reference, and satisfies the following relationship:<br/>
/// 
/// EQSTART p^{veh} = q^{-1} p^{sen} q EQEND<br/>
/// 
/// Where:<br/>
/// EQSTART q = (q_w, q_x, q_y, q_z) EQEND is the quaternion desrcribing the transformation. <br/>
/// EQSTART p^{sen} = (0, v^{sen}_x, v^{sen}_y, v^{sen}_z) EQEND and EQSTART v^{sen} EQEND is a 3-element vector expressed in the sensor body frame.<br/>
/// EQSTART p^{veh} = (0, v^{veh}_x, v^{veh}_y, v^{veh}_z) EQEND and EQSTART v^{veh} EQEND is a 3-element vector expressed in the vehicle frame.<br/>
/// 
/// The transformation may be stored in the device as a matrix or a quaternion.  When the quaternion is read back from the device, it may not
/// be exactly equal to the quaternion used to set the transformation, but it is functionally equivalent.<br/>
/// <br/><br/>
/// This transformation affects the following output quantities:<br/><br/>
/// IMU:<br/>
/// Scaled Acceleration<br/>
/// Scaled Gyro<br/>
/// Scaled Magnetometer<br/>
/// Delta Theta<br/>
/// Delta Velocity<br/>
/// <br/><br/>
/// Estimation Filter:<br/>
/// Estimated Orientation, Quaternion<br/>
/// Estimated Orientation, Matrix<br/>
/// Estimated Orientation, Euler Angles<br/>
/// Estimated Linear Acceleration<br/>
/// Estimated Angular Rate<br/>
/// Estimated Gravity Vector<br/>
/// <br/>
/// Changing this setting will force all low-pass filters, the complementary filter, and the estimation filter to reset.
/// @param[out] q Unit length quaternion representing transform [w, i, j, k]
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_read_sensor_2_vehicle_transform_quaternion(struct mip_interface* device, float* q)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_SENSOR2VEHICLE_TRANSFORM_QUAT, buffer, serializer.offset, MIP_REPLY_DESC_3DM_SENSOR2VEHICLE_TRANSFORM_QUAT, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        struct mip_serializer serializer;
        mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
        
        for(unsigned int i=0; i < 4; i++)
            extract_float(&serializer, &q[i]);
        
        if( !mip_serializer_ok(&serializer) )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Set the sensor to vehicle frame transformation using unit length quaternion.
/// 
/// Note: This is the transformation, the inverse of the rotation.
/// 
/// This quaternion describes the transformation of vectors from the sensor body frame to the vehicle frame of reference, and satisfies the following relationship:<br/>
/// 
/// EQSTART p^{veh} = q^{-1} p^{sen} q EQEND<br/>
/// 
/// Where:<br/>
/// EQSTART q = (q_w, q_x, q_y, q_z) EQEND is the quaternion desrcribing the transformation. <br/>
/// EQSTART p^{sen} = (0, v^{sen}_x, v^{sen}_y, v^{sen}_z) EQEND and EQSTART v^{sen} EQEND is a 3-element vector expressed in the sensor body frame.<br/>
/// EQSTART p^{veh} = (0, v^{veh}_x, v^{veh}_y, v^{veh}_z) EQEND and EQSTART v^{veh} EQEND is a 3-element vector expressed in the vehicle frame.<br/>
/// 
/// The transformation may be stored in the device as a matrix or a quaternion.  When the quaternion is read back from the device, it may not
/// be exactly equal to the quaternion used to set the transformation, but it is functionally equivalent.<br/>
/// <br/><br/>
/// This transformation affects the following output quantities:<br/><br/>
/// IMU:<br/>
/// Scaled Acceleration<br/>
/// Scaled Gyro<br/>
/// Scaled Magnetometer<br/>
/// Delta Theta<br/>
/// Delta Velocity<br/>
/// <br/><br/>
/// Estimation Filter:<br/>
/// Estimated Orientation, Quaternion<br/>
/// Estimated Orientation, Matrix<br/>
/// Estimated Orientation, Euler Angles<br/>
/// Estimated Linear Acceleration<br/>
/// Estimated Angular Rate<br/>
/// Estimated Gravity Vector<br/>
/// <br/>
/// Changing this setting will force all low-pass filters, the complementary filter, and the estimation filter to reset.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_save_sensor_2_vehicle_transform_quaternion(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_SENSOR2VEHICLE_TRANSFORM_QUAT, buffer, serializer.offset);
}

/// @brief Set the sensor to vehicle frame transformation using unit length quaternion.
/// 
/// Note: This is the transformation, the inverse of the rotation.
/// 
/// This quaternion describes the transformation of vectors from the sensor body frame to the vehicle frame of reference, and satisfies the following relationship:<br/>
/// 
/// EQSTART p^{veh} = q^{-1} p^{sen} q EQEND<br/>
/// 
/// Where:<br/>
/// EQSTART q = (q_w, q_x, q_y, q_z) EQEND is the quaternion desrcribing the transformation. <br/>
/// EQSTART p^{sen} = (0, v^{sen}_x, v^{sen}_y, v^{sen}_z) EQEND and EQSTART v^{sen} EQEND is a 3-element vector expressed in the sensor body frame.<br/>
/// EQSTART p^{veh} = (0, v^{veh}_x, v^{veh}_y, v^{veh}_z) EQEND and EQSTART v^{veh} EQEND is a 3-element vector expressed in the vehicle frame.<br/>
/// 
/// The transformation may be stored in the device as a matrix or a quaternion.  When the quaternion is read back from the device, it may not
/// be exactly equal to the quaternion used to set the transformation, but it is functionally equivalent.<br/>
/// <br/><br/>
/// This transformation affects the following output quantities:<br/><br/>
/// IMU:<br/>
/// Scaled Acceleration<br/>
/// Scaled Gyro<br/>
/// Scaled Magnetometer<br/>
/// Delta Theta<br/>
/// Delta Velocity<br/>
/// <br/><br/>
/// Estimation Filter:<br/>
/// Estimated Orientation, Quaternion<br/>
/// Estimated Orientation, Matrix<br/>
/// Estimated Orientation, Euler Angles<br/>
/// Estimated Linear Acceleration<br/>
/// Estimated Angular Rate<br/>
/// Estimated Gravity Vector<br/>
/// <br/>
/// Changing this setting will force all low-pass filters, the complementary filter, and the estimation filter to reset.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_load_sensor_2_vehicle_transform_quaternion(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_SENSOR2VEHICLE_TRANSFORM_QUAT, buffer, serializer.offset);
}

/// @brief Set the sensor to vehicle frame transformation using unit length quaternion.
/// 
/// Note: This is the transformation, the inverse of the rotation.
/// 
/// This quaternion describes the transformation of vectors from the sensor body frame to the vehicle frame of reference, and satisfies the following relationship:<br/>
/// 
/// EQSTART p^{veh} = q^{-1} p^{sen} q EQEND<br/>
/// 
/// Where:<br/>
/// EQSTART q = (q_w, q_x, q_y, q_z) EQEND is the quaternion desrcribing the transformation. <br/>
/// EQSTART p^{sen} = (0, v^{sen}_x, v^{sen}_y, v^{sen}_z) EQEND and EQSTART v^{sen} EQEND is a 3-element vector expressed in the sensor body frame.<br/>
/// EQSTART p^{veh} = (0, v^{veh}_x, v^{veh}_y, v^{veh}_z) EQEND and EQSTART v^{veh} EQEND is a 3-element vector expressed in the vehicle frame.<br/>
/// 
/// The transformation may be stored in the device as a matrix or a quaternion.  When the quaternion is read back from the device, it may not
/// be exactly equal to the quaternion used to set the transformation, but it is functionally equivalent.<br/>
/// <br/><br/>
/// This transformation affects the following output quantities:<br/><br/>
/// IMU:<br/>
/// Scaled Acceleration<br/>
/// Scaled Gyro<br/>
/// Scaled Magnetometer<br/>
/// Delta Theta<br/>
/// Delta Velocity<br/>
/// <br/><br/>
/// Estimation Filter:<br/>
/// Estimated Orientation, Quaternion<br/>
/// Estimated Orientation, Matrix<br/>
/// Estimated Orientation, Euler Angles<br/>
/// Estimated Linear Acceleration<br/>
/// Estimated Angular Rate<br/>
/// Estimated Gravity Vector<br/>
/// <br/>
/// Changing this setting will force all low-pass filters, the complementary filter, and the estimation filter to reset.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_default_sensor_2_vehicle_transform_quaternion(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_SENSOR2VEHICLE_TRANSFORM_QUAT, buffer, serializer.offset);
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

/// @brief Set the sensor to vehicle frame transformation using a using a 3 x 3 direction cosine matrix EQSTART M_{ned}^{veh} EQEND, stored in row-major order in a 9-element array.
/// 
/// These angles define the transformation of vectors from the sensor body frame to the fixed vehicle frame, according to:<br/>
/// EQSTART v^{veh} = M_{sen}^{veh} v^{sen} EQEND<br/>
/// 
/// Where:<br/>
/// 
/// EQSTART v^{sen} EQEND is a 3-element vector expressed in the sensor body frame. <br/>
/// EQSTART v^{veh} EQEND is the same 3-element vector expressed in the vehicle frame.  <br/>
/// <br/>
/// The matrix elements are stored is row-major order: EQSTART M_{sen}^{veh} = \begin{bmatrix} M_{11}, M_{12}, M_{13}, M_{21}, M_{22}, M_{23}, M_{31}, M_{32}, M_{33} \end{bmatrix} EQEND
/// The transformation may be stored in the device as a matrix or a quaternion. When EQSTART M_{sen}^{veh} EQEND is read back from the device, it may not
/// be exactly equal to array used to set the transformation, but it is functionally equivalent.<br/>
/// <br/><br/>
/// This transformation affects the following output quantities:<br/><br/>
/// IMU:<br/>
/// Scaled Acceleration<br/>
/// Scaled Gyro<br/>
/// Scaled Magnetometer<br/>
/// Delta Theta<br/>
/// Delta Velocity<br/>
/// <br/><br/>
/// Estimation Filter:<br/>
/// Estimated Orientation, Quaternion<br/>
/// Estimated Orientation, Matrix<br/>
/// Estimated Orientation, Euler Angles<br/>
/// Estimated Linear Acceleration<br/>
/// Estimated Angular Rate<br/>
/// Estimated Gravity Vector<br/>
/// <br/>
/// Changing this setting will force all low-pass filters, the complementary filter, and the estimation filter to reset.
/// @param dcm 3 x 3 direction cosine matrix, stored in row-major order
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_write_sensor_2_vehicle_transform_dcm(struct mip_interface* device, const float* dcm)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    for(unsigned int i=0; i < 9; i++)
        insert_float(&serializer, dcm[i]);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_SENSOR2VEHICLE_TRANSFORM_DCM, buffer, serializer.offset);
}

/// @brief Set the sensor to vehicle frame transformation using a using a 3 x 3 direction cosine matrix EQSTART M_{ned}^{veh} EQEND, stored in row-major order in a 9-element array.
/// 
/// These angles define the transformation of vectors from the sensor body frame to the fixed vehicle frame, according to:<br/>
/// EQSTART v^{veh} = M_{sen}^{veh} v^{sen} EQEND<br/>
/// 
/// Where:<br/>
/// 
/// EQSTART v^{sen} EQEND is a 3-element vector expressed in the sensor body frame. <br/>
/// EQSTART v^{veh} EQEND is the same 3-element vector expressed in the vehicle frame.  <br/>
/// <br/>
/// The matrix elements are stored is row-major order: EQSTART M_{sen}^{veh} = \begin{bmatrix} M_{11}, M_{12}, M_{13}, M_{21}, M_{22}, M_{23}, M_{31}, M_{32}, M_{33} \end{bmatrix} EQEND
/// The transformation may be stored in the device as a matrix or a quaternion. When EQSTART M_{sen}^{veh} EQEND is read back from the device, it may not
/// be exactly equal to array used to set the transformation, but it is functionally equivalent.<br/>
/// <br/><br/>
/// This transformation affects the following output quantities:<br/><br/>
/// IMU:<br/>
/// Scaled Acceleration<br/>
/// Scaled Gyro<br/>
/// Scaled Magnetometer<br/>
/// Delta Theta<br/>
/// Delta Velocity<br/>
/// <br/><br/>
/// Estimation Filter:<br/>
/// Estimated Orientation, Quaternion<br/>
/// Estimated Orientation, Matrix<br/>
/// Estimated Orientation, Euler Angles<br/>
/// Estimated Linear Acceleration<br/>
/// Estimated Angular Rate<br/>
/// Estimated Gravity Vector<br/>
/// <br/>
/// Changing this setting will force all low-pass filters, the complementary filter, and the estimation filter to reset.
/// @param[out] dcm 3 x 3 direction cosine matrix, stored in row-major order
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_read_sensor_2_vehicle_transform_dcm(struct mip_interface* device, float* dcm)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_SENSOR2VEHICLE_TRANSFORM_DCM, buffer, serializer.offset, MIP_REPLY_DESC_3DM_SENSOR2VEHICLE_TRANSFORM_DCM, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        struct mip_serializer serializer;
        mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
        
        for(unsigned int i=0; i < 9; i++)
            extract_float(&serializer, &dcm[i]);
        
        if( !mip_serializer_ok(&serializer) )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Set the sensor to vehicle frame transformation using a using a 3 x 3 direction cosine matrix EQSTART M_{ned}^{veh} EQEND, stored in row-major order in a 9-element array.
/// 
/// These angles define the transformation of vectors from the sensor body frame to the fixed vehicle frame, according to:<br/>
/// EQSTART v^{veh} = M_{sen}^{veh} v^{sen} EQEND<br/>
/// 
/// Where:<br/>
/// 
/// EQSTART v^{sen} EQEND is a 3-element vector expressed in the sensor body frame. <br/>
/// EQSTART v^{veh} EQEND is the same 3-element vector expressed in the vehicle frame.  <br/>
/// <br/>
/// The matrix elements are stored is row-major order: EQSTART M_{sen}^{veh} = \begin{bmatrix} M_{11}, M_{12}, M_{13}, M_{21}, M_{22}, M_{23}, M_{31}, M_{32}, M_{33} \end{bmatrix} EQEND
/// The transformation may be stored in the device as a matrix or a quaternion. When EQSTART M_{sen}^{veh} EQEND is read back from the device, it may not
/// be exactly equal to array used to set the transformation, but it is functionally equivalent.<br/>
/// <br/><br/>
/// This transformation affects the following output quantities:<br/><br/>
/// IMU:<br/>
/// Scaled Acceleration<br/>
/// Scaled Gyro<br/>
/// Scaled Magnetometer<br/>
/// Delta Theta<br/>
/// Delta Velocity<br/>
/// <br/><br/>
/// Estimation Filter:<br/>
/// Estimated Orientation, Quaternion<br/>
/// Estimated Orientation, Matrix<br/>
/// Estimated Orientation, Euler Angles<br/>
/// Estimated Linear Acceleration<br/>
/// Estimated Angular Rate<br/>
/// Estimated Gravity Vector<br/>
/// <br/>
/// Changing this setting will force all low-pass filters, the complementary filter, and the estimation filter to reset.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_save_sensor_2_vehicle_transform_dcm(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_SENSOR2VEHICLE_TRANSFORM_DCM, buffer, serializer.offset);
}

/// @brief Set the sensor to vehicle frame transformation using a using a 3 x 3 direction cosine matrix EQSTART M_{ned}^{veh} EQEND, stored in row-major order in a 9-element array.
/// 
/// These angles define the transformation of vectors from the sensor body frame to the fixed vehicle frame, according to:<br/>
/// EQSTART v^{veh} = M_{sen}^{veh} v^{sen} EQEND<br/>
/// 
/// Where:<br/>
/// 
/// EQSTART v^{sen} EQEND is a 3-element vector expressed in the sensor body frame. <br/>
/// EQSTART v^{veh} EQEND is the same 3-element vector expressed in the vehicle frame.  <br/>
/// <br/>
/// The matrix elements are stored is row-major order: EQSTART M_{sen}^{veh} = \begin{bmatrix} M_{11}, M_{12}, M_{13}, M_{21}, M_{22}, M_{23}, M_{31}, M_{32}, M_{33} \end{bmatrix} EQEND
/// The transformation may be stored in the device as a matrix or a quaternion. When EQSTART M_{sen}^{veh} EQEND is read back from the device, it may not
/// be exactly equal to array used to set the transformation, but it is functionally equivalent.<br/>
/// <br/><br/>
/// This transformation affects the following output quantities:<br/><br/>
/// IMU:<br/>
/// Scaled Acceleration<br/>
/// Scaled Gyro<br/>
/// Scaled Magnetometer<br/>
/// Delta Theta<br/>
/// Delta Velocity<br/>
/// <br/><br/>
/// Estimation Filter:<br/>
/// Estimated Orientation, Quaternion<br/>
/// Estimated Orientation, Matrix<br/>
/// Estimated Orientation, Euler Angles<br/>
/// Estimated Linear Acceleration<br/>
/// Estimated Angular Rate<br/>
/// Estimated Gravity Vector<br/>
/// <br/>
/// Changing this setting will force all low-pass filters, the complementary filter, and the estimation filter to reset.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_load_sensor_2_vehicle_transform_dcm(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_SENSOR2VEHICLE_TRANSFORM_DCM, buffer, serializer.offset);
}

/// @brief Set the sensor to vehicle frame transformation using a using a 3 x 3 direction cosine matrix EQSTART M_{ned}^{veh} EQEND, stored in row-major order in a 9-element array.
/// 
/// These angles define the transformation of vectors from the sensor body frame to the fixed vehicle frame, according to:<br/>
/// EQSTART v^{veh} = M_{sen}^{veh} v^{sen} EQEND<br/>
/// 
/// Where:<br/>
/// 
/// EQSTART v^{sen} EQEND is a 3-element vector expressed in the sensor body frame. <br/>
/// EQSTART v^{veh} EQEND is the same 3-element vector expressed in the vehicle frame.  <br/>
/// <br/>
/// The matrix elements are stored is row-major order: EQSTART M_{sen}^{veh} = \begin{bmatrix} M_{11}, M_{12}, M_{13}, M_{21}, M_{22}, M_{23}, M_{31}, M_{32}, M_{33} \end{bmatrix} EQEND
/// The transformation may be stored in the device as a matrix or a quaternion. When EQSTART M_{sen}^{veh} EQEND is read back from the device, it may not
/// be exactly equal to array used to set the transformation, but it is functionally equivalent.<br/>
/// <br/><br/>
/// This transformation affects the following output quantities:<br/><br/>
/// IMU:<br/>
/// Scaled Acceleration<br/>
/// Scaled Gyro<br/>
/// Scaled Magnetometer<br/>
/// Delta Theta<br/>
/// Delta Velocity<br/>
/// <br/><br/>
/// Estimation Filter:<br/>
/// Estimated Orientation, Quaternion<br/>
/// Estimated Orientation, Matrix<br/>
/// Estimated Orientation, Euler Angles<br/>
/// Estimated Linear Acceleration<br/>
/// Estimated Angular Rate<br/>
/// Estimated Gravity Vector<br/>
/// <br/>
/// Changing this setting will force all low-pass filters, the complementary filter, and the estimation filter to reset.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_default_sensor_2_vehicle_transform_dcm(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_SENSOR2VEHICLE_TRANSFORM_DCM, buffer, serializer.offset);
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

/// @brief Configure the settings for the complementary filter which produces the following (0x80) descriptor set values: attitude matrix (0x80,09), quaternion (0x80,0A), and  Euler angle (0x80,0C) outputs.
/// 
/// The filter can be configured to correct for pitch and roll using the accelerometer (with the assumption that linear acceleration is minimal),
/// and to correct for heading using the magnetomer (with the assumption that the local magnetic field is dominated by the Earth's own magnetic field).
/// Pitch/roll and heading corrections each have their own configurable time constants, with a valid range of 1-1000 seconds. The default time constant is 10 seconds.
/// @param pitch_roll_enable Enable Pitch/Roll corrections
/// @param heading_enable Enable Heading corrections (only available on devices with magnetometer)
/// @param pitch_roll_time_constant Time constant associated with the pitch/roll corrections [s]
/// @param heading_time_constant Time constant associated with the heading corrections [s]
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_write_complementary_filter(struct mip_interface* device, bool pitch_roll_enable, bool heading_enable, float pitch_roll_time_constant, float heading_time_constant)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    insert_bool(&serializer, pitch_roll_enable);
    insert_bool(&serializer, heading_enable);
    insert_float(&serializer, pitch_roll_time_constant);
    insert_float(&serializer, heading_time_constant);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_LEGACY_COMP_FILTER, buffer, serializer.offset);
}

/// @brief Configure the settings for the complementary filter which produces the following (0x80) descriptor set values: attitude matrix (0x80,09), quaternion (0x80,0A), and  Euler angle (0x80,0C) outputs.
/// 
/// The filter can be configured to correct for pitch and roll using the accelerometer (with the assumption that linear acceleration is minimal),
/// and to correct for heading using the magnetomer (with the assumption that the local magnetic field is dominated by the Earth's own magnetic field).
/// Pitch/roll and heading corrections each have their own configurable time constants, with a valid range of 1-1000 seconds. The default time constant is 10 seconds.
/// @param[out] pitch_roll_enable Enable Pitch/Roll corrections
/// @param[out] heading_enable Enable Heading corrections (only available on devices with magnetometer)
/// @param[out] pitch_roll_time_constant Time constant associated with the pitch/roll corrections [s]
/// @param[out] heading_time_constant Time constant associated with the heading corrections [s]
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_read_complementary_filter(struct mip_interface* device, bool* pitch_roll_enable, bool* heading_enable, float* pitch_roll_time_constant, float* heading_time_constant)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_LEGACY_COMP_FILTER, buffer, serializer.offset, MIP_REPLY_DESC_3DM_LEGACY_COMP_FILTER, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        struct mip_serializer serializer;
        mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
        
        extract_bool(&serializer, pitch_roll_enable);
        extract_bool(&serializer, heading_enable);
        extract_float(&serializer, pitch_roll_time_constant);
        extract_float(&serializer, heading_time_constant);
        
        if( !mip_serializer_ok(&serializer) )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Configure the settings for the complementary filter which produces the following (0x80) descriptor set values: attitude matrix (0x80,09), quaternion (0x80,0A), and  Euler angle (0x80,0C) outputs.
/// 
/// The filter can be configured to correct for pitch and roll using the accelerometer (with the assumption that linear acceleration is minimal),
/// and to correct for heading using the magnetomer (with the assumption that the local magnetic field is dominated by the Earth's own magnetic field).
/// Pitch/roll and heading corrections each have their own configurable time constants, with a valid range of 1-1000 seconds. The default time constant is 10 seconds.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_save_complementary_filter(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_LEGACY_COMP_FILTER, buffer, serializer.offset);
}

/// @brief Configure the settings for the complementary filter which produces the following (0x80) descriptor set values: attitude matrix (0x80,09), quaternion (0x80,0A), and  Euler angle (0x80,0C) outputs.
/// 
/// The filter can be configured to correct for pitch and roll using the accelerometer (with the assumption that linear acceleration is minimal),
/// and to correct for heading using the magnetomer (with the assumption that the local magnetic field is dominated by the Earth's own magnetic field).
/// Pitch/roll and heading corrections each have their own configurable time constants, with a valid range of 1-1000 seconds. The default time constant is 10 seconds.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_load_complementary_filter(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_LEGACY_COMP_FILTER, buffer, serializer.offset);
}

/// @brief Configure the settings for the complementary filter which produces the following (0x80) descriptor set values: attitude matrix (0x80,09), quaternion (0x80,0A), and  Euler angle (0x80,0C) outputs.
/// 
/// The filter can be configured to correct for pitch and roll using the accelerometer (with the assumption that linear acceleration is minimal),
/// and to correct for heading using the magnetomer (with the assumption that the local magnetic field is dominated by the Earth's own magnetic field).
/// Pitch/roll and heading corrections each have their own configurable time constants, with a valid range of 1-1000 seconds. The default time constant is 10 seconds.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_default_complementary_filter(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_LEGACY_COMP_FILTER, buffer, serializer.offset);
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

/// @brief Changes the IMU sensor gain.
/// 
/// This allows you to optimize the range to get the best accuracy and performance
/// while minimizing overrange events.
/// 
/// Use the 3DM Get Calibrated Sensor Ranges (0x0C,0x53) command to determine
/// the appropriate setting value for your application. Using values other than
/// those specified may result in a NACK or inaccurate measurement data.
/// @param sensor Which type of sensor will get the new range value.
/// @param setting Use the 3DM Get Calibrated Sensor Ranges (0x0C,0x53) command to determine this value.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_write_sensor_range(struct mip_interface* device, enum mip_sensor_range_type sensor, uint8_t setting)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    insert_mip_sensor_range_type(&serializer, sensor);
    insert_u8(&serializer, setting);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_SENSOR_RANGE, buffer, serializer.offset);
}

/// @brief Changes the IMU sensor gain.
/// 
/// This allows you to optimize the range to get the best accuracy and performance
/// while minimizing overrange events.
/// 
/// Use the 3DM Get Calibrated Sensor Ranges (0x0C,0x53) command to determine
/// the appropriate setting value for your application. Using values other than
/// those specified may result in a NACK or inaccurate measurement data.
/// @param sensor Which type of sensor will get the new range value.
/// @param[out] sensor Which type of sensor will get the new range value.
/// @param[out] setting Use the 3DM Get Calibrated Sensor Ranges (0x0C,0x53) command to determine this value.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_read_sensor_range(struct mip_interface* device, enum mip_sensor_range_type sensor, uint8_t* setting)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    insert_mip_sensor_range_type(&serializer, sensor);
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_SENSOR_RANGE, buffer, serializer.offset, MIP_REPLY_DESC_3DM_SENSOR_RANGE, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        struct mip_serializer serializer;
        mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
        
        extract_mip_sensor_range_type(&serializer, &sensor);
        extract_u8(&serializer, setting);
        
        if( !mip_serializer_ok(&serializer) )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Changes the IMU sensor gain.
/// 
/// This allows you to optimize the range to get the best accuracy and performance
/// while minimizing overrange events.
/// 
/// Use the 3DM Get Calibrated Sensor Ranges (0x0C,0x53) command to determine
/// the appropriate setting value for your application. Using values other than
/// those specified may result in a NACK or inaccurate measurement data.
/// @param sensor Which type of sensor will get the new range value.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_save_sensor_range(struct mip_interface* device, enum mip_sensor_range_type sensor)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    insert_mip_sensor_range_type(&serializer, sensor);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_SENSOR_RANGE, buffer, serializer.offset);
}

/// @brief Changes the IMU sensor gain.
/// 
/// This allows you to optimize the range to get the best accuracy and performance
/// while minimizing overrange events.
/// 
/// Use the 3DM Get Calibrated Sensor Ranges (0x0C,0x53) command to determine
/// the appropriate setting value for your application. Using values other than
/// those specified may result in a NACK or inaccurate measurement data.
/// @param sensor Which type of sensor will get the new range value.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_load_sensor_range(struct mip_interface* device, enum mip_sensor_range_type sensor)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    insert_mip_sensor_range_type(&serializer, sensor);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_SENSOR_RANGE, buffer, serializer.offset);
}

/// @brief Changes the IMU sensor gain.
/// 
/// This allows you to optimize the range to get the best accuracy and performance
/// while minimizing overrange events.
/// 
/// Use the 3DM Get Calibrated Sensor Ranges (0x0C,0x53) command to determine
/// the appropriate setting value for your application. Using values other than
/// those specified may result in a NACK or inaccurate measurement data.
/// @param sensor Which type of sensor will get the new range value.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_default_sensor_range(struct mip_interface* device, enum mip_sensor_range_type sensor)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    insert_mip_sensor_range_type(&serializer, sensor);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_SENSOR_RANGE, buffer, serializer.offset);
}

void insert_mip_3dm_calibrated_sensor_ranges_command(struct mip_serializer* serializer, const struct mip_3dm_calibrated_sensor_ranges_command* self)
{
    insert_mip_sensor_range_type(serializer, self->sensor);
}

void extract_mip_3dm_calibrated_sensor_ranges_command(struct mip_serializer* serializer, struct mip_3dm_calibrated_sensor_ranges_command* self)
{
    extract_mip_sensor_range_type(serializer, &self->sensor);
}

void insert_mip_3dm_calibrated_sensor_ranges_command_entry(struct mip_serializer* serializer, const struct mip_3dm_calibrated_sensor_ranges_command_entry* self)
{
    insert_u8(serializer, self->setting);
    insert_float(serializer, self->range);
}

void extract_mip_3dm_calibrated_sensor_ranges_command_entry(struct mip_serializer* serializer, struct mip_3dm_calibrated_sensor_ranges_command_entry* self)
{
    extract_u8(serializer, &self->setting);
    extract_float(serializer, &self->range);
}

/// @brief Returns the supported sensor ranges which may be used with the 3DM Sensor Range (0x0C,0x52) command.
/// 
/// The response includes an array of (u8, float) pairs which map each allowed setting
/// to the corresponding maximum range in physical units. See SensorRangeType for units.
/// @param sensor The sensor to query. Cannot be ALL.
/// @param[out] sensor The sensor type from the command.
/// @param[out] num_ranges Number of supported ranges.
/// @param[out] ranges List of possible range settings.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_3dm_calibrated_sensor_ranges(struct mip_interface* device, enum mip_sensor_range_type sensor, uint8_t* num_ranges, struct mip_3dm_calibrated_sensor_ranges_command_entry* ranges)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_sensor_range_type(&serializer, sensor);
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_CALIBRATED_RANGES, buffer, serializer.offset, MIP_REPLY_DESC_3DM_CALIBRATED_RANGES, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        struct mip_serializer serializer;
        mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
        
        extract_mip_sensor_range_type(&serializer, &sensor);
        extract_count(&serializer, num_ranges, *num_ranges);
        for(unsigned int i=0; i < *num_ranges; i++)
            extract_mip_3dm_calibrated_sensor_ranges_command_entry(&serializer, &ranges[i]);
        
        if( !mip_serializer_ok(&serializer) )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}


#ifdef __cplusplus
} // namespace C
} // namespace mscl
} // extern "C"
#endif // __cplusplus

