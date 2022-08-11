
#include "commands_base.h"

#include "../../utils/serialization.h"
#include "../mip_interface.h"

#include <assert.h>


#ifdef __cplusplus
namespace mip {
namespace C {
extern "C" {

#endif // __cplusplus
struct mip_interface;
struct mip_serializer;


////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////

void insert_mip_base_device_info(struct mip_serializer* serializer, const struct mip_base_device_info* self)
{
    insert_u16(serializer, self->firmware_version);
    for(unsigned int i=0; i < 16; i++)
        insert_char(serializer, self->model_name[i]);
    for(unsigned int i=0; i < 16; i++)
        insert_char(serializer, self->model_number[i]);
    for(unsigned int i=0; i < 16; i++)
        insert_char(serializer, self->serial_number[i]);
    for(unsigned int i=0; i < 16; i++)
        insert_char(serializer, self->lot_number[i]);
    for(unsigned int i=0; i < 16; i++)
        insert_char(serializer, self->device_options[i]);
}

void extract_mip_base_device_info(struct mip_serializer* serializer, struct mip_base_device_info* self)
{
    extract_u16(serializer, &self->firmware_version);
    for(unsigned int i=0; i < 16; i++)
        extract_char(serializer, &self->model_name[i]);
    for(unsigned int i=0; i < 16; i++)
        extract_char(serializer, &self->model_number[i]);
    for(unsigned int i=0; i < 16; i++)
        extract_char(serializer, &self->serial_number[i]);
    for(unsigned int i=0; i < 16; i++)
        extract_char(serializer, &self->lot_number[i]);
    for(unsigned int i=0; i < 16; i++)
        extract_char(serializer, &self->device_options[i]);
}

void insert_mip_time_format(struct mip_serializer* serializer, const enum mip_time_format self)
{
    return insert_u8(serializer, (uint8_t)(self));
}
void extract_mip_time_format(struct mip_serializer* serializer, enum mip_time_format* self)
{
    uint8_t tmp = 0;
    extract_u8(serializer, &tmp);
    *self = tmp;
}

void insert_mip_commanded_test_bits_gq7(struct mip_serializer* serializer, const enum mip_commanded_test_bits_gq7 self)
{
    return insert_u32(serializer, (uint32_t)(self));
}
void extract_mip_commanded_test_bits_gq7(struct mip_serializer* serializer, enum mip_commanded_test_bits_gq7* self)
{
    uint32_t tmp = 0;
    extract_u32(serializer, &tmp);
    *self = tmp;
}


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

/// @brief Test Communications with a device.
/// 
/// The Device will respond with an ACK, if present and operating correctly.
/// 
/// If the device is not in a normal operating mode, it may NACK.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_base_ping(struct mip_interface* device)
{
    return mip_interface_run_command(device, MIP_BASE_CMD_DESC_SET, MIP_CMD_DESC_BASE_PING, NULL, 0);
}

/// @brief Turn off all device data streams.
/// 
/// The Device will respond with an ACK, if present and operating correctly.
/// This command will suspend streaming (if enabled) or wake the device from sleep (if sleeping) to allow it to respond to status and setup commands.
/// You may restore the device mode by issuing the Resume command.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_base_set_idle(struct mip_interface* device)
{
    return mip_interface_run_command(device, MIP_BASE_CMD_DESC_SET, MIP_CMD_DESC_BASE_SET_TO_IDLE, NULL, 0);
}

/// @brief Get the device ID strings and firmware version number.
/// 
/// @param[out] device_info 
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_base_get_device_info(struct mip_interface* device, struct mip_base_device_info* device_info)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_BASE_CMD_DESC_SET, MIP_CMD_DESC_BASE_GET_DEVICE_INFO, NULL, 0, MIP_REPLY_DESC_BASE_DEVICE_INFO, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        struct mip_serializer serializer;
        mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
        
        extract_mip_base_device_info(&serializer, device_info);
        
        if( !mip_serializer_ok(&serializer) )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Get the command and data descriptors supported by the device.
/// 
/// Reply has two fields: "ACK/NACK" and "Descriptors". The "Descriptors" field is an array of 16 bit values.
/// The MSB specifies the descriptor set and the LSB specifies the descriptor.
/// @param[out] descriptors_count 
/// @param[out] descriptors 
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_base_get_device_descriptors(struct mip_interface* device, uint8_t* descriptors_count, uint16_t* descriptors)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_BASE_CMD_DESC_SET, MIP_CMD_DESC_BASE_GET_DEVICE_DESCRIPTORS, NULL, 0, MIP_REPLY_DESC_BASE_DEVICE_DESCRIPTORS, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        struct mip_serializer serializer;
        mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
        
        extract_u8(&serializer, descriptors_count);
        for(unsigned int i=0; (i < *descriptors_count) && mip_serializer_ok(&serializer); i++)
            extract_u16(&serializer, &descriptors[i]);
        
        if( !mip_serializer_ok(&serializer) )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Run the device Built-In Test (BIT).
/// 
/// The Built-In Test command always returns a 32 bit value.
/// A value of 0 means that all tests passed.
/// A non-zero value indicates that not all tests passed.
/// Reference the device user manual to decode the result.
/// @param[out] result 
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_base_built_in_test(struct mip_interface* device, uint32_t* result)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_BASE_CMD_DESC_SET, MIP_CMD_DESC_BASE_BUILT_IN_TEST, NULL, 0, MIP_REPLY_DESC_BASE_BUILT_IN_TEST, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        struct mip_serializer serializer;
        mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
        
        extract_u32(&serializer, result);
        
        if( !mip_serializer_ok(&serializer) )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Take the device out of idle mode.
/// 
/// The device responds with ACK upon success.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_base_resume(struct mip_interface* device)
{
    return mip_interface_run_command(device, MIP_BASE_CMD_DESC_SET, MIP_CMD_DESC_BASE_RESUME, NULL, 0);
}

/// @brief Get the command and data descriptors supported by the device.
/// 
/// Reply has two fields: "ACK/NACK" and "Descriptors". The "Descriptors" field is an array of 16 bit values.
/// The MSB specifies the descriptor set and the LSB specifies the descriptor.
/// @param[out] descriptors_count 
/// @param[out] descriptors 
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_base_get_extended_descriptors(struct mip_interface* device, uint8_t* descriptors_count, uint16_t* descriptors)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_BASE_CMD_DESC_SET, MIP_CMD_DESC_BASE_GET_EXTENDED_DESCRIPTORS, NULL, 0, MIP_REPLY_DESC_BASE_GET_EXTENDED_DESCRIPTORS, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        struct mip_serializer serializer;
        mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
        
        extract_u8(&serializer, descriptors_count);
        for(unsigned int i=0; (i < *descriptors_count) && mip_serializer_ok(&serializer); i++)
            extract_u16(&serializer, &descriptors[i]);
        
        if( !mip_serializer_ok(&serializer) )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Report result of continous built-in test.
/// 
/// This test is non-disruptive but is not as thorough as the commanded BIT.
/// @param[out] result Device-specific bitfield (128 bits). See device user manual. Bits are least-significant-byte first. For example, bit 0 is located at bit 0 of result[0], bit 1 is located at bit 1 of result[0], bit 8 is located at bit 0 of result[1], and bit 127 is located at bit 7 of result[15].
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_base_continuous_bit(struct mip_interface* device, uint8_t* result)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_BASE_CMD_DESC_SET, MIP_CMD_DESC_BASE_CONTINUOUS_BIT, NULL, 0, MIP_REPLY_DESC_BASE_CONTINUOUS_BIT, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        struct mip_serializer serializer;
        mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
        
        for(unsigned int i=0; i < 16; i++)
            extract_u8(&serializer, &result[i]);
        
        if( !mip_serializer_ok(&serializer) )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

void insert_mip_base_comm_speed_command(struct mip_serializer* serializer, const struct mip_base_comm_speed_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    insert_u8(serializer, self->port);
    insert_u32(serializer, self->baud);
}

void extract_mip_base_comm_speed_command(struct mip_serializer* serializer, struct mip_base_comm_speed_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    extract_u8(serializer, &self->port);
    extract_u32(serializer, &self->baud);
}

/// @brief Controls the baud rate of a specific port on the device.
/// 
/// Please see the device user manual for supported baud rates on each port.
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
/// @param port Port ID number, starting with 1. When function is SAVE, LOAD, or DEFAULT, this can be 0 to apply to all ports. See the device user manual for details.
/// @param baud Port baud rate. Must be a supported rate.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_base_write_comm_speed(struct mip_interface* device, uint8_t port, uint32_t baud)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    insert_u8(&serializer, port);
    insert_u32(&serializer, baud);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_BASE_CMD_DESC_SET, MIP_CMD_DESC_BASE_COMM_SPEED, buffer, serializer.offset);
}

/// @brief Controls the baud rate of a specific port on the device.
/// 
/// Please see the device user manual for supported baud rates on each port.
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
/// @param port Port ID number, starting with 1. When function is SAVE, LOAD, or DEFAULT, this can be 0 to apply to all ports. See the device user manual for details.
/// @param[out] port Port ID number, starting with 1. When function is SAVE, LOAD, or DEFAULT, this can be 0 to apply to all ports. See the device user manual for details.
/// @param[out] baud Port baud rate. Must be a supported rate.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_base_read_comm_speed(struct mip_interface* device, uint8_t port, uint32_t* baud)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    insert_u8(&serializer, port);
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_BASE_CMD_DESC_SET, MIP_CMD_DESC_BASE_COMM_SPEED, buffer, serializer.offset, MIP_REPLY_DESC_BASE_COMM_SPEED, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        struct mip_serializer serializer;
        mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
        
        extract_u8(&serializer, &port);
        extract_u32(&serializer, baud);
        
        if( !mip_serializer_ok(&serializer) )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Controls the baud rate of a specific port on the device.
/// 
/// Please see the device user manual for supported baud rates on each port.
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
/// @param port Port ID number, starting with 1. When function is SAVE, LOAD, or DEFAULT, this can be 0 to apply to all ports. See the device user manual for details.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_base_save_comm_speed(struct mip_interface* device, uint8_t port)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    insert_u8(&serializer, port);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_BASE_CMD_DESC_SET, MIP_CMD_DESC_BASE_COMM_SPEED, buffer, serializer.offset);
}

/// @brief Controls the baud rate of a specific port on the device.
/// 
/// Please see the device user manual for supported baud rates on each port.
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
/// @param port Port ID number, starting with 1. When function is SAVE, LOAD, or DEFAULT, this can be 0 to apply to all ports. See the device user manual for details.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_base_load_comm_speed(struct mip_interface* device, uint8_t port)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    insert_u8(&serializer, port);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_BASE_CMD_DESC_SET, MIP_CMD_DESC_BASE_COMM_SPEED, buffer, serializer.offset);
}

/// @brief Controls the baud rate of a specific port on the device.
/// 
/// Please see the device user manual for supported baud rates on each port.
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
/// @param port Port ID number, starting with 1. When function is SAVE, LOAD, or DEFAULT, this can be 0 to apply to all ports. See the device user manual for details.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_base_default_comm_speed(struct mip_interface* device, uint8_t port)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    insert_u8(&serializer, port);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_BASE_CMD_DESC_SET, MIP_CMD_DESC_BASE_COMM_SPEED, buffer, serializer.offset);
}

void insert_mip_base_gps_time_update_command(struct mip_serializer* serializer, const struct mip_base_gps_time_update_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    insert_mip_base_gps_time_update_command_field_id(serializer, self->field_id);
    insert_u32(serializer, self->value);
}

void extract_mip_base_gps_time_update_command(struct mip_serializer* serializer, struct mip_base_gps_time_update_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    extract_mip_base_gps_time_update_command_field_id(serializer, &self->field_id);
    extract_u32(serializer, &self->value);
}

void insert_mip_base_gps_time_update_command_field_id(struct mip_serializer* serializer, const enum mip_base_gps_time_update_command_field_id self)
{
    return insert_u8(serializer, (uint8_t)(self));
}
void extract_mip_base_gps_time_update_command_field_id(struct mip_serializer* serializer, enum mip_base_gps_time_update_command_field_id* self)
{
    uint8_t tmp = 0;
    extract_u8(serializer, &tmp);
    *self = tmp;
}

/// @brief When combined with a PPS input signal applied to the I/O connector, this command enables complete synchronization of data outputs
/// with an external time base, such as GPS system time. Since the hardware PPS synchronization can only detect the fractional number of seconds when pulses arrive,
/// complete synchronization requires that the user provide the whole number of seconds via this command. After achieving PPS synchronization, this command should be sent twice: once to set the time-of-week and once to set the week number. PPS synchronization can be verified by monitoring the time sync status message (0xA0, 0x02) or the valid flags of any shared external timestamp (0x--, D7) data field.
/// @param field_id Determines how to interpret value.
/// @param value Week number or time of week, depending on the field_id.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_base_write_gps_time_update(struct mip_interface* device, enum mip_base_gps_time_update_command_field_id field_id, uint32_t value)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    insert_mip_base_gps_time_update_command_field_id(&serializer, field_id);
    insert_u32(&serializer, value);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_BASE_CMD_DESC_SET, MIP_CMD_DESC_BASE_GPS_TIME_BROADCAST_NEW, buffer, serializer.offset);
}

/// @brief Resets the device.
/// 
/// Device responds with ACK and immediately resets.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_base_soft_reset(struct mip_interface* device)
{
    return mip_interface_run_command(device, MIP_BASE_CMD_DESC_SET, MIP_CMD_DESC_BASE_SOFT_RESET, NULL, 0);
}


#ifdef __cplusplus
} // namespace C
} // namespace mip
} // extern "C"
#endif // __cplusplus

