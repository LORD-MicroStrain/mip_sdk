
#include "commands_base.hpp"

#include "../../utils/serialization.h"
#include "../mip_interface.h"

#include <assert.h>


namespace mscl {
class MipSerializer;

namespace C {
struct mip_interface;
} // namespace C

namespace commands_base {

using ::mscl::insert;
using ::mscl::extract;
using namespace ::mscl::C;

////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////

void insert(MipSerializer& serializer, const BaseDeviceInfo& self)
{
    insert(serializer, self.firmware_version);
    for(unsigned int i=0; i < 16; i++)
        insert(serializer, self.model_name[i]);
    for(unsigned int i=0; i < 16; i++)
        insert(serializer, self.model_number[i]);
    for(unsigned int i=0; i < 16; i++)
        insert(serializer, self.serial_number[i]);
    for(unsigned int i=0; i < 16; i++)
        insert(serializer, self.lot_number[i]);
    for(unsigned int i=0; i < 16; i++)
        insert(serializer, self.device_options[i]);
}

void extract(MipSerializer& serializer, BaseDeviceInfo& self)
{
    extract(serializer, self.firmware_version);
    for(unsigned int i=0; i < 16; i++)
        extract(serializer, self.model_name[i]);
    for(unsigned int i=0; i < 16; i++)
        extract(serializer, self.model_number[i]);
    for(unsigned int i=0; i < 16; i++)
        extract(serializer, self.serial_number[i]);
    for(unsigned int i=0; i < 16; i++)
        extract(serializer, self.lot_number[i]);
    for(unsigned int i=0; i < 16; i++)
        extract(serializer, self.device_options[i]);
}


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

void insert(MipSerializer& serializer, const Ping& self)
{
    (void)serializer;
    (void)self;
}

void extract(MipSerializer& serializer, Ping& self)
{
    (void)serializer;
    (void)self;
}

/// @brief Test Communications with a device.
/// 
/// The Device will respond with an ACK, if present and operating correctly.
/// 
/// If the device is not in a normal operating mode, it may NACK.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult ping(C::mip_interface& device)
{
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_PING, NULL, 0);
}

void insert(MipSerializer& serializer, const SetIdle& self)
{
    (void)serializer;
    (void)self;
}

void extract(MipSerializer& serializer, SetIdle& self)
{
    (void)serializer;
    (void)self;
}

/// @brief Turn off all device data streams.
/// 
/// The Device will respond with an ACK, if present and operating correctly.
/// This command will suspend streaming (if enabled) or wake the device from sleep (if sleeping) to allow it to respond to status and setup commands.
/// You may restore the device mode by issuing the Resume command.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult setIdle(C::mip_interface& device)
{
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SET_TO_IDLE, NULL, 0);
}

void insert(MipSerializer& serializer, const GetDeviceInfo& self)
{
    (void)serializer;
    (void)self;
}

void extract(MipSerializer& serializer, GetDeviceInfo& self)
{
    (void)serializer;
    (void)self;
}

/// @brief Get the device ID strings and firmware version number.
/// 
/// @param[out] device_info 
/// 
/// @returns MipCmdResult
/// 
MipCmdResult getDeviceInfo(C::mip_interface& device, BaseDeviceInfo& device_info)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    mip_cmd_result result_local = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GET_DEVICE_INFO, NULL, 0, REPLY_DEVICE_INFO, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        MipSerializer serializer(buffer, sizeof(buffer));
        
        extract(serializer, device_info);
        
        if( !!!serializer )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

void insert(MipSerializer& serializer, const GetDeviceDescriptors& self)
{
    (void)serializer;
    (void)self;
}

void extract(MipSerializer& serializer, GetDeviceDescriptors& self)
{
    (void)serializer;
    (void)self;
}

/// @brief Get the command and data descriptors supported by the device.
/// 
/// Reply has two fields: "ACK/NACK" and "Descriptors". The "Descriptors" field is an array of 16 bit values.
/// The MSB specifies the descriptor set and the LSB specifies the descriptor.
/// @param[out] descriptors_count 
/// @param[out] descriptors 
/// 
/// @returns MipCmdResult
/// 
MipCmdResult getDeviceDescriptors(C::mip_interface& device, uint8_t& descriptors_count, uint16_t* descriptors)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    mip_cmd_result result_local = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GET_DEVICE_DESCRIPTORS, NULL, 0, REPLY_DEVICE_DESCRIPTORS, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        MipSerializer serializer(buffer, sizeof(buffer));
        
        extract(serializer, descriptors_count);
        for(unsigned int i=0; (i < descriptors_count) && !!serializer; i++)
            extract(serializer, descriptors[i]);
        
        if( !!!serializer )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

void insert(MipSerializer& serializer, const BuiltInTest& self)
{
    (void)serializer;
    (void)self;
}

void extract(MipSerializer& serializer, BuiltInTest& self)
{
    (void)serializer;
    (void)self;
}

/// @brief Run the device Built-In Test (BIT).
/// 
/// The Built-In Test command always returns a 32 bit value.
/// A value of 0 means that all tests passed.
/// A non-zero value indicates that not all tests passed.
/// Reference the device user manual to decode the result.
/// @param[out] result 
/// 
/// @returns MipCmdResult
/// 
MipCmdResult builtInTest(C::mip_interface& device, uint32_t& result)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    mip_cmd_result result_local = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_BUILT_IN_TEST, NULL, 0, REPLY_BUILT_IN_TEST, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        MipSerializer serializer(buffer, sizeof(buffer));
        
        extract(serializer, result);
        
        if( !!!serializer )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

void insert(MipSerializer& serializer, const Resume& self)
{
    (void)serializer;
    (void)self;
}

void extract(MipSerializer& serializer, Resume& self)
{
    (void)serializer;
    (void)self;
}

/// @brief Take the device out of idle mode.
/// 
/// The device responds with ACK upon success.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult resume(C::mip_interface& device)
{
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_RESUME, NULL, 0);
}

void insert(MipSerializer& serializer, const GetExtendedDescriptors& self)
{
    (void)serializer;
    (void)self;
}

void extract(MipSerializer& serializer, GetExtendedDescriptors& self)
{
    (void)serializer;
    (void)self;
}

/// @brief Get the command and data descriptors supported by the device.
/// 
/// Reply has two fields: "ACK/NACK" and "Descriptors". The "Descriptors" field is an array of 16 bit values.
/// The MSB specifies the descriptor set and the LSB specifies the descriptor.
/// @param[out] descriptors_count 
/// @param[out] descriptors 
/// 
/// @returns MipCmdResult
/// 
MipCmdResult getExtendedDescriptors(C::mip_interface& device, uint8_t& descriptors_count, uint16_t* descriptors)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    mip_cmd_result result_local = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GET_EXTENDED_DESCRIPTORS, NULL, 0, REPLY_GET_EXTENDED_DESCRIPTORS, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        MipSerializer serializer(buffer, sizeof(buffer));
        
        extract(serializer, descriptors_count);
        for(unsigned int i=0; (i < descriptors_count) && !!serializer; i++)
            extract(serializer, descriptors[i]);
        
        if( !!!serializer )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

void insert(MipSerializer& serializer, const ContinuousBit& self)
{
    (void)serializer;
    (void)self;
}

void extract(MipSerializer& serializer, ContinuousBit& self)
{
    (void)serializer;
    (void)self;
}

/// @brief Report result of continous built-in test.
/// 
/// This test is non-disruptive but is not as thorough as the commanded BIT.
/// @param[out] result Device-specific bitfield (128 bits). See device user manual. Bits are least-significant-byte first. For example, bit 0 is located at bit 0 of result[0], bit 1 is located at bit 1 of result[0], bit 8 is located at bit 0 of result[1], and bit 127 is located at bit 7 of result[15].
/// 
/// @returns MipCmdResult
/// 
MipCmdResult continuousBit(C::mip_interface& device, uint8_t* result)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    mip_cmd_result result_local = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_CONTINUOUS_BIT, NULL, 0, REPLY_CONTINUOUS_BIT, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        MipSerializer serializer(buffer, sizeof(buffer));
        
        for(unsigned int i=0; i < 16; i++)
            extract(serializer, result[i]);
        
        if( !!!serializer )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

void insert(MipSerializer& serializer, const CommSpeed& self)
{
    insert(serializer, self.function);
    insert(serializer, self.port);
    insert(serializer, self.baud);
}

void extract(MipSerializer& serializer, CommSpeed& self)
{
    extract(serializer, self.function);
    extract(serializer, self.port);
    extract(serializer, self.baud);
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
/// @returns MipCmdResult
/// 
MipCmdResult writeCommSpeed(C::mip_interface& device, uint8_t port, uint32_t baud)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::WRITE);
    insert(serializer, port);
    insert(serializer, baud);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_COMM_SPEED, buffer, serializer.offset);
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
/// @returns MipCmdResult
/// 
MipCmdResult readCommSpeed(C::mip_interface& device, uint8_t port, uint32_t& baud)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::READ);
    insert(serializer, port);
    assert(!!serializer);
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_COMM_SPEED, buffer, serializer.offset, REPLY_COMM_SPEED, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        MipSerializer serializer(buffer, sizeof(buffer));
        
        extract(serializer, port);
        extract(serializer, baud);
        
        if( !!!serializer )
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
/// @returns MipCmdResult
/// 
MipCmdResult saveCommSpeed(C::mip_interface& device, uint8_t port)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::SAVE);
    insert(serializer, port);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_COMM_SPEED, buffer, serializer.offset);
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
/// @returns MipCmdResult
/// 
MipCmdResult loadCommSpeed(C::mip_interface& device, uint8_t port)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::LOAD);
    insert(serializer, port);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_COMM_SPEED, buffer, serializer.offset);
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
/// @returns MipCmdResult
/// 
MipCmdResult defaultCommSpeed(C::mip_interface& device, uint8_t port)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::RESET);
    insert(serializer, port);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_COMM_SPEED, buffer, serializer.offset);
}

void insert(MipSerializer& serializer, const GpsTimeUpdate& self)
{
    insert(serializer, self.function);
    insert(serializer, self.field_id);
    insert(serializer, self.value);
}

void extract(MipSerializer& serializer, GpsTimeUpdate& self)
{
    extract(serializer, self.function);
    extract(serializer, self.field_id);
    extract(serializer, self.value);
}

/// @brief When combined with a PPS input signal applied to the I/O connector, this command enables complete synchronization of data outputs
/// with an external time base, such as GPS system time. Since the hardware PPS synchronization can only detect the fractional number of seconds when pulses arrive,
/// complete synchronization requires that the user provide the whole number of seconds via this command. After achieving PPS synchronization, this command should be sent twice: once to set the time-of-week and once to set the week number. PPS synchronization can be verified by monitoring the time sync status message (0xA0, 0x02) or the valid flags of any shared external timestamp (0x--, D7) data field.
/// @param field_id Determines how to interpret value.
/// @param value Week number or time of week, depending on the field_id.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult writeGpsTimeUpdate(C::mip_interface& device, GpsTimeUpdate::FieldId field_id, uint32_t value)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::WRITE);
    insert(serializer, field_id);
    insert(serializer, value);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GPS_TIME_BROADCAST_NEW, buffer, serializer.offset);
}

void insert(MipSerializer& serializer, const SoftReset& self)
{
    (void)serializer;
    (void)self;
}

void extract(MipSerializer& serializer, SoftReset& self)
{
    (void)serializer;
    (void)self;
}

/// @brief Resets the device.
/// 
/// Device responds with ACK and immediately resets.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult softReset(C::mip_interface& device)
{
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SOFT_RESET, NULL, 0);
}


} // namespace commands_base
} // namespace mscl

