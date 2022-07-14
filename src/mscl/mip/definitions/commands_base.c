
#include "commands_base.h"

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

size_t insert_MipBaseDeviceInfo(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipBaseDeviceInfo* self)
{
    offset = insert_u16(buffer, bufferSize, offset, self->firmware_version);
    for(unsigned int i=0; i < 16; i++)
        offset = insert_char(buffer, bufferSize, offset, self->model_name[i]);
    for(unsigned int i=0; i < 16; i++)
        offset = insert_char(buffer, bufferSize, offset, self->model_number[i]);
    for(unsigned int i=0; i < 16; i++)
        offset = insert_char(buffer, bufferSize, offset, self->serial_number[i]);
    for(unsigned int i=0; i < 16; i++)
        offset = insert_char(buffer, bufferSize, offset, self->lot_number[i]);
    for(unsigned int i=0; i < 16; i++)
        offset = insert_char(buffer, bufferSize, offset, self->device_options[i]);
    
    return offset;
}

size_t extract_MipBaseDeviceInfo(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipBaseDeviceInfo* self)
{
    offset = extract_u16(buffer, bufferSize, offset, &self->firmware_version);
    for(unsigned int i=0; i < 16; i++)
        offset = extract_char(buffer, bufferSize, offset, &self->model_name[i]);
    for(unsigned int i=0; i < 16; i++)
        offset = extract_char(buffer, bufferSize, offset, &self->model_number[i]);
    for(unsigned int i=0; i < 16; i++)
        offset = extract_char(buffer, bufferSize, offset, &self->serial_number[i]);
    for(unsigned int i=0; i < 16; i++)
        offset = extract_char(buffer, bufferSize, offset, &self->lot_number[i]);
    for(unsigned int i=0; i < 16; i++)
        offset = extract_char(buffer, bufferSize, offset, &self->device_options[i]);
    
    return offset;
}


size_t insert_MipTimeFormat(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipTimeFormat self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}
size_t extract_MipTimeFormat(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipTimeFormat* self)
{
    uint8_t tmp;
    offset = extract_u8(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}

size_t insert_MipCommandedTestBitsGq7(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipCommandedTestBitsGq7 self)
{
    return insert_u32(buffer, bufferSize, offset, self);
}
size_t extract_MipCommandedTestBitsGq7(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipCommandedTestBitsGq7* self)
{
    uint32_t tmp;
    offset = extract_u32(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}



////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_Base_Ping(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Base_Ping* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}

size_t extract_MipCmd_Base_Ping(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Base_Ping* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}


/// @brief Test Communications with a device.
/// 
/// The Device will respond with an ACK, if present and operating correctly.
/// 
/// If the device is not in a normal operating mode, it may NACK.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult ping(struct MipInterfaceState* device)
{
    return MipInterface_runCommand(device, MIP_BASE_COMMAND_DESC_SET, MIP_CMD_DESC_BASE_PING, NULL, 0);
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_Base_SetIdle(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Base_SetIdle* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}

size_t extract_MipCmd_Base_SetIdle(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Base_SetIdle* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}


/// @brief Turn off all device data streams.
/// 
/// The Device will respond with an ACK, if present and operating correctly.
/// This command will suspend streaming (if enabled) or wake the device from sleep (if sleeping) to allow it to respond to status and setup commands.
/// You may restore the device mode by issuing the Resume command.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult set_to_idle(struct MipInterfaceState* device)
{
    return MipInterface_runCommand(device, MIP_BASE_COMMAND_DESC_SET, MIP_CMD_DESC_BASE_SET_TO_IDLE, NULL, 0);
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_Base_GetDeviceInfo(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Base_GetDeviceInfo* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}

size_t extract_MipCmd_Base_GetDeviceInfo(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Base_GetDeviceInfo* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}


size_t insert_MipCmd_Base_GetDeviceInfo_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Base_GetDeviceInfo_Response* self)
{
    offset = insert_MipBaseDeviceInfo(buffer, bufferSize, offset, &self->device_info);
    
    return offset;
}

size_t extract_MipCmd_Base_GetDeviceInfo_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Base_GetDeviceInfo_Response* self)
{
    offset = extract_MipBaseDeviceInfo(buffer, bufferSize, offset, &self->device_info);
    
    return offset;
}


/// @brief Get the device ID strings and firmware version number.
/// 
/// @param[out] device_info 
/// 
/// @returns MipCmdResult
/// 
MipCmdResult get_device_information(struct MipInterfaceState* device, struct MipBaseDeviceInfo* device_info)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    
    uint8_t responseLength = sizeof(buffer);
    MipCmdResult result_local = MipInterface_runCommandWithResponse(device, MIP_BASE_COMMAND_DESC_SET, MIP_CMD_DESC_BASE_GET_DEVICE_INFO, NULL, 0, MIP_REPLY_DESC_BASE_DEVICE_INFO, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        size_t responseUsed = 0;
        responseUsed = extract_MipBaseDeviceInfo(buffer, sizeof(buffer), responseUsed, device_info);
        
        if( responseUsed != responseLength )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_Base_GetDeviceDescriptors(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Base_GetDeviceDescriptors* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}

size_t extract_MipCmd_Base_GetDeviceDescriptors(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Base_GetDeviceDescriptors* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}


size_t insert_MipCmd_Base_GetDeviceDescriptors_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Base_GetDeviceDescriptors_Response* self)
{
    assert(self->descriptors_count <= 0);
    for(unsigned int i=0; i < self->descriptors_count; i++)
        offset = insert_u16(buffer, bufferSize, offset, self->descriptors[i]);
    
    return offset;
}

size_t extract_MipCmd_Base_GetDeviceDescriptors_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Base_GetDeviceDescriptors_Response* self)
{
    for(unsigned int max_descriptors=self->descriptors_count; (self->descriptors_count < max_descriptors) && (offset < bufferSize); (self->descriptors_count)++)
        offset = extract_u16(buffer, bufferSize, offset, &self->descriptors[self->descriptors_count]);
    
    return offset;
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
MipCmdResult get_device_descriptors(struct MipInterfaceState* device, uint8_t* descriptors_count, uint16_t* descriptors)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    
    uint8_t responseLength = sizeof(buffer);
    MipCmdResult result_local = MipInterface_runCommandWithResponse(device, MIP_BASE_COMMAND_DESC_SET, MIP_CMD_DESC_BASE_GET_DEVICE_DESCRIPTORS, NULL, 0, MIP_REPLY_DESC_BASE_DEVICE_DESCRIPTORS, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        size_t responseUsed = 0;
        responseUsed = extract_u8(buffer, sizeof(buffer), responseUsed, descriptors_count);
        for(unsigned int max_descriptors=*descriptors_count; (*descriptors_count < max_descriptors) && (responseUsed < sizeof(buffer)); (*descriptors_count)++)
            responseUsed = extract_u16(buffer, sizeof(buffer), responseUsed, &descriptors[*descriptors_count]);
        
        if( responseUsed != responseLength )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_Base_BuiltInTest(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Base_BuiltInTest* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}

size_t extract_MipCmd_Base_BuiltInTest(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Base_BuiltInTest* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}


size_t insert_MipCmd_Base_BuiltInTest_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Base_BuiltInTest_Response* self)
{
    offset = insert_u32(buffer, bufferSize, offset, self->result);
    
    return offset;
}

size_t extract_MipCmd_Base_BuiltInTest_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Base_BuiltInTest_Response* self)
{
    offset = extract_u32(buffer, bufferSize, offset, &self->result);
    
    return offset;
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
MipCmdResult built_in_test(struct MipInterfaceState* device, uint32_t* result)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    
    uint8_t responseLength = sizeof(buffer);
    MipCmdResult result_local = MipInterface_runCommandWithResponse(device, MIP_BASE_COMMAND_DESC_SET, MIP_CMD_DESC_BASE_BUILT_IN_TEST, NULL, 0, MIP_REPLY_DESC_BASE_BUILT_IN_TEST, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        size_t responseUsed = 0;
        responseUsed = extract_u32(buffer, sizeof(buffer), responseUsed, result);
        
        if( responseUsed != responseLength )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_Base_Resume(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Base_Resume* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}

size_t extract_MipCmd_Base_Resume(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Base_Resume* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}


/// @brief Take the device out of idle mode.
/// 
/// The device responds with ACK upon success.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult resume(struct MipInterfaceState* device)
{
    return MipInterface_runCommand(device, MIP_BASE_COMMAND_DESC_SET, MIP_CMD_DESC_BASE_RESUME, NULL, 0);
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_Base_GetExtendedDescriptors(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Base_GetExtendedDescriptors* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}

size_t extract_MipCmd_Base_GetExtendedDescriptors(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Base_GetExtendedDescriptors* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}


size_t insert_MipCmd_Base_GetExtendedDescriptors_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Base_GetExtendedDescriptors_Response* self)
{
    assert(self->descriptors_count <= 0);
    for(unsigned int i=0; i < self->descriptors_count; i++)
        offset = insert_u16(buffer, bufferSize, offset, self->descriptors[i]);
    
    return offset;
}

size_t extract_MipCmd_Base_GetExtendedDescriptors_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Base_GetExtendedDescriptors_Response* self)
{
    for(unsigned int max_descriptors=self->descriptors_count; (self->descriptors_count < max_descriptors) && (offset < bufferSize); (self->descriptors_count)++)
        offset = extract_u16(buffer, bufferSize, offset, &self->descriptors[self->descriptors_count]);
    
    return offset;
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
MipCmdResult get_device_descriptors_extended(struct MipInterfaceState* device, uint8_t* descriptors_count, uint16_t* descriptors)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    
    uint8_t responseLength = sizeof(buffer);
    MipCmdResult result_local = MipInterface_runCommandWithResponse(device, MIP_BASE_COMMAND_DESC_SET, MIP_CMD_DESC_BASE_GET_EXTENDED_DESCRIPTORS, NULL, 0, MIP_REPLY_DESC_BASE_GET_EXTENDED_DESCRIPTORS, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        size_t responseUsed = 0;
        responseUsed = extract_u8(buffer, sizeof(buffer), responseUsed, descriptors_count);
        for(unsigned int max_descriptors=*descriptors_count; (*descriptors_count < max_descriptors) && (responseUsed < sizeof(buffer)); (*descriptors_count)++)
            responseUsed = extract_u16(buffer, sizeof(buffer), responseUsed, &descriptors[*descriptors_count]);
        
        if( responseUsed != responseLength )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_Base_ContinuousBit(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Base_ContinuousBit* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}

size_t extract_MipCmd_Base_ContinuousBit(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Base_ContinuousBit* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}


size_t insert_MipCmd_Base_ContinuousBit_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Base_ContinuousBit_Response* self)
{
    for(unsigned int i=0; i < 16; i++)
        offset = insert_u8(buffer, bufferSize, offset, self->result[i]);
    
    return offset;
}

size_t extract_MipCmd_Base_ContinuousBit_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Base_ContinuousBit_Response* self)
{
    for(unsigned int i=0; i < 16; i++)
        offset = extract_u8(buffer, bufferSize, offset, &self->result[i]);
    
    return offset;
}


/// @brief Report result of continous built-in test.
/// 
/// This test is non-disruptive but is not as thorough as the commanded BIT.
/// @param[out] result Device-specific bitfield (128 bits). See device user manual. Bits are least-significant-byte first. For example, bit 0 is located at bit 0 of result[0], bit 1 is located at bit 1 of result[0], bit 8 is located at bit 0 of result[1], and bit 127 is located at bit 7 of result[15].
/// 
/// @returns MipCmdResult
/// 
MipCmdResult continuous_built_in_test(struct MipInterfaceState* device, uint8_t* result)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    
    uint8_t responseLength = sizeof(buffer);
    MipCmdResult result_local = MipInterface_runCommandWithResponse(device, MIP_BASE_COMMAND_DESC_SET, MIP_CMD_DESC_BASE_CONTINUOUS_BIT, NULL, 0, MIP_REPLY_DESC_BASE_CONTINUOUS_BIT, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        size_t responseUsed = 0;
        for(unsigned int i=0; i < 16; i++)
            responseUsed = extract_u8(buffer, sizeof(buffer), responseUsed, &result[i]);
        
        if( responseUsed != responseLength )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_Base_CommSpeed(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Base_CommSpeed* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->port);
    offset = insert_u32(buffer, bufferSize, offset, self->baud);
    
    return offset;
}

size_t extract_MipCmd_Base_CommSpeed(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Base_CommSpeed* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->port);
    offset = extract_u32(buffer, bufferSize, offset, &self->baud);
    
    return offset;
}


size_t insert_MipCmd_Base_CommSpeed_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Base_CommSpeed_Response* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->port);
    offset = insert_u32(buffer, bufferSize, offset, self->baud);
    
    return offset;
}

size_t extract_MipCmd_Base_CommSpeed_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Base_CommSpeed_Response* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->port);
    offset = extract_u32(buffer, bufferSize, offset, &self->baud);
    
    return offset;
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
MipCmdResult write_comm_port_speed(struct MipInterfaceState* device, uint8_t port, uint32_t baud)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 1);;
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, port);
    cmdUsed = insert_u32(buffer, sizeof(buffer), cmdUsed, baud);
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_BASE_COMMAND_DESC_SET, MIP_CMD_DESC_BASE_COMM_SPEED, buffer, cmdUsed);
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
MipCmdResult read_comm_port_speed(struct MipInterfaceState* device, uint8_t port, uint32_t* baud)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 2);;
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, port);
    assert(cmdUsed <= sizeof(buffer));
    
    uint8_t responseLength;
    MipCmdResult result_local = MipInterface_runCommandWithResponse(device, MIP_BASE_COMMAND_DESC_SET, MIP_CMD_DESC_BASE_COMM_SPEED, buffer, cmdUsed, MIP_REPLY_DESC_BASE_COMM_SPEED, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        size_t responseUsed = 0;
        responseUsed = extract_u8(buffer, sizeof(buffer), responseUsed, &port);
        responseUsed = extract_u32(buffer, sizeof(buffer), responseUsed, baud);
        
        if( responseUsed != responseLength )
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
MipCmdResult save_comm_port_speed(struct MipInterfaceState* device, uint8_t port)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 3);;
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, port);
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_BASE_COMMAND_DESC_SET, MIP_CMD_DESC_BASE_COMM_SPEED, buffer, cmdUsed);
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
MipCmdResult load_comm_port_speed(struct MipInterfaceState* device, uint8_t port)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 4);;
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, port);
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_BASE_COMMAND_DESC_SET, MIP_CMD_DESC_BASE_COMM_SPEED, buffer, cmdUsed);
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
MipCmdResult default_comm_port_speed(struct MipInterfaceState* device, uint8_t port)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 5);;
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, port);
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_BASE_COMMAND_DESC_SET, MIP_CMD_DESC_BASE_COMM_SPEED, buffer, cmdUsed);
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_Base_GpsTimeUpdate_Fieldid(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipCmd_Base_GpsTimeUpdate_Fieldid self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}
size_t extract_MipCmd_Base_GpsTimeUpdate_Fieldid(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipCmd_Base_GpsTimeUpdate_Fieldid* self)
{
    uint8_t tmp;
    offset = extract_u8(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}

size_t insert_MipCmd_Base_GpsTimeUpdate(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Base_GpsTimeUpdate* self)
{
    offset = insert_MipCmd_Base_GpsTimeUpdate_Fieldid(buffer, bufferSize, offset, self->field_id);
    offset = insert_u32(buffer, bufferSize, offset, self->value);
    
    return offset;
}

size_t extract_MipCmd_Base_GpsTimeUpdate(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Base_GpsTimeUpdate* self)
{
    offset = extract_MipCmd_Base_GpsTimeUpdate_Fieldid(buffer, bufferSize, offset, &self->field_id);
    offset = extract_u32(buffer, bufferSize, offset, &self->value);
    
    return offset;
}


/// @brief When combined with a PPS input signal applied to the I/O connector, this command enables complete synchronization of data outputs
/// with an external time base, such as GPS system time. Since the hardware PPS synchronization can only detect the fractional number of seconds when pulses arrive,
/// complete synchronization requires that the user provide the whole number of seconds via this command. After achieving PPS synchronization, this command should be sent twice: once to set the time-of-week and once to set the week number. PPS synchronization can be verified by monitoring the time sync status message (0xA0, 0x02) or the valid flags of any shared external timestamp (0x--, D7) data field.
/// @param field_id Determines how to interpret value.
/// @param value Week number or time of week, depending on the field_id.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult write_time_broadcast_command(struct MipInterfaceState* device, enum MipCmd_Base_GpsTimeUpdate_Fieldid field_id, uint32_t value)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 1);;
    cmdUsed = insert_MipCmd_Base_GpsTimeUpdate_Fieldid(buffer, sizeof(buffer), cmdUsed, field_id);
    cmdUsed = insert_u32(buffer, sizeof(buffer), cmdUsed, value);
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_BASE_COMMAND_DESC_SET, MIP_CMD_DESC_BASE_GPS_TIME_BROADCAST_NEW, buffer, cmdUsed);
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_Base_SoftReset(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Base_SoftReset* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}

size_t extract_MipCmd_Base_SoftReset(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Base_SoftReset* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}


/// @brief Resets the device.
/// 
/// Device responds with ACK and immediately resets.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult reset_device(struct MipInterfaceState* device)
{
    return MipInterface_runCommand(device, MIP_BASE_COMMAND_DESC_SET, MIP_CMD_DESC_BASE_SOFT_RESET, NULL, 0);
}


#ifdef __cplusplus
} // extern "C"
} // namespace mscl
#endif // __cplusplus
