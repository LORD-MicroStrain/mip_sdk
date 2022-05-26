#pragma once

#include <stdint.h>
#include <stdbool.h>

struct MipPacket;


////////////////////////////////////////////////////////////////////////////////
///@defgroup MipField  MipField - Functions for processing received MIP fields.
///
///~~~
/// ---------------+------------+------------+-----/ /-----+------------+
///  ...   Header  |   Field    |   Field    |     ...     |  Checksum  |
/// ---------------+------------+------------+-----/ /-----+------------+
///              _/              \_
///            _/                  \_
///           /                      \.
///          +------+------+---/ /----+
///          | Len  | desc | payload  |        The MIP Field
///          +------+------+---/ /----+
///~~~
///
///@{
///

////////////////////////////////////////////////////////////////////////////////
///@brief A structure representing a MIP field.
///
/// Use to access fields from a received MIP packet.
///
/// This structure references the original packet data and does not contain a
/// copy of the field payload. Therefore, the data buffer must exist as long as
/// there are %MipField instances which reference it (even if the field payload
/// itself is not used directly).
///
///@note This should be considered an "opaque" structure; its members should be
/// considered an internal implementation detail. Avoid accessing them directly
/// as they are subject to change in future versions of this software.
///
struct MipField
{
    const uint8_t* payload;   ///<@internal The field payload, excluding the header.
    uint8_t payloadLength;    ///<@internal The length of the payload, excluding the header.
    uint8_t fieldDescriptor;  ///<@internal MIP field descriptor. Field not valid if set to 0x00.
    uint8_t descriptorSet;    ///<@internal MIP descriptor set (from the packet)
    uint8_t remainingLength;  ///<@internal Remaining space after this field.
};


void MipField_init(struct MipField* field, uint8_t descriptorSet, uint8_t fieldDescriptor, const uint8_t* payload, uint8_t payloadLength);

////////////////////////////////////////////////////////////////////////////////
///@defgroup FieldAccess  Field Accessors - Functions for inspecting a MIP field.
///@{

uint8_t MipField_descriptorSet(const struct MipField* field);
uint8_t MipField_fieldDescriptor(const struct MipField* field);
uint8_t MipField_payloadLength(const struct MipField* field);
const uint8_t* MipField_payload(const struct MipField* field);

bool MipField_isValid(const struct MipField* field);

///@}
////////////////////////////////////////////////////////////////////////////////
///@defgroup FieldIteration  Field Iteration - Functions for iterating over fields in a MIP packet.
///
/// Use these functions to iterate over the fields in a MIP packet.
///
/// Example:
///~~~{.c}
/// void handlScaledAccelData(const uint8_t* ptr, uint8_t length);
///
/// // Assume this is called from the MIP parser for packets with the Sensor Data descriptor set.
/// void handleSensorDataPacket(const struct MipPacket* packet)
/// {
///
///     // Iterate over fields, starting with the first in the packet.
///     // Continue as long as a valid field is found.
///     for(struct MipField field = MipField_fromPacket(packet); MipField_isValid(&field); MipField_next(&field))
///     {
///
///         // Check the field descriptor for what kind of data it holds.
///         switch( MipField_fieldDesriptor(&field) )
///         {
///         case MIP_DATA_DESC_SENSOR_SCALED_ACCEL:
///
///             // This field holds scaled acceleration data, so call the function to deal with it.
///             handleScaledAccelData( MipField_payload(&field), MipField_payloadLength(&field) );
///
///             break;
///
///         // ...
///
///         default:
///             fprintf(stderr, "Unexpected field with descriptor %02X,%02X and payload length %d!",
///                 MipField_descriptorSet(&field),
///                 MipField_fieldDescriptor(&field),
///                 MipField_payloadLength(&field)
///             );
///             break;
///         }
///     }
/// }
///~~~
///
///@{

struct MipField MipField_fromHeaderPtr(const uint8_t* header, uint8_t totalLength, uint8_t descriptorSet);

struct MipField MipField_fromPacket(const struct MipPacket* packet);
struct MipField MipField_nextAfter(const struct MipField* field);
bool MipField_next(struct MipField* field);

// bool MipField_isAtEnd(const struct MipField* field);

///@}
///@}
////////////////////////////////////////////////////////////////////////////////
///
