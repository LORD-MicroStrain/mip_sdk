
#include "mip_field.h"

#include "mip_packet.h"
#include "mip_offsets.h"

#include <assert.h>


////////////////////////////////////////////////////////////////////////////////
///@brief Constructs a %MipField given the parameters.
///
///@param field
///@param descriptorSet
///       The MIP descriptor set of the packet.
///@param fieldDescriptor
///       The MIP field descriptor.
///@param payload
///       A pointer to a buffer containing the field payload, not including the
///       field header. The data must exist while the field is in use. Can be
///       NULL if payloadLength is 0.
///@param payloadLength
///       The length of the payload. Cannot exceed MIP_FIELD_PAYLOAD_LENGTH_MAX.
///
///@returns A %MipField initialized with the specified values.
///
void MipField_init(struct MipField* field, uint8_t descriptorSet, uint8_t fieldDescriptor, const uint8_t* payload, uint8_t payloadLength)
{
    assert( payloadLength <= MIP_FIELD_PAYLOAD_LENGTH_MAX );
    if( payloadLength > MIP_FIELD_PAYLOAD_LENGTH_MAX )
        payloadLength = MIP_FIELD_PAYLOAD_LENGTH_MAX;

    field->payload         = payload;
    field->payloadLength   = payloadLength;
    field->fieldDescriptor = fieldDescriptor;
    field->descriptorSet   = descriptorSet;
    field->remainingLength = 0;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Returns the descriptor set of the packet containing this field.
///
uint8_t MipField_descriptorSet(const struct MipField* field)
{
    return field->descriptorSet;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Returns the field descriptor.
///
uint8_t MipField_fieldDescriptor(const struct MipField* field)
{
    return field->fieldDescriptor;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Returns the length of the payload.
///
uint8_t MipField_payloadLength(const struct MipField* field)
{
    assert(field->payloadLength <= MIP_FIELD_PAYLOAD_LENGTH_MAX);

    return field->payloadLength;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Returns the payload pointer for the field data.
///
const uint8_t* MipField_payload(const struct MipField* field)
{
    return field->payload;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Returns true if the field has a valid field descriptor.
///
bool MipField_isValid(const struct MipField* field)
{
    return field->fieldDescriptor != 0x00;
}



////////////////////////////////////////////////////////////////////////////////
///@brief Constructs a %MipField from a pointer to the heaader.
///
/// Generally you should use MipField_fromPacket() or MipField_create() instead.
///
///@param header
///       A pointer to the header and payload. Usually inside of a MIP packet.
///@param totalLength
///       The total length of either the field or packet payload, starting from
///       headerPtr and including the header bytes. If totalLength is longer
///       than the field (i.e. if it's the packet payload length) then
///       MipField_next() may be used to iterate fields.
///@param descriptorSet
///       The descriptor set for the packet containing this field. May be 0x00
///       if not used by any function handling the field.
///
///@returns a MipField struct with the field data.
///
struct MipField MipField_fromHeaderPtr(const uint8_t* header, uint8_t totalLength, uint8_t descriptorSet)
{
    struct MipField field;

    field.payload         = header + MIP_INDEX_FIELD_PAYLOAD;
    field.descriptorSet   = descriptorSet;

    // Default invalid values.
    field.payloadLength   = 0;
    field.fieldDescriptor = 0x00;  // This makes the field invalid.
    field.remainingLength = 0;

    if( totalLength >= MIP_FIELD_HEADER_LENGTH )
    {
        // Field length is external input so it must be sanitized.
        uint8_t fieldLength = header[MIP_INDEX_FIELD_LEN];

        // Ensure field length does not exceed totalLength.
        if( fieldLength > totalLength )
            fieldLength = totalLength;

        // Check for invalid field length.
        if( fieldLength >= MIP_FIELD_HEADER_LENGTH )
        {
            field.fieldDescriptor = header[MIP_INDEX_FIELD_DESC];
            field.payloadLength   = fieldLength - MIP_FIELD_HEADER_LENGTH;
            field.remainingLength = totalLength - fieldLength;
        }
    }

    return field;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Extracts the first field from a MIP packet.
///
/// Typically this would be used as the first step when iterating through all
/// fields in a received packet. To access each field after the first, call
/// MipField_next(). You should call MipField_isAtEnd() or MipField_isValid()
/// after obtaining each field (including the first) to determine if the field
/// is actually valid.
///
///@param packet
///       The mip packet containing 0 or more fields. Assumed to be valid, and
///       the payload pointer and size must be correct.
///
///@returns A MipField struct with the first field from the packet.
///
struct MipField MipField_fromPacket(const struct MipPacket* packet)
{
    return MipField_fromHeaderPtr( MipPacket_payload(packet), MipPacket_payloadLength(packet), MipPacket_descriptorSet(packet) );
}

////////////////////////////////////////////////////////////////////////////////
///@brief Gets the next field after the specified field.
///
///@param field
///       An existing MIP field in a packet. Can be invalid, in which case the
///       result will also be invalid.
///
///@returns A MipField struct referencing the next field after the input
///         field. Check MipField_isValid() to see if the field exists.
///
struct MipField MipField_nextAfter(const struct MipField* field)
{
    const uint8_t* nextHeader = field->payload + field->payloadLength;

    return MipField_fromHeaderPtr(nextHeader, field->remainingLength, field->descriptorSet);
}

////////////////////////////////////////////////////////////////////////////////
///@brief Updates the %MipField to refer to the next field in a packet.
///
///@param field
///       This %MipField struct will be updated to the next field. Can be an
///       invalid field, in which case the result will be invalid as well.
///
///@returns true if the field exists and is valid.
///
bool MipField_next(struct MipField* field)
{
    *field = MipField_nextAfter(field);

    return MipField_isValid(field);
}

