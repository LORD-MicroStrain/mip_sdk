
#include <mscl/mip/mip_field.h>
#include <mscl/mip/mip_packet.h>
#include <mscl/mip/mip_offsets.h>

#include <stdio.h>
#include <stdlib.h>


uint8_t packetBuffer[MIP_PACKET_LENGTH_MAX];
struct MipPacket packet;

struct MipField fields[MIP_PACKET_PAYLOAD_LENGTH_MAX / MIP_FIELD_LENGTH_MIN];
unsigned int numFields = 0;


bool addRandomField()
{
    const uint8_t length = rand() % 10;
    const uint8_t fieldDesc = (rand() % 255) + 1;

    uint8_t* payload;
    if( !MipPacket_allocField(&packet, fieldDesc, length, &payload) )
        return false;

    for(unsigned int i=0; i<length; i++)
        payload[i] = rand() & 0xFF;

    MipField_init(&fields[numFields++], MipPacket_descriptorSet(&packet), fieldDesc, payload, length);

    return true;
}


int main(int argc, const char* argv[])
{
    srand(0);

    const unsigned int NUM_ITERATIONS = 100;

    unsigned int numErrors = 0;

    for(unsigned int i=0; i<NUM_ITERATIONS; i++)
    {
        // Create a packet with a random number of fields.
        const uint8_t descSet = (rand() % 255) + 1;

        MipPacket_create(&packet, packetBuffer, sizeof(packetBuffer), descSet);

        numFields = 0;
        while( addRandomField() )
        {
            // 20% chance of not adding any more fields.
            if( (rand() % 5) == 0 )
                break;
        }

        MipPacket_finalize(&packet);

        bool error = false;

        // Now iterate the fields and verify they match.
        unsigned int scannedFields = 0;
        for(struct MipField field = MipField_fromPacket(&packet); MipField_isValid(&field); MipField_next(&field))
        {
            const uint8_t test_fieldDesc      = MipField_fieldDescriptor(&field);
            const uint8_t test_descSet        = MipField_descriptorSet(&field);
            const uint8_t test_paylen         = MipField_payloadLength(&field);
            const uint8_t* const test_payload = MipField_payload(&field);

            const uint8_t ref_fieldDesc      = MipField_fieldDescriptor(&fields[scannedFields]);
            const uint8_t ref_descSet        = MipField_descriptorSet(&fields[scannedFields]);
            const uint8_t ref_paylen         = MipField_payloadLength(&fields[scannedFields]);
            const uint8_t* const ref_payload = MipField_payload(&fields[scannedFields]);

            if( test_fieldDesc != ref_fieldDesc )
            {
                error = true;
                fprintf(stderr, "Field descriptor %02X does not match reference %02X.\n", test_fieldDesc, ref_fieldDesc);
            }
            if( test_descSet != ref_descSet )
            {
                error = true;
                fprintf(stderr, "Descriptor set %02X does not match reference %02X.\n", test_descSet, ref_descSet);
            }
            if( test_paylen != ref_paylen )
            {
                error = true;
                fprintf(stderr, "Payload length %d does not match reference %d.\n", test_paylen, ref_paylen);
            }
            if( test_payload != ref_payload )
            {
                error = true;
                fprintf(stderr, "Payload %p does not match reference %p.\n", test_payload, ref_payload);
            }

            scannedFields++;
        }

        if( scannedFields != numFields )
        {
            error = true;
            fprintf(stderr, "Found %d fields but expected %d.\n", scannedFields, numFields);
        }

        if( error )
        {
            numErrors++;

            fprintf(stderr, "Error(s) detected for field list (descriptor/length):");
            for(unsigned int f=0; f<numFields; f++)
                fprintf(stderr, " %02X/%d", MipField_fieldDescriptor(&fields[f]), MipField_payloadLength(&fields[f]));
            fputc('\n', stderr);
        }

        // Bail if too many errors.
        if( numErrors > 10 )
        {
            fprintf(stderr, "***\nToo many errors, aborting.\n");
            break;
        }
    }

    return numErrors;
}
