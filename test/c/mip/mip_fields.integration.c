// TODO: This test has serious issues. Setting the number of iterations to be higher results
//       in: "EXCEPTION_ACCESS_VIOLATION occurred at X". Fix this so we can increase the number
//       of iterations and remove test brittleness. Looks like some possible dangling references
//       as well.

// TODO: Replace these tests completely with new integration test structure once implemented.

#include <stdio.h>
#include <stdlib.h>

#include <mip_cmocka.h>
#include <mip/mip_field.h>
#include <mip/mip_offsets.h>
#include <mip/mip_packet.h>


uint8_t packet_buffer[MIP_PACKET_LENGTH_MAX];
mip_packet_view packet;

mip_field_view fields[MIP_PACKET_PAYLOAD_LENGTH_MAX / MIP_FIELD_LENGTH_MIN];
size_t num_fields = 0;


bool add_random_field()
{
    const uint8_t length = rand() % 10;
    const uint8_t field_desc = (rand() % 255) + 1;

    uint8_t* payload;
    if( !mip_packet_create_field(&packet, field_desc, length, &payload) )
        return false;

    for (unsigned int i = 0; i < length; ++i)
        payload[i] = rand() & 0xFF;

    mip_field_init(&fields[num_fields++], mip_packet_descriptor_set(&packet), field_desc, payload, length);

    return true;
}

MICROSTRAIN_TEST_CASE(Rename_me)
{
    srand(0);

    const size_t NUM_ITERATIONS = 10000;

    unsigned int num_errors = 0;

    for (size_t i = 0; i < NUM_ITERATIONS; ++i)
    {
        // Create a packet with a random number of fields.
        const uint8_t desc_set = (rand() % 255) + 1;

        mip_packet_create(&packet, packet_buffer, sizeof(packet_buffer), desc_set);

        num_fields = 0;
        while( add_random_field() )
        {
            // 20% chance of not adding any more fields.
            if( (rand() % 5) == 0 )
                break;
        }

        mip_packet_finalize(&packet);

        // Now iterate the fields and verify they match.
        unsigned int scanned_fields = 0;
        for (mip_field_view field = mip_field_first_from_packet(&packet); mip_field_is_valid(&field); mip_field_next(&field))
        {
            const uint8_t test_field_desc     = mip_field_field_descriptor(&field);
            const uint8_t test_desc_set       = mip_field_descriptor_set(&field);
            const uint8_t test_payload_length = mip_field_payload_length(&field);
            const uint8_t* const test_payload = mip_field_payload(&field);

            const uint8_t ref_field_desc     = mip_field_field_descriptor(&fields[scanned_fields]);
            const uint8_t ref_desc_set       = mip_field_descriptor_set(&fields[scanned_fields]);
            const uint8_t ref_payload_length = mip_field_payload_length(&fields[scanned_fields]);
            const uint8_t* const ref_payload = mip_field_payload(&fields[scanned_fields]);

            MICROSTRAIN_TEST_ASSERT_MESSAGE(test_field_desc == ref_field_desc,
                "Field descriptor %02X does not match reference %02X.", test_field_desc, ref_field_desc);
            MICROSTRAIN_TEST_ASSERT_MESSAGE(test_desc_set == ref_desc_set,
                "Descriptor set %02X does not match reference %02X.", test_desc_set, ref_desc_set);
            MICROSTRAIN_TEST_ASSERT_MESSAGE(test_payload_length == ref_payload_length,
                "Payload length %d does not match reference %d.", test_payload_length, ref_payload_length);
            MICROSTRAIN_TEST_ASSERT_MESSAGE(test_payload == ref_payload,
                "Payload %p does not match reference %p.", test_payload, ref_payload);

            scanned_fields++;
        }

        MICROSTRAIN_TEST_ASSERT_MESSAGE(scanned_fields == num_fields,
            "Found %d fields but expected %d.", scanned_fields, num_fields);
    }
}

int main()
{
    MICROSTRAIN_TEST_INIT;

    MICROSTRAIN_TEST_SUITE_START(mip_fields);

    MICROSTRAIN_TEST_ADD(mip_fields, Rename_me);
    MICROSTRAIN_TEST_SUITE_RUN("Mip Fields", mip_fields);

    MICROSTRAIN_TEST_SUITE_END(mip_fields);

    return MICROSTRAIN_TEST_FAILURE_COUNT;
}
