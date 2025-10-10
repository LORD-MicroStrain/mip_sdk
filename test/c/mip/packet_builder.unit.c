#include <stdarg.h>
#include <string.h>

#include <mip/mip_offsets.h>
#include <mip/mip_packet.h>
#include <microstrain_test/unity_wrappers.h>

void setUp(void) {}
void tearDown(void) {}

void Creating_a_mip_packet_handles_when_the_buffer_size_is_too_large(void)
{
    mip_packet_view packet;
    uint8_t buffer[MIP_PACKET_LENGTH_MAX + 12345];
    const size_t buffer_size = sizeof(buffer);
    const uint8_t descriptor_set = 0x80;

    mip_packet_create(&packet, buffer, buffer_size, descriptor_set);

    ASSERT_EQUAL_INT(mip_packet_total_length(&packet), MIP_PACKET_LENGTH_MIN);
    ASSERT_EQUAL_INT(mip_packet_payload_length(&packet), 0);
    ASSERT_TRUE(mip_packet_is_sane(&packet));
}

void A_field_with_an_empty_payload_can_be_added_to_a_mip_packet(void)
{
    mip_packet_view packet;
    uint8_t buffer[MIP_PACKET_LENGTH_MAX];
    mip_packet_create(&packet, buffer, sizeof(buffer), 0x80);
    const uint8_t field_descriptor = 0x04;
    const uint8_t *empty_payload = NULL;
    const uint8_t payload_length = 0;

    const bool success = mip_packet_add_field(&packet, field_descriptor, empty_payload, payload_length);

    ASSERT_TRUE(success);
    ASSERT_EQUAL_INT(mip_packet_payload_length(&packet), MIP_FIELD_HEADER_LENGTH);
    ASSERT_EQUAL_INT(mip_packet_payload(&packet)[MIP_INDEX_FIELD_DESC], field_descriptor);
    ASSERT_EQUAL_INT(mip_packet_payload(&packet)[MIP_INDEX_FIELD_LEN], 2);

    mip_packet_finalize(&packet);
}

void A_pre_constructed_mip_field_can_be_added_to_a_mip_packet(void)
{
    mip_packet_view packet;
    uint8_t buffer[MIP_PACKET_LENGTH_MAX];
    mip_packet_create(&packet, buffer, sizeof(buffer), 0x80);
    const uint8_t field_descriptor = 0x05;
    const uint8_t payload[] = { 1, 2, 3, 4, 5, 6 };
    const uint8_t payload_length = sizeof(payload);

    const bool success = mip_packet_add_field(&packet, field_descriptor, payload, payload_length);

    ASSERT_TRUE(success);
    ASSERT_EQUAL_INT(mip_packet_payload(&packet)[MIP_INDEX_FIELD_DESC], field_descriptor);
    ASSERT_EQUAL_INT(mip_packet_payload(&packet)[MIP_INDEX_FIELD_LEN], 8);
    ASSERT_EQUAL_UINT8_ARRAY(&mip_packet_payload(&packet)[MIP_INDEX_FIELD_PAYLOAD], payload, payload_length);
}

void A_field_can_be_allocated_within_a_mip_packet(void)
{

    mip_packet_view packet;
    uint8_t buffer[MIP_PACKET_LENGTH_MAX];
    mip_packet_create(&packet, buffer, sizeof(buffer), 0x80);
    const uint8_t field_descriptor = 0x06;
    const uint8_t payload[] = { 0xAA, 0xBA, 0xAC, 0xDE, 0xFF, 0xFF, 0x99, 0x55 };
    const uint8_t payload_length = sizeof(payload);
    uint8_t* payload_pointer;

    const int result = mip_packet_create_field(&packet, field_descriptor, payload_length, &payload_pointer);
    memcpy(payload_pointer, payload, payload_length);

    ASSERT_GREATER_OR_EQUAL_INT_MESSAGE(result, 0, "Space couldn't be allocated for the field");
    ASSERT_EQUAL_PTR(payload_pointer, &mip_packet_payload(&packet)[MIP_INDEX_FIELD_PAYLOAD]);
    ASSERT_EQUAL_INT(mip_packet_payload(&packet)[MIP_INDEX_FIELD_DESC], field_descriptor);
    ASSERT_EQUAL_INT(mip_packet_payload(&packet)[MIP_INDEX_FIELD_LEN], 10);
    ASSERT_EQUAL_UINT8_ARRAY(&mip_packet_payload(&packet)[MIP_INDEX_FIELD_PAYLOAD], payload, payload_length);
}

void A_mip_packet_can_be_properly_prepared_for_transmission(void)
{
    mip_packet_view packet;
    uint8_t buffer[MIP_PACKET_LENGTH_MAX];
    mip_packet_create(&packet, buffer, sizeof(buffer), 0x80);

    mip_packet_finalize(&packet);

    // TODO: Uncomment when proper values are determined
    //ASSERT_EQUAL_INT(mip_packet_checksum_value(&packet), 0x80F0);
    //MICROSTRAIN_ASSERT_MESSAGE(buffer[MIP_PACKET_LENGTH_MAX] == 0x00, "Extra byte at end of buffer got clobbered");
}

int main()
{
    UNITY_BEGIN();

    // Test suite: Mip packet builder
    RUN_TEST(Creating_a_mip_packet_handles_when_the_buffer_size_is_too_large);
    RUN_TEST(A_field_with_an_empty_payload_can_be_added_to_a_mip_packet);
    RUN_TEST(A_pre_constructed_mip_field_can_be_added_to_a_mip_packet);
    RUN_TEST(A_field_can_be_allocated_within_a_mip_packet);
    RUN_TEST(A_mip_packet_can_be_properly_prepared_for_transmission);

    return UNITY_END();
}
