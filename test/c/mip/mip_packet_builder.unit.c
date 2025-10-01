#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include <mip_cmocka.h>
#include <mip/mip_offsets.h>
#include <mip/mip_packet.h>


MICROSTRAIN_TEST_CASE(Creating_a_mip_packet_handles_when_the_buffer_size_is_too_large)
{
    mip_packet_view packet;
    uint8_t buffer[MIP_PACKET_LENGTH_MAX + 12345];

    mip_packet_create(&packet, buffer, sizeof(buffer), 0x80);

    assert_int_equal(mip_packet_total_length(&packet), MIP_PACKET_LENGTH_MIN);
    assert_int_equal(mip_packet_payload_length(&packet), 0);
    assert_true(mip_packet_is_sane(&packet));
}

int main()
{
    MICROSTRAIN_TEST_INIT;

    MICROSTRAIN_TEST_SUITE_START(mip_packet_builder);

    MICROSTRAIN_TEST_ADD(mip_packet_builder, Creating_a_mip_packet_handles_when_the_buffer_size_is_too_large);
    MICROSTRAIN_TEST_SUITE_RUN("Mip packet builder", mip_packet_builder);

    MICROSTRAIN_TEST_SUITE_END(mip_packet_builder);

    return MICROSTRAIN_TEST_FAILURE_COUNT;
}
