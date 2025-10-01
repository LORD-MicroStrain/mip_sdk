#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include <mip_cmocka.h>
#include <mip/mip_offsets.h>
#include <mip/mip_packet.h>

#define EXTRA 1


MICROSTRAIN_TEST_CASE(A_mip_packet_can_be_built_from_an_existing_buffer)
{
    mip_packet_view packet;
    uint8_t buffer[MIP_PACKET_LENGTH_MAX + EXTRA];

    mip_packet_from_buffer(&packet, buffer, sizeof(buffer));

    assert_memory_equal(packet._buffer, buffer, sizeof(buffer));
    assert_int_equal(packet._buffer_length, sizeof(buffer) - EXTRA);
}

int main()
{
    MICROSTRAIN_TEST_INIT;

    MICROSTRAIN_TEST_SUITE_START(mip_packet_builder);

    MICROSTRAIN_TEST_ADD(mip_packet_builder, A_mip_packet_can_be_built_from_an_existing_buffer);
    MICROSTRAIN_TEST_SUITE_RUN("Mip packet builder", mip_packet_builder);

    MICROSTRAIN_TEST_SUITE_END(mip_packet_builder);

    return MICROSTRAIN_TEST_FAILURE_COUNT;
}
