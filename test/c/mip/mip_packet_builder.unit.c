#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include <mip_cmocka.h>
#include <mip/mip_offsets.h>
#include <mip/mip_packet.h>

#define EXTRA 1


int main()
{
    MICROSTRAIN_TEST_INIT;

    MICROSTRAIN_TEST_SUITE_START(mip_packet_builder);

    MICROSTRAIN_TEST_SUITE_RUN("Mip packet builder", mip_packet_builder);

    MICROSTRAIN_TEST_SUITE_END(mip_packet_builder);

    return MICROSTRAIN_TEST_FAILURE_COUNT;
}
