#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include <mip_cmocka.h>
#include <mip/mip_offsets.h>
#include <mip/mip_packet.h>

 /*
    //payload_size += 2;



    payload_size += 2+sizeof(payload1);


*/

int main()
{
    MICROSTRAIN_TEST_INIT;

    MICROSTRAIN_TEST_SUITE_START(mip_packet_builder);

    // TODO: Add tests here
    MICROSTRAIN_TEST_SUITE_RUN("Mip packet builder", mip_packet_builder);

    MICROSTRAIN_TEST_SUITE_END(mip_packet_builder);

    return MICROSTRAIN_TEST_FAILURE_COUNT;
}


/*

void test_short_buffer()
{
    struct mip_packet_view packet;

    const int payload_space = 2;
    mip_packet_create(&packet, buffer, MIP_PACKET_LENGTH_MIN+MIP_FIELD_HEADER_LENGTH+payload_space, 0x80);

    uint8_t* p;
    check_equal( mip_packet_create_field(&packet, 0x04, 3, &p), -1, "Wrong remaining count after allocating 1 too many bytes" );
    check_equal( mip_packet_create_field(&packet, 0x04, MIP_FIELD_PAYLOAD_LENGTH_MAX, &p), payload_space-MIP_FIELD_PAYLOAD_LENGTH_MAX, "Wrong remaining count after allocating max payload" );
    check_equal( mip_packet_create_field(&packet, 0x04, 253, &p), payload_space-253, "Wrong remaining count after allocating excessive payload" );
    check_equal( mip_packet_create_field(&packet, 0x05, 1, &p), 1, "Wrong remaining size after allocating 3 bytes" );
    check_equal( mip_packet_create_field(&packet, 0x06, 1, &p), payload_space-3-1, "Wrong remaining size after allocating 3 more bytes" );
}
*/
