#include <doctest/doctest.h>

TEST_CASE("REMOVE ME")
{
	REQUIRE(true);
}

/*
extern "C" {
#include "../../c/microstrain/testutil_strings.h"
}

#include <mip/mip_logging.hpp>

// 7565010C 080901010001c200 04090301 CE80
const uint8_t SET_SAVE_COMM_SPEED[] = {
    0x75, 0x65, 0x01, 0x0c, 0x08, 0x09,
    0x01, 0x01, 0x00, 0x01, 0xc2, 0x00,
    0x04, 0x09, 0x03, 0x01,
    0xCE, 0x80
};


void packet_to_bytes_works()
{
    char buffer[128];
    size_t index = 0;

    bool ok = mip::formatPacketBytes(buffer, &index, {SET_SAVE_COMM_SPEED});

    TEST_ASSERT(ok, "Success");
    TEST_ASSERT_EQ(index, 39, "Index is correct");
    TEST_ASSERT_BUFFER_COMPARE(buffer, "7565010C 080901010001C200 04090301 CE80", 39+1, "");
}

void format_packet_works()
{
    char buffer[128];
    size_t index = 0;

    bool ok = mip::formatPacket(buffer, &index, {SET_SAVE_COMM_SPEED});

    TEST_ASSERT(ok, "Success");
    TEST_ASSERT_EQ(index, 68, "Index is correct");
    TEST_ASSERT_BUFFER_COMPARE(buffer, "Packet(DS=0x01){ Field(FD=0x09)[01010001C200] Field(FD=0x09)[0301] }", 68+1, "");
}

void format_field_works()
{
    char buffer[128];
    size_t index = 0;
    mip::FieldView field = mip::PacketView(SET_SAVE_COMM_SPEED).firstField();

    bool ok = mip::formatField(buffer, &index, field);

    TEST_ASSERT(ok, "Success");
    TEST_ASSERT_EQ(index, 28, "Index is correct");
    TEST_ASSERT_BUFFER_COMPARE(buffer, "Field(FD=0x09)[01010001C200]", 28+1, "");
}

int main()
{
    packet_to_bytes_works();
    format_packet_works();
    format_field_works();

    return (int)g_fail_count;
}
*/