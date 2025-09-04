#include <doctest/doctest.h>

#include <mip/mip_logging.hpp>


constexpr uint8_t SET_AND_SAVE_COMM_SPEED_PAYLOAD[] = {
    0x75, 0x65, 0x01, 0x0c, 0x08, 0x09,
    0x01, 0x01, 0x00, 0x01, 0xc2, 0x00,
    0x04, 0x09, 0x03, 0x01,
    0xCE, 0x80
};

TEST_SUITE_BEGIN("Formatting");

TEST_CASE("Formatting | A packet can be properly formatted to bytes")
{
    char buffer[128];
    size_t index = 0;

    bool ok = mip::formatPacketBytes(buffer, &index, {SET_AND_SAVE_COMM_SPEED_PAYLOAD});

    CHECK(ok);
    CHECK(index == 39);
    CHECK(strcmp(buffer, "7565010C 080901010001C200 04090301 CE80") == 0);
}

TEST_CASE("Formatting | A packet can be properly formatted to a human-readable string")
{
    char buffer[128];
    size_t index = 0;

    bool ok = mip::formatPacket(buffer, &index, {SET_AND_SAVE_COMM_SPEED_PAYLOAD});

    CHECK(ok);
    CHECK(index == 68);
    // TODO: Add assertion for strings that outputs the string values
    CHECK(strcmp(buffer, "Packet(DS=0x01){ Field(FD=0x09)[01010001C200] Field(FD=0x09)[0301] }") == 0);
}

TEST_CASE("Formatting | A field can be properly formatted to a human-readable string")
{
    char buffer[128];
    size_t index = 0;
    const mip::FieldView field = mip::PacketView(SET_AND_SAVE_COMM_SPEED_PAYLOAD).firstField();

    bool ok = mip::formatField(buffer, &index, field);

    CHECK(ok);
    CHECK(index == 28);
    CHECK(strcmp(buffer, "Field(FD=0x09)[01010001C200]") == 0);
}

TEST_SUITE_END();
