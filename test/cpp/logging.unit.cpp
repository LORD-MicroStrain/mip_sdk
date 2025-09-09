#include <microstrain_test.hpp>
#include <mip/mip_logging.hpp>

constexpr uint8_t SET_AND_SAVE_COMM_SPEED_PACKET[] = {
    0x75, 0x65, 0x01, 0x0c, 0x08, 0x09,
    0x01, 0x01, 0x00, 0x01, 0xc2, 0x00,
    0x04, 0x09, 0x03, 0x01,
    0xCE, 0x80
};

TEST("Formatting", "A packet can be properly formatted to bytes")
{
    char buffer[128];
    size_t index = 0;

    const bool ok = mip::formatPacketBytes(buffer, &index, {SET_AND_SAVE_COMM_SPEED_PACKET});

    EXPECT_TO_BE_TRUE(ok);
    EXPECT_TO_BE_EQUAL(38, index);
    EXPECT_C_STRINGS_TO_BE_EQUAL(buffer, "7565010C 080901010001C200 04090301 CE80");
}

TEST("Formatting", "A packet can be properly formatted to a human-readable string")
{
    char buffer[128];
    size_t index = 0;

    const bool ok = mip::formatPacket(buffer, &index, {SET_AND_SAVE_COMM_SPEED_PACKET});

    EXPECT_TO_BE_TRUE(ok);
    EXPECT_TO_BE_EQUAL(68, index);
    EXPECT_C_STRINGS_TO_BE_EQUAL(buffer, "Packet(DS=0x01){ Field(FD=0x09)[01010001C200] Field(FD=0x09)[0301] }");
}

TEST("Formatting", "A field can be properly formatted to a human-readable string")
{
    char buffer[128];
    size_t index = 0;
    const mip::FieldView field = mip::PacketView(SET_AND_SAVE_COMM_SPEED_PACKET).firstField();

    const bool ok = mip::formatField(buffer, &index, field);

    EXPECT_TO_BE_TRUE(ok);
    EXPECT_TO_BE_EQUAL(28, index);
    EXPECT_C_STRINGS_TO_BE_EQUAL(buffer, "Field(FD=0x09)[01010001C200]");
}
