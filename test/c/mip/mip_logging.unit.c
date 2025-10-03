// TODO: Convert these tests when logging structure decided on.
/*
#include <stdio.h>
#include <string.h>
*/
#include <mip_cmocka.h>
/*
#include <microstrain/strings.h>
#include <mip/mip_logging.h>


// 75650102 0201 E0C6
const uint8_t PING_PACKET[] = { 0x75, 0x65, 0x01, 0x02, 0x02, 0x01, 0xE0, 0xC6 };

//7565010C 080901010001c200 04090301 CE80
const uint8_t SET_SAVE_COMM_SPEED[] = {
    0x75, 0x65, 0x01, 0x0c, 0x08, 0x09,
    0x01, 0x01, 0x00, 0x01, 0xc2, 0x00,
    0x04, 0x09, 0x03, 0x01,
    0xCE, 0x80
};

// 756501FF FF0102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F202122232425262728292A2B2C2D2E2F303132333435363738393A3B3C3D3E3F404142434445464748494A4B4C4D4E4F505152535455565758595A5B5C5D5E5F606162636465666768696A6B6C6D6E6F707172737475767778797A7B7C7D7E7F808182838485868788898A8B8C8D8E8F909192939495969798999A9B9C9D9E9FA0A1A2A3A4A5A6A7A8A9AAABACADAEAFB0B1B2B3B4B5B6B7B8B9BABBBCBDBEBFC0C1C2C3C4C5C6C7C8C9CACBCCCDCECFD0D1D2D3D4D5D6D7D8D9DADBDCDDDEDFE0E1E2E3E4E5E6E7E8E9EAEBECEDEEEFF0F1F2F3F4F5F6F7F8F9FAFBFCFDFE 5A2B
const uint8_t MAX_PACKET_SINGLE_FIELD[] = {
    0x75, 0x65, 0x01, 0xff,
    0xff, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
    0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f,
    0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d, 0x2e, 0x2f,
    0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f,
    0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a, 0x4b, 0x4c, 0x4d, 0x4e, 0x4f,
    0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x5b, 0x5c, 0x5d, 0x5e, 0x5f,
    0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x6b, 0x6c, 0x6d, 0x6e, 0x6f,
    0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7a, 0x7b, 0x7c, 0x7d, 0x7e, 0x7f,
    0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8a, 0x8b, 0x8c, 0x8d, 0x8e, 0x8f,
    0x90, 0x91, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9a, 0x9b, 0x9c, 0x9d, 0x9e, 0x9f,
    0xa0, 0xa1, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xab, 0xac, 0xad, 0xae, 0xaf,
    0xb0, 0xb1, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xbb, 0xbc, 0xbd, 0xbe, 0xbf,
    0xc0, 0xc1, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xcb, 0xcc, 0xcd, 0xce, 0xcf,
    0xd0, 0xd1, 0xd2, 0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xdb, 0xdc, 0xdd, 0xde, 0xdf,
    0xe0, 0xe1, 0xe2, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea, 0xeb, 0xec, 0xed, 0xee, 0xef,
    0xf0, 0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8, 0xf9, 0xfa, 0xfb, 0xfc, 0xfd, 0xfe,
    0x5a, 0x2b
};

// 756501FF 200102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F 200102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F 200102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F 200102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F 200102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F 200102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F 200102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F 200102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E 3a0b
const uint8_t MAX_PACKET_MULTIPLE_FIELDS[] = {
    0x75, 0x65, 0x01, 0xff,
    0x20, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f,
    0x20, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f,
    0x20, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f,
    0x20, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f,
    0x20, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f,
    0x20, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f,
    0x20, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f,
    0x1F, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e,
    0x3A, 0x0B
};

char g_buffer[4096];
size_t g_length = 0;

// TODO: Replace callback with mock that calls microstrain_string_format_v
// TODO: Replace g_buffer with local buffer initialized in each test
// TODO: Make sure user is null so there are no shared dependencies
void log_callback(const void* user, const microstrain_log_level level, const char* fmt, const va_list args)
{
    (void)user;

    microstrain_string_format_v(g_buffer, sizeof(g_buffer), &g_length, fmt, args);
}


MICROSTRAIN_TEST_CASE(Bytes_can_be_logged_correctly)
{
    microstrain_logging_init(&log_callback, MICROSTRAIN_LOG_LEVEL_INFO, NULL);

    MICROSTRAIN_LOG_BYTES(MICROSTRAIN_LOG_LEVEL_INFO, "Test: ", PING_PACKET, sizeof(PING_PACKET));

    /*
    microstrain_logging_init(NULL, MICROSTRAIN_LOG_LEVEL_OFF, NULL);
    TEST_ASSERT_BUFFER_COMPARE(g_buffer, "Test: 7565 0102 0201 E0C6\n", 6+19+1+1, "");
#1#
}
*/
int main()
{
    MICROSTRAIN_TEST_INIT;

    MICROSTRAIN_TEST_SUITE_START(mip_logging);

    //MICROSTRAIN_TEST_ADD(mip_logging, Bytes_can_be_logged_correctly);
    MICROSTRAIN_TEST_SUITE_RUN("Mip logging", mip_logging);

    MICROSTRAIN_TEST_SUITE_END(mip_logging);

    return MICROSTRAIN_TEST_FAILURE_COUNT;
}

/*

void fmt_ping_packet_matches_expected_result()
{
    mip_packet_view packet;
    mip_packet_from_buffer(&packet, PING_PACKET, sizeof(PING_PACKET));

    microstrain_logging_init(&log_callback, MICROSTRAIN_LOG_LEVEL_INFO, NULL);
    g_length = 0;

    mip_log_packet(&packet, MICROSTRAIN_LOG_LEVEL_INFO);

    microstrain_logging_init(NULL, MICROSTRAIN_LOG_LEVEL_OFF, NULL);

    const char* compare = "Packet(DS=0x01){ Field(FD=0x01)[] }\n";

    TEST_ASSERT_BUFFER_COMPARE(g_buffer, compare, strlen(compare), "");
}

void fmt_packet_with_multiple_fields_works()
{
    mip_packet_view packet;
    mip_packet_from_buffer(&packet, SET_SAVE_COMM_SPEED, sizeof(SET_SAVE_COMM_SPEED));

    char buffer[4096];
    microstrain_logging_init(&log_callback, MICROSTRAIN_LOG_LEVEL_INFO, &buffer);
    g_length = 0;

    mip_log_packet(&packet, MICROSTRAIN_LOG_LEVEL_INFO);
    // mip_log_packet(&packet, 1, MICROSTRAIN_LOG_LEVEL_INFO);
    // mip_log_packet(&packet, 2, MICROSTRAIN_LOG_LEVEL_INFO);
    // mip_log_packet(&packet, 4, MICROSTRAIN_LOG_LEVEL_INFO);

    microstrain_logging_init(NULL, MICROSTRAIN_LOG_LEVEL_OFF, NULL);

    // 7565010C 080901010001c200 04090301 CE80
    const char* compare = (
        "Packet(DS=0x01){ Field(FD=0x09)[01010001C200] Field(FD=0x09)[0301] }\n"
        // "Packet(DS=0x01){ Field(FD=0x01)[02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F 10 11 12 13 14 15 16 17 18 19 1A 1B 1C 1D 1E 1F] }\n"
        // "Packet(DS=0x01){ Field(FD=0x01)[0203 0405 0607 0809 0A0B 0C0D 0E0F 1011 1213 1415 1617 1819 1A1B 1C1D 1E1F] }\n"
        // "Packet(DS=0x01){ Field(FD=0x01)[02030405 06070809 0A0B0C0D 0E0F1011 12131415 16171819 1A1B1C1D 1E1F] }\n"
    );

    TEST_ASSERT_BUFFER_COMPARE(g_buffer, compare, strlen(compare), "");
}

void fmt_max_length_packet_with_single_field_works()
{
    mip_packet_view packet;
    mip_packet_from_buffer(&packet, MAX_PACKET_SINGLE_FIELD, sizeof(MAX_PACKET_SINGLE_FIELD));

    char buffer[4096];
    microstrain_logging_init(&log_callback, MICROSTRAIN_LOG_LEVEL_INFO, &buffer);
    g_length = 0;

    mip_log_packet(&packet, MICROSTRAIN_LOG_LEVEL_INFO);
    // mip_log_packet(&packet, 1, MICROSTRAIN_LOG_LEVEL_INFO);
    // mip_log_packet(&packet, 2, MICROSTRAIN_LOG_LEVEL_INFO);
    // mip_log_packet(&packet, 4, MICROSTRAIN_LOG_LEVEL_INFO);

    microstrain_logging_init(NULL, MICROSTRAIN_LOG_LEVEL_OFF, NULL);

    // 756501FF FF0102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F202122232425262728292A2B2C2D2E2F303132333435363738393A3B3C3D3E3F404142434445464748494A4B4C4D4E4F505152535455565758595A5B5C5D5E5F606162636465666768696A6B6C6D6E6F707172737475767778797A7B7C7D7E7F808182838485868788898A8B8C8D8E8F909192939495969798999A9B9C9D9E9FA0A1A2A3A4A5A6A7A8A9AAABACADAEAFB0B1B2B3B4B5B6B7B8B9BABBBCBDBEBFC0C1C2C3C4C5C6C7C8C9CACBCCCDCECFD0D1D2D3D4D5D6D7D8D9DADBDCDDDEDFE0E1E2E3E4E5E6E7E8E9EAEBECEDEEEFF0F1F2F3F4F5F6F7F8F9FAFBFCFDFE 5A2B
    const char* compare = (
        "Packet(DS=0x01){ Field(FD=0x01)[02030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F202122232425262728292A2B2C2D2E2F303132333435363738393A3B3C3D3E3F404142434445464748494A4B4C4D4E4F505152535455565758595A5B5C5D5E5F606162636465666768696A6B6C6D6E6F707172737475767778797A7B7C7D7E7F808182838485868788898A8B8C8D8E8F909192939495969798999A9B9C9D9E9FA0A1A2A3A4A5A6A7A8A9AAABACADAEAFB0B1B2B3B4B5B6B7B8B9BABBBCBDBEBFC0C1C2C3C4C5C6C7C8C9CACBCCCDCECFD0D1D2D3D4D5D6D7D8D9DADBDCDDDEDFE0E1E2E3E4E5E6E7E8E9EAEBECEDEEEFF0F1F2F3F4F5F6F7F8F9FAFBFCFDFE] }\n"
        // "Packet(DS=0x01){ Field(FD=0x01)[02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F 10 11 12 13 14 15 16 17 18 19 1A 1B 1C 1D 1E 1F 20 21 22 23 24 25 26 27 28 29 2A 2B 2C 2D 2E 2F 30 31 32 33 34 35 36 37 38 39 3A 3B 3C 3D 3E 3F 40 41 42 43 44 45 46 47 48 49 4A 4B 4C 4D 4E 4F 50 51 52 53 54 55 56 57 58 59 5A 5B 5C 5D 5E 5F 60 61 62 63 64 65 66 67 68 69 6A 6B 6C 6D 6E 6F 70 71 72 73 74 75 76 77 78 79 7A 7B 7C 7D 7E 7F 80 81 82 83 84 85 86 87 88 89 8A 8B 8C 8D 8E 8F 90 91 92 93 94 95 96 97 98 99 9A 9B 9C 9D 9E 9F A0 A1 A2 A3 A4 A5 A6 A7 A8 A9 AA AB AC AD AE AF B0 B1 B2 B3 B4 B5 B6 B7 B8 B9 BA BB BC BD BE BF C0 C1 C2 C3 C4 C5 C6 C7 C8 C9 CA CB CC CD CE CF D0 D1 D2 D3 D4 D5 D6 D7 D8 D9 DA DB DC DD DE DF E0 E1 E2 E3 E4 E5 E6 E7 E8 E9 EA EB EC ED EE EF F0 F1 F2 F3 F4 F5 F6 F7 F8 F9 FA FB FC FD FE] }\n"
        // "Packet(DS=0x01){ Field(FD=0x01)[0203 0405 0607 0809 0A0B 0C0D 0E0F 1011 1213 1415 1617 1819 1A1B 1C1D 1E1F 2021 2223 2425 2627 2829 2A2B 2C2D 2E2F 3031 3233 3435 3637 3839 3A3B 3C3D 3E3F 4041 4243 4445 4647 4849 4A4B 4C4D 4E4F 5051 5253 5455 5657 5859 5A5B 5C5D 5E5F 6061 6263 6465 6667 6869 6A6B 6C6D 6E6F 7071 7273 7475 7677 7879 7A7B 7C7D 7E7F 8081 8283 8485 8687 8889 8A8B 8C8D 8E8F 9091 9293 9495 9697 9899 9A9B 9C9D 9E9F A0A1 A2A3 A4A5 A6A7 A8A9 AAAB ACAD AEAF B0B1 B2B3 B4B5 B6B7 B8B9 BABB BCBD BEBF C0C1 C2C3 C4C5 C6C7 C8C9 CACB CCCD CECF D0D1 D2D3 D4D5 D6D7 D8D9 DADB DCDD DEDF E0E1 E2E3 E4E5 E6E7 E8E9 EAEB ECED EEEF F0F1 F2F3 F4F5 F6F7 F8F9 FAFB FCFD FE] }\n"
        // "Packet(DS=0x01){ Field(FD=0x01)[02030405 06070809 0A0B0C0D 0E0F1011 12131415 16171819 1A1B1C1D 1E1F2021 22232425 26272829 2A2B2C2D 2E2F3031 32333435 36373839 3A3B3C3D 3E3F4041 42434445 46474849 4A4B4C4D 4E4F5051 52535455 56575859 5A5B5C5D 5E5F6061 62636465 66676869 6A6B6C6D 6E6F7071 72737475 76777879 7A7B7C7D 7E7F8081 82838485 86878889 8A8B8C8D 8E8F9091 92939495 96979899 9A9B9C9D 9E9FA0A1 A2A3A4A5 A6A7A8A9 AAABACAD AEAFB0B1 B2B3B4B5 B6B7B8B9 BABBBCBD BEBFC0C1 C2C3C4C5 C6C7C8C9 CACBCCCD CECFD0D1 D2D3D4D5 D6D7D8D9 DADBDCDD DEDFE0E1 E2E3E4E5 E6E7E8E9 EAEBECED EEEFF0F1 F2F3F4F5 F6F7F8F9 FAFBFCFD FE] }\n"
    );

    TEST_ASSERT_BUFFER_COMPARE(g_buffer, compare, strlen(compare), "");
}

void fmt_max_length_packet_with_multiple_fields_works()
{
    mip_packet_view packet;
    mip_packet_from_buffer(&packet, MAX_PACKET_MULTIPLE_FIELDS, sizeof(MAX_PACKET_MULTIPLE_FIELDS));

    char buffer[4096];
    microstrain_logging_init(&log_callback, MICROSTRAIN_LOG_LEVEL_INFO, &buffer);
    g_length = 0;

    mip_log_packet(&packet, MICROSTRAIN_LOG_LEVEL_INFO);

    microstrain_logging_init(NULL, MICROSTRAIN_LOG_LEVEL_OFF, NULL);

    // 756501FF 200102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F 200102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F 200102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F 200102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F 200102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F 200102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F 200102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F 200102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E 3a0b
    const char* compare = (
        "Packet(DS=0x01){ Field(FD=0x01)[02030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F] Field(FD=0x01)[02030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F] Field(FD=0x01)[02030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F] Field(FD=0x01)[02030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F] Field(FD=0x01)[02030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F] Field(FD=0x01)[02030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F] Field(FD=0x01)[02030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F] Field(FD=0x01)[02030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E] }\n"
        // "Packet(DS=0x01){ Field(FD=0x01)[02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F 10 11 12 13 14 15 16 17 18 19 1A 1B 1C 1D 1E 1F] Field(FD=0x01)[02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F 10 11 12 13 14 15 16 17 18 19 1A 1B 1C 1D 1E 1F] Field(FD=0x01)[02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F 10 11 12 13 14 15 16 17 18 19 1A 1B 1C 1D 1E 1F] Field(FD=0x01)[02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F 10 11 12 13 14 15 16 17 18 19 1A 1B 1C 1D 1E 1F] Field(FD=0x01)[02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F 10 11 12 13 14 15 16 17 18 19 1A 1B 1C 1D 1E 1F] Field(FD=0x01)[02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F 10 11 12 13 14 15 16 17 18 19 1A 1B 1C 1D 1E 1F] Field(FD=0x01)[02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F 10 11 12 13 14 15 16 17 18 19 1A 1B 1C 1D 1E 1F] Field(FD=0x01)[02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F 10 11 12 13 14 15 16 17 18 19 1A 1B 1C 1D 1E] }\n"
        // "Packet(DS=0x01){ Field(FD=0x01)[0203 0405 0607 0809 0A0B 0C0D 0E0F 1011 1213 1415 1617 1819 1A1B 1C1D 1E1F] Field(FD=0x01)[0203 0405 0607 0809 0A0B 0C0D 0E0F 1011 1213 1415 1617 1819 1A1B 1C1D 1E1F] Field(FD=0x01)[0203 0405 0607 0809 0A0B 0C0D 0E0F 1011 1213 1415 1617 1819 1A1B 1C1D 1E1F] Field(FD=0x01)[0203 0405 0607 0809 0A0B 0C0D 0E0F 1011 1213 1415 1617 1819 1A1B 1C1D 1E1F] Field(FD=0x01)[0203 0405 0607 0809 0A0B 0C0D 0E0F 1011 1213 1415 1617 1819 1A1B 1C1D 1E1F] Field(FD=0x01)[0203 0405 0607 0809 0A0B 0C0D 0E0F 1011 1213 1415 1617 1819 1A1B 1C1D 1E1F] Field(FD=0x01)[0203 0405 0607 0809 0A0B 0C0D 0E0F 1011 1213 1415 1617 1819 1A1B 1C1D 1E1F] Field(FD=0x01)[0203 0405 0607 0809 0A0B 0C0D 0E0F 1011 1213 1415 1617 1819 1A1B 1C1D 1E] }\n"
        // "Packet(DS=0x01){ Field(FD=0x01)[02030405 06070809 0A0B0C0D 0E0F1011 12131415 16171819 1A1B1C1D 1E1F] Field(FD=0x01)[02030405 06070809 0A0B0C0D 0E0F1011 12131415 16171819 1A1B1C1D 1E1F] Field(FD=0x01)[02030405 06070809 0A0B0C0D 0E0F1011 12131415 16171819 1A1B1C1D 1E1F] Field(FD=0x01)[02030405 06070809 0A0B0C0D 0E0F1011 12131415 16171819 1A1B1C1D 1E1F] Field(FD=0x01)[02030405 06070809 0A0B0C0D 0E0F1011 12131415 16171819 1A1B1C1D 1E1F] Field(FD=0x01)[02030405 06070809 0A0B0C0D 0E0F1011 12131415 16171819 1A1B1C1D 1E1F] Field(FD=0x01)[02030405 06070809 0A0B0C0D 0E0F1011 12131415 16171819 1A1B1C1D 1E1F] Field(FD=0x01)[02030405 06070809 0A0B0C0D 0E0F1011 12131415 16171819 1A1B1C1D 1E] }\n"
    );

    TEST_ASSERT_BUFFER_COMPARE(g_buffer, compare, strlen(compare), "");
}

void fmt_invalid_packet_views_match_expected_results()
{
    microstrain_logging_init(&log_callback, MICROSTRAIN_LOG_LEVEL_INFO, NULL);
    g_length = 0;

    mip_packet_view packet;

    // Test up to (but not including) the minimum packet length.
    for(unsigned int i=0; i<6; i++)
    {
        mip_packet_from_buffer(&packet, PING_PACKET, i);
        mip_log_packet(&packet, MICROSTRAIN_LOG_LEVEL_INFO);
    }

    microstrain_logging_init(NULL, MICROSTRAIN_LOG_LEVEL_OFF, NULL);

    const char* compare = (
        "Invalid Packet: []\n"
        "Invalid Packet: [75]\n"
        "Invalid Packet: [7565]\n"
        "Invalid Packet: [756501]\n"
        "Invalid Packet: [75650102]\n"
        "Invalid Packet: [7565010202]\n"
    );

    TEST_ASSERT_BUFFER_COMPARE(g_buffer, compare, strlen(compare), "");
}

void fmt_invalid_packet_matches_expected_result()
{
    microstrain_logging_init(&log_callback, MICROSTRAIN_LOG_LEVEL_INFO, NULL);
    g_length = 0;

    const uint8_t packet_buffer[] = {
        0x75, 0x65, 0x01, 0x20,
        0x20, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
        0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f,
        0xDE, 0xAD //0x0b, 0xd5
    };
    mip_packet_view packet;
    mip_packet_from_buffer(&packet, packet_buffer, sizeof(packet_buffer));

    mip_log_packet(&packet, MICROSTRAIN_LOG_LEVEL_INFO);

    microstrain_logging_init(NULL, MICROSTRAIN_LOG_LEVEL_OFF, NULL);

    const char* compare = "Packet(DS=0x01){ Field(FD=0x01)[02030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F] BAD_CHECKSUM(DEAD!=0BD5) }\n";

    TEST_ASSERT_BUFFER_COMPARE(g_buffer, compare, strlen(compare), "");
}

void verbose_log_packet_matches_expected_result()
{
    mip_packet_view packet;
    mip_packet_from_buffer(&packet, SET_SAVE_COMM_SPEED, sizeof(SET_SAVE_COMM_SPEED));

    microstrain_logging_init(&log_callback, MICROSTRAIN_LOG_LEVEL_INFO, NULL);
    g_length = 0;

    mip_log_packet_verbose(&packet, MICROSTRAIN_LOG_LEVEL_INFO);

    microstrain_logging_init(NULL, MICROSTRAIN_LOG_LEVEL_OFF, NULL);

    // 7565010C 080901010001c200 04090301 CE80
    const char* compare = (
        "Packet: [7565010C 080901010001C200 04090301 CE80]\n"
        "    Total Length         = 18\n"
        "    MIP SYNC1            = 0x75\n"
        "    MIP SYNC2            = 0x65\n"
        "    Descriptor Set       = 0x01\n"
        "    Payload Length       = 12 (0x0C)\n"
        "    Checksum             = 0xCE80 (valid)\n"
        "    Field 1: [0809 01010001C200]\n"
        "        Field Length     = 8 (0x08)\n"
        "        Field Descriptor = 0x09\n"
        "    Field 2: [0409 0301]\n"
        "        Field Length     = 4 (0x04)\n"
        "        Field Descriptor = 0x09\n"
    );

    TEST_ASSERT_BUFFER_COMPARE(g_buffer, compare, strlen(compare), "");
}

void verbose_log_insane_packet_matches_expected_result()
{
    mip_packet_view packet;

    microstrain_logging_init(&log_callback, MICROSTRAIN_LOG_LEVEL_INFO, NULL);
    g_length = 0;

    for(unsigned int i=0; i<6; i++)
    {
        mip_packet_from_buffer(&packet, PING_PACKET, i);
        mip_log_packet_verbose(&packet, MICROSTRAIN_LOG_LEVEL_INFO);
    }

    microstrain_logging_init(NULL, MICROSTRAIN_LOG_LEVEL_OFF, NULL);

    const char* compare = (
        "Invalid Packet: []\n"
        "Invalid Packet: [75]\n"
        "Invalid Packet: [7565]\n"
        "Invalid Packet: [756501]\n"
        "Invalid Packet: [75650102]\n"
        "Invalid Packet: [7565010202]\n"
    );

    TEST_ASSERT_BUFFER_COMPARE(g_buffer, compare, strlen(compare), "");
}

void verbose_log_packet_with_bad_checksum_matches_expected_result()
{
    mip_packet_view packet;
    const uint8_t packet_buffer[] = { 0x75, 0x65, 0x01, 0x02, 0x02, 0x01, 0xDE, 0xAD };
    mip_packet_from_buffer(&packet, packet_buffer, sizeof(packet_buffer));

    microstrain_logging_init(&log_callback, MICROSTRAIN_LOG_LEVEL_INFO, NULL);
    g_length = 0;

    mip_log_packet_verbose(&packet, MICROSTRAIN_LOG_LEVEL_INFO);

    microstrain_logging_init(NULL, MICROSTRAIN_LOG_LEVEL_OFF, NULL);

    const char* compare = (
        "Packet: [75650102 0201 DEAD]\n"
        "    Total Length         = 8\n"
        "    MIP SYNC1            = 0x75\n"
        "    MIP SYNC2            = 0x65\n"
        "    Descriptor Set       = 0x01\n"
        "    Payload Length       = 2 (0x02)\n"
        "    Checksum             = 0xDEAD (INVALID)\n"
        "    Field 1: [0201 ]\n"
        "        Field Length     = 2 (0x02)\n"
        "        Field Descriptor = 0x01\n"
    );

    TEST_ASSERT_BUFFER_COMPARE(g_buffer, compare, strlen(compare), "");
}


int main()
{
    microstrain_log_bytes_works();

    fmt_ping_packet_matches_expected_result();
    fmt_packet_with_multiple_fields_works();
    fmt_max_length_packet_with_single_field_works();
    fmt_max_length_packet_with_multiple_fields_works();
    fmt_invalid_packet_views_match_expected_results();
    fmt_invalid_packet_matches_expected_result();

    verbose_log_packet_matches_expected_result();
    verbose_log_packet_with_bad_checksum_matches_expected_result();
    verbose_log_insane_packet_matches_expected_result();

    return (int)g_fail_count;
}
#1#
*/
