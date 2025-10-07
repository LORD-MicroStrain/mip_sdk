// TODO: This is not a structured properly due to the current interface. Refactor this to be
//       structured properly when interface supports it.

#include <stdio.h>
#include <stdlib.h>

#include <mip_cmocka.h>
#include <mip/mip_parser.h>


struct ParseResults
{
    size_t length;
    uint8_t packet_buffer[MIP_PACKET_LENGTH_MAX];
    uint8_t parse_buffer[1024];
    size_t bytes_parsed;
};

void openFile(FILE **file, const char *filename)
{
    const errno_t error_code = fopen_s(file, filename, "rb");

    if (error_code != 0)
    {
        fail_msg("Could not open file: %s, error code: %d", filename, error_code);
    }
}

bool handle_packet(void* p, const mip_packet_view* packet, mip_timestamp t)
{
    (void)t;

    struct ParseResults *parse_results = p;

    parse_results->length = mip_packet_total_length(packet);

    // size_t written = fwrite(mip_packet_buffer(packet), 1, length, outfile);
    // return written == length;

    parse_results->bytes_parsed += parse_results->length;

    memcpy(parse_results->packet_buffer, mip_packet_pointer(packet), parse_results->length);

    return true;
}


MICROSTRAIN_TEST_CASE(Mip_packets_can_be_parsed_correctly)
{
    // TODO: Copy files over during build + switch to build versions
    // Arrange
    mip_parser parser;
    struct ParseResults parse_results = {.length = 0, .packet_buffer = {0}, .parse_buffer = {0}, .bytes_parsed = 0};
    mip_parser_init(&parser, &handle_packet, &parse_results, MIP_PARSER_DEFAULT_TIMEOUT_MS);

    FILE *actual_data_file = NULL; openFile(&actual_data_file, "C:/HBK/Dev/mip_sdk/test/c/mip/../../data/mip_data.bin");
    uint8_t input_buffer[1024];
    const size_t actual_element_count = fread(input_buffer, 1, 1024, actual_data_file);

    // Act
    mip_parser_parse(&parser, input_buffer, actual_element_count, 0);

    // Assert
    assert_int_less_or_equal(parse_results.length, MIP_PACKET_LENGTH_MAX);

    uint8_t check_buffer[MIP_PACKET_LENGTH_MAX];
    FILE *expected_data_file = NULL; openFile(&expected_data_file, "C:/HBK/Dev/mip_sdk/test/c/mip/../../data/packet_example_cpp_check.txt");
    const size_t expected_element_count = fread(check_buffer, 1, parse_results.length, expected_data_file);

    assert_int_equal(parse_results.length, expected_element_count);
    assert_int_equal(parse_results.bytes_parsed, expected_element_count);
    assert_memory_equal(check_buffer, parse_results.packet_buffer, parse_results.length);

    fclose(actual_data_file);
    fclose(expected_data_file);
}

int main()
{
    MICROSTRAIN_TEST_INIT;

    MICROSTRAIN_TEST_SUITE_START(mip_parser);

    MICROSTRAIN_TEST_ADD(mip_parser, Mip_packets_can_be_parsed_correctly);
    MICROSTRAIN_TEST_SUITE_RUN("Mip parser", mip_parser);

    MICROSTRAIN_TEST_SUITE_END(mip_parser);

    return MICROSTRAIN_TEST_FAILURE_COUNT;
}

