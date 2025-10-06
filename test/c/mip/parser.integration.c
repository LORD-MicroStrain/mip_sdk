// TODO: This is not a structured properly due to the current interface. Refactor this to be
//       structured properly when interface supports it.

#include <stdio.h>
#include <stdlib.h>

#include <mip_cmocka.h>
#include <mip/mip_parser.h>


struct ParseResults
{
    size_t length;
};

struct ParseResults parse_results;

uint8_t parse_buffer[1024];
size_t bytes_parsed = 0;
uint8_t check_buffer[MIP_PACKET_LENGTH_MAX];

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

    FILE* expected_data_file = p;

    parse_results.length = mip_packet_total_length(packet);

    // size_t written = fwrite(mip_packet_buffer(packet), 1, length, outfile);
    // return written == length;

    bytes_parsed += parse_results.length;

    const uint8_t* packet_buffer = mip_packet_pointer(packet);

    //assert_memory_equal(check_buffer, packet_buffer, parse_results.length); // TODO: move and remove

    return true;
}


MICROSTRAIN_TEST_CASE(RENAME_ME)
{
    // TODO: Copy files over during build + switch to build versions
    FILE *actual_data_file = NULL; openFile(&actual_data_file, "C:/HBK/Dev/mip_sdk/test/c/mip/../../data/mip_data.bin");
    FILE *expected_data_file = NULL; openFile(&expected_data_file, "C:/HBK/Dev/mip_sdk/test/c/mip/../../data/packet_example_cpp_check.txt");
    uint8_t input_buffer[1024];
    mip_parser parser;
    size_t bytes_read = 0;

    mip_parser_init(&parser, &handle_packet, expected_data_file, MIP_PARSER_DEFAULT_TIMEOUT_MS);

    const size_t num_to_read = 1024; // Arbitrary number picked for no particular reason
    const size_t num_read = fread(input_buffer, 1, num_to_read, actual_data_file);
    bytes_read += num_read;


    mip_parser_parse(&parser, input_buffer, num_read, 0);

    const size_t read = fread(check_buffer, 1, parse_results.length, expected_data_file);

    assert_int_less_or_equal(parse_results.length, MIP_PACKET_LENGTH_MAX);
    assert_int_equal(read, parse_results.length);
    assert_int_equal(bytes_parsed, bytes_read);
    fclose(actual_data_file);
    fclose(expected_data_file);
}

int main()
{
    MICROSTRAIN_TEST_INIT;

    MICROSTRAIN_TEST_SUITE_START(mip_parser);

    MICROSTRAIN_TEST_ADD(mip_parser, RENAME_ME);
    MICROSTRAIN_TEST_SUITE_RUN("Mip parser", mip_parser);

    MICROSTRAIN_TEST_SUITE_END(mip_parser);

    return MICROSTRAIN_TEST_FAILURE_COUNT;
}

