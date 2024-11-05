
#include <mip/mip_packet.h>
#include <mip/mip_parser.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>


struct mip_parser parser;

unsigned int num_errors = 0;

unsigned int num_packets_parsed = 0;
mip_timestamp parsed_packet_timestamp = 0;
size_t parsed_packet_length = 0;
uint8_t parsed_buffer[MIP_PACKET_LENGTH_MAX];
mip_packet_view parsed_packet = { ._buffer=parsed_buffer, ._buffer_length=0, };


void print_packet(FILE* out, const struct mip_packet_view* packet)
{
    size_t size = mip_packet_total_length(packet);
    const uint8_t* ptr = mip_packet_pointer(packet);

    for(size_t i=0; i<size; i++)
    {
        fprintf(out, " %02X", ptr[i]);
    }
    fputc('\n', out);
}


void handle_packet(void* p, const struct mip_packet_view* packet, mip_timestamp timestamp)
{
    (void)p;

    num_packets_parsed++;
    parsed_packet_timestamp = timestamp;
    parsed_packet_length = mip_packet_total_length(packet);
    memcpy(parsed_buffer, mip_packet_pointer(packet), parsed_packet_length);
    mip_packet_from_buffer(&parsed_packet, parsed_buffer, parsed_packet_length);
}

const bool PRINT_DEBUG = false;

uint8_t frand()
{
    static unsigned int seed = 0;
    seed = 214013*seed+2531011;
    return (seed >> 16) & 0xFF;
}

int main(int argc, const char* argv[])
{
    mip_parser_init(&parser, &handle_packet, NULL, MIP_PARSER_DEFAULT_TIMEOUT_MS);

    srand(0);

    mip_timestamp start_time = rand() % 500;

    const unsigned int NUM_ITERATIONS = 100*1000*1000;

    unsigned int last_parsed = 0;

    uint8_t buffer     [MIP_PACKET_LENGTH_MAX];
    uint8_t prev_buffer[MIP_PACKET_LENGTH_MAX];
    mip_packet_view packet      = { ._buffer=buffer,      ._buffer_length=sizeof(buffer) };
    mip_packet_view prev_packet = { ._buffer=prev_buffer, ._buffer_length=sizeof(buffer) };

    bool prev_timeout = false;

    unsigned int i;
    for(i=0; i<NUM_ITERATIONS; i++)
    {
        uint8_t desc_set = (rand() % 255) + 1;  // Random descriptor set.

        mip_packet_create(&packet, packet._buffer, packet._buffer_length, desc_set);

        for(unsigned int f=0; ; f++)
        {
            const size_t max_field_len = mip_packet_remaining_space(&packet);
            if( max_field_len < MIP_FIELD_HEADER_LENGTH )
                break;

            const uint8_t max_payload = (uint8_t)max_field_len - MIP_FIELD_HEADER_LENGTH;

            const uint8_t paylen = (rand() % (max_payload+1)) >> (rand() % 8);

            const uint8_t field_desc = (rand() % 255) + 1;  // Random field descriptor.

            uint8_t* payload;
            int available = mip_packet_create_field(&packet, field_desc, paylen, &payload);

            if( available < 0 )
            {
                num_errors++;
                fprintf(stderr, "Failed to create field of length %u\n", paylen+MIP_FIELD_HEADER_LENGTH);
                fprintf(stderr, "  max_len=%zu, available=%u\n", max_field_len, available);
                break;
            }

            // Random payload.
            for(unsigned int p=0; p<paylen; p++)
            {
                // Make it slightly more likely to produce nested packets.
                unsigned int r = rand();
                if((r & 0xFF00) < 0x200)
                    payload[p] = 0x75;
                else
                    payload[p] = r & 0xFF;
            }

            // Random chance of not adding another field.
            if( rand() % 5 == 0 )
                break;
        }

        mip_packet_finalize(&packet);

        bool potentially_nested = false;
        for(unsigned int b=1; b<mip_packet_total_length(&packet)-1; b++)
        {
            if(mip_packet_pointer(&packet)[b] == MIP_SYNC1 && mip_packet_pointer(&packet)[b+1] == MIP_SYNC2)
                potentially_nested = true;
        }

        //
        // Send this packet to the parser in small chunks.
        //

        const unsigned int MAX_CHUNKS = 5;

        const size_t packet_size = mip_packet_total_length(&packet);

        if(PRINT_DEBUG)
        {
            printf("Packet %u:", i + 1);
            print_packet(stdout, &packet);
        }

        // Keep track of offsets and timestamps for debug purposes.
        size_t offsets[MIP_PACKET_PAYLOAD_LENGTH_MAX / MIP_FIELD_HEADER_LENGTH] = {0};
        mip_timestamp timestamps[MIP_PACKET_PAYLOAD_LENGTH_MAX / MIP_FIELD_HEADER_LENGTH] = {0};
        unsigned int c = 0;

        mip_timestamp timestamp = start_time;
        size_t sent = 0;
        bool error = false;

        // Send all but the last chunk.
        while( sent < (packet_size-MIP_PACKET_LENGTH_MIN) )
        {
            const size_t count = rand() % (packet_size - sent);

            if(PRINT_DEBUG)
                printf("  send %zu @ time %zu\n", count, timestamp);
            mip_parser_parse(&parser, mip_packet_pointer(&packet)+sent, count, timestamp);

            sent += count;
            timestamps[c] = timestamp;
            offsets[c++] = sent;

            // Don't bump timestamp if no data sent to avoid screwing up the test code later.
            if( count > 0 )
                timestamp += (rand() % mip_parser_timeout(&parser));
        }

        // Final chunk
        const size_t extra = 0;

        const size_t count = packet_size - sent;

        if(PRINT_DEBUG)
             printf("  send %zu @ time %zu\n", count, timestamp);
        mip_parser_parse(&parser, mip_packet_pointer(&packet)+sent, count, timestamp);

        sent += count;
        timestamps[c] = timestamp;
        offsets[c++] = sent;

        bool timedout = (timestamps[c-1] - start_time) >= mip_parser_timeout(&parser);

        // If the previous packet timed out, there could be a nested header that also needs
        // to time out. Ensure that it times out and that the parser gets to the new packet.
        // This is similar to how real applications should behave; continually poll the parser
        // for packets even if no new data arrives.
        if(prev_timeout)
        {
            // Try to flush the parser repeatedly in case there were multiple nested packets
            // which each need to time out separately.
            for(unsigned int t=0; parser._buffered_length > packet_size && t < 5; t++)
            {
                timestamp += mip_parser_timeout(&parser);

                if(PRINT_DEBUG)
                    printf("  send 0 bytes @ time %zu (forced timeout).\n", timestamp);

                mip_parser_parse(&parser, NULL, 0, timestamp);
            }
        }

        if( timedout )
        {
            if( num_packets_parsed != last_parsed )
            {
                // Don't consider this a true error since the timeout handling can't be perfect.
                // The parser must not miss valid packets, but catching delayed packets is OK.
                // E.g. this data and timing results in receiving packet 9 because of a nested 75 65 in packet 8.
                // Packet 8 (131 bytes):
                //   75 65 25 7D 0D E4 04 F4 EC 0B B9 20 BA 86 C3 3E 05 70 65 33 B7 99 50 A3 E3 14 D3 D9 34 F7 5E A0
                //   F2 10 A8 F6 05 94 01 BE B4 BC 44 78 FA 49 69 E6 23 D0 1A DA 69 6A 7E 4C 7E 51 25 B3 48 84 53 3A
                //   94 FB 31 99 90 32 57 44 EE 9B BC E9 E5 25 CF 08 F5 E9 E2 5E 53 60 AA D2 B2 D0 85 FA 54 D8 35 E8
                //   D4 66 82 64 98 D9 A8 87 75_65_70_5A 8A 3F 62 80 29 44 DE 7C A5 89 4E 57 59 D3 51 AD AC 86 95 80
                //   EC 2E 35                ^^^^^^^^^^^ nested header (92-byte length)
                //   send 104 @ time 1275
                //   send 18 @ time 1332
                //   send 3 @ time 1368
                //   send 6 @ time 1382  (times out outer packet, now inner "packet" in buffer with 21+6 bytes)
                // Packet 9:
                //   75 65 9D 58 58 72 BB 22 FC E4 66 DA 61 0B 63 AF 62 BC 83 B4 69 2F 3A FF AF 27 16 93 AC 07 1F B8
                //   6D 11 34 2D 8D EF 4F 89 D4 B6 63 35 C1 C7 E4 24 83 67 D8 ED 96 12 EC 45 39 02 D8 E5 0A F8 9D 77
                //   09 D1 A5 96 C1 F4 1F 95 AA 82 CA 6C 49 AE 90 CD 16 68 BA AC 7A A6 F2 B4 A8 CA 99 B2 59 2A
                //   send 39 @ time 1382
                //   send 26 @ time 1452
                //   send 25 @ time 1532 (nested packet fails, start_time now 1532)
                //   send 4 @ time 1625 (1625-1532 < timeout so packet is received)

                //num_errors++;
                //error = true;  // Uncomment to log details
                //fprintf(stderr, "Note: Parser produced %u packet(s) but should have timed out.\n", num_packets_parsed-last_parsed);
            }
        }
        else if( num_packets_parsed != (last_parsed + 1) )
        {
            num_errors++;
            error = true;
            fprintf(stderr, "Parser produced %u packet(s) but expected exactly 1.\n", num_packets_parsed-last_parsed);
        }
        else if( parsed_packet_length != packet_size )
        {
            num_errors++;
            error = true;
            fprintf(stderr, "Parsed packet size is wrong (%zu bytes should be %zu)\n", parsed_packet_length, packet_size);
        }
        else
        {
            if(!prev_timeout && parsed_packet_timestamp != start_time)
            {
                num_errors++;
                error = true;
                fprintf(stderr, "Parsed packet has wrong timestamp %zu.\n", parsed_packet_timestamp);
            }
            if(memcmp(parsed_buffer, packet._buffer, packet_size) != 0)
            {
                num_errors++;
                error = true;
                fprintf(stderr, "Parsed packet data doesn't match.\n");
            }
        }

        start_time = timestamp;

        if( error )
        {
            fprintf(stderr, "  packet_size=%zu, last_count=%zu, extra=%zu, start_time=%zu\n", packet_size, count, extra, start_time);

            fprintf(stderr, "  Sent chunks:");
            for(unsigned int d=0; d<c; d++)
                fprintf(stderr, " %zu", offsets[d]);
            fputc('\n', stderr);

            fprintf(stderr, "  Sent timestamps:");
            for(unsigned int d=0; d<c; d++)
                fprintf(stderr, " %zu", timestamps[d]);
            fputc('\n', stderr);

            fprintf(stderr, "  Expected packet :");
            print_packet(stderr, &packet);

            fprintf(stderr, "  Parsed packet   :");
            print_packet(stderr, &parsed_packet);

            fprintf(stderr, prev_timeout ? "  Prev packet T.O.:" : "  Prev packet     :");
            print_packet(stderr, &prev_packet);

            fprintf(stderr, "  (packet %d / %d)\n\n", i+1, NUM_ITERATIONS);
        }

        if(num_errors > 10)
            break;
        else if((i+1) % 100000 == 0)
            printf("Progress: %u/%u iterations.\n", i+1, NUM_ITERATIONS);

        last_parsed = num_packets_parsed;
        prev_timeout = timedout;

        // Swap packets
        uint8_t* tmp = prev_packet._buffer;
        prev_packet._buffer = packet._buffer;
        packet._buffer = tmp;
    }

    printf("Completed %u/%u iterations with %u errors.\n", i, NUM_ITERATIONS, num_errors);

    return num_errors;
}
