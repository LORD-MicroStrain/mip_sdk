
#include <mip/mip_packet.h>
#include <mip/mip_parser.h>

#include <stdio.h>
#include <stdlib.h>


uint8_t parseBuffer[1024];
struct MipParsingState parser;

unsigned int numErrors = 0;

unsigned int numPacketsParsed = 0;
size_t parsedPacketLength = 0;
Timestamp parsedPacketTimestamp = 0;


void printPacket(FILE* out, const struct MipPacket* packet)
{
    size_t size = MipPacket_totalLength(packet);
    const uint8_t* ptr = MipPacket_pointer(packet);

    for(size_t i=0; i<size; i++)
    {
        fprintf(out, " %02X", ptr[i]);
    }
    fputc('\n', out);
}


bool handlePacket(void* p, const struct MipPacket* packet, Timestamp timestamp)
{
    (void)p;

    numPacketsParsed++;
    parsedPacketLength = MipPacket_totalLength(packet);
    parsedPacketTimestamp = timestamp;

    return true;
}


int main(int argc, const char* argv[])
{
    MipParser_init(&parser, parseBuffer, sizeof(parseBuffer), &handlePacket, NULL, MIPPARSER_DEFAULT_TIMEOUT_MS);

    srand(0);

    const unsigned int NUM_ITERATIONS = 100;

    unsigned int lastParsed = 0;
    for(unsigned int i=0; i<NUM_ITERATIONS; i++)
    {
        uint8_t descSet = (rand() % 255) + 1;  // Random descriptor set.

        uint8_t buffer[MIP_PACKET_LENGTH_MAX];
        struct MipPacket packet;
        MipPacket_create(&packet, buffer, sizeof(buffer), descSet);

        for(unsigned int f=0; ; f++)
        {
            const size_t maxFieldLen = MipPacket_remainingSpace(&packet);
            if( maxFieldLen < MIP_FIELD_HEADER_LENGTH )
                break;

            const uint8_t maxPayload = maxFieldLen - MIP_FIELD_HEADER_LENGTH;

            const uint8_t paylen = (rand() % (maxPayload+1)) >> (rand() % 8);

            const uint8_t fieldDesc = (rand() % 255) + 1;  // Random field descriptor.

            uint8_t* payload;
            RemainingCount available = MipPacket_allocField(&packet, fieldDesc, paylen, &payload);

            if( available < 0 )
            {
                numErrors++;
                fprintf(stderr, "Failed to create field of length %d\n", paylen+MIP_FIELD_HEADER_LENGTH);
                fprintf(stderr, "  maxLen=%ld, available=%d\n", maxFieldLen, available);
                break;
            }

            // Random payload.
            for(unsigned int p=0; p<paylen; p++)
                payload[p] = rand() & 0xFF;

            // Random chance of not adding another field.
            if( rand() % 5 == 0 )
                break;
        }

        MipPacket_finalize(&packet);

        //
        // Send this packet to the parser in small chunks.
        //

        const unsigned int MAX_CHUNKS = 5;

        const size_t packetSize = MipPacket_totalLength(&packet);


        // Keep track of offsets and timestamps for debug purposes.
        size_t offsets[MIP_PACKET_PAYLOAD_LENGTH_MAX / MIP_FIELD_HEADER_LENGTH] = {0};
        Timestamp timestamps[MIP_PACKET_PAYLOAD_LENGTH_MAX / MIP_FIELD_HEADER_LENGTH] = {0};
        unsigned int c = 0;

        const Timestamp startTime = rand();
        Timestamp timestamp = startTime;
        size_t sent = 0;

        // Send all but the last chunk.
        while( sent < (packetSize-MIP_PACKET_LENGTH_MIN) )
        {
            const size_t count = rand() % (packetSize - sent);

            MipParser_parse(&parser, MipPacket_pointer(&packet)+sent, count, timestamp, MIPPARSER_UNLIMITED_PACKETS);

            sent += count;
            timestamps[c] = timestamp;
            offsets[c++] = sent;

            // Don't bump timestamp if no data sent to avoid screwing up the test code later.
            if( count > 0 )
                timestamp += (rand() % MipParser_timeout(&parser));
        }

        // Final chunk
        const size_t extra = 0;

        const size_t count = packetSize - sent;

        MipParser_parse(&parser, MipPacket_pointer(&packet)+sent, count, timestamp, MIPPARSER_UNLIMITED_PACKETS);

        sent += count;
        timestamps[c] = timestamp;
        offsets[c++] = sent;

        bool timedout = (timestamps[c-1] - startTime) > MipParser_timeout(&parser);

        bool error = false;

        if( timedout )
        {
            if( numPacketsParsed != lastParsed )
            {
                numErrors++;
                error = true;
                fprintf(stderr, "Parser produced %d packet(s) but should have timed out.\n", numPacketsParsed-lastParsed);
            }
        }
        else if( numPacketsParsed != (lastParsed + 1) )
        {
            numErrors++;
            error = true;
            fprintf(stderr, "Parser produced %d packet(s) but expected exactly 1.\n", numPacketsParsed-lastParsed);
        }
        else if( parsedPacketLength != packetSize )
        {
            numErrors++;
            error = true;
            fprintf(stderr, "Parsed packet size is wrong (%ld bytes)\n", parsedPacketLength);
        }
        else if( parsedPacketTimestamp != startTime )
        {
            numErrors++;
            error = true;
            fprintf(stderr, "Parsed packet has wrong timestamp %d\n", parsedPacketTimestamp);
        }
        lastParsed = numPacketsParsed;

        if( error )
        {
            fprintf(stderr, "  packetSize=%ld, lastCount=%ld, extra=%ld, startTime=%d\n", packetSize, count, extra, startTime);

            fprintf(stderr, "  Sent chunks:");
            for(unsigned int d=0; d<c; d++)
                fprintf(stderr, " %ld", offsets[d]);
            fputc('\n', stderr);

            fprintf(stderr, "  Sent timestamps:");
            for(unsigned int d=0; d<c; d++)
                fprintf(stderr, " %d", timestamps[d]);
            fputc('\n', stderr);

            fprintf(stderr, "  Expected packet:");
            printPacket(stderr, &packet);

            fprintf(stderr, "  (packet %d / %d)\n\n", i+1, NUM_ITERATIONS);
        }
    }

    return numErrors;
}
