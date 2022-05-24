
#include <mip/mip_parser.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>


uint8_t parseBuffer[1024];
uint8_t inputBuffer[1024];
uint8_t checkBuffer[MIP_PACKET_LENGTH_MAX];

struct MipParsingState parser;

unsigned int numErrors = 0;
size_t bytesRead = 0;
size_t bytesParsed = 0;

bool handle_packet(void* p, const struct MipPacket* packet, Timestamp t)
{
    (void)t;

    FILE* infile2 = (FILE*)p;

    size_t length = MipPacket_totalLength(packet);
    if( length > MIP_PACKET_LENGTH_MAX )
    {
        numErrors++;
        fprintf(stderr, "Packet with length too long (%ld)\n", length);
        return false;
    }
    // size_t written = fwrite(MipPacket_buffer(packet), 1, length, outfile);
    // return written == length;

    bytesParsed += length;

    size_t read = fread(checkBuffer, 1, length, infile2);

    if( read != length )
    {
        numErrors++;
        fprintf(stderr, "Failed to read from input file (2).\n");
        return false;
    }

    const uint8_t* packetBuffer = MipPacket_pointer(packet);

    // printf("Packet: ");
    // for(size_t i=0; i<length; i++)
    //     printf(" %02X", packetBuffer[i]);
    // fputc('\n', stdout);

    bool good = memcmp(checkBuffer, packetBuffer, length) == 0;

    if( !good )
    {
        fprintf(stderr, "Packet does not match next sequence in input file:\n");

        fputs("Packet:    ", stderr);
        for(size_t i=0; i<length; i++)
            fprintf(stderr, " %02X", packetBuffer[i]);

        fputs("Reference: ", stderr);
        for(size_t i=0; i<length; i++)
            fprintf(stderr, " %02X", checkBuffer[i]);

        fputc('\n', stderr);
    }

    return good;
}


int main(int argc, const char* argv[])
{
    if( argc < 2 )
    {
        fprintf(stderr, "Usage: %s <input-file>\n", argv[0]);
        return 1;
    }

    const char* inputFilename = argv[1];

    FILE* infile = fopen(inputFilename, "rb");
    if( !infile )
    {
        fprintf(stderr, "Error: could not open input file '%s'.", inputFilename);
        return 1;
    }

    FILE* infile2 = fopen(inputFilename, "rb");
    if( !infile2 )
    {
        fclose(infile);
        fprintf(stderr, "Error: could not open input file '%s' (2).", inputFilename);
        return 1;
    }

    srand(0);

    MipParser_init(&parser, parseBuffer, sizeof(parseBuffer), &handle_packet, infile2, MIPPARSER_DEFAULT_TIMEOUT_MS);

    do
    {
        const size_t numToRead = rand() % sizeof(inputBuffer);

        const size_t numRead = fread(inputBuffer, 1, numToRead, infile);
        bytesRead += numRead;

        MipParser_parse(&parser, inputBuffer, numRead, 0, MIPPARSER_UNLIMITED_PACKETS);

        // End of file (or error)
        if( numRead != numToRead )
            break;

    } while(numErrors == 0);

    fclose(infile);

    if( bytesParsed != bytesRead )
    {
        numErrors++;
        fprintf(stderr, "Read %ld bytes but only parsed %ld bytes (delta %ld).\n", bytesRead, bytesParsed, bytesRead-bytesParsed);
    }

    fclose(infile2);

    return numErrors;
}
