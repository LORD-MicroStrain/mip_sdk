#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <iomanip>

#include <microstrain_test.hpp>
#include <mip/mip.hpp>

using namespace mip;
using namespace mip::C;

uint8_t packetBuffer[PACKET_LENGTH_MAX];
uint8_t parseBuffer[1024];

FieldView fields[(unsigned int)MIP_PACKET_PAYLOAD_LENGTH_MAX / (unsigned int)MIP_FIELD_LENGTH_MIN];
unsigned int numFields = 0;

bool packetCallback(void*, const PacketView *parsedPacket, Timestamp timestamp)
{
    (void)timestamp;

    unsigned int numParsedFields = 0;

    for(FieldView field : *parsedPacket)
    {
        // TODO: Figure out solution for INFO calls
        INFO("  From field " << numParsedFields << "/" << numFields);
        INFO("  Descriptor set: " << std::fixed << std::setprecision(2) << field.descriptorSet() << "/" << fields[numParsedFields].descriptorSet());
        INFO("  Field Descriptor: " << std::fixed << std::setprecision(2) << field.fieldDescriptor() << "/" << fields[numParsedFields].fieldDescriptor());
        INFO("  Payload Length: " << std::fixed << std::setprecision(2) << field.payloadLength() << "/" << fields[numParsedFields].payloadLength());

        INFO("Descriptor set does not match.");
        FAIL_IF_NOT_EQUAL(field.descriptorSet(), fields[numParsedFields].descriptorSet());

        INFO("Field descriptor does not match.");
        FAIL_IF_NOT_EQUAL(field.fieldDescriptor(), fields[numParsedFields].fieldDescriptor());

        INFO("Payload length does not match.");
        FAIL_IF_NOT_EQUAL(field.payloadLength(), fields[numParsedFields].payloadLength());

        INFO("Payloads do not match.");
        int result = std::memcmp(
            field.payload().data(),
            fields[numParsedFields].payload().data(),
            std::min(field.payloadLength(), fields[numParsedFields].payloadLength())
        );
        FAIL_IF_NOT_EQUAL(result, 0);

        numParsedFields++;
    }

    INFO("Field count mismatch: " << numParsedFields << " != " << numFields);
    FAIL_IF_NOT_EQUAL(numParsedFields, numFields);

    return true;
}

TEST("Packet Builder", "Packets can be built and parsed correctly")
{
    Parser parser(packetCallback, nullptr, MIP_PARSER_DEFAULT_TIMEOUT_MS);

    constexpr unsigned int NUM_ITERATIONS = 1000000;

    for(unsigned int i=0; i<NUM_ITERATIONS; i++)
    {
        PacketView packet(packetBuffer, sizeof(packetBuffer), 0x80);

        for(numFields = 0; ; numFields++)
        {
            const uint8_t fieldDescriptor = (rand() % 255) + 1;
            const uint8_t payloadLength = (rand() % MIP_FIELD_PAYLOAD_LENGTH_MAX) + 1;

            Serializer payload = packet.createField(fieldDescriptor, payloadLength);
            if(!payload.hasRemaining())
                break;

            for(unsigned int p=0; p<payloadLength; p++)
                payload.insert<uint8_t>(rand() & 0xFF);

            INFO("Field " << numFields << " did not have the right size (wrote " << payload.offset() << ", expected " << payloadLength << ", max " << payload.capacity() << ").");
            FAIL_IF_NOT_TRUE(payload.isFinished());

            fields[numFields] = FieldView(packet.descriptorSet(), fieldDescriptor, payload.basePointer(), payloadLength);
        }

        packet.finalize();

        parser.parse(packet.pointer(), packet.totalLength(), 0);
    }
}
