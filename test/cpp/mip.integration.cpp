#include <algorithm>
#include <cstdlib>
#include <iomanip>
#include <random>

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
        LOG_ON_FAIL("\tFrom field " << numParsedFields << "/" << numFields);
        LOG_ON_FAIL("\tDescriptor set: " << std::fixed << std::setprecision(2) << field.descriptorSet() << "/" << fields[numParsedFields].descriptorSet());
        LOG_ON_FAIL("\tField Descriptor: " << std::fixed << std::setprecision(2) << field.fieldDescriptor() << "/" << fields[numParsedFields].fieldDescriptor());
        LOG_ON_FAIL("\tPayload Length: " << std::fixed << std::setprecision(2) << field.payloadLength() << "/" << fields[numParsedFields].payloadLength());

        FAIL_AND_LOG_IF_NOT_EQUAL(field.descriptorSet(), fields[numParsedFields].descriptorSet(),
            "Descriptor set does not match.");

        FAIL_AND_LOG_IF_NOT_EQUAL(field.fieldDescriptor(), fields[numParsedFields].fieldDescriptor(),
            "Field descriptor does not match.");

        FAIL_AND_LOG_IF_NOT_EQUAL(field.payloadLength(), fields[numParsedFields].payloadLength(),
            "Payload length does not match.");

        int result = std::memcmp(
            field.payload().data(),
            fields[numParsedFields].payload().data(),
            std::min(field.payloadLength(), fields[numParsedFields].payloadLength())
        );
        FAIL_AND_LOG_IF_NOT_EQUAL(result, 0, "Payloads do not match.");

        numParsedFields++;
    }

    FAIL_AND_LOG_IF_NOT_EQUAL(numParsedFields, numFields,
        "Field count mismatch: " << numParsedFields << " != " << numFields);

    return true;
}

TEST("Packet Builder", "Packets can be built and parsed correctly")
{
    std::random_device random_device;
    std::mt19937 random_generator(random_device());
    std::uniform_int_distribution<int> field_descriptor_distribution(1, 255);
    std::uniform_int_distribution<int> payload_length_distribution(1, MIP_FIELD_PAYLOAD_LENGTH_MAX);
    std::uniform_int_distribution<int> rand_max_distribution(1, RAND_MAX);

    Parser parser(packetCallback, nullptr, MIP_PARSER_DEFAULT_TIMEOUT_MS);

    constexpr unsigned int NUM_ITERATIONS = 1000000;

    for(unsigned int i = 0; i < NUM_ITERATIONS; ++i)
    {
        PacketView packet(packetBuffer, sizeof(packetBuffer), 0x80);

        for(numFields = 0; ; numFields++)
        {
            const uint8_t fieldDescriptor = field_descriptor_distribution(random_generator);
            const uint8_t payloadLength = payload_length_distribution(random_generator);

            Serializer payload = packet.createField(fieldDescriptor, payloadLength);
            if(!payload.hasRemaining())
                break;

            for(unsigned int p=0; p<payloadLength; p++)
                payload.insert<uint8_t>(rand_max_distribution(random_generator) & 0xFF);

            INFO("Field " << numFields << " did not have the right size (wrote " << payload.offset() << ", expected " << payloadLength << ", max " << payload.capacity() << ").");
            FAIL_IF_NOT_TRUE(payload.isFinished());

            fields[numFields] = FieldView(packet.descriptorSet(), fieldDescriptor, payload.basePointer(), payloadLength);
        }

        packet.finalize();

        parser.parse(packet.pointer(), packet.totalLength(), 0);
    }
}
