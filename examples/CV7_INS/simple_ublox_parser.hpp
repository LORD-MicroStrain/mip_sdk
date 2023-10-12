//
// Created by davidrobbins on 10/12/23.
//

#ifndef MIP_SDK_SIMPLE_UBLOX_PARSER_HPP
#define MIP_SDK_SIMPLE_UBLOX_PARSER_HPP

#include <vector>
#include <deque>
#include <cstdint>
#include <cstring>
#include <functional>

const int HEADER_SIZE = 6;
const int CHECKSUM_SIZE = 2;

namespace mip::ublox
{

    bool verify_checksum(const std::vector<uint8_t>& packet)
    {
        uint8_t ck_a, ck_b;

        ck_a = 0;
        ck_b = 0;

        int num_bytes = packet.size();
        int num_bytes_without_checksum = num_bytes-2;

        for (int i = 2; i < num_bytes_without_checksum; i++) {
            ck_a += packet[i];
            ck_b += ck_a;
        }

        if (ck_a == packet[num_bytes - 2] && ck_b == packet[num_bytes - 1])
            return true;

        return false;
    }


    class UbloxMessageParser
    {
    public:

        UbloxMessageParser(std::function<void (std::vector<uint8_t>)> packet_callback) : _packet_callback(packet_callback)
        {}

        void parse_bytes(uint8_t* buffer, size_t num_input_bytes)
        {
            // Copy into parser buffer
            for (size_t i = 0; i<num_input_bytes; i++)
            {
                _buffer.emplace_back(buffer[i]);
            }

            // Wait for header bytes
            while (_buffer.size() >= 2)
            {
                if (header_found())
                    break;

                _buffer.pop_front();
            }

            // Check if header is valid
            if (!header_found())
                return;

            // Check if buffer has full message header
            if (_buffer.size() < 6)
                return;

            // Get message length
            uint8_t payload_length_bytes[2] = {_buffer[4], _buffer[5]};
            uint16_t payload_length;
            memcpy(&payload_length, payload_length_bytes, sizeof(uint16_t));

            int total_message_length = HEADER_SIZE + payload_length + CHECKSUM_SIZE;

            // Check if buffer contains full packet size
            if (_buffer.size() < total_message_length)
                return;

            // Extract packet
            std::vector<uint8_t> packet(total_message_length);
            for (int i = 0; i<total_message_length; i++)
                packet[i] = _buffer[i];

            // Validate checksum
            if (verify_checksum(packet))
            {
                _packet_callback(packet);
                for (int i = 0; i<total_message_length; i++)
                    _buffer.pop_front();
            }
            else
                _buffer.pop_front();

        }

        bool header_found()
        {
            if (_buffer.size() < 2)
                return false;

            return (_buffer[0] == 0xB5) && (_buffer[1] == 0x62);
        }

    protected:

        std::function<void (std::vector<uint8_t>)> _packet_callback;

        std::deque<uint8_t> _buffer;
    };
}


#endif //MIP_SDK_SIMPLE_UBLOX_PARSER_HPP
