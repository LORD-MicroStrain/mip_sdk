//
// Created by davidrobbins on 10/12/23.
//

#ifndef MIP_SDK_UBLOX_PARSER_H
#define MIP_SDK_UBLOX_PARSER_H

#include <vector>
#include <deque>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <cmath>

#include "mip/platform/serial_connection.hpp"

#define PVT_PAYLOAD_SIZE 92

const int HEADER_SIZE = 6;
const int CHECKSUM_SIZE = 2;

struct UBlox_PVT_Message {
    uint32_t iTOW;
    uint16_t utc_year;
    uint8_t utc_month;
    uint8_t utc_day;
    uint8_t utc_hour;
    uint8_t utc_minute;
    double utc_second;
    uint8_t lat_lon_valid_flag;
    double nano_second;
    uint8_t time_valid_flag;
    double utc_time_accuracy;
    double latitude;
    double longitude;
    uint8_t number_of_satellites;
    float ned_velocity_uncertainty[3];
    double height_above_ellipsoid;
    double horizontal_accuracy;
    double vertical_accuracy;
    float ned_velocity[3];
    float llh_uncertainty[3];
    double ground_speed;
    double heading_of_motion_2d;
    double heading_accuracy;
    double heading_of_vehicle;
    double magnetic_declination;
    double magnetic_declination_accuracy;
};

struct UBlox_Payload_Part {
    uint8_t start_index;
    uint8_t num_bytes;
};

constexpr UBlox_Payload_Part PAYLOAD_PART_ITOW = {0, 4};
constexpr UBlox_Payload_Part PAYLOAD_PART_YEAR = {4, 2};
constexpr UBlox_Payload_Part PAYLOAD_PART_MONTH = {6, 1};
constexpr UBlox_Payload_Part PAYLOAD_PART_DAY = {7, 1};
constexpr UBlox_Payload_Part PAYLOAD_PART_HOUR = {8, 1};
constexpr UBlox_Payload_Part PAYLOAD_PART_MIN = {9, 1};
constexpr UBlox_Payload_Part PAYLOAD_PART_SEC = {10, 1};
constexpr UBlox_Payload_Part PAYLOAD_PART_VALID = {11, 1};
constexpr UBlox_Payload_Part PAYLOAD_PART_T_ACC = {12, 4};
constexpr UBlox_Payload_Part PAYLOAD_PART_NANO_SEC = {16, 4};
constexpr UBlox_Payload_Part PAYLOAD_PART_FIX_TYPE = {20, 1};
constexpr UBlox_Payload_Part PAYLOAD_PART_FLAGS = {21, 1};
constexpr UBlox_Payload_Part PAYLOAD_PART_FLAGS_2 = {22, 1};
constexpr UBlox_Payload_Part PAYLOAD_PART_NUM_SV = {23, 1};
constexpr UBlox_Payload_Part PAYLOAD_PART_LON = {24, 4};
constexpr UBlox_Payload_Part PAYLOAD_PART_LAT = {28, 4};
constexpr UBlox_Payload_Part PAYLOAD_PART_HEIGHT = {32, 4};
constexpr UBlox_Payload_Part PAYLOAD_PART_H_MSL = {36, 4};
constexpr UBlox_Payload_Part PAYLOAD_PART_H_ACC = {40, 4};
constexpr UBlox_Payload_Part PAYLOAD_PART_V_ACC = {44, 4};
constexpr UBlox_Payload_Part PAYLOAD_PART_VEL_N = {48, 4};
constexpr UBlox_Payload_Part PAYLOAD_PART_VEL_E = {52, 4};
constexpr UBlox_Payload_Part PAYLOAD_PART_VEL_D = {56, 4};
constexpr UBlox_Payload_Part PAYLOAD_PART_G_SPEED = {60, 4};
constexpr UBlox_Payload_Part PAYLOAD_PART_HEAD_MOT= {64, 4};
constexpr UBlox_Payload_Part PAYLOAD_PART_S_ACC = {68, 4};
constexpr UBlox_Payload_Part PAYLOAD_PART_HEAD_ACC = {72, 4};
constexpr UBlox_Payload_Part PAYLOAD_PART_FLAGS_3 = {78, 4};
constexpr UBlox_Payload_Part PAYLOAD_PART_HEAD_VEH = {84, 4};
constexpr UBlox_Payload_Part PAYLOAD_PART_MAG_DEC= {88, 4};
constexpr UBlox_Payload_Part PAYLOAD_PART_MAG_ACC = {90, 4};

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

template<typename T>
T parse_bytes(T data_type, const uint8_t* payload_start, int data_start_index) {
    std::memcpy(&data_type, &payload_start[data_start_index], sizeof(data_type));
    return data_type;
}

template<typename T>
void convert_mm_to_m(T& value) {
    value = value / 1000.0;
}

template<typename T>
void convert_degrees_to_radians(T& degrees) {
    degrees = degrees * (M_PI / 180.0);
}

UBlox_PVT_Message extract_pvt_message(const uint8_t payload[PVT_PAYLOAD_SIZE])
{
    UBlox_PVT_Message ublox_message;

    int32_t four_bytes;
    int16_t two_bytes;

    ublox_message.iTOW = parse_bytes(four_bytes, payload, PAYLOAD_PART_ITOW.start_index);

    ublox_message.utc_year = parse_bytes(two_bytes, payload, PAYLOAD_PART_YEAR.start_index);

    ublox_message.utc_month = payload[PAYLOAD_PART_MONTH.start_index];

    ublox_message.utc_day = payload[PAYLOAD_PART_DAY.start_index];

    ublox_message.utc_hour = payload[PAYLOAD_PART_HOUR.start_index];

    ublox_message.utc_minute = payload[PAYLOAD_PART_MIN.start_index];

    ublox_message.utc_second = payload[PAYLOAD_PART_SEC.start_index];

    ublox_message.time_valid_flag = payload[PAYLOAD_PART_FLAGS.start_index];

    ublox_message.utc_time_accuracy = parse_bytes(four_bytes, payload, PAYLOAD_PART_T_ACC.start_index);

    ublox_message.nano_second = parse_bytes(four_bytes, payload, PAYLOAD_PART_NANO_SEC.start_index);

    ublox_message.longitude = parse_bytes(four_bytes, payload, PAYLOAD_PART_LON.start_index) * 1e-7;
    ublox_message.latitude = parse_bytes(four_bytes, payload, PAYLOAD_PART_LAT.start_index) * 1e-7;

    ublox_message.height_above_ellipsoid = parse_bytes(four_bytes, payload, PAYLOAD_PART_HEIGHT.start_index);
    convert_mm_to_m(ublox_message.height_above_ellipsoid);

    ublox_message.llh_uncertainty[0] = parse_bytes(four_bytes, payload, PAYLOAD_PART_V_ACC.start_index);
    convert_mm_to_m(ublox_message.llh_uncertainty[0]);

    ublox_message.llh_uncertainty[1] = parse_bytes(four_bytes, payload, PAYLOAD_PART_H_ACC.start_index);
    convert_mm_to_m(ublox_message.llh_uncertainty[1]);

    ublox_message.llh_uncertainty[2] = parse_bytes(four_bytes, payload, PAYLOAD_PART_V_ACC.start_index);
    convert_mm_to_m(ublox_message.llh_uncertainty[2]);

    ublox_message.ned_velocity[0] = parse_bytes(four_bytes, payload, PAYLOAD_PART_VEL_N.start_index);
    convert_mm_to_m(ublox_message.ned_velocity[0]);

    ublox_message.ned_velocity[1] = parse_bytes(four_bytes, payload, PAYLOAD_PART_VEL_E.start_index);
    convert_mm_to_m(ublox_message.ned_velocity[1]);

    ublox_message.ned_velocity[2] = parse_bytes(four_bytes, payload, PAYLOAD_PART_VEL_D.start_index);
    convert_mm_to_m(ublox_message.ned_velocity[2]);

    ublox_message.heading_of_motion_2d = parse_bytes(four_bytes, payload, PAYLOAD_PART_HEAD_MOT.start_index);

    ublox_message.heading_accuracy = parse_bytes(four_bytes, payload, PAYLOAD_PART_HEAD_ACC.start_index) * 1e-5;
    convert_degrees_to_radians(ublox_message.heading_accuracy);

    ublox_message.lat_lon_valid_flag = parse_bytes(four_bytes, payload, PAYLOAD_PART_FLAGS_3.start_index);

    ublox_message.heading_of_vehicle = parse_bytes(four_bytes, payload, PAYLOAD_PART_HEAD_VEH.start_index);
    convert_degrees_to_radians(ublox_message.heading_of_vehicle);

    return ublox_message;
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
            packet.emplace_back(_buffer[i]);

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


class UbloxDevice
{
public:

    UbloxDevice(std::unique_ptr<mip::Connection> connection) : _connection(std::move(connection)),
    _message_parser([this](const std::vector<uint8_t>& packet){ handle_packet(packet);})
    {

    }

    void handle_packet(const std::vector<uint8_t>& packet)
    {
        bool is_pvt_message = (packet[2] == 0x01) && (packet[3] == 0x07);
        if (!is_pvt_message)
            return;

        // Should never happen
        size_t expected_packet_size = HEADER_SIZE + PVT_PAYLOAD_SIZE + CHECKSUM_SIZE;
        if (packet.size() != expected_packet_size)
            return;

        // Extract message payload
        uint8_t payload_bytes[PVT_PAYLOAD_SIZE];
        for (int i = 0; i < PVT_PAYLOAD_SIZE; i++)
            payload_bytes[i] = packet[i + HEADER_SIZE];

        _current_message = extract_pvt_message(payload_bytes);
        _new_message_received = true;
    }

    std::pair<bool, UBlox_PVT_Message> update()
    {
        _new_message_received = false;

        uint8_t input_bytes[1024];
        size_t num_input_bytes;
        mip::Timestamp timestamp_out;
        _connection->recvFromDevice(input_bytes, 1024, 1, &num_input_bytes, &timestamp_out);

        _message_parser.parse_bytes(input_bytes, num_input_bytes);

        return {_new_message_received, _current_message};
    }

protected:

    std::unique_ptr<mip::Connection> _connection;
    UbloxMessageParser _message_parser;

    bool _new_message_received = false;
    UBlox_PVT_Message _current_message;
};

#endif //MIP_SDK_UBLOX_PARSER_H
