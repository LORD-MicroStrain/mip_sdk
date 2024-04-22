
#include "mip/mip.hpp"

#include <vector>
#include <chrono>
#include <numeric>

#include <cstdio>

const uint8_t PING_PACKET[] = {0x75, 0x65, 0x01, 0x02, 0x02, 0x01, 0xE0, 0xC6};

const uint8_t DATA_PACKET[] = {
    0x75,0x65,0x82,0xc1,0x0e,0xd3,0x40,0x8c,0x84,0xef,0x9d,0xb2,0x2d,0x0f,0x00,0x00,
    0x00,0x00,0x0a,0xd5,0x00,0x00,0x00,0xd4,0x7c,0x36,0x4c,0x40,0x10,0x05,0x7f,0xff,
    0xff,0xf8,0x7f,0xc0,0x00,0x00,0x7f,0xff,0xff,0xf8,0x00,0x01,0x10,0x06,0x7f,0xc0,
    0x00,0x00,0x7f,0xc0,0x00,0x00,0x7f,0xc0,0x00,0x00,0x00,0x01,0x10,0x07,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x08,0x10,0x00,0x02,
    0x00,0x00,0x00,0x00,0x1c,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x10,0x02,0x7f,0xc0,0x00,0x00,0x7f,0xc0,0x00,0x00,0x7f,0xc0,0x00,0x00,0x00,0x00,
    0x1c,0x42,0x7f,0xf8,0x00,0x00,0x00,0x00,0x00,0x00,0x7f,0xf8,0x00,0x00,0x00,0x00,
    0x00,0x00,0x7f,0xf8,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x09,0x46,0x44,0x64,
    0x26,0x87,0x00,0x04,0x01,0x10,0x08,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x10,0x09,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x87,0x56,  // 199 bytes
};

const char NMEA_SENTENCE[] = "$GPGGA,123456.7,1234.5678,N,1234.5678,W,1,5,1.2,12.345,M,12.345,M,*4F";


struct Test
{
    const char* name = nullptr;
    unsigned int num_iterations = 1;
    size_t num_packets = 0;
    std::vector<uint8_t> data;
};


Test generate_pings()
{
    Test test;

    test.name = "Pings";
    test.num_packets = 200000;
    test.num_iterations = 100;

    test.data.resize(sizeof(PING_PACKET)*test.num_packets);

    for(unsigned int i=0; i<test.num_packets; i++)
        std::copy(std::begin(PING_PACKET), std::end(PING_PACKET), test.data.data()+i*sizeof(PING_PACKET));

    return test;
}

Test generate_long_pkts()
{
    Test test;

    test.name = "Long Pkts";
    test.num_packets = 100'000;
    test.num_iterations = 100;

    test.data.resize(sizeof(DATA_PACKET)*test.num_packets);

    for(unsigned int i=0; i<test.num_packets; i++)
        std::copy(std::begin(DATA_PACKET),  std::end(DATA_PACKET), test.data.data()+i*sizeof(DATA_PACKET));

    return test;
}

Test generate_interleaved()
{
    Test test;

    test.name = "InterleavedNMEA";
    test.num_packets = 100'000;
    test.num_iterations = 100;

    const size_t INTERVAL = sizeof(DATA_PACKET) + sizeof(NMEA_SENTENCE);
    test.data.resize(INTERVAL*test.num_packets);

    for(unsigned int i=0; i<test.num_packets; i++)
    {
        std::copy(std::begin(DATA_PACKET),   std::end(DATA_PACKET),   test.data.data() + i * INTERVAL);
        std::copy(std::begin(NMEA_SENTENCE), std::end(NMEA_SENTENCE), test.data.data() + i * INTERVAL + sizeof(DATA_PACKET));
    }

    return test;
}

uint8_t parse_buffer[1024];

void run_parser(const Test& test)
{
    struct Stats
    {
        size_t num_bytes = 0;
        size_t num_pkts  = 0;
    };

    Stats stats;
    auto callback = +[](void* v, const mip::PacketRef* p, mip::Timestamp)
    {
        Stats& s = *static_cast<Stats*>(v);
        s.num_bytes += p->totalLength();
        s.num_pkts++;
        return true;
    };
    mip::Parser parser(parse_buffer, sizeof(parse_buffer), callback, &stats, MIPPARSER_DEFAULT_TIMEOUT_MS);

    std::vector<float> timing(test.num_iterations, 0);

    assert(test.num_iterations > 0);
    for(unsigned int i=0; i<test.num_iterations; i++)
    {
        parser.reset();

        auto start = std::chrono::steady_clock::now();
        parser.parse(test.data.data(), test.data.size(), 0);
        auto stop = std::chrono::steady_clock::now();

        std::chrono::duration<float> duration = stop - start;

        timing[i] = duration.count();
    }

    const float total_time = std::accumulate(timing.begin(), timing.end(), 0.0f);
    const float avg_time = total_time / std::size(timing);
    const float max_time = *std::max_element(timing.begin(), timing.end());
    const float bytes_per_sec   = test.data.size()*test.num_iterations / total_time;
    const float packets_per_sec = test.num_packets*test.num_iterations / total_time;

    std::printf(
        "\nTest %s: %u iterations\n"
        "  Bytes/iter:    %.3f MB\n"
        "  Pkts/iter:     %.3f kPkt\n"
        "  Total Packets: %.f / %.f%s MPkt\n"
        "  Average Time:  %f ms\n"
        "  Maximum Time:  %f ms\n"
        "  Bytes/s:       %.3f MB/s\n"
        "  Pkts/s:        %.3f MPkt/s\n",
        test.name, test.num_iterations,
        float(test.data.size())/1e6f,
        float(test.num_packets)/1e3f,
        float(stats.num_pkts)/1e6f, float(test.num_packets*test.num_iterations)/1e6f, (stats.num_pkts != test.num_packets*test.num_iterations ? "***" : ""),
        avg_time*1e3f,
        max_time*1e3f,
        bytes_per_sec/1e6f,
        packets_per_sec/1e6f
    );
}


int main(int argc, const char* argv[])
{
    run_parser(generate_long_pkts());
    run_parser(generate_pings());
    run_parser(generate_interleaved());

    return 0;
}
