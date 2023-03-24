
#include <mip/mip.hpp>
#include <mip/mip_device.hpp>

#include <mip/definitions/commands_base.hpp>
#include <mip/definitions/commands_filter.hpp>

#include <stdexcept>
#include <vector>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <deque>
#include <cstring>
#include <stdio.h>


class BufferedPacket : public mip::Packet
{
public:
    BufferedPacket(size_t length, uint8_t descriptorSet) : mip::Packet(new uint8_t[length], length, descriptorSet) {}
    BufferedPacket(const uint8_t* data, size_t length) : mip::Packet(new uint8_t[length], length) { std::memcpy(_buffer, data, length); }

    ~BufferedPacket() { delete [] _buffer; _buffer = nullptr; }

    BufferedPacket(BufferedPacket&& other) : mip::Packet(other) { other._buffer = nullptr; other._buffer_length = 0; }
    BufferedPacket& operator=(BufferedPacket& other) { _buffer = other._buffer; _buffer_length = other._buffer_length; other._buffer = nullptr; other._buffer_length = 0; return *this; }

    BufferedPacket(const BufferedPacket&) = delete;
    BufferedPacket& operator=(const BufferedPacket&) = delete;
};


class FakeDevice
{
public:
    bool sendToDevice(const uint8_t* data, size_t length);
    bool recvFromDevice(uint8_t* buffer, size_t maxLength, mip::Timeout timeout, size_t* length_out, mip::Timestamp* time_out);

    void start() { mDeviceThread = std::thread(&FakeDevice::run, this); }
    void stop()  { mStop = true; mCmdReady.notify_all(); mDeviceThread.join(); }

    size_t numErrors() const { return mNumErrors; }

private:
    void run();
    void reply(const mip::Packet& packet);

private:
    size_t mNumErrors = 0;
    bool   mStop = false;

    std::deque<BufferedPacket> mCommands;
    std::deque<BufferedPacket> mReplies;
    size_t                     mReplyIndex = 0;

    std::thread             mDeviceThread;
    std::condition_variable mCmdReady;
    std::mutex              mCmdMutex;
    std::mutex              mReplyMutex;
};

bool FakeDevice::sendToDevice(const uint8_t* data, size_t length)
{
    if( length < mip::PACKET_LENGTH_MIN )
    {
        mNumErrors++;
        std::fprintf(stderr, "Mip packet too short! Got %ld bytes.\n", length);
        return false;
    }
    if( data[mip::C::MIP_INDEX_SYNC1] != 0x75 || data[mip::C::MIP_INDEX_SYNC2] != 0x65 || (data[mip::C::MIP_INDEX_LENGTH] + mip::PACKET_LENGTH_MIN) != length )
    {
        mNumErrors++;
        std::fprintf(stderr, "Invalid mip packet (length=%ld, plen=%d).\n", length, data[mip::C::MIP_INDEX_LENGTH]);
        return false;
    }

    try
    {
        std::lock_guard<std::mutex> lock(mCmdMutex);

        mCommands.push_back({data, length});

        mCmdReady.notify_all();
    }
    catch(std::exception& ex)
    {
        std::fprintf(stderr, "Failed to buffer packet: %s\n", ex.what());
        mNumErrors++;
        return false;
    }

    return true;
}

bool FakeDevice::recvFromDevice(uint8_t* buffer, size_t maxLength, mip::Timeout timeout, size_t* length_out, mip::Timestamp* time_out)
{
    *time_out = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();

    *length_out = 0;

    if( mReplies.empty() )
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(timeout));
        if( mReplies.empty() )
            return true;
    }

    while(maxLength > 0 && !mReplies.empty())
    {
        const size_t numBytes = std::min<size_t>(maxLength, mReplies.front().totalLength()-mReplyIndex);

        maxLength -= numBytes;
        *length_out += numBytes;

        std::memcpy(buffer, mReplies.front().pointer()+mReplyIndex, numBytes);

        mReplyIndex += numBytes;
        if( mReplyIndex >= mReplies.front().totalLength() )
        {
            std::lock_guard<std::mutex> lock(mReplyMutex);

            mReplies.pop_front();
            mReplyIndex = 0;
        }
    }

    return true;
}

void FakeDevice::run()
{
    std::unique_lock<std::mutex> lock(mReplyMutex);

    while(true)
    {
        mCmdReady.wait(lock);

        if( mStop )
            break;

        if( mCommands.empty() )
            continue;

        reply(mCommands.front());
        mCommands.pop_front();
    }
}

void FakeDevice::reply(const mip::Packet& packet)
{
    BufferedPacket tmp(mip::PACKET_LENGTH_MIN + 4, packet.descriptorSet());

    mip::Field field = packet.firstField();

    uint8_t* payload;
    tmp.allocField(0xF1, 2, &payload);
    payload[0] = field.fieldDescriptor();
    payload[1] = 0x00;
    tmp.finalize();

    mReplies.push_back(std::move(tmp));
}


uint8_t buffer[1024];
mip::DeviceInterface device(buffer, sizeof(buffer), 100, 100);
FakeDevice fake;


class CommandThread
{
public:
    void start() { mThread = std::thread(&CommandThread::run, this); }
    void stop() { mThread.join(); }

    const std::vector<mip::CmdResult>& results() const { return mResults; }

private:
    void run()
    {
        for(unsigned int i=0; i<10; i++)
        {
            mResults.push_back( mip::commands_base::ping(device) );
        }
    }

private:
    std::thread mThread;
    std::vector<mip::CmdResult> mResults;
};



int main(int argc, const char* argv[])
{
    device.setCallbacks<FakeDevice, &FakeDevice::sendToDevice, &FakeDevice::recvFromDevice>(&fake);

    fake.start();

    try
    {
        unsigned int N = 10;
        std::printf("Single-threaded pings...\n");
        for(unsigned int i=0; i<N; i++)
        {
            mip::CmdResult result = mip::commands_base::ping(device);
            std::printf("Result (%2d/%2d): %d (%s)\n", i+1, N, result.value, result.name());
        }

        unsigned int M = 2;
        std::printf("Running %d threads...\n", M);

        std::vector<CommandThread> threads(M);
        for(auto& thread : threads)
            thread.start();

        for(auto& thread : threads)
            thread.stop();

        for(unsigned int i=0; i<M; i++)
        {
            const std::vector<mip::CmdResult> results = threads[i].results();

            std::printf("Thread %d/%d:\n", i+1, M);
            for(unsigned int j=0; j<results.size(); j++)
            {
                std::printf("  Result %2d/%2d: %d (%s)\n", j+1, int(results.size()), results[j].value, results[j].name());
            }
        }
    }
    catch(const std::exception& ex)
    {
        std::fprintf(stderr, "Fatal error: %s\n", ex.what());
    }

    fake.stop();

    return 0;
}