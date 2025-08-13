#include "microstrain/connections/recording/recording_connection.h"

#include <microstrain/logging.h>

#include <assert.h>
#include <stdbool.h>
#include <string.h>

void recording_connection_init(recording_connection* _recording_connection)
{
    if (_recording_connection)
    {
        _recording_connection->receive_file          = NULL;
        _recording_connection->send_file             = NULL;
        _recording_connection->receive_bytes_written = 0;
        _recording_connection->send_bytes_written    = 0;
    }
    else
    {
        MICROSTRAIN_LOG_ERROR("Null recording connection for initialization\n");

        assert(false);
    }
}

void recording_connection_init_receive_stream(recording_connection* _recording_connection, FILE* _receive_stream)
{
    if (_recording_connection && _receive_stream)
    {
        // Make sure the stream is closed before initializing a new one
        recording_connection_close_receive_file(_recording_connection);

        MICROSTRAIN_LOG_DEBUG("Initializing the recording stream for receive bytes\n");

        _recording_connection->receive_file = _receive_stream;

        assert(_recording_connection->receive_file);

        _recording_connection->receive_bytes_written = 0;
    }
    else
    {
        if (!_recording_connection)
        {
            MICROSTRAIN_LOG_ERROR("Null recording connection for receive bytes stream initialization\n");
        }
        else
        {
            MICROSTRAIN_LOG_ERROR("Null stream for receive bytes stream initialization\n");
        }

        assert(false);
    }
}

void recording_connection_open_receive_file(recording_connection* _recording_connection, const char* _receive_file_name)
{
    if (_recording_connection && strlen(_receive_file_name) > 0)
    {
        MICROSTRAIN_LOG_DEBUG("Opening the recording file, %s, for received bytes\n", _receive_file_name);

        FILE* receive_stream = fopen(_receive_file_name, "wb");

        recording_connection_init_receive_stream(_recording_connection, receive_stream);
    }
    else
    {
        if (!_recording_connection)
        {
            MICROSTRAIN_LOG_ERROR("Null recording connection for receive bytes file opening\n");
        }
        else
        {
            MICROSTRAIN_LOG_ERROR("Empty or invalid receive bytes recording file name, %s\n", _receive_file_name);
        }

        assert(false);
    }
}

void recording_connection_close_receive_file(recording_connection* _recording_connection)
{
    if (_recording_connection)
    {
        if (_recording_connection->receive_file)
        {
            MICROSTRAIN_LOG_DEBUG("Closing the recording file for receive bytes\n");

            fclose(_recording_connection->receive_file);

            _recording_connection->receive_file = NULL;
        }
    }
    else
    {
        MICROSTRAIN_LOG_ERROR("Null recording connection for receive bytes file closing\n");

        assert(false);
    }
}

void recording_connection_write_received_bytes(recording_connection* _recording_connection,
    const uint8_t* _receive_buffer, const size_t _byte_count, size_t* _bytes_written_out)
{
    if (_recording_connection)
    {
        if (_recording_connection->receive_file)
        {
            const size_t bytes_written = fwrite(_receive_buffer, sizeof(_receive_buffer[0]), _byte_count,
                _recording_connection->receive_file);

            if (_bytes_written_out)
            {
                *_bytes_written_out = bytes_written;
            }

            if (bytes_written != _byte_count)
            {
                MICROSTRAIN_LOG_WARN(
                    "Received bytes size mismatch for recording connection. Written: %d, Requested: %d\n",
                    bytes_written, _byte_count);
            }

            _recording_connection->receive_bytes_written += bytes_written;
        }
    }
    else
    {
        MICROSTRAIN_LOG_ERROR("Null recording connection for received bytes file writing\n");

        assert(false);
    }
}

void recording_connection_init_send_stream(recording_connection* _recording_connection, FILE* _send_stream)
{
    if (_recording_connection && _send_stream)
    {
        // Make sure the stream is closed before initializing a new one
        recording_connection_close_send_file(_recording_connection);

        MICROSTRAIN_LOG_DEBUG("Initializing the recording stream for send bytes\n");

        _recording_connection->send_file = _send_stream;

        assert(_recording_connection->send_file);

        _recording_connection->send_bytes_written = 0;
    }
    else
    {
        if (!_recording_connection)
        {
            MICROSTRAIN_LOG_ERROR("Null recording connection for send bytes stream initialization\n");
        }
        else
        {
            MICROSTRAIN_LOG_ERROR("Null stream for send bytes stream initialization\n");
        }

        assert(false);
    }
}

void recording_connection_open_send_file(recording_connection* _recording_connection, const char* _send_file_name)
{
    if (_recording_connection && strlen(_send_file_name) > 0)
    {
        MICROSTRAIN_LOG_DEBUG("Opening the recording file, %s, for send bytes\n", _send_file_name);

        FILE* send_stream = fopen(_send_file_name, "wb");

        recording_connection_init_send_stream(_recording_connection, send_stream);
    }
    else
    {
        if (!_recording_connection)
        {
            MICROSTRAIN_LOG_ERROR("Null recording connection for send bytes file opening\n");
        }
        else
        {
            MICROSTRAIN_LOG_ERROR("Empty or invalid send bytes recording file name, %s\n", _send_file_name);
        }

        assert(false);
    }
}

void recording_connection_close_send_file(recording_connection* _recording_connection)
{
    if (_recording_connection)
    {
        if (_recording_connection->send_file)
        {
            MICROSTRAIN_LOG_DEBUG("Closing the recording file for send bytes\n");

            fclose(_recording_connection->send_file);

            _recording_connection->send_file = NULL;
        }
    }
    else
    {
        MICROSTRAIN_LOG_ERROR("Null recording connection for send bytes file closing\n");

        assert(false);
    }
}

void recording_connection_write_sent_bytes(recording_connection* _recording_connection, const uint8_t* _send_buffer,
    const size_t _byte_count, size_t* _bytes_written_out)
{
    if (_recording_connection)
    {
        if (_recording_connection->send_file)
        {
            const size_t bytes_written = fwrite(_send_buffer, sizeof(_send_buffer[0]), _byte_count,
                _recording_connection->send_file);

            if (_bytes_written_out)
            {
                *_bytes_written_out = bytes_written;
            }

            if (bytes_written != _byte_count)
            {
                MICROSTRAIN_LOG_WARN("Sent bytes size mismatch for recording connection. Written: %d, Requested: %d\n",
                    bytes_written, _byte_count);
            }

            _recording_connection->send_bytes_written += bytes_written;
        }
    }
    else
    {
        MICROSTRAIN_LOG_ERROR("Null recording connection for send bytes file writing\n");

        assert(false);
    }
}

void recording_connection_init_streams(recording_connection* _recording_connection, FILE* _receive_stream,
    FILE* _send_stream)
{
    if (_recording_connection)
    {
        if (_receive_stream)
        {
            recording_connection_init_receive_stream(_recording_connection, _receive_stream);
        }

        if (_send_stream)
        {
            recording_connection_init_send_stream(_recording_connection, _receive_stream);
        }

        if (!_receive_stream && !_send_stream)
        {
            MICROSTRAIN_LOG_ERROR("Null streams for stream initialization\n");

            assert(false);
        }
    }
    else
    {
        MICROSTRAIN_LOG_ERROR("Null recording connection for stream initialization\n");

        assert(false);
    }
}

void recording_connection_open_files(recording_connection* _recording_connection, const char* _receive_file_name,
    const char* _send_file_name)
{
    if (_recording_connection)
    {
        const bool receive_file_exists = strlen(_receive_file_name) > 0;

        if (receive_file_exists)
        {
            recording_connection_open_receive_file(_recording_connection, _receive_file_name);
        }

        const bool send_file_exists = strlen(_send_file_name) > 0;

        if (send_file_exists)
        {
            recording_connection_open_send_file(_recording_connection, _send_file_name);
        }

        if (!receive_file_exists && !send_file_exists)
        {
            MICROSTRAIN_LOG_ERROR("Empty or invalid recording file names, Receive: %s    Send: %s\n",
                _receive_file_name, _send_file_name);

            assert(false);
        }
    }
    else
    {
        MICROSTRAIN_LOG_ERROR("Null recording connection for file opening\n");

        assert(false);
    }
}

void recording_connection_close_files(recording_connection* _recording_connection)
{
    if (_recording_connection)
    {
        if (_recording_connection->receive_file)
        {
            recording_connection_close_receive_file(_recording_connection);
        }

        if (_recording_connection->send_file)
        {
            recording_connection_close_send_file(_recording_connection);
        }

        if (!_recording_connection->receive_file && !_recording_connection->send_file)
        {
            MICROSTRAIN_LOG_ERROR("Cannot close nonexistent recording files\n");

            assert(false);
        }
    }
    else
    {
        MICROSTRAIN_LOG_ERROR("Null recording connection for send bytes file closing\n");

        assert(false);
    }
}

bool recording_connection_receive_recording_enabled(const recording_connection* _recording_connection)
{
    if (_recording_connection)
    {
        return _recording_connection->receive_file != NULL && ferror(_recording_connection->receive_file) == 0;
    }

    MICROSTRAIN_LOG_ERROR("Cannot check if receive recording is enabled on a NULL connection\n");

    assert(false);

    return false;
}

bool recording_connection_send_recording_enabled(const recording_connection* _recording_connection)
{
    if (_recording_connection)
    {
        return _recording_connection->send_file != NULL && ferror(_recording_connection->send_file) == 0;
    }

    MICROSTRAIN_LOG_ERROR("Cannot check if send recording is enabled on a NULL connection\n");

    assert(false);

    return false;
}

bool recording_connection_enabled(const recording_connection* _recording_connection)
{
    if (_recording_connection)
    {
        return recording_connection_receive_recording_enabled(_recording_connection) ||
            recording_connection_send_recording_enabled(_recording_connection);
    }

    MICROSTRAIN_LOG_ERROR("Cannot check if recording is enabled on a NULL connection\n");

    assert(false);

    return false;
}
