
#include <mip/mip_dispatch.h>
#include <mip/mip_field.h>
#include <mip/mip_interface.h>
#include <mip/mip_result.h>
#include <mip/mip_types.h>
#include <mip/utils/serialization.h>

#include <mip/definitions/descriptors.h>
#include <mip/definitions/commands_base.h>
#include <mip/definitions/commands_3dm.h>
#include <mip/definitions/data_sensor.h>


#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <time.h>

#ifdef WIN32
#else
    #include <unistd.h>
    #include <fcntl.h>
    #include <sys/ioctl.h>
    #include <termios.h>
#endif

int port = -1;
uint8_t parse_buffer[1024];
struct mip_interface device;
struct mip_sensor_scaled_accel_data scaled_accel;

void handlePacket(void* unused, const struct mip_packet* packet, timestamp_type timestamp)
{
    (void)unused;

    printf("\nGot packet with descriptor set 0x%02X:", mip_packet_descriptor_set(packet));

    struct mip_field field;
    mip_field_init_empty(&field);
    while( mip_field_next_in_packet(&field, packet) )
    {
        printf(" %02X", mip_field_field_descriptor(&field));
    }
    printf("\n");
}

void handleAccel(void* user, const struct mip_field* field, timestamp_type timestamp)
{
    (void)user;
    struct mip_sensor_scaled_accel_data data;

    if(extract_mip_sensor_scaled_accel_data_from_field(field, &data))
    {
        // Compute delta from last packet (the extractor runs after this, so the data is one packet behind).
        float delta[3] = {
            data.scaled_accel[0] - scaled_accel.scaled_accel[0],
            data.scaled_accel[1] - scaled_accel.scaled_accel[1],
            data.scaled_accel[2] - scaled_accel.scaled_accel[2],
        };
        printf("Accel Data: %f, %f, %f (delta %f, %f, %f)\n", data.scaled_accel[0], data.scaled_accel[1], data.scaled_accel[2], delta[0], delta[1], delta[2]);
    }
}

void handleGyro(void* user, const struct mip_field* field, timestamp_type timestamp)
{
    (void)user;
    struct mip_sensor_scaled_gyro_data data;

    if(extract_mip_sensor_scaled_gyro_data_from_field(field, &data))
        printf("Gyro Data:  %f, %f, %f\n", data.scaled_gyro[0], data.scaled_gyro[1], data.scaled_gyro[2]);
}

void handleMag(void* user, const struct mip_field* field, timestamp_type timestamp)
{
    (void)user;
    struct mip_sensor_scaled_mag_data data;

    if(extract_mip_sensor_scaled_mag_data_from_field(field, &data))
        printf("Mag Data:   %f, %f, %f\n", data.scaled_mag[0], data.scaled_mag[1], data.scaled_mag[2]);
}


time_t startTime;

timestamp_type get_current_timestamp()
{
    time_t t;
    time(&t);

    double delta = difftime(t, startTime);

    return (timestamp_type)(delta * 1000);
}


bool mip_interface_user_update(struct mip_interface* device)
{
    timestamp_type now = get_current_timestamp();

    // Ensure commands can time out even if no data is received.
    mip_cmd_queue_update(mip_interface_cmd_queue(device), now);

    uint8_t buffer[256];

#ifdef WIN32
#else
    ssize_t bytes_read = read(port, buffer, sizeof(buffer));
    if( bytes_read < 0 && errno != EAGAIN )
        return false;
#endif

    mip_interface_receive_bytes(device, buffer, bytes_read, now);

    return true;
}


bool mip_interface_user_send_to_device(struct mip_interface* device, const uint8_t* data, size_t length)
{
    (void)device;

#ifdef WIN32
#else
    if( write(port, data, length) != length )
        return false;
#endif

    return true;
}


bool open_port(const char* name, uint32_t baudrate)
{
#ifdef WIN32
#else
    speed_t speed = 0;
    switch(baudrate)
    {
    case 115200: speed = B115200; break;
    case 230400: speed = B230400; break;
    case 460800: speed = B460800; break;
    case 921600: speed = B921600; break;
    default:
        fprintf(stderr, "Unsupported baud rate %d. Must be 115200, 230400, 460800, or 921600.", baudrate);
        return false;
    }

    port = open(name, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if(port == -1)
    {
        perror("Could not open port: ");
        close(port);
        return false;
    }

    struct termios options;
    if(tcgetattr(port, &options) == -1)
    {
        perror("Failed to query port settings: ");
        close(port);
        return false;
    }

    options.c_cflag |= (tcflag_t)(CLOCAL | CREAD);
    options.c_lflag &= (tcflag_t)~(ICANON | ECHO | ECHOE | ECHOK | ECHONL | ISIG | IEXTEN);

    options.c_oflag &= (tcflag_t)~(OPOST);
    options.c_iflag &= (tcflag_t)~(INLCR | IGNCR | ICRNL | IGNBRK);

    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);

    // Set no parity, 8 bits per byte, no flow control.
    options.c_cflag = (options.c_cflag & (tcflag_t)~(CSIZE|CSTOPB|INPCK|ISTRIP|PARENB|PARODD)) | CS8;
    options.c_iflag &= (tcflag_t) ~(IXON | IXOFF | IXANY);

    if( tcsetattr(port, TCSANOW, &options) != 0 )
        perror("Warning, failed to set port parameters: ");
#endif

    return true;
}

int usage(const char* argv0)
{
    fprintf(stderr, "Usage: %s <port> <baudrate>\n", argv0);
    return 1;
}

int main(int argc, const char* argv[])
{
    if(argc != 3)
        return usage(argv[0]);

    uint32_t baudrate = atoi(argv[2]);
    if( baudrate == 0 )
        return usage(argv[0]);

    if( !open_port(argv[1], baudrate) )
        return 1;

    mip_interface_init(&device, parse_buffer, sizeof(parse_buffer), mip_timeout_from_baudrate(baudrate), 1000);

    // Record program start time for use with difftime in getTimestamp().
    time(&startTime);

    enum mip_cmd_result result;

    // Get the base rate.
    volatile uint32_t now = clock();
    uint16_t base_rate;
    result = mip_3dm_get_base_rate(&device, MIP_SENSOR_DATA_DESC_SET, &base_rate);

    if( result != MIP_ACK_OK )
    {
        fprintf(stderr, "Failed to get base rate: %s (%d)\n", mip_cmd_result_to_string(result), result);
        goto done;
    }

    // Set the message format to stream at 100 Hz.

    const uint16_t sample_rate = 100; // Hz
    const uint16_t decimation = base_rate / sample_rate;

    const struct mip_descriptor_rate descriptors[3] = {
        { MIP_DATA_DESC_SENSOR_ACCEL_SCALED, decimation },
        { MIP_DATA_DESC_SENSOR_GYRO_SCALED,  decimation },
        { MIP_DATA_DESC_SENSOR_MAG_SCALED,   decimation },
    };

    result = mip_3dm_write_message_format(&device, MIP_SENSOR_DATA_DESC_SET, 3, descriptors);
    if( result == MIP_NACK_INVALID_PARAM )
    {
        // Failed to set message format - maybe this device doesn't have a magnetometer.
        // Try again without the last descriptor (scaled mag).
        result = mip_3dm_write_message_format(&device, MIP_SENSOR_DATA_DESC_SET, 2, descriptors);
    }
    if( result != MIP_ACK_OK )
    {
        fprintf(stderr, "Failed to set message format: %s (%d)\n", mip_cmd_result_to_string(result), result);
        goto done;
    }

    // Register some callbacks.
    struct mip_dispatch_handler packet_handler;
    struct mip_dispatch_handler data_handlers[4];
    mip_interface_register_packet_callback(&device, &packet_handler, MIP_DISPATCH_ANY_DATA_SET, false, &handlePacket, NULL);
    mip_interface_register_field_callback(&device, &data_handlers[0], MIP_SENSOR_DATA_DESC_SET, MIP_DATA_DESC_SENSOR_ACCEL_SCALED, &handleAccel, NULL);
    mip_interface_register_field_callback(&device, &data_handlers[1], MIP_SENSOR_DATA_DESC_SET, MIP_DATA_DESC_SENSOR_GYRO_SCALED , &handleGyro , NULL);
    mip_interface_register_field_callback(&device, &data_handlers[2], MIP_SENSOR_DATA_DESC_SET, MIP_DATA_DESC_SENSOR_MAG_SCALED  , &handleMag  , NULL);
    mip_interface_register_extractor(&device, &data_handlers[3], MIP_SENSOR_DATA_DESC_SET, MIP_DATA_DESC_SENSOR_ACCEL_SCALED, &extract_mip_sensor_scaled_accel_data_from_field, &scaled_accel);

    result = mip_base_resume(&device);
    if( result != MIP_ACK_OK )
    {
        fprintf(stderr, "Failed to resume device: %s (%d)\n", mip_cmd_result_to_string(result), result);
        goto done;
    }

    // Process data for 3 seconds.
    for(unsigned int i=0; i<30; i++)
    {
#ifdef WIN32
#else
        usleep(100000);
#endif
        mip_interface_update(&device);
    }

    result = mip_base_set_idle(&device);
    if( result != MIP_ACK_OK )
    {
        fprintf(stderr, "Failed to idle device: %s (%d)\n", mip_cmd_result_to_string(result), result);
        goto done;
    }

done:

#ifdef WIN32
#else
    close(port);
#endif

    return result == MIP_ACK_OK ? 0 : 2;
}
