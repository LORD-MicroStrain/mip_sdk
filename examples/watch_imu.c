
#include <mscl/mip/mip_dispatch.h>
#include <mscl/mip/mip_field.h>
#include <mscl/mip/mip_interface.h>
#include <mscl/mip/mip_result.h>
#include <mscl/types.h>

#include <mscl/mip/definitions/descriptors.h>
#include <mscl/mip/definitions/commands_base.h>
#include <mscl/mip/definitions/commands_3dm.h>
#include <mscl/mip/definitions/data_sensor.h>


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
uint8_t parseBuffer[1024];
struct MipInterfaceState device;


void handlePacket(void* unused, const struct MipPacket* packet, Timestamp timestamp)
{
    (void)unused;

    printf("\nGot packet with descriptor set 0x%02X:", MipPacket_descriptorSet(packet));

    for(struct MipField field = MipField_fromPacket(packet); MipField_isValid(&field); MipField_next(&field))
    {
        printf(" %02X", MipField_fieldDescriptor(&field));
    }
    printf("\n");
}

void handleAccel(void* user, const struct MipField* field, Timestamp timestamp)
{
    (void)user;
    struct MipData_Sensor_ScaledAccel data;

    size_t readBytes = extract_MipData_Sensor_ScaledAccel(MipField_payload(field), MipField_payloadLength(field), 0, &data);

    if(readBytes == MipField_payloadLength(field))
        printf("Accel Data: %f, %f, %f\n", data.scaled_accel[0], data.scaled_accel[1], data.scaled_accel[2]);
}

void handleGyro(void* user, const struct MipField* field, Timestamp timestamp)
{
    (void)user;
    struct MipData_Sensor_ScaledGyro data;

    size_t readBytes = extract_MipData_Sensor_ScaledGyro(MipField_payload(field), MipField_payloadLength(field), 0, &data);

    if(readBytes == MipField_payloadLength(field))
        printf("Gyro Data:  %f, %f, %f\n", data.scaled_gyro[0], data.scaled_gyro[1], data.scaled_gyro[2]);
}

void handleMag(void* user, const struct MipField* field, Timestamp timestamp)
{
    (void)user;
    struct MipData_Sensor_ScaledMag data;

    size_t readBytes = extract_MipData_Sensor_ScaledMag(MipField_payload(field), MipField_payloadLength(field), 0, &data);

    if(readBytes == MipField_payloadLength(field))
        printf("Mag Data:   %f, %f, %f\n", data.scaled_mag[0], data.scaled_mag[1], data.scaled_mag[2]);
}


time_t startTime;

Timestamp getCurrentTimestamp()
{
    time_t t;
    time(&t);

    double delta = difftime(t, startTime);

    return (Timestamp)(delta * 1000);
}


bool MipInterface_userPoll(struct MipInterfaceState* device)
{
    Timestamp now = getCurrentTimestamp();

    // Ensure commands can time out even if no data is received.
    MipCmdQueue_update(MipInterface_cmdQueue(device), now);

    uint8_t buffer[256];

#ifdef WIN32
#else
    ssize_t bytes_read = read(port, buffer, sizeof(buffer));
    if( bytes_read < 0 && errno != EAGAIN )
        return false;
#endif

    MipInterface_receiveBytes(device, buffer, bytes_read, now);

    return true;
}


bool MipInterface_userSendToDevice(struct MipInterfaceState* device, const uint8_t* data, size_t length)
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

    MipInterface_init(&device, parseBuffer, sizeof(parseBuffer), mipTimeoutFromBaudrate(baudrate), 1000);

    // Record program start time for use with difftime in getTimestamp().
    time(&startTime);

    MipCmdResult result;

    // Get the base rate.
    volatile uint32_t now = clock();
    uint16_t base_rate;
    result = get_data_base_rate(&device, MIP_SENSOR_DATA_DESC_SET, &base_rate);

    if( result != MIP_ACK_OK )
    {
        fprintf(stderr, "Failed to get base rate: %s (%d)\n", MipCmdResult_toString(result), result);
        goto done;
    }

    // Set the message format to stream at 100 Hz.

    const uint16_t sample_rate = 100; // Hz
    const uint16_t decimation = base_rate / sample_rate;

    const struct MipDescriptorRate descriptors[3] = {
        { MIP_DATA_DESC_SENSOR_ACCEL_SCALED, decimation },
        { MIP_DATA_DESC_SENSOR_GYRO_SCALED,  decimation },
        { MIP_DATA_DESC_SENSOR_MAG_SCALED,   decimation },
    };

    result = write_mip_cmd_3dm_message_format(&device, MIP_SENSOR_DATA_DESC_SET, 3, descriptors);
    if( result == MIP_NACK_INVALID_PARAM )
    {
        // Failed to set message format - maybe this device doesn't have a magnetometer.
        // Try again without the last descriptor (scaled mag).
        result = write_mip_cmd_3dm_message_format(&device, MIP_SENSOR_DATA_DESC_SET, 2, descriptors);
    }
    if( result != MIP_ACK_OK )
    {
        fprintf(stderr, "Failed to set message format: %s (%d)\n", MipCmdResult_toString(result), result);
        goto done;
    }

    // Register some callbacks.
    struct MipDispatchHandler packetHandler;
    struct MipDispatchHandler dataHandlers[3];
    MipInterface_registerPacketCallback(&device, &packetHandler, MIP_DISPATCH_DESCSET_DATA, &handlePacket, NULL);
    MipInterface_registerFieldCallback(&device, &dataHandlers[0], MIP_SENSOR_DATA_DESC_SET, MIP_DATA_DESC_SENSOR_ACCEL_SCALED, &handleAccel, NULL);
    MipInterface_registerFieldCallback(&device, &dataHandlers[1], MIP_SENSOR_DATA_DESC_SET, MIP_DATA_DESC_SENSOR_GYRO_SCALED , &handleGyro , NULL);
    MipInterface_registerFieldCallback(&device, &dataHandlers[2], MIP_SENSOR_DATA_DESC_SET, MIP_DATA_DESC_SENSOR_MAG_SCALED  , &handleMag  , NULL);

    result = resume(&device);
    if( result != MIP_ACK_OK )
    {
        fprintf(stderr, "Failed to resume device: %s (%d)\n", MipCmdResult_toString(result), result);
        goto done;
    }

    // Process data for 3 seconds.
    for(unsigned int i=0; i<30; i++)
    {
#ifdef WIN32
#else
        usleep(100000);
#endif
        MipInterface_poll(&device);
    }

    result = set_to_idle(&device);
    if( result != MIP_ACK_OK )
    {
        fprintf(stderr, "Failed to idle device: %s (%d)\n", MipCmdResult_toString(result), result);
        goto done;
    }

done:

#ifdef WIN32
#else
    close(port);
#endif

    return result == MIP_ACK_OK ? 0 : 2;
}
