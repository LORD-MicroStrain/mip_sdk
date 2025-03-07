
#include "example_utils.h"

#include <mip/mip_dispatch.h>
#include <mip/mip_interface.h>
#include <microstrain/connections/serial/serial_port.h>
#include <microstrain/platform.h>

#include <mip/definitions/commands_base.h>
#include <mip/definitions/commands_3dm.h>
#include <mip/definitions/data_sensor.h>

#include <stdio.h>
#include <stdlib.h>

mip_interface device;
mip_sensor_scaled_accel_data scaled_accel;

void handlePacket(void* unused, const mip_packet_view* packet, mip_timestamp timestamp)
{
    (void)unused;
    (void)timestamp;

    printf("\nGot packet with descriptor set 0x%02X:", mip_packet_descriptor_set(packet));

    mip_field_view field;
    mip_field_init_empty(&field);
    while( mip_field_next_in_packet(&field, packet) )
    {
        printf(" %02X", mip_field_field_descriptor(&field));
    }
    printf("\n");
}

void handleAccel(void* user, const mip_field_view* field, mip_timestamp timestamp)
{
    (void)user;
    (void)timestamp;

    mip_sensor_scaled_accel_data data;

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

void handleGyro(void* user, const mip_field_view* field, mip_timestamp timestamp)
{
    (void)user;
    (void)timestamp;

    mip_sensor_scaled_gyro_data data;

    if(extract_mip_sensor_scaled_gyro_data_from_field(field, &data))
        printf("Gyro Data:  %f, %f, %f\n", data.scaled_gyro[0], data.scaled_gyro[1], data.scaled_gyro[2]);
}

void handleMag(void* user, const mip_field_view* field, mip_timestamp timestamp)
{
    (void)user;
    (void)timestamp;

    mip_sensor_scaled_mag_data data;

    if(extract_mip_sensor_scaled_mag_data_from_field(field, &data))
        printf("Mag Data:   %f, %f, %f\n", data.scaled_mag[0], data.scaled_mag[1], data.scaled_mag[2]);
}



bool open_port(const char* name, uint32_t baudrate)
{
    return serial_port_open(&device_port, name, (int)baudrate);
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

    uint32_t baudrate = strtoul(argv[2], NULL, 10);
    if( baudrate == 0 )
        return usage(argv[0]);

    mip_example_init();

    if( !open_port(argv[1], baudrate) )
        return 1;

    mip_interface_init(
        &device, mip_timeout_from_baudrate(baudrate), 1000,
        &mip_interface_user_send_to_device, &mip_interface_user_recv_from_device, &mip_interface_default_update, NULL
    );

    enum mip_cmd_result result;

    // Get the base rate.
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

    const mip_descriptor_rate descriptors[3] = {
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
    mip_dispatch_handler packet_handler;
    mip_dispatch_handler data_handlers[4];
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
#ifdef MICROSTRAIN_PLATFORM_WINDOWS
#else
        usleep(100000);
#endif
        mip_interface_update(&device, 0, false);
    }

    result = mip_base_set_idle(&device);
    if( result != MIP_ACK_OK )
    {
        fprintf(stderr, "Failed to idle device: %s (%d)\n", mip_cmd_result_to_string(result), result);
        goto done;
    }

done:

#ifdef MIP_ENABLE_DIAGNOSTICS
    printf(
        "\nDiagnostics:\n"
        "\n"
        "Commands:\n"
        "  Sent:     %" PRIu16 "\n"
        "  Acks:     %" PRIu16 "\n"
        "  Nacks:    %" PRIu16 "\n"
        "  Timeouts: %" PRIu16 "\n"
        "  Errors:   %" PRIu16 "\n"
        "\n"
        "Parser:\n"
        "  Valid packets:    %" PRIu32 "\n"
        "  Invalid packets:  %" PRIu32 "\n"
        "  Timeouts:         %" PRIu32 "\n"
        "\n"
        "  Bytes read:       %" PRIu32 "\n"
        "  Valid bytes:      %" PRIu32 "\n"
        "  Unparsed bytes:   %" PRIu32 "\n",
        mip_cmd_queue_diagnostic_cmds_queued(mip_interface_cmd_queue(&device)),
        mip_cmd_queue_diagnostic_cmd_acks(mip_interface_cmd_queue(&device)),
        mip_cmd_queue_diagnostic_cmd_nacks(mip_interface_cmd_queue(&device)),
        mip_cmd_queue_diagnostic_cmd_timeouts(mip_interface_cmd_queue(&device)),
        mip_cmd_queue_diagnostic_cmd_errors(mip_interface_cmd_queue(&device)),

        mip_parser_diagnostic_valid_packets(mip_interface_parser(&device)),
        mip_parser_diagnostic_invalid_packets(mip_interface_parser(&device)),
        mip_parser_diagnostic_timeouts(mip_interface_parser(&device)),
        mip_parser_diagnostic_bytes_read(mip_interface_parser(&device)),
        mip_parser_diagnostic_packet_bytes(mip_interface_parser(&device)),
        mip_parser_diagnostic_bytes_skipped(mip_interface_parser(&device))
    );
#endif // MIP_ENABLE_DIAGNOSTICS

    serial_port_close(&device_port);
    return result == MIP_ACK_OK ? 0 : 2;
}
