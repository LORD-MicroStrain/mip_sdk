
/////////////////////////////////////////////////////////////////////////////
//
// CX5_GX5_CV5_15_25_Example.c
//
// C Example set-up program for the CX5-15, CX5-25, GX5-15, GX5-25, CV5-15, and CV5-25.
//
// This example shows a typical setup for the CX5-15 sensor in a wheeled-vehicle application using using C.
// It is not an exhaustive example of all CX5-15 settings.
// If your specific setup needs are not met by this example, please consult
// the MSCL-embedded API documentation for the proper commands.
//
//
//!@section LICENSE
//!
//! THE PRESENT SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING
//! CUSTOMERS WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER
//! FOR THEM TO SAVE TIME. AS A RESULT, MICROSTRAIN BY HBK SHALL NOT BE HELD
//! LIABLE FOR ANY DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY
//! CLAIMS ARISING FROM THE CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS
//! OF THE CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
//
/////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Include Files
////////////////////////////////////////////////////////////////////////////////

#include "example_utils.h"

#include <mip/mip_all.h>
#include <microstrain/connections/serial/serial_port.h>

#include <stdio.h>
#include <stdlib.h>

////////////////////////////////////////////////////////////////////////////////
// Global Variables
////////////////////////////////////////////////////////////////////////////////

int port = -1;
mip_interface device;

//Sensor-to-vehicle frame rotation (Euler Angles)
float sensor_to_vehicle_rotation_euler[3] = {0.0, 0.0, 0.0};

//Device data stores
mip_sensor_gps_timestamp_data sensor_gps_time;
mip_sensor_scaled_accel_data  sensor_accel;
mip_sensor_scaled_gyro_data   sensor_gyro;

mip_filter_timestamp_data     filter_gps_time;
mip_filter_status_data        filter_status;
mip_filter_euler_angles_data  filter_euler_angles;
mip_filter_comp_angular_rate_data  filter_comp_angular_rate;
mip_filter_comp_accel_data  filter_comp_accel;

bool filter_state_running = false;


////////////////////////////////////////////////////////////////////////////////
// Function Prototypes
////////////////////////////////////////////////////////////////////////////////

int usage(const char* argv0);
bool should_exit();


////////////////////////////////////////////////////////////////////////////////
// Main Function
////////////////////////////////////////////////////////////////////////////////


int main(int argc, const char* argv[])
{

    //
    //Process arguments
    //

    if(argc != 3)
        return usage(argv[0]);

    const char* port_name = argv[1];
    uint32_t baudrate     = atoi(argv[2]);

    if(baudrate == 0)
        return usage(argv[0]);

    mip_example_init();

    printf("Connecting to and configuring sensor.\n");

    //
    //Open the device port
    //

    if(!serial_port_open(&device_port, port_name, baudrate))
        exit_gracefully("ERROR: Could not open device port!");


    //
    //Initialize the MIP interface
    //

    mip_interface_init(
        &device, mip_timeout_from_baudrate(baudrate), 1000,
        &mip_interface_user_send_to_device, &mip_interface_user_recv_from_device, &mip_interface_default_update, NULL
    );



    //
    //Ping the device (note: this is good to do to make sure the device is present)
    //

    if(mip_base_ping(&device) != MIP_ACK_OK)
        exit_gracefully("ERROR: Could not ping the device!");


    //
    //Idle the device (note: this is good to do during setup)
    //

    if(mip_base_set_idle(&device) != MIP_ACK_OK)
        exit_gracefully("ERROR: Could not set the device to idle!");


    //
    //Load the device default settings (so the device is in a known state)
    //

    if(mip_3dm_default_device_settings(&device) != MIP_ACK_OK)
        exit_gracefully("ERROR: Could not load default device settings!");

    const uint16_t sampling_time = 2000; // The default is 15000 ms and longer sample times are recommended but shortened for convenience
    mip_cmd_queue* queue = mip_interface_cmd_queue(&device);
    const mip_timeout old_mip_sdk_timeout = mip_cmd_queue_base_reply_timeout(queue);
    printf("Capturing gyro bias. This will take %d seconds. \n", sampling_time/1000);
    mip_cmd_queue_set_base_reply_timeout(queue, sampling_time * 2);    
    float gyro_bias[3] = {0, 0, 0};
        
    if(mip_3dm_capture_gyro_bias(&device, sampling_time, gyro_bias) != MIP_ACK_OK)
        exit_gracefully("ERROR: Could not load default device settings!");

    mip_cmd_queue_set_base_reply_timeout(queue, old_mip_sdk_timeout);

    //
    //Setup Sensor data format to 100 Hz
    //

    uint16_t sensor_base_rate;

    //Note: Querying the device base rate is only one way to calculate the descriptor decimation.
    //We could have also set it directly with information from the datasheet.

    if(mip_3dm_imu_get_base_rate(&device, &sensor_base_rate) != MIP_ACK_OK)
         exit_gracefully("ERROR: Could not get sensor base rate format!");

    const uint16_t sensor_sample_rate = 100; // Hz
    const uint16_t sensor_decimation = sensor_base_rate / sensor_sample_rate;

    const mip_descriptor_rate sensor_descriptors[3] = {
        { MIP_DATA_DESC_SENSOR_TIME_STAMP_GPS, sensor_decimation },
        { MIP_DATA_DESC_SENSOR_ACCEL_SCALED,   sensor_decimation },
        { MIP_DATA_DESC_SENSOR_GYRO_SCALED,    sensor_decimation },
    };

    if(mip_3dm_write_imu_message_format(&device, 3, sensor_descriptors) != MIP_ACK_OK)
        exit_gracefully("ERROR: Could not set sensor message format!");


    //
    //Setup FILTER data format
    //

    uint16_t filter_base_rate;

    if(mip_3dm_filter_get_base_rate(&device, &filter_base_rate) != MIP_ACK_OK)
         exit_gracefully("ERROR: Could not get filter base rate format!");

    const uint16_t filter_sample_rate = 100; // Hz
    const uint16_t filter_decimation = filter_base_rate / filter_sample_rate;

    const mip_descriptor_rate filter_descriptors[5] = {
        { MIP_DATA_DESC_FILTER_FILTER_TIMESTAMP, filter_decimation },
        { MIP_DATA_DESC_FILTER_FILTER_STATUS,    filter_decimation },
        { MIP_DATA_DESC_FILTER_ATT_EULER_ANGLES, filter_decimation },
        { MIP_DATA_DESC_FILTER_COMPENSATED_ANGULAR_RATE, filter_decimation },
        { MIP_DATA_DESC_FILTER_COMPENSATED_ACCELERATION, filter_decimation },
    };

    if(mip_3dm_write_filter_message_format(&device, 5, filter_descriptors) != MIP_ACK_OK)
        exit_gracefully("ERROR: Could not set filter message format!");


    //
    //Setup the sensor to vehicle rotation
    //

    if(mip_filter_write_sensor_to_vehicle_rotation_euler(&device, sensor_to_vehicle_rotation_euler[0], sensor_to_vehicle_rotation_euler[1], sensor_to_vehicle_rotation_euler[2]) != MIP_ACK_OK)
        exit_gracefully("ERROR: Could not set sensor-2-vehicle rotation!");

    //
    //Enable filter auto-initialization
    //

    if(mip_filter_write_auto_init_control(&device, 1) != MIP_ACK_OK)
        exit_gracefully("ERROR: Could not set filter autoinit control!");

    //
    // Register data callbacks
    //

    //Sensor Data
    mip_dispatch_handler sensor_data_handlers[3];

    mip_interface_register_extractor(&device, &sensor_data_handlers[0], MIP_SENSOR_DATA_DESC_SET, MIP_DATA_DESC_SENSOR_TIME_STAMP_GPS, extract_mip_sensor_gps_timestamp_data_from_field,    &sensor_gps_time);
    mip_interface_register_extractor(&device, &sensor_data_handlers[1], MIP_SENSOR_DATA_DESC_SET, MIP_DATA_DESC_SENSOR_ACCEL_SCALED,   extract_mip_sensor_scaled_accel_data_from_field, &sensor_accel);
    mip_interface_register_extractor(&device, &sensor_data_handlers[2], MIP_SENSOR_DATA_DESC_SET, MIP_DATA_DESC_SENSOR_GYRO_SCALED,    extract_mip_sensor_scaled_gyro_data_from_field,  &sensor_gyro);

    //Filter Data
    mip_dispatch_handler filter_data_handlers[5];

    mip_interface_register_extractor(&device, &filter_data_handlers[0], MIP_FILTER_DATA_DESC_SET, MIP_DATA_DESC_FILTER_FILTER_TIMESTAMP, extract_mip_filter_timestamp_data_from_field,                  &filter_gps_time);
    mip_interface_register_extractor(&device, &filter_data_handlers[1], MIP_FILTER_DATA_DESC_SET, MIP_DATA_DESC_FILTER_FILTER_STATUS,    extract_mip_filter_status_data_from_field,                     &filter_status);
    mip_interface_register_extractor(&device, &filter_data_handlers[2], MIP_FILTER_DATA_DESC_SET, MIP_DATA_DESC_FILTER_ATT_EULER_ANGLES, extract_mip_filter_euler_angles_data_from_field,               &filter_euler_angles);
    mip_interface_register_extractor(&device, &filter_data_handlers[3], MIP_FILTER_DATA_DESC_SET, MIP_DATA_DESC_FILTER_COMPENSATED_ANGULAR_RATE, extract_mip_filter_comp_angular_rate_data_from_field,  &filter_comp_angular_rate);
    mip_interface_register_extractor(&device, &filter_data_handlers[4], MIP_FILTER_DATA_DESC_SET, MIP_DATA_DESC_FILTER_COMPENSATED_ACCELERATION, extract_mip_filter_comp_accel_data_from_field,         &filter_comp_accel);


    //
    //Resume the device
    //

    if(mip_base_resume(&device) != MIP_ACK_OK)
        exit_gracefully("ERROR: Could not resume the device!");


    //
    //Main Loop: Update the interface and process data
    //

    bool running = true;
    mip_timestamp prev_print_timestamp = 0;

    printf("Sensor is configured... waiting for filter to enter running mode.\n");

    while(running)
    {
        mip_interface_update(&device, 0, false);

        //Check Filter State
        if((!filter_state_running) && ((filter_status.filter_state == MIP_FILTER_MODE_GX5_RUN_SOLUTION_ERROR) || (filter_status.filter_state == MIP_FILTER_MODE_GX5_RUN_SOLUTION_VALID)))
        {
            printf("NOTE: Filter has entered running mode.\n");
            filter_state_running = true;
        }

        //Once in running mode, print out data at 1 Hz
        if(filter_state_running)
        {
           mip_timestamp curr_time = get_current_timestamp();

           if(curr_time - prev_print_timestamp >= 1000)
           {
                printf("TOW = %f: ATT_EULER = [%f %f %f]: COMP_ANG_RATE = [%f %f %f]: COMP_ACCEL = [%f %f %f]\n",
                       filter_gps_time.tow, filter_euler_angles.roll, filter_euler_angles.pitch, filter_euler_angles.yaw, 
                       filter_comp_angular_rate.gyro[0], filter_comp_angular_rate.gyro[1], filter_comp_angular_rate.gyro[2],
                       filter_comp_accel.accel[0], filter_comp_accel.accel[1], filter_comp_accel.accel[2]);

                prev_print_timestamp = curr_time;
           }
        }

        running = !should_exit();
    }

    exit_gracefully("Example Completed Successfully.");
}


////////////////////////////////////////////////////////////////////////////////
// Print Usage Function
////////////////////////////////////////////////////////////////////////////////

int usage(const char* argv0)
{
    printf("Usage: %s <port> <baudrate>\n", argv0);
    return 1;
}


////////////////////////////////////////////////////////////////////////////////
// Check for Exit Condition
////////////////////////////////////////////////////////////////////////////////

bool should_exit()
{
  return false;

}

