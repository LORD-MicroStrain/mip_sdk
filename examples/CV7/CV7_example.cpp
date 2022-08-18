
/////////////////////////////////////////////////////////////////////////////
//
// CV7_Example.c 
//
// C++ Example set-up program for the CV7
//
// This example shows a typical setup for the CV7 sensor using C++.  
// It is not an exhaustive example of all CV7 settings.
// If your specific setup needs are not met by this example, please consult
// the MSCL-embedded API documentation for the proper commands.
//
//
//!@section LICENSE
//!
//! THE PRESENT SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING 
//! CUSTOMERS WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER 
//! FOR THEM TO SAVE TIME. AS A RESULT, PARKER MICROSTRAIN SHALL NOT BE HELD 
//! LIABLE FOR ANY DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY 
//! CLAIMS ARISING FROM THE CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS 
//! OF THE CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
//
/////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Include Files
////////////////////////////////////////////////////////////////////////////////

#include <mip/mip_all.hpp>
#include <array>
#include "../example_utils.hpp"

using namespace mip;

////////////////////////////////////////////////////////////////////////////////
// Global Variables
////////////////////////////////////////////////////////////////////////////////

std::unique_ptr<mip::DeviceInterface> device;

//Device data stores
data_shared::GpsTimestamp sensor_gps_time;
data_sensor::ScaledAccel  sensor_accel;
data_sensor::ScaledGyro   sensor_gyro;
data_sensor::ScaledMag    sensor_mag;

data_shared::GpsTimestamp filter_gps_time;
data_filter::Status       filter_status;
data_filter::EulerAngles  filter_euler_angles;

bool filter_state_ahrs = false;


////////////////////////////////////////////////////////////////////////////////
// Function Prototypes
////////////////////////////////////////////////////////////////////////////////

int usage(const char* argv0);

void exit_gracefully(const char *message);
bool should_exit();


////////////////////////////////////////////////////////////////////////////////
// Main Function
////////////////////////////////////////////////////////////////////////////////


void main(int argc, const char* argv[])
{

    device = handleCommonArgs(argc, argv);

    //
    //Ping the device (note: this is good to do to make sure the device is present)
    //

    if(commands_base::ping(*device) != CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not ping the device!");


    //
    //Idle the device (note: this is good to do during setup)
    //

    if(commands_base::setIdle(*device) != CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not set the device to idle!");


    //
    //Load the device default settings (so the device is in a known state)
    //

    if(commands_3dm::defaultDeviceSettings(*device) != CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not load default device settings!");


    //
    //Setup Sensor data format to 100 Hz
    //
    
    uint16_t sensor_base_rate;

    //Note: Querying the device base rate is only one way to calculate the descriptor decimation.
    //We could have also set it directly with information from the datasheet (shown in GNSS setup).

    if(commands_3dm::getBaseRate(*device, data_sensor::DESCRIPTOR_SET, &sensor_base_rate) != CmdResult::ACK_OK)
         exit_gracefully("ERROR: Could not get sensor base rate format!");
   
    const uint16_t sensor_sample_rate = 100; // Hz
    const uint16_t sensor_decimation = sensor_base_rate / sensor_sample_rate;

    std::array<DescriptorRate, 4> sensor_descriptors = {{
        { data_shared::DATA_GPS_TIME,     sensor_decimation },
        { data_sensor::DATA_ACCEL_SCALED, sensor_decimation },
        { data_sensor::DATA_GYRO_SCALED,  sensor_decimation },
        { data_sensor::DATA_MAG_SCALED,   sensor_decimation },
    }};

    if(commands_3dm::writeMessageFormat(*device, data_sensor::DESCRIPTOR_SET, sensor_descriptors.size(), sensor_descriptors.data()) != CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not set sensor message format!");


    //
    //Setup FILTER data format
    //

    uint16_t filter_base_rate;

    if(commands_3dm::getBaseRate(*device, data_filter::DESCRIPTOR_SET, &filter_base_rate) != CmdResult::ACK_OK)
         exit_gracefully("ERROR: Could not get filter base rate format!");
   
    const uint16_t filter_sample_rate = 100; // Hz
    const uint16_t filter_decimation = filter_base_rate / filter_sample_rate;

    std::array<DescriptorRate, 3> filter_descriptors = {{
        { data_shared::DATA_GPS_TIME,         filter_decimation },
        { data_filter::DATA_FILTER_STATUS,    filter_decimation },
        { data_filter::DATA_ATT_EULER_ANGLES, filter_decimation },
    }};

    if(commands_3dm::writeMessageFormat(*device, data_filter::DESCRIPTOR_SET, filter_descriptors.size(), filter_descriptors.data()) != CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not set filter message format!");

  
    //
    //Setup the filter aiding measurements (GNSS position/velocity and dual antenna [aka gnss heading])
    //

    if(commands_filter::writeAidingMeasurementEnable(*device, commands_filter::AidingMeasurementEnable::AidingSource::MAGNETOMETER, true) != CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not set filter aiding measurement enable!");


    //
    //Reset the filter (note: this is good to do after filter setup is complete)
    //
 
    if(commands_filter::reset(*device) != CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not reset the filter!");


    //
    // Register data callbacks
    //

    //Sensor Data
    DispatchHandler sensor_data_handlers[4];
  
    device->registerExtractor(sensor_data_handlers[0], &sensor_gps_time, data_sensor::DESCRIPTOR_SET);
    device->registerExtractor(sensor_data_handlers[1], &sensor_accel);
    device->registerExtractor(sensor_data_handlers[2], &sensor_gyro);
    device->registerExtractor(sensor_data_handlers[3], &sensor_mag);

    //Filter Data
    DispatchHandler filter_data_handlers[3];

    device->registerExtractor(filter_data_handlers[0], &filter_gps_time, data_filter::DESCRIPTOR_SET);
    device->registerExtractor(filter_data_handlers[1], &filter_status);
    device->registerExtractor(filter_data_handlers[4], &filter_euler_angles);


    //
    //Resume the device
    //

    if(commands_base::resume(*device) != CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not resume the device!");


    //
    //Main Loop: Update the interface and process data
    //

    bool running = true;
    mip::Timestamp prev_print_timestamp = getCurrentTimestamp();

    printf("Sensor is configured... waiting for filter to enter AHRS mode.\n");

    while(running)
    {
        device->update();

        //Check Filter State
        if((!filter_state_ahrs) && (filter_status.filter_state == data_filter::FilterMode::AHRS))
        {
            printf("NOTE: Filter has entered AHRS mode.\n");
            filter_state_ahrs = true;
        }

        //Once in full nav, print out data at 10 Hz
        if(filter_state_ahrs)
        {
           mip::Timestamp curr_timestamp = getCurrentTimestamp();

           if(curr_timestamp - prev_print_timestamp >= 1000)
           {
                printf("TOW = %f: ATT_EULER = [%f %f %f]\n",
                       filter_gps_time.tow, filter_euler_angles.roll, filter_euler_angles.pitch, filter_euler_angles.yaw);

                prev_print_timestamp = curr_timestamp;
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
// Exit Function
////////////////////////////////////////////////////////////////////////////////

void exit_gracefully(const char *message)
{
    if(message)
        printf("%s\n", message);

    //Close com port
    //if(serial_port_is_open(&device_port))
    //    serial_port_close(&device_port);

    int dummy = getchar();

    exit(0);
}


////////////////////////////////////////////////////////////////////////////////
// Check for Exit Condition
////////////////////////////////////////////////////////////////////////////////

bool should_exit()
{
  return false;

}

