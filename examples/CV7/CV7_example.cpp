
/////////////////////////////////////////////////////////////////////////////
//
// CV7_Example.cpp
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
//! FOR THEM TO SAVE TIME. AS A RESULT, MICROSTRAIN BY HBK SHALL NOT BE HELD
//! LIABLE FOR ANY DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY
//! CLAIMS ARISING FROM THE CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS
//! OF THE CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
//
/////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Include Files
////////////////////////////////////////////////////////////////////////////////

#include "example_utils.hpp"

#include <mip/mip_all.hpp>

#include <array>

////////////////////////////////////////////////////////////////////////////////
// Global Variables
////////////////////////////////////////////////////////////////////////////////

//Sensor-to-vehicle frame transformation (Euler Angles)
float sensor_to_vehicle_transformation_euler[3] = {0.0, 0.0, 0.0};

//Device data stores
mip::data_shared::GpsTimestamp sensor_gps_time;
mip::data_sensor::ScaledAccel  sensor_accel;
mip::data_sensor::ScaledGyro   sensor_gyro;
mip::data_sensor::ScaledMag    sensor_mag;

mip::data_shared::GpsTimestamp filter_gps_time;
mip::data_filter::Status       filter_status;
mip::data_filter::EulerAngles  filter_euler_angles;

bool filter_state_ahrs = false;


const uint8_t FILTER_ROLL_EVENT_ACTION_ID  = 1;
const uint8_t FILTER_PITCH_EVENT_ACTION_ID = 2;


////////////////////////////////////////////////////////////////////////////////
// Function Prototypes
////////////////////////////////////////////////////////////////////////////////

int usage(const char* argv0);

void exit_gracefully(const char *message);
bool should_exit();

void handleFilterEventSource(void*, const mip::FieldView& field, mip::Timestamp timestamp);

////////////////////////////////////////////////////////////////////////////////
// Main Function
////////////////////////////////////////////////////////////////////////////////


int main(int argc, const char* argv[])
{

    std::unique_ptr<ExampleUtils> utils;
    try {
        utils = handleCommonArgs(argc, argv);
    } catch(const std::underflow_error&) {
        return printCommonUsage(argv);
    } catch(const std::exception& ex) {
        fprintf(stderr, "Error: %s\n", ex.what());
        return 1;
    }

    std::unique_ptr<mip::Interface>& device = utils->device;

    //
    //Ping the device (note: this is good to do to make sure the device is present)
    //

    if(mip::commands_base::ping(*device) != mip::CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not ping the device!");


    //
    //Idle the device (note: this is good to do during setup)
    //

    if(mip::commands_base::setIdle(*device) != mip::CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not set the device to idle!");


    //
    //Load the device default settings (so the device is in a known state)
    //

    if(mip::commands_3dm::defaultDeviceSettings(*device) != mip::CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not load default device settings!");


    //
    //Setup Sensor data format to 100 Hz
    //

    uint16_t sensor_base_rate;

    //Note: Querying the device base rate is only one way to calculate the descriptor decimation.
    //We could have also set it directly with information from the datasheet (shown in GNSS setup).

    if(mip::commands_3dm::getBaseRate(*device, mip::data_sensor::DESCRIPTOR_SET, &sensor_base_rate) != mip::CmdResult::ACK_OK)
         exit_gracefully("ERROR: Could not get sensor base rate format!");

    const uint16_t sensor_sample_rate = 100; // Hz
    const uint16_t sensor_decimation = sensor_base_rate / sensor_sample_rate;

    std::array<mip::DescriptorRate, 4> sensor_descriptors = {{
        { mip::data_shared::DATA_GPS_TIME,     sensor_decimation },
        { mip::data_sensor::DATA_ACCEL_SCALED, sensor_decimation },
        { mip::data_sensor::DATA_GYRO_SCALED,  sensor_decimation },
        { mip::data_sensor::DATA_MAG_SCALED,   sensor_decimation },
    }};

    if(mip::commands_3dm::writeMessageFormat(*device, mip::data_sensor::DESCRIPTOR_SET, static_cast<uint8_t>(sensor_descriptors.size()), sensor_descriptors.data()) != mip::CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not set sensor message format!");


    //
    //Setup FILTER data format
    //

    uint16_t filter_base_rate;

    if(mip::commands_3dm::getBaseRate(*device, mip::data_filter::DESCRIPTOR_SET, &filter_base_rate) != mip::CmdResult::ACK_OK)
         exit_gracefully("ERROR: Could not get filter base rate format!");

    const uint16_t filter_sample_rate = 100; // Hz
    const uint16_t filter_decimation = filter_base_rate / filter_sample_rate;

    std::array<mip::DescriptorRate, 3> filter_descriptors = {{
        { mip::data_shared::DATA_GPS_TIME,         filter_decimation },
        { mip::data_filter::DATA_FILTER_STATUS,    filter_decimation },
        { mip::data_filter::DATA_ATT_EULER_ANGLES, filter_decimation },
    }};

    if(mip::commands_3dm::writeMessageFormat(*device, mip::data_filter::DESCRIPTOR_SET, static_cast<uint8_t>(filter_descriptors.size()), filter_descriptors.data()) != mip::CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not set filter message format!");


    //
    // Setup event triggers/actions on > 45 degrees filter pitch and roll Euler angles
    // (Note 1: we are reusing the event and action structs, since the settings for pitch/roll are so similar)
    // (Note 2: we are using the same value for event and action ids.  This is not necessary, but done here for convenience)
    //

    //EVENTS

    //Roll
    mip::commands_3dm::EventTrigger::Parameters event_params;
    event_params.threshold.type       = mip::commands_3dm::EventTrigger::ThresholdParams::Type::WINDOW;
    event_params.threshold.desc_set   = mip::data_filter::DESCRIPTOR_SET;
    event_params.threshold.field_desc = mip::data_filter::DATA_ATT_EULER_ANGLES;
    event_params.threshold.param_id   = 1;
    event_params.threshold.high_thres = -0.7853981;
    event_params.threshold.low_thres  = 0.7853981;

    if(mip::commands_3dm::writeEventTrigger(*device, FILTER_ROLL_EVENT_ACTION_ID, mip::commands_3dm::EventTrigger::Type::THRESHOLD, event_params) != mip::CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not set roll event parameters!");

    //Pitch
    event_params.threshold.param_id = 2;

    if(mip::commands_3dm::writeEventTrigger(*device, FILTER_PITCH_EVENT_ACTION_ID, mip::commands_3dm::EventTrigger::Type::THRESHOLD, event_params) != mip::CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not set pitch event parameters!");

    //ACTIONS

    //Roll
    mip::commands_3dm::EventAction::Parameters event_action;
    event_action.message.desc_set       = mip::data_filter::DESCRIPTOR_SET;
    event_action.message.num_fields     = 1;
    event_action.message.descriptors[0] = mip::data_shared::DATA_EVENT_SOURCE;
    event_action.message.decimation     = 0;

    if(writeEventAction(*device, FILTER_ROLL_EVENT_ACTION_ID, FILTER_ROLL_EVENT_ACTION_ID, mip::commands_3dm::EventAction::Type::MESSAGE, event_action) != mip::CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not set roll action parameters!");

    //Pitch
    if(writeEventAction(*device, FILTER_PITCH_EVENT_ACTION_ID, FILTER_PITCH_EVENT_ACTION_ID, mip::commands_3dm::EventAction::Type::MESSAGE, event_action) != mip::CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not set pitch action parameters!");


    //ENABLE EVENTS

    //Roll
    if(writeEventControl(*device, FILTER_ROLL_EVENT_ACTION_ID, mip::commands_3dm::EventControl::Mode::ENABLED) != mip::CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not enable roll event!");

    //Pitch
    if(writeEventControl(*device, FILTER_PITCH_EVENT_ACTION_ID, mip::commands_3dm::EventControl::Mode::ENABLED) != mip::CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not enable pitch event!");

    //
    //Setup the sensor to vehicle transformation
    //

    if(mip::commands_3dm::writeSensor2VehicleTransformEuler(*device, sensor_to_vehicle_transformation_euler[0], sensor_to_vehicle_transformation_euler[1], sensor_to_vehicle_transformation_euler[2]) != mip::CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not set sensor-to-vehicle transformation!");


    //
    //Setup the filter aiding measurements (GNSS position/velocity and dual antenna [aka gnss heading])
    //

    if(mip::commands_filter::writeAidingMeasurementEnable(*device, mip::commands_filter::AidingMeasurementEnable::AidingSource::MAGNETOMETER, true) != mip::CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not set filter aiding measurement enable!");


    //
    //Reset the filter (note: this is good to do after filter setup is complete)
    //

    if(mip::commands_filter::reset(*device) != mip::CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not reset the filter!");


    //
    // Register data callbacks
    //

    //Sensor Data
    mip::DispatchHandler sensor_data_handlers[4];

    device->registerExtractor(sensor_data_handlers[0], &sensor_gps_time, mip::data_sensor::DESCRIPTOR_SET);
    device->registerExtractor(sensor_data_handlers[1], &sensor_accel);
    device->registerExtractor(sensor_data_handlers[2], &sensor_gyro);
    device->registerExtractor(sensor_data_handlers[3], &sensor_mag);

    //Filter Data
    mip::DispatchHandler filter_data_handlers[4];

    device->registerExtractor(filter_data_handlers[0], &filter_gps_time, mip::data_filter::DESCRIPTOR_SET);
    device->registerExtractor(filter_data_handlers[1], &filter_status);
    device->registerExtractor(filter_data_handlers[2], &filter_euler_angles);

    device->registerFieldCallback<&handleFilterEventSource>(filter_data_handlers[3], mip::data_filter::DESCRIPTOR_SET, mip::data_shared::DATA_EVENT_SOURCE);

    //
    //Resume the device
    //

    if(mip::commands_base::resume(*device) != mip::CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not resume the device!");


    //
    //Main Loop: Update the interface and process data
    //

    bool running = true;
    mip::Timestamp prev_print_timestamp = getCurrentTimestamp();

    printf("Sensor is configured... waiting for filter to enter AHRS mode (AHRS).\n");

    std::string current_state = "";
    while(running)
    {
        device->update();
        displayFilterState(filter_status.filter_state, current_state);

        //Check Filter State
        if((!filter_state_ahrs) && (filter_status.filter_state == mip::data_filter::FilterMode::AHRS))
        {
            printf("NOTE: Filter has entered AHRS mode.\n");
            filter_state_ahrs = true;
        }

        //Once in AHRS Flter Mode, print out data at 10 Hz
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
// Filter Event Source Field Handler
////////////////////////////////////////////////////////////////////////////////

void handleFilterEventSource(void*, const mip::FieldView& field, mip::Timestamp)
{
    mip::data_shared::EventSource data;

    if(field.extract(data))
    {
      if(data.trigger_id == FILTER_ROLL_EVENT_ACTION_ID)
        printf("WARNING: Roll event triggered!\n");
      else if(data.trigger_id == FILTER_PITCH_EVENT_ACTION_ID)
        printf("WARNING: Pitch event triggered!\n");
    }
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

#ifdef MICROSTRAIN_PLATFORM_WINDOWS
    printf("Press ENTER to exit...\n");
    getchar();
#endif

    exit(0);
}


////////////////////////////////////////////////////////////////////////////////
// Check for Exit Condition
////////////////////////////////////////////////////////////////////////////////

bool should_exit()
{
  return false;

}
