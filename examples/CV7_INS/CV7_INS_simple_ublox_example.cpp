////////////////////////////////////////////////////////////////////////////////
// Include Files
////////////////////////////////////////////////////////////////////////////////

#include "mip/mip_all.hpp"
#include "../example_utils.hpp"
#include <map>
#include <cmath>
#include <cstring>

using namespace mip;

////////////////////////////////////////////////////////////////////////////////
// Global Variables
////////////////////////////////////////////////////////////////////////////////

data_shared::GpsTimestamp filter_gps_time;
data_filter::Status       filter_status;
data_filter::EulerAngles  filter_euler_angles;
data_filter::PositionLlh  filter_llh_position;
data_filter::VelocityNed  filter_ned_velocity;

uint8_t external_heading_sensor_id = 1;
uint8_t gnss_antenna_sensor_id = 2;
uint8_t vehicle_frame_velocity_sensor_id = 3;
uint8_t header_size = 6;
uint8_t checksum_size = 2;
uint8_t payload_size = 92;
uint8_t PVT_message_size = header_size + checksum_size + payload_size;

bool filter_state_full_nav = false;

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

struct UBlox_Payload_Part{
    uint8_t start_index;
    uint8_t num_bytes;
};

std::map<std::string, UBlox_Payload_Part> PAYLOAD_PART_MAP = {
     {"iTOW", {0, 4}},
     {"YEAR", {4, 2}}, 
     {"MONTH", {6, 1}}, 
     {"DAY", {7, 1}}, 
     {"HOUR", {8, 1}}, 
     {"MIN", {9, 1}}, 
     {"SEC", {10, 1}}, 
     {"VALID", {11, 1}}, 
     {"T_ACC", {12, 4}}, 
     {"NANO_SEC", {16, 4}},
     {"FIX_TYPE", {20, 1}},
     {"FLAGS", {21, 1}}, 
     {"FLAGS_2", {22, 1}}, 
     {"NUM_SV", {23, 1}},
     {"LON", {24, 4}}, 
     {"LAT", {28, 4}}, 
     {"HEIGHT", {32, 4}}, 
     {"H_MSL", {36, 4}},
     {"H_ACC", {40, 4}}, 
     {"V_ACC", {44, 4}}, 
     {"VEL_N", {48, 4}}, 
     {"VEL_E", {52, 4}}, 
     {"VEL_D", {56, 4}}, 
     {"G_SPEED", {60, 4}}, 
     {"HEAD_MOT", {64, 4}}, 
     {"S_ACC", {68, 4}}, 
     {"HEAD_ACC", {72, 4}}, 
     {"FLAGS_3", {78, 1}}, 
     {"HEAD_VEH", {84, 4}}, 
     {"MAG_DEC", {88, 2}}, 
     {"MAG_ACC", {90, 2}}
};



////////////////////////////////////////////////////////////////////////////////
// Function Prototypes
////////////////////////////////////////////////////////////////////////////////

std::unique_ptr<UBlox_PVT_Message> parse_PVT_ublox_message(const uint8_t test_ublox_message[100]);

template <typename T> 
void convert_mm_to_m(T& value);

template<typename T>
void convert_degrees_to_radians(T& degrees);

float parse_2_bytes(const uint8_t payload[92], int start_index);

double parse_4_bytes(const uint8_t payload[92], int start_index);

double parse_long_lat(const uint8_t payload[92], int start_index);

HANDLE open_uBlox_serial_port(const char* com_port, const int baud_rate);

bool verify_checksum(uint8_t *data);

int usage(const char* argv0);

void print_device_information(const commands_base::BaseDeviceInfo& device_info);

void exit_gracefully(const char *message);
bool should_exit();


int main(int argc, const char* argv[])
{
    // for (int i = 0; i < argc; ++i) {
    //     std::cout << "Argument " << i << ": " << argv[i] << std::endl;
    // }
    
    std::unique_ptr<ExampleUtils> utils = handleCommonArgs(3, argv);
    std::unique_ptr<mip::DeviceInterface>& device = utils->device;

    //
    //Ping the device (note: this is good to do to make sure the device is present)
    //

    if(commands_base::ping(*device) != CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not ping the device!");

    //
    //Read device information
    //

    commands_base::BaseDeviceInfo device_info;
    if(commands_base::getDeviceInfo(*device, &device_info) != CmdResult::ACK_OK)
        exit_gracefully("ERROR: Failed to get device info");
    print_device_information(device_info);

    //
    // Open ublox serial port
    // TODO: Needs be refactored but works
    const char* ublox_argv[2];
    ublox_argv[1] = argv[3];
    ublox_argv[2] = argv[4];

    std::unique_ptr<ExampleUtils> utils_ublox = handleCommonArgs(3, ublox_argv);
    printf("Connecting to UBlox receiver ..." );
    printf("Connected to %d at %d\n", std::string(ublox_argv[1]), std::string(ublox_argv[2]));
   
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
    //Setup external aiding sensor reference frames
    //


    //
    //External heading sensor reference frame.
    //
    float external_heading_sensor_to_vehicle_frame_rotation_euler[4] = {0.0, 0.0, 0.0, 0.0};  // External heading sensor is aligned with vehicle frame
    float external_heading_sensor_to_vehicle_frame_translation[3] = {0.0, 0.0, 0.0};  // Heading measurements are agnostic to translation, translation set to zero
    if(commands_aiding::writeReferenceFrame(*device, external_heading_sensor_id, mip::commands_aiding::ReferenceFrame::Format::EULER,
                                            external_heading_sensor_to_vehicle_frame_translation, external_heading_sensor_to_vehicle_frame_rotation_euler) != CmdResult::ACK_OK)
        exit_gracefully("ERROR: Unable to configure external heading sensor frame ID");


    //
    //External GNSS antenna reference frame
    //
    float external_gnss_antenna_to_vehicle_frame_rotation_euler[4] = {0.0, 0.0, 0.0, 0.0};  // GNSS position/velocity measurements are agnostic to rotation, rotation set to zero
    float external_gnss_antenna_to_vehicle_frame_translation[3] = {0.0, 1.0, 0.0};  // Antenna is translated 1 meter in vehicle frame Y direction
    if(commands_aiding::writeReferenceFrame(*device, gnss_antenna_sensor_id, mip::commands_aiding::ReferenceFrame::Format::EULER,
                                            external_gnss_antenna_to_vehicle_frame_translation, external_gnss_antenna_to_vehicle_frame_rotation_euler) != CmdResult::ACK_OK)
        exit_gracefully("ERROR: Unable to configure external GNSS antenna frame ID");


    //
    //External bodyframe velocity reference frame
    //
    float external_velocity_sensor_to_vehicle_frame_rotation_euler[4] = {0.0, 0.0, 1.57, 0.0};  // Rotated 90 deg around yaw axis
    float external_velocity_sensor_to_vehicle_frame_translation[3] = {1.0, 0.0, 0.0};  // Sensor is translated 1 meter in X direction
    if(commands_aiding::writeReferenceFrame(*device, vehicle_frame_velocity_sensor_id, mip::commands_aiding::ReferenceFrame::Format::EULER,
                                            external_velocity_sensor_to_vehicle_frame_translation, external_velocity_sensor_to_vehicle_frame_rotation_euler) != CmdResult::ACK_OK)
        exit_gracefully("ERROR: Unable to configure external vehicle frame velocity sensor ID");


    //
    //Setup FILTER data format
    //

    uint16_t filter_base_rate;
    if(commands_3dm::getBaseRate(*device, data_filter::DESCRIPTOR_SET, &filter_base_rate) != CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not get filter base rate format!");

    const uint16_t filter_sample_rate = 100; // Hz
    const uint16_t filter_decimation = filter_base_rate / filter_sample_rate;

    std::array<DescriptorRate, 5> filter_descriptors = {{
                                                                { data_shared::DATA_GPS_TIME,         filter_decimation },
                                                                { data_filter::DATA_FILTER_STATUS,    filter_decimation },
                                                                { data_filter::DATA_ATT_EULER_ANGLES, filter_decimation },
                                                                { data_filter::DATA_POS_LLH,          filter_decimation },
                                                                { data_filter::DATA_VEL_NED,          filter_decimation },
                                                        }};

    if(commands_3dm::writeMessageFormat(*device, data_filter::DESCRIPTOR_SET, filter_descriptors.size(), filter_descriptors.data()) != CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not set filter message format!");


    //
    //Reset the filter (note: this is good to do after filter setup is complete)
    //

    if(commands_filter::reset(*device) != CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not reset the filter!");


    //
    // Register data callbacks
    //

    //Filter Data
    DispatchHandler filter_data_handlers[5];

    device->registerExtractor(filter_data_handlers[0], &filter_gps_time, data_filter::DESCRIPTOR_SET);
    device->registerExtractor(filter_data_handlers[1], &filter_status);
    device->registerExtractor(filter_data_handlers[2], &filter_euler_angles);
    device->registerExtractor(filter_data_handlers[3], &filter_llh_position);
    device->registerExtractor(filter_data_handlers[4], &filter_ned_velocity);

    //
    //Resume the device
    //

    if(commands_base::resume(*device) != CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not resume the device!");
    
    //Main Loop: Update the interface and process data
    //

    bool running = true;
    mip::Timestamp prev_print_timestamp = getCurrentTimestamp();
    mip::Timestamp prev_measurement_update_timestamp = getCurrentTimestamp();
    
    mip::Timeout wait_time = 60000;
    int buffer_length = 1024;
    uint8_t ublox_message_bytes[buffer_length];
    size_t max_length = sizeof(ublox_message_bytes);
    size_t read_out = 0;
    size_t* length_out = &read_out;
    mip::Timestamp timestamp;
    uint8_t PVT_message[PVT_message_size];

    printf("Sensor is configured... waiting for filter to initialize...\n");

    while(running) { 
        std::unique_ptr<UBlox_PVT_Message> ublox_message;
        bool pvt_message_found = false;
        bool pvt_message_overflow = false;
        // Poll ublox receiver for PVT message ... 
        if (!utils_ublox->connection->recvFromDevice(ublox_message_bytes, max_length, wait_time, length_out, &timestamp)) {
            exit_gracefully("ERROR: Error reading from serial port");
        }

        for (int i = buffer_length - 1 - 4; i >= 0; i--) {
            // look for 0xB5
            if (ublox_message_bytes[i] != 0xB5)
                continue;
            if (ublox_message_bytes[i+1] != 0x62)
                continue;
            if (ublox_message_bytes[i+2] != 0x01)
                continue;
            if (ublox_message_bytes[i+3] != 0x07)
                continue;

            // If there are 100 bytes in buffer after (0xB5) is found and there were at least 100 bytes read
            if (i + (PVT_message_size - 1) < buffer_length && (*length_out - i) >= PVT_message_size ) {
                
                // Extract payload
                int payload_index = 0;
                
                for (int z = i; z < PVT_message_size; z++) {
                    PVT_message[payload_index] = ublox_message_bytes[z];
                    payload_index += 1;
                }
                // If checksum valid
                if (verify_checksum(PVT_message)) {
                   ublox_message = parse_PVT_ublox_message(PVT_message); 
                   // send parsed ublox_message to CV7
                   break;
                }
                else {
                    printf("Found packet, failed checksum verification");
                    break;
                }
            }

            size_t start_index = *length_out - i;
            size_t package_bytes_remaining = PVT_message_size - start_index;

            while (*length_out == package_bytes_remaining) {
                std::memmove(ublox_message_bytes, ublox_message_bytes + start_index, package_bytes_remaining);

                if (!utils_ublox->connection->recvFromDevice(&ublox_message_bytes[start_index], package_bytes_remaining, wait_time, length_out, &timestamp))
                    exit_gracefully("ERROR: Error reading from serial port");
                
                start_index += *length_out;
                package_bytes_remaining = PVT_message_size - start_index;
            }

            for (int i = 0; i < payload_size; i++) {
                PVT_message[i] = ublox_message_bytes[i];
            }

            // check checksum
            if (verify_checksum(PVT_message)) {
                
                ublox_message = parse_PVT_ublox_message(PVT_message);
                // send parsed ublox_message to CV7
                break;
            }
            // else then I clear ublox_message bytes and break;
            std::memset(ublox_message_bytes, 0, sizeof(ublox_message_bytes));
            break;
        }
        

        device->update();

        //Check for full nav filter state transition
        if((!filter_state_full_nav) && (filter_status.filter_state == data_filter::FilterMode::FULL_NAV))
        {
            printf("NOTE: Filter has entered full navigation mode.\n");
            filter_state_full_nav = true;
        }

        // Check that enough time has elapsed to send a new measurement update
        mip::Timestamp current_timestamp = getCurrentTimestamp();
        mip::Timestamp elapsed_time_from_last_measurement_update = current_timestamp - prev_measurement_update_timestamp;
        mip::Timestamp elapsed_time_from_last_message_print = current_timestamp - prev_print_timestamp;

        if (elapsed_time_from_last_measurement_update > 500 && pvt_message_found)
        {
            // Use measurement time of arrival for timestamping method
            commands_aiding::Time external_measurement_time;
            external_measurement_time.timebase = commands_aiding::Time::Timebase::TIME_OF_ARRIVAL;

            // External position command
            if(!ublox_message->lat_lon_valid_flag) {
               if(commands_aiding::llhPos(*device, external_measurement_time, gnss_antenna_sensor_id, ublox_message->latitude, ublox_message->longitude, ublox_message->height_above_ellipsoid, ublox_message->llh_uncertainty, 1) != CmdResult::ACK_OK)
                printf("WARNING: Failed to send external position to CV7-INS\n"); 
            }
            else
                printf("WARNING: Invalid lon, lat, height and hMSL from ublox receiver");

            // External global velocity command
            if(commands_aiding::nedVel(*device, external_measurement_time, gnss_antenna_sensor_id, ublox_message->ned_velocity, ublox_message->ned_velocity_uncertainty, 1) != CmdResult::ACK_OK)
                printf("WARNING: Failed to send external NED velocity to CV7-INS\n");
        }

        running = !should_exit();
    }

    exit_gracefully("Example Completed Successfully.");

}

////////////////////////////////////////////////////////////////////////////////
// Verify UBX-NAV-PVT Checksum
////////////////////////////////////////////////////////////////////////////////

bool verify_checksum(uint8_t *data) {
    unsigned i, j;
    uint8_t a, b;

    j = ((unsigned)data[4] + ((unsigned)data[5] << 8) + 6);

    a = 0;
    b = 0;

    for(i=2; i<j; i++) {
        a += data[i];
        b += a;
    }

    if (a == data[i+0] && b == data[i+1])
        return true;
    
    return false;
}

////////////////////////////////////////////////////////////////////////////////
// Parse UBX-NAV-PVT Message
////////////////////////////////////////////////////////////////////////////////

std::unique_ptr<UBlox_PVT_Message> parse_PVT_ublox_message(const uint8_t ublox_message_bytes[]) {

    // TODO: Needs some refactoring but works!!

    std::unique_ptr<UBlox_PVT_Message> ublox_message = std::make_unique<UBlox_PVT_Message>();
    int payload_index = 0;

    // Check header
    if (ublox_message_bytes[0] != 0xB5 || ublox_message_bytes[1] != 0x62)
        exit_gracefully("ERROR: Incorrect message header");

    // Check class and ID to make sure its a UBX-NAV-PVT message
    if (ublox_message_bytes[2] != 0x01 || ublox_message_bytes[3] != 0x07)
        exit_gracefully("ERROR: Not a UBX-NAV-PVT message");

    // Check payload length
    if (ublox_message_bytes[4] != 0x5C)
        exit_gracefully("ERROR: Incorrect payload size for PVT message");
    
    payload_size = ublox_message_bytes[4];
    uint8_t payload[payload_size];

    // Extract payload
    for (int i = header_size; i < (PVT_message_size - checksum_size); i++) {
         payload[payload_index] = ublox_message_bytes[i];
         payload_index += 1;
    }

    ublox_message->iTOW = parse_4_bytes(payload, PAYLOAD_PART_MAP["iTOW"].start_index);

    ublox_message->utc_year = parse_2_bytes(payload, PAYLOAD_PART_MAP["YEAR"].start_index);
    
    ublox_message->utc_month = payload[PAYLOAD_PART_MAP["MONTH"].start_index];

    ublox_message->utc_day = payload[PAYLOAD_PART_MAP["DAY"].start_index];

    ublox_message->utc_hour = payload[PAYLOAD_PART_MAP["HOUR"].start_index];

    ublox_message->utc_minute = payload[PAYLOAD_PART_MAP["MIN"].start_index];

    ublox_message->utc_second = payload[PAYLOAD_PART_MAP["SEC"].start_index];

    ublox_message->time_valid_flag = payload[PAYLOAD_PART_MAP["VALID"].start_index];

    ublox_message->utc_time_accuracy = parse_4_bytes(payload, PAYLOAD_PART_MAP["T_ACC"].start_index);

    ublox_message->nano_second = parse_4_bytes(payload, PAYLOAD_PART_MAP["NANO_SEC"].start_index); 
    
    ublox_message->longitude = parse_long_lat(payload, PAYLOAD_PART_MAP["LON"].start_index);

    ublox_message->latitude = parse_long_lat(payload, PAYLOAD_PART_MAP["LAT"].start_index);

    ublox_message->height_above_ellipsoid = parse_4_bytes(payload, PAYLOAD_PART_MAP["HEIGHT"].start_index);
    convert_mm_to_m(ublox_message->height_above_ellipsoid);

    // TODO: Make sure UBlox values are mapped correctly here
    ublox_message->llh_uncertainty[0] = parse_4_bytes(payload, PAYLOAD_PART_MAP["V_ACC"].start_index);
    convert_mm_to_m(ublox_message->llh_uncertainty[0]);

    ublox_message->llh_uncertainty[1] = parse_4_bytes(payload, PAYLOAD_PART_MAP["H_ACC"].start_index);
    convert_mm_to_m(ublox_message->llh_uncertainty[1]);

    ublox_message->llh_uncertainty[2] = parse_4_bytes(payload, PAYLOAD_PART_MAP["V_ACC"].start_index);
    convert_mm_to_m(ublox_message->llh_uncertainty[2]);

    ublox_message->ned_velocity[0] = parse_4_bytes(payload, PAYLOAD_PART_MAP["VEL_N"].start_index);
    convert_mm_to_m(ublox_message->ned_velocity[0]);
    
    ublox_message->ned_velocity[1] = parse_4_bytes(payload, PAYLOAD_PART_MAP["VEL_E"].start_index);
    convert_mm_to_m(ublox_message->ned_velocity[1]);
    
    ublox_message->ned_velocity[2] = parse_4_bytes(payload, PAYLOAD_PART_MAP["VEL_D"].start_index);
    convert_mm_to_m(ublox_message->ned_velocity[2]);
    
    ublox_message->heading_of_motion_2d = parse_4_bytes(payload, PAYLOAD_PART_MAP["HEAD_MOT"].start_index);

    ublox_message->heading_accuracy = parse_4_bytes(payload, PAYLOAD_PART_MAP["HEAD_ACC"].start_index) * 1e-5;
    convert_degrees_to_radians(ublox_message->heading_accuracy);

    ublox_message->lat_lon_valid_flag = payload[PAYLOAD_PART_MAP["FLAGS_3"].start_index];

    ublox_message->heading_of_vehicle = parse_4_bytes(payload, PAYLOAD_PART_MAP["HEAD_VEH"].start_index);
    convert_degrees_to_radians(ublox_message->heading_of_vehicle);

    return ublox_message;
}


////////////////////////////////////////////////////////////////////////////////
// Parse 2 bytes
////////////////////////////////////////////////////////////////////////////////

float parse_2_bytes(const uint8_t payload[92], int start_index) { 
        return (float)(*((int16_t *)&payload[start_index]));
}

////////////////////////////////////////////////////////////////////////////////
// Parse 4 bytes
////////////////////////////////////////////////////////////////////////////////

double parse_4_bytes(const uint8_t payload[92], int start_index) { 
        return (double)(*((int32_t *)&payload[start_index]));
}

////////////////////////////////////////////////////////////////////////////////
// Print device information
////////////////////////////////////////////////////////////////////////////////

double parse_long_lat(const uint8_t payload[92], int start_index) {
    return (parse_4_bytes(payload, start_index) * 1e-7);
}

template<typename T>
void convert_mm_to_m(T& value) {
    value = value / 1000.0;
}

template<typename T>
void convert_degrees_to_radians(T& degrees) {
   degrees = degrees * (M_PI / 180.0);
}

////////////////////////////////////////////////////////////////////////////////
// Print device information
////////////////////////////////////////////////////////////////////////////////

void print_device_information(const commands_base::BaseDeviceInfo& device_info)
{
    printf("Connected to:\n");

    auto print_info = [](const char* name, const char info[16])
    {
        char msg[17] = {0};
        std::strncpy(msg, info, 16);
        printf("  %s%s\n", name, msg);
    };

    print_info("Model name:       ", device_info.model_name);
    print_info("Model number:     ", device_info.model_number);
    print_info("Serial Number:    ", device_info.serial_number);
    print_info("Device Options:   ", device_info.device_options);
    print_info("Lot Number:       ", device_info.lot_number);

    printf(  "  Firmware version:           %d.%d.%d\n\n",
             (device_info.firmware_version / 1000),
             (device_info.firmware_version / 100) % 10,
             (device_info.firmware_version / 1)   % 100
    );
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

#ifdef _WIN32
    int dummy = getchar();
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


