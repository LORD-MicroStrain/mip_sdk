/////////////////////////////////////////////////////////////////////////////
//
// CV7_Example.c
//
// C Example set-up program for the CV7
//
// This example shows a typical setup for the CV7 sensor using C.
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

#include <mip_all.h>


////////////////////////////////////////////////////////////////////////////////
// Global Variables
////////////////////////////////////////////////////////////////////////////////

const unsigned int baudrate = 9600;
const unsigned int monitorBaudrate = 9600;
// TODO: Change serial number to match device setup
// Serial connections should not be on Serial0 to avoid corruptions when uploading sketches
HardwareSerial* serialConnection = &Serial2;
unsigned long start_time;

int port = -1;
uint8_t parse_buffer[64];
mip::C::mip_interface device;

mip::C::timeout_type parseTimeout;
mip::C::timeout_type monitorParseTimeout;

//Sensor-to-vehicle frame transformation (Euler Angles)
float sensor_to_vehicle_transformation_euler[3] = {0.0, 0.0, 0.0};

//Device data stores
mip::C::mip_shared_gps_timestamp_data sensor_gps_time;
mip::C::mip_sensor_scaled_accel_data  sensor_accel;
mip::C::mip_sensor_scaled_gyro_data   sensor_gyro;
mip::C::mip_sensor_scaled_mag_data    sensor_mag;

mip::C::mip_shared_gps_timestamp_data filter_gps_time;
mip::C::mip_filter_status_data        filter_status;
mip::C::mip_filter_euler_angles_data  filter_euler_angles;

bool filter_state_ahrs = false;

const uint8_t FILTER_ROLL_EVENT_ACTION_ID  = 1;
const uint8_t FILTER_PITCH_EVENT_ACTION_ID = 2;

mip::C::mip_cmd_result cmd_result;


////////////////////////////////////////////////////////////////////////////////
// Function Prototypes
////////////////////////////////////////////////////////////////////////////////

//Required MIP interface user-defined functions
mip::C::timestamp_type get_current_timestamp();

void exit_gracefully(const char *message);
bool should_exit();

// Used to forward raw data from Serial Monitor
void forwardSerialMonitor();


////////////////////////////////////////////////////////////////////////////////
// Setup Function
////////////////////////////////////////////////////////////////////////////////

void setup()
{

    //
    //Process arguments
    //

    Serial.begin(monitorBaudrate);
    if (!Serial)
        exit_gracefully("ERROR: Could not open serial monitor connection!");


    //
    //Calculate the timeout for data parsing
    //

    parseTimeout = mip::C::mip_timeout_from_baudrate(baudrate);
    monitorParseTimeout = mip::C::mip_timeout_from_baudrate(monitorBaudrate);

    //
    //Get the program start time
    //

    start_time = millis();

    Serial.println("Connecting to and configuring sensor.");
    Serial.flush();

    //
    //Open the device port
    //

    serialConnection->begin(baudrate);
    if (!serialConnection)
        exit_gracefully("ERROR: Could not open connection to device!");

    //
    //Set the parsing timeout for the serial connections
    //
    // Serial.setTimeout(monitorParseTimeout);
    // serialConnection->setTimeout(parseTimeout);
    Serial.setTimeout(2000);
    serialConnection->setTimeout(2000);


    //
    //Initialize the MIP interface
    //

    mip::C::mip_interface_init(&device, parse_buffer, sizeof(parse_buffer), parseTimeout, 1000);


    //
    //Ping the device (note: this is good to do to make sure the device is present)
    //

    cmd_result = mip::C::mip_base_ping(&device);
    char message[256];
    sprintf(message, "Result: %d", cmd_result);
    Serial.println(message);
    Serial.flush();
    // if (mip_base_ping(&device) != mip::C::MIP_ACK_OK)
    if (cmd_result != mip::C::MIP_ACK_OK)
        exit_gracefully("ERROR: Could not ping the device!");


    //
    //Idle the device (note: this is good to do during setup)
    //

    cmd_result = mip::C::mip_base_set_idle(&device);
    if (cmd_result != mip::C::MIP_ACK_OK)
        exit_gracefully("ERROR: Could not set the device to idle!");


    //
    //Load the device default settings (so the device is in a known state)
    //

    cmd_result = mip::C::mip_3dm_default_device_settings(&device);
    if (cmd_result != mip::C::MIP_ACK_OK)
        exit_gracefully("ERROR: Could not load default device settings!");


    //
    //Setup Sensor data format to 100 Hz
    //

    uint16_t sensor_base_rate;

    //Note: Querying the device base rate is only one way to calculate the descriptor decimation.
    //We could have also set it directly with information from the datasheet (shown in GNSS setup).

    cmd_result = mip::C::mip_3dm_get_base_rate(&device, mip::C::MIP_SENSOR_DATA_DESC_SET, &sensor_base_rate);
    if (cmd_result != mip::C::MIP_ACK_OK)
        exit_gracefully("ERROR: Could not get sensor base rate format!");

    const uint16_t sensor_sample_rate = 100; // Hz
    const uint16_t sensor_decimation = sensor_base_rate / sensor_sample_rate;

    const mip::C::mip_descriptor_rate sensor_descriptors[4] = {
        { mip::C::MIP_DATA_DESC_SHARED_GPS_TIME,     sensor_decimation },
        { mip::C::MIP_DATA_DESC_SENSOR_ACCEL_SCALED, sensor_decimation },
        { mip::C::MIP_DATA_DESC_SENSOR_GYRO_SCALED,  sensor_decimation },
        { mip::C::MIP_DATA_DESC_SENSOR_MAG_SCALED,   sensor_decimation },
    };

    cmd_result = mip::C::mip_3dm_write_message_format(&device, mip::C::MIP_SENSOR_DATA_DESC_SET, 4, sensor_descriptors);
    if (cmd_result != mip::C::MIP_ACK_OK)
        exit_gracefully("ERROR: Could not set sensor message format!");


    //
    //Setup FILTER data format
    //

    uint16_t filter_base_rate;

    cmd_result = mip::C::mip_3dm_get_base_rate(&device, mip::C::MIP_FILTER_DATA_DESC_SET, &filter_base_rate);
    if (cmd_result != mip::C::MIP_ACK_OK)
        exit_gracefully("ERROR: Could not get filter base rate format!");

    const uint16_t filter_sample_rate = 100; // Hz
    const uint16_t filter_decimation = filter_base_rate / filter_sample_rate;

    const mip::C::mip_descriptor_rate filter_descriptors[3] = {
        { mip::C::MIP_DATA_DESC_SHARED_GPS_TIME,         filter_decimation },
        { mip::C::MIP_DATA_DESC_FILTER_FILTER_STATUS,    filter_decimation },
        { mip::C::MIP_DATA_DESC_FILTER_ATT_EULER_ANGLES, filter_decimation },
    };

    cmd_result = mip::C::mip_3dm_write_message_format(&device, mip::C::MIP_FILTER_DATA_DESC_SET, 3, filter_descriptors);
    if (cmd_result != mip::C::MIP_ACK_OK)
        exit_gracefully("ERROR: Could not set filter message format!");


    //
    //Setup the sensor to vehicle transformation
    //

    cmd_result = mip::C::mip_3dm_write_sensor_2_vehicle_transform_euler(&device, sensor_to_vehicle_transformation_euler[0], sensor_to_vehicle_transformation_euler[1], sensor_to_vehicle_transformation_euler[2]);
    if (cmd_result != mip::C::MIP_ACK_OK)
        exit_gracefully("ERROR: Could not set sensor-to-vehicle transformation!");


    //
    //Setup the filter aiding measurements (GNSS position/velocity and dual antenna [aka gnss heading])
    //

    cmd_result = mip::C::mip_filter_write_aiding_measurement_enable(&device, mip::C::MIP_FILTER_AIDING_MEASUREMENT_ENABLE_COMMAND_AIDING_SOURCE_MAGNETOMETER, true);
    if (cmd_result != mip::C::MIP_ACK_OK)
        exit_gracefully("ERROR: Could not set filter aiding measurement enable!");


    //
    //Reset the filter (note: this is good to do after filter setup is complete)
    //

    cmd_result = mip::C::mip_filter_reset(&device);
    if (cmd_result != mip::C::MIP_ACK_OK)
        exit_gracefully("ERROR: Could not reset the filter!");


    //
    // Register data callbacks
    //

    //Sensor Data
    mip::C::mip_dispatch_handler sensor_data_handlers[4];

    mip::C::mip_interface_register_extractor(&device, &sensor_data_handlers[0], mip::C::MIP_SENSOR_DATA_DESC_SET, mip::C::MIP_DATA_DESC_SHARED_GPS_TIME,     mip::C::extract_mip_shared_gps_timestamp_data_from_field, &sensor_gps_time);
    mip::C::mip_interface_register_extractor(&device, &sensor_data_handlers[1], mip::C::MIP_SENSOR_DATA_DESC_SET, mip::C::MIP_DATA_DESC_SENSOR_ACCEL_SCALED, mip::C::extract_mip_sensor_scaled_accel_data_from_field,  &sensor_accel);
    mip::C::mip_interface_register_extractor(&device, &sensor_data_handlers[2], mip::C::MIP_SENSOR_DATA_DESC_SET, mip::C::MIP_DATA_DESC_SENSOR_GYRO_SCALED,  mip::C::extract_mip_sensor_scaled_gyro_data_from_field,   &sensor_gyro);
    mip::C::mip_interface_register_extractor(&device, &sensor_data_handlers[3], mip::C::MIP_SENSOR_DATA_DESC_SET, mip::C::MIP_DATA_DESC_SENSOR_MAG_SCALED,   mip::C::extract_mip_sensor_scaled_mag_data_from_field,    &sensor_mag);

    //Filter Data
    mip::C::mip_dispatch_handler filter_data_handlers[4];

    mip::C::mip_interface_register_extractor(&device, &filter_data_handlers[0], mip::C::MIP_FILTER_DATA_DESC_SET, mip::C::MIP_DATA_DESC_SHARED_GPS_TIME,         mip::C::extract_mip_shared_gps_timestamp_data_from_field, &filter_gps_time);
    mip::C::mip_interface_register_extractor(&device, &filter_data_handlers[1], mip::C::MIP_FILTER_DATA_DESC_SET, mip::C::MIP_DATA_DESC_FILTER_FILTER_STATUS,    mip::C::extract_mip_filter_status_data_from_field,        &filter_status);
    mip::C::mip_interface_register_extractor(&device, &filter_data_handlers[2], mip::C::MIP_FILTER_DATA_DESC_SET, mip::C::MIP_DATA_DESC_FILTER_ATT_EULER_ANGLES, mip::C::extract_mip_filter_euler_angles_data_from_field,  &filter_euler_angles);

    //
    //Resume the device
    //

    cmd_result = mip::C::mip_base_resume(&device);
    if (cmd_result != mip::C::MIP_ACK_OK)
        exit_gracefully("ERROR: Could not resume the device!");

    Serial.println("Sensor is configured... waiting for filter to enter AHRS mode.");
    Serial.flush();
}

//
//Main Loop: Update the interface and process data
//

void loop()
{
    Serial.println("Begin loop.");
    Serial.flush();

    mip::C::timestamp_type prev_print_timestamp = get_current_timestamp();
    mip::C::mip_interface_update(&device, false);

    //Check Filter State
    if ((!filter_state_ahrs) && (filter_status.filter_state == mip::C::MIP_FILTER_MODE_AHRS))
    {
        Serial.print("NOTE: Filter has entered AHRS mode.\n");
        Serial.flush();
        filter_state_ahrs = true;
    }

    Serial.flush();

    //Once in full nav, print out data at 10 Hz
    if (filter_state_ahrs)
    {
        mip::C::timestamp_type curr_time = get_current_timestamp();

        if (curr_time - prev_print_timestamp >= 100)
        {
            char message[256];
            sprintf(message, "TOW = %f: ATT_EULER = [%f %f %f]\n",
                    filter_gps_time.tow, filter_euler_angles.roll, filter_euler_angles.pitch, filter_euler_angles.yaw);
            Serial.println(message);
            Serial.flush();

            prev_print_timestamp = curr_time;
        }
    }

    forwardSerialMonitor();
}


////////////////////////////////////////////////////////////////////////////////
// MIP Interface Time Access Function
////////////////////////////////////////////////////////////////////////////////

mip::C::timestamp_type get_current_timestamp()
{
    unsigned long curr_time = millis();

    return (mip::C::timestamp_type)(curr_time - start_time);
}


////////////////////////////////////////////////////////////////////////////////
// MIP Interface User Recv Data Function
////////////////////////////////////////////////////////////////////////////////

// bool mip::C::mip_interface_user_recv_from_device(mip::C::mip_interface* device, uint8_t* buffer, size_t max_length, size_t* out_length, mip::C::timestamp_type* timestamp_out)
// {
//     serialConnection->flush();
//     max_length = min(serialConnection->available(), max_length);

//     size_t incomingDataCount = serialConnection->available();

//     if (max_length > 0)
//     {
//         Serial.println("Receive from device.");
//         Serial.flush();

//         *out_length = serialConnection->readBytes(buffer, max_length);

//         Serial.flush();
//         for (size_t i = 0; i < max_length; ++i)
//         {
//             Serial.print(buffer[i], HEX);
//             Serial.flush();
//         }
//         Serial.println();
//         Serial.flush();

//         return true;
//     }

//     return false;
// }

bool mip::C::mip_interface_user_recv_from_device(mip::C::mip_interface* device, uint8_t* buffer, size_t max_length, size_t* out_length, mip::C::timestamp_type* timestamp_out)
{
    Serial.println("Receiving packet.");
    Serial.flush();
    const char headerSync[2] = { 0x75, 0x65 };

    enum PacketHeaderBitmask : uint8_t
    {
        NONE           = 0x00,
        SYNC1          = 0x01,
        SYNC2          = 0x02,
        DESCRIPTOR_SET = 0x04,
        PAYLOAD_LENGTH = 0x08
    };
    //  Bitflag used to track packet header being found
    static uint8_t foundPacketHeader = 0x00;

    static mip::C::packet_length readCount = 0x00;
    static mip::C::packet_length payloadLength = 0x00;
    static bool recvInProgress = false;

    char receivedByte;
    bool finishedReceiving = false;

    size_t startTime = millis();
    size_t currentTime = startTime;

    // Make sure out_length is reset on consecutive reads until finished reading
    *out_length = 0;

    // while (!finishedReceiving)
    // {
        // Read the packet only if the MIP checksum is found
        while (serialConnection->available() > 0)
        {
            receivedByte = serialConnection->read();

            // Start receiving data
            if (recvInProgress)
            {
                *out_length = 4 + serialConnection->readBytesUntil(NULL, buffer+readCount, payloadLength + 2);
                // Finish reading the incoming bytes until the end of the packet
                // Full packet consists of 4-byte header, payload length, and a 2-byte checksum
                if (readCount < payloadLength + 0x06)
                {
                    // Add byte to buffer
                    buffer[readCount] = receivedByte;

                    // Increment read length
                    ++readCount;
                }
                // Finished receiving the packet
                else
                {
                    finishedReceiving = true;
                    recvInProgress = false;

                    // Set the final packet length
                    *out_length = payloadLength + 0x06;
                }
            }
            // Already found response descriptor, processing packet length
            else if ((foundPacketHeader & DESCRIPTOR_SET) == DESCRIPTOR_SET)
            {
                // Set the payload length
                payloadLength = receivedByte;

                // Add byte to buffer
                buffer[readCount] = receivedByte;

                // Increment read length
                ++readCount;

                // Add response descriptor flag
                foundPacketHeader |= PAYLOAD_LENGTH;

                // Continue receiving packets until the end of the packet
                recvInProgress = true;
            }
            // Already found full checksum, process response descriptor
            else if ((foundPacketHeader & SYNC2) == SYNC2)
            {
                // Add byte to buffer
                buffer[readCount] = receivedByte;

                // Increment read length
                ++readCount;

                // Add response descriptor flag
                foundPacketHeader |= DESCRIPTOR_SET;
            }
            // Found second header sync byte after first
            else if ((foundPacketHeader & SYNC1) == SYNC1 && receivedByte == headerSync[1])
            {
                // Add byte to buffer
                buffer[readCount] = receivedByte;

                // Increment read length
                ++readCount;

                // Add second checksum flag
                foundPacketHeader |= SYNC2;
            }
            // Found first header sync byte
            else if (foundPacketHeader == NONE && receivedByte == headerSync[0])
            {
                // Add byte to buffer
                buffer[readCount] = receivedByte;

                // Increment read length
                ++readCount;

                // Add first checksum flag
                foundPacketHeader |= SYNC1;
            }
            // Not a valid packet. Reset everything
            else
            {
                foundPacketHeader = 0x00;
                readCount = 0x00;
                payloadLength = 0x00;
                recvInProgress = false;
            }
        }

        // if (!finishedReceiving)
        // {
        //     Serial.println("Waiting for recived bytes.");
        //     Serial.flush();

        //     for (size_t i = 0; i < readCount; ++i)
        //     {
        //         Serial.print(buffer[i], HEX);
        //         Serial.flush();
        //     }
        //     serialEventRun();
        //     continue;
        // }

        // Not finished receiving the packet
        if (*out_length == 0x00)
        {
            return false;
        }

        Serial.println("Received packet.");
        Serial.flush();

        // Print the packet
        for (size_t writeIndex = 0;
             writeIndex < *out_length;
             ++writeIndex)
        {
            // Whitespace separator for easier readability
            if (writeIndex != 0)
            {
                Serial.print(' ');
                Serial.flush();
            }

            Serial.print(buffer[writeIndex], HEX);
            Serial.flush();
        }
        Serial.println("\n");
        // Wait for all bytes to finish sending
        Serial.flush();

        // Reset the static variables
        foundPacketHeader = 0x00;
        readCount = 0x00;
        payloadLength = 0x00;

        return true;
    }
// }


////////////////////////////////////////////////////////////////////////////////
// MIP Interface User Send Data Function
////////////////////////////////////////////////////////////////////////////////

bool mip::C::mip_interface_user_send_to_device(mip::C::mip_interface* device, const uint8_t* data, size_t length)
{
    if (data)
    {
        Serial.println("Sending to device.");
        Serial.flush();

        serialConnection->write(data, length);
        serialConnection->flush();

        for (size_t i = 0;
             i < length;
             ++i)
        {
            if (i > 0)
            {
                Serial.print(' ');
                Serial.flush();
            }
            // Serial.print(data[i], HEX);
            char temp[16];
            sprintf(temp, "%2X", data[i]);
            Serial.print(temp);
            Serial.flush();
        }
        Serial.println("\n");
        Serial.flush();

        return true;
    }

    return false;
}

void forwardSerialMonitor()
{
    size_t packetSize = Serial.available();

    // Received raw data from Serial Monitor
    if (packetSize > 0)
    {
        byte buffer[packetSize];
        Serial.readBytes(buffer, packetSize);
        Serial.println("Forwarding raw data.");
        Serial.flush();

        serialConnection->write(buffer, packetSize);
        serialConnection->flush();
    }
}


////////////////////////////////////////////////////////////////////////////////
// Exit Function
////////////////////////////////////////////////////////////////////////////////

void exit_gracefully(const char *message)
{
    Serial.println("Exiting gracefully.");
    Serial.flush();

    if (message)
    {
      Serial.println(message);
      Serial.flush();
    }

    //Close comm port
    // if (serialConnection)
    //     serialConnection->end();

    // if (Serial)
    //   Serial.end();

    // exit(0);
}
