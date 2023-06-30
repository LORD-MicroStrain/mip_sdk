#pragma once

#include "descriptors.h"
#include "../mip_result.h"

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

namespace mip {
class Serializer;

namespace C {
struct mip_interface;
} // namespace C

namespace commands_aiding {

////////////////////////////////////////////////////////////////////////////////
///@addtogroup MipCommands_cpp  MIP Commands [CPP]
///@{
///@defgroup aiding_commands_cpp  Aiding Commands [CPP]
///
///@{

////////////////////////////////////////////////////////////////////////////////
// Descriptors
////////////////////////////////////////////////////////////////////////////////

enum 
{
    DESCRIPTOR_SET         = 0x13,
    
    CMD_FRAME_CONFIG       = 0x01,
    CMD_SENSOR_FRAME_MAP   = 0x02,
    CMD_LOCAL_FRAME        = 0x03,
    CMD_ECHO_CONTROL       = 0x1F,
    CMD_POS_LOCAL          = 0x20,
    CMD_POS_ECEF           = 0x21,
    CMD_POS_LLH            = 0x22,
    CMD_HEIGHT_ABS         = 0x23,
    CMD_HEIGHT_REL         = 0x24,
    CMD_VEL_ECEF           = 0x28,
    CMD_VEL_NED            = 0x29,
    CMD_VEL_ODOM           = 0x2A,
    CMD_WHEELSPEED         = 0x2B,
    CMD_HEADING_TRUE       = 0x31,
    CMD_DELTA_POSITION     = 0x38,
    CMD_DELTA_ATTITUDE     = 0x39,
    CMD_LOCAL_ANGULAR_RATE = 0x3A,
    
    REPLY_SENSOR_FRAME_MAP = 0x82,
    REPLY_FRAME_CONFIG     = 0x81,
    REPLY_ECHO_CONTROL     = 0x9F,
};

////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////

struct Time
{
    enum class Timebase : uint8_t
    {
        INTERNAL_REFERENCE = 1,  ///<  Internal reference time from the device.
        EXTERNAL_TIME      = 2,  ///<  External reference time from PPS.
    };
    
    Timebase timebase = static_cast<Timebase>(0); ///< Timebase reference, e.g. internal, external, GPS, UTC, etc.
    uint8_t reserved = 0; ///< Reserved, set to 0x01.
    uint64_t nanoseconds = 0; ///< Nanoseconds since the timebase epoch.
    
};
void insert(Serializer& serializer, const Time& self);
void extract(Serializer& serializer, Time& self);


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_aiding_sensor_frame_mapping  (0x13,0x02) Sensor Frame Mapping [CPP]
///
///@{

struct SensorFrameMapping
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_aiding::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_aiding::CMD_SENSOR_FRAME_MAP;
    
    static const bool HAS_WRITE_FUNCTION = true;
    static const bool HAS_READ_FUNCTION = true;
    static const bool HAS_SAVE_FUNCTION = true;
    static const bool HAS_LOAD_FUNCTION = true;
    static const bool HAS_RESET_FUNCTION = true;
    
    FunctionSelector function = static_cast<FunctionSelector>(0);
    uint8_t sensor_id = 0; ///< Sensor ID to configure. Cannot be 0.
    uint8_t frame_id = 0; ///< Frame ID to assign to the sensor. Defaults to 1.
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_aiding::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_aiding::REPLY_SENSOR_FRAME_MAP;
        
        uint8_t sensor_id = 0; ///< Sensor ID to configure. Cannot be 0.
        uint8_t frame_id = 0; ///< Frame ID to assign to the sensor. Defaults to 1.
        
    };
};
void insert(Serializer& serializer, const SensorFrameMapping& self);
void extract(Serializer& serializer, SensorFrameMapping& self);

void insert(Serializer& serializer, const SensorFrameMapping::Response& self);
void extract(Serializer& serializer, SensorFrameMapping::Response& self);

CmdResult writeSensorFrameMapping(C::mip_interface& device, uint8_t sensorId, uint8_t frameId);
CmdResult readSensorFrameMapping(C::mip_interface& device, uint8_t* sensorIdOut, uint8_t* frameIdOut);
CmdResult saveSensorFrameMapping(C::mip_interface& device);
CmdResult loadSensorFrameMapping(C::mip_interface& device);
CmdResult defaultSensorFrameMapping(C::mip_interface& device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_aiding_reference_frame  (0x13,0x01) Reference Frame [CPP]
///
///@{

struct ReferenceFrame
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_aiding::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_aiding::CMD_FRAME_CONFIG;
    
    static const bool HAS_WRITE_FUNCTION = true;
    static const bool HAS_READ_FUNCTION = true;
    static const bool HAS_SAVE_FUNCTION = true;
    static const bool HAS_LOAD_FUNCTION = true;
    static const bool HAS_RESET_FUNCTION = true;
    
    enum class Format : uint8_t
    {
        EULER      = 1,  ///<  Translation vector followed by euler angles (roll, pitch, yaw).
        QUATERNION = 2,  ///<  Translation vector followed by quaternion (w, x, y, z).
    };
    
    FunctionSelector function = static_cast<FunctionSelector>(0);
    uint8_t frame_id = 0; ///< Reference frame number. Cannot be 0.
    Format format = static_cast<Format>(0); ///< Format of the transformation.
    float translation[3] = {0}; ///< Translation X, Y, and Z.
    float rotation[4] = {0}; ///< Depends on the format parameter. Unused values are ignored.
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_aiding::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_aiding::REPLY_FRAME_CONFIG;
        
        uint8_t frame_id = 0; ///< Reference frame number. Cannot be 0.
        Format format = static_cast<Format>(0); ///< Format of the transformation.
        float translation[3] = {0}; ///< Translation X, Y, and Z.
        float rotation[4] = {0}; ///< Depends on the format parameter. Unused values are ignored.
        
    };
};
void insert(Serializer& serializer, const ReferenceFrame& self);
void extract(Serializer& serializer, ReferenceFrame& self);

void insert(Serializer& serializer, const ReferenceFrame::Response& self);
void extract(Serializer& serializer, ReferenceFrame::Response& self);

CmdResult writeReferenceFrame(C::mip_interface& device, uint8_t frameId, ReferenceFrame::Format format, const float* translation, const float* rotation);
CmdResult readReferenceFrame(C::mip_interface& device, uint8_t* frameIdOut, ReferenceFrame::Format* formatOut, float* translationOut, float* rotationOut);
CmdResult saveReferenceFrame(C::mip_interface& device);
CmdResult loadReferenceFrame(C::mip_interface& device);
CmdResult defaultReferenceFrame(C::mip_interface& device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_aiding_aiding_echo_control  (0x13,0x1F) Aiding Echo Control [CPP]
///
///@{

struct AidingEchoControl
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_aiding::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_aiding::CMD_ECHO_CONTROL;
    
    static const bool HAS_WRITE_FUNCTION = true;
    static const bool HAS_READ_FUNCTION = true;
    static const bool HAS_SAVE_FUNCTION = true;
    static const bool HAS_LOAD_FUNCTION = true;
    static const bool HAS_RESET_FUNCTION = true;
    
    enum class Mode : uint8_t
    {
        SUPPRESS_ACK = 0,  ///<  Suppresses the usual command ack field for aiding messages.
        STANDARD     = 1,  ///<  Normal ack/nack behavior.
        RESPONSE     = 2,  ///<  Echo the data back as a response.
    };
    
    FunctionSelector function = static_cast<FunctionSelector>(0);
    Mode mode = static_cast<Mode>(0); ///< Controls data echoing.
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_aiding::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_aiding::REPLY_ECHO_CONTROL;
        
        Mode mode = static_cast<Mode>(0); ///< Controls data echoing.
        
    };
};
void insert(Serializer& serializer, const AidingEchoControl& self);
void extract(Serializer& serializer, AidingEchoControl& self);

void insert(Serializer& serializer, const AidingEchoControl::Response& self);
void extract(Serializer& serializer, AidingEchoControl::Response& self);

CmdResult writeAidingEchoControl(C::mip_interface& device, AidingEchoControl::Mode mode);
CmdResult readAidingEchoControl(C::mip_interface& device, AidingEchoControl::Mode* modeOut);
CmdResult saveAidingEchoControl(C::mip_interface& device);
CmdResult loadAidingEchoControl(C::mip_interface& device);
CmdResult defaultAidingEchoControl(C::mip_interface& device);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_aiding_ecef_pos  (0x13,0x21) Ecef Pos [CPP]
/// Cartesian vector position aiding command. Coordinates are given in the WGS84 ECEF system.
///
///@{

struct EcefPos
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_aiding::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_aiding::CMD_POS_ECEF;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    struct ValidFlags : Bitfield<ValidFlags>
    {
        enum _enumType : uint16_t
        {
            NONE = 0x0000,
            X    = 0x0002,  ///<  
            Y    = 0x0004,  ///<  
            Z    = 0x0008,  ///<  
            ALL  = 0x000E,
        };
        uint16_t value = NONE;
        
        ValidFlags() : value(NONE) {}
        ValidFlags(int val) : value((uint16_t)val) {}
        operator uint16_t() const { return value; }
        ValidFlags& operator=(uint16_t val) { value = val; return *this; }
        ValidFlags& operator=(int val) { value = val; return *this; }
        ValidFlags& operator|=(uint16_t val) { return *this = value | val; }
        ValidFlags& operator&=(uint16_t val) { return *this = value & val; }
        
        bool x() const { return (value & X) > 0; }
        void x(bool val) { if(val) value |= X; else value &= ~X; }
        bool y() const { return (value & Y) > 0; }
        void y(bool val) { if(val) value |= Y; else value &= ~Y; }
        bool z() const { return (value & Z) > 0; }
        void z(bool val) { if(val) value |= Z; else value &= ~Z; }
        
        bool allSet() const { return value == ALL; }
        void setAll() { value |= ALL; }
    };
    
    Time time; ///< Timestamp of the measurement.
    uint8_t sensor_id = 0; ///< Sensor ID.
    double position[3] = {0}; ///< ECEF position [m].
    float uncertainty[3] = {0}; ///< ECEF position uncertainty [m].
    ValidFlags valid_flags; ///< Valid flags.
    
};
void insert(Serializer& serializer, const EcefPos& self);
void extract(Serializer& serializer, EcefPos& self);

CmdResult ecefPos(C::mip_interface& device, const Time& time, uint8_t sensorId, const double* position, const float* uncertainty, EcefPos::ValidFlags validFlags);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_aiding_llh_pos  (0x13,0x22) Llh Pos [CPP]
/// Geodetic position aiding command. Coordinates are given in WGS84 geodetic latitude, longitude, and height above the ellipsoid.
/// Uncertainty is given in NED coordinates, which are parallel to incremental changes in latitude, longitude, and height.
///
///@{

struct LlhPos
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_aiding::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_aiding::CMD_POS_LLH;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    struct ValidFlags : Bitfield<ValidFlags>
    {
        enum _enumType : uint16_t
        {
            NONE      = 0x0000,
            LATITUDE  = 0x0002,  ///<  
            LONGITUDE = 0x0004,  ///<  
            HEIGHT    = 0x0008,  ///<  
            ALL       = 0x000E,
        };
        uint16_t value = NONE;
        
        ValidFlags() : value(NONE) {}
        ValidFlags(int val) : value((uint16_t)val) {}
        operator uint16_t() const { return value; }
        ValidFlags& operator=(uint16_t val) { value = val; return *this; }
        ValidFlags& operator=(int val) { value = val; return *this; }
        ValidFlags& operator|=(uint16_t val) { return *this = value | val; }
        ValidFlags& operator&=(uint16_t val) { return *this = value & val; }
        
        bool latitude() const { return (value & LATITUDE) > 0; }
        void latitude(bool val) { if(val) value |= LATITUDE; else value &= ~LATITUDE; }
        bool longitude() const { return (value & LONGITUDE) > 0; }
        void longitude(bool val) { if(val) value |= LONGITUDE; else value &= ~LONGITUDE; }
        bool height() const { return (value & HEIGHT) > 0; }
        void height(bool val) { if(val) value |= HEIGHT; else value &= ~HEIGHT; }
        
        bool allSet() const { return value == ALL; }
        void setAll() { value |= ALL; }
    };
    
    Time time; ///< Timestamp of the measurement.
    uint8_t sensor_id = 0; ///< Sensor ID.
    double latitude = 0; ///< [deg]
    double longitude = 0; ///< [deg]
    double height = 0; ///< [m]
    float uncertainty[3] = {0}; ///< NED position uncertainty.
    ValidFlags valid_flags; ///< Valid flags.
    
};
void insert(Serializer& serializer, const LlhPos& self);
void extract(Serializer& serializer, LlhPos& self);

CmdResult llhPos(C::mip_interface& device, const Time& time, uint8_t sensorId, double latitude, double longitude, double height, const float* uncertainty, LlhPos::ValidFlags validFlags);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_aiding_ecef_vel  (0x13,0x28) Ecef Vel [CPP]
/// ECEF velocity aiding command. Coordinates are given in the WGS84 ECEF frame.
///
///@{

struct EcefVel
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_aiding::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_aiding::CMD_VEL_ECEF;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    struct ValidFlags : Bitfield<ValidFlags>
    {
        enum _enumType : uint16_t
        {
            NONE = 0x0000,
            X    = 0x0002,  ///<  
            Y    = 0x0004,  ///<  
            Z    = 0x0008,  ///<  
            ALL  = 0x000E,
        };
        uint16_t value = NONE;
        
        ValidFlags() : value(NONE) {}
        ValidFlags(int val) : value((uint16_t)val) {}
        operator uint16_t() const { return value; }
        ValidFlags& operator=(uint16_t val) { value = val; return *this; }
        ValidFlags& operator=(int val) { value = val; return *this; }
        ValidFlags& operator|=(uint16_t val) { return *this = value | val; }
        ValidFlags& operator&=(uint16_t val) { return *this = value & val; }
        
        bool x() const { return (value & X) > 0; }
        void x(bool val) { if(val) value |= X; else value &= ~X; }
        bool y() const { return (value & Y) > 0; }
        void y(bool val) { if(val) value |= Y; else value &= ~Y; }
        bool z() const { return (value & Z) > 0; }
        void z(bool val) { if(val) value |= Z; else value &= ~Z; }
        
        bool allSet() const { return value == ALL; }
        void setAll() { value |= ALL; }
    };
    
    Time time; ///< Timestamp of the measurement.
    uint8_t sensor_id = 0; ///< Sensor ID.
    float velocity[3] = {0}; ///< ECEF velocity [m/s].
    float uncertainty[3] = {0}; ///< ECEF velocity uncertainty [m/s].
    ValidFlags valid_flags; ///< Valid flags.
    
};
void insert(Serializer& serializer, const EcefVel& self);
void extract(Serializer& serializer, EcefVel& self);

CmdResult ecefVel(C::mip_interface& device, const Time& time, uint8_t sensorId, const float* velocity, const float* uncertainty, EcefVel::ValidFlags validFlags);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_aiding_ned_vel  (0x13,0x29) Ned Vel [CPP]
/// NED velocity aiding command. Coordinates are given in the local North-East-Down frame.
///
///@{

struct NedVel
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_aiding::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_aiding::CMD_VEL_NED;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    struct ValidFlags : Bitfield<ValidFlags>
    {
        enum _enumType : uint16_t
        {
            NONE = 0x0000,
            X    = 0x0002,  ///<  
            Y    = 0x0004,  ///<  
            Z    = 0x0008,  ///<  
            ALL  = 0x000E,
        };
        uint16_t value = NONE;
        
        ValidFlags() : value(NONE) {}
        ValidFlags(int val) : value((uint16_t)val) {}
        operator uint16_t() const { return value; }
        ValidFlags& operator=(uint16_t val) { value = val; return *this; }
        ValidFlags& operator=(int val) { value = val; return *this; }
        ValidFlags& operator|=(uint16_t val) { return *this = value | val; }
        ValidFlags& operator&=(uint16_t val) { return *this = value & val; }
        
        bool x() const { return (value & X) > 0; }
        void x(bool val) { if(val) value |= X; else value &= ~X; }
        bool y() const { return (value & Y) > 0; }
        void y(bool val) { if(val) value |= Y; else value &= ~Y; }
        bool z() const { return (value & Z) > 0; }
        void z(bool val) { if(val) value |= Z; else value &= ~Z; }
        
        bool allSet() const { return value == ALL; }
        void setAll() { value |= ALL; }
    };
    
    Time time; ///< Timestamp of the measurement.
    uint8_t sensor_id = 0; ///< Sensor ID.
    float velocity[3] = {0}; ///< NED velocity [m/s].
    float uncertainty[3] = {0}; ///< NED velocity uncertainty [m/s].
    ValidFlags valid_flags; ///< Valid flags.
    
};
void insert(Serializer& serializer, const NedVel& self);
void extract(Serializer& serializer, NedVel& self);

CmdResult nedVel(C::mip_interface& device, const Time& time, uint8_t sensorId, const float* velocity, const float* uncertainty, NedVel::ValidFlags validFlags);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_aiding_vehicle_fixed_frame_velocity  (0x13,0x2A) Vehicle Fixed Frame Velocity [CPP]
/// Estimate of velocity of the vehicle in the frame associated
/// with the given sensor ID.
///
///@{

struct VehicleFixedFrameVelocity
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_aiding::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_aiding::CMD_VEL_ODOM;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    struct ValidFlags : Bitfield<ValidFlags>
    {
        enum _enumType : uint16_t
        {
            NONE = 0x0000,
            X    = 0x0002,  ///<  
            Y    = 0x0004,  ///<  
            Z    = 0x0008,  ///<  
            ALL  = 0x000E,
        };
        uint16_t value = NONE;
        
        ValidFlags() : value(NONE) {}
        ValidFlags(int val) : value((uint16_t)val) {}
        operator uint16_t() const { return value; }
        ValidFlags& operator=(uint16_t val) { value = val; return *this; }
        ValidFlags& operator=(int val) { value = val; return *this; }
        ValidFlags& operator|=(uint16_t val) { return *this = value | val; }
        ValidFlags& operator&=(uint16_t val) { return *this = value & val; }
        
        bool x() const { return (value & X) > 0; }
        void x(bool val) { if(val) value |= X; else value &= ~X; }
        bool y() const { return (value & Y) > 0; }
        void y(bool val) { if(val) value |= Y; else value &= ~Y; }
        bool z() const { return (value & Z) > 0; }
        void z(bool val) { if(val) value |= Z; else value &= ~Z; }
        
        bool allSet() const { return value == ALL; }
        void setAll() { value |= ALL; }
    };
    
    Time time; ///< Timestamp of the measurement.
    uint8_t sensor_id = 0; ///< Source ID for this estimate ( source_id == 0 indicates this sensor, source_id > 0 indicates an external estimate )
    float velocity[3] = {0}; ///< [m/s]
    float uncertainty[3] = {0}; ///< [m/s] 1-sigma uncertainty (if velocity_uncertainty[i] <= 0, then velocity[i] should be treated as invalid and ingnored)
    ValidFlags valid_flags;
    
};
void insert(Serializer& serializer, const VehicleFixedFrameVelocity& self);
void extract(Serializer& serializer, VehicleFixedFrameVelocity& self);

CmdResult vehicleFixedFrameVelocity(C::mip_interface& device, const Time& time, uint8_t sensorId, const float* velocity, const float* uncertainty, VehicleFixedFrameVelocity::ValidFlags validFlags);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_aiding_true_heading  (0x13,0x31) True Heading [CPP]
///
///@{

struct TrueHeading
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_aiding::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_aiding::CMD_HEADING_TRUE;
    
    static const bool HAS_FUNCTION_SELECTOR = false;
    
    Time time;
    uint8_t sensor_id = 0;
    float heading = 0; ///< Heading in [radians]
    float uncertainty = 0;
    uint16_t valid_flags = 0;
    
};
void insert(Serializer& serializer, const TrueHeading& self);
void extract(Serializer& serializer, TrueHeading& self);

CmdResult trueHeading(C::mip_interface& device, const Time& time, uint8_t sensorId, float heading, float uncertainty, uint16_t validFlags);
///@}
///

///@}
///@}
///
////////////////////////////////////////////////////////////////////////////////
} // namespace commands_aiding
} // namespace mip
