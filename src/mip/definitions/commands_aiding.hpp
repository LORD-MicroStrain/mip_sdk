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
    
    CMD_ECEF_POS           = 0x01,
    CMD_LLH_POS            = 0x02,
    CMD_LOCAL_POS          = 0x03,
    CMD_HEIGHT_ABS         = 0x04,
    CMD_HEIGHT_REL         = 0x05,
    CMD_ECEF_VEL           = 0x08,
    CMD_NED_VEL            = 0x09,
    CMD_ODOM_VEL           = 0x0A,
    CMD_WHEELSPEED         = 0x0B,
    CMD_HEADING_TRUE       = 0x11,
    CMD_DELTA_POSITION     = 0x18,
    CMD_DELTA_ATTITUDE     = 0x19,
    CMD_LOCAL_ANGULAR_RATE = 0x1A,
    CMD_ECHO_CONTROL       = 0x6F,
    
    REPLY_ECEF_POS         = 0x81,
    REPLY_LLH_POS          = 0x82,
    REPLY_ODOM_VEL         = 0x8A,
    REPLY_HEADING_TRUE     = 0x91,
    REPLY_ECHO_CONTROL     = 0xEF,
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
///@defgroup cpp_aiding_ecef_pos  (0x13,0x01) Ecef Pos [CPP]
///
///@{

struct EcefPos
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_aiding::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_aiding::CMD_ECEF_POS;
    
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
    double position[3] = {0}; ///< ECEF position.
    float uncertainty[3] = {0}; ///< ECEF position uncertainty.
    ValidFlags valid_flags; ///< Valid flags.
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_aiding::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_aiding::REPLY_ECEF_POS;
        
        Time time; ///< Timestamp of the measurement.
        uint8_t sensor_id = 0; ///< Sensor ID.
        double position[3] = {0}; ///< ECEF position.
        float uncertainty[3] = {0}; ///< ECEF position uncertainty.
        ValidFlags valid_flags; ///< Valid flags.
        
    };
};
void insert(Serializer& serializer, const EcefPos& self);
void extract(Serializer& serializer, EcefPos& self);

void insert(Serializer& serializer, const EcefPos::Response& self);
void extract(Serializer& serializer, EcefPos::Response& self);

CmdResult ecefPos(C::mip_interface& device, const Time& time, uint8_t sensorId, const double* position, const float* uncertainty, EcefPos::ValidFlags validFlags, Time* timeOut, uint8_t* sensorIdOut, double* positionOut, float* uncertaintyOut, EcefPos::ValidFlags* validFlagsOut);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_aiding_llh_pos  (0x13,0x02) Llh Pos [CPP]
///
///@{

struct LlhPos
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_aiding::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_aiding::CMD_LLH_POS;
    
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
    double latitude = 0;
    double longitude = 0;
    double height = 0;
    float uncertainty[3] = {0}; ///< ECEF position uncertainty.
    ValidFlags valid_flags; ///< Valid flags.
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_aiding::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_aiding::REPLY_LLH_POS;
        
        Time time; ///< Timestamp of the measurement.
        uint8_t sensor_id = 0; ///< Sensor ID.
        double latitude = 0;
        double longitude = 0;
        double height = 0;
        float uncertainty[3] = {0}; ///< ECEF position uncertainty.
        ValidFlags valid_flags; ///< Valid flags.
        
    };
};
void insert(Serializer& serializer, const LlhPos& self);
void extract(Serializer& serializer, LlhPos& self);

void insert(Serializer& serializer, const LlhPos::Response& self);
void extract(Serializer& serializer, LlhPos::Response& self);

CmdResult llhPos(C::mip_interface& device, const Time& time, uint8_t sensorId, double latitude, double longitude, double height, const float* uncertainty, LlhPos::ValidFlags validFlags, Time* timeOut, uint8_t* sensorIdOut, double* latitudeOut, double* longitudeOut, double* heightOut, float* uncertaintyOut, LlhPos::ValidFlags* validFlagsOut);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_aiding_vehicle_fixed_frame_velocity  (0x13,0x0A) Vehicle Fixed Frame Velocity [CPP]
/// Estimate of velocity of the vehicle in the frame associated
/// with the given sensor ID.
///
///@{

struct VehicleFixedFrameVelocity
{
    static const uint8_t DESCRIPTOR_SET = ::mip::commands_aiding::DESCRIPTOR_SET;
    static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_aiding::CMD_ODOM_VEL;
    
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
    float velocity[3] = {0}; ///< [meters/second]
    float uncertainty[3] = {0}; ///< [meters/second] 1-sigma uncertainty (if velocity_uncertainty[i] <= 0, then velocity[i] should be treated as invalid and ingnored)
    ValidFlags valid_flags;
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_aiding::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_aiding::REPLY_ODOM_VEL;
        
        Time time; ///< Timestamp of the measurement.
        uint8_t sensor_id = 0; ///< Source ID for this estimate ( source_id == 0 indicates this sensor, source_id > 0 indicates an external estimate )
        float velocity[3] = {0}; ///< [meters/second]
        float uncertainty[3] = {0}; ///< [meters/second] 1-sigma uncertainty (if velocity_uncertainty[i] <= 0, then velocity[i] should be treated as invalid and ingnored)
        ValidFlags valid_flags;
        
    };
};
void insert(Serializer& serializer, const VehicleFixedFrameVelocity& self);
void extract(Serializer& serializer, VehicleFixedFrameVelocity& self);

void insert(Serializer& serializer, const VehicleFixedFrameVelocity::Response& self);
void extract(Serializer& serializer, VehicleFixedFrameVelocity::Response& self);

CmdResult vehicleFixedFrameVelocity(C::mip_interface& device, const Time& time, uint8_t sensorId, const float* velocity, const float* uncertainty, VehicleFixedFrameVelocity::ValidFlags validFlags, Time* timeOut, uint8_t* sensorIdOut, float* velocityOut, float* uncertaintyOut, VehicleFixedFrameVelocity::ValidFlags* validFlagsOut);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_aiding_true_heading  (0x13,0x11) True Heading [CPP]
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
    
    struct Response
    {
        static const uint8_t DESCRIPTOR_SET = ::mip::commands_aiding::DESCRIPTOR_SET;
        static const uint8_t FIELD_DESCRIPTOR = ::mip::commands_aiding::REPLY_HEADING_TRUE;
        
        Time time;
        uint8_t sensor_id = 0;
        float heading = 0; ///< Heading in [radians]
        float uncertainty = 0;
        uint16_t valid_flags = 0;
        
    };
};
void insert(Serializer& serializer, const TrueHeading& self);
void extract(Serializer& serializer, TrueHeading& self);

void insert(Serializer& serializer, const TrueHeading::Response& self);
void extract(Serializer& serializer, TrueHeading::Response& self);

CmdResult trueHeading(C::mip_interface& device, const Time& time, uint8_t sensorId, float heading, float uncertainty, uint16_t validFlags, Time* timeOut, uint8_t* sensorIdOut, float* headingOut, float* uncertaintyOut, uint16_t* validFlagsOut);
///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_aiding_aiding_echo_control  (0x13,0x6F) Aiding Echo Control [CPP]
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

///@}
///@}
///
////////////////////////////////////////////////////////////////////////////////
} // namespace commands_aiding
} // namespace mip

