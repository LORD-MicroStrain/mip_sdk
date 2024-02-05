#pragma once

#include "common.h"
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
    CMD_PRESSURE           = 0x25,
    CMD_VEL_ECEF           = 0x28,
    CMD_VEL_NED            = 0x29,
    CMD_VEL_ODOM           = 0x2A,
    CMD_WHEELSPEED         = 0x2B,
    CMD_HEADING_TRUE       = 0x31,
    CMD_MAGNETIC_FIELD     = 0x32,
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
        INTERNAL_REFERENCE = 1,  ///<  Timestamp provided is with respect to internal clock.
        EXTERNAL_TIME      = 2,  ///<  Timestamp provided is with respect to external clock, synced by PPS source.
        TIME_OF_ARRIVAL    = 3,  ///<  Timestamp provided is a fixed latency relative to time of message arrival.
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
    FunctionSelector function = static_cast<FunctionSelector>(0);
    uint8_t sensor_id = 0; ///< Sensor ID to configure. Cannot be 0.
    uint8_t frame_id = 0; ///< Frame ID to assign to the sensor. Defaults to 1.
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_aiding::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_aiding::CMD_SENSOR_FRAME_MAP;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    static constexpr const uint32_t WRITE_PARAMS   = 0x8003;
    static constexpr const uint32_t READ_PARAMS    = 0x8000;
    static constexpr const uint32_t SAVE_PARAMS    = 0x8000;
    static constexpr const uint32_t LOAD_PARAMS    = 0x8000;
    static constexpr const uint32_t DEFAULT_PARAMS = 0x8000;
    static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple(sensor_id,frame_id);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(sensor_id),std::ref(frame_id));
    }
    
    static SensorFrameMapping create_sld_all(::mip::FunctionSelector function)
    {
        SensorFrameMapping cmd;
        cmd.function = function;
        return cmd;
    }
    
    struct Response
    {
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_aiding::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_aiding::REPLY_SENSOR_FRAME_MAP;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        
        static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
        static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
        uint8_t sensor_id = 0; ///< Sensor ID to configure. Cannot be 0.
        uint8_t frame_id = 0; ///< Frame ID to assign to the sensor. Defaults to 1.
        
        
        auto as_tuple()
        {
            return std::make_tuple(std::ref(sensor_id),std::ref(frame_id));
        }
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
/// Defines a reference frame associated with a specific sensor frame ID.  The frame ID used in this command
/// should mirror the frame ID used in the aiding command (if that aiding measurement is measured in this reference frame)
/// 
/// This transform satisfies the following relationship:
/// 
/// EQSTART p^{veh} = R p^{sensor_frame} + t EQEND<br/>
/// 
/// Where:<br/>
/// EQSTART R EQEND is rotation matrix defined by the rotation component and EQSTART t EQEND is the translation vector<br/><br/>
/// EQSTART p^{sensor_frame} EQEND is a 3-element position vector expressed in the external sensor frame<br/>
/// EQSTART p^{veh} EQEND is a 3-element position vector expressed in the vehicle frame<br/>
/// 
/// Rotation can be defined using Euler angles OR quaternions.  If Format selector is set to Euler Angles, the fourth element
/// in the rotation vector is ignored and should be set to 0.
/// 
/// Example: GNSS antenna lever arm
/// 
/// Frame ID: 1
/// Format: 1 (Euler)
/// Translation: [0,1,] (GNSS with a 1 meter Y offset in the vehicle frame)
/// Rotation: [0,0,0,0] (Rotational component is not relevant for GNSS measurements, set to zero)
/// 
///
///@{

struct ReferenceFrame
{
    enum class Format : uint8_t
    {
        EULER      = 1,  ///<  Translation vector followed by euler angles (roll, pitch, yaw).
        QUATERNION = 2,  ///<  Translation vector followed by quaternion (w, x, y, z).
    };
    
    union Rotation
    {
        Rotation() {}
        Vector3f euler;
        Quatf quaternion;
    };
    
    FunctionSelector function = static_cast<FunctionSelector>(0);
    uint8_t frame_id = 0; ///< Reference frame number. Cannot be 0.
    Format format = static_cast<Format>(0); ///< Format of the transformation.
    Vector3f translation; ///< Translation X, Y, and Z.
    Rotation rotation; ///< Rotation as specified by format.
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_aiding::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_aiding::CMD_FRAME_CONFIG;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    static constexpr const uint32_t WRITE_PARAMS   = 0x800F;
    static constexpr const uint32_t READ_PARAMS    = 0x8003;
    static constexpr const uint32_t SAVE_PARAMS    = 0x8001;
    static constexpr const uint32_t LOAD_PARAMS    = 0x8001;
    static constexpr const uint32_t DEFAULT_PARAMS = 0x8001;
    static constexpr const uint32_t ECHOED_PARAMS  = 0x0003;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple(frame_id,format,translation,rotation);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(frame_id),std::ref(format),std::ref(translation),std::ref(rotation));
    }
    
    static ReferenceFrame create_sld_all(::mip::FunctionSelector function)
    {
        ReferenceFrame cmd;
        cmd.function = function;
        cmd.frame_id = 0;
        return cmd;
    }
    
    struct Response
    {
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_aiding::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_aiding::REPLY_FRAME_CONFIG;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        
        static constexpr const uint32_t ECHOED_PARAMS  = 0x0003;
        static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
        uint8_t frame_id = 0; ///< Reference frame number. Cannot be 0.
        Format format = static_cast<Format>(0); ///< Format of the transformation.
        Vector3f translation; ///< Translation X, Y, and Z.
        Rotation rotation; ///< Rotation as specified by format.
        
        
        auto as_tuple()
        {
            return std::make_tuple(std::ref(frame_id),std::ref(format),std::ref(translation),std::ref(rotation));
        }
    };
};
void insert(Serializer& serializer, const ReferenceFrame& self);
void extract(Serializer& serializer, ReferenceFrame& self);

void insert(Serializer& serializer, const ReferenceFrame::Response& self);
void extract(Serializer& serializer, ReferenceFrame::Response& self);

CmdResult writeReferenceFrame(C::mip_interface& device, uint8_t frameId, ReferenceFrame::Format format, const float* translation, const ReferenceFrame::Rotation& rotation);
CmdResult readReferenceFrame(C::mip_interface& device, uint8_t frameId, ReferenceFrame::Format format, float* translationOut, ReferenceFrame::Rotation* rotationOut);
CmdResult saveReferenceFrame(C::mip_interface& device, uint8_t frameId);
CmdResult loadReferenceFrame(C::mip_interface& device, uint8_t frameId);
CmdResult defaultReferenceFrame(C::mip_interface& device, uint8_t frameId);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_aiding_aiding_echo_control  (0x13,0x1F) Aiding Echo Control [CPP]
/// Controls command response behavior to external aiding commands
///
///@{

struct AidingEchoControl
{
    enum class Mode : uint8_t
    {
        SUPPRESS_ACK = 0,  ///<  Suppresses the usual command ack field for aiding messages.
        STANDARD     = 1,  ///<  Normal ack/nack behavior.
        RESPONSE     = 2,  ///<  Echo the data back as a response.
    };
    
    FunctionSelector function = static_cast<FunctionSelector>(0);
    Mode mode = static_cast<Mode>(0); ///< Controls data echoing.
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_aiding::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_aiding::CMD_ECHO_CONTROL;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    static constexpr const uint32_t WRITE_PARAMS   = 0x8001;
    static constexpr const uint32_t READ_PARAMS    = 0x8000;
    static constexpr const uint32_t SAVE_PARAMS    = 0x8000;
    static constexpr const uint32_t LOAD_PARAMS    = 0x8000;
    static constexpr const uint32_t DEFAULT_PARAMS = 0x8000;
    static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple(mode);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(mode));
    }
    
    static AidingEchoControl create_sld_all(::mip::FunctionSelector function)
    {
        AidingEchoControl cmd;
        cmd.function = function;
        return cmd;
    }
    
    struct Response
    {
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_aiding::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_aiding::REPLY_ECHO_CONTROL;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        
        static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
        static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
        Mode mode = static_cast<Mode>(0); ///< Controls data echoing.
        
        
        auto as_tuple()
        {
            return std::make_tuple(std::ref(mode));
        }
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
    struct ValidFlags : Bitfield<ValidFlags>
    {
        enum _enumType : uint16_t
        {
            NONE = 0x0000,
            X    = 0x0001,  ///<  
            Y    = 0x0002,  ///<  
            Z    = 0x0004,  ///<  
            ALL  = 0x0007,
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
    Vector3d position; ///< ECEF position [m].
    Vector3f uncertainty; ///< ECEF position uncertainty [m].
    ValidFlags valid_flags; ///< Valid flags.
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_aiding::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_aiding::CMD_POS_ECEF;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple(time,sensor_id,position,uncertainty,valid_flags);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(time),std::ref(sensor_id),std::ref(position),std::ref(uncertainty),std::ref(valid_flags));
    }
    typedef void Response;
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
    struct ValidFlags : Bitfield<ValidFlags>
    {
        enum _enumType : uint16_t
        {
            NONE      = 0x0000,
            LATITUDE  = 0x0001,  ///<  
            LONGITUDE = 0x0002,  ///<  
            HEIGHT    = 0x0004,  ///<  
            ALL       = 0x0007,
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
    Vector3f uncertainty; ///< NED position uncertainty.
    ValidFlags valid_flags; ///< Valid flags.
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_aiding::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_aiding::CMD_POS_LLH;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple(time,sensor_id,latitude,longitude,height,uncertainty,valid_flags);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(time),std::ref(sensor_id),std::ref(latitude),std::ref(longitude),std::ref(height),std::ref(uncertainty),std::ref(valid_flags));
    }
    typedef void Response;
};
void insert(Serializer& serializer, const LlhPos& self);
void extract(Serializer& serializer, LlhPos& self);

CmdResult llhPos(C::mip_interface& device, const Time& time, uint8_t sensorId, double latitude, double longitude, double height, const float* uncertainty, LlhPos::ValidFlags validFlags);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_aiding_height  (0x13,0x23) Height [CPP]
/// Estimated value of height.
///
///@{

struct Height
{
    Time time;
    uint8_t sensor_id = 0;
    float height = 0; ///< [m]
    float uncertainty = 0; ///< [m]
    uint16_t valid_flags = 0;
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_aiding::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_aiding::CMD_HEIGHT_ABS;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple(time,sensor_id,height,uncertainty,valid_flags);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(time),std::ref(sensor_id),std::ref(height),std::ref(uncertainty),std::ref(valid_flags));
    }
    typedef void Response;
};
void insert(Serializer& serializer, const Height& self);
void extract(Serializer& serializer, Height& self);

CmdResult height(C::mip_interface& device, const Time& time, uint8_t sensorId, float height, float uncertainty, uint16_t validFlags);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_aiding_pressure  (0x13,0x25) Pressure [CPP]
/// Estimated value of air pressure.
///
///@{

struct Pressure
{
    Time time;
    uint8_t sensor_id = 0;
    float pressure = 0; ///< [mbar]
    float uncertainty = 0; ///< [mbar]
    uint16_t valid_flags = 0;
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_aiding::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_aiding::CMD_PRESSURE;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple(time,sensor_id,pressure,uncertainty,valid_flags);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(time),std::ref(sensor_id),std::ref(pressure),std::ref(uncertainty),std::ref(valid_flags));
    }
    typedef void Response;
};
void insert(Serializer& serializer, const Pressure& self);
void extract(Serializer& serializer, Pressure& self);

CmdResult pressure(C::mip_interface& device, const Time& time, uint8_t sensorId, float pressure, float uncertainty, uint16_t validFlags);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_aiding_ecef_vel  (0x13,0x28) Ecef Vel [CPP]
/// ECEF velocity aiding command. Coordinates are given in the WGS84 ECEF frame.
///
///@{

struct EcefVel
{
    struct ValidFlags : Bitfield<ValidFlags>
    {
        enum _enumType : uint16_t
        {
            NONE = 0x0000,
            X    = 0x0001,  ///<  
            Y    = 0x0002,  ///<  
            Z    = 0x0004,  ///<  
            ALL  = 0x0007,
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
    Vector3f velocity; ///< ECEF velocity [m/s].
    Vector3f uncertainty; ///< ECEF velocity uncertainty [m/s].
    ValidFlags valid_flags; ///< Valid flags.
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_aiding::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_aiding::CMD_VEL_ECEF;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple(time,sensor_id,velocity,uncertainty,valid_flags);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(time),std::ref(sensor_id),std::ref(velocity),std::ref(uncertainty),std::ref(valid_flags));
    }
    typedef void Response;
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
    struct ValidFlags : Bitfield<ValidFlags>
    {
        enum _enumType : uint16_t
        {
            NONE = 0x0000,
            X    = 0x0001,  ///<  
            Y    = 0x0002,  ///<  
            Z    = 0x0004,  ///<  
            ALL  = 0x0007,
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
    Vector3f velocity; ///< NED velocity [m/s].
    Vector3f uncertainty; ///< NED velocity uncertainty [m/s].
    ValidFlags valid_flags; ///< Valid flags.
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_aiding::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_aiding::CMD_VEL_NED;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple(time,sensor_id,velocity,uncertainty,valid_flags);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(time),std::ref(sensor_id),std::ref(velocity),std::ref(uncertainty),std::ref(valid_flags));
    }
    typedef void Response;
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
    struct ValidFlags : Bitfield<ValidFlags>
    {
        enum _enumType : uint16_t
        {
            NONE = 0x0000,
            X    = 0x0001,  ///<  
            Y    = 0x0002,  ///<  
            Z    = 0x0004,  ///<  
            ALL  = 0x0007,
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
    Vector3f velocity; ///< [m/s]
    Vector3f uncertainty; ///< [m/s] 1-sigma uncertainty (if uncertainty[i] <= 0, then velocity[i] should be treated as invalid and ingnored)
    ValidFlags valid_flags;
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_aiding::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_aiding::CMD_VEL_ODOM;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple(time,sensor_id,velocity,uncertainty,valid_flags);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(time),std::ref(sensor_id),std::ref(velocity),std::ref(uncertainty),std::ref(valid_flags));
    }
    typedef void Response;
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
    Time time;
    uint8_t sensor_id = 0;
    float heading = 0; ///< Heading in [radians]
    float uncertainty = 0;
    uint16_t valid_flags = 0;
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_aiding::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_aiding::CMD_HEADING_TRUE;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple(time,sensor_id,heading,uncertainty,valid_flags);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(time),std::ref(sensor_id),std::ref(heading),std::ref(uncertainty),std::ref(valid_flags));
    }
    typedef void Response;
};
void insert(Serializer& serializer, const TrueHeading& self);
void extract(Serializer& serializer, TrueHeading& self);

CmdResult trueHeading(C::mip_interface& device, const Time& time, uint8_t sensorId, float heading, float uncertainty, uint16_t validFlags);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_aiding_magnetic_field  (0x13,0x32) Magnetic Field [CPP]
/// Estimate of magnetic field in the frame associated with the given sensor ID.
///
///@{

struct MagneticField
{
    struct ValidFlags : Bitfield<ValidFlags>
    {
        enum _enumType : uint16_t
        {
            NONE = 0x0000,
            X    = 0x0001,  ///<  
            Y    = 0x0002,  ///<  
            Z    = 0x0004,  ///<  
            ALL  = 0x0007,
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
    Vector3f magnetic_field; ///< [G]
    Vector3f uncertainty; ///< [G] 1-sigma uncertainty (if uncertainty[i] <= 0, then magnetic_field[i] should be treated as invalid and ingnored)
    ValidFlags valid_flags;
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_aiding::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_aiding::CMD_MAGNETIC_FIELD;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple(time,sensor_id,magnetic_field,uncertainty,valid_flags);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(time),std::ref(sensor_id),std::ref(magnetic_field),std::ref(uncertainty),std::ref(valid_flags));
    }
    typedef void Response;
};
void insert(Serializer& serializer, const MagneticField& self);
void extract(Serializer& serializer, MagneticField& self);

CmdResult magneticField(C::mip_interface& device, const Time& time, uint8_t sensorId, const float* magneticField, const float* uncertainty, MagneticField::ValidFlags validFlags);

///@}
///

///@}
///@}
///
////////////////////////////////////////////////////////////////////////////////
} // namespace commands_aiding
} // namespace mip

