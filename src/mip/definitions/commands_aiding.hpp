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
    DESCRIPTOR_SET             = 0x13,
    
    CMD_FRAME_CONFIG           = 0x01,
    CMD_LOCAL_FRAME            = 0x03,
    CMD_ECHO_CONTROL           = 0x1F,
    CMD_POS_LOCAL              = 0x20,
    CMD_POS_ECEF               = 0x21,
    CMD_POS_LLH                = 0x22,
    CMD_HEIGHT_ABOVE_ELLIPSOID = 0x23,
    CMD_HEIGHT_REL             = 0x24,
    CMD_VEL_ECEF               = 0x28,
    CMD_VEL_NED                = 0x29,
    CMD_VEL_BODY_FRAME         = 0x2A,
    CMD_WHEELSPEED             = 0x2B,
    CMD_HEADING_TRUE           = 0x31,
    CMD_MAGNETIC_FIELD         = 0x32,
    CMD_PRESSURE               = 0x33,
    CMD_DELTA_POSITION         = 0x38,
    CMD_DELTA_ATTITUDE         = 0x39,
    CMD_ANGULAR_RATE_LOCAL     = 0x3A,
    
    REPLY_FRAME_CONFIG         = 0x81,
    REPLY_ECHO_CONTROL         = 0x9F,
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
///@defgroup cpp_aiding_frame_config  (0x13,0x01) Frame Config [CPP]
/// Defines an aiding frame associated with a specific sensor frame ID.  The frame ID used in this command
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
/// When the tracking_enabled flag is 1, the Kalman filter will track errors in the provided frame definition; when 0, no errors are tracked.
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

struct FrameConfig
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
    uint8_t frame_id = 0; ///< Reference frame number. Limit 4.
    Format format = static_cast<Format>(0); ///< Format of the transformation.
    bool tracking_enabled = 0; ///< If enabled, the Kalman filter will track errors.
    Vector3f translation; ///< Translation X, Y, and Z.
    Rotation rotation; ///< Rotation as specified by format.
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_aiding::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_aiding::CMD_FRAME_CONFIG;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "FrameConfig";
    static constexpr const char* DOC_NAME = "Frame Configuration";
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    static constexpr const uint32_t WRITE_PARAMS   = 0x801F;
    static constexpr const uint32_t READ_PARAMS    = 0x8003;
    static constexpr const uint32_t SAVE_PARAMS    = 0x8001;
    static constexpr const uint32_t LOAD_PARAMS    = 0x8001;
    static constexpr const uint32_t DEFAULT_PARAMS = 0x8001;
    static constexpr const uint32_t ECHOED_PARAMS  = 0x0003;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple(frame_id,format,tracking_enabled,translation,rotation);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(frame_id),std::ref(format),std::ref(tracking_enabled),std::ref(translation),std::ref(rotation));
    }
    
    static FrameConfig create_sld_all(::mip::FunctionSelector function)
    {
        FrameConfig cmd;
        cmd.function = function;
        cmd.frame_id = 0;
        return cmd;
    }
    
    struct Response
    {
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_aiding::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_aiding::REPLY_FRAME_CONFIG;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "FrameConfig::Response";
        static constexpr const char* DOC_NAME = "Frame Configuration Response";
        
        static constexpr const uint32_t ECHOED_PARAMS  = 0x0003;
        static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
        uint8_t frame_id = 0; ///< Reference frame number. Limit 4.
        Format format = static_cast<Format>(0); ///< Format of the transformation.
        bool tracking_enabled = 0; ///< If enabled, the Kalman filter will track errors.
        Vector3f translation; ///< Translation X, Y, and Z.
        Rotation rotation; ///< Rotation as specified by format.
        
        
        auto as_tuple()
        {
            return std::make_tuple(std::ref(frame_id),std::ref(format),std::ref(tracking_enabled),std::ref(translation),std::ref(rotation));
        }
    };
};
void insert(Serializer& serializer, const FrameConfig& self);
void extract(Serializer& serializer, FrameConfig& self);

void insert(Serializer& serializer, const FrameConfig::Response& self);
void extract(Serializer& serializer, FrameConfig::Response& self);

TypedResult<FrameConfig> writeFrameConfig(C::mip_interface& device, uint8_t frameId, FrameConfig::Format format, bool trackingEnabled, const float* translation, const FrameConfig::Rotation& rotation);
TypedResult<FrameConfig> readFrameConfig(C::mip_interface& device, uint8_t frameId, FrameConfig::Format format, bool* trackingEnabledOut, float* translationOut, FrameConfig::Rotation* rotationOut);
TypedResult<FrameConfig> saveFrameConfig(C::mip_interface& device, uint8_t frameId);
TypedResult<FrameConfig> loadFrameConfig(C::mip_interface& device, uint8_t frameId);
TypedResult<FrameConfig> defaultFrameConfig(C::mip_interface& device, uint8_t frameId);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_aiding_echo_control  (0x13,0x1F) Echo Control [CPP]
/// Controls command response behavior to external aiding commands
///
///@{

struct EchoControl
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
    static constexpr const char* NAME = "EchoControl";
    static constexpr const char* DOC_NAME = "Aiding Command Echo Control";
    
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
    
    static EchoControl create_sld_all(::mip::FunctionSelector function)
    {
        EchoControl cmd;
        cmd.function = function;
        return cmd;
    }
    
    struct Response
    {
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_aiding::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_aiding::REPLY_ECHO_CONTROL;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "EchoControl::Response";
        static constexpr const char* DOC_NAME = "Aiding Command Echo Control Response";
        
        static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
        static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
        Mode mode = static_cast<Mode>(0); ///< Controls data echoing.
        
        
        auto as_tuple()
        {
            return std::make_tuple(std::ref(mode));
        }
    };
};
void insert(Serializer& serializer, const EchoControl& self);
void extract(Serializer& serializer, EchoControl& self);

void insert(Serializer& serializer, const EchoControl::Response& self);
void extract(Serializer& serializer, EchoControl::Response& self);

TypedResult<EchoControl> writeEchoControl(C::mip_interface& device, EchoControl::Mode mode);
TypedResult<EchoControl> readEchoControl(C::mip_interface& device, EchoControl::Mode* modeOut);
TypedResult<EchoControl> saveEchoControl(C::mip_interface& device);
TypedResult<EchoControl> loadEchoControl(C::mip_interface& device);
TypedResult<EchoControl> defaultEchoControl(C::mip_interface& device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_aiding_pos_ecef  (0x13,0x21) Pos Ecef [CPP]
/// Cartesian vector position aiding command. Coordinates are given in the WGS84 ECEF system.
///
///@{

struct PosEcef
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
        ValidFlags& operator=(int val) { value = uint16_t(val); return *this; }
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
    uint8_t frame_id = 0; ///< Source ID for this estimate (source_id == 0 indicates this sensor, source_id > 0 indicates an external estimate).
    Vector3d position; ///< ECEF position [m].
    Vector3f uncertainty; ///< ECEF position uncertainty [m]. Cannot be 0 unless the corresponding valid flags are 0.
    ValidFlags valid_flags; ///< Valid flags. Axes with 0 will be completely ignored.
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_aiding::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_aiding::CMD_POS_ECEF;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "PosEcef";
    static constexpr const char* DOC_NAME = "ECEF Position";
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple(time,frame_id,position,uncertainty,valid_flags);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(time),std::ref(frame_id),std::ref(position),std::ref(uncertainty),std::ref(valid_flags));
    }
    typedef void Response;
};
void insert(Serializer& serializer, const PosEcef& self);
void extract(Serializer& serializer, PosEcef& self);

TypedResult<PosEcef> posEcef(C::mip_interface& device, const Time& time, uint8_t frameId, const double* position, const float* uncertainty, PosEcef::ValidFlags validFlags);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_aiding_pos_llh  (0x13,0x22) Pos Llh [CPP]
/// Geodetic position aiding command. Coordinates are given in WGS84 geodetic latitude, longitude, and height above the ellipsoid.
/// Uncertainty is given in NED coordinates, which are parallel to incremental changes in latitude, longitude, and height.
///
///@{

struct PosLlh
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
        ValidFlags& operator=(int val) { value = uint16_t(val); return *this; }
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
    uint8_t frame_id = 0; ///< Source ID for this estimate (source_id == 0 indicates this sensor, source_id > 0 indicates an external estimate).
    double latitude = 0; ///< [deg]
    double longitude = 0; ///< [deg]
    double height = 0; ///< [m]
    Vector3f uncertainty; ///< NED position uncertainty. Cannot be 0 unless the corresponding valid flags are 0.
    ValidFlags valid_flags; ///< Valid flags. Axes with 0 will be completely ignored.
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_aiding::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_aiding::CMD_POS_LLH;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "PosLlh";
    static constexpr const char* DOC_NAME = "LLH Position";
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple(time,frame_id,latitude,longitude,height,uncertainty,valid_flags);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(time),std::ref(frame_id),std::ref(latitude),std::ref(longitude),std::ref(height),std::ref(uncertainty),std::ref(valid_flags));
    }
    typedef void Response;
};
void insert(Serializer& serializer, const PosLlh& self);
void extract(Serializer& serializer, PosLlh& self);

TypedResult<PosLlh> posLlh(C::mip_interface& device, const Time& time, uint8_t frameId, double latitude, double longitude, double height, const float* uncertainty, PosLlh::ValidFlags validFlags);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_aiding_height_above_ellipsoid  (0x13,0x23) Height Above Ellipsoid [CPP]
/// Estimated value of the height above ellipsoid.
///
///@{

struct HeightAboveEllipsoid
{
    Time time; ///< Timestamp of the measurement.
    uint8_t frame_id = 0; ///< Source ID for this estimate (source_id == 0 indicates this sensor, source_id > 0 indicates an external estimate).
    float height = 0; ///< [m]
    float uncertainty = 0; ///< [m]
    uint16_t valid_flags = 0;
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_aiding::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_aiding::CMD_HEIGHT_ABOVE_ELLIPSOID;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "HeightAboveEllipsoid";
    static constexpr const char* DOC_NAME = "Height Above Ellipsoid";
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple(time,frame_id,height,uncertainty,valid_flags);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(time),std::ref(frame_id),std::ref(height),std::ref(uncertainty),std::ref(valid_flags));
    }
    typedef void Response;
};
void insert(Serializer& serializer, const HeightAboveEllipsoid& self);
void extract(Serializer& serializer, HeightAboveEllipsoid& self);

TypedResult<HeightAboveEllipsoid> heightAboveEllipsoid(C::mip_interface& device, const Time& time, uint8_t frameId, float height, float uncertainty, uint16_t validFlags);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_aiding_vel_ecef  (0x13,0x28) Vel Ecef [CPP]
/// ECEF velocity aiding command. Coordinates are given in the WGS84 ECEF frame.
///
///@{

struct VelEcef
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
        ValidFlags& operator=(int val) { value = uint16_t(val); return *this; }
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
    uint8_t frame_id = 0; ///< Source ID for this estimate (source_id == 0 indicates this sensor, source_id > 0 indicates an external estimate).
    Vector3f velocity; ///< ECEF velocity [m/s].
    Vector3f uncertainty; ///< ECEF velocity uncertainty [m/s]. Cannot be 0 unless the corresponding valid flags are 0.
    ValidFlags valid_flags; ///< Valid flags. Axes with 0 will be completely ignored.
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_aiding::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_aiding::CMD_VEL_ECEF;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "VelEcef";
    static constexpr const char* DOC_NAME = "ECEF Velocity";
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple(time,frame_id,velocity,uncertainty,valid_flags);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(time),std::ref(frame_id),std::ref(velocity),std::ref(uncertainty),std::ref(valid_flags));
    }
    typedef void Response;
};
void insert(Serializer& serializer, const VelEcef& self);
void extract(Serializer& serializer, VelEcef& self);

TypedResult<VelEcef> velEcef(C::mip_interface& device, const Time& time, uint8_t frameId, const float* velocity, const float* uncertainty, VelEcef::ValidFlags validFlags);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_aiding_vel_ned  (0x13,0x29) Vel Ned [CPP]
/// NED velocity aiding command. Coordinates are given in the local North-East-Down frame.
///
///@{

struct VelNed
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
        ValidFlags& operator=(int val) { value = uint16_t(val); return *this; }
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
    uint8_t frame_id = 0; ///< Source ID for this estimate (source_id == 0 indicates this sensor, source_id > 0 indicates an external estimate).
    Vector3f velocity; ///< NED velocity [m/s].
    Vector3f uncertainty; ///< NED velocity uncertainty [m/s]. Cannot be 0 unless the corresponding valid flags are 0.
    ValidFlags valid_flags; ///< Valid flags. Axes with 0 will be completely ignored.
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_aiding::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_aiding::CMD_VEL_NED;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "VelNed";
    static constexpr const char* DOC_NAME = "NED Velocity";
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple(time,frame_id,velocity,uncertainty,valid_flags);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(time),std::ref(frame_id),std::ref(velocity),std::ref(uncertainty),std::ref(valid_flags));
    }
    typedef void Response;
};
void insert(Serializer& serializer, const VelNed& self);
void extract(Serializer& serializer, VelNed& self);

TypedResult<VelNed> velNed(C::mip_interface& device, const Time& time, uint8_t frameId, const float* velocity, const float* uncertainty, VelNed::ValidFlags validFlags);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_aiding_vel_body_frame  (0x13,0x2A) Vel Body Frame [CPP]
/// Estimate of velocity of the vehicle in the frame associated
/// with the given sensor ID, relative to the vehicle frame.
///
///@{

struct VelBodyFrame
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
        ValidFlags& operator=(int val) { value = uint16_t(val); return *this; }
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
    uint8_t frame_id = 0; ///< Source ID for this estimate (source_id == 0 indicates this sensor, source_id > 0 indicates an external estimate).
    Vector3f velocity; ///< [m/s]
    Vector3f uncertainty; ///< [m/s] 1-sigma uncertainty. Cannot be 0 unless the corresponding valid flags are 0.
    ValidFlags valid_flags; ///< Valid flags. Axes with 0 will be completely ignored.
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_aiding::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_aiding::CMD_VEL_BODY_FRAME;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "VelBodyFrame";
    static constexpr const char* DOC_NAME = "Body Frame Velocity";
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple(time,frame_id,velocity,uncertainty,valid_flags);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(time),std::ref(frame_id),std::ref(velocity),std::ref(uncertainty),std::ref(valid_flags));
    }
    typedef void Response;
};
void insert(Serializer& serializer, const VelBodyFrame& self);
void extract(Serializer& serializer, VelBodyFrame& self);

TypedResult<VelBodyFrame> velBodyFrame(C::mip_interface& device, const Time& time, uint8_t frameId, const float* velocity, const float* uncertainty, VelBodyFrame::ValidFlags validFlags);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_aiding_heading_true  (0x13,0x31) Heading True [CPP]
///
///@{

struct HeadingTrue
{
    Time time; ///< Timestamp of the measurement.
    uint8_t frame_id = 0; ///< Source ID for this estimate (source_id == 0 indicates this sensor, source_id > 0 indicates an external estimate).
    float heading = 0; ///< Heading [radians]. Range +/- Pi.
    float uncertainty = 0; ///< Cannot be 0 unless the valid flags are 0.
    uint16_t valid_flags = 0;
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_aiding::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_aiding::CMD_HEADING_TRUE;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "HeadingTrue";
    static constexpr const char* DOC_NAME = "True Heading";
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple(time,frame_id,heading,uncertainty,valid_flags);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(time),std::ref(frame_id),std::ref(heading),std::ref(uncertainty),std::ref(valid_flags));
    }
    typedef void Response;
};
void insert(Serializer& serializer, const HeadingTrue& self);
void extract(Serializer& serializer, HeadingTrue& self);

TypedResult<HeadingTrue> headingTrue(C::mip_interface& device, const Time& time, uint8_t frameId, float heading, float uncertainty, uint16_t validFlags);

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
        ValidFlags& operator=(int val) { value = uint16_t(val); return *this; }
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
    uint8_t frame_id = 0; ///< Source ID for this estimate (source_id == 0 indicates this sensor, source_id > 0 indicates an external estimate).
    Vector3f magnetic_field; ///< [G]
    Vector3f uncertainty; ///< [G] 1-sigma uncertainty. Cannot be 0 unless the corresponding valid flags are 0.
    ValidFlags valid_flags; ///< Valid flags. Axes with 0 will be completely ignored.
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_aiding::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_aiding::CMD_MAGNETIC_FIELD;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "MagneticField";
    static constexpr const char* DOC_NAME = "Magnetic Field";
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple(time,frame_id,magnetic_field,uncertainty,valid_flags);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(time),std::ref(frame_id),std::ref(magnetic_field),std::ref(uncertainty),std::ref(valid_flags));
    }
    typedef void Response;
};
void insert(Serializer& serializer, const MagneticField& self);
void extract(Serializer& serializer, MagneticField& self);

TypedResult<MagneticField> magneticField(C::mip_interface& device, const Time& time, uint8_t frameId, const float* magneticField, const float* uncertainty, MagneticField::ValidFlags validFlags);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_aiding_pressure  (0x13,0x33) Pressure [CPP]
/// Estimated value of air pressure.
///
///@{

struct Pressure
{
    Time time; ///< Timestamp of the measurement.
    uint8_t frame_id = 0; ///< Source ID for this estimate (source_id == 0 indicates this sensor, source_id > 0 indicates an external estimate).
    float pressure = 0; ///< [mbar]
    float uncertainty = 0; ///< [mbar] 1-sigma uncertainty. Cannot be 0 unless the valid flags are 0.
    uint16_t valid_flags = 0;
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_aiding::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_aiding::CMD_PRESSURE;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "Pressure";
    static constexpr const char* DOC_NAME = "Pressure";
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple(time,frame_id,pressure,uncertainty,valid_flags);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(time),std::ref(frame_id),std::ref(pressure),std::ref(uncertainty),std::ref(valid_flags));
    }
    typedef void Response;
};
void insert(Serializer& serializer, const Pressure& self);
void extract(Serializer& serializer, Pressure& self);

TypedResult<Pressure> pressure(C::mip_interface& device, const Time& time, uint8_t frameId, float pressure, float uncertainty, uint16_t validFlags);

///@}
///

///@}
///@}
///
////////////////////////////////////////////////////////////////////////////////
} // namespace commands_aiding
} // namespace mip

