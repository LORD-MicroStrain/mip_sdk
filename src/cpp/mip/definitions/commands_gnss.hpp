#pragma once

#include <mip/definitions/common.hpp>
#include <mip/mip_descriptors.hpp>
#include <mip/mip_result.hpp>
#include <mip/mip_interface.hpp>

#include <stdint.h>
#include <stddef.h>

namespace mip {
namespace C {
struct mip_interface;
} // namespace C

namespace commands_gnss {

////////////////////////////////////////////////////////////////////////////////
///@addtogroup MipCommands_cpp
///@{
///@defgroup gnss_commands_cpp  Gnss Commands
///
///@{

////////////////////////////////////////////////////////////////////////////////
// Descriptors
////////////////////////////////////////////////////////////////////////////////

enum 
{
    DESCRIPTOR_SET                 = 0x0E,
    
    CMD_LIST_RECEIVERS             = 0x01,
    CMD_SIGNAL_CONFIGURATION       = 0x02,
    CMD_RTK_DONGLE_CONFIGURATION   = 0x10,
    CMD_SPARTN_CONFIGURATION       = 0x20,
    
    REPLY_LIST_RECEIVERS           = 0x81,
    REPLY_SIGNAL_CONFIGURATION     = 0x82,
    REPLY_RTK_DONGLE_CONFIGURATION = 0x90,
    REPLY_SPARTN_CONFIGURATION     = 0xA0,
};

////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////

static constexpr const uint16_t GNSS_GPS_ENABLE_L1CA = 0x0001;
static constexpr const uint16_t GNSS_GPS_ENABLE_L2C = 0x0002;
static constexpr const uint16_t GNSS_GPS_ENABLE_L5 = 0x0004;
static constexpr const uint16_t GNSS_GLONASS_ENABLE_L1OF = 0x0001;
static constexpr const uint16_t GNSS_GLONASS_ENABLE_L2OF = 0x0002;
static constexpr const uint16_t GNSS_GALILEO_ENABLE_E1 = 0x0001;
static constexpr const uint16_t GNSS_GALILEO_ENABLE_E5B = 0x0002;
static constexpr const uint16_t GNSS_GALILEO_ENABLE_E5A = 0x0004;
static constexpr const uint16_t GNSS_BEIDOU_ENABLE_B1 = 0x0001;
static constexpr const uint16_t GNSS_BEIDOU_ENABLE_B2 = 0x0002;
static constexpr const uint16_t GNSS_BEIDOU_ENABLE_B2A = 0x0004;

////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
///@defgroup gnss_receiver_info_cpp  (0x0E,0x01) Receiver Info
/// Return information about the GNSS receivers in the device.
/// 
///
///@{

struct ReceiverInfo
{
    struct Info
    {
        /// Parameters
        uint8_t receiver_id = 0; ///< Receiver id: e.g. 1, 2, etc.
        uint8_t mip_data_descriptor_set = 0; ///< MIP descriptor set associated with this receiver
        char description[32] = {0}; ///< Ascii description of receiver. Contains the following info (comma-delimited):<br/> Module name/model<br/> Firmware version info
        
        /// Serialization
        void insert(Serializer& serializer) const;
        void extract(Serializer& serializer);
        
    };
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_gnss::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_gnss::CMD_LIST_RECEIVERS;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "ReceiverInfo";
    static constexpr const char* DOC_NAME = "ReceiverInfo";
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    
    auto asTuple() const
    {
        return std::make_tuple();
    }
    
    auto asTuple()
    {
        return std::make_tuple();
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
    struct Response
    {
        /// Parameters
        uint8_t num_receivers = 0; ///< Number of physical receivers in the device
        Info receiver_info[5];
        
        /// Descriptors
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_gnss::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_gnss::REPLY_LIST_RECEIVERS;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "ReceiverInfo::Response";
        static constexpr const char* DOC_NAME = "ReceiverInfo Response";
        static constexpr const bool HAS_FUNCTION_SELECTOR = false;
        
        auto asTuple() const
        {
            return std::make_tuple(num_receivers,receiver_info);
        }
        
        auto asTuple()
        {
            return std::make_tuple(std::ref(num_receivers),std::ref(receiver_info));
        }
        
        /// Serialization
        void insert(Serializer& serializer) const;
        void extract(Serializer& serializer);
        
    };
};
TypedResult<ReceiverInfo> receiverInfo(C::mip_interface& device, uint8_t* numReceiversOut, uint8_t numReceiversOutMax, ReceiverInfo::Info* receiverInfoOut);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup gnss_signal_configuration_cpp  (0x0E,0x02) Signal Configuration
/// Configure the GNSS signals used by the device.
/// 
///
///@{

struct SignalConfiguration
{
    /// Parameters
    FunctionSelector function = static_cast<FunctionSelector>(0);
    uint8_t gps_enable = 0; ///< Bitfield 0: Enable L1CA, 1: Enable L2C, 2: Enable L5
    uint8_t glonass_enable = 0; ///< Bitfield 0: Enable L1OF, 1: Enable L2OF
    uint8_t galileo_enable = 0; ///< Bitfield 0: Enable E1,   1: Enable E5B, 2: Enable E5A
    uint8_t beidou_enable = 0; ///< Bitfield 0: Enable B1,   1: Enable B2,  2: Enable B2A
    uint8_t reserved[4] = {0};
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_gnss::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_gnss::CMD_SIGNAL_CONFIGURATION;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "SignalConfiguration";
    static constexpr const char* DOC_NAME = "SignalConfiguration";
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    
    auto asTuple() const
    {
        return std::make_tuple(gps_enable,glonass_enable,galileo_enable,beidou_enable,reserved);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(gps_enable),std::ref(glonass_enable),std::ref(galileo_enable),std::ref(beidou_enable),std::ref(reserved));
    }
    
    static SignalConfiguration create_sld_all(::mip::FunctionSelector function)
    {
        SignalConfiguration cmd;
        cmd.function = function;
        return cmd;
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
    struct Response
    {
        /// Parameters
        uint8_t gps_enable = 0; ///< Bitfield 0: Enable L1CA, 1: Enable L2C, 2: Enable L5
        uint8_t glonass_enable = 0; ///< Bitfield 0: Enable L1OF, 1: Enable L2OF
        uint8_t galileo_enable = 0; ///< Bitfield 0: Enable E1,   1: Enable E5B, 2: Enable E5A
        uint8_t beidou_enable = 0; ///< Bitfield 0: Enable B1,   1: Enable B2,  2: Enable B2A
        uint8_t reserved[4] = {0};
        
        /// Descriptors
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_gnss::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_gnss::REPLY_SIGNAL_CONFIGURATION;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "SignalConfiguration::Response";
        static constexpr const char* DOC_NAME = "SignalConfiguration Response";
        static constexpr const bool HAS_FUNCTION_SELECTOR = false;
        
        auto asTuple() const
        {
            return std::make_tuple(gps_enable,glonass_enable,galileo_enable,beidou_enable,reserved);
        }
        
        auto asTuple()
        {
            return std::make_tuple(std::ref(gps_enable),std::ref(glonass_enable),std::ref(galileo_enable),std::ref(beidou_enable),std::ref(reserved));
        }
        
        /// Serialization
        void insert(Serializer& serializer) const;
        void extract(Serializer& serializer);
        
    };
};
TypedResult<SignalConfiguration> writeSignalConfiguration(C::mip_interface& device, uint8_t gpsEnable, uint8_t glonassEnable, uint8_t galileoEnable, uint8_t beidouEnable, const uint8_t* reserved);
TypedResult<SignalConfiguration> readSignalConfiguration(C::mip_interface& device, uint8_t* gpsEnableOut, uint8_t* glonassEnableOut, uint8_t* galileoEnableOut, uint8_t* beidouEnableOut, uint8_t* reservedOut);
TypedResult<SignalConfiguration> saveSignalConfiguration(C::mip_interface& device);
TypedResult<SignalConfiguration> loadSignalConfiguration(C::mip_interface& device);
TypedResult<SignalConfiguration> defaultSignalConfiguration(C::mip_interface& device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup gnss_spartn_configuration_cpp  (0x0E,0x20) Spartn Configuration
/// Configure the SPARTN corrections service parameters.
/// Notes:<br/>
/// - Enable and type settings will only update after a power cycle <br/>
/// - Type settings will only take effect after a power cycle <br/>
/// - Key information can be updated while running
///
///@{

struct SpartnConfiguration
{
    /// Parameters
    FunctionSelector function = static_cast<FunctionSelector>(0);
    uint8_t enable = 0; ///< Enable/Disable the SPARTN subsystem (0 = Disabled, 1 = Enabled)
    uint8_t type = 0; ///< Connection type (0 - None, 1 = Network, 2 = L-Band)
    uint32_t current_key_tow = 0; ///< The GPS time of week the current key is valid until
    uint16_t current_key_week = 0; ///< The GPS week number the current key is valid until
    uint8_t current_key[32] = {0}; ///< 32 character string of ASCII hex values for the current key (e.g. "bc" for 0xBC)
    uint32_t next_key_tow = 0; ///< The GPS time of week the next key is valid until
    uint16_t next_key_week = 0; ///< The GPS week number the next key is valid until
    uint8_t next_key[32] = {0}; ///< 32 character string of ASCII hex valuesfor the next key (e.g. "bc" for 0xBC)
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_gnss::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_gnss::CMD_SPARTN_CONFIGURATION;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "SpartnConfiguration";
    static constexpr const char* DOC_NAME = "SpartnConfiguration";
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    
    auto asTuple() const
    {
        return std::make_tuple(enable,type,current_key_tow,current_key_week,current_key,next_key_tow,next_key_week,next_key);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(enable),std::ref(type),std::ref(current_key_tow),std::ref(current_key_week),std::ref(current_key),std::ref(next_key_tow),std::ref(next_key_week),std::ref(next_key));
    }
    
    static SpartnConfiguration create_sld_all(::mip::FunctionSelector function)
    {
        SpartnConfiguration cmd;
        cmd.function = function;
        return cmd;
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
    struct Response
    {
        /// Parameters
        uint8_t enable = 0; ///< Enable/Disable the SPARTN subsystem (0 = Disabled, 1 = Enabled)
        uint8_t type = 0; ///< Connection type (0 - None, 1 = Network, 2 = L-Band)
        uint32_t current_key_tow = 0; ///< The GPS time of week the current key is valid until
        uint16_t current_key_week = 0; ///< The GPS week number the current key is valid until
        uint8_t current_key[32] = {0}; ///< 32 character string of ASCII hex values for the current key (e.g. "bc" for 0xBC)
        uint32_t next_key_tow = 0; ///< The GPS time of week the next key is valid until
        uint16_t next_key_week = 0; ///< The GPS week number the next key is valid until
        uint8_t next_key[32] = {0}; ///< 32 character string of ASCII hex valuesfor the next key (e.g. "bc" for 0xBC)
        
        /// Descriptors
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_gnss::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_gnss::REPLY_SPARTN_CONFIGURATION;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "SpartnConfiguration::Response";
        static constexpr const char* DOC_NAME = "SpartnConfiguration Response";
        static constexpr const bool HAS_FUNCTION_SELECTOR = false;
        
        auto asTuple() const
        {
            return std::make_tuple(enable,type,current_key_tow,current_key_week,current_key,next_key_tow,next_key_week,next_key);
        }
        
        auto asTuple()
        {
            return std::make_tuple(std::ref(enable),std::ref(type),std::ref(current_key_tow),std::ref(current_key_week),std::ref(current_key),std::ref(next_key_tow),std::ref(next_key_week),std::ref(next_key));
        }
        
        /// Serialization
        void insert(Serializer& serializer) const;
        void extract(Serializer& serializer);
        
    };
};
TypedResult<SpartnConfiguration> writeSpartnConfiguration(C::mip_interface& device, uint8_t enable, uint8_t type, uint32_t currentKeyTow, uint16_t currentKeyWeek, const uint8_t* currentKey, uint32_t nextKeyTow, uint16_t nextKeyWeek, const uint8_t* nextKey);
TypedResult<SpartnConfiguration> readSpartnConfiguration(C::mip_interface& device, uint8_t* enableOut, uint8_t* typeOut, uint32_t* currentKeyTowOut, uint16_t* currentKeyWeekOut, uint8_t* currentKeyOut, uint32_t* nextKeyTowOut, uint16_t* nextKeyWeekOut, uint8_t* nextKeyOut);
TypedResult<SpartnConfiguration> saveSpartnConfiguration(C::mip_interface& device);
TypedResult<SpartnConfiguration> loadSpartnConfiguration(C::mip_interface& device);
TypedResult<SpartnConfiguration> defaultSpartnConfiguration(C::mip_interface& device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup gnss_rtk_dongle_configuration_cpp  (0x0E,0x10) Rtk Dongle Configuration
/// Configure the communications with the RTK Dongle connected to the device.
/// 
///
///@{

struct RtkDongleConfiguration
{
    /// Parameters
    FunctionSelector function = static_cast<FunctionSelector>(0);
    uint8_t enable = 0; ///< 0 - Disabled, 1- Enabled
    uint8_t reserved[3] = {0};
    
    /// Descriptors
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_gnss::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_gnss::CMD_RTK_DONGLE_CONFIGURATION;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "RtkDongleConfiguration";
    static constexpr const char* DOC_NAME = "RtkDongleConfiguration";
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    
    auto asTuple() const
    {
        return std::make_tuple(enable,reserved);
    }
    
    auto asTuple()
    {
        return std::make_tuple(std::ref(enable),std::ref(reserved));
    }
    
    static RtkDongleConfiguration create_sld_all(::mip::FunctionSelector function)
    {
        RtkDongleConfiguration cmd;
        cmd.function = function;
        return cmd;
    }
    
    /// Serialization
    void insert(Serializer& serializer) const;
    void extract(Serializer& serializer);
    
    struct Response
    {
        /// Parameters
        uint8_t enable = 0;
        uint8_t reserved[3] = {0};
        
        /// Descriptors
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_gnss::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_gnss::REPLY_RTK_DONGLE_CONFIGURATION;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "RtkDongleConfiguration::Response";
        static constexpr const char* DOC_NAME = "RtkDongleConfiguration Response";
        static constexpr const bool HAS_FUNCTION_SELECTOR = false;
        
        auto asTuple() const
        {
            return std::make_tuple(enable,reserved);
        }
        
        auto asTuple()
        {
            return std::make_tuple(std::ref(enable),std::ref(reserved));
        }
        
        /// Serialization
        void insert(Serializer& serializer) const;
        void extract(Serializer& serializer);
        
    };
};
TypedResult<RtkDongleConfiguration> writeRtkDongleConfiguration(C::mip_interface& device, uint8_t enable, const uint8_t* reserved);
TypedResult<RtkDongleConfiguration> readRtkDongleConfiguration(C::mip_interface& device, uint8_t* enableOut, uint8_t* reservedOut);
TypedResult<RtkDongleConfiguration> saveRtkDongleConfiguration(C::mip_interface& device);
TypedResult<RtkDongleConfiguration> loadRtkDongleConfiguration(C::mip_interface& device);
TypedResult<RtkDongleConfiguration> defaultRtkDongleConfiguration(C::mip_interface& device);

///@}
///

///@}
///@}
///
////////////////////////////////////////////////////////////////////////////////
} // namespace commands_gnss
} // namespace mip

