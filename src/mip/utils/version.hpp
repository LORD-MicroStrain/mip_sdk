#pragma once

#include <assert.h>
#include <string>
#include <cstdio>
#include <regex>

////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_serialization  MIP Serialization
///
///@brief Serialization Functions for reading and writing to byte buffers.
///@{
///@defgroup mip_serialization_c   MIP Serialization [C]
///@defgroup mip_serialization_cpp MIP Serialization [CPP]
///@}

#ifdef __cplusplus
#include <type_traits>

namespace mip {

    ////////////////////////////////////////////////////////////////////////////////
    ///@addtogroup version_cpp
    ///
    ///@brief Version interpretation in C++.
    ///
    ///@{

    ////////////////////////////////////////////////////////////////////////////////
    ///@brief Structure used interpretting version strings.
    ///
    ///
    struct Version
    {
        Version(uint16_t device_value=0) : m_value(device_value) {}
        Version(uint8_t major_, uint8_t minor_, uint8_t patch_) : m_value(fromParts(major_, minor_, patch_)) { assert(major_<10 && minor_<10 && patch_<100); }

        uint8_t major() const { return m_value / 100; }
        uint8_t minor() const { return (m_value / 10) % 10;}
        uint8_t patch() const { return m_value % 100; }
        uint16_t bcd() const { return m_value; }

        bool isDevVersion()     const { return major() == 0; }
        bool isReleaseVersion() const { return major() >  0; }
        bool isSpecialVersion() const { return major() >  1; }

        bool operator==(Version other) const { return m_value == other.m_value; }
        bool operator!=(Version other) const { return m_value != other.m_value; }
        bool operator<=(Version other) const { return m_value <= other.m_value; }
        bool operator>=(Version other) const { return m_value <= other.m_value; }
        bool operator< (Version other) const { return m_value < other.m_value; }
        bool operator> (Version other) const { return m_value < other.m_value; }

        static constexpr size_t STRING_BUFFER_SIZE = 7+1;  // Including null
        void toString(char* buffer) const { std::snprintf(buffer, STRING_BUFFER_SIZE, "%u.%u.%02u", major(), minor(), patch()); }
        std::string toString() const { char buffer[STRING_BUFFER_SIZE]; toString(buffer); return std::string(buffer); }

        bool fromString(std::string str)
        {
            // Pattern to extract only numbers Major.Minor.Patch. Anything else before or after is ignored
            const std::regex pattern(R"((\d+)\.?(\d+)?\.?(\d+)?)");

            std::smatch match;

            // Pattern will always find a Major number if it exists
            if (std::regex_search(str, match, pattern))
            {
                uint8_t major = std::stoi(match[1]);

                // Minor is optionally found
                uint8_t minor = match[2].matched ? std::stoi(match[2]) : 0;

                // Patch is optionally found
                uint8_t patch = match[3].matched ? std::stoi(match[3]) : 0;

                m_value = fromParts(major, minor, patch);

                return true;
            }

            return false;
        }

    private:
        uint16_t m_value = 0;

    private:
        static uint16_t fromParts(uint8_t major, uint8_t minor, uint8_t patch) { return major * 100 + minor * 10 + patch; }
    };

} // namespace mip
#endif // __cplusplus

////////////////////////////////////////////////////////////////////////////////