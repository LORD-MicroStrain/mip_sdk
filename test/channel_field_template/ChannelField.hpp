//
// Created by Nick on 7/23/2022.
//

#ifndef CHANNELFIELD_HPP
#define CHANNELFIELD_HPP

#include <string>
#include <vector>

#include "Utils.hpp"

namespace mscl
{
    class NamedValue
    {
    public:
        // Gets the name
        std::string name() const;

        // Gets the value
        uint8 value() const;

        // Gets the  value as a hex string
        std::string stringValue() const;

        // Print full information
        friend std::ostream& operator<<(std::ostream& stream, const NamedValue& namedValue);

        // Compare values
        bool operator==(const NamedValue& rhs) const;

        // Copy constructor
        NamedValue(const NamedValue& namedValue) = default;

    protected:
        // Default constructor
        NamedValue() = default;

        // Create a copy with the same name, but new value
        explicit NamedValue(const NamedValue& namedValue, const uint8& value);

        // Create a named value
        explicit NamedValue(std::string  name, const uint8& value = 0x00);

        // Print full information
        virtual std::ostream& print(std::ostream& stream) const;

    private:
        // Name
        std::string m_name;

        // Value
        uint8 m_value = 0x00;
    };

    class DescriptorSet : public NamedValue
    {
        // Only allow channel fields to create descriptor sets
        friend class ChannelField;

        // Default constructor
        DescriptorSet() = default;

        // Create a new descriptor set
        explicit DescriptorSet(const std::string& name, const uint8& descriptorSet);

    public:
        // Copy constructor
        DescriptorSet(const DescriptorSet& other) = default;

        // Compare values
        bool operator==(const DescriptorSet& rhs) const;

        // Default unknown descriptor set
        static const DescriptorSet Unknown;

        // Base command descriptor set
        static const DescriptorSet CommandBase;
        // 3DM command descriptor set
        static const DescriptorSet Command3DM;
        // GNSS command descriptor set
        static const DescriptorSet CommandGNSS;

        // Sensor data descriptor set
        static const DescriptorSet DataSensor;
        // GNSS data descriptor set
        static const DescriptorSet DataGNSS;
        // Filter data descriptor set
        static const DescriptorSet DataFilter;
        // GNSS1 data descriptor set
        static const DescriptorSet DataGNSS1;
        // GNSS2 data descriptor set
        static const DescriptorSet DataGNSS2;
        // GNSS3 data descriptor set
        static const DescriptorSet DataGNSS3;
        // GNSS4 data descriptor set
        static const DescriptorSet DataGNSS4;
        // GNSS5 data descriptor set
        static const DescriptorSet DataGNSS5;
        // Shared data descriptor set
        static const DescriptorSet DataShared;
    };

    class Descriptor : public NamedValue
    {
        // Only allow channel fields to create descriptors
        friend class ChannelField;

        // Default constructor
        Descriptor() = default;

        // Create a new descriptor set
        explicit Descriptor(const std::string& name, const uint8& descriptor);

        // Compare values
        bool operator==(const Descriptor& rhs) const;

    public:
        // Copy constructor
        Descriptor(const Descriptor& other) = default;

        // Default unknown descriptor
        static const Descriptor Unknown;
    };

    //TODO: name, index, value, unit
    class ChannelQualifier : public NamedValue
    {
        // Default constructor
        ChannelQualifier() = default;

        // Copy a qualifier with a new index
        explicit ChannelQualifier(const ChannelQualifier& other, const uint8& index);

        // Create a new channel qualifier
        explicit ChannelQualifier(const std::string& name, const uint8& index = 0x00);

        // Common qualifier X
        static const ChannelQualifier X;
        // Common qualifier X
        static const ChannelQualifier Y;
        // Common qualifier X
        static const ChannelQualifier Z;
        // Common qualifier W
        static const ChannelQualifier W;

    public:
        // Copy constructor
        ChannelQualifier(const ChannelQualifier& other) = default;
    };

    // Channel qualifier list
    typedef std::vector<ChannelQualifier> ChannelQualifiers;

    class ChannelField
    {
    protected:
        // Default constructor
        ChannelField() = default;

        // Copy a channel field into a different descriptor set
        explicit ChannelField(const ChannelField& other, const DescriptorSet& descriptorSet);

        // Create a new channel field
        explicit ChannelField(const Descriptor& descriptor, const DescriptorSet& descriptorSet);

        // Create a new channel field
        explicit ChannelField(const std::string& descriptorName, const uint8& descriptor, const DescriptorSet& descriptorSet);

    private:
        // Descriptor set
        DescriptorSet m_descriptorSet = DescriptorSet::Unknown;

        // Descriptor
        Descriptor m_descriptor = Descriptor::Unknown;

        // Channel qualifiers
        ChannelQualifiers m_qualifiers;

    public:
        // Copy constructor
        ChannelField(const ChannelField& other) = default;

        // Gets the descriptor
        Descriptor descriptor() const;

        // Gets the descriptor set
        DescriptorSet descriptorSet() const;

        // Gets the channel qualifiers
        ChannelQualifiers qualifiers() const;

        // Print full information
        friend std::ostream& operator<<(std::ostream& stream, const ChannelField& field);

        // Print full information
        std::ostream& print(std::ostream& stream) const;

        // Get full string information
        std::string info() const;
    };

    class SensorData : public ChannelField
    {
        // Default constructor
        SensorData() = default;

        // Creates a new Sensor Data channel field
        explicit SensorData(const Descriptor& descriptor);

        // Creates a new Sensor Data channel field
        explicit SensorData(const std::string& name, const uint8& descriptor);

    public:
        // Copy constructor
        SensorData(const SensorData& other) = default;

        // Raw accelerometer sensor data
        static const SensorData AccelRaw;
        // Raw gyroscope sensor data
        static const SensorData GyroRaw;
    };

    class GnssData : public ChannelField
    {
    protected:
        // Default constructor
        GnssData() = default;

        // Creates a new GNSS Data channel field
        explicit GnssData(const Descriptor& descriptor,
                          const DescriptorSet& descriptorSet = DescriptorSet::DataGNSS);

        // Creates a new GNSS Data channel field
        explicit GnssData(const std::string& name, const uint8& descriptor,
                          const DescriptorSet& descriptorSet = DescriptorSet::DataGNSS);

    public:
        // Copy constructor
        GnssData(const GnssData& other, const DescriptorSet& descriptorSet);

        // Position LLH GNSS data
        static const GnssData PositionLLH;
    };

    class Gnss1Data : public GnssData
    {
    protected:
        // Default constructor
        Gnss1Data() = default;

        // Create a copy of the base GNSS Data Field with the proper descriptor set
        explicit Gnss1Data(const GnssData& base);

        // Creates a new GNSS1 Data channel field
        explicit Gnss1Data(const Descriptor& descriptor);

        // Creates a new GNSS1 Data channel field
        explicit Gnss1Data(const std::string& name, const uint8& descriptor);

    public:
        // Copy constructor
        Gnss1Data(const Gnss1Data& other) = default;

        // Position LLH GNSS data
        static const Gnss1Data PositionLLH;
    };

    class Gnss2Data : public GnssData
    {
    protected:
        // Default constructor
        Gnss2Data() = default;

        // Create a copy of the base GNSS Data Field with the proper descriptor set
        explicit Gnss2Data(const GnssData& base);

        // Creates a new GNSS2 Data channel field
        explicit Gnss2Data(const Descriptor& descriptor);

        // Creates a new GNSS2 Data channel field
        explicit Gnss2Data(const std::string& name, const uint8& descriptor);

    public:
        // Copy constructor
        Gnss2Data(const Gnss2Data& other) = default;

        // Position LLH GNSS data
        static const Gnss2Data PositionLLH;
    };

    class Gnss3Data : public GnssData
    {
    protected:
        // Default constructor
        Gnss3Data() = default;

        // Create a copy of the base GNSS Data Field with the proper descriptor set
        explicit Gnss3Data(const GnssData& base);

        // Creates a new GNSS3 Data channel field
        explicit Gnss3Data(const Descriptor& descriptor);

        // Creates a new GNSS3 Data channel field
        explicit Gnss3Data(const std::string& name, const uint8& descriptor);

    public:
        // Copy constructor
        Gnss3Data(const Gnss3Data& other) = default;

        // Position LLH GNSS data
        static const Gnss3Data PositionLLH;
    };

    class Gnss4Data : public GnssData
    {
    protected:
        // Default constructor
        Gnss4Data() = default;

        // Create a copy of the base GNSS Data Field with the proper descriptor set
        explicit Gnss4Data(const GnssData& base);

        // Creates a new GNSS4 Data channel field
        explicit Gnss4Data(const Descriptor& descriptor);

        // Creates a new GNSS4 Data channel field
        explicit Gnss4Data(const std::string& name, const uint8& descriptor);

    public:
        // Copy constructor
        Gnss4Data(const Gnss4Data& other) = default;

        // Position LLH GNSS data
        static const Gnss4Data PositionLLH;
    };

    class Gnss5Data : public GnssData
    {
    protected:
        // Default constructor
        Gnss5Data() = default;

        // Create a copy of the base GNSS Data Field with the proper descriptor set
        explicit Gnss5Data(const GnssData& base);

        // Creates a new GNSS5 Data channel field
        explicit Gnss5Data(const Descriptor& descriptor);

        // Creates a new GNSS5 Data channel field
        explicit Gnss5Data(const std::string& name, const uint8& descriptor);

    public:
        // Copy constructor
        Gnss5Data(const Gnss5Data& other) = default;

        // Position LLH GNSS data
        static const Gnss5Data PositionLLH;
    };
} // mscl

#endif //CHANNELFIELD_HPP