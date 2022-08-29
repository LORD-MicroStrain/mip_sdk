//
// Created by Nick on 7/23/2022.
//

#include "ChannelField.hpp"

#include <utility>
#include <iostream>
#include <sstream>

namespace mscl
{
    std::string NamedValue::name() const
    {
        return m_name;
    }

    uint8 NamedValue::value() const
    {
        return m_value;
    }

    std::string NamedValue::stringValue() const
    {
        std::stringstream stream;
        stream << std::hex << std::showbase << static_cast<uint32>(m_value) << std::noshowbase;
        return stream.str();
    }

    std::ostream& operator<<(std::ostream& stream, const NamedValue& namedValue)
    {
        return namedValue.print(stream);
    }

    bool NamedValue::operator==(const NamedValue& rhs) const
    {
        return m_value == rhs.m_value;
    }

    NamedValue::NamedValue(std::string name, const uint8& value) :
        m_name(std::move(name)),
        m_value(value)
    {}

    NamedValue::NamedValue(const NamedValue& namedValue, const uint8& value/* = 0x00 */) :
        m_name(namedValue.m_name),
        m_value(value)
    {}

    std::ostream& NamedValue::print(std::ostream& stream) const
    {
        return stream << m_name << ": " << stringValue();
    }

    DescriptorSet::DescriptorSet(const std::string& name, const uint8& descriptorSet) :
        NamedValue(name, descriptorSet)
    {}

    bool DescriptorSet::operator==(const DescriptorSet& rhs) const
    {
        return NamedValue::operator==(rhs);
    }

    const DescriptorSet DescriptorSet::Unknown("Unknown Descriptor Set", 0x00);

    const DescriptorSet DescriptorSet::CommandBase("Base Command", 0x01);
    const DescriptorSet DescriptorSet::Command3DM("3DM Command"  , 0x0C);
    const DescriptorSet DescriptorSet::CommandGNSS("GNSS Command", 0x0E);

    const DescriptorSet DescriptorSet::DataSensor("Sensor Data", 0x80);
    const DescriptorSet DescriptorSet::DataGNSS("GNSS Data"    , 0x81);
    const DescriptorSet DescriptorSet::DataFilter("Filter Data", 0x82);
    const DescriptorSet DescriptorSet::DataGNSS1("GNSS1 Data"  , 0x91);
    const DescriptorSet DescriptorSet::DataGNSS2("GNSS2 Data"  , 0x92);
    const DescriptorSet DescriptorSet::DataGNSS3("GNSS3 Data"  , 0x93);
    const DescriptorSet DescriptorSet::DataGNSS4("GNSS4 Data"  , 0x94);
    const DescriptorSet DescriptorSet::DataGNSS5("GNSS5 Data"  , 0x95);
    const DescriptorSet DescriptorSet::DataShared("Shared Data", 0xFF);

    Descriptor::Descriptor(const std::string& name, const uint8& descriptor) :
        NamedValue(name, descriptor)
    {}

    bool Descriptor::operator==(const Descriptor& rhs) const
    {
        return NamedValue::operator==(rhs);
    }

    const Descriptor Descriptor::Unknown("Unknown Descriptor", 0x00);

    ChannelQualifier::ChannelQualifier(const ChannelQualifier& other, const uint8& index) :
        NamedValue(other.name(), index)
    {}

    ChannelQualifier::ChannelQualifier(const std::string& name, const uint8& index/* = 0 */) :
        NamedValue(name, index)
    {}

    const ChannelQualifier ChannelQualifier::X("X");
    const ChannelQualifier ChannelQualifier::Y("Y");
    const ChannelQualifier ChannelQualifier::Z("Z");
    const ChannelQualifier ChannelQualifier::W("W");

    ChannelField::ChannelField(const ChannelField& other, const DescriptorSet& descriptorSet) :
        m_descriptor(other.m_descriptor),
        m_descriptorSet(descriptorSet)
    {}

    ChannelField::ChannelField(const Descriptor& descriptor, const DescriptorSet& descriptorSet) :
        m_descriptorSet(descriptorSet),
        m_descriptor(descriptor)
    {}

    ChannelField::ChannelField(const std::string& descriptorName, const uint8& descriptor, const DescriptorSet& descriptorSet) :
        m_descriptor(descriptorName, descriptor),
        m_descriptorSet(descriptorSet)
    {}

    Descriptor ChannelField::descriptor() const
    {
        return m_descriptor;
    }

    DescriptorSet ChannelField::descriptorSet() const
    {
        return m_descriptorSet;
    }

    ChannelQualifiers ChannelField::qualifiers() const
    {
        return m_qualifiers;
    }

    std::ostream& operator<<(std::ostream& stream, const ChannelField& field)
    {
        return field.print(stream);
    }

    std::ostream& ChannelField::print(std::ostream& stream) const
    {
        return stream << m_descriptorSet << " " << m_descriptor;
    }

    std::string ChannelField::info() const
    {
        std::stringstream stream;
        print(stream);
        return stream.str();
    }

    SensorData::SensorData(const Descriptor& descriptor) :
        ChannelField(descriptor, DescriptorSet::DataSensor)
    {}

    SensorData::SensorData(const std::string& name, const uint8& descriptor) :
        ChannelField(name, descriptor, DescriptorSet::DataSensor)
    {}

    const SensorData SensorData::AccelRaw("Raw Accelerometer", 0x01);
    const SensorData SensorData::GyroRaw("Raw Gyroscope"     , 0x02);

    GnssData::GnssData(const GnssData& other, const DescriptorSet& descriptorSet) :
        ChannelField(other.descriptor(), descriptorSet)
    {}

    GnssData::GnssData(const Descriptor& descriptor, const DescriptorSet& descriptorSet) :
        ChannelField(descriptor, descriptorSet)
    {}

    GnssData::GnssData(const std::string &name, const uint8& descriptor, const DescriptorSet& descriptorSet) :
        ChannelField(name, descriptor,descriptorSet)
    {}

    const GnssData GnssData::PositionLLH("Position LLH", 0x30);

    Gnss1Data::Gnss1Data(const GnssData& base) :
        GnssData(base, DescriptorSet::DataGNSS1)
    {}

    Gnss1Data::Gnss1Data(const Descriptor& descriptor) :
        GnssData(descriptor, DescriptorSet::DataGNSS1)
    {}

    Gnss1Data::Gnss1Data(const std::string& name, const uint8& descriptor) :
        GnssData(name, descriptor, DescriptorSet::DataGNSS1)
    {}

    const Gnss1Data Gnss1Data::PositionLLH(GnssData::PositionLLH);

    Gnss2Data::Gnss2Data(const GnssData& base) :
        GnssData(base, DescriptorSet::DataGNSS2)
    {}

    Gnss2Data::Gnss2Data(const Descriptor& descriptor) :
        GnssData(descriptor, DescriptorSet::DataGNSS2)
    {}

    Gnss2Data::Gnss2Data(const std::string& name, const uint8& descriptor) :
        GnssData(name, descriptor, DescriptorSet::DataGNSS2)
    {}

    const Gnss2Data Gnss2Data::PositionLLH(GnssData::PositionLLH);

    Gnss3Data::Gnss3Data(const GnssData& base) :
        GnssData(base, DescriptorSet::DataGNSS3)
    {}

    Gnss3Data::Gnss3Data(const Descriptor& descriptor) :
        GnssData(descriptor, DescriptorSet::DataGNSS3)
    {}

    Gnss3Data::Gnss3Data(const std::string& name, const uint8& descriptor) :
        GnssData(name, descriptor, DescriptorSet::DataGNSS3)
    {}

    const Gnss3Data Gnss3Data::PositionLLH(GnssData::PositionLLH);

    Gnss4Data::Gnss4Data(const GnssData& base) :
        GnssData(base, DescriptorSet::DataGNSS4)
    {}

    Gnss4Data::Gnss4Data(const Descriptor& descriptor) :
        GnssData(descriptor, DescriptorSet::DataGNSS4)
    {}

    Gnss4Data::Gnss4Data(const std::string& name, const uint8& descriptor) :
        GnssData(name, descriptor, DescriptorSet::DataGNSS4)
    {}

    const Gnss4Data Gnss4Data::PositionLLH(GnssData::PositionLLH);

    Gnss5Data::Gnss5Data(const GnssData& base) :
        GnssData(base, DescriptorSet::DataGNSS5)
    {}

    Gnss5Data::Gnss5Data(const Descriptor& descriptor) :
        GnssData(descriptor, DescriptorSet::DataGNSS5)
    {}

    Gnss5Data::Gnss5Data(const std::string& name, const uint8& descriptor) :
        GnssData(name, descriptor, DescriptorSet::DataGNSS5)
    {}

    const Gnss5Data Gnss5Data::PositionLLH(GnssData::PositionLLH);
} // mscl