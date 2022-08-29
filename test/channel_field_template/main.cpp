#include <iostream>

#include "ChannelField.hpp"

int main()
{
    std::string info = mscl::Gnss1Data::PositionLLH.info();
    mscl::DescriptorSet descriptorSet = mscl::Gnss1Data::PositionLLH.descriptorSet();
    mscl::Descriptor descriptor = mscl::Gnss1Data::PositionLLH.descriptor();

    std::cout << "Channel Info: " << info << std::endl;
    std::cout << "DescriptorSet: " << descriptorSet << std::endl;
    std::cout << "Descriptor: " << descriptor << std::endl;
    std::cout << mscl::Gnss1Data::PositionLLH << std::endl;

    std::cout << std::endl;

    std::cout << mscl::GnssData::PositionLLH << std::endl;
    std::cout << mscl::Gnss1Data::PositionLLH << std::endl;
    std::cout << mscl::Gnss2Data::PositionLLH << std::endl;
    std::cout << mscl::Gnss3Data::PositionLLH << std::endl;
    std::cout << mscl::Gnss4Data::PositionLLH << std::endl;
    std::cout << mscl::Gnss5Data::PositionLLH << std::endl;
    return 0;
}