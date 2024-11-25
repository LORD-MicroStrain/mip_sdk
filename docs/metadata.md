Metadata  {#metadata}
========

The MIP SDK includes an additional set of definitions for each MIP field which contain additional information beyond
that needed for the core functionality of sending commands and receiving data. It includes human-readable field names
and information on each parameter, such as name, type, size, etc. This information is accessible at compile time and
can be used for things like pretty-printing, CSV generation, and more.

Metadata is currently an experimental feature and is subject to changes. It requires C++17 or later.

Usage:
1. Include the metadata definition header for the corresponding descriptor set.
2. Get a reference or pointer to the metadata for your field.
3. Access members of the reference.

~~~~~~~~{.cpp}
#include <cstdio>

template<class FieldType>
void print_parameters()
{
    // Obtain a reference for easier access.
    constexpr const mip::metadata::FieldInfo& field = mip::metadata::MetadataFor<FieldType>::value;
    
    // Print field name and descriptor.
    std::printf("Field (0x%02X,%02X) %s:\n", field.descriptor.descriptorSet, field.descriptor.fieldDescriptor, field.title);

    // Iterate each parameter.
    unsigned int p=0;
    for(const mip::metadata::ParameterInfo& param : field.parameters)
    {
        // Single element (most parameters)?
        if(param.count.count == 1)
            std::printf("  Parameter %u: %s\n", p, param.name);
            
        // Fixed-size array?
        else if(param.count.isFixed())
            std::printf("  Parameter %u: %s[%u]\n", p, param.name, param.count.count);
            
        // Variable-sized array with another parameter holding the count?
        else if(param.count.hasCounter())
            std::printf("  Parameter %u: %s[%s]\n", p, param.name, field.parameters[param.count.counter.index()].name);
            
        // Variable-length array which is determined by the payload length.
        else
            std::printf("  Parameter %u: %s[]\n", p, param.name);

        p++;
    }
    
    std::printf("\n");
}

#include <mip/metadata/definitions/commands_3dm.hpp>
#include <mip/metadata/definitions/data_sensor.hpp>

int main()
{
    print_parameters<mip::commands_3dm::MessageFormat>();
    print_parameters<mip::commands_3dm::PpsSource>();
    print_parameters<mip::data_sensor::ScaledAccel>();

    return 0;
}

~~~~~~~~
