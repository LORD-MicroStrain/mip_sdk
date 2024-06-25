
#include <mip/definitions/commands_3dm.hpp>

#include <mip/metadata/mip_metadata.hpp>

namespace mip::metadata
{

template<>
struct MetadataFor<commands_3dm::MessageFormat>
{
    using Cmd = commands_3dm::MessageFormat;

    static constexpr inline FieldInfo value = {
        /* .name        = */ "MessageFormat",
        /* .title       = */ "Message Format",
        /* .docs        = */ "Sets up the message format.",
        /* .parameters  = */ {
            utils::FUNCTION_SELECTOR_PARAM,
            {
                /* .name          = */ "desc_set",
                /* .docs          = */ "Data descriptor set. Must be supported. When function is SAVE, LOAD, or DEFAULT, can be 0 to apply to all descriptor sets.",
                /* .type          = */ {Type::U8},
                /* .accessor      = */ utils::access<Cmd,uint8_t,&Cmd::desc_set>,
                /* .byte_offset   = */ 1,
                /* .functions     = */ ALL_FUNCTIONS,
            },
            {
                /* .name          = */ "desc_set",
                /* .docs          = */ "Number of descriptors (limited by payload size)",
                /* .type          = */ {Type::U8},
                /* .accessor      = */ utils::access<Cmd,uint8_t,&Cmd::desc_set>,
                /* .byte_offset   = */ 2,
                /* .functions     = */ ALL_FUNCTIONS,
                /* .count         = */ 1,
                /* .counter_idx   = */ 0,
            },
            {
                /* .name          = */ "descriptors",
                /* .docs          = */ "List of descriptors and decimations.",
                /* .type          = */ {Type::STRUCT, &MetadataFor<DescriptorRate>::value},
                /* .accessor      = */ utils::access<Cmd,uint8_t,&Cmd::desc_set>,
                /* .byte_offset   = */ 3,
                /* .functions     = */ ALL_FUNCTIONS,
                /* .count         = */ 0,
                /* .counter_idx   = */ 2,
            },
        },
        /* .functions   = */ ALL_FUNCTIONS,
        /* .proprietary = */ false,
        /* .response    = */ nullptr,
    };
};

} // namespace mip::metadata

