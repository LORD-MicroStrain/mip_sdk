#pragma once

#include "mip_definitions.hpp"

#include "definitions/commands_3dm.hpp"
#include "definitions/commands_aiding.hpp"
#include "definitions/commands_base.hpp"
#include "definitions/commands_filter.hpp"
#include "definitions/commands_gnss.hpp"
#include "definitions/commands_rtk.hpp"
#include "definitions/commands_system.hpp"

#include "definitions/data_filter.hpp"
#include "definitions/data_gnss.hpp"
#include "definitions/data_sensor.hpp"
#include "definitions/data_shared.hpp"
#include "definitions/data_system.hpp"

namespace mip::metadata
{
#define DECLARE_GNSS_DATASET_I(I) \
    static constexpr inline const DescriptorSetInfo DATA_GNSS##I = \
    { \
        .descriptor = mip::data_gnss::MIP_GNSS##I##_DATA_DESC_SET, \
        .name       = "Gnss" #I " Data", \
        .fields     = DATA_GNSS_FIELDS, \
    }; \
    struct DataSetGnss##I : public DataSetGnss \
    { \
        static constexpr inline const uint8_t             DESCRIPTOR_SET = DATA_GNSS##I.descriptor; \
        static constexpr inline const CompositeDescriptor DESCRIPTOR     = {DESCRIPTOR_SET, INVALID_FIELD_DESCRIPTOR}; \
    };\
    template<> \
    struct MetadataFor<DataSetGnss##I> : public MetadataFor<DataSetGnss> \
    { \
        using type = DataSetGnss##I; \
        static inline constexpr const DescriptorSetInfo& value = DATA_GNSS##I; \
    };\

    DECLARE_GNSS_DATASET_I(1)
    DECLARE_GNSS_DATASET_I(2)
    DECLARE_GNSS_DATASET_I(3)
    DECLARE_GNSS_DATASET_I(4)
    DECLARE_GNSS_DATASET_I(5)

#undef DECLARE_GNSS_DATASET_I

    static constexpr inline const DescriptorSetInfo* ALL_DESCRIPTOR_SETS[] = {
        // Commands
        &COMMANDS_BASE,
        &COMMANDS_3DM,
        &COMMANDS_FILTER,
        &COMMANDS_GNSS,
        &COMMANDS_RTK,
        &COMMANDS_AIDING,
        &COMMANDS_SYSTEM,
        // Data
        &DATA_SENSOR,
        &DATA_GNSS,
        &DATA_FILTER,
        &DATA_GNSS1,
        &DATA_GNSS2,
        &DATA_GNSS3,
        &DATA_GNSS4,
        &DATA_GNSS5,
        &DATA_SYSTEM,
        &DATA_SHARED,
    };

    using AllDescriptorSets = std::tuple<
        // Commands
        CommandSetBase,
        CommandSet3dm,
        CommandSetFilter,
        CommandSetGnss,
        CommandSetRtk,
        CommandSetAiding,
        CommandSetSystem,
        // Data
        DataSetSensor,
        DataSetGnss,
        DataSetFilter,
        DataSetGnss1,
        DataSetGnss2,
        DataSetGnss3,
        DataSetGnss4,
        DataSetGnss5,
        DataSetSystem,
        DataSetShared
    >;

} // namespace mip::metadata
