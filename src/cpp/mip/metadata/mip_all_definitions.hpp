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

static constexpr inline const DescriptorSetInfo DATA_GNSS1 =
{
    .descriptor = mip::data_gnss::MIP_GNSS1_DATA_DESC_SET,
    .name = "Gnss1 Data",
    .fields = DATA_GNSS_FIELDS,
};
static constexpr inline const DescriptorSetInfo DATA_GNSS2 =
{
    .descriptor = mip::data_gnss::MIP_GNSS2_DATA_DESC_SET,
    .name = "Gnss2 Data",
    .fields = DATA_GNSS_FIELDS,
};
static constexpr inline const DescriptorSetInfo DATA_GNSS3 =
{
    .descriptor = mip::data_gnss::MIP_GNSS3_DATA_DESC_SET,
    .name = "Gnss3 Data",
    .fields = DATA_GNSS_FIELDS,
};
static constexpr inline const DescriptorSetInfo DATA_GNSS4 =
{
    .descriptor = mip::data_gnss::MIP_GNSS4_DATA_DESC_SET,
    .name = "Gnss4 Data",
    .fields = DATA_GNSS_FIELDS,
};
static constexpr inline const DescriptorSetInfo DATA_GNSS5 =
{
    .descriptor = mip::data_gnss::MIP_GNSS5_DATA_DESC_SET,
    .name = "Gnss5 Data",
    .fields = DATA_GNSS_FIELDS,
};


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

} // namespace mip::metadata
