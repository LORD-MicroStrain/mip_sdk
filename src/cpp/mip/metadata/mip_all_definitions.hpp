
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

#include "mip_definitions.hpp"

namespace mip::metadata
{

static constexpr inline const DescriptorSet DATA_GNSS1_DS(data_gnss::MIP_GNSS1_DATA_DESC_SET, "data_gnss_1", DATA_GNSS_DS.allFields());
static constexpr inline const DescriptorSet DATA_GNSS2_DS(data_gnss::MIP_GNSS2_DATA_DESC_SET, "data_gnss_2", DATA_GNSS_DS.allFields());
static constexpr inline const DescriptorSet DATA_GNSS3_DS(data_gnss::MIP_GNSS3_DATA_DESC_SET, "data_gnss_3", DATA_GNSS_DS.allFields());
static constexpr inline const DescriptorSet DATA_GNSS4_DS(data_gnss::MIP_GNSS4_DATA_DESC_SET, "data_gnss_4", DATA_GNSS_DS.allFields());
static constexpr inline const DescriptorSet DATA_GNSS5_DS(data_gnss::MIP_GNSS5_DATA_DESC_SET, "data_gnss_5", DATA_GNSS_DS.allFields());

static constexpr inline const DescriptorSet* ALL_DESCRIPTOR_SETS[] = {
    &COMMANDS_3DM_DS,
    &COMMANDS_AIDING_DS,
    &COMMANDS_BASE_DS,
    &COMMANDS_FILTER_DS,
    &COMMANDS_GNSS_DS,
    &COMMANDS_RTK_DS,
    &COMMANDS_SYSTEM_DS,
    &DATA_FILTER_DS,
    &DATA_GNSS_DS,
    &DATA_GNSS1_DS,
    &DATA_GNSS2_DS,
    &DATA_GNSS3_DS,
    &DATA_GNSS4_DS,
    &DATA_GNSS5_DS,
    &DATA_SENSOR_DS,
    &DATA_SYSTEM_DS,
};

static constexpr inline Definitions definitions(ALL_DESCRIPTOR_SETS);

} // namespace mip::metadata
