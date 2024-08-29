
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

// Caution: these must be in ascending order!
static constexpr inline const DescriptorSet* ALL_DESCRIPTOR_SETS[] = {
    /* 0x01 */ &COMMANDS_BASE_DS,
    /* 0x0C */ &COMMANDS_3DM_DS,
    /* 0x0D */ &COMMANDS_FILTER_DS,
    /* 0x0E */ &COMMANDS_GNSS_DS,
    /* 0x0F */ &COMMANDS_RTK_DS,
    /* 0x13 */ &COMMANDS_AIDING_DS,
    /* 0x7F */ &COMMANDS_SYSTEM_DS,
    /* 0x80 */ &DATA_SENSOR_DS,
    /* 0x81 */ &DATA_GNSS_DS,
    /* 0x82 */ &DATA_FILTER_DS,
    /* 0x91 */ &DATA_GNSS1_DS,
    /* 0x92 */ &DATA_GNSS2_DS,
    /* 0x93 */ &DATA_GNSS3_DS,
    /* 0x94 */ &DATA_GNSS4_DS,
    /* 0x95 */ &DATA_GNSS5_DS,
    /* 0xA0 */ &DATA_SYSTEM_DS,
};

static constexpr inline Definitions definitions(ALL_DESCRIPTOR_SETS);

} // namespace mip::metadata
