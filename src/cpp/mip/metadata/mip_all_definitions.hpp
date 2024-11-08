
#include "definitions/commands_base.hpp"
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

static constexpr inline std::initializer_list< const std::initializer_list<const FieldInfo*>* > ALL_COMMANDS = {
    &COMMANDS_3DM,
    &COMMANDS_AIDING,
    &COMMANDS_BASE,
    &COMMANDS_FILTER,
    &COMMANDS_GNSS,
    &COMMANDS_RTK,
    &COMMANDS_SYSTEM,
};
static constexpr inline std::initializer_list< const std::initializer_list<const FieldInfo*>* > ALL_DATA = {
    &DATA_FILTER,
    &DATA_GNSS,
    &DATA_SENSOR,
    &DATA_SHARED,
    &DATA_SYSTEM,
};

static constexpr inline const DescriptorSet DATA_GNSS1(data_gnss::MIP_GNSS1_DATA_DESC_SET, "data_gnss_1", DATA_GNSS_DS.allFields());
static constexpr inline const DescriptorSet DATA_GNSS2(data_gnss::MIP_GNSS2_DATA_DESC_SET, "data_gnss_2", DATA_GNSS_DS.allFields());
static constexpr inline const DescriptorSet DATA_GNSS3(data_gnss::MIP_GNSS3_DATA_DESC_SET, "data_gnss_3", DATA_GNSS_DS.allFields());
static constexpr inline const DescriptorSet DATA_GNSS4(data_gnss::MIP_GNSS4_DATA_DESC_SET, "data_gnss_4", DATA_GNSS_DS.allFields());
static constexpr inline const DescriptorSet DATA_GNSS5(data_gnss::MIP_GNSS5_DATA_DESC_SET, "data_gnss_5", DATA_GNSS_DS.allFields());

// Caution: these must be in ascending order!
static constexpr inline const DescriptorSet* ALL_DESCRIPTOR_SETS[] = {
    /* 0x01 */ &COMMANDS_BASE,
    /* 0x0C */ &COMMANDS_3DM,
    /* 0x0D */ &COMMANDS_FILTER,
    /* 0x0E */ &COMMANDS_GNSS,
    /* 0x0F */ &COMMANDS_RTK,
    /* 0x13 */ &COMMANDS_AIDING,
    /* 0x7F */ &COMMANDS_SYSTEM,
    /* 0x80 */ &DATA_SENSOR,
    /* 0x81 */ &DATA_GNSS,
    /* 0x82 */ &DATA_FILTER,
    /* 0x91 */ &DATA_GNSS1,
    /* 0x92 */ &DATA_GNSS2,
    /* 0x93 */ &DATA_GNSS3,
    /* 0x94 */ &DATA_GNSS4,
    /* 0x95 */ &DATA_GNSS5,
    /* 0xA0 */ &DATA_SYSTEM,
};

static constexpr inline std::initializer_list< const std::initializer_list< const FieldInfo* >* > MIP_FIELDS = {
    // Commands
    &COMMANDS_3DM,
    &COMMANDS_AIDING,
    &COMMANDS_BASE,
    &COMMANDS_FILTER,
    &COMMANDS_GNSS,
    &COMMANDS_RTK,
    &COMMANDS_SYSTEM,
    // Data
    &DATA_FILTER,
    &DATA_GNSS,
    &DATA_SENSOR,
    &DATA_SHARED,
    &DATA_SYSTEM,
};

static constexpr inline Definitions definitions(ALL_DESCRIPTOR_SETS);

} // namespace mip::metadata
