

#include <mip/metadata/definitions/commands_3dm.hpp>
#include <mip/metadata/definitions/commands_aiding.hpp>
#include <mip/metadata/definitions/commands_base.hpp>
#include <mip/metadata/definitions/commands_filter.hpp>
#include <mip/metadata/definitions/commands_gnss.hpp>
#include <mip/metadata/definitions/commands_rtk.hpp>
#include <mip/metadata/definitions/commands_system.hpp>

#include "definitions/data_filter.hpp"
#include "definitions/data_gnss.hpp"
#include "definitions/data_sensor.hpp"
#include "definitions/data_shared.hpp"
#include "definitions/data_system.hpp"


namespace mip::metadata
{

static constexpr inline std::initializer_list< const std::initializer_list<const FieldInfo*>* > ALL_COMMANDS = {
    &ALL_COMMANDS_3DM,
    &ALL_COMMANDS_AIDING,
    &ALL_COMMANDS_BASE,
    &ALL_COMMANDS_FILTER,
    &ALL_COMMANDS_GNSS,
    &ALL_COMMANDS_RTK,
    &ALL_COMMANDS_SYSTEM,
};
static constexpr inline std::initializer_list< const std::initializer_list<const FieldInfo*>* > ALL_DATA = {
    &ALL_DATA_FILTER,
    &ALL_DATA_GNSS,
    &ALL_DATA_SENSOR,
    &ALL_DATA_SHARED,
    &ALL_DATA_SYSTEM,
};

static constexpr inline std::initializer_list< const std::initializer_list< const FieldInfo* >* > ALL_FIELDS = {
    // Commands
    &ALL_COMMANDS_3DM,
    &ALL_COMMANDS_AIDING,
    &ALL_COMMANDS_BASE,
    &ALL_COMMANDS_FILTER,
    &ALL_COMMANDS_GNSS,
    &ALL_COMMANDS_RTK,
    &ALL_COMMANDS_SYSTEM,
    // Data
    &ALL_DATA_FILTER,
    &ALL_DATA_GNSS,
    &ALL_DATA_SENSOR,
    &ALL_DATA_SHARED,
    &ALL_DATA_SYSTEM,
};

} // namespace mip

