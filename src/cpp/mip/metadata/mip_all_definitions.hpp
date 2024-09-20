
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

static constexpr inline std::initializer_list< const std::initializer_list< const FieldInfo* >* > ALL_FIELDS = {
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

} // namespace mip::metadata
