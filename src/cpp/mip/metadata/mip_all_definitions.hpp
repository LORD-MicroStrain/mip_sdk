

#include <mip/metadata/definitions/commands_base.hpp>
#include <mip/metadata/definitions/commands_3dm.hpp>
#include <mip/metadata/definitions/data_sensor.hpp>


namespace mip::metadata
{

static constexpr inline std::initializer_list< const std::initializer_list<const FieldInfo*>* > ALL_COMMANDS = {
    &ALL_COMMANDS_BASE,
    // &ALL_3DM_COMMANDS,
};
static constexpr inline std::initializer_list< const std::initializer_list<const FieldInfo*>* > ALL_DATA = {
    &ALL_SENSOR_DATA,
};

static constexpr inline std::initializer_list< const std::initializer_list< const FieldInfo* >* > ALL_FIELDS = {
    // Commands
    &ALL_COMMANDS_BASE,
    // &ALL_3DM_COMMANDS,
    // Data
    &ALL_SENSOR_DATA,
};

} // namespace mip

