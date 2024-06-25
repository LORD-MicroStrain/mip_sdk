
# microstrain_common is defined by the C api.
target_sources(microstrain_common PUBLIC
    serialization.hpp
    descriptor_id.hpp
    index.hpp
    embedded_time.hpp
    logging.hpp
    platform.hpp
)

target_compile_features(microstrain_common PUBLIC cxx_std_11)
