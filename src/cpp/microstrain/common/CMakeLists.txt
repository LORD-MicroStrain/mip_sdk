
# microstrain_common is defined by the C api.
target_sources(microstrain_common PRIVATE
    embedded_time.hpp
    index.hpp
    logging.hpp
    platform.hpp
    serialization.hpp
    serialization/readwrite.hpp
    serialization/serializer.hpp
)

target_compile_features(microstrain_common PUBLIC cxx_std_11)
