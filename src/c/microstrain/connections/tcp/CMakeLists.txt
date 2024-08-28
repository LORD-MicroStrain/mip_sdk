set(MICROSTRAIN_SOCKET_LIBRARY "microstrain_socket" CACHE INTERNAL "Name of the MicroStrain TCP socket connections library")
mark_as_advanced(MICROSTRAIN_SOCKET_LIBRARY)

add_library(${MICROSTRAIN_SOCKET_LIBRARY}
    "tcp_socket.h"
    "tcp_socket.c"
)

target_compile_features(${MICROSTRAIN_SOCKET_LIBRARY} PUBLIC c_std_11)
target_link_libraries(${MICROSTRAIN_SOCKET_LIBRARY}
    PRIVATE ${MICROSTRAIN_PRIVATE_LIBRARY}
    PUBLIC ${MICROSTRAIN_COMMON_LIBRARY}
)
target_compile_definitions(${MICROSTRAIN_SOCKET_LIBRARY} PUBLIC "MICROSTRAIN_ENABLE_TCP=1")

if(WIN32)
    target_link_libraries(${MICROSTRAIN_SOCKET_LIBRARY} PUBLIC "ws2_32")
endif()

target_link_libraries(${MICROSTRAIN_LIBRARIES} INTERFACE ${MICROSTRAIN_SOCKET_LIBRARY})
