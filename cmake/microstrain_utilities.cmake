macro(microstrain_setup_install_headers LIBRARY)
#    message(WARNING "Lib headers: ${LIBRARY}")

    # Only install headers that we build the source files for
    get_target_property(ALL_HEADERS ${LIBRARY} SOURCES)
    list(FILTER ALL_HEADERS INCLUDE REGEX "^.*\.(h|hpp)$")
#    message(WARNING "${LIBRARY} Headers: ${ALL_HEADERS}")
#    foreach(HEADER ${ALL_HEADERS})
#        string(REPLACE "${CMAKE_CURRENT_LIST_DIR}/" "" HEADER_RELATIVE "${HEADER}")
#        set(HEADER_DESTINATION_FULL "${CMAKE_INSTALL_INCLUDEDIR}/${HEADER_RELATIVE}")
#        get_filename_component(HEADER_DESTINATION "${HEADER_DESTINATION_FULL}" DIRECTORY)
#        install(
#            FILES "${HEADER}"
#            DESTINATION "${HEADER_DESTINATION}"
#            COMPONENT ${LIBRARY}
#        )
#    endforeach()
endmacro()

macro(microstrain_setup_library_install LIBRARY EXPORT_TARGET)
#    message(WARNING "Lib install: ${LIBRARY}")

    microstrain_setup_install_headers(${LIBRARY})

    install(
        TARGETS ${LIBRARY}
        EXPORT mip-targets
        ARCHIVE
        COMPONENT ${LIBRARY}
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/
    )
#    message(WARNING "${LIBRARY} install dir: ${CMAKE_INSTALL_LIBDIR}")

    include(CMakePackageConfigHelpers)

    set(CONFIG_EXPORT_DIR "${CMAKE_INSTALL_DATADIR}/cmake/${LIBRARY}")
#    message(WARNING "${LIBRARY} config dir: ${CONFIG_EXPORT_DIR}")

#    install(
#        EXPORT mip-targets
#        COMPONENT ${LIBRARY}
#        DESTINATION ${CONFIG_EXPORT_DIR}
#    )

    install(
        FILES "${CMAKE_BINARY_DIR}/${LIBRARY}-config.cmake" "${CMAKE_BINARY_DIR}/${LIBRARY}-config-version.cmake"
        DESTINATION ${CONFIG_EXPORT_DIR}
        COMPONENT ${LIBRARY}
    )
endmacro()
