target_sources(UnitTests PRIVATE
    "logging.unit.cpp"
    "strings.unit.cpp"
)

target_sources(IntegrationTests PRIVATE
    "mip.integration.cpp"
    # TODO: Broken, can't be fixed until naming conventions decided
    #"packet_interface.integration.cpp"
)
