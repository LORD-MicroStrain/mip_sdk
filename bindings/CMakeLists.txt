
option(MIP_BIND_PYTHON "Enables the python bindings." ON)

if(MIP_BIND_PYTHON)
    add_subdirectory("python")
endif()
