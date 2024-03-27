
set(PYTHON_DIR "${CMAKE_CURRENT_LIST_DIR}")

set(PYTHON_SOURCES
    "${CMAKE_CURRENT_LIST_DIR}/src/main.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/src/mip_python.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/src/mip_python.hpp"
    "${CMAKE_CURRENT_LIST_DIR}/src/commands_3dm.cpp"
)


add_subdirectory(pybind11)
pybind11_add_module(mip_python ${PYTHON_SOURCES})

# Target name can't be "mip" because the top-level MIP SDK target has that name.
# Rename the output file so we can import it like "import mip" instead of "import mip_python".
# Note the name must match the module declaration in the source code otherwise
# python gives "ImportError: dynamic module does not define module export function (PyInit_mip_python)".
set_target_properties(mip_python PROPERTIES OUTPUT_NAME "mip")

target_link_libraries(mip_python PUBLIC mip)
