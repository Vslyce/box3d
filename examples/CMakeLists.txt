cmake_minimum_required(VERSION 3.22)
project(box3d_moving_mesh_example LANGUAGES C CXX)

option(BOX3D_EXAMPLE_ESOTERICA_CONFIG "Use the optional embedding configuration" OFF)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
add_subdirectory(.. box3d)

if(BOX3D_EXAMPLE_ESOTERICA_CONFIG)
    # PUBLIC applies the same header to the library and its consumers.
    target_compile_definitions(box3d PUBLIC
        BOX3D_USER_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/esoterica_config.h")
endif()

add_executable(moving_mesh moving_mesh.c)
set_target_properties(moving_mesh PROPERTIES C_STANDARD 17 C_STANDARD_REQUIRED YES C_EXTENSIONS NO)
target_link_libraries(moving_mesh PRIVATE box3d::box3d)
