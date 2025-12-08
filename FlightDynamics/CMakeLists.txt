cmake_minimum_required(VERSION 3.15)
project(FlightDynamics)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Output all object files to build directory
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

# Enable testing
enable_testing()

# SDL3 Setup
set(SDL3_DIR ${CMAKE_SOURCE_DIR}/external/SDL2)
set(SDL3_INCLUDE_DIR ${SDL3_DIR}/include)
set(SDL3_LIB_DIR ${SDL3_DIR}/lib/x64)

# ImGui Setup
set(IMGUI_DIR ${CMAKE_SOURCE_DIR}/external/imgui)
set(IMGUI_SOURCES
    ${IMGUI_DIR}/imgui.cpp
    ${IMGUI_DIR}/imgui_draw.cpp
    ${IMGUI_DIR}/imgui_tables.cpp
    ${IMGUI_DIR}/imgui_widgets.cpp
    ${IMGUI_DIR}/imgui_demo.cpp
    ${IMGUI_DIR}/backends/imgui_impl_sdl3.cpp
    ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
)

# Source files
set(ATMOSPHERE_SRC src/atmosphere.cpp)
set(AERO_SRC src/aero.cpp)
set(INTEGRATOR_SRC src/integrator.cpp)

# Catch2 test framework
add_library(catch_amalgamated STATIC tests/catch_amalgamated.cpp)
target_include_directories(catch_amalgamated PUBLIC tests)

# Atmosphere library
add_library(atmosphere OBJECT ${ATMOSPHERE_SRC})
target_include_directories(atmosphere PUBLIC src)

# Aero library
add_library(aero OBJECT ${AERO_SRC})
target_include_directories(aero PUBLIC src)

# Integrator library
add_library(integrator OBJECT ${INTEGRATOR_SRC})
target_include_directories(integrator PUBLIC src)

# Main executable
add_executable(FlightDynamics src/main.cpp)
target_link_libraries(FlightDynamics atmosphere aero integrator)
target_include_directories(FlightDynamics PRIVATE src)

# GUI executable with ImGui
add_executable(FlightDynamicsGUI src/main_gui.cpp ${IMGUI_SOURCES})
target_link_libraries(FlightDynamicsGUI 
    atmosphere aero integrator
    ${SDL3_LIB_DIR}/SDL3.lib
    opengl32
)
target_include_directories(FlightDynamicsGUI PRIVATE 
    src 
    ${SDL3_INCLUDE_DIR}
    ${IMGUI_DIR}
    ${IMGUI_DIR}/backends
)

# Copy SDL3.dll to build directory
add_custom_command(TARGET FlightDynamicsGUI POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    ${SDL3_LIB_DIR}/SDL3.dll
    $<TARGET_FILE_DIR:FlightDynamicsGUI>
)

# Atmosphere tests
add_executable(atmos_tests tests/atmos_tests.cpp)
target_link_libraries(atmos_tests catch_amalgamated atmosphere)
target_include_directories(atmos_tests PRIVATE src tests)
add_test(NAME AtmosphereTests COMMAND atmos_tests)

# Aero tests
add_executable(aero_tests tests/aero_tests.cpp)
target_link_libraries(aero_tests catch_amalgamated aero atmosphere)
target_include_directories(aero_tests PRIVATE src tests)
add_test(NAME AeroTests COMMAND aero_tests)

# Integrator tests
add_executable(integrator_tests tests/integrator_tests.cpp)
target_link_libraries(integrator_tests catch_amalgamated integrator)
target_include_directories(integrator_tests PRIVATE src tests)
add_test(NAME IntegratorTests COMMAND integrator_tests)

# Custom target to run all tests
add_custom_target(run_tests
    COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure
    DEPENDS atmos_tests aero_tests integrator_tests
    COMMENT "Running all tests..."
)
