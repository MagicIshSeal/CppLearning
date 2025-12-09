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

# SDL3 Setup - Build from submodule
set(SDL_SHARED ON CACHE BOOL "Build SDL3 as shared library" FORCE)
set(SDL_STATIC OFF CACHE BOOL "Don't build static library" FORCE)
set(SDL_TEST OFF CACHE BOOL "Don't build SDL3 test programs" FORCE)
set(SDL_EXAMPLES OFF CACHE BOOL "Don't build SDL3 example programs" FORCE)
add_subdirectory(external/SDL3 EXCLUDE_FROM_ALL)

# ImGui Setup - Build from submodule
set(IMGUI_DIR ${CMAKE_SOURCE_DIR}/external/imgui)
add_library(imgui STATIC
    ${IMGUI_DIR}/imgui.cpp
    ${IMGUI_DIR}/imgui_draw.cpp
    ${IMGUI_DIR}/imgui_tables.cpp
    ${IMGUI_DIR}/imgui_widgets.cpp
    ${IMGUI_DIR}/imgui_demo.cpp
    ${IMGUI_DIR}/backends/imgui_impl_sdl3.cpp
    ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
)
target_include_directories(imgui PUBLIC 
    ${IMGUI_DIR}
    ${IMGUI_DIR}/backends
)
target_link_libraries(imgui PUBLIC SDL3::SDL3)

# Source files
set(ATMOSPHERE_SRC src/atmosphere.cpp)
set(AERO_SRC src/aero.cpp)
set(INTEGRATOR_SRC src/integrator.cpp)
set(PID_SRC src/pid.cpp)

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

# PID controller library
add_library(pid OBJECT ${PID_SRC})
target_include_directories(pid PUBLIC src)

# Main executable
add_executable(FlightDynamics src/main.cpp)
target_link_libraries(FlightDynamics atmosphere aero integrator)
target_include_directories(FlightDynamics PRIVATE src)

# GUI executable with ImGui
add_executable(FlightDynamicsGUI src/main_gui.cpp)
target_link_libraries(FlightDynamicsGUI 
    atmosphere aero integrator pid
    imgui
    SDL3::SDL3
    opengl32
)
target_include_directories(FlightDynamicsGUI PRIVATE src)

# SDL3.dll will be automatically placed next to the executable by SDL3's CMake configuration

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

# PID controller tests
add_executable(pid_tests tests/pid_tests.cpp)
target_link_libraries(pid_tests catch_amalgamated pid)
target_include_directories(pid_tests PRIVATE src tests)
add_test(NAME PIDTests COMMAND pid_tests)

# Custom target to run all tests
add_custom_target(run_tests
    COMMAND ${CMAKE_CTEST_COMMAND} -C $<CONFIG> --output-on-failure
    DEPENDS atmos_tests aero_tests integrator_tests pid_tests
    COMMENT "Running all tests..."
)

# Optionally run tests after build (can be disabled with -DRUN_TESTS_POST_BUILD=OFF)
option(RUN_TESTS_POST_BUILD "Run tests automatically after building GUI" ON)
if(RUN_TESTS_POST_BUILD)
    add_custom_command(
        TARGET FlightDynamicsGUI POST_BUILD
        COMMAND ${CMAKE_CTEST_COMMAND} -C $<CONFIG> --output-on-failure
        COMMENT "Running tests after build..."
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    )
endif()

# Installation rules for creating releases
install(TARGETS FlightDynamicsGUI
    RUNTIME DESTINATION .
)

# Install SDL3.dll alongside the executable
install(FILES $<TARGET_FILE:SDL3::SDL3-shared>
    DESTINATION .
)

# Create a custom target for packaging releases
add_custom_target(package_release
    COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_BINARY_DIR}/FlightDynamicsGUI-Release
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:FlightDynamicsGUI> ${CMAKE_BINARY_DIR}/FlightDynamicsGUI-Release/
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:SDL3::SDL3-shared> ${CMAKE_BINARY_DIR}/FlightDynamicsGUI-Release/
    COMMAND ${CMAKE_COMMAND} -E tar cfv ${CMAKE_BINARY_DIR}/FlightDynamicsGUI-Release.zip --format=zip ${CMAKE_BINARY_DIR}/FlightDynamicsGUI-Release
    DEPENDS FlightDynamicsGUI
    COMMENT "Creating release package..."
)
