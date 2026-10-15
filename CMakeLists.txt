# carla-track-control: the track_control library, an offline race-track simulation
# (bicycle and Project Chrono plants), and the Python module the CARLA bridge uses.
#
#   cmake -S . -B build && cmake --build build -j       (setup.sh does this)
#
# Python (for CasADi and pybind11) is the repository's .venv unless
# Python3_EXECUTABLE is given. Project Chrono is optional: it is looked for in
# external/chrono/install (scripts/install_chrono.sh puts it there), then in
# Chrono_DIR; without it only the bicycle plant is available.
cmake_minimum_required(VERSION 3.18)
project(carla_track_control CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
# no fused multiply-add, so results do not depend on the CPU the code was built for
add_compile_options(-Wall -Wextra -ffp-contract=off)
# CasADi's pip wheel uses the old libstdc++ string ABI; so must everything linked with it, Chrono included
add_compile_definitions(_GLIBCXX_USE_CXX11_ABI=0)

# ---- Python: CasADi's CMake package and pybind11 come from its site-packages

if(NOT Python3_EXECUTABLE AND EXISTS ${CMAKE_SOURCE_DIR}/.venv/bin/python)
    set(Python3_EXECUTABLE ${CMAKE_SOURCE_DIR}/.venv/bin/python)
endif()
find_package(Python3 COMPONENTS Interpreter Development.Module REQUIRED)
if(NOT casadi_DIR)
    execute_process(
        COMMAND ${Python3_EXECUTABLE} -c "import casadi, os; print(os.path.join(os.path.dirname(casadi.__file__), 'cmake'))"
        OUTPUT_VARIABLE casadi_DIR OUTPUT_STRIP_TRAILING_WHITESPACE RESULT_VARIABLE casadi_missing)
    if(casadi_missing)
        message(FATAL_ERROR "CasADi not found in ${Python3_EXECUTABLE}: pip install casadi")
    endif()
endif()
execute_process(
    COMMAND ${Python3_EXECUTABLE} -m pybind11 --cmakedir
    OUTPUT_VARIABLE pybind11_DIR OUTPUT_STRIP_TRAILING_WHITESPACE)
find_package(pybind11 CONFIG REQUIRED)
find_package(casadi CONFIG REQUIRED)

# ---- the library

enable_testing()
set(TRACK_CONTROL_BUILD_TESTS ON)  # a normal variable overrides the subproject's option() (CMP0077)
set(TRACK_CONTROL_BUILD_EXAMPLES ON)
add_subdirectory(track_control)

# ---- Project Chrono (optional)

find_package(Chrono COMPONENTS Vehicle CONFIG QUIET
    PATHS $ENV{CHRONO_ROOT}/install/lib/cmake/Chrono ${CMAKE_SOURCE_DIR}/external/chrono/install/lib/cmake/Chrono
          $ENV{HOME}/opt/chrono/install/lib/cmake/Chrono)

# ---- the offline simulation: plants, race track, perception

set(SIM_SOURCES
    sim/plant/bicycle_plant.cpp
    sim/racetrack/circuit.cpp
    sim/racetrack/perception.cpp
    sim/racetrack/driver.cpp
    sim/racetrack/run_lap.cpp)
if(Chrono_FOUND)
    list(APPEND SIM_SOURCES sim/plant/chrono_plant.cpp)
else()
    list(APPEND SIM_SOURCES sim/plant/chrono_plant_missing.cpp)
endif()

add_library(sim STATIC ${SIM_SOURCES})
target_include_directories(sim PUBLIC ${CMAKE_SOURCE_DIR})
target_link_libraries(sim PUBLIC track_control::track_control PRIVATE nlohmann_json::nlohmann_json)
target_compile_definitions(sim PUBLIC SIM_CONFIG_DIR="${CMAKE_SOURCE_DIR}/config")
get_target_property(CASADI_LIBRARY casadi::casadi IMPORTED_LOCATION_RELEASE)
get_filename_component(CASADI_LIB_DIR ${CASADI_LIBRARY} DIRECTORY)
set(RPATH ${CASADI_LIB_DIR})

if(Chrono_FOUND)
    message(STATUS "Project Chrono found: the Chrono plant is available")
    target_include_directories(sim SYSTEM PRIVATE ${CHRONO_INCLUDE_DIRS})
    # Chrono is built with -march=native; its headers must be compiled the same way
    set_source_files_properties(sim/plant/chrono_plant.cpp PROPERTIES COMPILE_OPTIONS "-march=native;-fopenmp")
    target_compile_definitions(sim PRIVATE OV_CHRONO_DATA="${CHRONO_DATA_DIR}" OV_CHRONO_VEHICLE_DATA="${CHRONO_VEHICLE_DATA_DIR}")
    target_compile_definitions(sim PUBLIC SIM_HAVE_CHRONO=1)
    target_link_libraries(sim PUBLIC ${CHRONO_LIBRARIES})
    list(APPEND RPATH ${CHRONO_LIB_DIR})
else()
    message(STATUS "Project Chrono not found: only the bicycle plant is available (scripts/install_chrono.sh)")
    target_compile_definitions(sim PUBLIC SIM_HAVE_CHRONO=0)
endif()

add_executable(simulate sim/simulate.cpp)
target_link_libraries(simulate PRIVATE sim nlohmann_json::nlohmann_json)
set_target_properties(simulate PROPERTIES BUILD_RPATH "${RPATH}")

if(Chrono_FOUND)
    add_executable(identify_chrono sim/plant/identify_chrono.cpp)
    target_link_libraries(identify_chrono PRIVATE sim nlohmann_json::nlohmann_json)
    set_target_properties(identify_chrono PROPERTIES BUILD_RPATH "${RPATH}")
endif()

# ---- Python module for the CARLA bridge: build/python/trackctl*.so

pybind11_add_module(trackctl bindings/trackctl.cpp)
target_link_libraries(trackctl PRIVATE sim)
set_target_properties(trackctl PROPERTIES BUILD_RPATH "${RPATH}" LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/python)
