# track_control: pure pursuit and NMPC for following a path, simulator-independent.
#
# Standalone:   cmake -S track_control -B build -Dcasadi_DIR=<prefix>/casadi/cmake
#               cmake --build build && ctest --test-dir build && cmake --install build --prefix <dir>
# As a subproject:   add_subdirectory(track_control)   then link track_control::track_control
# Installed:    find_package(track_control CONFIG REQUIRED)   then link track_control::track_control
#
# CasADi (with IPOPT) is found through its CMake package. The pip wheel ships one:
#   python -c "import casadi, os; print(os.path.join(os.path.dirname(casadi.__file__), 'cmake'))"
cmake_minimum_required(VERSION 3.18)
project(track_control VERSION 0.1.0 LANGUAGES CXX)

if(PROJECT_IS_TOP_LEVEL)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    if(NOT CMAKE_BUILD_TYPE)
        set(CMAKE_BUILD_TYPE Release)
    endif()
endif()

option(TRACK_CONTROL_BUILD_TESTS "Build the tests" ${PROJECT_IS_TOP_LEVEL})
option(TRACK_CONTROL_BUILD_EXAMPLES "Build the examples" ${PROJECT_IS_TOP_LEVEL})
# The CasADi pip wheels are built with the old libstdc++ string ABI; everything
# that passes a std::string to CasADi has to be as well. Turn this off for a
# CasADi built from source with the default ABI.
option(TRACK_CONTROL_OLD_CXX_ABI "Compile with _GLIBCXX_USE_CXX11_ABI=0 (needed with the CasADi pip wheel)" ON)

include(GNUInstallDirs)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(casadi CONFIG REQUIRED)
find_package(nlohmann_json 3.2 QUIET)
if(NOT nlohmann_json_FOUND)
    include(FetchContent)
    FetchContent_Declare(json URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz)
    FetchContent_MakeAvailable(json)
endif()

add_library(track_control
    src/vehicle_params.cpp
    src/bicycle.cpp
    src/path.cpp
    src/speed_profile.cpp
    src/controller.cpp
    src/pure_pursuit.cpp
    src/nmpc.cpp)
add_library(track_control::track_control ALIAS track_control)
target_compile_features(track_control PUBLIC cxx_std_17)
set_target_properties(track_control PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(track_control PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(track_control
    PUBLIC Eigen3::Eigen
    PRIVATE casadi::casadi $<BUILD_INTERFACE:nlohmann_json::nlohmann_json>)
# no fused multiply-add, so results do not depend on the CPU the code was built for
target_compile_options(track_control PRIVATE -Wall -Wextra -ffp-contract=off)
if(TRACK_CONTROL_OLD_CXX_ABI)
    target_compile_definitions(track_control PUBLIC _GLIBCXX_USE_CXX11_ABI=0)
endif()

# ---- tests and examples

if(TRACK_CONTROL_BUILD_TESTS)
    enable_testing()
    foreach(name test_path test_model test_controllers)
        add_executable(${name} tests/${name}.cpp)
        target_link_libraries(${name} PRIVATE track_control)
        add_test(NAME track_control.${name} COMMAND ${name})
    endforeach()
endif()

if(TRACK_CONTROL_BUILD_EXAMPLES)
    add_executable(track_control_example_oval examples/oval.cpp)
    target_link_libraries(track_control_example_oval PRIVATE track_control)
endif()

# ---- install, with a CMake package for find_package(track_control)

include(CMakePackageConfigHelpers)
install(TARGETS track_control EXPORT track_controlTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY include/track_control DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT track_controlTargets NAMESPACE track_control::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/track_control)
configure_package_config_file(cmake/track_controlConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/track_controlConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/track_control)
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/track_controlConfigVersion.cmake
    COMPATIBILITY SameMinorVersion)
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/track_controlConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/track_controlConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/track_control)
