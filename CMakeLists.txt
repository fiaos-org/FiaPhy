cmake_minimum_required(VERSION 3.10)
project(FiaPhy VERSION 1.0.0 LANGUAGES CXX)

# C++ Standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Compiler flags
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -O2)
endif()

# Library source files (header-only currently, but prepared for future .cpp files)
set(FIAPHY_HEADERS
    src/FiaPhy.h
    src/core/Types.h
    src/core/SensorHub.h
    src/core/PhysicsEngine.h
    src/physics/Thermodynamics.h
    src/filters/INR.h
    src/platform/Logger.h
)

# Create interface library (header-only)
add_library(FiaPhy INTERFACE)

target_include_directories(FiaPhy INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>
)

# Install rules
install(DIRECTORY src/
    DESTINATION include/FiaPhy
    FILES_MATCHING PATTERN "*.h"
)

install(TARGETS FiaPhy
    EXPORT FiaPhyTargets
    INCLUDES DESTINATION include
)

install(EXPORT FiaPhyTargets
    FILE FiaPhyTargets.cmake
    NAMESPACE FiaPhy::
    DESTINATION lib/cmake/FiaPhy
)

# Create package config file
include(CMakePackageConfigHelpers)

configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/FiaPhyConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/FiaPhyConfig.cmake
    INSTALL_DESTINATION lib/cmake/FiaPhy
)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/FiaPhyConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/FiaPhyConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/FiaPhyConfigVersion.cmake
    DESTINATION lib/cmake/FiaPhy
)

# Build examples (optional)
option(BUILD_EXAMPLES "Build example programs" OFF)

if(BUILD_EXAMPLES)
    add_subdirectory(examples/RaspberryPi)
endif()

# Print summary
message(STATUS "")
message(STATUS "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
message(STATUS "FiaPhy Configuration Summary")
message(STATUS "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
message(STATUS "Version: ${PROJECT_VERSION}")
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Install prefix: ${CMAKE_INSTALL_PREFIX}")
message(STATUS "Build examples: ${BUILD_EXAMPLES}")
message(STATUS "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
message(STATUS "")
