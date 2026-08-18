cmake_minimum_required(VERSION 3.20)

project(
    GyPhysics
    VERSION 0.1.0
    DESCRIPTION "A modular physics simulation framework"
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(GY_PHYSICS_BUILD_EXAMPLES "Build GyPhysics example programs" ON)

set(
    GY_PHYSICS_REAL_TYPE
    "DOUBLE"
    CACHE STRING
    "Floating-point type used by GyPhysics: FLOAT or DOUBLE"
)
set_property(CACHE GY_PHYSICS_REAL_TYPE PROPERTY STRINGS FLOAT DOUBLE)
string(TOUPPER "${GY_PHYSICS_REAL_TYPE}" GY_PHYSICS_REAL_TYPE)

if(GY_PHYSICS_REAL_TYPE STREQUAL "DOUBLE")
    set(GY_PHYSICS_REAL_IS_DOUBLE 1)
elseif(GY_PHYSICS_REAL_TYPE STREQUAL "FLOAT")
    set(GY_PHYSICS_REAL_IS_DOUBLE 0)
else()
    message(FATAL_ERROR
        "GY_PHYSICS_REAL_TYPE must be FLOAT or DOUBLE, but got: "
        "${GY_PHYSICS_REAL_TYPE}"
    )
endif()

find_package(Eigen3 REQUIRED NO_MODULE)

include(CTest)
include(cmake/GyPhysicsWarnings.cmake)

add_subdirectory(modules/math)
add_subdirectory(modules/geometry)
add_subdirectory(modules/material)
add_subdirectory(modules/mechanics)
add_subdirectory(modules/rigid)

if(GY_PHYSICS_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

message(STATUS "GyPhysics scalar type: ${GY_PHYSICS_REAL_TYPE}")
message(STATUS "GyPhysics build tests: ${BUILD_TESTING}")
message(STATUS "GyPhysics build examples: ${GY_PHYSICS_BUILD_EXAMPLES}")
