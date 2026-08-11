add_library(gyphysics_math INTERFACE)
add_library(GyPhysics::Math ALIAS gyphysics_math)

target_include_directories(
    gyphysics_math
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(gyphysics_math INTERFACE Eigen3::Eigen)
target_compile_definitions(
    gyphysics_math
    INTERFACE GY_PHYSICS_REAL_IS_DOUBLE=${GY_PHYSICS_REAL_IS_DOUBLE}
)
target_compile_features(gyphysics_math INTERFACE cxx_std_17)

if(BUILD_TESTING)
    add_subdirectory(tests)
endif()

