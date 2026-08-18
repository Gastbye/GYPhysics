add_library(gyphysics_mechanics INTERFACE)
add_library(GyPhysics::Mechanics ALIAS gyphysics_mechanics)

target_include_directories(
    gyphysics_mechanics
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(gyphysics_mechanics INTERFACE GyPhysics::Math)
target_compile_features(gyphysics_mechanics INTERFACE cxx_std_17)
