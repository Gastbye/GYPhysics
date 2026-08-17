add_library(
    gyphysics_material
    src/Material.cpp
    src/MaterialRegistry.cpp
)
add_library(GyPhysics::Material ALIAS gyphysics_material)

target_include_directories(
    gyphysics_material
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(gyphysics_material PUBLIC GyPhysics::Math)
target_compile_features(gyphysics_material PUBLIC cxx_std_17)
gyphysics_enable_warnings(gyphysics_material)

if(BUILD_TESTING)
    add_subdirectory(tests)
endif()
