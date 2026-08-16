add_library(
    gyphysics_geometry
    src/TriMeshData.cpp
)
add_library(GyPhysics::Geometry ALIAS gyphysics_geometry)

target_include_directories(
    gyphysics_geometry
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(gyphysics_geometry PUBLIC GyPhysics::Math)
target_compile_features(gyphysics_geometry PUBLIC cxx_std_17)
gyphysics_enable_warnings(gyphysics_geometry)

if(BUILD_TESTING)
    add_subdirectory(tests)
endif()
