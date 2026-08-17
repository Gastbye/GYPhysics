add_executable(gyphysics_geometry_tests GeometryTest.cpp)

target_link_libraries(gyphysics_geometry_tests PRIVATE GyPhysics::Geometry)
gyphysics_enable_warnings(gyphysics_geometry_tests)

add_test(NAME gyphysics.geometry COMMAND gyphysics_geometry_tests)

add_executable(
    gyphysics_solid_geometry_properties_tests
    SolidGeometryPropertiesTest.cpp
)

target_link_libraries(
    gyphysics_solid_geometry_properties_tests
    PRIVATE GyPhysics::Geometry
)
gyphysics_enable_warnings(gyphysics_solid_geometry_properties_tests)

add_test(
    NAME gyphysics.solid_geometry_properties
    COMMAND gyphysics_solid_geometry_properties_tests
)

add_executable(
    gyphysics_shape_registry_tests
    ShapeRegistryTest.cpp
)

target_link_libraries(
    gyphysics_shape_registry_tests
    PRIVATE GyPhysics::Geometry
)
gyphysics_enable_warnings(gyphysics_shape_registry_tests)

add_test(
    NAME gyphysics.shape_registry
    COMMAND gyphysics_shape_registry_tests
)
