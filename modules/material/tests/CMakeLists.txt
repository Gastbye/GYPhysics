add_executable(gyphysics_material_tests MaterialTest.cpp)
target_link_libraries(gyphysics_material_tests PRIVATE GyPhysics::Material)
gyphysics_enable_warnings(gyphysics_material_tests)
add_test(NAME gyphysics.material COMMAND gyphysics_material_tests)

add_executable(
    gyphysics_material_registry_tests
    MaterialRegistryTest.cpp
)
target_link_libraries(
    gyphysics_material_registry_tests
    PRIVATE GyPhysics::Material
)
gyphysics_enable_warnings(gyphysics_material_registry_tests)
add_test(
    NAME gyphysics.material_registry
    COMMAND gyphysics_material_registry_tests
)
