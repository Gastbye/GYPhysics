add_executable(gyphysics_rigid_tests RigidBodyTest.cpp)

target_link_libraries(gyphysics_rigid_tests PRIVATE GyPhysics::Rigid)
gyphysics_enable_warnings(gyphysics_rigid_tests)

add_test(NAME gyphysics.rigid COMMAND gyphysics_rigid_tests)

add_executable(
    gyphysics_rigid_body_registry_tests
    RigidBodyRegistryTest.cpp
)

target_link_libraries(
    gyphysics_rigid_body_registry_tests
    PRIVATE GyPhysics::Rigid
)
gyphysics_enable_warnings(gyphysics_rigid_body_registry_tests)

add_test(
    NAME gyphysics.rigid_body_registry
    COMMAND gyphysics_rigid_body_registry_tests
)

add_executable(
    gyphysics_rigid_body_actor_tests
    RigidBodyActorTest.cpp
)

target_link_libraries(
    gyphysics_rigid_body_actor_tests
    PRIVATE GyPhysics::Rigid
)
gyphysics_enable_warnings(gyphysics_rigid_body_actor_tests)

add_test(
    NAME gyphysics.rigid_body_actor
    COMMAND gyphysics_rigid_body_actor_tests
)

