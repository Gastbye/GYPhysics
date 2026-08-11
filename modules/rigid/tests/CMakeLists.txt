add_executable(gyphysics_rigid_tests RigidBodyTest.cpp)

target_link_libraries(gyphysics_rigid_tests PRIVATE GyPhysics::Rigid)
gyphysics_enable_warnings(gyphysics_rigid_tests)

add_test(NAME gyphysics.rigid COMMAND gyphysics_rigid_tests)

