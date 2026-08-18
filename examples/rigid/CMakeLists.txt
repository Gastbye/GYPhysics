add_executable(gyphysics_rigid_example main.cpp)

target_link_libraries(
    gyphysics_rigid_example
    PRIVATE GyPhysics::Rigid GyPhysics::Geometry
)
gyphysics_enable_warnings(gyphysics_rigid_example)

