add_library(
    gyphysics_rigid
    src/RigidBody.cpp
    src/RigidBodyRegistry.cpp
)
add_library(GyPhysics::Rigid ALIAS gyphysics_rigid)

target_include_directories(
    gyphysics_rigid
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(gyphysics_rigid PUBLIC GyPhysics::Math GyPhysics::Mechanics)
target_compile_features(gyphysics_rigid PUBLIC cxx_std_17)
gyphysics_enable_warnings(gyphysics_rigid)

if(BUILD_TESTING)
    add_subdirectory(tests)
endif()

