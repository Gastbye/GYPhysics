add_executable(gyphysics_math_tests MathTypesTest.cpp)

target_link_libraries(gyphysics_math_tests PRIVATE GyPhysics::Math)
gyphysics_enable_warnings(gyphysics_math_tests)

add_test(NAME gyphysics.math COMMAND gyphysics_math_tests)
