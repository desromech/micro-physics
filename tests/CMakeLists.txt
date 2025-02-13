add_executable(GJKTests GJKTests.cpp)
add_test(NAME GJKTests COMMAND GJKTests)

add_executable(MathTests MathTests.cpp)
target_link_libraries(MathTests UPhysics)
add_test(NAME MathTests COMMAND MathTests)

add_executable(RigidBodyTests RigidBodyTests.cpp)
target_link_libraries(RigidBodyTests UPhysics)
add_test(NAME RigidBodyTests COMMAND RigidBodyTests)

add_executable(RayCastingTest RayCastingTest.cpp)
target_link_libraries(RayCastingTest UPhysics)
