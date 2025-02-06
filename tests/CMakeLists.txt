add_executable(GJKTests GJKTests.cpp)
add_test(NAME GJKTests COMMAND GJKTests)

add_executable(RigidBodyTests RigidBodyTests.cpp)
target_link_libraries(RigidBodyTests UPhysics)
add_test(NAME RigidBodyTests COMMAND RigidBodyTests)