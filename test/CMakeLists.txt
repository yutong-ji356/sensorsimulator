# test/CMakeLists.txt

#simple test executables

add_executable(test_quaternion test_quaternion.cpp)
target_link_libraries(test_quaternion PRIVATE sensorsim)
add_test(NAME quaternion_test COMMAND test_quaternion)

add_executable(test_trajectory test_trajectory.cpp)
target_link_libraries(test_trajectory PRIVATE sensorsim)
add_test(NAME trajectory_test COMMAND test_trajectory)