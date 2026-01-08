# examples/CMakeLists.txt

# Trajectory demo
add_executable(trajectory_demo 01_trajectory_demo.cpp)
target_link_libraries(trajectory_demo PRIVATE sensorsim)

# Future examples (will implement later)
# add_executable(complementary_demo 02_complementary_filter.cpp)
# target_link_libraries(complementary_demo PRIVATE sensorsim)

# add_executable(ekf_demo 03_ekf_navigation.cpp)
# target_link_libraries(ekf_demo PRIVATE sensorsim)