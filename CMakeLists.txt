cmake_minimum_required(VERSION 3.0)
find_package(Eigen3 REQUIRED)
find_package(GTSAM 4.0.2 REQUIRED)
set(CMAKE_CXX_STANDARD 17)

# TODO : This should be compiled as a library, then linked with test
# Fix this given https://stackoverflow.com/q/61577040/2992024 
add_executable(
  test_orient 
  src/from_rotation_matrix.cpp
  test/from_quaternion_test.cpp 
  test/from_angle_axis_test.cpp
  test/from_rotation_matrix_test.cpp
  test/euler_from_rotation_matrix_test.cpp
  test/derivative_helpers_test.cpp
  test/from_euler_test.cpp)

target_compile_options(test_orient PUBLIC -Wall -Wextra -Wfloat-conversion)
target_include_directories(test_orient PUBLIC include ${GTSAM_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR})
target_link_libraries(test_orient PUBLIC gtsam)
