cmake_minimum_required(VERSION 3.0)
find_package(GTSAM)
set(CMAKE_CXX_STANDARD 17)

##########################
## target definitions ####
##########################

macro(add_test name)
  add_executable(${name} ${ARGN})
  target_compile_options(${name} PUBLIC -Wall -Wextra -Wfloat-conversion)
  target_include_directories(${name} PUBLIC 
    include

    GTSAM_INCLUDE_DIR
  )
  target_link_libraries(${name} PUBLIC
    gtsam
  )
endmacro(add_test)

add_test(test_test 
  test/euler_from_rotation_matrix_test.cpp
  test/from_quaternion_test.cpp src/from_quaternion.cpp
  test/from_angle_axis_test.cpp src/from_angle_axis.cpp
  test/from_rotation_matrix_test.cpp src/from_rotation_matrix.cpp
  test/trigonometry_test.cpp
  test/from_euler_test.cpp
  test/normalize_test.cpp)
