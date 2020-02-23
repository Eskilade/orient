cmake_minimum_required(VERSION 3.0)
find_package(GTSAM)
set(CMAKE_CXX_STANDARD 17)

###############################################################################
## target definitions #########################################################
###############################################################################

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

add_test(testTest 
  test/testEulerFromRotationMatrix.cpp
  test/testFromQuaternion.cpp src/fromQuaternion.cpp
  test/testFromAngleAxis.cpp src/fromAngleAxis.cpp
  test/testFromRotationMatrix.cpp src/fromRotationMatrix.cpp
  test/testTrigo.cpp
  test/testFromEuler.cpp
  test/testNormalize.cpp)
