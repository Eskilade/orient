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

add_test(testEulerFromRotationMatrix test/testEulerFromRotationMatrix.cpp)
add_test(fromQuaternion test/testFromQuaternion.cpp src/fromQuaternion.cpp)
add_test(fromAngleAxis test/testFromAngleAxis.cpp src/fromAngleAxis.cpp)
add_test(fromRotationMatrix test/testFromRotationMatrix.cpp src/fromRotationMatrix.cpp)
add_test(trigo test/testTrigo.cpp)
add_test(fromEuler test/testFromEuler.cpp)
add_test(testNormalize test/testNormalize.cpp)
