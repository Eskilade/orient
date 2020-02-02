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

#add_test(example src/simu.cpp)
#add_test(aa2eu src/aa2eu.cpp src/rodrigues.cpp)
#add_test(OptionalRef src/OptionalRef.cpp)
#add_test(testEulerFromRotationMatrix test/testEulerFromRotationMatrix.cpp)
add_test(fromQuaternion test/testFromQuaternion.cpp src/fromQuaternion.cpp)
#add_test(trigo test/testTrigo.cpp)
