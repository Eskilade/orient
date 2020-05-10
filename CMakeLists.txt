cmake_minimum_required(VERSION 3.7)
project(orient VERSION 0.1 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)

option(BUILD_TESTS "Enable tests" OFF)

find_package(Eigen3 REQUIRED)

# Define the library target
add_library(orient INTERFACE)
target_include_directories(orient INTERFACE 
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
		${EIGEN3_INCLUDE_DIR})

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "${PROJECT_BINARY_DIR}/orientConfigVersion.cmake"
    VERSION 0.1
    COMPATIBILITY AnyNewerVersion
)

install(TARGETS orient
    EXPORT orientTargets
    LIBRARY DESTINATION lib COMPONENT Runtime
    ARCHIVE DESTINATION lib COMPONENT Development
    RUNTIME DESTINATION bin COMPONENT Runtime
    PUBLIC_HEADER DESTINATION include COMPONENT Development
    BUNDLE DESTINATION bin COMPONENT Runtime
)

include(CMakePackageConfigHelpers)
configure_package_config_file(
    "${PROJECT_SOURCE_DIR}/cmake/orientConfig.cmake.in"
    "${PROJECT_BINARY_DIR}/orientConfig.cmake"
    INSTALL_DESTINATION lib/cmake/orient
)

install(EXPORT orientTargets DESTINATION lib/cmake/orient)
install(FILES "${PROJECT_BINARY_DIR}/orientConfigVersion.cmake"
              "${PROJECT_BINARY_DIR}/orientConfig.cmake"
        DESTINATION lib/cmake/orient)
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/ DESTINATION include)

if(BUILD_TESTS)
  enable_testing()
  find_package(GTSAM 4.0.2 REQUIRED)
  add_executable(test_orient 
    test/from_quaternion_test.cpp 
    test/from_angle_axis_test.cpp
    test/aa_quat_from_rotation_matrix_test.cpp
    test/euler_from_rotation_matrix_test.cpp
    test/derivative_helpers_test.cpp
    test/from_euler_test.cpp)
  target_compile_options(test_orient PUBLIC -Wall -Wextra -Wfloat-conversion)
  target_include_directories(test_orient PUBLIC ${GTSAM_INCLUDE_DIR})
  target_link_libraries(test_orient PUBLIC gtsam orient)
  add_test(NAME test_orient COMMAND test_orient)
endif(BUILD_TESTS)
