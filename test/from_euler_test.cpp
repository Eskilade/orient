#include <catch/catch2.hpp>

#include <orient/from_euler.hpp>

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Rot3.h>

#define WRAP(f) \
  [&] (auto&&... args) -> decltype(auto) \
  { return f (std::forward<decltype(args)>(args)...); }

#define MAKE_TEST_SINGLE(a1) \
  TEST_CASE(#a1)\
  {\
    double angle;\
    SECTION("Zero"){\
      angle = 0;\
    }\
    SECTION("Random"){\
      angle = 0.2;\
    }\
    const Eigen::Matrix3d expected = gtsam::Rot3::R##a1(angle).matrix();\
    Eigen::Matrix3d actual = orient::rotationMatrixFromEuler<orient::Axis::a1>(angle);\
    CHECK( actual.isApprox(expected) );\
  }\
  TEST_CASE(#a1 "_derivative")\
  {\
    double angle;\
    SECTION("Zero"){\
      angle = 0;\
    }\
    SECTION("Random"){\
      angle = 0.2;\
    }\
    Eigen::Matrix3d H;\
    orient::rotationMatrixFromEuler<orient::Axis::a1>(angle, H);\
    /* TODO: Investigate why a wrapper is needed here.
     * an unresolved overloaded function type error */\
    auto numeric = gtsam::numericalDerivative11<Eigen::Matrix3d, double>(WRAP(orient::rotationMatrixFromEuler<orient::Axis::a1>), angle);\
    CHECK( H.isApprox(Eigen::Map<Eigen::Matrix3d>(numeric.data(), 3,3), 1e-10) );\
  }\

#define MAKE_TEST_FULL(a1, a2, a3) \
  /* Set static seeds for unit test repeatability */ \
  TEST_CASE(#a1 "_" #a2 "_" #a3)\
  {\
    std::srand(0);\
    Eigen::Vector3d aa;\
    SECTION("Zero"){\
      aa = Eigen::Vector3d::Zero();\
    }\
    SECTION("Random"){\
      aa = Eigen::Vector3d::Random();\
    }\
    const Eigen::Matrix3d expected = \
      gtsam::Rot3::R##a1(aa[0]).matrix()*\
      gtsam::Rot3::R##a2(aa[1]).matrix()*\
      gtsam::Rot3::R##a3(aa[2]).matrix();\
    Eigen::Matrix3d actual = orient::rotationMatrixFromEuler<orient::Axis::a1, orient::Axis::a2, orient::Axis::a3>(aa);\
    CHECK( actual.isApprox(expected) );\
  }\
  TEST_CASE(#a1 "_" #a2 "_" #a3 "_derivative")\
  {\
    std::srand(2);\
    Eigen::Vector3d aa;\
    SECTION("Zero"){\
      aa = Eigen::Vector3d::Zero();\
    }\
    SECTION("Random"){\
      aa = Eigen::Vector3d::Random();\
    }\
    Eigen::Matrix<double, 9, 3> H;\
    orient::rotationMatrixFromEuler<orient::Axis::a1, orient::Axis::a2, orient::Axis::a3>(aa, H);\
    const auto numeric = gtsam::numericalDerivative11(\
                          orient::rotationMatrixFromEuler<orient::Axis::a1, orient::Axis::a2, orient::Axis::a3>, aa);\
    CHECK( H.isApprox(numeric, 1e-10) );\
  }\

MAKE_TEST_SINGLE(x)
MAKE_TEST_SINGLE(y)
MAKE_TEST_SINGLE(z)

MAKE_TEST_FULL(x, y, z)
MAKE_TEST_FULL(y, x, z)
MAKE_TEST_FULL(z, x, y)
MAKE_TEST_FULL(x, z, y)
MAKE_TEST_FULL(y, z, x)
MAKE_TEST_FULL(z, y, x)
