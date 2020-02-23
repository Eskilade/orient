#define CATCH_CONFIG_MAIN
#include <catch.hpp>

#include <eulerFromRotationMatrix.hpp>

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Rot3.h>

// Define a helper class to map axis of rotation
// to the gtsam function calculating the rotation
// matrix of that rotation

template<Axis>
struct Rot;

#define MakeRotMat(axis) \
  template<> struct Rot<Axis::axis> { \
    static gtsam::Matrix3 Mat(double a){ return gtsam::Rot3::R##axis(a).matrix();}; \
  }

MakeRotMat(x);
MakeRotMat(y);
MakeRotMat(z);

template<Axis a1, Axis a2, Axis a3>
Eigen::Matrix3d ToGtsamRotationMatrix(Eigen::Vector3d const& v)
{
  return Rot<a1>::Mat(v[0]) * Rot<a2>::Mat(v[1]) * Rot<a3>::Mat(v[2]);
}

#define MAKE_TEST(a1, a2, a3) \
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
    SECTION("GimballLock_1_TaitBryan"){\
      aa = Eigen::Vector3d::Random();\
      aa[1] = M_PI/2.0;\
    }\
    SECTION("GimballLock_2_TaitBryan"){\
      aa = Eigen::Vector3d::Random();\
      aa[1] = - M_PI/2.0;\
    }\
    SECTION("GimballLock_1_ProperEuler"){\
      aa = Eigen::Vector3d::Random();\
      aa[1] = M_PI;\
    }\
    SECTION("GimballLock_2_ProperEuler"){\
      aa = Eigen::Vector3d::Random();\
      aa[1] = 0;\
    }\
    Eigen::Matrix3d R = ToGtsamRotationMatrix<a1, a2, a3>(aa);\
    Eigen::Vector3d angles = eulerFromRotationMatrix<a1, a2, a3>(R);\
    Eigen::Matrix3d R2 = ToGtsamRotationMatrix<a1, a2, a3>(angles);\
    CHECK( R2.isApprox(R) ); \
  }\
  TEST_CASE(#a1 "_" #a2 "_" #a3 "_derivative")\
  {\
    std::srand(1);\
    Eigen::Vector3d aa;\
    SECTION("Random"){\
      aa = Eigen::Vector3d::Random();\
    }\
    Eigen::Matrix3d R = ToGtsamRotationMatrix<a1, a2, a3>(aa);\
    auto num = gtsam::numericalDerivative11(eulerFromRotationMatrix<a1, a2, a3>, R);\
    Eigen::Matrix<double, 3, 9> calc;\
    eulerFromRotationMatrix<a1, a2, a3>(R, calc);\
    CHECK( calc.isApprox(num, 1e-6) );\
  }

MAKE_TEST(Axis::x, Axis::y, Axis::x)
MAKE_TEST(Axis::x, Axis::y, Axis::z)
MAKE_TEST(Axis::x, Axis::z, Axis::x)
MAKE_TEST(Axis::x, Axis::z, Axis::y)
MAKE_TEST(Axis::y, Axis::x, Axis::y)
MAKE_TEST(Axis::y, Axis::x, Axis::z)
MAKE_TEST(Axis::y, Axis::z, Axis::x)
MAKE_TEST(Axis::y, Axis::z, Axis::y)
MAKE_TEST(Axis::z, Axis::x, Axis::y)
MAKE_TEST(Axis::z, Axis::x, Axis::z)
MAKE_TEST(Axis::z, Axis::y, Axis::x)
MAKE_TEST(Axis::z, Axis::y, Axis::z)
