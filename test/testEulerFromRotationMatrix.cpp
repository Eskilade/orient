#define CATCH_CONFIG_MAIN

#include <catch.hpp>
#include <gtsam/base/numericalDerivative.h>
#include <eulerFromRotationMatrix.hpp>
#include <iostream>

using namespace gtsam;
#define Wrap(FNAME) \
  [](auto&& ... args){\
    return FNAME(std::forward<decltype(args)>(args) ... );\
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
    Mat<3,3> R = toR<a1, a2, a3>(aa);\
    Eigen::Vector3d angles = eulerFromRotationMatrix<a1, a2, a3>(R);\
    Mat<3,3> R2 = toR<a1, a2, a3>(angles);\
    CHECK( R2.isApprox(R) ); \
  }\
  TEST_CASE(#a1 "_" #a2 "_" #a3 "_derivative")\
  {\
    std::srand(1);\
    Eigen::Vector3d aa;\
    SECTION("Random"){\
      aa = Eigen::Vector3d::Random();\
    }\
    auto wrap = [](auto&& ... args){\
      return eulerFromRotationMatrix<a1, a2, a3>(std::forward<decltype(args)>(args) ... );\
    };\
    Mat<3,3> R = toR<a1, a2, a3>(aa);\
    auto num = numericalDerivative11<Eigen::Vector3d, Mat<3,3>>(wrap, R);\
    Mat<3,9> calc;\
    eulerFromRotationMatrix<a1, a2, a3>(R, calc);\
    CHECK( calc.isApprox(num, 1e-6) );\
  }



// Invalid configurations are commented

//MAKE_TEST(Axis::x, Axis::x, Axis::x)
//MAKE_TEST(Axis::x, Axis::x, Axis::y)
//MAKE_TEST(Axis::x, Axis::x, Axis::z)
  MAKE_TEST(Axis::x, Axis::y, Axis::x)
//MAKE_TEST(Axis::x, Axis::y, Axis::y)
  MAKE_TEST(Axis::x, Axis::y, Axis::z)
  MAKE_TEST(Axis::x, Axis::z, Axis::x)
  MAKE_TEST(Axis::x, Axis::z, Axis::y)
//MAKE_TEST(Axis::x, Axis::z, Axis::z)
//MAKE_TEST(Axis::y, Axis::x, Axis::x)
  MAKE_TEST(Axis::y, Axis::x, Axis::y)
  MAKE_TEST(Axis::y, Axis::x, Axis::z)
//MAKE_TEST(Axis::y, Axis::y, Axis::x)
//MAKE_TEST(Axis::y, Axis::y, Axis::y)
//MAKE_TEST(Axis::y, Axis::y, Axis::z)
  MAKE_TEST(Axis::y, Axis::z, Axis::x)
  MAKE_TEST(Axis::y, Axis::z, Axis::y)
//MAKE_TEST(Axis::y, Axis::z, Axis::z)
//MAKE_TEST(Axis::z, Axis::x, Axis::x)
  MAKE_TEST(Axis::z, Axis::x, Axis::y)
  MAKE_TEST(Axis::z, Axis::x, Axis::z)
  MAKE_TEST(Axis::z, Axis::y, Axis::x)
//MAKE_TEST(Axis::z, Axis::y, Axis::y)
  MAKE_TEST(Axis::z, Axis::y, Axis::z)
//MAKE_TEST(Axis::z, Axis::z, Axis::x)
//MAKE_TEST(Axis::z, Axis::z, Axis::y)
//MAKE_TEST(Axis::z, Axis::z, Axis::z)
