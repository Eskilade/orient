#define CATCH_CONFIG_MAIN

#include <catch.hpp>
#include <gtsam/base/numericalDerivative.h>
#include <fromAxisAngle.hpp>
#include <iostream>

using namespace gtsam;
#define Wrap(FNAME) \
  [](auto&& ... args){\
    return FNAME(std::forward<decltype(args)>(args) ... );\
  }

#define Str(x) #x
#define TestName(a,b,c) Str(a ## b ## c)

#define MAKE_TEST(a1, a2, a3) \
  TEST_CASE(TestName(a1,a2,a3))\
  {\
    std::srand(0);\
    Vect<3> aa;\
    SECTION("Random"){\
      aa = Vect<3>::Random();\
    }\
    /*SECTION("GimballLock_1_TaitBryan"){\
      aa = Vect<3>::Random();\
      aa[1] = M_PI/2.0;\
    }\
    SECTION("GimballLock_2_TaitBryan"){\
      aa = Vect<3>::Random();\
      aa[1] = - M_PI/2.0;\
    }\
    SECTION("GimballLock_1_ProperEuler"){\
      aa = Vect<3>::Random();\
      aa[1] = M_PI;\
    }\
    SECTION("GimballLock_2_ProperEuler"){\
      aa = Vect<3>::Random();\
      aa[1] = M_PI;\
    }*/\
    Mat<3,3> R = toR<a1, a2, a3>(aa);\
    R = toR<a1, a2, a3>(aa); \
    Mat<3,9> calc;\
    Vect<3> angles = rotationMatrixToEuler<a1, a2, a3>(R, calc);\
    Mat<3,3> R2 = toR<a1, a2, a3>(angles);\
    CHECK( R2.isApprox(R) ); \
    auto wrap = [](auto&& ... args){\
      return rotationMatrixToEuler<a1, a2, a3>(std::forward<decltype(args)>(args) ... );\
    };\
    auto num = numericalDerivative11<Vect<3>, Mat<3,3>>(wrap, R);\
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
