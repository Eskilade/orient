#define CATCH_CONFIG_MAIN

#include <catch.hpp>

#include <iostream>
#include <common.hpp>
#include <traits.hpp>
#include <type_traits>

template<Axis A1, Axis A2, Axis A3, std::enable_if_t<isTaitBryan<A1,A2,A3>(), int> = 0>
Vect<3> rotationMatrixToEuler(Mat<3,3> const& R)
{
  using T = TaitBryanTraits<A1,A2,A3>;
  const auto W = Wrap{R};
  const auto lone_v = W.uncorrected(T::a2s);
  const auto thres = 1.0 - std::numeric_limits<float>::epsilon();
  double a1,a2,a3;
  if( lone_v < thres ){
    if( lone_v > - thres ){
      a2 = std::asin( W(T::a2s));
      a1 = std::atan2(W(T::a1s), W(T::a1c) );
      a3 = std::atan2(W(T::a3s), W(T::a3c) );
    } 
    else { // Gimbal lock. lone value ~= -1
      using TGL = typename T::GimbalLock;
      a2 =  - W.sign(T::a2s) * M_PI / 2.0;
      a1 =  - std::atan2( W(TGL::a1s), W(TGL::a1c) );
      a3 = 0.;
    }
  }
  else { // Gimbal lock. lone value ~= +1
    using TGL = typename T::GimbalLock;
    a2 =  W.sign(T::a2s) * M_PI / 2.0;
    a1 =  std::atan2( W(TGL::a1s), W(TGL::a1c) );
    a3 = 0.;
  }
  return (Vect<3>() << a1,a2,a3).finished();
}

template<Axis A1, Axis A2, Axis A3, std::enable_if_t<isProperEuler<A1,A2,A3>(), int> = 0>
Vect<3> rotationMatrixToEuler(Mat<3,3> const& R)
{
  using T = ProperEulerTraits<A1,A2>; 
  const auto W = Wrap{R};
  const auto lone_v = W.uncorrected(T::a2c);
  const auto thres = 1.0 - std::numeric_limits<float>::epsilon();
  double a1,a2,a3;
  if( lone_v < thres ){
    if( lone_v > - thres ){
      a2 = std::acos( W(T::a2c));
      a1 = std::atan2(W.uncorrected(T::a1s), W(T::a1c) );
      a3 = std::atan2(W.uncorrected(T::a3s), W(T::a3c) );
    } 
    else { // Gimbal lock. lone value ~= -1
      using TGL = typename T::GimbalLock;
      a2 =  M_PI;
      a1 =  - std::atan2( W(TGL::a1s), W(TGL::a1c) );
      a3 = 0.;
    }
  }
  else { // Gimbal lock. lone value ~= +1
    using TGL = typename T::GimbalLock;
    a2 =  0.0;
    a1 =  std::atan2( W(TGL::a1s), W(TGL::a1c) );
    a3 = 0.;
  }
  return (Vect<3>() << a1,a2,a3).finished();
}

template<Axis A1, Axis A2, Axis A3, std::enable_if_t<isMalformed<A1,A2,A3>(), int> = 0>
void rotationMatrixToEuler(Mat<3,3> const&)
{
  constexpr bool alwaysFalse = ( A1 != A1);
  static_assert( alwaysFalse, "Passed a malformed rotation sequence. Please choose either a propor euler sequence or a Tait-Bryan sequence");
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
    SECTION("GimballLock_1_TaitBryan"){\
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
    }\
    Mat<3,3> R = toR<a1, a2, a3>(aa);\
    R = toR<a1, a2, a3>(aa); \
    Vect<3> angles = rotationMatrixToEuler<a1, a2, a3>(R);\
    Mat<3,3> R2 = toR<a1, a2, a3>(angles);\
    CHECK( R2.isApprox(R) ); \
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
