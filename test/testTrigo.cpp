#define CATCH_CONFIG_MAIN

#include <catch.hpp>

#include <iostream>
#include <trigonometric_derivatives.hpp>
#include <gtsam/base/numericalDerivative.h>

#define Wrap(FNAME) \
  [](auto&& ... args){\
    return FNAME(std::forward<decltype(args)>(args) ... );\
  }

using namespace gtsam;
TEST_CASE("asin")
{
  auto x = 0.3;
  auto num = numericalDerivative11<double, double>(Wrap(detail::asin), x);
  Mat<1,1> calc;
  detail::asin(x, calc);
  CHECK( calc.isApprox( num, 1e-10) ); 
}

TEST_CASE("acos")
{
  auto x = 0.3;
  auto num = numericalDerivative11<double, double>(Wrap(detail::acos), x);
  Mat<1,1> calc;
  detail::acos(x, calc);
  CHECK( calc.isApprox( num, 1e-10) ); 
}

TEST_CASE("atan2")
{
  auto x = 0.3;
  auto y = 0.9;
  Mat<1,1> num, calc;
  SECTION("Hy"){
    num = numericalDerivative21<double, double, double>(Wrap(detail::atan2), y, x);
    detail::atan2(y, x, calc);
  }
  SECTION("Hx"){
    num = numericalDerivative22<double, double, double>(Wrap(detail::atan2), y, x);
    detail::atan2(y, x, {}, calc);
  }
  CHECK( calc.isApprox( num, 1e-10) ); 
}
