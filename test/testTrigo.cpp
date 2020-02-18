#define CATCH_CONFIG_MAIN

#include <catch.hpp>

#include <iostream>
#include <detail/trigonometric_derivatives.hpp>
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
  Eigen::Matrix<double, 1, 1> calc;
  detail::asin(x, calc(0,0));
  CHECK( calc.isApprox( num, 1e-10) ); 
}

TEST_CASE("acos")
{
  auto x = 0.3;
  auto num = numericalDerivative11<double, double>(Wrap(detail::acos), x);
  Eigen::Matrix<double, 1, 1> calc;
  detail::acos(x, calc(0,0));
  CHECK( calc.isApprox( num, 1e-10) ); 
}

TEST_CASE("atan2")
{
  auto x = 0.3;
  auto y = 0.9;
  Eigen::Matrix<double, 1, 1> calc1, calc2;
  auto num1 = numericalDerivative21<double, double, double>(Wrap(detail::atan2), y, x);
  auto num2 = numericalDerivative22<double, double, double>(Wrap(detail::atan2), y, x);
  detail::atan2(y, x, calc1(0,0), calc2(0,0));
  CHECK( calc1.isApprox( num1, 1e-10) ); 
  CHECK( calc2.isApprox( num2, 1e-10) ); 
}
