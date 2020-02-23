#define CATCH_CONFIG_MAIN
#include <catch.hpp>

#include <detail/trigonometric_derivatives.hpp>

#include <gtsam/base/numericalDerivative.h>

#define WRAP(f) \
  [&] (auto&&... args) -> decltype(auto) \
  { return f (std::forward<decltype(args)>(args)...); }


TEST_CASE("asin")
{
  auto x = 0.3;
  auto num = gtsam::numericalDerivative11<double, double>(WRAP(detail::asin), x);
  Eigen::Matrix<double, 1, 1> calc;
  detail::asin(x, calc(0,0));
  CHECK( calc.isApprox( num, 1e-10) ); 
}

TEST_CASE("acos")
{
  auto x = 0.3;
  auto num = gtsam::numericalDerivative11<double, double>(WRAP(detail::acos), x);
  Eigen::Matrix<double, 1, 1> calc;
  detail::acos(x, calc(0,0));
  CHECK( calc.isApprox( num, 1e-10) ); 
}

TEST_CASE("atan2")
{
  auto x = 0.3;
  auto y = 0.9;
  Eigen::Matrix<double, 1, 1> calc1, calc2;
  auto num1 = gtsam::numericalDerivative21<double, double, double>(WRAP(detail::atan2), y, x);
  auto num2 = gtsam::numericalDerivative22<double, double, double>(WRAP(detail::atan2), y, x);
  detail::atan2(y, x, calc1(0,0), calc2(0,0));
  CHECK( calc1.isApprox( num1, 1e-10) ); 
  CHECK( calc2.isApprox( num2, 1e-10) ); 
}
