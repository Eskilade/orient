#include <catch/catch2.hpp>

#include <orient/detail/trigonometric_derivatives.hpp>

#include <gtsam/base/numericalDerivative.h>

#define WRAP(f) \
  [&] (auto&&... args) -> decltype(auto) \
  { return f (std::forward<decltype(args)>(args)...); }


TEST_CASE("asin")
{
  auto x = 0.3;
  auto num = gtsam::numericalDerivative11<double, double>(WRAP(std::asin), x);
  const auto [angle, angleHx] = orient::detail::asinWD(x);
  CHECK( angle == Approx(std::asin(x)) );
  CHECK( angleHx == Approx(num(0)) ); 
}

TEST_CASE("acos")
{
  auto x = 0.3;
  auto num = gtsam::numericalDerivative11<double, double>(WRAP(std::acos), x);
  const auto [angle, angleHx] = orient::detail::acosWD(x);
  CHECK( angle == Approx(std::acos(x)) );
  CHECK( angleHx == Approx(num(0)) );
}

TEST_CASE("atan2")
{
  auto x = 0.3;
  auto y = 0.9;
  Eigen::Matrix<double, 1, 1> calc1, calc2;
  auto num_Hy = gtsam::numericalDerivative21<double, double, double>(WRAP(std::atan2), y, x);
  auto num_Hx = gtsam::numericalDerivative22<double, double, double>(WRAP(std::atan2), y, x);
  const auto [angle, Hy, Hx] = orient::detail::atan2WD(y, x);
  CHECK(angle == Approx( std::atan2(y,x) ));
  CHECK( Hy == Approx(num_Hy(0)) ); 
  CHECK( Hx == Approx(num_Hx(0)) ); 
}
