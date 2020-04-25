#include <catch/catch2.hpp>

#include <orient/detail/derivative_helpers.hpp>

#include <gtsam/base/numericalDerivative.h>
#include <iostream>

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

TEST_CASE("transpose")
{
  const Eigen::Matrix3d M = Eigen::Matrix3d::Random();
  auto num = gtsam::numericalDerivative11<Eigen::Matrix3d, Eigen::Matrix3d>([](auto&& M){return M.transpose();}, M);
  const auto [v, J] = orient::detail::transposeWD(M);
  CHECK( v.isApprox(M.transpose()) );
  CHECK( J.isApprox(num, 1e-10) );
}

TEST_CASE("trace")
{
  const Eigen::Matrix3d M = Eigen::Matrix3d::Random();
  auto num = gtsam::numericalDerivative11<double, Eigen::Matrix3d>([](auto&& M){return M.trace();}, M);
  const auto [v, J] = orient::detail::traceWD(M);
  CHECK( v == Approx(M.trace()) );
  CHECK( J.isApprox(num, 1e-10) );
}
