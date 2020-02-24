#include <catch/catch2.hpp>

#include <orient/normalize.hpp>

#include <gtsam/base/numericalDerivative.h>

TEST_CASE("normalize")
{
  Eigen::Vector4d v = Eigen::Vector4d::Random();
  Eigen::Vector4d nv = orient::normalize(v);
  double norm = nv.dot(nv);
  CHECK( std::abs( norm - 1.0) < std::numeric_limits<double>::epsilon() );
}

TEST_CASE("normalize_derivative")
{
  auto wrap = boost::function<Eigen::Vector4d(Eigen::Vector4d const&)>{[](auto&& ... args){
    return orient::normalize(std::forward<decltype(args)>(args) ... );
  }};

  Eigen::Vector4d v = Eigen::Vector4d::Random();
  auto num = gtsam::numericalDerivative11(wrap, v);
  Eigen::Matrix<double, 4, 4> calc;
  orient::normalize(v, calc);
  CHECK( calc.isApprox( num, 1e-9) );
}
