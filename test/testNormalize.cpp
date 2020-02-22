#define CATCH_CONFIG_MAIN

#include <catch.hpp>
#include <gtsam/base/numericalDerivative.h>
#include <normalize.hpp>

#define Wrap(FNAME, RetT, ...) \
  boost::function<RetT (__VA_ARGS__ )>{[](auto&& ... args){\
    return FNAME(std::forward<decltype(args)>(args) ... );\
  }}

TEST_CASE("normalize")
{
  Eigen::Vector4d v = Eigen::Vector4d::Random();
  Eigen::Vector4d nv = normalize(v);
  double norm = nv.dot(nv);
  CHECK( std::abs( norm - 1.0) < std::numeric_limits<double>::epsilon() );
}

TEST_CASE("normalize_derivative")
{
  auto wrap = boost::function<Eigen::Vector4d(Eigen::Vector4d const&)>{[](auto&& ... args){
    return normalize(std::forward<decltype(args)>(args) ... );
  }};

  Eigen::Vector4d v = Eigen::Vector4d::Random();
  auto num = gtsam::numericalDerivative11(wrap, v);
  Eigen::Matrix<double, 4, 4> calc;
  normalize(v, calc);
  CHECK( calc.isApprox( num, 1e-9) );
}
