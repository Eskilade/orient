#define CATCH_CONFIG_MAIN

#include <catch.hpp>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Rot3.h>
#include <Eigen/Geometry>
#include <fromAngleAxis.hpp>
#include <iostream>
#include <common.hpp>
#include <Eigen/Geometry>

using namespace gtsam;
#define Wrap(FNAME, RetT, ...) \
  boost::function<RetT (__VA_ARGS__ )>{[](auto&& ... args){\
    return FNAME(std::forward<decltype(args)>(args) ... );\
  }}

#define wrap(name) [](auto&&... args){ return name(std::forward<decltype(args)>(args) ...);}
TEST_CASE("rotationMatrixFromAngleAxis")
{ 
  Vector3 aa;
  SECTION("Random"){
    aa = Vector3::Random();
  }
  SECTION("Zero"){
    aa = Vector3::Zero();
  }

  const auto calc = rotationMatrixFromAngleAxis(aa);
  const auto act = Rot3::Expmap(aa).matrix();
  CHECK( calc.isApprox(act) );
}

TEST_CASE("rotationMatrixFromAngleAxis_derivative")
{ 
  Vector3 aa;
  SECTION("Random"){
    aa = Vector3::Random();
  }
  SECTION("Zero"){
    aa = Vector3::Zero();
  }

  Mat<9,3> calc;
  rotationMatrixFromAngleAxis(aa, calc);
  auto numeric = numericalDerivative11<Mat<3,3>, Vect<3>>(wrap(rotationMatrixFromAngleAxis), aa);
  CHECK( calc.isApprox(numeric, 1e-10) );
}

TEST_CASE("quaternionFromAngleAxis")
{
  Vect<3> aa;
  SECTION("Random"){
    aa = Vect<3>::Random();
  }
  SECTION("half_PI_angle"){
    aa = Vect<3>::Random();
    aa *= M_PI / (2*std::sqrt(aa.dot(aa)));
  }
  SECTION("minus_half_PI_angle"){
    aa = Vect<3>::Random();
    aa *= -M_PI / (2*std::sqrt(aa.dot(aa)));
  }
  SECTION("PI_angle"){
    aa = Vect<3>::Random();
    aa *= M_PI / std::sqrt(aa.dot(aa));
  }
  SECTION("minus_PI_angle"){
    aa = Vect<3>::Random();
    aa *= -M_PI / std::sqrt(aa.dot(aa));
  }
  SECTION("2PI_angle"){
    aa = Vect<3>::Random();
    aa *= 2*M_PI / std::sqrt(aa.dot(aa));
  }
  SECTION("minus_2PI_angle"){
    aa = Vect<3>::Random();
    aa *= -2*M_PI / std::sqrt(aa.dot(aa));
  }



  const auto angle = std::sqrt(aa.dot(aa));
  const Vect<3> axis = aa / angle;
  Eigen::AngleAxisd eaa{angle, axis};
  Eigen::Quaterniond equat{eaa};

  Vect<4> expected = (Vect<4>() << equat.w(), equat.vec()).finished();
  Vect<4> actual = quaternionFromAngleAxis(aa);
  CHECK( expected.isApprox(actual) );
}
TEST_CASE("quaternionFromAngleAxis_derivative")
{
  Vect<3> aa;
  SECTION("Random"){
    aa = Vect<3>::Random();
  }
  SECTION("half_PI_angle"){
    aa = Vect<3>::Random();
    aa *= M_PI / (2*std::sqrt(aa.dot(aa)));
  }
  SECTION("minus_half_PI_angle"){
    aa = Vect<3>::Random();
    aa *= -M_PI / (2*std::sqrt(aa.dot(aa)));
  }
  SECTION("PI_angle"){
    aa = Vect<3>::Random();
    aa *= M_PI / std::sqrt(aa.dot(aa));
  }
  SECTION("minus_PI_angle"){
    aa = Vect<3>::Random();
    aa *= -M_PI / std::sqrt(aa.dot(aa));
  }
  SECTION("2PI_angle"){
    aa = Vect<3>::Random();
    aa *= 2*M_PI / std::sqrt(aa.dot(aa));
  }
  SECTION("minus_2PI_angle"){
    aa = Vect<3>::Random();
    aa *= -2*M_PI / std::sqrt(aa.dot(aa));
  }

  Mat<4,3> calc;
  quaternionFromAngleAxis(aa, calc);
  auto numeric = numericalDerivative11<Vect<4>, Vect<3>>(wrap(quaternionFromAngleAxis), aa);
  CHECK( calc.isApprox(numeric, 1e-10) );
}


