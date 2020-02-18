#include <fromRotationMatrix.hpp>

#define CATCH_CONFIG_MAIN
#include <catch.hpp>

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Rot3.h>
#include <Eigen/Geometry>
#include <iostream>
#include <common.hpp>

using namespace gtsam;
#define Wrap(FNAME, RetT, ...) \
  boost::function<RetT (__VA_ARGS__ )>{[](auto&& ... args){\
    return FNAME(std::forward<decltype(args)>(args) ... );\
  }}

#define wrap(name) [](auto&&... args){ return name(std::forward<decltype(args)>(args) ...);}

TEST_CASE("angleAxisFromRotationMatrix")
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

  Mat<3,3> R = gtsam::Rot3::Rodrigues(aa).matrix();
  Eigen::AngleAxisd eaa{R};
  Vect<3> expected = eaa.angle() * eaa.axis();
  const Vect<3> actual = angleAxisFromRotationMatrix(R);
  std::cout << expected.transpose() << "\n";
  std::cout << actual.transpose() << "\n";
}

TEST_CASE("quaternionFromRotationMatrix")
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

  Mat<3,3> R = gtsam::Rot3::Rodrigues(aa).matrix();
  Eigen::Quaterniond equat{R};
  Vect<4> expected = (Vect<4>() << equat.w(), equat.vec()).finished();
  const Vect<4> actual = quaternionFromRotationMatrix(R);
  std::cout << expected.transpose() << "\n";
  std::cout << actual.transpose() << "\n";
  std::cout << "-----\n";
}
