#define CATCH_CONFIG_MAIN
#include <catch.hpp>

#include <Eigen/Geometry>

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Rot3.h>

#include <fromAngleAxis.hpp>

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

  Eigen::Matrix<double, 9, 3> calc;
  rotationMatrixFromAngleAxis(aa, calc);
  auto numeric = numericalDerivative11<Eigen::Matrix3d, Eigen::Vector3d>(wrap(rotationMatrixFromAngleAxis), aa);
  CHECK( calc.isApprox(numeric, 1e-10) );
}

TEST_CASE("quaternionFromAngleAxis")
{
  Eigen::Vector3d aa;
  SECTION("Random"){
    aa = Eigen::Vector3d::Random();
  }
  SECTION("half_PI_angle"){
    aa = Eigen::Vector3d::Random();
    aa *= M_PI / (2*std::sqrt(aa.dot(aa)));
  }
  SECTION("minus_half_PI_angle"){
    aa = Eigen::Vector3d::Random();
    aa *= -M_PI / (2*std::sqrt(aa.dot(aa)));
  }
  SECTION("PI_angle"){
    aa = Eigen::Vector3d::Random();
    aa *= M_PI / std::sqrt(aa.dot(aa));
  }
  SECTION("minus_PI_angle"){
    aa = Eigen::Vector3d::Random();
    aa *= -M_PI / std::sqrt(aa.dot(aa));
  }
  SECTION("2PI_angle"){
    aa = Eigen::Vector3d::Random();
    aa *= 2*M_PI / std::sqrt(aa.dot(aa));
  }
  SECTION("minus_2PI_angle"){
    aa = Eigen::Vector3d::Random();
    aa *= -2*M_PI / std::sqrt(aa.dot(aa));
  }



  const auto angle = std::sqrt(aa.dot(aa));
  const Eigen::Vector3d axis = aa / angle;
  Eigen::AngleAxisd eaa{angle, axis};
  Eigen::Quaterniond equat{eaa};

  Eigen::Vector4d expected = (Eigen::Vector4d() << equat.w(), equat.vec()).finished();
  Eigen::Vector4d actual = quaternionFromAngleAxis(aa);
  CHECK( expected.isApprox(actual) );
}
TEST_CASE("quaternionFromAngleAxis_derivative")
{
  Eigen::Vector3d aa;
  SECTION("Random"){
    aa = Eigen::Vector3d::Random();
  }
  SECTION("half_PI_angle"){
    aa = Eigen::Vector3d::Random();
    aa *= M_PI / (2*std::sqrt(aa.dot(aa)));
  }
  SECTION("minus_half_PI_angle"){
    aa = Eigen::Vector3d::Random();
    aa *= -M_PI / (2*std::sqrt(aa.dot(aa)));
  }
  SECTION("PI_angle"){
    aa = Eigen::Vector3d::Random();
    aa *= M_PI / std::sqrt(aa.dot(aa));
  }
  SECTION("minus_PI_angle"){
    aa = Eigen::Vector3d::Random();
    aa *= -M_PI / std::sqrt(aa.dot(aa));
  }
  SECTION("2PI_angle"){
    aa = Eigen::Vector3d::Random();
    aa *= 2*M_PI / std::sqrt(aa.dot(aa));
  }
  SECTION("minus_2PI_angle"){
    aa = Eigen::Vector3d::Random();
    aa *= -2*M_PI / std::sqrt(aa.dot(aa));
  }

  Eigen::Matrix<double, 4, 3> calc;
  quaternionFromAngleAxis(aa, calc);
  auto numeric = numericalDerivative11<Eigen::Vector4d, Eigen::Vector3d>(wrap(quaternionFromAngleAxis), aa);
  CHECK( calc.isApprox(numeric, 1e-10) );
}


