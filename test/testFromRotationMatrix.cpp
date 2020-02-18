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

  Eigen::Matrix3d R = gtsam::Rot3::Rodrigues(aa).matrix();
  Eigen::AngleAxisd eaa{R};
  Eigen::Vector3d expected = eaa.angle() * eaa.axis();
  const Eigen::Vector3d actual = angleAxisFromRotationMatrix(R);
  std::cout << expected.transpose() << "\n";
  std::cout << actual.transpose() << "\n";
}

TEST_CASE("quaternionFromRotationMatrix")
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

  Eigen::Matrix3d R = gtsam::Rot3::Rodrigues(aa).matrix();
  Eigen::Quaterniond equat{R};
  Eigen::Vector4d expected = (Eigen::Vector4d() << equat.w(), equat.vec()).finished();
  const Eigen::Vector4d actual = quaternionFromRotationMatrix(R);
  std::cout << expected.transpose() << "\n";
  std::cout << actual.transpose() << "\n";
  std::cout << "-----\n";
}

TEST_CASE("angleAxisFromRotationMatrix_derivative")
{
  auto wrap = [](auto&&... args){
    return angleAxisFromRotationMatrix(args...);
  };
  const Eigen::Matrix3d R = Eigen::Matrix3d::Random();
  Eigen::Matrix<double, 3, 9> H;
  angleAxisFromRotationMatrix(R, H);
  auto num = gtsam::numericalDerivative11<Eigen::Vector3d, Eigen::Matrix3d>(wrap, R);
  CHECK( H.isApprox(num, 1e-10) );
}

TEST_CASE("quaternionFromRotationMatrix_derivative")
{
  auto wrap = [](auto&&... args){
    return quaternionFromRotationMatrix(args...);
  };
  const Eigen::Matrix3d R = Eigen::Matrix3d::Random();
  Eigen::Matrix<double, 4, 9> H;
  quaternionFromRotationMatrix(R, H);
  auto num = gtsam::numericalDerivative11<Eigen::Vector4d, Eigen::Matrix3d>(wrap, R);
  CHECK( H.isApprox(num, 1e-10) );
}
