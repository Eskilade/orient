#include <catch/catch2.hpp>

#include <from_rotation_matrix.hpp>

#include <Eigen/Geometry>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Rot3.h>

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
  const Eigen::Matrix3d R = Eigen::Matrix3d::Random();
  Eigen::Matrix<double, 3, 9> H;
  angleAxisFromRotationMatrix(R, H);
  auto num = gtsam::numericalDerivative11(angleAxisFromRotationMatrix, R);
  CHECK( H.isApprox(num, 1e-10) );
}

TEST_CASE("quaternionFromRotationMatrix_derivative")
{
  const Eigen::Matrix3d R = Eigen::Matrix3d::Random();
  Eigen::Matrix<double, 4, 9> H;
  quaternionFromRotationMatrix(R, H);
  auto num = gtsam::numericalDerivative11(quaternionFromRotationMatrix, R);
  CHECK( H.isApprox(num, 1e-10) );
}
