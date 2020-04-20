#include <catch/catch2.hpp>

#include <orient/from_quaternion.hpp>

#include <Eigen/Geometry>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Rot3.h>

TEST_CASE("rotationMatrixFromQuaternion"){
  Eigen::Quaterniond eq;
  SECTION("zero_angle"){
    Eigen::Vector3d u = Eigen::Vector3d::Random();
    u *= u.squaredNorm();
    eq = Eigen::AngleAxisd{0.f, u};
  }
  SECTION("almost_zero_angle"){
    Eigen::Vector3d u = Eigen::Vector3d::Random();
    u *= u.squaredNorm();
    eq = Eigen::AngleAxisd{1e-10, u};
  }
  SECTION("random"){
    eq = Eigen::Quaterniond::UnitRandom();
  }
  Eigen::Vector4d q;
  q << eq.w(), eq.x(), eq.y(), eq.z();
  Eigen::Matrix3d expected = eq.toRotationMatrix();
  Eigen::Matrix3d actual = orient::rotationMatrixFromQuaternion(q);
  CHECK( actual.isApprox( expected ) );
}

TEST_CASE("rotationMatrixFromQuaternion_derivative")
{
  Eigen::Quaterniond eq;
  SECTION("zero_angle"){
    Eigen::Vector3d u = Eigen::Vector3d::Random();
    u *= u.squaredNorm();
    eq = Eigen::AngleAxisd{0.f, u};
  }
  SECTION("almost_zero_angle"){
    Eigen::Vector3d u = Eigen::Vector3d::Random();
    u *= u.squaredNorm();
    eq = Eigen::AngleAxisd{1e-10, u};
  }
  SECTION("random"){
    eq = Eigen::Quaterniond::UnitRandom();
  }

  Eigen::Vector4d q;
  q << eq.w(), eq.x(), eq.y(), eq.z();

  auto num = gtsam::numericalDerivative11(orient::rotationMatrixFromQuaternion, q);
  Eigen::Matrix<double, 9, 4> calc;
  orient::rotationMatrixFromQuaternion(q, calc);
  CHECK( calc.isApprox( num, 1e-9) );
}

TEST_CASE("angleAxisFromQuaternion")
{
  Eigen::Quaterniond eq;
  SECTION("zero_angle"){
    Eigen::Vector3d u = Eigen::Vector3d::Random();
    u *= u.squaredNorm();
    eq = Eigen::AngleAxisd{0.f, u};
  }
  SECTION("almost_zero_angle"){
    Eigen::Vector3d u = Eigen::Vector3d::Random();
    u *= u.squaredNorm();
    eq = Eigen::AngleAxisd{1e-11, u};
  }
  SECTION("almost_zero_angle"){
    Eigen::Vector3d u = Eigen::Vector3d::Random();
    u *= u.squaredNorm();
    eq = Eigen::AngleAxisd{M_PI/4, u};
  }
  SECTION("random"){
    eq = Eigen::Quaterniond::UnitRandom();
  }
  Eigen::Vector4d q;
  q << eq.w(), eq.x(), eq.y(), eq.z();

  Eigen::AngleAxisd eaa{eq}; 
  Eigen::Vector3d expected = eaa.angle() * eaa.axis();
  Eigen::Vector3d actual = orient::angleAxisFromQuaternion(q);

  Eigen::Matrix3d eR = gtsam::Rot3::Rodrigues(expected).matrix();
  Eigen::Matrix3d aR = gtsam::Rot3::Rodrigues(actual).matrix();
  CHECK( aR.isApprox(eR, 1e-9) );
}

TEST_CASE("angleAxisFromQuaternion_derivative")
{
  Eigen::Quaterniond eq;
  SECTION("almost_zero_angle"){
    Eigen::Vector3d u = Eigen::Vector3d::Random();
    u *= u.squaredNorm();
    eq = Eigen::AngleAxisd{1e-10, u};
  }
  SECTION("random"){
    eq = Eigen::Quaterniond::UnitRandom();
  }
  Eigen::Vector4d q;
  q << eq.w(), eq.x(), eq.y(), eq.z();

  auto num = gtsam::numericalDerivative11(orient::angleAxisFromQuaternion, q);
  Eigen::Matrix<double, 3, 4> calc;
  orient::angleAxisFromQuaternion(q, calc);
  CHECK( calc.isApprox( num, 1e-9) );
}
