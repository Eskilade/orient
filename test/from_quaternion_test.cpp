#include <catch/catch2.hpp>

#include <from_quaternion.hpp>

#include <Eigen/Geometry>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Rot3.h>

TEST_CASE("rotationMatrixFromQuaternion"){
  Eigen::Quaterniond rand = Eigen::Quaterniond::UnitRandom();
  Eigen::Vector4d q;
  q << rand.w(), rand.x(), rand.y(), rand.z();
  Eigen::Matrix3d expected = rand.toRotationMatrix();
  Eigen::Matrix3d actual = rotationMatrixFromQuaternion(q);
  CHECK( actual.isApprox( expected ) );
}

TEST_CASE("rotationMatrixFromQuaternion_derivative")
{
  Eigen::Quaterniond rand = Eigen::Quaterniond::UnitRandom();
  Eigen::Vector4d q = (Eigen::Vector4d() << rand.w(), rand.vec()).finished();
  auto num = gtsam::numericalDerivative11(rotationMatrixFromQuaternion, q);
  Eigen::Matrix<double, 9, 4> calc;
  rotationMatrixFromQuaternion(q, calc);
  CHECK( calc.isApprox( num, 1e-9) );
}

TEST_CASE("angleAxisFromQuaternion")
{
  Eigen::Quaterniond rand = Eigen::Quaterniond::UnitRandom();
  Eigen::Vector4d q = (Eigen::Vector4d() << rand.w(), rand.vec()).finished();
  Eigen::AngleAxisd eaa{rand}; 
  Eigen::Vector3d expected = eaa.angle() * eaa.axis();
  Eigen::Vector3d actual = angleAxisFromQuaternion(q);

  Eigen::Matrix3d eR = gtsam::Rot3::Rodrigues(expected).matrix();
  Eigen::Matrix3d aR = gtsam::Rot3::Rodrigues(actual).matrix();
  CHECK( aR.isApprox( eR) );
}

TEST_CASE("angleAxisFromQuaternion_derivative")
{
  Eigen::Vector4d unq = Eigen::Vector4d::Random();
  auto num = gtsam::numericalDerivative11(angleAxisFromQuaternion, unq);
  Eigen::Matrix<double, 3, 4> calc;
  angleAxisFromQuaternion(unq, calc);
  CHECK( calc.isApprox( num, 1e-9) );
}
