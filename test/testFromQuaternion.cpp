#define CATCH_CONFIG_MAIN

#include <catch.hpp>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Rot3.h>
#include <fromQuaternion.hpp>
#include <iostream>

#include <Eigen/Geometry>

using namespace gtsam;
#define Wrap(FNAME, RetT, ...) \
  boost::function<RetT (__VA_ARGS__ )>{[](auto&& ... args){\
    return FNAME(std::forward<decltype(args)>(args) ... );\
  }}

TEST_CASE("rotationMatrixFromQuaternion"){
  Eigen::Quaterniond rand = Eigen::Quaterniond::UnitRandom();
  Eigen::Vector4d q;
  q << rand.w(), rand.x(), rand.y(), rand.z();
  Mat<3,3> expected = rand.toRotationMatrix();
  Mat<3,3> actual = rotationMatrixFromQuaternion(q);
  CHECK( actual.isApprox( expected ) );
}

TEST_CASE("rotationMatrixFromQuaternion_derivative")
{
  auto wrap = boost::function<Mat<3,3>(Eigen::Vector4d const&)>{[](auto&& ... args){
    return rotationMatrixFromQuaternion(std::forward<decltype(args)>(args) ... );
  }};

  Eigen::Quaterniond rand = Eigen::Quaterniond::UnitRandom();
  Eigen::Vector4d q = (Eigen::Vector4d() << rand.w(), rand.vec()).finished();
  auto num = numericalDerivative11(wrap, q);
  Mat<9, 4> calc;
  rotationMatrixFromQuaternion(q, calc);
  CHECK( calc.isApprox( num, 1e-10) );
}

TEST_CASE("normalize")
{
  Eigen::Vector4d unq = Eigen::Vector4d::Random();
  Eigen::Vector4d q = normalize(unq);
  double norm = q.dot(q);
  CHECK( std::abs( norm - 1.0) < std::numeric_limits<double>::epsilon() );
}

TEST_CASE("normalize_derivative")
{
  auto wrap = boost::function<Eigen::Vector4d(Eigen::Vector4d const&)>{[](auto&& ... args){
    return normalize(std::forward<decltype(args)>(args) ... );
  }};

  Eigen::Vector4d unq = Eigen::Vector4d::Random();
  auto num = numericalDerivative11(wrap, unq);
  Mat<4, 4> calc;
  normalize<4>(unq, calc);
  CHECK( calc.isApprox( num, 1e-9) );
}

TEST_CASE("angleAxisFromQuaternion")
{
  Eigen::Quaterniond rand = Eigen::Quaterniond::UnitRandom();
  Eigen::Vector4d q = (Eigen::Vector4d() << rand.w(), rand.vec()).finished();
  Eigen::AngleAxisd eaa{rand}; 
  Eigen::Vector3d expected = eaa.angle() * eaa.axis();
  Eigen::Vector3d actual = angleAxisFromQuaternion(q);

  Mat<3,3> eR = gtsam::Rot3::Rodrigues(expected).matrix();
  Mat<3,3> aR = gtsam::Rot3::Rodrigues(actual).matrix();
  CHECK( aR.isApprox( eR) );
}

TEST_CASE("angleAxisFromQuaternion_derivative")
{
  auto wrap = boost::function<Eigen::Vector3d(Eigen::Vector4d const&)>{[](auto&& ... args){
    return angleAxisFromQuaternion(std::forward<decltype(args)>(args) ... );
  }};

  Eigen::Vector4d unq = Eigen::Vector4d::Random();
  auto num = numericalDerivative11(wrap, unq);
  Mat<3, 4> calc;
  angleAxisFromQuaternion(unq, calc);
  CHECK( calc.isApprox( num, 1e-9) );
}


