#include <catch/catch2.hpp>

#include <orient/from_angle_axis.hpp>

#include <Eigen/Geometry>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Rot3.h>

TEST_CASE("rotationMatrixFromAngleAxis")
{ 
  Eigen::Vector3d aa;
  SECTION("Zero"){
    aa = Eigen::Vector3d::Zero();
  }
  SECTION("almost_zero"){
    aa = 1e-15 * Eigen::Vector3d::Random();
  }
  SECTION("Random"){
    aa = Eigen::Vector3d::Random();
  }

  const auto calc = orient::rotationMatrixFromAngleAxis(aa);
  const auto act = gtsam::Rot3::Expmap(aa).matrix();
  CHECK( calc.isApprox(act) );
}

TEST_CASE("rotationMatrixFromAngleAxis_derivative")
{ 
  Eigen::Vector3d aa;
  SECTION("Zero"){
    aa = Eigen::Vector3d::Zero();
  }
  SECTION("almost_zero"){
    aa = 1e-15 * Eigen::Vector3d::Random();
  }
  SECTION("Random"){
    aa = Eigen::Vector3d::Random();
  }

  Eigen::Matrix<double, 9, 3> calc;
  orient::rotationMatrixFromAngleAxis(aa, calc);
  auto numeric = gtsam::numericalDerivative11(orient::rotationMatrixFromAngleAxis, aa);
  CHECK( calc.isApprox(numeric, 1e-10) );
}

TEST_CASE("quaternionFromAngleAxis")
{
  Eigen::Vector3d aa;
  SECTION("zero"){
    aa = Eigen::Vector3d::Zero();
  }
  SECTION("almost_zero"){
    aa = Eigen::Vector3d::Random();
    aa *= 1e-11 / aa.squaredNorm();
  }
  SECTION("random"){
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
  Eigen::Vector3d axis;
  // use a random axis when angle is really small
  if (angle > 1e-50)
    axis = aa / angle;
  else
    axis << 1.,0.,0.;
  Eigen::AngleAxisd eaa{angle, axis};
  Eigen::Quaterniond equat{eaa};

  Eigen::Vector4d expected = (Eigen::Vector4d() << equat.w(), equat.vec()).finished();
  Eigen::Vector4d actual = orient::quaternionFromAngleAxis(aa);
  CHECK( expected.isApprox(actual) );
}

TEST_CASE("quaternionFromAngleAxis_derivative")
{
  Eigen::Vector3d aa;
  SECTION("zero"){
    aa = Eigen::Vector3d::Zero();
  }
  SECTION("almost_zero"){
    aa = Eigen::Vector3d::Random();
    aa *= 1e-11 / aa.squaredNorm();
  }
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
  orient::quaternionFromAngleAxis(aa, calc);
  auto numeric = gtsam::numericalDerivative11(orient::quaternionFromAngleAxis, aa);
  CHECK( calc.isApprox(numeric, 1e-10) );
  if( not calc.isApprox(numeric, 1e-10) ){
    std::cout << numeric << "\n\n";
    std::cout << calc << "\n\n";
    std::cout << numeric - calc << "\n\n";
  }
}
