#include <catch/catch2.hpp>

#include <orient/from_rotation_matrix.hpp>

#include <Eigen/Geometry>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Rot3.h>

TEST_CASE("angleAxisFromRotationMatrix")
{
  Eigen::Vector3d aa;
  SECTION("Zero"){
    aa = Eigen::Vector3d::Zero();
  }
  SECTION("Random"){
    std::srand(1);
    aa = Eigen::Vector3d::Random();
  }
  SECTION("half_PI_angle"){
    std::srand(2);
    aa = Eigen::Vector3d::Random();
    aa *= M_PI / (2*aa.norm());
  }
  SECTION("minus_half_PI_angle"){
    std::srand(3);
    aa = Eigen::Vector3d::Random();
    aa *= -M_PI / (2*aa.norm());
  }
  SECTION("PI_angle"){
    std::srand(4);
    aa = Eigen::Vector3d::Random();
    aa *= M_PI / aa.norm();
  }
  SECTION("minus_PI_angle"){
    std::srand(5);
    aa = Eigen::Vector3d::Random();
    aa *= -M_PI / aa.norm();
  }
  SECTION("2PI_angle"){
    std::srand(6);
    aa = Eigen::Vector3d::Random();
    aa *= 2*M_PI / aa.norm();
  }
  SECTION("minus_2PI_angle"){
    std::srand(7);
    aa = Eigen::Vector3d::Random();
    aa *= -2*M_PI / aa.norm();
  }
  SECTION("special1"){
    aa << M_PI,0,0;
  }
  SECTION("special2"){
    aa << 0,M_PI,0;
  }
  SECTION("special3"){
    aa << 0,0,M_PI;
  }
  SECTION("special4"){
    const auto a = M_PI/std::sqrt(2);
    aa << a,a,0;
  }
  SECTION("special5"){
    const auto a = M_PI/std::sqrt(2);
    aa << a,0,a;
  }
  SECTION("special6"){
    const auto a = M_PI/std::sqrt(2);
    aa << 0,a,a;
  }

  const Eigen::Matrix3d R = gtsam::Rot3::Rodrigues(aa).matrix();
  const Eigen::Vector3d actual_aa = orient::angleAxisFromRotationMatrix(R);
  const Eigen::Matrix3d actual_R = gtsam::Rot3::Rodrigues(actual_aa).matrix();
  CHECK( actual_R.isApprox(R) );
}

TEST_CASE("quaternionFromRotationMatrix")
{
  Eigen::Vector3d aa;
  SECTION("Zero"){
    aa = Eigen::Vector3d::Zero();
  }
  SECTION("Random"){
    std::srand(0);
    aa = Eigen::Vector3d::Random();
  }
  SECTION("half_PI_angle"){
    aa = Eigen::Vector3d::Random();
    aa *= M_PI / (2 * aa.norm());
  }
  SECTION("minus_half_PI_angle"){
    aa = Eigen::Vector3d::Random();
    aa *= -M_PI / (2*aa.dot(aa));
  }
  SECTION("PI_angle"){
    aa = Eigen::Vector3d::Random();
    aa *= M_PI / aa.norm();
  }
  SECTION("minus_PI_angle"){
    aa = Eigen::Vector3d::Random();
    aa *= -M_PI / aa.norm();
  }
  SECTION("2PI_angle"){
    aa = Eigen::Vector3d::Random();
    aa *= 2*M_PI / aa.norm();
  }
  SECTION("minus_2PI_angle"){
    aa = Eigen::Vector3d::Random();
    aa *= -2*M_PI / aa.norm();
  }
  SECTION("special1"){
    aa << M_PI,0,0;
  }
  SECTION("special2"){
    aa << 0,M_PI,0;
  }
  SECTION("special3"){
    aa << 0,0,M_PI;
  }
  SECTION("special4"){
    const auto a = M_PI/std::sqrt(2);
    aa << a,a,0;
  }
  SECTION("special5"){
    const auto a = M_PI/std::sqrt(2);
    aa << a,0,a;
  }
  SECTION("special6"){
    const auto a = M_PI/std::sqrt(2);
    aa << 0,a,a;
  }

  Eigen::Matrix3d R = gtsam::Rot3::Rodrigues(aa).matrix();
  Eigen::Quaterniond equat{R};
  Eigen::Vector4d expected = (Eigen::Vector4d() << equat.w(), equat.vec()).finished();
  const Eigen::Vector4d actual = orient::quaternionFromRotationMatrix(R);
  CHECK( actual.isApprox( expected ) );
}

TEST_CASE("angleAxisFromRotationMatrix_derivative")
{
  std::srand(7);
  Eigen::Vector3d aa;
  SECTION("Random"){
    std::srand(0);
    aa = Eigen::Vector3d::Random();
  }
  SECTION("half_PI_angle"){
    aa = Eigen::Vector3d::Random();
    aa *= M_PI / (2 * aa.norm());
  }
  SECTION("minus_half_PI_angle"){
    aa = Eigen::Vector3d::Random();
    aa *= -M_PI / (2*aa.dot(aa));
  }
  Eigen::Matrix3d R = gtsam::Rot3::Rodrigues(aa).matrix();

  const auto [v, J] = orient::angleAxisFromRotationMatrixWD(R);
  auto num = gtsam::numericalDerivative11(orient::angleAxisFromRotationMatrix, R);
  CHECK( v.isApprox(orient::angleAxisFromRotationMatrix(R)) );
  CHECK( J.isApprox(num, 1e-8) );
}

TEST_CASE("quaternionFromRotationMatrix_derivative")
{
  std::srand(8);
  Eigen::Vector3d aa;
  SECTION("Zero"){
    aa = Eigen::Vector3d::Zero();
  }
  SECTION("Random"){
    std::srand(0);
    aa = Eigen::Vector3d::Random();
  }
  SECTION("half_PI_angle"){
    aa = Eigen::Vector3d::Random();
    aa *= M_PI / (2 * aa.norm());
  }
  SECTION("minus_half_PI_angle"){
    aa = Eigen::Vector3d::Random();
    aa *= -M_PI / (2*aa.dot(aa));
  }

  Eigen::Matrix3d R = gtsam::Rot3::Rodrigues(aa).matrix();

  const auto [v, J] = orient::quaternionFromRotationMatrixWD(R);
  auto num = gtsam::numericalDerivative11(orient::quaternionFromRotationMatrix, R);
  CHECK( v.isApprox(orient::quaternionFromRotationMatrix(R)) );
  CHECK( J.isApprox(num, 1e-10) );
}
