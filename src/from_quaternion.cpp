#include <orient/from_quaternion.hpp>

#include <orient/detail/skew_symmetric.hpp>
#include <orient/detail/kronecker_product.hpp>

namespace orient {

Eigen::Matrix3d rotationMatrixFromQuaternion(Eigen::Vector4d const& q)
{
  const Eigen::Matrix3d skew = detail::skewSymmetric(q.segment<3>(1));
  return Eigen::Matrix3d::Identity() + 2*q[0]*skew + 2 * skew * skew;
} 
 
std::pair<Eigen::Matrix3d, Eigen::Matrix<double, 9, 4>> rotationMatrixFromQuaternionWD(Eigen::Vector4d const& q)
{
  const auto w = q[0];
  const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();

  const auto [skew, skewJv] = detail::skewSymmetricWPD(q.segment<3>(1));
  const Eigen::Matrix<double, 9, 1> RJw= 2*Eigen::Map<const Eigen::Matrix<double, 9, 1>>{skew.data(), skew.size()};

  const Eigen::Matrix3d skew2 = skew*skew;
  const Eigen::Matrix<double, 9, 3>
    skew2Jv = (detail::kroneckerProduct(I, skew) + Eigen::kroneckerProduct(skew.transpose(), I)) * skewJv;

  const Eigen::Matrix<double, 9, 3> RJv= 2.*w*skewJv + 2.*skew2Jv;
  Eigen::Matrix<double, 9, 4> J;
  J.block<9,1>(0,0) = RJw;
  J.block<9,3>(0,1) = RJv;
  return std::make_pair(Eigen::Matrix3d::Identity() + 2.*w*skew + 2.*skew2, J);
} 

Eigen::Vector3d angleAxisFromQuaternion(Eigen::Vector4d const& q)
{
  const auto w = q[0];
  const Eigen::Vector3d v = q.segment<3>(1);
  if(1. - w < 1e-10){
    return v * 6. /(w + 2.);
  }
  const auto y = std::sqrt(v.dot(v));
  return 2 * v * std::atan2(y, w) / y;
}

std::pair<Eigen::Vector3d, Eigen::Matrix<double, 3, 4>> angleAxisFromQuaternionWD(Eigen::Vector4d const& q)
{
  const auto w = q[0];
  const Eigen::Vector3d v = q.segment<3>(1);
  if(1. - w < 1e-10){
    const auto k = 6. /(w + 2.);
    const auto kJw = -6./((w+2.)*(w+2.));
    const Eigen::Vector3d aaJw = kJw*v;
    const Eigen::Matrix3d aaJv = k*Eigen::Matrix3d::Identity();

    Eigen::Matrix<double, 3, 4> J;
    J.block<3,1>(0,0) = aaJw;
    J.block<3,3>(0,1) = aaJv;
    return std::make_pair(k*v, J);
  }

  const auto y2 = v.dot(v);
  const auto y = std::sqrt(y2);

  const auto arg = std::atan2(y, w);
  const auto k = 2. * arg / y;
  const auto kJw = -2.;
  const auto kJy = 2.*(w*y - arg)/y2;

  const Eigen::Matrix<double, 1, 3> yJv = v.transpose() / y;

  Eigen::Matrix<double, 3, 4> J;
  J.block<3,1>(0,0) = kJw * v;
  J.block<3,3>(0,1) = v * kJy * yJv + k*Eigen::Matrix3d::Identity();

  return std::make_pair(k*v, J);
}

}
