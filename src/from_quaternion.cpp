#include <orient/from_quaternion.hpp>

#include <orient/detail/skew_symmetric.hpp>
#include <orient/detail/kronecker_product.hpp>

namespace orient {

Eigen::Matrix3d rotationMatrixFromQuaternion(Eigen::Vector4d const& q)
{
  const Eigen::Matrix3d skew = detail::skewSymmetric(q.segment<3>(1));
  return Eigen::Matrix3d::Identity() + 2*q[0]*skew + 2 * skew * skew;
} 
 
Eigen::Matrix3d rotationMatrixFromQuaternion(Eigen::Vector4d const& q, Eigen::Ref<Eigen::Matrix<double, 9, 4>> H)
{
  const auto w = q[0];
  const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();

  const auto [skew, skewHv] = detail::skewSymmetricWPD(q.segment<3>(1));
  const Eigen::Matrix<double, 9, 1> RHw= 2*Eigen::Map<const Eigen::Matrix<double, 9, 1>>{skew.data(), skew.size()};

  const Eigen::Matrix3d skew2 = skew*skew;
  const Eigen::Matrix<double, 9, 3>
    skew2Hv = (detail::kroneckerProduct(I, skew) + Eigen::kroneckerProduct(skew.transpose(), I)) * skewHv;

  const Eigen::Matrix<double, 9, 3> RHv= 2.*w*skewHv + 2.*skew2Hv;

  H.block<9,1>(0,0) = RHw;
  H.block<9,3>(0,1) = RHv;
  return Eigen::Matrix3d::Identity() + 2.*w*skew + 2.*skew2;
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

Eigen::Vector3d angleAxisFromQuaternion(Eigen::Vector4d const& q, Eigen::Ref<Eigen::Matrix<double, 3, 4>> H )
{
  const auto w = q[0];
  const Eigen::Vector3d v = q.segment<3>(1);
  if(1. - w < 1e-10){
    const auto k = 6. /(w + 2.);
    const auto kHw = -6./((w+2.)*(w+2.));
    const Eigen::Vector3d aaHw = kHw*v;
    const Eigen::Matrix3d aaHv = k*Eigen::Matrix3d::Identity();
    H.block<3,1>(0,0) = aaHw;
    H.block<3,3>(0,1) = aaHv;
    return k*v;
  }

  const auto y2 = v.dot(v);
  const auto y = std::sqrt(y2);

  const auto arg = std::atan2(y, w);
  const auto k = 2. * arg / y;
  const auto kHw = -2.;
  const auto kHy = 2.*(w*y - arg)/y2;

  const Eigen::Matrix<double, 1, 3> yHv = v.transpose() / y;

  H.block<3,1>(0,0) = kHw * v;
  H.block<3,3>(0,1) = v * kHy * yHv + k*Eigen::Matrix3d::Identity();
  return k*v;
}

}
