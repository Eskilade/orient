/* @copyright The code is licensed under the MIT License
 *            <https://opensource.org/licenses/MIT>,
 *            Copyright (c) 2020 Christian Eskil Vaugelade Berg
 * @author Christian Eskil Vaugelade Berg
*/
#include <orient/from_quaternion.hpp>

#include <orient/detail/skew_symmetric.hpp>
#include <orient/detail/kronecker_product.hpp>

namespace orient {

template<typename Scalar>
Eigen::Matrix<Scalar, 3, 3> rotationMatrixFromQuaternion(Eigen::Matrix<Scalar, 4, 1> const& q)
{
 const Eigen::Matrix<Scalar,3,3> skew = detail::skewSymmetric(q.template segment<3>(1));
  return Eigen::Matrix<Scalar,3,3>::Identity() + 2*q[0]*skew + 2 * skew * skew;
} 

template<typename Scalar>
std::pair<Eigen::Matrix<Scalar, 3, 3>, Eigen::Matrix<Scalar, 9, 4>> rotationMatrixFromQuaternionWD(Eigen::Matrix<Scalar, 4, 1> const& q)
{
  const auto w = q[0];
  const Eigen::Matrix<Scalar,3,3> I = Eigen::Matrix<Scalar,3,3>::Identity();

  const auto [skew, skewJv] = detail::skewSymmetricWPD(q.template segment<3>(1));
  const Eigen::Matrix<Scalar, 9, 1> RJw= 2*Eigen::Map<const Eigen::Matrix<Scalar, 9, 1>>{skew.data(), skew.size()};

  const Eigen::Matrix<Scalar,3,3> skew2 = skew*skew;
  const Eigen::Matrix<Scalar, 9, 3> skew2Jv = (detail::kroneckerProduct(I, skew) + Eigen::kroneckerProduct(skew.transpose(), I)) * skewJv;

  const Eigen::Matrix<Scalar, 9, 3> RJv= 2.*w*skewJv + 2.*skew2Jv;
  Eigen::Matrix<Scalar, 9, 4> J;
  J.template block<9,1>(0,0) = RJw;
  J.template block<9,3>(0,1) = RJv;
  return std::make_pair(Eigen::Matrix<Scalar,3,3>::Identity() + 2.*w*skew + 2.*skew2, J);
} 

template<typename Scalar>
Eigen::Matrix<Scalar, 3, 1> angleAxisFromQuaternion(Eigen::Matrix<Scalar, 4, 1> const& q)
{
  const auto w = q[0];
  const Eigen::Vector3d v = q.template segment<3>(1);
  if(1. - w < 1e-10){
    return v * 6. /(w + 2.);
  }
  const auto y = std::sqrt(v.dot(v));
  return 2 * v * std::atan2(y, w) / y;
}

template<typename Scalar>
std::pair<Eigen::Matrix<Scalar, 3, 1>, Eigen::Matrix<Scalar, 3, 4>> angleAxisFromQuaternionWD(Eigen::Matrix<Scalar, 4, 1> const& q)
{
  const auto w = q[0];
  const Eigen::Vector3d v = q.template segment<3>(1);
  if(1. - w < 1e-10){
    const auto k = 6. /(w + 2.);
    const auto kJw = -6./((w+2.)*(w+2.));
    const Eigen::Vector3d aaJw = kJw*v;
    const Eigen::Matrix<Scalar,3,3> aaJv = k*Eigen::Matrix<Scalar,3,3>::Identity();

    Eigen::Matrix<Scalar, 3, 4> J;
    J.template block<3,1>(0,0) = aaJw;
    J.template block<3,3>(0,1) = aaJv;
    return std::make_pair(k*v, J);
  }

  const auto y2 = v.dot(v);
  const auto y = std::sqrt(y2);

  const auto arg = std::atan2(y, w);
  const auto k = 2. * arg / y;
  const auto kJw = -2.;
  const auto kJy = 2.*(w*y - arg)/y2;

  const Eigen::Matrix<Scalar, 1, 3> yJv = v.transpose() / y;

  Eigen::Matrix<Scalar, 3, 4> J;
  J.template block<3,1>(0,0) = kJw * v;
  J.template block<3,3>(0,1) = v * kJy * yJv + k*Eigen::Matrix<Scalar,3,3>::Identity();

  return std::make_pair(k*v, J);
}

}
