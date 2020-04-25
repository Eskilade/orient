#include <orient/from_rotation_matrix.hpp>

#include <orient/detail/skew_symmetric.hpp>
#include <orient/detail/so3_generator.hpp>
#include <orient/detail/derivative_helpers.hpp>
#include <orient/detail/kronecker_product.hpp>
#include <orient/detail/is_almost_eq.hpp>

namespace orient {

Eigen::Vector3d angleAxisFromRotationMatrix(Eigen::Matrix3d const& R)
{
  // trace = 2*cos(angle) + 1
  const auto trace = R.trace();
  if( detail::isAlmostEq(trace, 3.0) ){
    // angle is close to 2*k*pi
    // Should be minus I but the diagonal does not change anything
    return orient::detail::unskewSymmetric(R);
  } else if( detail::isAlmostEq(trace, -1.0) ){
    // Derivative is possibly not defined here. There are 2 solutions, as
    // Theta and minus Theta would produce the same result ?
    // soluation possible
    // angle is close to (2*k + 1)*pi
    const Eigen::Matrix3d uut = (R + Eigen::Matrix3d::Identity()) / 2;
    int max_i;
    const auto ui = std::sqrt(uut.diagonal().maxCoeff(&max_i));
    return M_PI * uut.block<3,1>(0, max_i) / ui;
  } else {
    const auto arg = (trace - 1)/2; 
    const auto angle = std::acos(arg);
    return detail::unskewSymmetric(R - R.transpose()) * angle / (2*std::sin(angle));
  }
}

std::pair<Eigen::Vector3d, Eigen::Matrix<double, 3, 9>> angleAxisFromRotationMatrixWD(Eigen::Matrix3d const& R)
{
  const auto [tr, trHR] = detail::traceWD(R);
  if( detail::isAlmostEq(tr, 3.0) ){
    const auto [ev, evHR] = detail::unskewSymmetricWPD(R);
    return std::make_pair(ev, evHR);
  }
  const auto x = 0.5*(tr - 1.0);

  const auto [angle, angleHx] = detail::acosWD(x);

  const auto sin = std::sin(angle);

  const auto k3 = 0.5*angle/sin;
  const auto k3Hangle = 0.5*(1. - angle/std::tan(angle))/sin;

  const auto [Rt, RtHR] = detail::transposeWD(R);
  const Eigen::Matrix3d eR = R - Rt;
  const Eigen::Matrix<double, 9, 9> eRHR = Eigen::Matrix<double, 9, 9>::Identity() - RtHR;

  const auto [ev, evHeR] = detail::unskewSymmetricWPD(eR);

  const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
  Eigen::Map<const Eigen::Matrix<double, 9, 1>> vecI{I.data(), I.size()};

  const Eigen::Matrix<double, 3, 9> H = detail::kroneckerProduct(ev.transpose(), I) * vecI *k3Hangle * angleHx * trHR / 2.
    + k3 * evHeR * eRHR;
   return std::make_pair(k3 * ev, H);
}

Eigen::Vector4d quaternionFromRotationMatrix(Eigen::Matrix3d const& R)
{
  const auto tr = R.trace();
  if( detail::isAlmostEq(tr, -1.0) ){
    // w is close to 0
    const Eigen::Matrix3d vvt = (R + Eigen::Matrix3d::Identity()) / 2;
    int max_i;
    const auto vi = std::sqrt(vvt.diagonal().maxCoeff(&max_i));
    return (Eigen::Vector4d() << 0, vvt.block<3,1>(0, max_i) / vi).finished();
  }
  const auto w = 0.5*std::sqrt(tr + 1.0);
  const Eigen::Vector3d v = 0.25*detail::unskewSymmetric(R - R.transpose()) / w;
  return (Eigen::Vector4d() << w, v).finished();
}

std::pair<Eigen::Vector4d, Eigen::Matrix<double, 4, 9>> quaternionFromRotationMatrixWD(Eigen::Matrix3d const& R)
{
  const auto [tr, trHR] = detail::traceWD(R);

  const auto w = 0.5*std::sqrt(tr + 1.0);
  Eigen::Matrix<double, 1 , 9> wHR = 0.25/std::sqrt(tr+1) * trHR;

  const auto k4 = 0.25/w;
  const auto k4Hw = - k4/w;
 
  const auto [Rt, RtHR] = detail::transposeWD(R);
  const Eigen::Matrix3d eR = R - Rt;
  const Eigen::Matrix<double, 9, 9> eRHR = Eigen::Matrix<double, 9, 9>::Identity() - RtHR;

  const auto [ev, evHeR] = detail::unskewSymmetricWPD(eR);

  const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
  Eigen::Map<const Eigen::Matrix<double, 9, 1>> vecI{I.data(), I.size()};

  const Eigen::Vector3d v = 0.25*detail::unskewSymmetric(R - R.transpose()) / w;
  Eigen::Matrix<double, 3 , 9> vHR = detail::kroneckerProduct(ev.transpose(), I) * vecI *k4Hw * wHR 
    + k4 * evHeR * eRHR;

  Eigen::Matrix<double, 4, 9> H;
  H.block<1,9>(0,0) = wHR;
  H.block<3,9>(1,0) = vHR;
  return std::make_pair((Eigen::Vector4d() << w, v).finished(), H);
}

}
