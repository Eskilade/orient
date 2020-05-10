/* @copyright The code is licensed under the MIT License
 *            <https://opensource.org/licenses/MIT>,
 *            Copyright (c) 2020 Christian Eskil Vaugelade Berg
 * @author Christian Eskil Vaugelade Berg
*/
#include <orient/from_rotation_matrix.hpp>

#include <orient/detail/skew_symmetric.hpp>
#include <orient/detail/so3_generator.hpp>
#include <orient/detail/derivative_helpers.hpp>
#include <orient/detail/kronecker_product.hpp>
#include <orient/detail/is_almost_eq.hpp>

namespace orient {

template<typename Scalar>
Eigen::Matrix<Scalar,3,1> angleAxisFromRotationMatrix(Eigen::Matrix<Scalar,3,3> const& R)
{
  const auto tr = R.trace();
  if( detail::isAlmostEq(tr, 3.0) ){
    // angle is close to 2*k*pi
    // Should be minus I but the diagonal does not change anything
    return orient::detail::unskewSymmetric(R);
  } else if( detail::isAlmostEq(tr, -1.0) ){
    const Eigen::Matrix<Scalar,3,3> uut = (R + Eigen::Matrix<Scalar,3,3>::Identity()) / 2;
    int max_i;
    const auto ui = std::sqrt(uut.diagonal().maxCoeff(&max_i));
    return M_PI * uut.template block<3,1>(0, max_i) / ui;
  }
  const auto arg = (tr - 1)/2; 
  const auto angle = std::acos(arg);
  return detail::unskewSymmetric(R - R.transpose()) * angle / (2*std::sin(angle));
}

template<typename Scalar>
std::pair<Eigen::Matrix<Scalar,3,1>, Eigen::Matrix<Scalar, 3, 9>> angleAxisFromRotationMatrixWD(Eigen::Matrix<Scalar,3,3> const& R)
{
  const auto [tr, trJR] = detail::traceWD(R);
  if( detail::isAlmostEq(tr, 3.0) ){
    const auto [ev, evJR] = detail::unskewSymmetricWPD(R);
    return std::make_pair(ev, evJR);
  } else if( detail::isAlmostEq(tr, -1.0) ){
    // Derivative is possibly not defined here. There are 2 solutions, as
    // Theta and minus Theta would produce the same result ?
    const Eigen::Matrix<Scalar,3,3> uut = (R + Eigen::Matrix<Scalar,3,3>::Identity()) / 2;
    int max_i;
    const auto ui = std::sqrt(uut.diagonal().maxCoeff(&max_i));
    return std::make_pair(M_PI * uut.template block<3,1>(0, max_i) / ui, Eigen::Matrix<Scalar, 3, 9>::Constant(std::nan("")));
  }

  const auto x = 0.5*(tr - 1.0);

  const auto [angle, angleJx] = detail::acosWD(x);

  const auto sin = std::sin(angle);

  const auto k3 = 0.5*angle/sin;
  const auto k3Jangle = 0.5*(1. - angle/std::tan(angle))/sin;

  const auto [Rt, RtJR] = detail::transposeWD(R);
  const Eigen::Matrix<Scalar,3,3> eR = R - Rt;
  const Eigen::Matrix<Scalar, 9, 9> eRJR = Eigen::Matrix<Scalar, 9, 9>::Identity() - RtJR;

  const auto [ev, evJeR] = detail::unskewSymmetricWPD(eR);

  const Eigen::Matrix<Scalar,3,3> I = Eigen::Matrix<Scalar,3,3>::Identity();
  Eigen::Map<const Eigen::Matrix<Scalar, 9, 1>> vecI{I.data(), I.size()};

  const Eigen::Matrix<Scalar, 3, 9> J = detail::kroneckerProduct(ev.transpose(), I) * vecI *k3Jangle * angleJx * trJR / 2.
    + k3 * evJeR * eRJR;
   return std::make_pair(k3 * ev, J);
}

template<typename Scalar>
Eigen::Matrix<Scalar,4,1> quaternionFromRotationMatrix(Eigen::Matrix<Scalar,3,3> const& R)
{
  const auto tr = R.trace();
  if( detail::isAlmostEq(tr, -1.0) ){
    // w is close to 0
    const Eigen::Matrix<Scalar,3,3> vvt = (R + Eigen::Matrix<Scalar,3,3>::Identity()) / 2;
    int max_i;
    const auto vi = std::sqrt(vvt.diagonal().maxCoeff(&max_i));
    return (Eigen::Matrix<Scalar,4,1>() << 0, vvt.template block<3,1>(0, max_i) / vi).finished();
  }
  const auto w = 0.5*std::sqrt(tr + 1.0);
  const Eigen::Matrix<Scalar,3,1> v = 0.25*detail::unskewSymmetric(R - R.transpose()) / w;
  return (Eigen::Matrix<Scalar,4,1>() << w, v).finished();
}

template<typename Scalar>
std::pair<Eigen::Matrix<Scalar,4,1>, Eigen::Matrix<Scalar, 4, 9>> quaternionFromRotationMatrixWD(Eigen::Matrix<Scalar,3,3> const& R)
{
  const auto [tr, trJR] = detail::traceWD(R);
  if( detail::isAlmostEq(tr, -1.0) ){
    // w is close to 0
    const Eigen::Matrix<Scalar,3,3> vvt = (R + Eigen::Matrix<Scalar,3,3>::Identity()) / 2;
    int max_i;
    const auto vi = std::sqrt(vvt.diagonal().maxCoeff(&max_i));
    return std::make_pair((Eigen::Matrix<Scalar,4,1>() << 0, vvt.template block<3,1>(0, max_i) / vi).finished(),  Eigen::Matrix<Scalar, 4, 9>::Constant(std::nan("")));
  }

  const auto w = 0.5*std::sqrt(tr + 1.0);
  Eigen::Matrix<Scalar, 1 , 9> wJR = 0.25/std::sqrt(tr+1) * trJR;

  const auto k4 = 0.25/w;
  const auto k4Jw = - k4/w;
 
  const auto [Rt, RtJR] = detail::transposeWD(R);
  const Eigen::Matrix<Scalar,3,3> eR = R - Rt;
  const Eigen::Matrix<Scalar, 9, 9> eRJR = Eigen::Matrix<Scalar, 9, 9>::Identity() - RtJR;

  const auto [ev, evJeR] = detail::unskewSymmetricWPD(eR);

  const Eigen::Matrix<Scalar,3,3> I = Eigen::Matrix<Scalar,3,3>::Identity();
  Eigen::Map<const Eigen::Matrix<Scalar, 9, 1>> vecI{I.data(), I.size()};

  const Eigen::Matrix<Scalar,3,1> v = 0.25*detail::unskewSymmetric(R - R.transpose()) / w;
  Eigen::Matrix<Scalar, 3 , 9> vJR = detail::kroneckerProduct(ev.transpose(), I) * vecI *k4Jw * wJR 
    + k4 * evJeR * eRJR;

  Eigen::Matrix<Scalar, 4, 9> J;
  J.template block<1,9>(0,0) = wJR;
  J.template block<3,9>(1,0) = vJR;
  return std::make_pair((Eigen::Matrix<Scalar,4,1>() << w, v).finished(), J);
}

}
