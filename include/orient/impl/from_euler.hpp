/* @copyright The code is licensed under the MIT License
 *            <https://opensource.org/licenses/MIT>,
 *            Copyright (c) 2020 Christian Eskil Vaugelade Berg
 * @author Christian Eskil Vaugelade Berg
*/
#pragma once

#include <orient/axis.hpp>

#include <Eigen/Dense>

namespace orient {

template<Axis axis, typename Scalar>
Eigen::Matrix<Scalar,3,3> rotationMatrixFromEuler(Scalar angle)
{
  const auto s = std::sin(angle);
  const auto c = std::cos(angle);

  constexpr auto i1 = asIndex<axis>;
  constexpr auto i2 = (i1 + 1) % 3;
  constexpr auto i3 = (i2 + 1) % 3;
  Eigen::Matrix<Scalar,3,3> R = Eigen::Matrix<Scalar,3,3>::Zero();
  R(i1, i1) = 1.0;
  R(i2, i2) = c;
  R(i3, i3) = c;
  R(i2, i3) = -s; 
  R(i3, i2) = s; 
  return R;
}

template<Axis axis, typename Scalar>
std::pair<Eigen::Matrix<Scalar,3,3>, Eigen::Matrix<Scalar,3,3>> rotationMatrixFromEulerWD(Scalar angle)
{
  const auto s = std::sin(angle);
  const auto c = std::cos(angle);

  constexpr auto i1 = asIndex<axis>;
  constexpr auto i2 = (i1 + 1) % 3;
  constexpr auto i3 = (i2 + 1) % 3;
  Eigen::Matrix<Scalar,3,3> J = Eigen::Matrix<Scalar,3,3>::Zero();
  Eigen::Matrix<Scalar,3,3> R = Eigen::Matrix<Scalar,3,3>::Zero();
  R(i1, i1) = 1.0;
  R(i2, i2) = c;
  J(i2, i2) = -s; 

  R(i3, i3) = c;
  J(i3, i3) = -s;

  R(i2, i3) = -s; 
  J(i2, i3) = -c;

  R(i3, i2) = s; 
  J(i3, i2) = c;
  return std::make_pair(R, J);
}

template<Axis A1, Axis A2, Axis A3, typename Scalar>
Eigen::Matrix<Scalar,3,3> rotationMatrixFromEuler(Eigen::Matrix<Scalar,3,1> const& angles)
{
  return 
    rotationMatrixFromEuler<A1>(angles[0]) *
    rotationMatrixFromEuler<A2>(angles[1]) *
    rotationMatrixFromEuler<A3>(angles[2]);
}

template<Axis A1, Axis A2, Axis A3, typename Scalar>
std::pair<Eigen::Matrix<Scalar,3,3>, Eigen::Matrix<Scalar, 9, 3>> rotationMatrixFromEulerWD(Eigen::Matrix<Scalar,3,1> const& angles)
{
  const auto [R1, J1] = rotationMatrixFromEulerWD<A1>(angles[0]);
  const auto [R2, J2] = rotationMatrixFromEulerWD<A2>(angles[1]);
  const auto [R3, J3] = rotationMatrixFromEulerWD<A3>(angles[2]);

  Eigen::Matrix<Scalar, 9, 3> J;
  Eigen::Map<Eigen::Matrix<Scalar,3,3>>(J.template block<9,1>(0,0).data(), 3, 3) = J1 * R2 * R3;
  Eigen::Map<Eigen::Matrix<Scalar,3,3>>(J.template block<9,1>(0,1).data(), 3, 3) = R1 * J2 * R3;
  Eigen::Map<Eigen::Matrix<Scalar,3,3>>(J.template block<9,1>(0,2).data(), 3, 3) = R1 * R2 * J3;
  return std::make_pair(R1 * R2 * R3, J);
}

}
