#pragma once

#include <orient/axis.hpp>

#include <Eigen/Dense>

namespace orient {

template<Axis axis>
Eigen::Matrix3d rotationMatrixFromEuler(double angle)
{
  const auto s = std::sin(angle);
  const auto c = std::cos(angle);

  constexpr auto i1 = asIndex<axis>;
  constexpr auto i2 = (i1 + 1) % 3;
  constexpr auto i3 = (i2 + 1) % 3;
  Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
  R(i1, i1) = 1.0;
  R(i2, i2) = c;
  R(i3, i3) = c;
  R(i2, i3) = -s; 
  R(i3, i2) = s; 
  return R;
}

template<Axis axis>
std::pair<Eigen::Matrix3d, Eigen::Matrix3d> rotationMatrixFromEulerWD(double angle)
{
  const auto s = std::sin(angle);
  const auto c = std::cos(angle);

  constexpr auto i1 = static_cast<std::underlying_type_t<Axis>>(axis);
  constexpr auto i2 = (i1 + 1) % 3;
  constexpr auto i3 = (i2 + 1) % 3;
  Eigen::Matrix3d J = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
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

template<Axis a1, Axis a2, Axis a3>
Eigen::Matrix3d rotationMatrixFromEuler(Eigen::Vector3d const& angles)
{
  return 
    rotationMatrixFromEuler<a1>(angles[0]) *
    rotationMatrixFromEuler<a2>(angles[1]) *
    rotationMatrixFromEuler<a3>(angles[2]);
}

template<Axis a1, Axis a2, Axis a3>
std::pair<Eigen::Matrix3d, Eigen::Matrix<double, 9, 3>> rotationMatrixFromEulerWD(Eigen::Vector3d const& angles)
{
  const auto [R1, J1] = rotationMatrixFromEulerWD<a1>(angles[0]);
  const auto [R2, J2] = rotationMatrixFromEulerWD<a2>(angles[1]);
  const auto [R3, J3] = rotationMatrixFromEulerWD<a3>(angles[2]);

  Eigen::Matrix<double, 9, 3> J;
  Eigen::Map<Eigen::Matrix3d>(J.block<9,1>(0,0).data(), 3, 3) = J1 * R2 * R3;
  Eigen::Map<Eigen::Matrix3d>(J.block<9,1>(0,1).data(), 3, 3) = R1 * J2 * R3;
  Eigen::Map<Eigen::Matrix3d>(J.block<9,1>(0,2).data(), 3, 3) = R1 * R2 * J3;
  return std::make_pair(R1 * R2 * R3, J);
}

}
