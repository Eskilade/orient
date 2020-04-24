#pragma once

#include <orient/axis.hpp>

#include <Eigen/Dense>

namespace orient {

template<Axis axis>
Eigen::Matrix3d rotationMatrixFromEuler(double angle)
{
  const auto s = std::sin(angle);
  const auto c = std::cos(angle);

  constexpr auto i1 = static_cast<std::underlying_type_t<Axis>>(axis);
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
Eigen::Matrix3d rotationMatrixFromEuler(double angle, Eigen::Ref<Eigen::Matrix3d> H)
{
  const auto s = std::sin(angle);
  const auto c = std::cos(angle);

  constexpr auto i1 = static_cast<std::underlying_type_t<Axis>>(axis);
  constexpr auto i2 = (i1 + 1) % 3;
  constexpr auto i3 = (i2 + 1) % 3;
  H = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
  R(i1, i1) = 1.0;
  R(i2, i2) = c;
  H(i2, i2) = -s; 

  R(i3, i3) = c;
  H(i3, i3) = -s;

  R(i2, i3) = -s; 
  H(i2, i3) = -c;

  R(i3, i2) = s; 
  H(i3, i2) = c;
  return R;
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
Eigen::Matrix3d rotationMatrixFromEuler(Eigen::Vector3d const& angles, Eigen::Ref<Eigen::Matrix<double, 9, 3>> H)
{
  Eigen::Matrix3d H1, H2, H3;
  const Eigen::Matrix3d R1 = rotationMatrixFromEuler<a1>(angles[0], H1);
  const Eigen::Matrix3d R2 = rotationMatrixFromEuler<a2>(angles[1], H2);
  const Eigen::Matrix3d R3 = rotationMatrixFromEuler<a3>(angles[2], H3);

  Eigen::Map<Eigen::Matrix3d>(H.block<9,1>(0,0).data(), 3, 3) = H1 * R2 * R3;
  Eigen::Map<Eigen::Matrix3d>(H.block<9,1>(0,1).data(), 3, 3) = R1 * H2 * R3;
  Eigen::Map<Eigen::Matrix3d>(H.block<9,1>(0,2).data(), 3, 3) = R1 * R2 * H3;
  return R1 * R2 * R3;
}

}
