#pragma once

#include <Eigen/Dense>

inline Eigen::Matrix3d skewSymmetric(double wx, double wy, double wz) {
  return (Eigen::Matrix3d() << 0.0, -wz, +wy, +wz, 0.0, -wx, -wy, +wx, 0.0).finished();
}

template <class Derived>
inline Eigen::Matrix3d skewSymmetric(const Eigen::MatrixBase<Derived>& w) {
  return skewSymmetric(w(0), w(1), w(2));
}

template <class Derived>
inline Eigen::Vector3d vectorFromSkewSymmetric(const Eigen::MatrixBase<Derived>& M)
{
  return (Eigen::Vector3d() << - M(1,2), M(0,2), - M(0,1)).finished();
}
