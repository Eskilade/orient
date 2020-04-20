#pragma once

#include <orient/detail/so3_generator.hpp>

#include <Eigen/Dense>

namespace orient::detail {

template <class Derived>
inline Eigen::Matrix3d skewSymmetric(const Eigen::MatrixBase<Derived>& w) {
  return (Eigen::Matrix3d() << 0.0, -w(2), w(1), w(2), 0.0, -w(0), -w(1), w(0), 0.0).finished();
}

template <class Derived>
inline std::pair<Eigen::Matrix3d, Eigen::Matrix<double, 9, 3>>
skewSymmetricWPD(const Eigen::MatrixBase<Derived>& w) {
  Eigen::Matrix<double, 9, 3> H{};
  Eigen::Map<Eigen::Matrix3d>(H.block<9,1>(0,0).data(), 3,3) = generator<Axis::x>;
  Eigen::Map<Eigen::Matrix3d>(H.block<9,1>(0,1).data(), 3,3) = generator<Axis::y>;
  Eigen::Map<Eigen::Matrix3d>(H.block<9,1>(0,2).data(), 3,3) = generator<Axis::z>;
  return std::make_pair(skewSymmetric(w), H);
}

template <class Derived>
inline Eigen::Vector3d unskewSymmetric(const Eigen::MatrixBase<Derived>& M)
{
  return (Eigen::Vector3d() << - M(1,2), M(0,2), - M(0,1)).finished();
}

template <class Derived>
inline 
std::pair<Eigen::Vector3d, Eigen::Matrix<double, 3, 9>>
unskewSymmetricWPD(const Eigen::MatrixBase<Derived>& M)
{
  Eigen::Matrix<double, 3, 9> H = Eigen::Matrix<double, 3, 9>::Zero();
  H(0, 2*3 + 1) = -1.;
  H(1, 2*3) = 1.;
  H(2, 1*3) = -1.;
  return std::make_pair(unskewSymmetric(M), H);
}
}
