#pragma once

#include <orient/detail/so3_generator.hpp>

#include <Eigen/Dense>

namespace orient::detail {

template <typename Derived, typename Scalar = typename Eigen::DenseBase<Derived>::Scalar>
Eigen::Matrix<Scalar,3,3> skewSymmetric(Eigen::DenseBase<Derived> const& w) {
  return (Eigen::Matrix<Scalar,3,3>() << 0.0, -w(2), w(1), w(2), 0.0, -w(0), -w(1), w(0), 0.0).finished();
}

template <typename Derived, typename Scalar = typename Eigen::DenseBase<Derived>::Scalar>
std::pair<Eigen::Matrix<Scalar,3,3>, Eigen::Matrix<Scalar, 9, 3>> skewSymmetricWPD(Eigen::MatrixBase<Derived> const& w) {
  Eigen::Matrix<Scalar, 9, 3> J{};
  Eigen::Map<Eigen::Matrix<Scalar,3,3>>(J.template block<9,1>(0,0).data(), 3,3) = generator<Axis::x>;
  Eigen::Map<Eigen::Matrix<Scalar,3,3>>(J.template block<9,1>(0,1).data(), 3,3) = generator<Axis::y>;
  Eigen::Map<Eigen::Matrix<Scalar,3,3>>(J.template block<9,1>(0,2).data(), 3,3) = generator<Axis::z>;
  return std::make_pair(skewSymmetric(w), J);
}

template <typename Derived, typename Scalar = typename Eigen::DenseBase<Derived>::Scalar>
Eigen::Matrix<Scalar,3,1> unskewSymmetric(Eigen::MatrixBase<Derived> const& M)
{
  return (Eigen::Matrix<Scalar,3,1>() << - M(1,2), M(0,2), - M(0,1)).finished();
}

template <typename Derived, typename Scalar = typename Eigen::DenseBase<Derived>::Scalar>
std::pair<Eigen::Matrix<Scalar,3,1>, Eigen::Matrix<Scalar, 3, 9>> unskewSymmetricWPD(Eigen::MatrixBase<Derived> const& M)
{
  Eigen::Matrix<Scalar, 3, 9> J = Eigen::Matrix<Scalar, 3, 9>::Zero();
  J(0, 2*3 + 1) = -1.;
  J(1, 2*3) = 1.;
  J(2, 1*3) = -1.;
  return std::make_pair(unskewSymmetric(M), J);
}

}
