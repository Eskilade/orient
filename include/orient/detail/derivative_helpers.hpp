#pragma once

#include <cmath>
#include <Eigen/Dense>

namespace orient::detail {

template<typename Scalar>
std::pair<Scalar, Scalar> asinWD(Scalar x)
{
  return std::make_pair(std::asin(x), 1.0 / std::sqrt(1.0 - x*x));
}

template<typename Scalar>
std::pair<Scalar, Scalar> acosWD(Scalar x)
{
  return std::make_pair(std::acos(x), -1.0 / std::sqrt(1.0 - x*x));
}


template<typename Scalar>
std::tuple<Scalar, Scalar, Scalar> atan2WD(Scalar y, Scalar x)
{
  const auto norm = x*x + y*y;
  const auto Jy = x / norm;
  const auto Jx = - y / norm;
  return std::make_tuple(std::atan2(y, x), Jy, Jx);
}

template<typename Derived, typename Scalar=typename Eigen::DenseBase<Derived>::Scalar>
std::pair<Eigen::Matrix<Scalar,3,3>, Eigen::Matrix<Scalar, 9, 9>> transposeWD(Eigen::MatrixBase<Derived> const& M)
{
  Eigen::Matrix<Scalar, 9,9> J = Eigen::Matrix<Scalar, 9,9>::Identity();
  J.col(1).swap(J.col(3));
  J.col(2).swap(J.col(6));
  J.col(5).swap(J.col(7));
  return std::make_pair(M.transpose(), J);
}

template<typename Derived, typename Scalar=typename Eigen::DenseBase<Derived>::Scalar>
std::pair<Scalar, Eigen::Matrix<Scalar, 1, 9>> traceWD(Eigen::MatrixBase<Derived> const& M)
{
  Eigen::Matrix<Scalar, 1, 9> J;
  J << 1,0,0,0,1,0,0,0,1;
  return std::make_pair(M.trace(), J);
}

}
