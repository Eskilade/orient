#pragma once

#include <cmath>
#include <Eigen/Dense>

namespace orient::detail {

inline std::pair<double, double> asinWD(double x)
{
  return std::make_pair(std::asin(x), 1.0 / std::sqrt(1.0 - x*x));
}

inline std::pair<double, double> acosWD(double x)
{
  return std::make_pair(std::acos(x), -1.0 / std::sqrt(1.0 - x*x));
}


inline std::tuple<double, double, double> atan2WD(double y, double x)
{
  const auto norm = x*x + y*y;
  const auto Jy = x / norm;
  const auto Jx = - y / norm;
  return std::make_tuple(std::atan2(y, x), Jy, Jx);
}

inline std::pair<Eigen::Matrix3d, Eigen::Matrix<double, 9, 9>> transposeWD(Eigen::Matrix3d const& M)
{
  Eigen::Matrix<double, 9,9> J = Eigen::Matrix<double, 9,9>::Identity();
  J.col(1).swap(J.col(3));
  J.col(2).swap(J.col(6));
  J.col(5).swap(J.col(7));
  return std::make_pair(M.transpose(), J);
}

inline std::tuple<double, Eigen::Matrix<double, 1, 9>> traceWD(Eigen::Matrix3d const& M)
{
  Eigen::Matrix<double, 1, 9> J;
  J << 1,0,0,0,1,0,0,0,1;
  return std::make_tuple(M.trace(), J);
}

}
