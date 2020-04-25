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
  const auto Hy = x / norm;
  const auto Hx = - y / norm;
  return std::make_tuple(std::atan2(y, x), Hy, Hx);
}

inline std::pair<Eigen::Matrix3d, Eigen::Matrix<double, 9, 9>> transposeWD(Eigen::Matrix3d const& M)
{
  Eigen::Matrix<double, 9,9> H = Eigen::Matrix<double, 9,9>::Identity();
  H.col(1).swap(H.col(3));
  H.col(2).swap(H.col(6));
  H.col(5).swap(H.col(7));
  return std::make_pair(M.transpose(), H);
}

inline std::tuple<double, Eigen::Matrix<double, 1, 9>> traceWD(Eigen::Matrix3d const& M)
{
  Eigen::Matrix<double, 1, 9> H;
  H << 1,0,0,0,1,0,0,0,1;
  return std::make_tuple(M.trace(), H);
}

}
