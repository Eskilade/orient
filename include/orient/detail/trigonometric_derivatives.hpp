#include <cmath>

namespace orient::detail {

inline std::pair<double, double> asinWD(double x)
{
  return std::make_pair(std::asin(x), 1.0 / std::sqrt(1.0 - x*x));
}

inline std::pair<double, double> acosWD(double x)
{
  return std::make_pair(std::acos(x), -1.0 / std::sqrt(1.0 - x*x));
}


inline std::tuple<double, double, double>
atan2WD(double y, double x)
{
  const auto norm = x*x + y*y;
  const auto Hy = x / norm;
  const auto Hx = - y / norm;
  return std::make_tuple(std::atan2(y, x), Hy, Hx);
}

}
