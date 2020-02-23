#include <cmath>

namespace detail {

using std::asin;
inline double asin(double x, double& H)
{
  H = ( std::abs(x) < 1.0 - std::numeric_limits<double>::epsilon() ) 
    ? 1.0 / std::sqrt(1.0 - x*x) : std::nan("");
  return std::asin(x);
}

using std::acos;
inline double acos(double x, double& H)
{
  H = ( std::abs(x) < 1.0 - std::numeric_limits<double>::epsilon() ) 
   ? - 1.0 / std::sqrt(1.0 - x*x) : std::nan("");
  return std::acos(x);
}

using std::atan2;
inline double atan2(double y, double x, double& Hy, double& Hx)
{
  auto norm = x*x + y*y;
  auto isValid = norm > std::numeric_limits<double>::epsilon();
  Hy = isValid ? x / norm : std::nan("");
  Hx = isValid ? - y / norm : std::nan("");
  return std::atan2(y, x);
}

}
