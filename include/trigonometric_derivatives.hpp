#include <cmath>
#include <common.hpp>
#include <OptionalRef.hpp>

namespace detail {
double asin(double x, OptionalRef<Mat<1,1>> H = {})
{
  if(H)
    (*H)(0,0) = ( std::abs(x) < 1.0 - std::numeric_limits<double>::epsilon() ) 
      ? 1.0 / std::sqrt(1.0 - x*x) : std::nan("");
  return std::asin(x);
}

double acos(double x, OptionalRef<Mat<1,1>> H = {})
{
  if(H)
    (*H)(0,0) = ( std::abs(x) < 1.0 - std::numeric_limits<double>::epsilon() ) 
      ? - 1.0 / std::sqrt(1.0 - x*x) : std::nan("");
  return std::acos(x);
}

double atan2(double y, double x, OptionalRef<Mat<1,1>> Hy = {}, OptionalRef<Mat<1,1>> Hx = {})
{
  if(Hy or Hx){
    auto norm = x*x + y*y;
    auto isValid = norm > std::numeric_limits<double>::epsilon();
    if(Hy)
      (*Hy)(0,0) = isValid ? x / norm : std::nan("");
    if(Hx)
      (*Hx)(0,0) = isValid ? - y / norm : std::nan("");
  }
  return std::atan2(y, x);
}
}
