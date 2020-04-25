#include <cmath>

namespace orient::detail {


template<typename T>
typename std::enable_if<std::is_floating_point_v<T>, bool>::type isAlmostEq(T a, T b, T epsilon = T{1e-10})
{
  return std::abs(a-b) < epsilon;
}

}
