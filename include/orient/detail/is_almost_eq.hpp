#include <cmath>

namespace orient::detail {


template<typename Scalar>
bool isAlmostEq(Scalar a, Scalar b, Scalar epsilon = Scalar{1e-10})
{
  return std::abs(a-b) < epsilon;
}

}
