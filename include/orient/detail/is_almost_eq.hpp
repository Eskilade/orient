/* @copyright The code is licensed under the MIT License
 *            <https://opensource.org/licenses/MIT>,
 *            Copyright (c) 2020 Christian Eskil Vaugelade Berg
 * @author Christian Eskil Vaugelade Berg
*/
#include <cmath>

namespace orient::detail {


template<typename Scalar>
bool isAlmostEq(Scalar a, Scalar b, Scalar epsilon = Scalar{1e-10})
{
  return std::abs(a-b) < epsilon;
}

}
