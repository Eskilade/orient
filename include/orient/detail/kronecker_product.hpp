/* @copyright The code is licensed under the MIT License
 *            <https://opensource.org/licenses/MIT>,
 *            Copyright (c) 2020 Christian Eskil Vaugelade Berg
 * @author Christian Eskil Vaugelade Berg
*/
#pragma once

#include <unsupported/Eigen/KroneckerProduct>

namespace orient::detail {

template <typename... T>
inline decltype(auto) kroneckerProduct(T&& ... args)
{
  return Eigen::kroneckerProduct(std::forward<T>(args) ...);
}

}
