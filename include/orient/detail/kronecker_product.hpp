#pragma once

#include <EigenUnsupported/KroneckerProduct>

namespace orient::detail {

template <typename... T>
inline decltype(auto) kroneckerProduct(T&& ... args)
{
  return Eigen::kroneckerProduct(std::forward<T>(args) ...);
}

}
