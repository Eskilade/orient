#pragma once

#include <Eigen/Dense>

template<int N>
Eigen::Matrix<double, N, 1> normalize(Eigen::Matrix<double, N, 1> const& v)
{
  double n2 = v.dot(v);
  if( n2 < std::numeric_limits<double>::epsilon())
    return Eigen::Matrix<double, N, 1>::Constant(std::nan(""));
  double n = std::sqrt(n2);
  return v / n;
}

// Note : 
// Template does not deduce the correct type for the matrix reference,
// so make it non-deducible as first argument is sufficient
// for the deduction of N
template< class T >
struct type_identity {
      using type = T;
};

template<int N>
Eigen::Matrix<double, N, 1> normalize(Eigen::Matrix<double, N, 1> const& v, typename type_identity<Eigen::Ref<Eigen::Matrix<double, N, N>>>::type H)
{
  double n2 = v.dot(v);
  if( n2 < std::numeric_limits<double>::epsilon()){
    H = Eigen::Matrix<double, N, N>::Constant(std::nan(""));
    return Eigen::Matrix<double, N, 1>::Constant(std::nan(""));
  }
  double n = std::sqrt(n2);
  H = - v * v.transpose() / ( n * n2) + Eigen::Matrix<double, N, N>::Identity() * 1.0 / n;
  return v / n;
}
