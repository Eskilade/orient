#pragma once
#include <Eigen/Dense>
#include <orient/detail/skew_symmetric.hpp>
#include <orient/axis.hpp>

namespace orient::detail {

template<Axis a1, Axis a2>
constexpr Axis missing_v = Axis{3 - asIndex<a1> - asIndex<a2>};

template<Axis a1, Axis a2, Axis a3>
constexpr bool isTaitBryan_v =(asIndex<a1> + asIndex<a2> + asIndex<a3>) == 3;

template<Axis a1, Axis a2, Axis a3>
constexpr bool isProperEuler_v = (a1 == a3) and (a1 != a2);

template<Axis a1, Axis a2, Axis a3>
constexpr bool isMalformed_v = (a1 == a2) or (a2 == a3);

template<Axis a1, Axis a2, Axis a3>
struct TaitBryanTraits
{
  static const Eigen::Matrix3d sign_matrix;
  static constexpr auto a2s = asIndex<a1> + asIndex<a3>*3;
  static constexpr auto a1s = asIndex<a2> + asIndex<a3>*3;
  static constexpr auto a1c = asIndex<a3> + asIndex<a3>*3;
  static constexpr auto a3s = asIndex<a1> + asIndex<a2>*3;
  static constexpr auto a3c = asIndex<a1> + asIndex<a1>*3;

  struct GimbalLock { 
    static constexpr auto a1s = asIndex<a2> + asIndex<a1>*3;
    static constexpr auto a1c = asIndex<a2> + asIndex<a2>*3;
  };
};

template<Axis a1, Axis a2, Axis a3>
const Eigen::Matrix3d TaitBryanTraits<a1,a2,a3>::sign_matrix = Eigen::Matrix3d::Identity()+orient::detail::skewSymmetric(Eigen::Vector3d::Ones());

template<Axis a1, Axis a2>
struct ProperEulerTraits
{
  static const Eigen::Matrix3d sign_matrix;
  static constexpr Axis ma = missing_v<a1,a2>; 
  
  static constexpr auto a2c = asIndex<a1> + asIndex<a1>*3;
  static constexpr auto a1s = asIndex<a2> + asIndex<a1>*3;
  static constexpr auto a1c = asIndex<ma> + asIndex<a1>*3;
  static constexpr auto a3s = asIndex<a1> + asIndex<a2>*3;
  static constexpr auto a3c = asIndex<a1> + asIndex<ma>*3;

  struct GimbalLock {
    static constexpr auto a1s =  asIndex<a2> + asIndex<ma>*3;
    static constexpr auto a1c =  asIndex<a2> + asIndex<a2>*3;
  };
};

template<Axis a1, Axis a2>
const Eigen::Matrix3d ProperEulerTraits<a1,a2>::sign_matrix = Eigen::Matrix3d::Identity()+orient::detail::skewSymmetric(Eigen::Vector3d::Ones());

}
