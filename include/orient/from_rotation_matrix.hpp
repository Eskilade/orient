#pragma once

#include <orient/axis.hpp>
#include <orient/detail/axis_traits.hpp>

#include <Eigen/Dense>

#include <type_traits>
#include <utility>

namespace orient {

Eigen::Vector3d angleAxisFromRotationMatrix(Eigen::Matrix3d const& R);
std::pair<Eigen::Vector3d, Eigen::Matrix<double, 3, 9>> angleAxisFromRotationMatrixWD(Eigen::Matrix3d const& R);

Eigen::Vector4d quaternionFromRotationMatrix(Eigen::Matrix3d const& R);
std::pair<Eigen::Vector4d, Eigen::Matrix<double, 4, 9>> quaternionFromRotationMatrixWD(Eigen::Matrix3d const& R);

template<Axis A1, Axis A2, Axis A3, std::enable_if_t<detail::isTaitBryan_v<A1,A2,A3>, int> = 0>
Eigen::Vector3d eulerFromRotationMatrix(Eigen::Matrix3d const& R);

template<Axis A1, Axis A2, Axis A3, std::enable_if_t<detail::isTaitBryan_v<A1,A2,A3>, int> = 0>
std::pair<Eigen::Vector3d, Eigen::Matrix<double, 3, 9>> eulerFromRotationMatrixWD(Eigen::Matrix3d const& R);

template<Axis A1, Axis A2, Axis A3, std::enable_if_t<detail::isProperEuler_v<A1,A2,A3>, int> = 0>
Eigen::Vector3d eulerFromRotationMatrix(Eigen::Matrix3d const& R);

template<Axis A1, Axis A2, Axis A3, std::enable_if_t<detail::isProperEuler_v<A1,A2,A3>, int> = 0>
std::pair<Eigen::Vector3d, Eigen::Matrix<double, 3, 9>> eulerFromRotationMatrixWD(Eigen::Matrix3d const& R);

}
 
/* imp eulerFromRotationMatrix */ 
#include <orient/detail/derivative_helpers.hpp>
#include <orient/detail/skew_symmetric.hpp>

namespace orient {

template<Axis A1, Axis A2, Axis A3, std::enable_if_t<detail::isTaitBryan_v<A1,A2,A3>, int>>
Eigen::Vector3d eulerFromRotationMatrix(Eigen::Matrix3d const& R)
{
  using T = detail::TaitBryanTraits<A1,A2,A3>;
  const auto& sign = T::sign_matrix;
  const auto s2 = R(T::a2s);
  const auto thres = 1.0 - 1e-20;
  double a1,a2,a3;
  if( s2 < thres ){
    if( s2 > - thres ){
      a2 = std::asin( sign(T::a2s) * s2);
      a1 = std::atan2(sign(T::a1s) * R(T::a1s), sign(T::a1c) * R(T::a1c));
      a3 = std::atan2(sign(T::a3s) * R(T::a3s), sign(T::a3c) * R(T::a3c));
    } 
    else { // Gimbal lock. s2 ~= -1
      using TGL = typename T::GimbalLock;
      a2 =  - sign(T::a2s) * M_PI / 2.0;
      a1 =  - std::atan2( sign(TGL::a1s) * R(TGL::a1s), sign(TGL::a1c) * R(TGL::a1c) );
      a3 = 0.;
    }
  }
  else { // Gimbal lock. s2 ~= +1
    using TGL = typename T::GimbalLock;
    a2 =  sign(T::a2s) * M_PI / 2.0;
    a1 =  std::atan2( sign(TGL::a1s) * R(TGL::a1s), sign(TGL::a1c) * R(TGL::a1c) );
    a3 = 0.;
  }
  return (Eigen::Vector3d() << a1,a2,a3).finished();
}

template<Axis A1, Axis A2, Axis A3, std::enable_if_t<detail::isTaitBryan_v<A1,A2,A3>, int> >
std::pair<Eigen::Vector3d, Eigen::Matrix<double, 3, 9>> eulerFromRotationMatrixWD(Eigen::Matrix3d const& R)
{
  using T = detail::TaitBryanTraits<A1,A2,A3>;
  Eigen::Matrix<double, 3, 9> J = Eigen::Matrix<double, 3, 9>::Zero();
  const auto& sign = T::sign_matrix;
  const auto s2 = R(T::a2s);
  const auto thres = 1.0 - 1e-20;
  double a1,a2,a3;
  if( s2 < thres ){
    if( s2 > - thres ){
      std::tie(a2, J(1, T::a2s)) = detail::asinWD( sign(T::a2s) * R(T::a2s) );
      J(1, T::a2s) *= sign(T::a2s);

      std::tie(a1, J(0, T::a1s), J(0, T::a1c)) = 
        detail::atan2WD(sign(T::a1s) * R(T::a1s), sign(T::a1c) * R(T::a1c));
      J(0, T::a1s) *= sign(T::a1s);
      J(0, T::a1c) *= sign(T::a1c);

      std::tie(a3, J(2, T::a3s), J(2, T::a3c)) =
        detail::atan2WD(sign(T::a3s)*R(T::a3s), sign(T::a3c)*R(T::a3c));
      J(2, T::a3s) *= sign(T::a3s);
      J(2, T::a3c) *= sign(T::a3c);
    } 
    else { // Gimbal lock. s2 ~= -1
      using TGL = typename T::GimbalLock;
      a2 =  - sign(T::a2s) * M_PI / 2.0;
      a1 =  - std::atan2( sign(TGL::a1s) * R(TGL::a1s), sign(TGL::a1c) * R(TGL::a1c) );
      a3 = 0.;
      J = Eigen::Matrix<double, 3, 9>::Constant(std::nan(""));
    }
  }
  else { // Gimbal lock. s2 ~= +1
    using TGL = typename T::GimbalLock;
    a2 =  sign(T::a2s) * M_PI / 2.0;
    a1 =  std::atan2( sign(TGL::a1s) * R(TGL::a1s), sign(TGL::a1c) * R(TGL::a1c) );
    a3 = 0.;
    J = Eigen::Matrix<double, 3, 9>::Constant(std::nan(""));
  }
  return std::make_pair((Eigen::Vector3d() << a1,a2,a3).finished(), J);
}

template<Axis A1, Axis A2, Axis A3, std::enable_if_t<detail::isProperEuler_v<A1,A2,A3>, int> >
Eigen::Vector3d eulerFromRotationMatrix(Eigen::Matrix3d const& R)
{
  using T = detail::ProperEulerTraits<A1,A2>; 
  const auto& sign = T::sign_matrix;
  const auto c2 = R(T::a2c);
  const auto thres = 1.0 - std::numeric_limits<float>::epsilon();
  double a1,a2,a3;
  if( c2 < thres ){
    if( c2 > - thres ){
      a2 = std::acos( sign(T::a2c) * R(T::a2c));
      a1 = std::atan2(R(T::a1s), sign(T::a1c) * R(T::a1c));
      a3 = std::atan2(R(T::a3s), sign(T::a3c) * R(T::a3c));
    } 
    else { // Gimbal lock. c2 ~= -1
      using TGL = typename T::GimbalLock;
      a2 =  M_PI;
      a1 =  - std::atan2( sign(TGL::a1s) * R(TGL::a1s), sign(TGL::a1c) * R(TGL::a1c) );
      a3 = 0.;
    }
  }
  else { // Gimbal lock. c2 ~= +1
    using TGL = typename T::GimbalLock;
    a2 = 0.0;
    a1 = std::atan2( sign(TGL::a1s) * R(TGL::a1s), sign(TGL::a1c) * R(TGL::a1c) );
    a3 = 0.;
  }
  return (Eigen::Vector3d() << a1,a2,a3).finished();
}

template<Axis A1, Axis A2, Axis A3, std::enable_if_t<detail::isProperEuler_v<A1,A2,A3>, int> >
std::pair<Eigen::Vector3d, Eigen::Matrix<double, 3, 9>> eulerFromRotationMatrixWD(Eigen::Matrix3d const& R)
{
  using T = detail::ProperEulerTraits<A1,A2>; 
  Eigen::Matrix<double, 3, 9> J = Eigen::Matrix<double, 3, 9>::Zero();
  const auto& sign = T::sign_matrix;
  const auto c2 = R(T::a2c);
  const auto thres = 1.0 - std::numeric_limits<float>::epsilon();
  double a1,a2,a3;
  if( c2 < thres ){
    if( c2 > - thres ){
      std::tie(a2, J(1, T::a2c)) =
        detail::acosWD( sign(T::a2c) * R(T::a2c) );
      J(1, T::a2c) *= sign(T::a2c);

      std::tie(a1, J(0, T::a1s), J(0, T::a1c)) =
        detail::atan2WD(R(T::a1s), sign(T::a1c)*R(T::a1c));
      J(0, T::a1c) *= sign(T::a1c);

      std::tie(a3, J(2, T::a3s), J(2, T::a3c)) =
        detail::atan2WD(R(T::a3s), sign(T::a3c) * R(T::a3c));
      J(2, T::a3c) *= sign(T::a3c);
    } 
    else { // Gimbal lock. c2 ~= -1
      using TGL = typename T::GimbalLock;
      a2 = M_PI;
      a1 = - std::atan2( sign(TGL::a1s) * R(TGL::a1s), sign(TGL::a1c) * R(TGL::a1c) );
      a3 = 0.;
      J = Eigen::Matrix<double, 3, 9>::Constant(std::nan(""));
    }
  }
  else { // Gimbal lock. c2 ~= +1
    using TGL = typename T::GimbalLock;
    a2 = 0.0;
    a1 = std::atan2( sign(TGL::a1s) * R(TGL::a1s), sign(TGL::a1c) * R(TGL::a1c) );
    a3 = 0.;
    J = Eigen::Matrix<double, 3, 9>::Constant(std::nan(""));
  }
  return std::make_pair((Eigen::Vector3d() << a1,a2,a3).finished(), J);
}

template<Axis A1, Axis A2, Axis A3, std::enable_if_t<detail::isMalformed_v<A1,A2,A3>, int> >
void eulerFromRotationMatrix(Eigen::Matrix3d const&)
{
  constexpr bool alwaysFalse = ( A1 != A1);
  static_assert( alwaysFalse, "Passed a malformed rotation sequence. Please choose either a propor euler sequence or a Tait-Bryan sequence");
}

template<Axis A1, Axis A2, Axis A3, std::enable_if_t<detail::isMalformed_v<A1,A2,A3>, int> >
void eulerFromRotationMatrixWD(Eigen::Matrix3d const& arg)
{
  // Reuse same compilation error message
  eulerFromRotationMatrix<A1, A2, A3>(arg);
}

}
