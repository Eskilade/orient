#pragma once

#include <axis.hpp>
#include <detail/axis_traits.hpp>
#include <type_traits>

template<Axis A1, Axis A2, Axis A3, std::enable_if_t<isTaitBryan<A1,A2,A3>(), int> = 0>
Eigen::Vector3d eulerFromRotationMatrix(Eigen::Matrix3d const& R);

template<Axis A1, Axis A2, Axis A3, std::enable_if_t<isTaitBryan<A1,A2,A3>(), int> = 0>
Eigen::Vector3d eulerFromRotationMatrix(Eigen::Matrix3d const& R, Eigen::Ref<Eigen::Matrix<double, 3, 9>> H);

template<Axis A1, Axis A2, Axis A3, std::enable_if_t<isProperEuler<A1,A2,A3>(), int> = 0>
Eigen::Vector3d eulerFromRotationMatrix(Eigen::Matrix3d const& R);

template<Axis A1, Axis A2, Axis A3, std::enable_if_t<isProperEuler<A1,A2,A3>(), int> = 0>
Eigen::Vector3d eulerFromRotationMatrix(Eigen::Matrix3d const& R, Eigen::Ref<Eigen::Matrix<double, 3, 9>> H);
 
/* imp */ 
#include <detail/trigonometric_derivatives.hpp>
#include <detail/skewSymmetric.hpp>

// @brief A helper class for accessing rotation matrix elements
// that also takes into account the sign
template<typename Derived>
class Wrap
{
public:
  Wrap(Eigen::MatrixBase<Derived> const& m):
      ref{m},
      sign_mat{Eigen::Matrix3d::Identity()+
        skewSymmetric(Eigen::Vector3d::Ones())}
    {};

  auto operator()(detail::Index const& e) const
  {
    return sign(e) * uncorrected(e);
  }

  auto uncorrected(detail::Index const& e) const
  {
    return ref(e.r, e.c);
  }

  auto sign(detail::Index const& e) const
  {
    return sign_mat(e.r, e.c);
  }

private:
  Eigen::Ref<const Eigen::Matrix3d> ref;
  Eigen::Matrix3d sign_mat;
};

template<Axis A1, Axis A2, Axis A3, std::enable_if_t<isTaitBryan<A1,A2,A3>(), int>>
Eigen::Vector3d eulerFromRotationMatrix(Eigen::Matrix3d const& R)
{
  using T = TaitBryanTraits<A1,A2,A3>;
  const auto W = Wrap{R};
  const auto lone_v = W.uncorrected(T::a2s);
  const auto thres = 1.0 - std::numeric_limits<float>::epsilon();
  double a1,a2,a3;
  if( lone_v < thres ){
    if( lone_v > - thres ){
      a2 = detail::asin( W(T::a2s));
      a1 = detail::atan2(W(T::a1s), W(T::a1c));
      a3 = detail::atan2(W(T::a3s), W(T::a3c));
    } 
    else { // Gimbal lock. lone value ~= -1
      using TGL = typename T::GimbalLock;
      a2 =  - W.sign(T::a2s) * M_PI / 2.0;
      a1 =  - detail::atan2( W(TGL::a1s), W(TGL::a1c) );
      a3 = 0.;
    }
  }
  else { // Gimbal lock. lone value ~= +1
    using TGL = typename T::GimbalLock;
    a2 =  W.sign(T::a2s) * M_PI / 2.0;
    a1 =  detail::atan2( W(TGL::a1s), W(TGL::a1c) );
    a3 = 0.;
  }
  return (Eigen::Vector3d() << a1,a2,a3).finished();
}

template<Axis A1, Axis A2, Axis A3, std::enable_if_t<isTaitBryan<A1,A2,A3>(), int> >
Eigen::Vector3d eulerFromRotationMatrix(Eigen::Matrix3d const& R, Eigen::Ref<Eigen::Matrix<double, 3, 9>> H)
{
  using T = TaitBryanTraits<A1,A2,A3>;
  H = Eigen::Matrix<double, 3, 9>::Zero();
  const auto W = Wrap{R};
  const auto lone_v = W.uncorrected(T::a2s);
  const auto thres = 1.0 - std::numeric_limits<float>::epsilon();
  double a1,a2,a3;
  if( lone_v < thres ){
    if( lone_v > - thres ){
      a2 = detail::asin( W(T::a2s), 
          H(1, T::a2s.c*3 + T::a2s.r));
      H(1, T::a2s.c*3 + T::a2s.r) *= W.sign(T::a2s);

      a1 = detail::atan2(W(T::a1s), W(T::a1c),
          H(0, T::a1s.c*3 + T::a1s.r),
          H(0, T::a1c.c*3 + T::a1c.r));
      H(0, T::a1s.c*3 + T::a1s.r) *= W.sign(T::a1s);
      H(0, T::a1c.c*3 + T::a1c.r) *= W.sign(T::a1c);

      a3 = detail::atan2(W(T::a3s), W(T::a3c),
      H(2, T::a3s.c*3 + T::a3s.r),
      H(2, T::a3c.c*3 + T::a3c.r));
      H(2, T::a3s.c*3 + T::a3s.r) *= W.sign(T::a3s);
      H(2, T::a3c.c*3 + T::a3c.r) *= W.sign(T::a3c);
    } 
    else { // Gimbal lock. lone value ~= -1
      using TGL = typename T::GimbalLock;
      a2 =  - W.sign(T::a2s) * M_PI / 2.0;
      a1 =  - detail::atan2( W(TGL::a1s), W(TGL::a1c) );
      a3 = 0.;
      H = Eigen::Matrix<double, 3, 9>::Constant(std::nan(""));
    }
  }
  else { // Gimbal lock. lone value ~= +1
    using TGL = typename T::GimbalLock;
    a2 =  W.sign(T::a2s) * M_PI / 2.0;
    a1 =  detail::atan2( W(TGL::a1s), W(TGL::a1c) );
    a3 = 0.;
    H = Eigen::Matrix<double, 3, 9>::Constant(std::nan(""));
  }
  return (Eigen::Vector3d() << a1,a2,a3).finished();
}

template<Axis A1, Axis A2, Axis A3, std::enable_if_t<isProperEuler<A1,A2,A3>(), int> >
Eigen::Vector3d eulerFromRotationMatrix(Eigen::Matrix3d const& R)
{
  using T = ProperEulerTraits<A1,A2>; 
  const auto W = Wrap{R};
  const auto lone_v = W.uncorrected(T::a2c);
  const auto thres = 1.0 - std::numeric_limits<float>::epsilon();
  double a1,a2,a3;
  if( lone_v < thres ){
    if( lone_v > - thres ){
      a2 = detail::acos( W(T::a2c));
      a1 = detail::atan2(W.uncorrected(T::a1s), W(T::a1c));
      a3 = detail::atan2(W.uncorrected(T::a3s), W(T::a3c));
    } 
    else { // Gimbal lock. lone value ~= -1
      using TGL = typename T::GimbalLock;
      a2 =  M_PI;
      a1 =  - detail::atan2( W(TGL::a1s), W(TGL::a1c) );
      a3 = 0.;
    }
  }
  else { // Gimbal lock. lone value ~= +1
    using TGL = typename T::GimbalLock;
    a2 =  0.0;
    a1 =  detail::atan2( W(TGL::a1s), W(TGL::a1c) );
    a3 = 0.;
  }
  return (Eigen::Vector3d() << a1,a2,a3).finished();
}

template<Axis A1, Axis A2, Axis A3, std::enable_if_t<isProperEuler<A1,A2,A3>(), int> >
Eigen::Vector3d eulerFromRotationMatrix(Eigen::Matrix3d const& R, Eigen::Ref<Eigen::Matrix<double, 3, 9>> H)
{
  using T = ProperEulerTraits<A1,A2>; 
  H = Eigen::Matrix<double, 3, 9>::Zero();
  const auto W = Wrap{R};
  const auto lone_v = W.uncorrected(T::a2c);
  const auto thres = 1.0 - std::numeric_limits<float>::epsilon();
  double a1,a2,a3;
  if( lone_v < thres ){
    if( lone_v > - thres ){
      a2 = detail::acos( W(T::a2c), 
          H(1, T::a2c.c*3 + T::a2c.r));
      H(1, T::a2c.c*3 + T::a2c.r) *= W.sign(T::a2c);

      a1 = detail::atan2(W.uncorrected(T::a1s), W(T::a1c),
          H(0, T::a1s.c*3 + T::a1s.r),
          H(0, T::a1c.c*3 + T::a1c.r) );
      H(0, T::a1c.c*3 + T::a1c.r) *= W.sign(T::a1c);
      a3 = detail::atan2(W.uncorrected(T::a3s), W(T::a3c),
          H(2, T::a3s.c*3 + T::a3s.r),
          H(2, T::a3c.c*3 + T::a3c.r) );
      H(2, T::a3c.c*3 + T::a3c.r) *= W.sign(T::a3c);
    } 
    else { // Gimbal lock. lone value ~= -1
      using TGL = typename T::GimbalLock;
      a2 =  M_PI;
      a1 =  - detail::atan2( W(TGL::a1s), W(TGL::a1c) );
      a3 = 0.;
      H = Eigen::Matrix<double, 3, 9>::Constant(std::nan(""));
    }
  }
  else { // Gimbal lock. lone value ~= +1
    using TGL = typename T::GimbalLock;
    a2 =  0.0;
    a1 =  detail::atan2( W(TGL::a1s), W(TGL::a1c) );
    a3 = 0.;
    H = Eigen::Matrix<double, 3, 9>::Constant(std::nan(""));
  }
  return (Eigen::Vector3d() << a1,a2,a3).finished();
}

template<Axis A1, Axis A2, Axis A3, std::enable_if_t<isMalformed<A1,A2,A3>(), int> >
void eulerFromRotationMatrix(Eigen::Matrix3d const&)
{
  constexpr bool alwaysFalse = ( A1 != A1);
  static_assert( alwaysFalse, "Passed a malformed rotation sequence. Please choose either a propor euler sequence or a Tait-Bryan sequence");
}

