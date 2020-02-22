#include <fromQuaternion.hpp>

#include <axis.hpp>
#include <detail/trigonometric_derivatives.hpp>
#include <detail/skewSymmetric.hpp>
#include <detail/so3_generator.hpp>

template<int I>
Eigen::Matrix3d rotFromQuatDer(Eigen::Vector4d const& q)
{
  double n = q.dot(q);
  double s = 1.0 / n;
  Eigen::Matrix3d X = skewSymmetric(q.segment<3>(1));
  auto sHqi = - 2 * q[I] * s * s;
  return 
    2 * sHqi * X * X +
    2 * s * generator<Axis(I-1)> * X +
    2 * s * X * generator<Axis(I-1)> +
    2* sHqi * q[0] * X + 
    2* s *q[0] * generator<Axis(I-1)>;
}

template<>
Eigen::Matrix3d rotFromQuatDer<0>(Eigen::Vector4d const& q)
{
  double n = q.dot(q);
  double s = 1.0 / n;
  Eigen::Matrix3d X = skewSymmetric(q.segment<3>(1));
  auto sHq0 = -2 * q[0]*s*s;
  return 2*sHq0 * X * X + 
           2*sHq0*q[0]*X + 
           2*s*X;
}

Eigen::Matrix3d rotationMatrixFromQuaternion(Eigen::Vector4d const& q)
{
  double s = 1 / q.dot(q);
  Eigen::Matrix3d X = skewSymmetric(q.segment<3>(1));
  return Eigen::Matrix3d::Identity() + 2 * s * X * X + 2*s*q[0]*X;
} 
 
Eigen::Matrix3d rotationMatrixFromQuaternion(Eigen::Vector4d const& q, Eigen::Ref<Eigen::Matrix<double, 9, 4>> H)
{
  Eigen::Map<Eigen::Matrix3d>(H.block<9,1>(0,0).data(), 3,3) = rotFromQuatDer<0>(q);
  Eigen::Map<Eigen::Matrix3d>(H.block<9,1>(0,1).data(), 3,3) = rotFromQuatDer<1>(q);
  Eigen::Map<Eigen::Matrix3d>(H.block<9,1>(0,2).data(), 3,3) = rotFromQuatDer<2>(q);
  Eigen::Map<Eigen::Matrix3d>(H.block<9,1>(0,3).data(), 3,3) = rotFromQuatDer<3>(q);
  return rotationMatrixFromQuaternion(q);
} 

Eigen::Vector3d angleAxisFromQuaternion(Eigen::Vector4d const& q)
{
  double w = q[0];
  Eigen::Vector3d v = q.segment<3>(1);
  double nv = std::sqrt(v.dot(v));
  const auto a =  detail::atan2(nv, w);
  return 2 * v * a / nv;
}

Eigen::Vector3d angleAxisFromQuaternion(Eigen::Vector4d const& q, Eigen::Ref<Eigen::Matrix<double, 3, 4>> H )
{
  double n = q.dot(q);
  if( n < std::numeric_limits<double>::epsilon()){
    H = Eigen::Matrix<double, 3, 4>::Constant(std::nan(""));
  }
  double w = q[0];
  Eigen::Vector3d v = q.segment<3>(1);
  double nv = std::sqrt(v.dot(v));
  double aHnv, aHw;
  const auto a =  detail::atan2(nv, w, aHnv, aHw);
  const auto aa = 2 * v * a / nv;
  const Eigen::Matrix<double, 1, 3> nvHv = v.transpose() / nv;
  H.block<3,1>(0,0) = 2 * v * aHw / nv;
  H.block<3,3>(0,1) = 2 * v * aHnv * nvHv / nv
    - 2 * a * skewSymmetric(v) * skewSymmetric(v) / (nv*nv*nv);
  return aa;
}
