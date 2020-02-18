#include <fromQuaternion.hpp>
#include <common.hpp>
#include <trigonometric_derivatives.hpp>

template<int I>
Mat<3,3> rotFromQuatDer(Vect<4> const& q)
{
  double n = q.dot(q);
  double s = 1.0 / n;
  Mat<3,3> X = gtsam::skewSymmetric(q.segment<3>(1));
  auto sHqi = - 2 * q[I] * s * s;
  return 
    2 * sHqi * X * X +
    2 * s * generator<Axis(I-1)> * X +
    2 * s * X * generator<Axis(I-1)> +
    2* sHqi * q[0] * X + 
    2* s *q[0] * generator<Axis(I-1)>;
}

template<>
Mat<3,3> rotFromQuatDer<0>(Vect<4> const& q)
{
  double n = q.dot(q);
  double s = 1.0 / n;
  Mat<3,3> X = gtsam::skewSymmetric(q.segment<3>(1));
  auto sHq0 = -2 * q[0]*s*s;
  return 2*sHq0 * X * X + 
           2*sHq0*q[0]*X + 
           2*s*X;
}

Mat<3,3> rotationMatrixFromQuaternion(Vect<4> const& q)
{
  double s = 1 / q.dot(q);
  Mat<3,3> X = gtsam::skewSymmetric(q.segment<3>(1));
  return Mat<3,3>::Identity() + 2 * s * X * X + 2*s*q[0]*X;
} 
 
Mat<3,3> rotationMatrixFromQuaternion(Vect<4> const& q, Eigen::Ref<Mat<9,4>> H)
{
  Eigen::Map<Mat<3,3>>(H.block<9,1>(0,0).data(), 3,3) = rotFromQuatDer<0>(q);
  Eigen::Map<Mat<3,3>>(H.block<9,1>(0,1).data(), 3,3) = rotFromQuatDer<1>(q);
  Eigen::Map<Mat<3,3>>(H.block<9,1>(0,2).data(), 3,3) = rotFromQuatDer<2>(q);
  Eigen::Map<Mat<3,3>>(H.block<9,1>(0,3).data(), 3,3) = rotFromQuatDer<3>(q);
  return rotationMatrixFromQuaternion(q);
} 

Vect<3> angleAxisFromQuaternion(Vect<4> const& q)
{
  double n = q.dot(q);
  double w = q[0];
  Vect<3> v = q.segment<3>(1);
  double nv = std::sqrt(v.dot(v));
  const auto a =  detail::atan2(nv, w);
  return 2 * v * a / nv;
}

Vect<3> angleAxisFromQuaternion(Vect<4> const& q, Eigen::Ref<Mat<3,4>> H )
{
  double n = q.dot(q);
  if( n < std::numeric_limits<double>::epsilon()){
    H = Mat<3,4>::Constant(std::nan(""));
  }
  double w = q[0];
  Vect<3> v = q.segment<3>(1);
  double nv = std::sqrt(v.dot(v));
  double aHnv, aHw;
  const auto a =  detail::atan2(nv, w, aHnv, aHw);
  const auto aa = 2 * v * a / nv;
  const Mat<1,3> nvHv = v.transpose() / nv;
  H.block<3,1>(0,0) = 2 * v * aHw / nv;
  H.block<3,3>(0,1) = 2 * v * aHnv * nvHv / nv
    - 2 * a * gtsam::skewSymmetric(v) * gtsam::skewSymmetric(v) / (nv*nv*nv);
  return aa;
}
