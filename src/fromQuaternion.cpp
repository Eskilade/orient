#include <fromQuaternion.hpp>
#include <common.hpp>

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
  
Mat<3,3> rotationMatrixFromQuaternion(Vect<4> const& q, OptionalRef<Mat<9,4>> H)
{
  if(H){
    Eigen::Map<Mat<3,3>>(H->block<9,1>(0,0).data(), 3,3) = rotFromQuatDer<0>(q);
    Eigen::Map<Mat<3,3>>(H->block<9,1>(0,1).data(), 3,3) = rotFromQuatDer<1>(q);
    Eigen::Map<Mat<3,3>>(H->block<9,1>(0,2).data(), 3,3) = rotFromQuatDer<2>(q);
    Eigen::Map<Mat<3,3>>(H->block<9,1>(0,3).data(), 3,3) = rotFromQuatDer<3>(q);
  }
  double s = 1 / q.dot(q);
  Mat<3,3> X = gtsam::skewSymmetric(q.segment<3>(1));
  return Mat<3,3>::Identity() + 2 * s * X * X + 2*s*q[0]*X;
} 


Vect<3> angleAxisFromQuaternion(Vect<4> const& q, OptionalRef<Mat<3,4>> H )
{
  double n = q.dot(q);
  if( n < std::numeric_limits<double>::epsilon()){
    if(H)
      *H = Mat<3,4>::Constant(std::nan(""));
    return Vect<3>::Constant(std::nan(""));
  }
  double w = q[0];
  Vect<3> v = q.segment<3>(1);
  double nv = std::sqrt(v.dot(v));
  return 2 * v * std::atan2(nv, w)/ nv;
}
//  if (q[0] > 1) q1.normalise(); // if w>1 acos and sqrt will produce errors, this cant happen if quaternion is normalised
//  angle = 2 * Math.acos(q1.w);
//  double s = Math.sqrt(1-q1.w*q1.w); // assuming quaternion normalised then w is less than 1, so term always positive.
//  if (s < 0.001) { // test to avoid divide by zero, s is always positive due to sqrt
//    // if s close to zero then direction of axis not important
//    x = q1.x; // if it is important that axis is normalised then replace with x=1; y=z=0;
//    y = q1.y;
//    z = q1.z;
//  } else {
//    x = q1.x / s; // normalise axis
//    y = q1.y / s;
//    z = q1.z / s;
//  }
