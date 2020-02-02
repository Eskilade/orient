#pragma once

#include <common.hpp>
#include <OptionalRef.hpp>

Mat<3,3> rotationMatrixFromQuaternion(Vect<4> const& q, OptionalRef<Mat<9,4>> H = {});

template<int N>
Vect<N> normalize(Vect<N> const& v, OptionalRef<Mat<N,N>> H = {})
{
  double n2 = v.dot(v);
  if( n2 < std::numeric_limits<double>::epsilon()){
    if(H)
      *H = Mat<N,N>::Constant(std::nan(""));
    return Vect<N>::Constant(std::nan(""));
  }
  double n = std::sqrt(n2);
  if(H){
    *H = - v * v.transpose() / ( n * n2) + Mat<N,N>::Identity() * 1.0 / n;
  }
  return v / n;
}

Vect<3> angleAxisFromQuaternion(Vect<4> const& q, OptionalRef<Mat<3,4>> H = {});
