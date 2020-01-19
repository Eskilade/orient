#pragma once 
#include <common.hpp>
#include <gtsam/base/OptionalJacobian.h>

template<Axis axis>
Mat<3,3> make_generator()
{
  Mat<3,3> M = Mat<3,3>::Zero();
  if constexpr(axis == Axis::x){
    M(1,2) = -1;
    M(2,1) = 1;
  }
  else if constexpr(axis == Axis::y){
    M(0,2) = 1;
    M(2,0) = -1;
  }
  else {
    M(0,1) = -1;
    M(1,0) = 1;
  }
  return M;
}

template<Axis axis>
static const auto generator = make_generator<axis>();

Mat<3,3> rodrigues(Vect<3> const& aa, gtsam::OptionalJacobian<9,3> H = boost::none);
