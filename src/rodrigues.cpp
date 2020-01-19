#include <common.hpp>
#include <rodrigues.hpp>
#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/base/Matrix.h>
#include <cmath>

using namespace gtsam;

template<Axis axis>
Matrix3 rodriguesDerivative(Vector3 const& aa)
{
  const auto t2 = aa.dot(aa);
  if (t2 < std::numeric_limits<double>::epsilon()) {
    return generator<axis>;
  }
  else {
    const auto t = std::sqrt(t2);
    const auto a = aa[static_cast<std::underlying_type_t<Axis>>(axis)];
    return 
      generator<axis> * std::sin(t) / t + 
      skewSymmetric(aa) * a * (t*std::cos(t) - std::sin(t)) / (t2*t) +
      generator<axis> * skewSymmetric(aa) * (1 - std::cos(t))/t2 + 
      skewSymmetric(aa) * generator<axis> * (1 - std::cos(t))/t2 + 
      skewSymmetric(aa) * skewSymmetric(aa) * a * (t*std::sin(t) + 2*std::cos(t) - 2) / (t2*t2); 
  }
}

Matrix3 rodrigues(Vector3 const& aa, OptionalJacobian<9,3> H)
{
  if(H){
    Eigen::Map<Matrix3>(H->block<9,1>(0,0).data(), 3,3) = rodriguesDerivative<Axis::x>(aa);
    Eigen::Map<Matrix3>(H->block<9,1>(0,1).data(), 3,3) = rodriguesDerivative<Axis::y>(aa);
    Eigen::Map<Matrix3>(H->block<9,1>(0,2).data(), 3,3) = rodriguesDerivative<Axis::z>(aa);
  }

  const auto t2 = aa.dot(aa);
  if (t2 < std::numeric_limits<double>::epsilon()) {
    return Matrix3::Identity();
  }
  else {
    const auto t = std::sqrt(t2);
    return 
      Matrix3::Identity() +
      skewSymmetric(aa) * std::sin(t) / t +
      skewSymmetric(aa) * skewSymmetric(aa) * (1 - std::cos(t)) / t2; 
  }
}
