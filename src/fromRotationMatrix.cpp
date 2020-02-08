#include <fromRotationMatrix.hpp>
#include <common.hpp>

using namespace gtsam;

Vect<3> angleAxisFromRotationMatrix(Mat<3,3> const& R)
{
  const auto angle = std::acos((R.trace() - 1)/2);
  return vectorFromSkewSymmetric(R - R.transpose()) * angle / (2*std::sin(angle));
}
