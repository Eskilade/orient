#include <fromRotationMatrix.hpp>
#include <common.hpp>

using namespace gtsam;

Vect<3> angleAxisFromRotationMatrix(Mat<3,3> const& R)
{
  const auto angle = std::acos((R.trace() - 1)/2);
  return vectorFromSkewSymmetric(R - R.transpose()) * angle / (2*std::sin(angle));
}

Vect<4> quaternionFromRotationMatrix(Mat<3,3> const& R)
{
  const auto qw = std::sqrt(R.trace() + 1.0)/2.0;
  const Vect<3> vec = vectorFromSkewSymmetric(R - R.transpose()) / (4*qw);
  return (Vect<4>() << qw, vec).finished();
}
