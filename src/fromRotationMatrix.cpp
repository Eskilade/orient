#include <fromRotationMatrix.hpp>
#include <common.hpp>

using namespace gtsam;

Eigen::Vector3d angleAxisFromRotationMatrix(Mat<3,3> const& R)
{
  const auto angle = std::acos((R.trace() - 1)/2);
  return vectorFromSkewSymmetric(R - R.transpose()) * angle / (2*std::sin(angle));
}

Eigen::Vector4d quaternionFromRotationMatrix(Mat<3,3> const& R)
{
  const auto qw = std::sqrt(R.trace() + 1.0)/2.0;
  const Eigen::Vector3d vec = vectorFromSkewSymmetric(R - R.transpose()) / (4*qw);
  return (Eigen::Vector4d() << qw, vec).finished();
}
