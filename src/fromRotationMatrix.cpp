#include <fromRotationMatrix.hpp>

#include <detail/trigonometric_derivatives.hpp>
#include <detail/so3_generator.hpp>
#include <detail/skewSymmetric.hpp>

using namespace detail;

double trace(Eigen::Matrix3d const& R, Eigen::Ref<Eigen::Matrix<double, 1, 9>> H)
{
  H << 1,0,0,0,1,0,0,0,1;
  return R.trace();
}

Eigen::Vector3d errorVector(Eigen::Matrix3d const& R)
{
  return vectorFromSkewSymmetric(R - R.transpose());
}

Eigen::Vector3d errorVector(Eigen::Matrix3d const& R, Eigen::Ref<Eigen::Matrix<double, 3, 9>> H)
{
  H = Eigen::Matrix<double, 3, 9>::Random();
  H << generator<Axis::x>, generator<Axis::y>, generator<Axis::z>;
  return errorVector(R);
}

Eigen::Vector3d angleAxisFromRotationMatrix(Eigen::Matrix3d const& R)
{
  const auto angle = std::acos((R.trace() - 1)/2);
  return vectorFromSkewSymmetric(R - R.transpose()) * angle / (2*std::sin(angle));
}

Eigen::Vector3d angleAxisFromRotationMatrix(
    Eigen::Matrix3d const& R,
    Eigen::Ref<Eigen::Matrix<double, 3, 9>> H)
{
  Eigen::Matrix<double, 1, 9> trHR;
  const auto tr = trace(R, trHR);
  const auto arg = (tr - 1)/2;
  Eigen::Matrix<double, 1 , 9> argHR = 0.5 * trHR;
  double angleHarg;
  auto angle = detail::acos(arg, angleHarg);
  Eigen::Matrix<double, 1, 9> angleHR = angleHarg * argHR;
  Eigen::Matrix<double, 3, 9> evHR;
  Eigen::Vector3d ev = errorVector(R, evHR);
  double k = angle / (2*std::sin(angle));
  double kHangle =  - 0.5 * (angle * std::cos(angle) / std::sin(angle) - 1) / std::sin(angle);
  H = ev * kHangle * angleHR + evHR * k;
  return ev * k;
}

Eigen::Vector4d quaternionFromRotationMatrix(Eigen::Matrix3d const& R)
{
  const auto qw = std::sqrt(R.trace() + 1.0)/2.0;
  const Eigen::Vector3d vec = vectorFromSkewSymmetric(R - R.transpose()) / (4*qw);
  return (Eigen::Vector4d() << qw, vec).finished();
}

Eigen::Vector4d quaternionFromRotationMatrix(
    Eigen::Matrix3d const& R,
    Eigen::Ref<Eigen::Matrix<double, 4, 9>> H)
{
  Eigen::Matrix<double, 1, 9> trHR;
  const auto tr = trace(R, trHR);
  const auto qw = std::sqrt(tr + 1)/2;
  Eigen::Matrix<double, 1 , 9> qwHR = 0.25/std::sqrt(tr+1) * trHR;

  Eigen::Matrix<double, 3, 9> evHR;
  Eigen::Vector3d ev = errorVector(R, evHR);
  const Eigen::Vector3d vec = ev/(4*qw);

  H.block<1,9>(0,0) = qwHR;
  H.block<3,9>(1,0) = evHR / (4*qw) - ev / (4*qw*qw) * qwHR;

  return (Eigen::Vector4d() << qw, vec).finished();
}
