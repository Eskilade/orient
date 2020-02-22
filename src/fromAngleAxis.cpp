#include <detail/trigonometric_derivatives.hpp>
#include <detail/so3_generator.hpp>
#include <detail/skewSymmetric.hpp>

#include <axis.hpp>
#include <fromAngleAxis.hpp>
#include <normalize.hpp>

template<Axis axis>
Eigen::Matrix3d rodriguesDerivative(Eigen::Vector3d const& aa)
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

Eigen::Matrix3d rodrigues(Eigen::Vector3d const& aa)
{
  const auto t2 = aa.dot(aa);
  if (t2 < std::numeric_limits<double>::epsilon()) {
    return Eigen::Matrix3d::Identity();
  }
  else {
    const auto t = std::sqrt(t2);
    return 
      Eigen::Matrix3d::Identity() +
      skewSymmetric(aa) * std::sin(t) / t +
      skewSymmetric(aa) * skewSymmetric(aa) * (1 - std::cos(t)) / t2; 
  }
}

Eigen::Matrix3d rotationMatrixFromAngleAxis(Eigen::Vector3d const& aa)
{
  return rodrigues(aa);
}

Eigen::Matrix3d rotationMatrixFromAngleAxis(Eigen::Vector3d const& aa, Eigen::Ref<Eigen::Matrix<double, 9, 3>> H)
{
  Eigen::Map<Eigen::Matrix3d>(H.block<9,1>(0,0).data(), 3,3) = rodriguesDerivative<Axis::x>(aa);
  Eigen::Map<Eigen::Matrix3d>(H.block<9,1>(0,1).data(), 3,3) = rodriguesDerivative<Axis::y>(aa);
  Eigen::Map<Eigen::Matrix3d>(H.block<9,1>(0,2).data(), 3,3) = rodriguesDerivative<Axis::z>(aa);
  return rotationMatrixFromAngleAxis(aa);
}

Eigen::Vector4d quaternionFromAngleAxis(Eigen::Vector3d const& aa)
{
  const auto angle = std::sqrt(aa.dot(aa));
  const Eigen::Vector3d axis = aa / angle; 
  const auto ha = angle/2;
  const auto qw = std::cos(ha);
  Eigen::Vector3d vec = std::sin(ha) * axis;
  return (Eigen::Vector4d() << qw, vec).finished();
}

Eigen::Vector4d quaternionFromAngleAxis(Eigen::Vector3d const& aa, Eigen::Ref<Eigen::Matrix<double, 4, 3>> H)
{
  const auto angle = std::sqrt(aa.dot(aa));
  Eigen::Matrix3d axisHaa;
  const Eigen::Vector3d axis = normalize(aa, axisHaa);
  const auto ha = angle/2;
  const auto cha = std::cos(ha);
  const auto sha = std::sin(ha);
  const auto& qw = cha;
  H.block<1,3>(0,0) = - sha * axis.transpose() / 2;
  Eigen::Vector3d vec = sha * axis;
  H.block<3,3>(1,0) = axis * axis.transpose() * cha / 2 + sha * axisHaa;
  return (Eigen::Vector4d() << qw, vec).finished();
}
