#include <fromAngleAxis.hpp>
#include <detail/trigonometric_derivatives.hpp>

template<Axis axis>
Mat<3,3> rodriguesDerivative(Eigen::Vector3d const& aa)
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
      skew(aa) * a * (t*std::cos(t) - std::sin(t)) / (t2*t) +
      generator<axis> * skew(aa) * (1 - std::cos(t))/t2 + 
      skew(aa) * generator<axis> * (1 - std::cos(t))/t2 + 
      skew(aa) * skew(aa) * a * (t*std::sin(t) + 2*std::cos(t) - 2) / (t2*t2); 
  }
}

Mat<3,3> rodrigues(Eigen::Vector3d const& aa)
{
  const auto t2 = aa.dot(aa);
  if (t2 < std::numeric_limits<double>::epsilon()) {
    return Mat<3,3>::Identity();
  }
  else {
    const auto t = std::sqrt(t2);
    return 
      Mat<3,3>::Identity() +
      skew(aa) * std::sin(t) / t +
      skew(aa) * skew(aa) * (1 - std::cos(t)) / t2; 
  }
}

Mat<3,3> rotationMatrixFromAngleAxis(Eigen::Vector3d const& aa)
{
  return rodrigues(aa);
}

Mat<3,3> rotationMatrixFromAngleAxis(Eigen::Vector3d const& aa, Eigen::Ref<Mat<9,3>> H)
{
  Eigen::Map<Mat<3,3>>(H.block<9,1>(0,0).data(), 3,3) = rodriguesDerivative<Axis::x>(aa);
  Eigen::Map<Mat<3,3>>(H.block<9,1>(0,1).data(), 3,3) = rodriguesDerivative<Axis::y>(aa);
  Eigen::Map<Mat<3,3>>(H.block<9,1>(0,2).data(), 3,3) = rodriguesDerivative<Axis::z>(aa);
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

Eigen::Vector4d quaternionFromAngleAxis(Eigen::Vector3d const& aa, Eigen::Ref<Mat<4,3>> H)
{
  const auto angle = std::sqrt(aa.dot(aa));
  Mat<3,3> axisHaa;
  const Eigen::Vector3d axis = normalize<3>(aa, axisHaa);
  const auto ha = angle/2;
  const auto cha = std::cos(ha);
  const auto sha = std::sin(ha);
  const auto& qw = cha;
  H.block<1,3>(0,0) = - sha * axis.transpose() / 2;
  Eigen::Vector3d vec = sha * axis;
  H.block<3,3>(1,0) = axis * axis.transpose() * cha / 2 + sha * axisHaa;
  return (Eigen::Vector4d() << qw, vec).finished();
}
