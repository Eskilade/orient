#include <orient/from_angle_axis.hpp>

#include <orient/detail/skew_symmetric.hpp>
#include <orient/detail/trigonometric_derivatives.hpp>
#include <orient/detail/kronecker_product.hpp>

namespace orient {

Eigen::Matrix3d rotationMatrixFromAngleAxis(Eigen::Vector3d const& aa)
{
  const auto angle2 = aa.dot(aa);
  if (angle2 < std::numeric_limits<double>::epsilon()) {
    return Eigen::Matrix3d::Identity();
  }
  const auto angle = std::sqrt(angle2);
  const Eigen::Matrix3d skew = orient::detail::skewSymmetric(aa);
  return Eigen::Matrix3d::Identity() + skew * std::sin(angle) / angle + skew * skew * (1 - std::cos(angle)) / angle2; 
}

Eigen::Matrix3d rotationMatrixFromAngleAxis(Eigen::Vector3d const& aa, Eigen::Ref<Eigen::Matrix<double, 9, 3>> H)
{
  const auto [skew, skewHaa] = detail::skewSymmetricWPD(aa);
  const auto angle2 = aa.dot(aa);
  if (angle2 < std::numeric_limits<double>::epsilon()) {
    H = skewHaa;
    return Eigen::Matrix3d::Identity();
  }
  const auto angle = std::sqrt(angle2);
  const auto s = std::sin(angle);
  const auto c = std::cos(angle);
  const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
  Eigen::Map<const Eigen::Matrix<double, 9, 1>> vecI{I.data(), I.size()};

  const auto k1 = s / angle;
  const Eigen::Matrix<double, 1, 3> k1Haa = (angle*c - s) * aa.transpose() / (angle2*angle);

  const auto k2 = (1. - c) / angle2;
  const Eigen::Matrix<double, 1, 3> k2Haa = (angle*s - 2.*(1.-c)) * aa.transpose() / (angle2*angle2);

  const Eigen::Matrix3d term1 = k1 * skew;
  const Eigen::Matrix<double, 9, 3> term1Haa = k1 * skewHaa - detail::kroneckerProduct(skew, I)*vecI*k1Haa;

  const Eigen::Matrix3d skew2 = skew*skew;
  const Eigen::Matrix<double, 9, 3> skew2Haa = (detail::kroneckerProduct(I, skew) + Eigen::kroneckerProduct(skew.transpose(), I)) * skewHaa;

  const Eigen::Matrix3d term2 = k2 * skew2;
  const Eigen::Matrix<double, 9, 3> term2Haa = k2 * skew2Haa + detail::kroneckerProduct(skew2, I)*vecI*k2Haa;

  H = term1Haa + term2Haa;
  return I + term1 + term2;
}

Eigen::Vector4d quaternionFromAngleAxis(Eigen::Vector3d const& aa)
{
  const auto angle2 = aa.dot(aa);
  if( angle2 < 1e-10){
    const auto w = 1 - angle2/8;
    const auto v = (0.5 - angle2/48)*aa;
    return (Eigen::Vector4d() << w, v).finished();
  }
  const auto angle = std::sqrt(angle2);
  const auto ha = angle/2;
  const auto w = std::cos(ha);
  Eigen::Vector3d v = std::sin(ha) * aa / angle;
  return (Eigen::Vector4d() << w, v).finished();
}

Eigen::Vector4d quaternionFromAngleAxis(Eigen::Vector3d const& aa, Eigen::Ref<Eigen::Matrix<double, 4, 3>> H)
{
  const auto angle2 = aa.dot(aa);
  if( angle2 < 1e-10){
    const auto w = 1 - angle2/8;
    const auto wHaa = -0.25 * aa.transpose();

    const auto k = (0.5 - angle2/48.);
    const auto kHaa = - (1./24.) *  aa.transpose();

    const auto v = k*aa;
    const auto vHaa = aa * kHaa + k * Eigen::Matrix3d::Identity();
    H.block<1,3>(0,0) = wHaa;
    H.block<3,3>(1,0) = vHaa;
    return (Eigen::Vector4d() << w, v).finished();
  }
  const auto angle = std::sqrt(angle2);
  const auto ha = angle/2.0;
  const auto cha = std::cos(ha);
  const auto sha = std::sin(ha);

  const auto w = cha;
  const auto wHaa = - sha * aa.transpose() / (2. * angle);

  const auto k = sha / angle;
  const Eigen::Matrix<double, 1, 3> kHaa = (angle * cha - 2*sha) * aa.transpose() / (2*angle2*angle);
   
  const Eigen::Vector3d v = k * aa;
  const Eigen::Matrix3d vHaa = aa * kHaa + k * Eigen::Matrix3d::Identity();

  H.block<1,3>(0,0) = wHaa;
  H.block<3,3>(1,0) = vHaa;

  return (Eigen::Vector4d() << w, v).finished();
}

}
