#include <orient/from_angle_axis.hpp>

#include <orient/detail/skew_symmetric.hpp>
#include <orient/detail/kronecker_product.hpp>

namespace orient {

template<typename Scalar>
Eigen::Matrix<Scalar, 4, 1> quaternionFromAngleAxis(Eigen::Matrix<Scalar, 3, 1> const& aa)
{
  const auto angle2 = aa.dot(aa);
  if( angle2 < 1e-10){
    const auto w = 1 - angle2/8;
    const auto v = (0.5 - angle2/48)*aa;
    return (Eigen::Matrix<Scalar,4,1>() << w, v).finished();
  }
  const auto angle = std::sqrt(angle2);
  const auto ha = angle/2;
  const auto w = std::cos(ha);
  Eigen::Matrix<Scalar,3,1> v = std::sin(ha) * aa / angle;
  return (Eigen::Matrix<Scalar,4,1>() << w, v).finished();
}

template<typename Scalar>
std::pair<Eigen::Matrix<Scalar, 4, 1>, Eigen::Matrix<Scalar, 4, 3>> quaternionFromAngleAxisWD(Eigen::Matrix<Scalar,3, 1> const& aa)
{
  const auto angle2 = aa.dot(aa);
  if( angle2 < 1e-10){
    const auto w = 1 - angle2/8;
    const auto wJaa = -0.25 * aa.transpose();

    const auto k = (0.5 - angle2/48.);
    const auto kJaa = - (1./24.) *  aa.transpose();

    const auto v = k*aa;
    const auto vJaa = aa * kJaa + k * Eigen::Matrix<Scalar,3,3>::Identity();

    Eigen::Matrix<Scalar, 4, 3> J;
    J.template block<1,3>(0,0) = wJaa;
    J.template block<3,3>(1,0) = vJaa;
    return std::make_pair((Eigen::Matrix<Scalar,4,1>() << w, v).finished(), J);
  }
  const auto angle = std::sqrt(angle2);
  const auto ha = angle/2.0;
  const auto cha = std::cos(ha);
  const auto sha = std::sin(ha);

  const auto w = cha;
  const auto wJaa = - sha * aa.transpose() / (2. * angle);

  const auto k = sha / angle;
  const Eigen::Matrix<Scalar, 1, 3> kJaa = (angle * cha - 2*sha) * aa.transpose() / (2*angle2*angle);
   
  const Eigen::Matrix<Scalar,3,1> v = k * aa;
  const Eigen::Matrix<Scalar,3,3> vJaa = aa * kJaa + k * Eigen::Matrix<Scalar,3,3>::Identity();

  Eigen::Matrix<Scalar, 4, 3> J;
  J.template block<1,3>(0,0) = wJaa;
  J.template block<3,3>(1,0) = vJaa;

  return std::make_pair((Eigen::Matrix<Scalar,4,1>() << w, v).finished(), J);
}

template<typename Scalar>
Eigen::Matrix<Scalar, 3, 3> rotationMatrixFromAngleAxis(Eigen::Matrix<Scalar,3, 1> const& aa)
{
  const auto angle2 = aa.dot(aa);
  if (angle2 < std::numeric_limits<Scalar>::epsilon()) {
    return Eigen::Matrix<Scalar,3,3>::Identity();
  }
  const auto angle = std::sqrt(angle2);
  const Eigen::Matrix<Scalar,3,3> skew = orient::detail::skewSymmetric(aa);
  return Eigen::Matrix<Scalar,3,3>::Identity() + skew * std::sin(angle) / angle + skew * skew * (1 - std::cos(angle)) / angle2; 
}

template<typename Scalar>
std::pair<Eigen::Matrix<Scalar, 3, 3>, Eigen::Matrix<Scalar, 9, 3>> rotationMatrixFromAngleAxisWD(Eigen::Matrix<Scalar, 3, 1> const& aa)
{
  const auto [skew, skewJaa] = detail::skewSymmetricWPD(aa);
  const auto angle2 = aa.dot(aa);
  if (angle2 < std::numeric_limits<Scalar>::epsilon()) {
    return std::make_pair(Eigen::Matrix<Scalar,3,3>::Identity(), skewJaa);
  }
  const auto angle = std::sqrt(angle2);
  const auto s = std::sin(angle);
  const auto c = std::cos(angle);
  const Eigen::Matrix<Scalar,3,3> I = Eigen::Matrix<Scalar,3,3>::Identity();
  Eigen::Map<const Eigen::Matrix<Scalar, 9, 1>> vecI{I.data(), I.size()};

  const auto k1 = s / angle;
  const Eigen::Matrix<Scalar, 1, 3> k1Jaa = (angle*c - s) * aa.transpose() / (angle2*angle);

  const auto k2 = (1. - c) / angle2;
  const Eigen::Matrix<Scalar, 1, 3> k2Jaa = (angle*s - 2.*(1.-c)) * aa.transpose() / (angle2*angle2);

  const Eigen::Matrix<Scalar,3,3> term1 = k1 * skew;
  const Eigen::Matrix<Scalar, 9, 3> term1Jaa = k1 * skewJaa - detail::kroneckerProduct(skew, I)*vecI*k1Jaa;

  const Eigen::Matrix<Scalar,3,3> skew2 = skew*skew;
  const Eigen::Matrix<Scalar, 9, 3> skew2Jaa = (detail::kroneckerProduct(I, skew) + Eigen::kroneckerProduct(skew.transpose(), I)) * skewJaa;

  const Eigen::Matrix<Scalar,3,3> term2 = k2 * skew2;
  const Eigen::Matrix<Scalar, 9, 3> term2Jaa = k2 * skew2Jaa + detail::kroneckerProduct(skew2, I)*vecI*k2Jaa;

  return std::make_pair(I + term1 + term2, term1Jaa + term2Jaa);
}

}
