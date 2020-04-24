#include <orient/from_rotation_matrix.hpp>

#include <orient/detail/skew_symmetric.hpp>
#include <orient/detail/so3_generator.hpp>
#include <orient/detail/trigonometric_derivatives.hpp>
#include <orient/detail/kronecker_product.hpp>

template<class T>
typename std::enable_if<not std::numeric_limits<T>::is_integer, bool>::type
    almost_equal(T x, T y, int ulp)
{
    // the machine epsilon has to be scaled to the magnitude of the values used
    // and multiplied by the desired precision in ULPs (units in the last place)
    return std::fabs(x-y) <= std::numeric_limits<T>::epsilon() * std::fabs(x+y) * ulp
        // unless the result is subnormal
        || std::fabs(x-y) < std::numeric_limits<T>::min();
}

double trace(Eigen::Matrix3d const& R, Eigen::Ref<Eigen::Matrix<double, 1, 9>> H)
{
  H << 1,0,0,0,1,0,0,0,1;
  return R.trace();
}

Eigen::Matrix<double, 9, 9> transposePD()
{
  Eigen::Matrix<double, 9,9> H = Eigen::Matrix<double, 9,9>::Identity();
  H.col(1).swap(H.col(3));
  H.col(2).swap(H.col(6));
  H.col(5).swap(H.col(7));
  return H;
}

std::tuple<double, Eigen::Matrix<double, 1, 9>> traceWPD(Eigen::Matrix3d const& R)
{
  Eigen::Matrix<double, 1, 9> H;
  H << 1,0,0,0,1,0,0,0,1;
  return std::make_tuple(R.trace(), H);
}

Eigen::Vector3d errorVector(Eigen::Matrix3d const& R)
{
  return orient::detail::unskewSymmetric(R - R.transpose());
}

std::tuple<Eigen::Vector3d, Eigen::Matrix<double, 3, 9>>
errorVectorWPD(Eigen::Matrix3d const& R)
{
  Eigen::Matrix<double, 3, 9> H;
  H << orient::detail::generator<orient::Axis::x>, orient::detail::generator<orient::Axis::y>, orient::detail::generator<orient::Axis::z>;
  return std::make_tuple(errorVector(R), H);
}

namespace orient {

Eigen::Vector3d angleAxisFromRotationMatrix(Eigen::Matrix3d const& R)
{
  // trace = 2*cos(angle) + 1
  const auto trace = R.trace();
  if( almost_equal(trace, 3.0, 10) ){
    // angle is close to 2*k*pi
    // Should be minus I but the diagonal does not change anything
    return orient::detail::unskewSymmetric(R);
  } else if( almost_equal(trace, -1.0, 10) ){
    // Derivative is possibly not defined here. There are 2 solutions, as
    // Theta and minus Theta would produce the same result ?
    // soluation possible
    // angle is close to (2*k + 1)*pi
    const Eigen::Matrix3d uut = (R + Eigen::Matrix3d::Identity()) / 2;
    int max_i;
    const auto ui = std::sqrt(uut.diagonal().maxCoeff(&max_i));
    return M_PI * uut.block<3,1>(0, max_i) / ui;
  } else {
    const auto arg = (trace - 1)/2; 
    const auto angle = std::acos(arg);
    return detail::unskewSymmetric(R - R.transpose()) * angle / (2*std::sin(angle));
  }
}

Eigen::Vector3d angleAxisFromRotationMatrix(
    Eigen::Matrix3d const& R,
    Eigen::Ref<Eigen::Matrix<double, 3, 9>> H)
{
  const auto [tr, trHR] = traceWPD(R);
  if( almost_equal(tr, 3.0, 10) ){
    const auto [ev, evHR] = detail::unskewSymmetricWPD(R);
    H = evHR;
    return ev;
  }
  const auto x = 0.5*(tr - 1.0);

  const auto [angle, angleHx] = detail::acosWD(x);

  const auto sin = std::sin(angle);

  const auto k3 = 0.5*angle/sin;
  const auto k3Hangle = 0.5*(1. - angle/std::tan(angle))/sin;

  const Eigen::Matrix3d eR = R - R.transpose();
  const Eigen::Matrix<double, 9, 9> eRHR = Eigen::Matrix<double, 9, 9>::Identity() - transposePD();

  const auto [ev, evHeR] = detail::unskewSymmetricWPD(eR);

  const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
  Eigen::Map<const Eigen::Matrix<double, 9, 1>> vecI{I.data(), I.size()};

  H = detail::kroneckerProduct(ev.transpose(), I) * vecI *k3Hangle * angleHx * trHR / 2.
    + k3 * evHeR * eRHR;
   return k3 * ev;
}

Eigen::Vector4d quaternionFromRotationMatrix(Eigen::Matrix3d const& R)
{
  // tr(R) = 3*w^2 - x^2 - y^2 - z^2
  //       = 4*w^2 - w^2 - x^2 - y^2 - z^2
  //       = 4*w^2 - 1 
  const auto tr = R.trace();
  if( almost_equal(tr, -1.0, 10) ){
    // w is close to 0
    const Eigen::Matrix3d vvt = (R + Eigen::Matrix3d::Identity()) / 2;
    int max_i;
    const auto vi = std::sqrt(vvt.diagonal().maxCoeff(&max_i));
    return (Eigen::Vector4d() << 0, vvt.block<3,1>(0, max_i) / vi).finished();
  }
  const auto w = 0.5*std::sqrt(tr + 1.0);
  const Eigen::Vector3d v = 0.25*detail::unskewSymmetric(R - R.transpose()) / w;
  return (Eigen::Vector4d() << w, v).finished();
}

Eigen::Vector4d quaternionFromRotationMatrix(
    Eigen::Matrix3d const& R,
    Eigen::Ref<Eigen::Matrix<double, 4, 9>> H)
{
  const auto [tr, trHR] = traceWPD(R);

  const auto w = 0.5*std::sqrt(tr + 1.0);
  Eigen::Matrix<double, 1 , 9> wHR = 0.25/std::sqrt(tr+1) * trHR;

  const auto k4 = 0.25/w;
  const auto k4Hw = - k4/w;
 
  const Eigen::Matrix3d eR = R - R.transpose();
  const Eigen::Matrix<double, 9, 9> eRHR = Eigen::Matrix<double, 9, 9>::Identity() - transposePD();

  const auto [ev, evHeR] = detail::unskewSymmetricWPD(eR);

  const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
  Eigen::Map<const Eigen::Matrix<double, 9, 1>> vecI{I.data(), I.size()};

  const Eigen::Vector3d v = 0.25*detail::unskewSymmetric(R - R.transpose()) / w;
  Eigen::Matrix<double, 3 , 9> vHR = detail::kroneckerProduct(ev.transpose(), I) * vecI *k4Hw * wHR 
    + k4 * evHeR * eRHR;

  H.block<1,9>(0,0) = wHR;
  H.block<3,9>(1,0) = vHR;

  return (Eigen::Vector4d() << w, v).finished();
}

}
