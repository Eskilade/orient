#include <orient/from_rotation_matrix.hpp>

#include <orient/detail/skew_symmetric.hpp>
#include <orient/detail/so3_generator.hpp>
#include <orient/detail/trigonometric_derivatives.hpp>

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

std::tuple<double, Eigen::Matrix<double, 1, 9>> traceWPD(Eigen::Matrix3d const& R)
{
  Eigen::Matrix<double, 1, 9> H;
  H << 1,0,0,0,1,0,0,0,1;
  return std::make_tuple(R.trace(), H);
}

Eigen::Vector3d errorVector(Eigen::Matrix3d const& R)
{
  return orient::detail::vectorFromSkewSymmetric(R - R.transpose());
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
    return orient::detail::vectorFromSkewSymmetric(R);
  } else if( almost_equal(trace, -1.0, 10) ){
    // angle is close to (2*k + 1)*pi
    const Eigen::Matrix3d uut = (R + Eigen::Matrix3d::Identity()) / 2;
    int max_i;
    const auto ui = std::sqrt(uut.diagonal().maxCoeff(&max_i));
    return M_PI * uut.block<3,1>(0, max_i) / ui;
  } else {
    const auto arg = (trace - 1)/2; 
    const auto angle = arg + 1 > std::numeric_limits<double>::epsilon() ? std::acos(arg) : M_PI;
    return errorVector(R) * angle / (2*std::sin(angle));
  }
}

Eigen::Vector3d angleAxisFromRotationMatrix(
    Eigen::Matrix3d const& R,
    Eigen::Ref<Eigen::Matrix<double, 3, 9>> H)
{
  const auto[tr, trHR] = traceWPD(R);
  const auto arg = 0.5*(tr - 1.0);
  Eigen::Matrix<double, 1 , 9> argHR = 0.5 * trHR;
  double angleHarg;
  auto angle = detail::acos(arg, angleHarg);
  Eigen::Matrix<double, 1, 9> angleHR = angleHarg * argHR;
  const auto [ev, evHR] = errorVectorWPD(R);
  double k = angle / (2*std::sin(angle));
  double kHangle =  - 0.5 * (angle * std::cos(angle) / std::sin(angle) - 1) / std::sin(angle);
  H = ev * kHangle * angleHR + evHR * k;
  return ev * k;
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
  } else {
    const auto qw = std::sqrt(tr + 1.0)/2.0;
    const Eigen::Vector3d vec = errorVector(R) / (4*qw);
    return (Eigen::Vector4d() << qw, vec).finished();
  }
}

Eigen::Vector4d quaternionFromRotationMatrix(
    Eigen::Matrix3d const& R,
    Eigen::Ref<Eigen::Matrix<double, 4, 9>> H)
{
  const auto [tr, trHR] = traceWPD(R);
  const auto qw = std::sqrt(tr + 1)/2;
  Eigen::Matrix<double, 1 , 9> qwHR = 0.25/std::sqrt(tr+1) * trHR;

  const auto [ev, evHR] = errorVectorWPD(R);
  const Eigen::Vector3d vec = ev/(4*qw);

  H.block<1,9>(0,0) = qwHR;
  H.block<3,9>(1,0) = evHR / (4*qw) - ev / (4*qw*qw) * qwHR;

  return (Eigen::Vector4d() << qw, vec).finished();
}

}
