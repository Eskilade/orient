#pragma once

#include <Eigen/Dense>
#include <gtsam/geometry/Rot3.h>

#include <axis.hpp>

inline Eigen::Matrix3d skew(double wx, double wy, double wz) {
  return (Eigen::Matrix3d() << 0.0, -wz, +wy, +wz, 0.0, -wx, -wy, +wx, 0.0).finished();
}

template <class Derived>
inline Eigen::Matrix3d skew(const Eigen::MatrixBase<Derived>& w) {
  return skew(w(0), w(1), w(2));
}

template <class Derived>
inline Eigen::Vector3d vectorFromSkewSymmetric(const Eigen::MatrixBase<Derived>& M)
{
  return (Eigen::Vector3d() << - M(1,2), M(0,2), - M(0,1)).finished();
}

struct Elem 
{
  int r, c;
};

template<typename Mat>
class Wrap
{
public:
  Wrap(Mat const& _m):m{_m}{};
  auto operator()(Elem const& e) const
  {
    return sign(e) * uncorrected(e);
  }
  auto uncorrected(Elem const& e) const
  {
    return m(e.r, e.c);
  }
  auto sign(Elem const& e) const
  {
    return sign_mat(e.r, e.c);
  }

private:
  gtsam::Matrix3 sign_mat = 
    gtsam::Matrix3::Identity() 
    + skew(gtsam::Vector3::Ones());
  Mat const& m;
};

template<Axis>
struct Rot;

#define MakeRotMat(name) \
  template<> struct Rot<Axis::name> { \
    static gtsam::Matrix3 Mat(double a){ return gtsam::Rot3::R##name(a).matrix();}; \
  }

MakeRotMat(x);
MakeRotMat(y);
MakeRotMat(z);

template<Axis a1, Axis a2, Axis a3>
Eigen::Matrix3d toR(Eigen::Vector3d const& v)
{
  return Rot<a1>::Mat(v[0]) * Rot<a2>::Mat(v[1]) * Rot<a3>::Mat(v[2]);
}

#include <normalize.hpp>
#include <detail/so3_generator.hpp>
