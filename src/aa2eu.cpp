#define CATCH_CONFIG_MAIN

#include <gtsam/geometry/Rot3.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/base/OptionalJacobian.h>
#include <iostream>
#include <catch/catch2.hpp>

#include <common.hpp>
#include <rodrigues.hpp>
#include <cmath>

// Doc : https://www.geometrictools.com/Documentation/EulerAngles.pdf
// Idea : use rodrigues formula for calculate derivative of rotation matrix wrt. aa
using namespace gtsam;

#define wrap(name) [](auto&&... args){ return name(std::forward<decltype(args)>(args) ...);}

double _atan2(
  double y,
  double x, 
  OptionalJacobian<1,1> Hy = boost::none,
  OptionalJacobian<1,1> Hx = boost::none)
{
  if(Hy)
    Hy->operator()(0,0) = x / (x*x + y*y);
  if(Hx)
    Hx->operator()(0,0) = - y / (x*x + y*y);

  return std::atan2(y,x);
}

Matrix3 Rx(double x, OptionalJacobian<9,1> H = boost::none)
{
  double s = std::sin(x), c = std::cos(x);
  if(H)
    *H << 0,0,0,0,-s,c,0,-c,-s;
  return Rot3::Rx(x).matrix();
}

Matrix3 Ry(double y, OptionalJacobian<9,1> H = boost::none)
{
  double s = std::sin(y), c = std::cos(y);
  if(H)
    *H << -s,0,-c,0,0,0,c,0,-s;
  return Rot3::Ry(y).matrix();
}

auto aa2eu(Vector3 const& aa, OptionalJacobian<3,3> H = boost::none)
{
  Mat<9,3> AJaa;
  const auto A = rodrigues(aa, AJaa);
  // A = Rz * Ry * Rx
  Mat<1,1> xJA21, xJA22;
  const auto x = - _atan2( - A(2,1), A(2,2), xJA21, xJA22);
  xJA22.array() *= -1;
  const auto xJaa = xJA21 * AJaa.block<1,3>(5,0) + xJA22 * AJaa.block<1,3>(8,0);

  Mat<9,1> QxJx;
  Matrix3 Qx = Rx(-x, QxJx);
  QxJx.array() *= -1;
  Mat<9,3> QxJaa = QxJx * xJaa;

  // B = Rz * Ry * Rx * Rx^-1
  const auto B = A * Qx;
  Mat<9,3> BJaa;
  Eigen::Map<Matrix3>(BJaa.block<9,1>(0,0).data(), 3,3) = 
      Eigen::Map<Matrix3>(AJaa.block<9,1>(0,0).data(), 3,3) * Qx
    + A * Eigen::Map<Matrix3>(QxJaa.block<9,1>(0,0).data(), 3,3);

  Eigen::Map<Matrix3>(BJaa.block<9,1>(0,1).data(), 3,3) = 
      Eigen::Map<Matrix3>(AJaa.block<9,1>(0,1).data(), 3,3) * Qx
    + A * Eigen::Map<Matrix3>(QxJaa.block<9,1>(0,1).data(), 3,3);

  Eigen::Map<Matrix3>(BJaa.block<9,1>(0,2).data(), 3,3) = 
      Eigen::Map<Matrix3>(AJaa.block<9,1>(0,2).data(), 3,3) * Qx
    + A * Eigen::Map<Matrix3>(QxJaa.block<9,1>(0,2).data(), 3,3);

  Mat<1,1> yJB20, yJB22;
  const auto y = - _atan2( B(2,0), B(2,2), yJB20, yJB22);
  yJB20.array() *= -1;
  yJB22.array() *= -1;
  const auto yJaa = yJB20 * BJaa.block<1,3>(2,0) + yJB22 * BJaa.block<1,3>(8,0);

  Mat<9,1> QyJy;
  Matrix3 Qy = Ry(-y, QyJy);
  QyJy.array() *= -1;
  Mat<9,3> QyJaa = QyJy * yJaa;

  const auto C = B * Qy.matrix();
  Mat<9,3> CJaa;
  Eigen::Map<Matrix3>(CJaa.block<9,1>(0,0).data(), 3,3) = 
      Eigen::Map<Matrix3>(BJaa.block<9,1>(0,0).data(), 3,3) * Qy
    + B * Eigen::Map<Matrix3>(QyJaa.block<9,1>(0,0).data(), 3,3);

  Eigen::Map<Matrix3>(CJaa.block<9,1>(0,1).data(), 3,3) = 
      Eigen::Map<Matrix3>(BJaa.block<9,1>(0,1).data(), 3,3) * Qy
    + B * Eigen::Map<Matrix3>(QyJaa.block<9,1>(0,1).data(), 3,3);

  Eigen::Map<Matrix3>(CJaa.block<9,1>(0,2).data(), 3,3) = 
      Eigen::Map<Matrix3>(BJaa.block<9,1>(0,2).data(), 3,3) * Qy
    + B * Eigen::Map<Matrix3>(QyJaa.block<9,1>(0,2).data(), 3,3);


  Mat<1,1> zJC10, zJC11;
  const auto z = - _atan2( - C(1,0), C(1,1), zJC10, zJC11);
  zJC11.array() *= -1;
  const auto zJaa = zJC10 * CJaa.block<1,3>(1,0) + zJC11 * CJaa.block<1,3>(4,0);
  
  if(H){
    H->block<1,3>(0,0) = xJaa;
    H->block<1,3>(1,0) = yJaa;
    H->block<1,3>(2,0) = zJaa;
  }

  return (Vector3() << x,y,z).finished();
}

template<typename E>
Eigen::Map<Eigen::RowVectorXd> RowVect(E e, int s, int off = 0)
{
  return {e.data() + off, s};
}

TEST_CASE("rodrigues")
{ 
  Vector3 aa;
  SECTION("Random"){
    aa = Vector3::Random();
  }
  SECTION("Zero"){
    aa = Vector3::Zero();
  }

  const auto calc = rodrigues(aa);
  const auto act = Rot3::Expmap(aa).matrix();
  CHECK( calc.isApprox(act) );
  std::cout << act << "\n\n";
  std::cout << calc << "\n\n";
}

TEST_CASE("rodriguesDerivative")
{ 
  Vector3 aa;
  SECTION("Random"){
    aa = Vector3::Random();
  }
  SECTION("Zero"){
    aa = Vector3::Zero();
  }

  Mat<9,3> H = Mat<9,3>::Zero();
  rodrigues(aa, H);
  auto numeric = numericalDerivative11<Matrix3, Vector3>(wrap(rodrigues), aa);
  CHECK( H.isApprox(numeric, 1e-10) );
  std::cout << numeric << "\n\n";
  std::cout << H << "\n\n";
}

TEST_CASE("atan2")
{
  Vector2 xy;
  SECTION("1")
  {
    xy << 0, -0.1;
  }
  SECTION("2")
  {
    xy << 0, -0.5;
  }
  SECTION("3")
  {
    xy << 0, -1;
  }
  SECTION("4")
  {
    xy << 0, -2;
  }

//  SECTION("Random"){
//    xy = Vector2::Random().array() + 1;
//  }
//  SECTION("NegaitveRandom"){
//    xy = Vector2::Random().array() - 1;
//  }
//  SECTION("ZeroXPosY"){
//    xy = Vector2::Random().array() + 1;
//    xy[0] = 0;
//  }
//  SECTION("ZeroXNegY"){
//    xy = Vector2::Random().array() - 1;
//    xy[0] = 0;
//  }
//  SECTION("ZeroYPosX"){
//    xy = Vector2::Random().array() + 1;
//    xy[1] = 0;
//  }
//  SECTION("ZeroYNegX"){
//    xy = Vector2::Random().array() - 1;
//    xy[1] = 0;
//  }

  std::cout << RowVect(xy,2) << "\n";
  Mat<1,1> numX = numericalDerivative21<double, double, double>(wrap(_atan2), xy[0], xy[1]);
  Mat<1,1> calcX;
  _atan2(xy[0], xy[1], calcX);

  Mat<1,1> numY = numericalDerivative22<double, double, double>(wrap(_atan2), xy[0], xy[1]);
  Mat<1,1> calcY;
  _atan2(xy[0], xy[1], boost::none, calcY);

  CHECK( calcX.isApprox(numX, 1e-9) );
  std::cout << calcX << " <--> " << numX << "\n";
  CHECK( calcY.isApprox(numY, 1e-9) );
  std::cout << calcY << " <--> " << numY << "\n";
}

TEST_CASE("Rx")
{ 
  Vector1 t;
  SECTION("Random"){
    t = Vector1::Random();
  }
  SECTION("RandomPos"){
    t = Vector1::Random().array() + 1;
  }
  SECTION("RandomNeg"){
    t = Vector1::Random().array() - 1;
  }
  SECTION("Zero"){
    t = Vector1::Zero();
  }

  Mat<9,1> calc = Mat<9,1>::Zero();
  Rx(t[0], calc);
  auto num = numericalDerivative11<Matrix3, double>(wrap(Rx), t[0]);
  CHECK( calc.isApprox(num,1e-10) );
}

TEST_CASE("Ry")
{ 
  Vector1 t;
  SECTION("Random"){
    t = Vector1::Random();
  }
  SECTION("RandomPos"){
    t = Vector1::Random().array() + 1;
  }
  SECTION("RandomNeg"){
    t = Vector1::Random().array() - 1;
  }
  SECTION("Zero"){
    t = Vector1::Zero();
  }

  Mat<9,1> calc = Mat<9,1>::Zero();
  Ry(t[0], calc);
  auto num = numericalDerivative11<Matrix3, double>(wrap(Ry), t[0]);
  CHECK( calc.isApprox(num,1e-10) );
}

TEST_CASE("aa2eu")
{ 
  Vector3 aa;
  SECTION("Random"){
    aa = Vector3::Random();
  }
  SECTION("RandomPos"){
    aa = Vector3::Random().array() + 1;
  }
  SECTION("RandomNeg"){
    aa = Vector3::Random().array() - 1;
  }
  SECTION("Zero"){
    aa = Vector3::Zero();
  }
  SECTION("Yaw"){
    aa << 0,1,0;
  }

  Matrix3 R = Rot3::Expmap(aa).matrix();
  auto act = RQ(R).second;
  auto calc = aa2eu(aa);
  CHECK( calc.isApprox(act) );
  std::cout << RowVect(aa,3) << "\n";
  std::cout << RowVect(calc,3) << "\n";
}


TEST_CASE("aa2euDerivative")
{ 
  Vector3 aa;
  SECTION("Random"){
    aa = Vector3::Random();
  }
  SECTION("RandomPos"){
    aa = Vector3::Random().array() + 1;
  }
  SECTION("RandomNeg"){
    aa = Vector3::Random().array() - 1;
  }
  SECTION("Zero"){
    aa = Vector3::Zero();
  }

  Matrix3 H = Matrix3::Zero();
  aa2eu(aa, H);
  auto numeric = numericalDerivative11<Vector3, Vector3>(wrap(aa2eu), aa);

  CHECK( H.isApprox(numeric,1e-10) );
  std::cout << numeric << "\n\n";
  std::cout << H << "\n\n";
  std::cout << (H - numeric) << "\n\n";
}
