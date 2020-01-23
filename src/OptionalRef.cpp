#define CATCH_CONFIG_MAIN

#include <iostream>
#include <catch/catch2.hpp>
#include <cmath>
#include <OptionalRef.hpp>

void h(OptionalRef<Mat<1,1>> rm = {})
{
  if(rm)
    (*rm) << -21;
}
void g(OptionalRef<Mat<2,2>> rm = {})
{
  if(rm)
    *rm = Mat<2,2>::Constant(3);
  h(rm.ifOk(0,1));
}
void f(OptionalRef<Mat<3,3>> rm = {})
{
  g(rm.blockIfOk<2,2>(1,1));
//  if(rm){
//    (*rm)(0,0) = 10;
//  }
}

TEST_CASE("TEST")
{
  Mat<3,3> m = Mat<3,3>::Identity();
  f(m);
  std::cout << m <<"\n\n";
}
