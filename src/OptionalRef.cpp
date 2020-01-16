#define CATCH_CONFIG_MAIN

#include <iostream>
#include <catch/catch2.hpp>
#include <cmath>
#include <Eigen/Dense>
#include <memory>
#include <tuple>

template<int A, int B>
using Mat = Eigen::Matrix<float, A, B>;

template<typename... T>
class OptionalRef
{
public:
  using RefT = Eigen::Ref<T...>;

  OptionalRef():
    m_ptr{},
    m_valid{false}{};


  OptionalRef(RefT ref):
    m_ptr{std::make_shared<RefT>(ref)},
    m_valid{true}{};

  template<typename D>
  OptionalRef(Eigen::PlainObjectBase<D>& mat):
    m_ptr{std::make_shared<RefT>(mat)},
    m_valid{true}{};

  operator bool(){return m_valid;}

  template<int A, int B>
  OptionalRef<Mat<A,B>> blockIfOk(int f, int t)
  {
    if(m_valid)
      return {m_ptr->template block<A,B>(f,t)};
    else
      return {};
  }

  OptionalRef<Mat<1,1>> ifOk(int f, int t)
  {
    return this->blockIfOk<1,1>(f,t);
  }


  RefT& operator*(){ return *m_ptr;}
  RefT* operator->(){ return m_ptr.get();}

private:
  std::shared_ptr<RefT> m_ptr;
  bool m_valid;
};

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
