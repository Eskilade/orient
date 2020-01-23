#pragma once 

#include <Eigen/Dense>
#include <memory>
#include <tuple>

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
  OptionalRef<Mat<A,B>> optionalBlock(int f, int t)
  {
    if(m_valid)
      return {m_ptr->template block<A,B>(f,t)};
    else
      return {};
  }

  OptionalRef<Mat<1,1>> optional(int f, int t)
  {
    return this->optionalBlock<1,1>(f,t);
  }


  RefT& operator*(){ return *m_ptr;}
  RefT* operator->(){ return m_ptr.get();}

private:
  std::shared_ptr<RefT> m_ptr;
  bool m_valid;
};
