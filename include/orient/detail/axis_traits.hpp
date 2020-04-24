#pragma once

#include <orient/axis.hpp>

namespace orient::detail {

template<Axis A>
constexpr auto asIndex = static_cast<std::underlying_type_t<Axis>>(A);


template<Axis a1, Axis a2>
constexpr Axis missing()
{
  return Axis{3 - asIndex<a1> -
    asIndex<a2>};
}

template<Axis a1, Axis a2, Axis a3>
constexpr bool isTaitBryan()
{
  return 
    (asIndex<a1> +
    asIndex<a2> +
    asIndex<a3>) == 3;
}

template<Axis a1, Axis a2, Axis a3>
constexpr bool isProperEuler()
{
  return (a1 == a3) and (a1 != a2);
}

template<Axis a1, Axis a2, Axis a3>
constexpr bool isMalformed()
{
  return (a1 == a2) or (a2 == a3);
}

template<Axis a1, Axis a2, Axis a3>
struct TaitBryanTraits
{
  static constexpr auto a2s = asIndex<a1> + asIndex<a3>*3;
  static constexpr auto a1s = asIndex<a2> + asIndex<a3>*3;
  static constexpr auto a1c = asIndex<a3> + asIndex<a3>*3;
  static constexpr auto a3s = asIndex<a1> + asIndex<a2>*3;
  static constexpr auto a3c = asIndex<a1> + asIndex<a1>*3;

  struct GimbalLock { 
    static constexpr auto a1s = asIndex<a2> + asIndex<a1>*3;
    static constexpr auto a1c = asIndex<a2> + asIndex<a2>*3;
  };
};

template<Axis a1, Axis a2>
struct ProperEulerTraits
{
  static constexpr Axis ma = missing<a1,a2>(); 
  
  static constexpr auto a2c = asIndex<a1> + asIndex<a1>*3;
  static constexpr auto a1s = asIndex<a2> + asIndex<a1>*3;
  static constexpr auto a1c = asIndex<ma> + asIndex<a1>*3;
  static constexpr auto a3s = asIndex<a1> + asIndex<a2>*3;
  static constexpr auto a3c = asIndex<a1> + asIndex<ma>*3;

  struct GimbalLock {
    static constexpr auto a1s =  asIndex<a2> + asIndex<ma>*3;
    static constexpr auto a1c =  asIndex<a2> + asIndex<a2>*3;
  };
};

}
