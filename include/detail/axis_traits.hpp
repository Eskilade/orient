#pragma once

#include <detail/Index.hpp>
#include <axis.hpp>

namespace detail {

template<Axis a1, Axis a2>
constexpr Axis missing()
{
  return Axis{3 - static_cast<std::underlying_type_t<Axis>>(a1) -
    static_cast<std::underlying_type_t<Axis>>(a2)};
}

template<Axis a1, Axis a2, Axis a3>
constexpr bool isTaitBryan()
{
  return 
    (static_cast<std::underlying_type_t<Axis>>(a1) +
    static_cast<std::underlying_type_t<Axis>>(a2) +
    static_cast<std::underlying_type_t<Axis>>(a3)) == 3;
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
  static constexpr auto a2s = detail::Index{
    static_cast<std::underlying_type_t<Axis>>(a1),
    static_cast<std::underlying_type_t<Axis>>(a3)};

  static constexpr auto a1s = detail::Index{
    static_cast<std::underlying_type_t<Axis>>(a2),
    static_cast<std::underlying_type_t<Axis>>(a3)};

  static constexpr auto a1c = detail::Index{
    static_cast<std::underlying_type_t<Axis>>(a3),
    static_cast<std::underlying_type_t<Axis>>(a3)};

  static constexpr auto a3s = detail::Index{
    static_cast<std::underlying_type_t<Axis>>(a1),
    static_cast<std::underlying_type_t<Axis>>(a2)};

  static constexpr auto a3c = detail::Index{
    static_cast<std::underlying_type_t<Axis>>(a1),
    static_cast<std::underlying_type_t<Axis>>(a1)};

  struct GimbalLock {
    static constexpr auto a1s = detail::Index{
      static_cast<std::underlying_type_t<Axis>>(a2),
      static_cast<std::underlying_type_t<Axis>>(a1)};

    static constexpr auto a1c = detail::Index{
      static_cast<std::underlying_type_t<Axis>>(a2),
      static_cast<std::underlying_type_t<Axis>>(a2)};
  };
};

template<Axis a1, Axis a2>
struct ProperEulerTraits
{
  static constexpr Axis ma = missing<a1,a2>();
  static constexpr auto a2c = detail::Index{
    static_cast<std::underlying_type_t<Axis>>(a1),
    static_cast<std::underlying_type_t<Axis>>(a1)};

  static constexpr auto a1s = detail::Index{
    static_cast<std::underlying_type_t<Axis>>(a2),
    static_cast<std::underlying_type_t<Axis>>(a1)};

  static constexpr auto a1c = detail::Index{
    static_cast<std::underlying_type_t<Axis>>(ma),
    static_cast<std::underlying_type_t<Axis>>(a1)};

  static constexpr auto a3s = detail::Index{
    static_cast<std::underlying_type_t<Axis>>(a1),
    static_cast<std::underlying_type_t<Axis>>(a2)};

  static constexpr auto a3c = detail::Index{
    static_cast<std::underlying_type_t<Axis>>(a1),
    static_cast<std::underlying_type_t<Axis>>(ma)};

  struct GimbalLock {
    static constexpr auto a1s = detail::Index{
      static_cast<std::underlying_type_t<Axis>>(a2),
      static_cast<std::underlying_type_t<Axis>>(ma)};

    static constexpr auto a1c = detail::Index{
      static_cast<std::underlying_type_t<Axis>>(a2),
      static_cast<std::underlying_type_t<Axis>>(a2)};
  };
};

}
