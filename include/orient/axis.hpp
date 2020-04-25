#pragma once

namespace orient {

enum class Axis{x=0,y=1,z=2};

template<Axis A>
constexpr auto asIndex = static_cast<std::underlying_type_t<Axis>>(A);

}
