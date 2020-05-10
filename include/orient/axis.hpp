/* @copyright The code is licensed under the MIT License
 *            <https://opensource.org/licenses/MIT>,
 *            Copyright (c) 2020 Christian Eskil Vaugelade Berg
 * @author Christian Eskil Vaugelade Berg
*/
#pragma once

#include <type_traits>

namespace orient {

enum class Axis{x=0,y=1,z=2};

template<Axis A>
constexpr auto asIndex = static_cast<std::underlying_type_t<Axis>>(A);

}
