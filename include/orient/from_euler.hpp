/* @copyright The code is licensed under the MIT License
 *            <https://opensource.org/licenses/MIT>,
 *            Copyright (c) 2020 Christian Eskil Vaugelade Berg
 * @author Christian Eskil Vaugelade Berg
*/
#pragma once

#include <orient/axis.hpp>

#include <Eigen/Dense>

namespace orient {

template<Axis axis, typename Scalar>
Eigen::Matrix<Scalar,3,3> rotationMatrixFromEuler(Scalar angle);

template<Axis axis, typename Scalar>
std::pair<Eigen::Matrix<Scalar,3,3>, Eigen::Matrix<Scalar,3,3>> rotationMatrixFromEulerWD(Scalar angle);

template<Axis a1, Axis a2, Axis a3, typename Scalar>
Eigen::Matrix<Scalar,3,3> rotationMatrixFromEuler(Eigen::Matrix<Scalar,3,1> const& angles);

template<Axis a1, Axis a2, Axis a3, typename Scalar>
std::pair<Eigen::Matrix<Scalar,3,3>, Eigen::Matrix<Scalar, 9, 3>> rotationMatrixFromEulerWD(Eigen::Matrix<Scalar,3,1> const& angles);

}

#include <orient/impl/from_euler.hpp>
