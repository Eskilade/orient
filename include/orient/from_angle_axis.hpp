/* @copyright The code is licensed under the MIT License
 *            <https://opensource.org/licenses/MIT>,
 *            Copyright (c) 2020 Christian Eskil Vaugelade Berg
 * @author Christian Eskil Vaugelade Berg
*/
#pragma once

#include <utility>

#include <Eigen/Dense>

namespace orient {

template<typename Scalar>
Eigen::Matrix<Scalar, 4, 1> quaternionFromAngleAxis(Eigen::Matrix<Scalar, 3, 1> const& aa);

template<typename Scalar>
std::pair<Eigen::Matrix<Scalar, 4, 1>, Eigen::Matrix<Scalar, 4, 3>> quaternionFromAngleAxisWD(Eigen::Matrix<Scalar,3, 1> const& aa);

template<typename Scalar>
Eigen::Matrix<Scalar, 3, 3> rotationMatrixFromAngleAxis(Eigen::Matrix<Scalar,3, 1> const& aa);

template<typename Scalar>
std::pair<Eigen::Matrix<Scalar, 3, 3>, Eigen::Matrix<Scalar, 9, 3>> rotationMatrixFromAngleAxisWD(Eigen::Matrix<Scalar, 3, 1> const& aa);

}

#include <orient/impl/from_angle_axis.hpp>
