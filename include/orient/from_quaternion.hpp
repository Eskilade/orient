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
Eigen::Matrix<Scalar, 3, 3> rotationMatrixFromQuaternion(Eigen::Matrix<Scalar, 4, 1> const& q);

template<typename Scalar>
std::pair<Eigen::Matrix<Scalar, 3, 3>, Eigen::Matrix<Scalar, 9, 4>> rotationMatrixFromQuaternionWD(Eigen::Matrix<Scalar, 4, 1> const& q);

template<typename Scalar>
Eigen::Matrix<Scalar, 3, 1> angleAxisFromQuaternion(Eigen::Matrix<Scalar, 4, 1> const& q);

template<typename Scalar>
std::pair<Eigen::Matrix<Scalar, 3, 1>, Eigen::Matrix<Scalar, 3, 4>> angleAxisFromQuaternionWD(Eigen::Matrix<Scalar, 4, 1> const& q);

}

#include <orient/impl/from_quaternion.hpp>
