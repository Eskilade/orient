/* @copyright The code is licensed under the MIT License
 *            <https://opensource.org/licenses/MIT>,
 *            Copyright (c) 2020 Christian Eskil Vaugelade Berg
 * @author Christian Eskil Vaugelade Berg
*/
#pragma once

#include <orient/axis.hpp>
#include <orient/detail/axis_traits.hpp>

#include <Eigen/Dense>

#include <type_traits>
#include <utility>

namespace orient {

template<typename Scalar>
Eigen::Matrix<Scalar,3,1> angleAxisFromRotationMatrix(Eigen::Matrix<Scalar,3,3> const& R);

template<typename Scalar>
std::pair<Eigen::Matrix<Scalar,3,1>, Eigen::Matrix<Scalar, 3, 9>> angleAxisFromRotationMatrixWD(Eigen::Matrix<Scalar,3,3> const& R);

template<typename Scalar>
Eigen::Matrix<Scalar,4,1> quaternionFromRotationMatrix(Eigen::Matrix<Scalar,3,3> const& R);

template<typename Scalar>
std::pair<Eigen::Matrix<Scalar,4,1>, Eigen::Matrix<Scalar, 4, 9>> quaternionFromRotationMatrixWD(Eigen::Matrix<Scalar,3,3> const& R);

template<Axis A1, Axis A2, Axis A3, typename Scalar, typename Dummy>
Eigen::Matrix<Scalar,3,1> eulerFromRotationMatrix(Eigen::Matrix<Scalar,3,3> const& R);

template<Axis A1, Axis A2, Axis A3, typename Scalar, typename Dummy>
std::pair<Eigen::Matrix<Scalar,3,1>, Eigen::Matrix<Scalar, 3, 9>> eulerFromRotationMatrixWD(Eigen::Matrix<Scalar,3,3> const& R);

}

#include <orient/impl/aa_quat_from_rotation_matrix.hpp>
#include <orient/impl/euler_from_rotation_matrix.hpp>
