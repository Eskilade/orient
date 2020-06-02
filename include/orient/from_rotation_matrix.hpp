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

// \brief Calculate angle axis from rotation matrix
// \param R Source rotation matrix
// \return The angle axis
template<typename Scalar>
Eigen::Matrix<Scalar,3,1> angleAxisFromRotationMatrix(Eigen::Matrix<Scalar,3,3> const& R);

// \brief Calculate angle axis with partial derivatives from rotation matrix
// \param R Source rotation matrix
// \return A pair containing first the angle axis then secondly the Jacobian matrix
template<typename Scalar>
std::pair<Eigen::Matrix<Scalar,3,1>, Eigen::Matrix<Scalar, 3, 9>> angleAxisFromRotationMatrixWD(Eigen::Matrix<Scalar,3,3> const& R);

// \brief Calculate quaternion from rotation matrix
// \param R Source rotation matrix
// \return The quaternion
template<typename Scalar>
Eigen::Matrix<Scalar,4,1> quaternionFromRotationMatrix(Eigen::Matrix<Scalar,3,3> const& R);

// \brief Calculate quaternion with partial derivatives from rotation matrix
// \param R Source rotation matrix
// \return A pair containing first the quaternion then secondly the Jacobian matrix
template<typename Scalar>
std::pair<Eigen::Matrix<Scalar,4,1>, Eigen::Matrix<Scalar, 4, 9>> quaternionFromRotationMatrixWD(Eigen::Matrix<Scalar,3,3> const& R);

// \brief Calculate Euler angles from rotation matrix and a rotation order
//        The order of rotation is expressed in instrinsic rotations.
//        For example, the angles associated to a rotation sequence
//        Rz(yaw) * Ry(pitch) * Rx(roll) = R
//        is retrieve with 
//        auto ypr = eulerFromRotationMatrix<Axis::z, Axis::y, Axis::x>(R)
//        where the angles in ypr are in the same order as the axes,
//        i.e. [yaw, pitch, roll] in this example
// \param R Source rotation matrix
// \template-params A1,A2,A3 the intrinsic rotation order
// \return A vector containing the angles in intrinsic order 
template<Axis A1, Axis A2, Axis A3, typename Scalar, typename Dummy>
Eigen::Matrix<Scalar,3,1> eulerFromRotationMatrix(Eigen::Matrix<Scalar,3,3> const& R);

// \brief Calculate Euler angles and partial derivatives from rotation matrix and a rotation order
//        See above for more detail
// \param R Source rotation matrix
// \template-params A1,A2,A3 The intrinsic rotation order
// \return A pair containg first a vector with angles in intrinsic order and secondly the Jacobian matrix
template<Axis A1, Axis A2, Axis A3, typename Scalar, typename Dummy>
std::pair<Eigen::Matrix<Scalar,3,1>, Eigen::Matrix<Scalar, 3, 9>> eulerFromRotationMatrixWD(Eigen::Matrix<Scalar,3,3> const& R);

}

#include <orient/impl/aa_quat_from_rotation_matrix.hpp>
#include <orient/impl/euler_from_rotation_matrix.hpp>
