/* @copyright The code is licensed under the MIT License
 *            <https://opensource.org/licenses/MIT>,
 *            Copyright (c) 2020 Christian Eskil Vaugelade Berg
 * @author Christian Eskil Vaugelade Berg
*/
#pragma once

#include <orient/axis.hpp>

#include <Eigen/Dense>

namespace orient {

// \brief Construct rotation matrix from scalar angle and a rotation axis
// \param angle Scalar angle
// \template-param axis Rotation axis
// \return Rotation matrix
template<Axis axis, typename Scalar>
Eigen::Matrix<Scalar,3,3> rotationMatrixFromEuler(Scalar angle);

// \brief Construct rotation matrix and partial derivatives from scalar angle and a rotation axis
// \param angle Scalar angle
// \template-param axis Rotation axis
// \return A pair containing first the rotation matrix and then secondly the Jacobian matrix
template<Axis axis, typename Scalar>
std::pair<Eigen::Matrix<Scalar,3,3>, Eigen::Matrix<Scalar,3,3>> rotationMatrixFromEulerWD(Scalar angle);

// \brief Calculate rotation matrix from Euler angles and a rotation order
//        The rotation order is expressed in instrisic rotations.
//        For example, the rotation matrix
//        R = Rz(yaw) * Ry(pitch) * Rx(roll)
//        is constructed with
//        auto R = rotationMatrixFromEuler<Axis::z, Axis::y, Axis::x>([yaw, pitch, roll]);
// \param anlges Source Euler angles
// \template-params A1,A2,A3 The intrinsic rotation order
// \return Rotation matrix
template<Axis A1, Axis A2, Axis A3, typename Scalar>
Eigen::Matrix<Scalar,3,3> rotationMatrixFromEuler(Eigen::Matrix<Scalar,3,1> const& angles);

// \brief Calculate rotation matrix and partial derivatves from Euler angles and a rotation order
//        See above for detail
// \param anlges Source Euler angles
// \template-params A1,A2,A3 The intrinsic rotation order
// \return A pair containing first the rotation matrix and then secondly the Jacobian matrix
template<Axis A1, Axis A2, Axis A3, typename Scalar>
std::pair<Eigen::Matrix<Scalar,3,3>, Eigen::Matrix<Scalar, 9, 3>> rotationMatrixFromEulerWD(Eigen::Matrix<Scalar,3,1> const& angles);

}

#include <orient/impl/from_euler.hpp>
