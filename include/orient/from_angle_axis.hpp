/* @copyright The code is licensed under the MIT License
 *            <https://opensource.org/licenses/MIT>,
 *            Copyright (c) 2020 Christian Eskil Vaugelade Berg
 * @author Christian Eskil Vaugelade Berg
*/
#pragma once

#include <utility>

#include <Eigen/Dense>

namespace orient {

// \brief Calculate quaternion from angle axis
//        Quaternion is expressed as a 4D-vector [w,a,b,c] such that
//        q = w + a*i + b*j + c*k
// \param aa Source angle axis
// \return The quaternion
template<typename Scalar>
Eigen::Matrix<Scalar, 4, 1> quaternionFromAngleAxis(Eigen::Matrix<Scalar, 3, 1> const& aa);

// \brief Calculate quaternion and partial derivatives from angle axis 
//        See above for more detail
// \param aa Source angle axis
// \return A pair containg first the quaternion then secondly the Jacobian matrix 
template<typename Scalar>
std::pair<Eigen::Matrix<Scalar, 4, 1>, Eigen::Matrix<Scalar, 4, 3>> quaternionFromAngleAxisWD(Eigen::Matrix<Scalar,3, 1> const& aa);

// \brief Calculate rotation matrix from angle axis
// \param aa Source angle axis
// \return Rotation matrix
template<typename Scalar>
Eigen::Matrix<Scalar, 3, 3> rotationMatrixFromAngleAxis(Eigen::Matrix<Scalar,3, 1> const& aa);

// \brief Calculate rotation matrix and partial derivatives from angle axis
// \param aa Source angle axis
// \return A pair containg first the rotation matrix then secondly the Jacobian matrix 
template<typename Scalar>
std::pair<Eigen::Matrix<Scalar, 3, 3>, Eigen::Matrix<Scalar, 9, 3>> rotationMatrixFromAngleAxisWD(Eigen::Matrix<Scalar, 3, 1> const& aa);

}

#include <orient/impl/from_angle_axis.hpp>
