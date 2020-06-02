/* @copyright The code is licensed under the MIT License
 *            <https://opensource.org/licenses/MIT>,
 *            Copyright (c) 2020 Christian Eskil Vaugelade Berg
 * @author Christian Eskil Vaugelade Berg
*/
#pragma once

#include <utility>

#include <Eigen/Dense>

namespace orient {

// \brief Calculate rotation matrix from quaternion
//        Quaternion is expressed as a 4D-vector [w,a,b,c] such that
//        q = w + a*i + b*j + c*k
// \param q Source quaternion
// \return Rotation matrix
template<typename Scalar>
Eigen::Matrix<Scalar, 3, 3> rotationMatrixFromQuaternion(Eigen::Matrix<Scalar, 4, 1> const& q);

// \brief Calculate rotation matrix and partial derivatives from quaternion
//        See above for more detail
// \param q Source quaternion
// \return A pair containg first the rotation matrix then secondly the Jacobian matrix 
template<typename Scalar>
std::pair<Eigen::Matrix<Scalar, 3, 3>, Eigen::Matrix<Scalar, 9, 4>> rotationMatrixFromQuaternionWD(Eigen::Matrix<Scalar, 4, 1> const& q);

// \brief Calculate angle axis derivatives from quaternion
//        See above for more detail
// \param q Source quaternion
// \return Angle axis
template<typename Scalar>
Eigen::Matrix<Scalar, 3, 1> angleAxisFromQuaternion(Eigen::Matrix<Scalar, 4, 1> const& q);

// \brief Calculate angle axis and partial derivatives from quaternion
//        See above for more detail
// \param q Source quaternion
// \return A pair containg first the angle axis secondly the Jacobian matrix 
template<typename Scalar>
std::pair<Eigen::Matrix<Scalar, 3, 1>, Eigen::Matrix<Scalar, 3, 4>> angleAxisFromQuaternionWD(Eigen::Matrix<Scalar, 4, 1> const& q);

}

#include <orient/impl/from_quaternion.hpp>
