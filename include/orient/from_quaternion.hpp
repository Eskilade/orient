#pragma once

#include <utility>

#include <Eigen/Dense>

namespace orient {

Eigen::Matrix3d rotationMatrixFromQuaternion(Eigen::Vector4d const& q);
std::pair<Eigen::Matrix3d, Eigen::Matrix<double, 9, 4>> rotationMatrixFromQuaternionWD(Eigen::Vector4d const& q);

Eigen::Vector3d angleAxisFromQuaternion(Eigen::Vector4d const& q);
std::pair<Eigen::Vector3d, Eigen::Matrix<double, 3, 4>> angleAxisFromQuaternionWD(Eigen::Vector4d const& q);

}
