#pragma once

#include <utility>

#include <Eigen/Dense>

namespace orient {

Eigen::Vector3d angleAxisFromRotationMatrix(Eigen::Matrix3d const& R);
std::pair<Eigen::Vector3d, Eigen::Matrix<double, 3, 9>> angleAxisFromRotationMatrixWD(Eigen::Matrix3d const& R);

Eigen::Vector4d quaternionFromRotationMatrix(Eigen::Matrix3d const& R);
std::pair<Eigen::Vector4d, Eigen::Matrix<double, 4, 9>> quaternionFromRotationMatrixWD(Eigen::Matrix3d const& R);

}
