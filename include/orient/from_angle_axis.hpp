#pragma once

#include <utility>

#include <Eigen/Dense>

namespace orient {

Eigen::Vector4d quaternionFromAngleAxis(Eigen::Vector3d const& aa);
std::pair<Eigen::Vector4d, Eigen::Matrix<double, 4, 3>> quaternionFromAngleAxisWD(Eigen::Vector3d const& aa);

Eigen::Matrix3d rotationMatrixFromAngleAxis(Eigen::Vector3d const& aa);
std::pair<Eigen::Matrix3d, Eigen::Matrix<double, 9, 3>> rotationMatrixFromAngleAxisWD(Eigen::Vector3d const& aa);

}
