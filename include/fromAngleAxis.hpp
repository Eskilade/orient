#pragma once

#include <Eigen/Dense>

Eigen::Vector4d quaternionFromAngleAxis(Eigen::Vector3d const& aa);
Eigen::Vector4d quaternionFromAngleAxis(Eigen::Vector3d const& aa, Eigen::Ref<Eigen::Matrix<double, 4, 3>> H);

Eigen::Matrix3d rotationMatrixFromAngleAxis(Eigen::Vector3d const& aa);
Eigen::Matrix3d rotationMatrixFromAngleAxis(Eigen::Vector3d const& aa, Eigen::Ref<Eigen::Matrix<double, 9, 3>> H);
