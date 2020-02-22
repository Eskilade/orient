#pragma once

#include <Eigen/Dense>

Eigen::Vector3d angleAxisFromRotationMatrix(Eigen::Matrix3d const& R);
Eigen::Vector3d angleAxisFromRotationMatrix(Eigen::Matrix3d const& R, Eigen::Ref<Eigen::Matrix<double, 3, 9>> H);

Eigen::Vector4d quaternionFromRotationMatrix(Eigen::Matrix3d const& R);
Eigen::Vector4d quaternionFromRotationMatrix(Eigen::Matrix3d const& R, Eigen::Ref<Eigen::Matrix<double, 4, 9>> H);
