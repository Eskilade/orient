#pragma once

#include <Eigen/Dense>

namespace orient {

Eigen::Matrix3d rotationMatrixFromQuaternion(Eigen::Vector4d const& q);
Eigen::Matrix3d rotationMatrixFromQuaternion(Eigen::Vector4d const& q, Eigen::Ref<Eigen::Matrix<double, 9, 4>> H);

Eigen::Vector3d angleAxisFromQuaternion(Eigen::Vector4d const& q);
Eigen::Vector3d angleAxisFromQuaternion(Eigen::Vector4d const& q, Eigen::Ref<Eigen::Matrix<double, 3, 4>> H);

}
