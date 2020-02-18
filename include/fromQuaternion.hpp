#pragma once

#include <common.hpp>
#include <Eigen/Dense>

Mat<3,3> rotationMatrixFromQuaternion(Eigen::Vector4d const& q);
Mat<3,3> rotationMatrixFromQuaternion(Eigen::Vector4d const& q, Eigen::Ref<Mat<9,4>> H);

Eigen::Vector3d angleAxisFromQuaternion(Eigen::Vector4d const& q);
Eigen::Vector3d angleAxisFromQuaternion(Eigen::Vector4d const& q, Eigen::Ref<Mat<3,4>> H);
