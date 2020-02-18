#pragma once

#include <common.hpp>
#include <Eigen/Dense>

Eigen::Vector3d angleAxisFromRotationMatrix(Mat<3,3> const& R);
Eigen::Vector3d angleAxisFromRotationMatrix(Mat<3,3> const& R, Eigen::Ref<Mat<3,9>> H);

Eigen::Vector4d quaternionFromRotationMatrix(Mat<3,3> const& R);
Eigen::Vector4d quaternionFromRotationMatrix(Mat<3,3> const& R, Eigen::Ref<Mat<4,9>> H);
