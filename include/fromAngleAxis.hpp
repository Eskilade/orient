#pragma once

#include <common.hpp>
#include <Eigen/Dense>

Eigen::Vector4d quaternionFromAngleAxis(Eigen::Vector3d const& aa);
Eigen::Vector4d quaternionFromAngleAxis(Eigen::Vector3d const& aa, Eigen::Ref<Mat<4,3>> H);

Mat<3,3> rotationMatrixFromAngleAxis(Eigen::Vector3d const& aa);
Mat<3,3> rotationMatrixFromAngleAxis(Eigen::Vector3d const& aa, Eigen::Ref<Mat<9,3>> H);
