#pragma once

#include <common.hpp>
#include <Eigen/Dense>

Mat<3,3> rotationMatrixFromQuaternion(Vect<4> const& q);
Mat<3,3> rotationMatrixFromQuaternion(Vect<4> const& q, Eigen::Ref<Mat<9,4>> H);

Vect<3> angleAxisFromQuaternion(Vect<4> const& q);
Vect<3> angleAxisFromQuaternion(Vect<4> const& q, Eigen::Ref<Mat<3,4>> H);
