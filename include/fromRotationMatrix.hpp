#pragma once

#include <common.hpp>
#include <Eigen/Dense>

Vect<3> angleAxisFromRotationMatrix(Mat<3,3> const& R);
Vect<3> angleAxisFromRotationMatrix(Mat<3,3> const& R, Eigen::Ref<Mat<3,9>> H);

Vect<4> quaternionFromRotationMatrix(Mat<3,3> const& R);
Vect<4> quaternionFromRotationMatrix(Mat<3,3> const& R, Eigen::Ref<Mat<4,9>> H);
