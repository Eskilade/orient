#pragma once

#include <common.hpp>
#include <Eigen/Dense>

Vect<4> quaternionFromAngleAxis(Vect<3> const& aa);
Vect<4> quaternionFromAngleAxis(Vect<3> const& aa, Eigen::Ref<Mat<4,3>> H);

Mat<3,3> rotationMatrixFromAngleAxis(Vect<3> const& aa);
Mat<3,3> rotationMatrixFromAngleAxis(Vect<3> const& aa, Eigen::Ref<Mat<9,3>> H);
