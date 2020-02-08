#pragma once

#include <common.hpp>
#include <OptionalRef.hpp>

Mat<3,3> rotationMatrixFromQuaternion(Vect<4> const& q, OptionalRef<Mat<9,4>> H = {});

Vect<3> angleAxisFromQuaternion(Vect<4> const& q, OptionalRef<Mat<3,4>> H = {});
