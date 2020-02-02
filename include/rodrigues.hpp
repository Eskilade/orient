#pragma once 
#include <common.hpp>
#include <gtsam/base/OptionalJacobian.h>

Mat<3,3> rodrigues(Vect<3> const& aa, gtsam::OptionalJacobian<9,3> H = boost::none);
