#pragma once

#include <Eigen/Dense>

template<int R, int C>
using Mat = Eigen::Matrix<double, R, C>;

template<int R>
using Vect = Eigen::Matrix<double, R, 1>;

enum class Axis{x=0,y=1,z=2};
