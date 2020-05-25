# orient

orient is a C++ header-only library that implements transformations between 3D orientation representations with their first order partial derivatives (aka. Jacobians). The following direct transformations with Jacobians are implemented:
```
AA ----- UQ
   \   /
    \ /
     R
     |
     |
   Euler
```

where
 * AA : Angle axis
 * UQ : Unit quaternion
 * R : Rotation matrix
 * Euler : Any Euler angle sequence

Note: One Euler sequence can be transformed to another Euler sequence by using
a rotation matrix as intermediate 

The remaining transformation can be constructed from the direct transformations as in the example below.
A brief overview of the different transformations and the steps to finding the Jacobian of each
transformation is given
<a href="https://github.com/Eskilade/orient/blob/master/documentation/conversion_formulas.pdf" target="_blank">here</a>.

## Application to propagation of uncertainty

Managing uncertainties is often important to building robust real-world robot
systems and are inherent to a probabilistic approach to robotics. In many of
these real-world systems, 3D rotations tend to play an important role.
Sometimes, we may want to alleviate the down sides of a given representation by
changing it to a different representation, in which case we would need to also
propagate the uncertainties. The expected value and variance (i.e. first and
second moment) of a transformed random variable can be approximated using a first
order Taylor series as follows:

Given some function _f_ and a random variable _X_ such that

  <img src="https://render.githubusercontent.com/render/math?math=E[X]=\mu_X">
  <img src="https://render.githubusercontent.com/render/math?math=Var(X)=\Sigma_X">

then

<img src="https://render.githubusercontent.com/render/math?math=E[f(X)]\approx f(\mu_X)">
<img src="https://render.githubusercontent.com/render/math?math=Var(f(X))\approx J\, \Sigma_X \, J^T">

where

<img src="https://render.githubusercontent.com/render/math?math=J=\left. \frac{\partial f(X)}{\partial X}\right|_{X=\mu_X}"> 

is the partial derivative of the function evaluated at the mean of _X_. For
more information on this subject, a good starting point is to have a look the
Wikipedia articles for
<a href="https://en.wikipedia.org/wiki/Propagation_of_uncertainty" target="_blank">propagation of uncertainty</a> and 
<a href="https://en.wikipedia.org/wiki/Taylor_expansions_for_the_moments_of_functions_of_random_variables" target="_blank">Taylor expansions for the moments of functions of random variables</a>.

The motivation for this library is make the Jacobians of transformations
between the different representations of rotations easily accessible in
application code.

## Example

```cpp
#include <orient/from_angle_axis.hpp>
#include <orient/from_rotation_matrix.hpp>

int main()
{
  // Goal: get roll, pitch, yaw with covariances
  // ... from an angle axis
  Eigen::Vector3d aa = Eigen::Vector3d::Random();
  // ... with associated covariance matrix
  Eigen::Matrix3d aa_cov = 0.1*(Eigen::Vector3d()<<1,2,3).finished().asDiagonal(); 

  // First: get rotation matrix with partial derivatives
  const auto [R, RJaa] = orient::rotationMatrixFromAngleAxisWD(aa);

  // Second: get roll, pitch and yaw with partial derivates.
  // Rotation sequence is given in intrinsic rotations
  using Axis = orient::Axis;
  const auto [ypr, yprJR] = orient::eulerFromRotationMatrixWD<Axis::z, Axis::y, Axis::x>(R);

  // Construct complete jacobian 
  const Eigen::Matrix3d yprJaa = yprJR * RJaa;

  // ... and propagate uncertainty
  const Eigen::Matrix3d ypr_cov = yprJaa * aa_cov * yprJaa.transpose();

  return 0;
}
```
## How to install
 * Clone library then `mkdir orient/build && cd orient/build && cmake .. && sudo make install`

## Prerequisites

- [Eigen](http://eigen.tuxfamily.org/) => 3.3.7
- [CMake](http://www.cmake.org/cmake/resources/software.html) >= 3.0 (Ubuntu: `sudo apt-get install cmake`)
 - A modern C++ compiler that supports C++17

## Prerequisites to compile tests:
- [GTSAM](https://gtsam.org/get_started/) => 4.0.0


## License

This project is licensed under the MIT License - see the [license.txt](license.txt) file for details
