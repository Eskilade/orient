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

  // Second: get roll, pitch and yaw with partial derivates
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
