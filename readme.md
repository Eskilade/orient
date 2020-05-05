# orient

orient is a C++ library that implements transformations between 3D orientation representations with their first order partial derivatives (aka. Jacobians). 

## Prerequisites

- [Eigen](http://eigen.tuxfamily.org/) => 3.3.7
- [CMake](http://www.cmake.org/cmake/resources/software.html) >= 3.0 (Ubuntu: `sudo apt-get install cmake`)
 - A modern C++ compiler that supports C++17
``

## Prerequisites to compile tests:
- [GTSAM](https://gtsam.org/get_started/) => 4.0.0

## Example

```cpp
#include <orient/from_angle_axis.hpp>
#include <orient/euler_from_rotation_matrix.hpp>

int main()
{
  // Goal: get roll, pitch and yaw and covariances
  Eigen::Vector3d aa = Eigen::Vector3d::Random(); // given some random angle axis
  Eigen::Matrix3d aa_cov = Eigen::Vector3d::Random().asDiagonal(); // ... with associated covariance matrix

  // First: get rotation matrix with partial derivative
  const auto [R, RJaa] = orient::rotationMatrixFromAngleAxisWD(aa);

  // Second: get yaw, pitch and roll with partial derivate
  using Axis = orient::Axis;
  const auto [ypr, yprJR] = orient::eulerFromRotationMatrixWD<Axis::z, Axis::y, Axis::x>(R);

  // Construct complete jacobian 
  const auto yprJaa = yprJR * RJaa;

  // ... and propagate uncertainty
  const auto ypr_cov = yprJaa * aa_cov * yprJaa.transpose();

  return 0;
}
```
## License

This project is licensed under the MIT License - see the [license.txt](license.txt) file for details
