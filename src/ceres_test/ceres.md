# install
http://ceres-solver.org/installation.html#version-2-0

# cmake
```cmake
add_executable(ceres_simple ceres_simple.cpp)
target_link_libraries(ceres_simple PUBLIC Ceres::ceres)
```

# numeric-derivatives
http://ceres-solver.org/nnls_tutorial.html#numeric-derivatives

### Numeric Derivatives
In some cases, its not possible to define a templated cost functor, for example when the evaluation of the residual involves a call to a library function that you do not have control over.
In such a situation, numerical differentiation can be used.
The user defines a functor which computes the residual value and construct a NumericDiffCostFunction using it.


# dynamic

# se2

# se3

# icp




