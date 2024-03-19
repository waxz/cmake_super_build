# install
http://ceres-solver.org/installation.html#version-2-0

# cmake
```cmake
add_executable(ceres_simple ceres_simple.cpp)
target_link_libraries(ceres_simple PUBLIC Ceres::ceres)
```
# cost template function
The first step is to write a functor that will evaluate this the function: f(x) = 10.0 - x

```c++

struct CostFunctor {
   template <typename T>
   bool operator()(const T* const x, T* residual) const {
     residual[0] = 10.0 - x[0];
     return true;
   }
};
```
The important thing to note here is that `operator()` is a templated method, which assumes that all its inputs and outputs are of some type `T`. The use of templating here allows Ceres to call `CostFunctor::operator<T>()`, with `T=double` when just the value of the `residual` is needed, and with a special type `T=Jet` when the `Jacobians` are needed.

```c++
int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);

  // The variable to solve for with its initial value.
  double initial_x = 5.0;
  double x = initial_x;

  // Build the problem.
  Problem problem;

  // Set up the only cost function (also known as residual). This uses
  // auto-differentiation to obtain the derivative (jacobian).
  CostFunction* cost_function =
      new AutoDiffCostFunction<CostFunctor, 1, 1>();
  problem.AddResidualBlock(cost_function, nullptr, &x);

  // Run the solver!
  Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = true;
  Solver::Summary summary;
  Solve(options, &problem, &summary);

  std::cout << summary.BriefReport() << "\n";
  std::cout << "x : " << initial_x
            << " -> " << x << "\n";
  return 0;
}
```
## ResidualBlock
### CostFunction
### ParameterBlock


### LossFunction
http://ceres-solver.org/nnls_modeling.html#_CPPv4N5ceres12LossFunctionE
  A LossFunction is a scalar function that is used to reduce the influence of outliers on the solution of non-linear least squares problems.
  Special case,  `LossFunction` is the identity function

# derivatives
http://ceres-solver.org/nnls_tutorial.html#numeric-derivatives
http://ceres-solver.org/numerical_derivatives.html
http://ceres-solver.org/automatic_derivatives.html
http://ceres-solver.org/interfacing_with_autodiff.html


### Numeric Derivatives
In some cases, its not possible to define a templated cost functor, for example when the evaluation of the residual involves a call to a library function that you do not have control over.
In such a situation, numerical differentiation can be used.
The user defines a functor which computes the residual value and construct a NumericDiffCostFunction using it.


### So far so good, but let us now consider three ways of defining
which are not directly amenable to being used with automatic differentiation:

A non-templated function that evaluates its value.

A function that evaluates its value and derivative.

A function that is defined as a table of values to be interpolated.

# dynamic

# interplator

# se2

# se3

# icp




