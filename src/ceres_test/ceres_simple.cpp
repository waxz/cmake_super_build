//
// Created by waxz on 3/7/24.
//

#include "ceres/ceres.h"
#include "ceres/cubic_interpolation.h"

//NumericDiffMethodType
using ceres::CENTRAL;
using ceres::FORWARD;
using ceres::RIDDERS;


using ceres::CostFunction;
using ceres::LossFunction;

using ceres::NumericDiffCostFunction;
using ceres::AutoDiffCostFunction;

using ceres::Problem;
using ceres::Solve;
using ceres::Solver;




struct Linear1d_CostFunctor {
    template<typename T>
    bool operator()(const T *const x, T *residual) const {
        residual[0] = ceres::abs( x[0] - x[1] - 1.0 ) ;
        return true;
    }
};
struct Circle_CostFunctor {
    template<typename T>
    bool operator()(const T *const x, T *residual) const {
        residual[0] = x[0]*x[0] + x[1]*x[1] - 100.0;
        return true;
    }
};
double distance_to_6(double x){
    return std::abs(x - 6.0);
}
double circle_to_10(double x, double y){
    return x*x + y*y - 100.0;
}
double linerar_diff_1(double x, double y){
    return std::abs(x - y- 1.0) ;
}

struct Linear1d_CostFunctor_Numeric {
    template<typename T>
    bool operator()(const T *const x, T *residual) const {
        residual[0] = linerar_diff_1( x[0],x[1]);
        return true;
    }
};
struct Circle_CostFunctor_Numeric {
    template<typename T>
    bool operator()(const T *const x, T *residual) const {
        residual[0] = circle_to_10( x[0],x[1]);
        return true;
    }
};
class MyScalarCostFunctor {
    MyScalarCostFunctor(double k): k_(k) {}

    bool operator()(const double* const x,
                    const double* const y,
                    double* residuals) const {
        residuals[0] = k_ - x[0] * y[0] + x[1] * y[1];
        return true;
    }

private:
    double k_;
};

void solve(){

}

int main(){
    double initial_x[2] = {0.01,0.01};

    // Build the problem.
    Problem problem;
    // Set up the only cost function (also known as residual). This uses
    // numeric differentiation to obtain the derivative (jacobian).

/*
template <typename CostFunctor,
          NumericDiffMethodType kMethod = CENTRAL,
          int kNumResiduals = 0,  // Number of residuals, or ceres::DYNAMIC
          int... Ns>              // Parameters dimensions for each block.
 */
/*
template <typename CostFunctor,
          int kNumResiduals,  // Number of residuals, or ceres::DYNAMIC.
          int... Ns>          // Number of parameters in each parameter block.
 */



    CostFunction *cost_function_circle_numd = new NumericDiffCostFunction<Circle_CostFunctor_Numeric, CENTRAL, 1, 2>(new Circle_CostFunctor_Numeric);
    CostFunction* cost_function_circle_autod = new AutoDiffCostFunction<Circle_CostFunctor, 1, 2>(new Circle_CostFunctor);
    CostFunction *cost_function_line_numd = new NumericDiffCostFunction<Linear1d_CostFunctor_Numeric, CENTRAL, 1, 2>(new Linear1d_CostFunctor_Numeric);
    CostFunction* cost_function_line_autod = new AutoDiffCostFunction<Linear1d_CostFunctor, 1, 2>(new Linear1d_CostFunctor);


    problem.AddResidualBlock(cost_function_circle_numd, nullptr, initial_x);
    problem.AddResidualBlock(cost_function_circle_autod, nullptr, initial_x);
    problem.AddResidualBlock(cost_function_line_numd, nullptr, initial_x);
    problem.AddResidualBlock(cost_function_line_autod, nullptr, initial_x);
    // Run the solver!

    ceres::Solver::Options ceres_solver_options;
    ceres_solver_options.linear_solver_type = ceres::DENSE_QR;
    ceres_solver_options.minimizer_progress_to_stdout = true;
    ceres_solver_options.num_threads = 1;
    ceres_solver_options.function_tolerance = 1e-3;  // Enough for denoising.
    ceres_solver_options.max_num_iterations = 100;
    ceres_solver_options.gradient_tolerance = 1e-3;
    ceres_solver_options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    ceres_solver_options.parameter_tolerance = 1e-6;


    Solver::Summary summary;
    Solve(ceres_solver_options, &problem, &summary);

    std::cout << summary.BriefReport() << "\n";

    std::cout << "final    summary.initial_cost : " << summary.initial_cost << ",    summary.final_cost : "
              << summary.final_cost << std::endl;
    std::cout << "final initial_x : " << initial_x[0] << ", " << initial_x[1] << std::endl;
    auto test_1 =  initial_x[0]*initial_x[0]   + initial_x[1] * initial_x[1];
    auto test_2 =  initial_x[0] -  initial_x[1];

    std::cout << "final test_1 : " << test_1 << std::endl;
    std::cout << "final test_2 : " << test_2 << std::endl;

}