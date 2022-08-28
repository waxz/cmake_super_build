#include <iostream>

#include "ceres/ceres.h"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

using ceres::CENTRAL;
using ceres::CostFunction;
using ceres::NumericDiffCostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;
// A cost functor that implements the residual r = 10 - x.
struct CostFunctor {
    bool operator()(const double* const x, double* residual) const {
        residual[0] = 10.0 - x[0];
        return true;
    }
};

#include "xmlrpcpp/XmlRpc.h"
#include "ros/ros.h"
#include "boost/container/deque.hpp"
bool  getParam(std::string name, XmlRpc::XmlRpcValue & value){

    name = ros::this_node::getName() + "/" + name;
    if (ros::param::has(name)) {
        std::cerr << "ros get xml value, has name: " << name << std::endl;

    } else {
        std::cerr << "ros get xml value,do not  has name: " << name << std::endl;
        return false;

    }

    ros::param::get(name, value);
    return true;

}


int main(int argc, char** argv) {

    {
        ros::init(argc,argv,"test");


    }
    {

        cv::Mat C = (cv::Mat_<double>(3,3) << 0, -1, 0, -1, 5, -1, 0, -1, 0);
        std::cout << "C = " << std::endl << " " << C << std::endl ;
    }
    // The variable to solve for with its initial value. It will be
    // mutated in place by the solver.
    double x = 0.5;
    const double initial_x = x;
    // Build the problem.
    Problem problem;
    // Set up the only cost function (also known as residual). This uses
    // numeric differentiation to obtain the derivative (jacobian).
    CostFunction* cost_function =
            new NumericDiffCostFunction<CostFunctor, CENTRAL, 1, 1>(new CostFunctor);
    problem.AddResidualBlock(cost_function, nullptr, &x);
    // Run the solver!
    Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    Solver::Summary summary;
    Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << "\n";
    std::cout << "x : " << initial_x << " -> " << x << "\n";
    return 1;
}
