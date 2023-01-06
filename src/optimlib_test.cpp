//
// Created by waxz on 23-1-6.
//


//https://github.com/kthohr/optim#installation-method-2-header-only-library

#define OPTIM_ENABLE_EIGEN_WRAPPERS

#include "optim.hpp"
//#include <autodiff/forward/real.hpp>
//#include <autodiff/forward/real/eigen.hpp>
#include <autodiff/reverse/var.hpp>
#include <autodiff/reverse/var/eigen.hpp>


autodiff::var
opt_fnd(const autodiff::ArrayXvar &x) {
    return (x(0) - 0.9) * (x(0) - 0.9)
           + (x(1) - 1.5) * (x(1) - 1.5)
           + (x(2) - 1.8) * (x(2) - 1.8);
}


double
opt_fn(const Eigen::VectorXd &x, Eigen::VectorXd *grad_out, void *opt_data) {

    autodiff::ArrayXvar xd = x.eval();

    autodiff::var y = opt_fnd(xd);

    if (grad_out) {
        Eigen::VectorXd grad_tmp = autodiff::gradient(y, xd);

        *grad_out = grad_tmp;
    }

    return autodiff::val(y);
}


autodiff::var
circle_opt_fnd(const autodiff::ArrayXvar& x, const std::vector<float>& points, float radius_2)
{
    autodiff::var r = 0.0;
    autodiff::var t = 0.0;

    for(int i = 0 ; i < 50;i++){
        t = (x(0) - points[i+i])*(x(0) - points[i+i]) + (x(1) - points[i+i+1])*(x(1) - points[i+i+1]) - radius_2;
        r += t*t;
//        r += t;

    }
    return r;
}
struct CostFunction{

    std::vector<float> points;
    float radius = 0.1;


    CostFunction(float cx, float cy, float r):radius(r){

        points.resize(100);
        float inc = M_PI / 50.0;
//        std::cout << "\nlist points:\n";

        for(int i = 0; i < 50;i++){
            points[i+i] = cx + std::cos(inc*i)*radius;
            points[i+i+1] = cy + std::sin(inc*i)*radius;
//            std::cout <<points[i+i] << ", " <<points[i+i+1] << "\n";
        }
    }

    double operator()(const Eigen::VectorXd& x, Eigen::VectorXd* grad_out, void* opt_data)
    {

        autodiff::ArrayXvar xd = x.eval();

        autodiff::var y = circle_opt_fnd(xd,points,radius*radius);

        if (grad_out) {
            Eigen::VectorXd grad_tmp = autodiff::gradient(y, xd);

            *grad_out = grad_tmp;
        }

        return autodiff::val(y);
    }
};


void test_1() {
    Eigen::VectorXd x(5);
    x << 1, 2, 3, 4, 5;

    bool success = optim::bfgs(x, opt_fn, nullptr);

    if (success) {
        std::cout << "bfgs: reverse-mode autodiff test completed successfully.\n" << std::endl;
    } else {
        std::cout << "bfgs: reverse-mode autodiff test completed unsuccessfully.\n" << std::endl;
    }

    std::cout << "solution: x = \n" << x << std::endl;

}

void test_2(){
    Eigen::VectorXd x(2);
    x << 0.04, 0.06;

    CostFunction opt_fn_obj(0.12,0.08,0.1) ;

    bool success = optim::bfgs(x, opt_fn_obj, nullptr);

    if (success) {
        std::cout << "bfgs: reverse-mode autodiff test completed successfully.\n" << std::endl;
    } else {
        std::cout << "bfgs: reverse-mode autodiff test completed unsuccessfully.\n" << std::endl;
    }

    std::cout << "solution: x = \n" << x << std::endl;

}

int main(int argc, char **argv) {

    test_1();
    test_2();

}
