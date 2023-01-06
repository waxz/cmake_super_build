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

struct Point{
    float x = 0.0;
    float y = 0.0;
    float z = 0.0;
    float r = 0.0;
};


autodiff::var
circle_opt_fnd(const autodiff::ArrayXvar& x, const std::vector<Point>& points,int point_num, float radius_2)
{
    autodiff::var r = 0.0;
    autodiff::var t = 0.0;

    for(int i = 0 ; i < point_num ;i++){
        t = (x(0) - points[i].x)*(x(0) - points[i].x) + (x(1) - points[i].y)*(x(1) - points[i].y) - radius_2;
        r += t*t;
    }
    return r;
}

struct CircleCostFunction{

    const std::vector<Point>& points;
    int point_num;
    float radius = 0.1;
    float radius_2 = radius;


    CircleCostFunction(const std::vector<Point>& t_points, int t_point_num, float t_radius):points(t_points),point_num(t_point_num), radius(t_radius),radius_2(radius*radius){
    }



    double operator()(const Eigen::VectorXd& x, Eigen::VectorXd* grad_out, void* opt_data)
    {

        autodiff::ArrayXvar xd = x.eval();

        autodiff::var y = circle_opt_fnd(xd,points,point_num,radius_2);

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

    int point_num = 100;
    std::vector<Point> points(point_num);
    float angle_inc = 2.0f*M_PI/float(point_num);

    float sample_cx = 0.13, sample_cy = 0.06, sample_radius = 0.1;

    for(int i = 0 ; i < point_num;i++){
        points[i].x = sample_cx + sample_radius*std::cos(i*angle_inc);
        points[i].y = sample_cy + sample_radius*std::sin(i*angle_inc);

    }

    Eigen::VectorXd x(2);
    x << 0.05, 0.05;

    CircleCostFunction opt_fn_obj(points,point_num, sample_radius) ;

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
