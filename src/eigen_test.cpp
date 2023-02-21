//
// Created by waxz on 23-2-21.
//

#include <Eigen/Core>
#include <Eigen/Geometry>


#include <iostream>

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





template<typename FloatType>
Eigen::Transform<FloatType,3,Eigen::Isometry> createSe3(FloatType tx, FloatType ty, FloatType tz, FloatType qw, FloatType qx, FloatType qy, FloatType qz){

    Eigen::Transform<FloatType,3,Eigen::Isometry> result = Eigen::Transform<FloatType,3,Eigen::Isometry>::Identity();;
//    Eigen::Quaternion<FloatType> Q = Eigen::Quaternion<FloatType>(qw, qx, qy, qz).normalized();
    Eigen::Quaternion<FloatType> Q(qw, qx, qy, qz);

    Q.normalize();
    Eigen::Matrix<FloatType,3,1> T(tx, ty, tz);

    result.rotate(Q.toRotationMatrix());
    result.pretranslate(T);
    return result;

}


//https://math.stackexchange.com/questions/90081/quaternion-distance
template<typename FloatType>
FloatType computeQuaternionDiff(const  Eigen::Quaternion<FloatType>& q1, const  Eigen::Quaternion<FloatType>& q2){
    FloatType qd1 = q1.x()*q2.x() + q1.y()*q2.y() + q1.z() * q2.z() + q1.w()*q2.w();
    FloatType diff = 2*acos(abs(qd1));
    return diff;
}

//https://stackoverflow.com/questions/63341630/angle-axis-to-quaternion-using-eigen
template<typename FloatType>
Eigen::Quaternion<FloatType> createQuaternion(FloatType roll, FloatType pitch, FloatType yaw){

    Eigen::Matrix<FloatType,3,1> rotation(roll, pitch, yaw);
    FloatType angle = rotation.norm();
    Eigen::Matrix<FloatType,3,1> axis = rotation.normalized();
    return Eigen::Quaternion<FloatType> (Eigen::AngleAxisd(angle, axis));

}

template<typename FloatType>
Eigen::Quaternion<FloatType> createQuaternion( FloatType qw, FloatType qx, FloatType qy, FloatType qz){
    Eigen::Quaternion<FloatType> Q(qw, qx, qy, qz);
    Q.normalize();
    return Q;

}




autodiff::var
tf_opt_fnd(const autodiff::ArrayXvar &x) {

    auto tf1 = createSe3<autodiff::var>(x(0),x(1),x(2),x(3),x(4),x(5),x(6));

    auto tf_base = createSe3<autodiff::var>(0.1,0.2,0.15,0.57,0.57,0.0,0.0);

    Eigen::Quaternion<autodiff::var>  Q1 (tf1.rotation());
    Eigen::Quaternion<autodiff::var>  Q2 (tf_base.rotation());

    autodiff::var angle_error = computeQuaternionDiff(Q1 ,Q2);
    autodiff::var x_error = tf1.translation()[0] - tf_base.translation()[0];
    autodiff::var y_error = tf1.translation()[1]- tf_base.translation()[1];
    autodiff::var z_error = tf1.translation()[2] - tf_base.translation()[2];
    autodiff::var q_error = (x(3)*x(3) + x(4)*x(4)  + x(5)*x(5) + x(6)*x(6)) - 1.0;

    autodiff::var r = angle_error*angle_error +x_error*x_error + y_error*y_error + z_error*z_error + q_error*q_error;


    return r;
}


double
tf_opt_fn(const Eigen::VectorXd &x, Eigen::VectorXd *grad_out, void *opt_data) {

    autodiff::ArrayXvar xd = x.eval();

    autodiff::var y = tf_opt_fnd(xd);

    if (grad_out) {
        Eigen::VectorXd grad_tmp = autodiff::gradient(y, xd);

        *grad_out = grad_tmp;
    }

    return autodiff::val(y);
}



void test_1(){
    Eigen::Quaterniond q1 = Eigen::Quaterniond(0.35, 0.2, 0.3, 0.1).normalized();
    Eigen::Quaterniond q2 = Eigen::Quaterniond(-0.5, 0.4, -0.1, 0.2).normalized();

    Eigen::Matrix3d r1 = q1.toRotationMatrix();
    Eigen::Matrix3d r2 = q2.toRotationMatrix();

    std::cout << "r1: \n" << r1  << "\n";
    std::cout << "r2: \n" << r2  << "\n";

    Eigen::Vector3d t1 = Eigen::Vector3d(0.3, 0.1, 0.1);
    Eigen::Vector3d t2 = Eigen::Vector3d(-0.1, 0.5, 0.3);

    std::cout << "t1: \n" << t1  << "\n";
    std::cout << "t2: \n" << t2  << "\n";

    Eigen::Isometry3d T1 = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d T2 = Eigen::Isometry3d::Identity();




    auto mm = T1.rotation();

    T1.rotate(q1.toRotationMatrix());
    T1.pretranslate(t1);
    T2.rotate(q2.toRotationMatrix());
    T2.pretranslate(t2);
//    T2.translate(t2);

    auto T3 = createSe3(-0.1, 0.5, 0.3, -0.5, 0.4, -0.1, 0.2);

    std::cout << "T1: \n" << T1.matrix()  << "\n";
    std::cout << "T2: \n" << T2.matrix()  << "\n";
    std::cout << "T3: \n" << T3.matrix()  << "\n";

    return;


    q1.setIdentity();
    q2.setIdentity();

    float roll = 0.0, pitch = 0.5, yaw = -0.5;

    Eigen::Vector3d rotation(roll, pitch, yaw);
    double angle = rotation.norm();
    Eigen::Vector3d axis = rotation.normalized();
    q2 = Eigen::Quaterniond (Eigen::AngleAxisd(angle, axis));


    Eigen::Quaterniond qd1 = q2*q1.inverse();

    float qd2 = q1.x()*q2.x() + q1.y()*q2.y() + q1.z() * q2.z() + q1.w()*q2.w();

    float qd3 = 2*acos(abs(qd2));
    std::cout << "angle diff = " << qd3 << std::endl;
    {
        auto Q1 = createQuaternion(0.0,0.0, 0.0);
        auto Q2 = createQuaternion(0.0, -0.0, 0.6);
        std::cout << "(Q1 - Q2) diff = " << computeQuaternionDiff(Q1,Q2) << std::endl;

    }


}

void test_2() {
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

void test_3() {
    // x cannot be zero
    Eigen::VectorXd x(7);
    x << 1e-5, 1e-5, 1e-5,
    1.-1e-5, 1e-5, 1e-5, 1e-5;

    optim::algo_settings_t settings;

    settings.vals_bound = true;

    settings.lower_bounds = optim::ColVec_t::Zero(7);
    settings.lower_bounds(0) = -1.0;
    settings.lower_bounds(1) = -1.0;
    settings.lower_bounds(2) = -1.0;
    settings.lower_bounds(3) = -1.0;
    settings.lower_bounds(4) = -1.0;
    settings.lower_bounds(5) = -1.0;
    settings.lower_bounds(6) = -1.0;


    settings.upper_bounds = optim::ColVec_t::Zero(7);
    settings.upper_bounds(0) = 1.0;
    settings.upper_bounds(1) = 1.0;
    settings.upper_bounds(2) = 1.0;
    settings.upper_bounds(3) = 1.0;
    settings.upper_bounds(4) = 1.0;
    settings.upper_bounds(5) = 1.0;
    settings.upper_bounds(6) = 1.0;



    bool success = optim::bfgs(x, tf_opt_fn, nullptr,settings);

    if (success) {
        std::cout << "bfgs: reverse-mode autodiff test completed successfully.\n" << std::endl;
    } else {
        std::cout << "bfgs: reverse-mode autodiff test completed unsuccessfully.\n" << std::endl;
    }

    std::cout << "solution: x = \n" << x << std::endl;

}
int main(int argc, char** argv){

    test_1();
//    test_2();
    test_3();


}