//
// Created by waxz on 22-9-27.
//


#include <iostream>
#include <cmath>
#include <vector>
#include <thread>
#include "omp.h"

#include "common/clock_time.h"

#include "nlohmann/json.hpp"
struct Transform2d{
    float matrix[3][3] = {};
    Transform2d(float x=0.0, float y=0.0,float yaw=0.0){
        // Calculate rotation about z axis
        /*
                 cos(yaw),   -sin(yaw),      0,
                 sin(yaw),   cos(yaw),       0,
                 0,          0,              1
             */
        matrix[0][0] = cos(yaw);
        matrix[0][1]  = -sin(yaw);
        matrix[1][0] = sin(yaw);
        matrix[1][1]  = cos(yaw);

        matrix[0][2]  = x;
        matrix[1][2]  = y;
        matrix[2][2]  = 1.0;

    }
    Transform2d operator*(const Transform2d& rhv){
        Transform2d result;

        auto& a = this->matrix;
        auto& b = rhv.matrix;
        auto& c = result.matrix;
        // Calculate the j-th column of the result in-place (in B) using the helper array
        for(int i=0 ; i<3 ; i++)
        {
            for(int j=0 ; j<3 ; j++)
            {

                c[i][j]=0;
                for(int k=0 ; k<3 ; k++)
                {
                    c[i][j]+=a[i][k]*b[k][j];
                    //--^-- should be k
                }
            }
        }
        return result;

    }
    void mul_v1(const std::vector<float>& points, std::vector<float>& result){

        /*
     r00 r01 r02 tx     x0        x1
     r10 r11 r12 ty  X  y0   =>   y1
     r20 r21 r22 tz     z0        z1
     0   0   0   1      1         1
    */

        if(points.size()%2 != 0 ){
            std::cerr << __FUNCTION__ << " ERROR : " << " points.size() = " << points.size() << std::endl;

        }
        result.resize(points.size());

        float r00 = this->matrix[0][0];
        float r01 = this->matrix[0][1];

        float r10 = this->matrix[1][0];
        float r11 = this->matrix[1][1];

        float tx = this->matrix[0][2];
        float ty = this->matrix[1][2];

        int n_dim = points.size()/2;

        const float *p_data_x = &(points[0]);
        const float *p_data_y = p_data_x + n_dim;

        float *p_x = &(result[0]);
        float *p_y = p_x + n_dim;

        for (int i = 0; i < n_dim; i++) {
            p_x[i] = r00 * p_data_x[i] + r01 * p_data_y[i] + tx;
            p_y[i] = r10 * p_data_x[i] + r11 * p_data_y[i] + ty;
        }

    }

    void mul_v2(const std::vector<float>& points, std::vector<float>& result){

        /*
     r00 r01 r02 tx     x0        x1
     r10 r11 r12 ty  X  y0   =>   y1
     r20 r21 r22 tz     z0        z1
     0   0   0   1      1         1
    */

        if(points.size()%2 != 0 ){
            std::cerr << __FUNCTION__ << " ERROR : " << " points.size() = " << points.size() << std::endl;

        }
        result.resize(points.size());

        float r00 = this->matrix[0][0];
        float r01 = this->matrix[0][1];

        float r10 = this->matrix[1][0];
        float r11 = this->matrix[1][1];

        float tx = this->matrix[0][2];
        float ty = this->matrix[1][2];

        int n_dim = points.size()/2;

        const float *p_data_x = &(points[0]);
        const float *p_data_y = p_data_x + n_dim;

        float *p_x = &(result[0]);
        float *p_y = p_x + n_dim;

        for (int i = 0; i < n_dim; i++) {
            p_x[i + i] = r00 * p_data_x[i+i] + r01 * p_data_x[i+i+1] + tx;
            p_x[i + i + 1] = r10 * p_data_x[i+i] + r11 * p_data_x[i+i+1] + ty;
        }

    }
    void mul_v3(const std::vector<float>& points, std::vector<float>& result){

        /*
     r00 r01 r02 tx     x0        x1
     r10 r11 r12 ty  X  y0   =>   y1
     r20 r21 r22 tz     z0        z1
     0   0   0   1      1         1
    */

        if(points.size()%2 != 0 ){
            std::cerr << __FUNCTION__ << " ERROR : " << " points.size() = " << points.size() << std::endl;

        }
        result.resize(points.size());

        float r00 = this->matrix[0][0];
        float r01 = this->matrix[0][1];

        float r10 = this->matrix[1][0];
        float r11 = this->matrix[1][1];

        float tx = this->matrix[0][2];
        float ty = this->matrix[1][2];

        int n_dim = points.size()/2;

        const float *p_data_x = &(points[0]);
        const float *p_data_y = p_data_x + n_dim;

        float *p_x = &(result[0]);
        float *p_y = p_x + n_dim;
#ifdef _OPENMP
#pragma omp simd
#endif
        for (int i = 0; i < n_dim; i++) {
            p_x[i] = r00 * p_data_x[i] + r01 * p_data_y[i] + tx;
            p_y[i] = r10 * p_data_x[i] + r11 * p_data_y[i] + ty;
        }

    }

    void mul_v4(const std::vector<float>& points, std::vector<float>& result){

        /*
     r00 r01 r02 tx     x0        x1
     r10 r11 r12 ty  X  y0   =>   y1
     r20 r21 r22 tz     z0        z1
     0   0   0   1      1         1
    */

        if(points.size()%2 != 0 ){
            std::cerr << __FUNCTION__ << " ERROR : " << " points.size() = " << points.size() << std::endl;

        }
        result.resize(points.size());

        float r00 = this->matrix[0][0];
        float r01 = this->matrix[0][1];

        float r10 = this->matrix[1][0];
        float r11 = this->matrix[1][1];

        float tx = this->matrix[0][2];
        float ty = this->matrix[1][2];

        int n_dim = points.size()/2;

        const float *p_data_x = &(points[0]);
        const float *p_data_y = p_data_x + n_dim;

        float *p_x = &(result[0]);
        float *p_y = p_x + n_dim;
#ifdef _OPENMP
#pragma omp simd
#endif
        for (int i = 0; i < n_dim; i++) {
            p_x[i + i] = r00 * p_data_x[i+i] + r01 * p_data_x[i+i+1] + tx;
            p_x[i + i + 1] = r10 * p_data_x[i+i] + r11 * p_data_x[i+i+1] + ty;
        }

    }

    void mul_v5(const std::vector<float>& points, std::vector<std::array<float,2>>& result){

        /*
     r00 r01 r02 tx     x0        x1
     r10 r11 r12 ty  X  y0   =>   y1
     r20 r21 r22 tz     z0        z1
     0   0   0   1      1         1
    */

        if(points.size()%2 != 0 ){
            std::cerr << __FUNCTION__ << " ERROR : " << " points.size() = " << points.size() << std::endl;

        }
        result.resize(points.size());

        float r00 = this->matrix[0][0];
        float r01 = this->matrix[0][1];

        float r10 = this->matrix[1][0];
        float r11 = this->matrix[1][1];

        float tx = this->matrix[0][2];
        float ty = this->matrix[1][2];

        int n_dim = points.size()/2;

        const float *p_data_x = &(points[0]);
        const float *p_data_y = p_data_x + n_dim;


        for (int i = 0; i < n_dim; i++) {
            result[i] [0]= r00 * p_data_x[i+i] + r01 * p_data_x[i+i+1] + tx;
            result[i] [1]= r10 * p_data_x[i+i] + r11 * p_data_x[i+i+1] + ty;
        }

    }


};
std::ostream& operator <<(std::ostream& out,const Transform2d& rhv ){
    out << "Transform2d:\n";
//    out.unsetf ( std::ios::floatfield );                // floatfield not set
    out.precision(5);
    out.setf( std::ios::fixed, std:: ios::floatfield ); // floatfield set to fixed

    out << "[" << rhv.matrix[0][0] << ", " << rhv.matrix[0][1] << ", " << rhv.matrix[0][2]<<"\n"
        << " " << rhv.matrix[1][0] << ", " << rhv.matrix[1][1] << ", " << rhv.matrix[1][2]<<"\n"
        <<" " << rhv.matrix[2][0] << ", " << rhv.matrix[2][1] << ", " << rhv.matrix[2][2]<<"]\n"
        << std::endl;
    out.unsetf ( std::ios::floatfield );                // floatfield not set

    return out;
}


template <typename T>
void lin_space(T start, T inc, int n, std::vector<T>& data){
    data.resize(n);
    std::for_each(data.begin(),data.end(),[&, inc = inc,start = start](auto& e)mutable {
        e = start;
        start += inc;
    });
}

template <typename T, typename F, typename ...Args>
void func_map( std::vector<T>& data_in, std::vector<T>& data_out,const  F& func, const Args& ... args){
    data_out.resize(data_in.size());


    std::for_each(data_out.begin(),data_out.end(),[&,i = 0 ](auto& e)mutable {
        e = func(data_in[i],   std::forward<const Args& ...>(args)...);
        i ++;
    });
}

template <typename T, typename F, typename ...Args>
void func_map_v2( const  F& func,  std::vector<T>& data_a,  Args& ... args){

    size_t arr[] = {  args.size()...  };
    int sz = sizeof...(args);
    for(int i = 0 ; i < sz;i++){
        std::cout << __FUNCTION__  << " check size: " << arr[i]<< "\n";

    }


    int N= data_a.size();
    for(int i = 0; i < N;i++){
        func(i, data_a[i],   args[i]...  );
    }
}


template <typename F, typename ...Args>
void func_map_v3( const  F& func,   Args& ... args){

    size_t arr[] = {  args.size()...  };
    int sz = sizeof...(args);
    for(int i = 0 ; i < sz;i++){
        std::cout << __FUNCTION__  << " check size: " << arr[i]<< "\n";
    }
    size_t N= arr[0];
    for(int i = 0; i < N;i++){
        func(i, args[i]...  );
    }
}
template <typename F, typename ...Args>
void func_map_v4( const  F& func,   Args& ... args){

    size_t arr[] = {  args.size()...  };
    const int sz = sizeof...(args);
    if(sz > 0){
        size_t N= arr[0];

        for(int i = 1 ; i < sz;i++){
            if(arr[i] == 0 || arr[i]%N != 0){
                std::cout << __FUNCTION__  << " check size error: " << arr[i]<< "\n";
            }
        }

        for(int i = 0; i < N;i++){
            func(args[i]...  );
        }
    }

}


struct Point{
    float x;
    float y;
};


void print_(){
    std::cout << "empty" << std::endl;
}

template <typename T, typename ...Args>
void print_(const T& head, Args...args){
    std::cout << "parameter: " << head << std::endl;
    print_(args...);
}

template<typename T>
int check_size_(T t){
    return t.size();
}

template <typename ...Args>
void check_size_expand(Args...args){
    int arr[] = { (check_size_(args)) ... };
    int sz = sizeof...(args);
    for(int i = 0 ; i < sz;i++){
        std::cout << "check size: " << arr[i]<< "\n";

    }

}




int main (int argc, char** argv){

#ifdef _OPENMP
    std::cout << "use _OPENMP" <<std::endl;

#endif


    auto t = Transform2d(1,0.0,0.0);
    auto t2 = Transform2d(1,0.0,0.0);

    auto t3 = t*t2;

    std::cout << t << std::endl;
    std::cout << t2 << std::endl;
    std::cout << t3 << std::endl;


    int N= 50000;
    std::vector<float> point_v1(2*N);
    std::vector<float> point_v2(2*N);
    std::vector<std::array<float,2>> point_v3(N);

    for(int i = 0 ;i < N;i++){
        point_v1[i+i] = i;
        point_v1[i+i+1] = i+1;

    }

    auto start1 = common::FromUnixNow();
    int N_run = 1000;

    for(int i = 0; i<N_run;i++){
        t3.mul_v1(point_v1,point_v2);

    }
    auto end1 = common::FromUnixNow();

    auto start2 = common::FromUnixNow();

    for(int i = 0; i<N_run;i++){
        t3.mul_v2(point_v1,point_v2);

    }
    auto end2 = common::FromUnixNow();

    auto start3 = common::FromUnixNow();

    for(int i = 0; i<N_run;i++){
        t3.mul_v3(point_v1,point_v2);

    }
    auto end3 = common::FromUnixNow();


    auto start4 = common::FromUnixNow();

    for(int i = 0; i<N_run;i++){
        t3.mul_v4(point_v1,point_v2);

    }
    auto end4 = common::FromUnixNow();
    auto start5 = common::FromUnixNow();

    for(int i = 0; i<N_run;i++){
        t3.mul_v5(point_v1,point_v3);

    }
    auto end5 = common::FromUnixNow();

    double dur1 =  common::ToMillSeconds(end1 - start1)  ;
    double dur2=  common::ToMillSeconds(end2 - start2)  ;
    double dur3=  common::ToMillSeconds(end3 - start3)  ;
    double dur4=  common::ToMillSeconds(end4 - start4)  ;
    double dur5=  common::ToMillSeconds(end5 - start5)  ;

    std::cout << "dur1: " << dur1 << ", dur2: " << dur2 << std::endl;

    std::cout << "dur3: " << dur3 << ", dur4: " << dur4 << ", dur5: " << dur5 << std::endl;



    std::vector<float> r;
    lin_space(-3.14f,2*3.14f/100,100,r);
    std::vector<float> r2;

    func_map(r,r2,[](auto& e,auto& bias){

        return cos(e) + bias;
    },10.0);
    func_map(r,r2,cos);
    std::vector<float> xy_flat(200);

    std::vector<std::array<float,2>> xy_v(2*100);
    std::vector<Point> xy_v2(100);

    func_map_v2([](int i, auto&a, auto&b, auto&xy,auto& xy_p){
        b = cos(a);
        xy[0] = cos(a);
        xy[1]= sin(a);
        xy_p.x = cos(a);
        xy_p.y = sin(a);

    },r,r2,xy_v,xy_v2);

    func_map_v3([](int i, auto&a, auto&b, auto&xy,auto& xy_p,auto& f){
        b = cos(a);
        xy[0] = cos(a);
        xy[1]= sin(a);
        xy_p.x = cos(a);
        xy_p.y = sin(a);
        f = cos(a);
        (&f)[1] = sin(a);


    },r,r2,xy_v,xy_v2,xy_flat);


    std::cout << "\nr:\n";
    for(auto e:r){
        std::cout << e << ", ";
    }
    std::cout << "\nr2:\n";
    for(auto e:r2){
        std::cout << e << ", ";
    }
    std::cout << std::endl;

    nlohmann::json  s1 = xy_v;
    std::cout << "s1:\n" << s1 << std::endl;


    std::cout << "test ..arg\n";
    print_(1,2,3);
    check_size_expand(r,r2,xy_v);

}