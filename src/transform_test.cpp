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

#define  USE_CATCH 1

// fakeit
#if USE_FAKEIT
#include "catch_amalgamated.hpp"
#include "fakeit.hpp"
#endif
#if USE_CATCH
#include "catch2/catch_amalgamated.hpp"
#endif

#include <iostream>


#include "transform/transform.h"
#include "transform/eigen_transform.h"

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


TEST_CASE("transform error"){
#ifdef _OPENMP
    std::cout << "use _OPENMP" <<std::endl;

#endif


    double yaw = 0.5, pitch = 0.0, roll =0.0;
    double qw = 0.5, qx = 0.0 , qy = 0.0 , qz = 0.0;


    bool ok = false;
    transform::toQuaternion(qw,qx,qy,qz,yaw,pitch,roll);


    transform::toEulerAngle(yaw, pitch, roll, qw,qx,qy,qz);


    ok = transform::toEulerAngle(yaw, pitch, roll, qw,qx,qy,qz);
    std::cout << "normalise\n";
    std::cout << "ok : " << ok << "\n";

    std::cout << "qw: " << qw << "\n";
    std::cout << "qx: " << qx << "\n";
    std::cout << "qy: " << qy << "\n";
    std::cout << "qz: " << qz << "\n";
    std::cout << "yaw: " << yaw << "\n";
    std::cout << "pitch: " << pitch << "\n";
    std::cout << "roll: " << roll << "\n";



    // not normalise
    qw += 0.0001;
    ok = transform::toEulerAngle(yaw, pitch, roll, qw,qx,qy,qz);

    std::cout << "not normalise\n";
    std::cout << "ok : " << ok << "\n";

    std::cout << "qw: " << qw << "\n";
    std::cout << "qx: " << qx << "\n";
    std::cout << "qy: " << qy << "\n";
    std::cout << "qz: " << qz << "\n";
    std::cout << "yaw: " << yaw << "\n";
    std::cout << "pitch: " << pitch << "\n";
    std::cout << "roll: " << roll << "\n";


}

TEST_CASE("eigen error"){

    double yaw = 0.5, pitch = 0.0, roll =0.0;
    double qw = 0.5, qx = 0.0 , qy = 0.0 , qz = 0.0;

    double tx = 0.0, ty = 0.0, tz = 0.0;

    bool ok = false;
    transform::toQuaternion(qw,qx,qy,qz,yaw,pitch,roll);


    Eigen::Isometry3d tf1;
    tf1 = transform::createSe3(tx,ty,tz,qw,qx,qy,qz);
    std::cout << "normalise\n";

    std::cout << "qw: " << qw << "\n";
    std::cout << "qx: " << qx << "\n";
    std::cout << "qy: " << qy << "\n";
    std::cout << "qz: " << qz << "\n";
    std::cout << "yaw: " << yaw << "\n";
    std::cout << "pitch: " << pitch << "\n";
    std::cout << "roll: " << roll << "\n";
    std::cout << "tf1:\n" << tf1.matrix() << "\n";

    qw += 0.01;
    tf1 = transform::createSe3(tx,ty,tz,qw,qx,qy,qz);

    std::cout << "not normalise\n";
    std::cout << "qw: " << qw << "\n";
    std::cout << "qx: " << qx << "\n";
    std::cout << "qy: " << qy << "\n";
    std::cout << "qz: " << qz << "\n";
    std::cout << "yaw: " << yaw << "\n";
    std::cout << "pitch: " << pitch << "\n";
    std::cout << "roll: " << roll << "\n";
    std::cout << "tf1:\n" << tf1.matrix() << "\n";

    transform::extractSe3(tf1,tx,ty,tz,qw,qx,qy,qz);
    std::cout << "not normalise extractSe3\n";
    std::cout << "qw: " << qw << "\n";
    std::cout << "qx: " << qx << "\n";
    std::cout << "qy: " << qy << "\n";
    std::cout << "qz: " << qz << "\n";
    std::cout << "yaw: " << yaw << "\n";
    std::cout << "pitch: " << pitch << "\n";
    std::cout << "roll: " << roll << "\n";
    std::cout << "tf1:\n" << tf1.matrix() << "\n";
}

#if 0


int main (int argc, char** argv){







    auto t = Transform2d(1,0.0,0.4);
    auto t2 = Transform2d(1,0.0,0.0);

    auto t3 = t*t2;

    std::cout << t << std::endl;
    std::cout << t2 << std::endl;
    std::cout << t3 << std::endl;

    auto t4 = t.inverse();
    auto t5 = t4*t;

    std::cout <<"t: " << t << std::endl;

    std::cout <<"t4: " << t4 << std::endl;

    std::cout <<"t5: " << t5 << std::endl;

    return 0;






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
#endif
