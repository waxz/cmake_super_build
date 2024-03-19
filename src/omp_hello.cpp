//
// Created by waxz on 8/28/22.
//

#include <iostream>
#include <thread>
#include "omp.h"


void avg_round_robin() {
    int N = 600000000;
    double tavg = 0;

    double timer_start = omp_get_wtime();
    omp_set_num_threads(16);
#pragma omp parallel
    {
        double avg;
        int id = omp_get_thread_num();
        int nthreads = omp_get_num_threads();

        for (int i = id; i < N; i+=nthreads) {
            avg += i;
        }
#pragma omp atomic
        tavg += avg;
    }
    double timer_elapsed = omp_get_wtime() - timer_start;
    tavg = tavg / N;

    std::cout << tavg << " took " << timer_elapsed << std::endl;
    //     1 threads took 2.1
    //     4 threads took 0.7
    //    48 threads took 0.65
}

void  avg_reduction() {
    int N = 600000000;
    int j = 0;
    double tavg = 0;

    double timer_start = omp_get_wtime();
    omp_set_num_threads(48);

#pragma omp parallel for reduction(+:tavg)
    for (j = 0; j < N; ++j) {
        tavg += j;
    }

    double timer_elapsed = omp_get_wtime() - timer_start;
    tavg = tavg / N;

    std::cout << tavg << " took " << timer_elapsed << std::endl;
    //     1 threads took 2.1
    //     4 threads took 0.69
    //    48 threads took 0.65
}


int main(int argc, char** argv){
#ifdef _OPENMP
    std::cout << "use _OPENMP" <<std::endl;

    std::cout << "device num: " << omp_get_num_devices() << std::endl;

#endif

    omp_set_dynamic(0);     // Explicitly disable dynamic teams
    omp_set_num_threads(4); // Use 4 threads for all consecutive parallel regions
    std::cout << "omp_get_max_threads() " << omp_get_max_threads() <<std::endl;

    std::cout << "omp_get_num_threads() " << omp_get_num_threads() <<std::endl;


    {

#pragma omp parallel
{
    int ID = omp_get_thread_num();
    std::cout << "hello " << ID << std::endl;
    std::cout << "world " << ID << std::endl;

}


        std::cout << "Exit region " << __LINE__   << std::endl;

#pragma omp parallel num_threads(5)
        {
            int ID = omp_get_thread_num();
            std::cout << "hello " << ID << std::endl;
            std::cout << "world " << ID << std::endl;

        }


        std::cout << "Exit region " << __LINE__   << std::endl;


    }




#pragma omp parallel proc_bind(spread) num_threads(16)
    {
#pragma omp single nowait
        printf("number of threads = %d\n"  ,omp_get_num_threads());
        int huh = 0;
        int M = 5;
        int N = 5;
        int j = 0;

#pragma omp for schedule(monotonic:dynamic,5) collapse(2)
        for(int i=0;i<N;i++)
            for (  j=0;j<M;j++){
                printf("omp_get_thread_num = %d\n"  ,omp_get_thread_num());

            }
    }


    const int N = 10;

    float a[N] = {0.0};
    float b[N] = {0.0};
    float c[N] = {0.0};

    return 0;





    float vec1[10] = {0,1,2,3,4,5,6,7,8,9};
    float vec2[10] = {0,1,2,3,4,5,6,7,8,9};
    float vec3[10] = {0,1,2,3,4,5,6,7,8,9};
    float vec4[10] = {0,1,2,3,4,5,6,7,8,9};
    float vec5[10];
    float vec6[10];

    float ratio = 1.0/3;

    for(int i = 0; i < 10; i++){
        vec1[i] = 0.2*i + 5.0;
        vec2[i] = 0.3*i + 0.3;
        vec3[i] = -0.5*i + 10.0;
        vec4[i] = ratio*(vec1[i] + vec2[i] + vec3[i]);
    }

#pragma omp parallel for
    for(int i = 0; i < 10; i++){
        vec5[i] = ratio*(vec1[i] + vec2[i] + vec3[i]);
    }

    for(int i = 0; i < 10; i++){
        printf("%d ::  %.3f, %.3f, %.3f, %.3f, %.3f\n", i ,  vec1[i] , vec2[i] , vec3[i] , vec4[i] , vec5[i] );
    }

//    avg_reduction();

//    avg_round_robin();

    return 0;



#pragma omp parallel
    {
#pragma omp critical
        {
            std::cout << "hello parallel " << std::this_thread::get_id() << std::endl;
            std::cout << "omp_get_num_threads() " << omp_get_num_threads() <<std::endl;

        }

    };


#pragma omp parallel for
    for(int i = 0; i < 5; i++){
#pragma omp critical
        {
            std::cout << "hello parallel for " << std::this_thread::get_id() << std::endl;

        }



    }


}