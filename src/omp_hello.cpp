//
// Created by waxz on 8/28/22.
//

#include <iostream>
#include <thread>
#include "omp.h"
int main(int argc, char** argv){
#ifdef _OPENMP
    std::cout << "use _OPENMP" <<std::endl;

#endif
    omp_set_dynamic(0);     // Explicitly disable dynamic teams
    omp_set_num_threads(4); // Use 4 threads for all consecutive parallel regions
    std::cout << "omp_get_max_threads() " << omp_get_max_threads() <<std::endl;

    std::cout << "omp_get_num_threads() " << omp_get_num_threads() <<std::endl;

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