//
// Created by waxz on 23-3-4.
//

#include <oneapi/tbb.h>

#include <iostream>
#include <vector>
#include <omp.h>
#include "common/clock_time.h"
#include "common/string_logger.h"

// 10000000 Byte = 10 MB
// 50000000 Byte = 50 MB

#define STATIC_MEMORY_SIZE 100000000
static unsigned char float_memory_pool[STATIC_MEMORY_SIZE];
static unsigned char char_memory_pool[STATIC_MEMORY_SIZE];

struct AppBody{

    void operator()(const tbb::blocked_range<size_t>& r)const {

        for(size_t  i = r.begin(); i != r.end(); i++){
            std::cout << "hello app " << i << std::endl;

        }
    }


};


#define TIMED(N) for(int ttt = 0 ; ttt < N; ttt++ )
int main(int argc, char** argv){




#ifdef _OPENMP

//    omp_set_dynamic(0);     // Explicitly disable dynamic teams
    omp_set_num_threads(8); // Use 4 threads for all consecutive parallel regions
    printf("use _OPENMP thread num: %i\n",  omp_get_num_threads());
#pragma omp parallel
    {
        int ID = omp_get_thread_num();
        printf("ID = %i\n", ID);

    }
#endif


    int point_num = 1920*800*3;
    float* float_buffer_1 = (float *) (float_memory_pool);
    float* float_buffer_2 =  (float *)  char_memory_pool;

    std::cout
            <<   "float_buffer_1: [" << float_buffer_1 << "]\n"
              << "float_buffer_2: [" << float_buffer_2 << "]\n";

    {
        common::Time t1 = common::FromUnixNow();

        TIMED(100){
            int i = 0 ;

            for(i = 0 ; i < point_num;i++){
                float_buffer_1[i*3 + 0 ] = *(float* )(float_buffer_2 + i*3 + 0);
                float_buffer_1[i*3 + 1 ] = *(float* )(float_buffer_2 + i*3 + 1);
                float_buffer_1[i*3 + 2 ] = *(float* )(float_buffer_2 + i*3 + 2);
            }

        }
        MLOGI("use time: %u ms\n", common::ToMillSeconds(common::FromUnixNow() - t1) );
    }
    {
        common::Time t1 = common::FromUnixNow();

        TIMED(100){
            int i = 0 ;
#ifdef _OPENMP
#pragma omp parallel for default(none) private(i) shared(point_num, float_buffer_1,float_buffer_2 ) schedule(static)

#endif
            for(i = 0 ; i < point_num;i++){
                float_buffer_1[i*3 + 0 ] = *(float* )(float_buffer_2 + i*3 + 0);
                float_buffer_1[i*3 + 1 ] = *(float* )(float_buffer_2 + i*3 + 1);
                float_buffer_1[i*3 + 2 ] = *(float* )(float_buffer_2 + i*3 + 2);
            }

        }
        MLOGI("use time: %u ms\n", common::ToMillSeconds(common::FromUnixNow() - t1) );
    }

    using namespace oneapi::tbb;


    {
        common::Time t1 = common::FromUnixNow();

        TIMED(100){
            parallel_for(0,point_num, [float_buffer_1, float_buffer_2](const auto& i){
                float_buffer_1[i*3 + 0 ] = *(float* )(float_buffer_2 + i*3 + 0);
                float_buffer_1[i*3 + 1 ] = *(float* )(float_buffer_2 + i*3 + 1);
                float_buffer_1[i*3 + 2 ] = *(float* )(float_buffer_2 + i*3 + 2);

            });


        }
        MLOGI("use time: %u ms\n", common::ToMillSeconds(common::FromUnixNow() - t1) );

    }
    {
        common::Time t1 = common::FromUnixNow();

        TIMED(100){

            oneapi::tbb::parallel_for(
                    oneapi::tbb::blocked_range<std::size_t>(0, point_num, 10240),
                    [&](oneapi::tbb::blocked_range<std::size_t>& r) {
                        for (std::size_t i = r.begin(); i != r.end(); ++i) {
                            float_buffer_1[i*3 + 0 ] = *(float* )(float_buffer_2 + i*3 + 0);
                            float_buffer_1[i*3 + 1 ] = *(float* )(float_buffer_2 + i*3 + 1);
                            float_buffer_1[i*3 + 2 ] = *(float* )(float_buffer_2 + i*3 + 2);
                        }
                    },
                    oneapi::tbb::simple_partitioner());
        }
        MLOGI("use time: %u ms\n", common::ToMillSeconds(common::FromUnixNow() - t1) );

    }

    {
        common::Time t1 = common::FromUnixNow();

        TIMED(100){

            oneapi::tbb::parallel_for(
                    oneapi::tbb::blocked_range<std::size_t>(0, point_num, point_num/5),
                    [&](oneapi::tbb::blocked_range<std::size_t>& r) {
                        for (std::size_t i = r.begin(); i != r.end(); ++i) {
                            float_buffer_1[i*3 + 0 ] = *(float* )(float_buffer_2 + i*3 + 0);
                            float_buffer_1[i*3 + 1 ] = *(float* )(float_buffer_2 + i*3 + 1);
                            float_buffer_1[i*3 + 2 ] = *(float* )(float_buffer_2 + i*3 + 2);
                        }
                    },
                    oneapi::tbb::simple_partitioner());
        }
        MLOGI("use time: %u ms\n", common::ToMillSeconds(common::FromUnixNow() - t1) );

    }
    {
        common::Time t1 = common::FromUnixNow();

        TIMED(100){

            oneapi::tbb::parallel_for(
                    oneapi::tbb::blocked_range<std::size_t>(0, point_num, 10240),
                    [&](oneapi::tbb::blocked_range<std::size_t>& r) {
                        for (std::size_t i = r.begin(); i != r.end(); ++i) {
                            float_buffer_1[i*3 + 0 ] = *(float* )(float_buffer_2 + i*3 + 0);
                            float_buffer_1[i*3 + 1 ] = *(float* )(float_buffer_2 + i*3 + 1);
                            float_buffer_1[i*3 + 2 ] = *(float* )(float_buffer_2 + i*3 + 2);
                        }
                    },
                    oneapi::tbb::auto_partitioner());
        }
        MLOGI("use time: %u ms\n", common::ToMillSeconds(common::FromUnixNow() - t1) );

    }
    static oneapi::tbb::affinity_partitioner g_ap;

    {
        common::Time t1 = common::FromUnixNow();

        TIMED(100){

            oneapi::tbb::parallel_for(
                    oneapi::tbb::blocked_range<std::size_t>(0, point_num, 10240),
                    [&](oneapi::tbb::blocked_range<std::size_t>& r) {
                        for (std::size_t i = r.begin(); i != r.end(); ++i) {
                            float_buffer_1[i*3 + 0 ] = *(float* )(float_buffer_2 + i*3 + 0);
                            float_buffer_1[i*3 + 1 ] = *(float* )(float_buffer_2 + i*3 + 1);
                            float_buffer_1[i*3 + 2 ] = *(float* )(float_buffer_2 + i*3 + 2);
                        }
                    },g_ap);
        }
        MLOGI("use time: %u ms\n", common::ToMillSeconds(common::FromUnixNow() - t1) );

    }

    return 0;

    parallel_for(0,5, [](const auto& i){

        std::cout << "hello lambda " << i << std::endl;

    });

    AppBody app;
    blocked_range<size_t> r(0,5);
    parallel_for(r , app);




}