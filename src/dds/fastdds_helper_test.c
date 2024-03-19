//
// Created by waxz on 9/12/23.
//
#include <sys/time.h> //timecheck,gettimeofday
#include <stdio.h>
#include <stdbool.h>
#include <time.h>
#include <errno.h>
#include <unistd.h> // usleep
//#include <stdint-gcc.h> //uint64_t,uint32_t,uint8_t
#include <stdint.h>//uint64_t,uint32_t,uint8_t
#include <threads.h> // thread_local


#include "common/signals.h"
#include "fastdds_helper.h"

#include "tinyalloc/tinyalloc.h"

typedef struct{
    size_t count;
    float weight;
    float size[3];
} BOX;


static bool program_run = true;

void signal_handler(int sig) {
    printf("get signal %i\n", sig);
    program_run = false;
}


typedef struct{
    uint64_t m_count;
    uint32_t m_payloadsize;
    uint8_t m_payload[1048564];
} DataType_1048576;


// 10000000 Byte = 10 MB
#define STATIC_MEMORY_SIZE 10000000
static unsigned char memory_pool[STATIC_MEMORY_SIZE];


int main(int argc, char** argv){
    set_signal_handler(signal_handler);


    if(argc <2){
        printf("usage:\n fastdds_helper_test config.toml\n");
        return 0;
    }

    bool ta_ok = ta_init(memory_pool,memory_pool + STATIC_MEMORY_SIZE,256,16,8);

    if(!ta_ok){
        printf("ta fail\n");
        return 0;
    }


    char* toml = argv[1];

    void* handler = dds_create_from_toml(toml);
    if(!handler){
        dds_clean_handler(handler);
        return 0;
    }
    printf("dds_create_from_toml ok\n");


    long start, end;
    struct timeval timecheck;

    gettimeofday(&timecheck, NULL);
    start = (long)timecheck.tv_sec * 1000 + (long)timecheck.tv_usec / 1000;

    double cpu_time_used;
    size_t cnt = 0;
    while (program_run){
        cnt++;
        dds_write_channel(handler, "channel_2");

        dds_read_channel(handler, "channel_1");
//        printf("p_box, %lu, %lu\n",p_box_write->count,  p_box_read->count);

        gettimeofday(&timecheck, NULL);
        end = (long)timecheck.tv_sec * 1000 + (long)timecheck.tv_usec / 1000;


//        printf("%ld milliseconds elapsed\n", (end - start));
        cpu_time_used =  (end - start)*0.001f  ;

//        if(cpu_time_used > 5.0){
//            cpu_time_used = 0.0;
//            cnt = 0;
//            start = (long)timecheck.tv_sec * 1000 + (long)timecheck.tv_usec / 1000;
//        }

        float rps = (float)cnt/cpu_time_used;
        if(cnt % 1000 == 10 && cpu_time_used > 1.0){
            printf("RPS = %f/s\n", rps);
            printf("BANDWIDTH = %f MB/s\n",  rps*1048576*(1e-6));
        }

//        usleep(10);
    }

    dds_clean_handler(handler);

}