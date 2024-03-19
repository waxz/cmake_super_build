//
// Created by waxz on 9/7/23.
//

#include <sys/time.h>
#include "tcc_dds_helper.h"
#include "ShmThroughput.h"

#include "common/signals.h"
#include "common/string_logger.h"

#include "common/csleep.h"

static bool program_run = true;

void signal_handler(int sig) {
    printf("get signal %i\n", sig);
    program_run = false;
}

int main(int argc, char **argv) {

    set_signal_handler(signal_handler);


    const char *reader_callback = "#include <ShmThroughput.h>"
                                  "size_t signal = 0;"
                                  "void process_sample(void* p){"
                                  "ThroughputModule_DataType_1048576* data = p;"
//                                  "printf(\" process_sample: get count = %i \",data->count );"
                                  " signal = data->count;"
                                  "data->count = 0;"
                                  " }";


    void *reader_handler = ddsh_create_reader(0, "ThroughputModule_DataType_1048576", "RoundTripModule_Msg", "hello", reader_callback,
                                              NULL);

    if (!reader_handler) {
        printf("create reader_handler failed!\n");
        return 0;
    }
    MLOGD("reader_handler %p", reader_handler);

    size_t* reader_signal = ddsh_reader_get_symbol(reader_handler,"signal");


    if(reader_signal){
        MLOGD("get reader_signal %p, %i\n", reader_signal,*reader_signal);
    }


    int cnt = 0;

    long start, end;
    struct timeval timecheck;

    gettimeofday(&timecheck, NULL);
    start = (long)timecheck.tv_sec * 1000 + (long)timecheck.tv_usec / 1000;

    double cpu_time_used;

    while (program_run){

        int rt = ddsh_read_signal(reader_handler);
        cnt += rt;
        if(cnt == 0){
            gettimeofday(&timecheck, NULL);
            start = (long)timecheck.tv_sec * 1000 + (long)timecheck.tv_usec / 1000;

        }
        if(rt){
            MLOGI("reader_signal = %i",*reader_signal);
//        usleep(1000);  // 200ms

            gettimeofday(&timecheck, NULL);
            end = (long)timecheck.tv_sec * 1000 + (long)timecheck.tv_usec / 1000;


            printf("%ld milliseconds elapsed\n", (end - start));
            cpu_time_used =  (end - start)*0.001f  ;
            float rps = (float)cnt/cpu_time_used;
            MLOGI("RPS = %f/s\n", rps);
            MLOGI("BANDWIDTH = %f MB/s\n",  rps*1048576*(1e-6));

        }


    }

    // create dds writer
    // bind tcc callback
    // get target symbol
    // run write

    // create dds reader
    // bind tcc callback
    // run read
    // get target symbol





    MLOGD("%s", "ddsh_stop_reader");

    ddsh_stop_reader(reader_handler);
    MLOGD("%s", "ddsh_stop_reader");


}