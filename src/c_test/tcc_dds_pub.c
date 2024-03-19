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

    const char *writer_callback = "#include <ShmThroughput.h>"
                                  "size_t signal = 0;"
                                  "void prepare_sample(void* p){"
                                  "ThroughputModule_DataType_1048576* data = p;"
                                  "data->count =signal;"
                                  " }";



    void *writer_handler = ddsh_create_writer(0, "ThroughputModule_DataType_1048576", "RoundTripModule_Msg", "hello", writer_callback,
                                              NULL);


    if (!writer_handler) {
        printf("create writer_handler failed!\n");
        return 0;
    }

    MLOGD("writer_handler %p", writer_handler);

    size_t* writer_signal = ddsh_writer_get_symbol(writer_handler,"signal");

    if(writer_signal){
        MLOGD("get writer_signal %p, %i\n", writer_signal,*writer_signal);
    }



    MLOGD("%s", "ddsh_write_signal");
    ddsh_write_signal(writer_handler);

    MLOGD("%s", "ddsh_write_signal");

    int cnt = 0;

    long start, end;
    struct timeval timecheck;

    gettimeofday(&timecheck, NULL);
    start = (long)timecheck.tv_sec * 1000 + (long)timecheck.tv_usec / 1000;

    double cpu_time_used;

    while (program_run){
        *writer_signal = cnt++;
        ddsh_write_signal(writer_handler);


        MLOGI("writer_signal = %i",*writer_signal);
//        usleep(1000);  // 200ms

        gettimeofday(&timecheck, NULL);
        end = (long)timecheck.tv_sec * 1000 + (long)timecheck.tv_usec / 1000;


        printf("%ld milliseconds elapsed\n", (end - start));
        cpu_time_used =  (end - start)*0.001f  ;
        float rps = (float)cnt/cpu_time_used;
        MLOGI("RPS = %f/s\n", rps);
        MLOGI("BANDWIDTH = %f MB/s\n",  rps*1048576*(1e-6));

    }

    // create dds writer
    // bind tcc callback
    // get target symbol
    // run write

    // create dds reader
    // bind tcc callback
    // run read
    // get target symbol





    MLOGD("%s", "ddsh_stop_writer");

    ddsh_stop_writer(writer_handler);
    MLOGD("%s", "ddsh_stop_writer");



}