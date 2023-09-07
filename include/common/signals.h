//
// Created by waxz on 4/28/23.
//

#ifndef KACANOPEN_SIGNALS_H
#define KACANOPEN_SIGNALS_H
//#include <functional>
#include <signal.h>
//#include "functions.h"


#ifdef __cplusplus
extern "C" {
#endif


/// \param my_handler\n
/// C++ usage:\n
///std::atomic_bool program_run(true);\n
///auto my_handler = common::fnptr<void(int)>([&](int sig){ std::cout << "get sig " << sig;program_run = false;});\n
///common::set_signal_handler(my_handler);\n
/// C usage:\n
///static bool program_run = true;\n
///void signal_handler(int sig)\n
///{\n
///    printf("get signal %i\n",sig);\n
///    program_run = false;\n
///}\n



void set_signal_handler(void (*my_handler)(int)){

    struct sigaction sigIntHandler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigIntHandler.sa_handler = my_handler;

    sigaction(SIGINT, &sigIntHandler, NULL);
    sigaction(SIGTERM, &sigIntHandler, NULL);
}

#ifdef __cplusplus
}
#endif

#endif //KACANOPEN_SIGNALS_H
