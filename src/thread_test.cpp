//
// Created by waxz on 4/12/23.
//
#include <thread>
#include <iostream>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <sched.h>
#include <thread>

#include <pthread.h>
#include <hwloc.h>

#include <sys/time.h>
#include <sys/resource.h>


//note: setpriority does not work with root permission

void test_1(){
    setpriority(PRIO_PROCESS, 0, -20);
}

void test_2(){

    int policy = SCHED_FIFO; // use FIFO scheduling policy
    struct sched_param param;
    param.sched_priority = 10; // set the priority to 99 (highest)

    // set the scheduling policy and priority of the thread
    pthread_setschedparam(pthread_self(), policy, &param);
    int canSetRealTimeThreadPriority = (pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) == 0);
    std::cout << "canSetRealTimeThreadPriority " << canSetRealTimeThreadPriority << std::endl;

}
int main(int argc, char** argv){
    test_2();
}