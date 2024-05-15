//
// Created by waxz on 2/24/24.
//
#include <thread>
#include <mutex>
#include <vector>
#include <unistd.h>
#include <string.h>
#include <iostream>
#include <atomic>
#include "common/functions.h"
#include "common/signals.h"

#include <pthread.h>

#define COMPILER_BARRIER() asm volatile("" ::: "memory")
#define RELEASE_FENCE() asm volatile("lwsync" ::: "memory")


typedef std::lock_guard<std::mutex> TGuard;

int totalCores = 0;
bool g_program_run = true;

int g_int_array[1000] = {0};


void set_value(){
    int a = 0;
    a = 1;
    a+=1;
    std::atomic_signal_fence(std::memory_order_acq_rel);

}

void runner(int idx) {
    int pinCore = idx % totalCores;
    std::cout << "Launching Running #" << idx << ". Pin to " << pinCore << std::endl;

    pthread_t self = pthread_self();
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(pinCore, &cpuset);

    int rc = pthread_setaffinity_np(self, sizeof(cpu_set_t), &cpuset);
    if (rc != 0) {
        std::cerr << "Failed to pin core: " << strerror(errno) << std::endl;
        exit(1);
    }

    while (g_program_run) {
        std::cout << "#" << idx << " Running: CPU " << sched_getcpu() << std::endl;
        sleep(1);

        int a = 1;
        a = 2;

        COMPILER_BARRIER();

        a+=1;
        printf("a = %i\n", a);
    }
}

int main(int argc, char **argv) {

    std::atomic_bool program_run(true);
    auto my_handler = common::fnptr<void(int)>([&](int sig){ std::cout << "get sig " << sig;program_run = false;g_program_run=false;});
    set_signal_handler(my_handler);



    totalCores = std::thread::hardware_concurrency();
    std::cout << "Starting. " << totalCores << " cores" << std::endl;
    std::vector<std::thread> threadList;

    const int N = 6;
    for (int i = 0; i < N; i++) {
        std::thread th = std::thread(runner, i);
        threadList.push_back(std::move(th));
    }

    for (auto &th : threadList) {
        if (th.joinable())
            th.join();
    }

    std::cout << "Complete" << std::endl;
}