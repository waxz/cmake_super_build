//
// Created by waxz on 2/24/24.
//
#include <thread>
#include <memory>
#include <vector>
#include <iostream>
#include <mutex>
#include <atomic>


#include <chrono>
#include <cstdlib>
#include <iostream>
#include <sched.h>
#include <thread>
#include <hwloc.h>

#include "common/functions.h"
#include "common/signals.h"
//https://eli.thegreenplace.net/2016/c11-threads-affinity-and-hyperthreading/

void show_cpu(){
    // Initialize hwloc.
    hwloc_topology_t topology;
    if (hwloc_topology_init(&topology) < 0) {
        std::cerr << "error calling hwloc_topology_init\n";
        std::exit(1);
    }

    if (hwloc_topology_load(topology) < 0) {
        std::cerr << "error calling hwloc_topology_load\n";
        std::exit(1);
    }

    // Describe the machine's HW topology, on each depth level.
    int topodepth = hwloc_topology_get_depth(topology);
    std::cout << "Depth of topology = " << topodepth << "\n";

    for (int depth = 0; depth < topodepth; depth++) {
        int num_objects = hwloc_get_nbobjs_by_depth(topology, depth);
        std::cout << "Level " << depth << ": " << num_objects << " objects\n";
        for (int i = 0; i < num_objects; i++) {
            hwloc_obj_t obj = hwloc_get_obj_by_depth(topology, depth, i);
            char s[512];
            hwloc_obj_type_snprintf(s, sizeof(s), obj, 1);
            std::cout << " " << i << ": " << s << "\n";
        }
    }
}




typedef std::lock_guard<std::mutex> TGuard;

int totalCores = 0;
bool g_program_run = true;

void runner(int idx) {
    int pinCore = idx % totalCores;
    std::cout << "Launching Running #" << idx << ". Pin to " << pinCore << std::endl;


    // Initialize hwloc.
    hwloc_topology_t topology;
    if (hwloc_topology_init(&topology) < 0) {
        std::cerr << "error calling hwloc_topology_init\n";
        std::exit(1);
    }

    if (hwloc_topology_load(topology) < 0) {
        std::cerr << "error calling hwloc_topology_load\n";
        std::exit(1);
    }

    // Describe the machine's HW topology, on each depth level.
    int topodepth = hwloc_topology_get_depth(topology);
    std::cout << "Depth of topology = " << topodepth << "\n";

    hwloc_obj_t choose_pu = hwloc_get_obj_by_depth(topology, topodepth - 1,
                                                   pinCore);

    if (hwloc_set_cpubind(topology, choose_pu->cpuset, HWLOC_CPUBIND_THREAD) < 0) {
        std::cerr << "Error calling hwloc_set_cpubind\n";
        return ;
    }


    while (g_program_run) {
        std::cout << "#" << idx << " Running: CPU " << sched_getcpu() << std::endl;
        sleep(1);
    }
}

int main(int argc, char **argv) {

    show_cpu();



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