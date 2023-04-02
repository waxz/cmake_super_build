//
// Created by waxz on 23-4-2.
//

#include <thread>
#include <memory>
#include <vector>
#include <iostream>
#include <mutex>

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <sched.h>
#include <thread>
#include <hwloc.h>

//https://eli.thegreenplace.net/2016/c11-threads-affinity-and-hyperthreading/



void report_thread_cpu() {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    std::cout << "Thread id " << std::this_thread::get_id() << ", running on CPU "
              << sched_getcpu() << "\n";
}

void proc(void)
{
    using namespace std::chrono_literals;
    std::this_thread::sleep_for(5s);
}


void test_1(){
    constexpr unsigned num_threads = 4;
    // A mutex ensures orderly access to std::cout from multiple threads.
    std::mutex iomutex;
    std::vector<std::thread> threads(num_threads);
    for (unsigned i = 0; i < num_threads; ++i) {
        threads[i] = std::thread([&iomutex, i] {
            while (1) {
                {
                    // Use a lexical scope and lock_guard to safely lock the mutex only
                    // for the duration of std::cout usage.
                    std::lock_guard<std::mutex> iolock(iomutex);
                    std::cout << "Thread #" << i << ": on CPU " << sched_getcpu() << std::endl;

                }

                // Simulate important work done by the tread by sleeping for a bit...
                std::this_thread::sleep_for(std::chrono::milliseconds(900));
            }
        });
    }

    for (auto& t : threads) {
        t.join();
    }
}

void show_thread_id(){
    std::mutex iomutex;
    std::thread t = std::thread([&iomutex] {
        {
            std::lock_guard<std::mutex> iolock(iomutex);
            std::cout << "Thread: my id = " << std::this_thread::get_id() << "\n"
                      << "        my pthread id = " << pthread_self() << "\n";
        }
    });

    {
        std::lock_guard<std::mutex> iolock(iomutex);
        std::cout << "Launched t: id = " << t.get_id() << "\n"
                  << "            native_handle = " << t.native_handle() << "\n";
    }

    t.join();
}

void test_2(){
    constexpr unsigned num_threads = 4;
    // A mutex ensures orderly access to std::cout from multiple threads.
    std::mutex iomutex;
    std::vector<std::thread> threads(num_threads);
    for (unsigned i = 0; i < num_threads; ++i) {
        threads[i] = std::thread([&iomutex, i] {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            while (1) {
                {
                    // Use a lexical scope and lock_guard to safely lock the mutex only
                    // for the duration of std::cout usage.
                    std::lock_guard<std::mutex> iolock(iomutex);
                    std::cout << "Thread #" << i << ": on CPU " << sched_getcpu() << std::endl;
                }

                // Simulate important work done by the tread by sleeping for a bit...
                std::this_thread::sleep_for(std::chrono::milliseconds(900));
            }
        });

        // Create a cpu_set_t object representing a set of CPUs. Clear it and mark
        // only CPU i as set.
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(i, &cpuset);
        int rc = pthread_setaffinity_np(threads[i].native_handle(),
                                        sizeof(cpu_set_t), &cpuset);
        if (rc != 0) {
            std::cerr << "Error calling pthread_setaffinity_np: " << rc << "\n";
        }
    }

    for (auto& t : threads) {
        t.join();
    }
}

void test_hwloc(){
    report_thread_cpu();

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

    // Now pin the calling thread to the last PU (logical CPU) on the system.
    int num_objects_last_depth =
            hwloc_get_nbobjs_by_depth(topology, topodepth - 1);
    hwloc_obj_t pu = hwloc_get_obj_by_depth(topology, topodepth - 1,
                                            num_objects_last_depth - 1);
    std::cout << "topodepth = " << topodepth<< "\n";
    std::cout << "num_objects_last_depth = " << num_objects_last_depth<< "\n";

    std::cout << "Pinning thread to last PU; OS index = " << pu->os_index << "\n";
    if (hwloc_set_cpubind(topology, pu->cpuset, HWLOC_CPUBIND_THREAD) < 0) {
        std::cerr << "Error calling hwloc_set_cpubind\n";
        std::exit(1);
    }

    report_thread_cpu();




    constexpr unsigned num_threads = 4;
    // A mutex ensures orderly access to std::cout from multiple threads.
    std::mutex iomutex;
    std::vector<std::thread> threads(num_threads);
    for (unsigned i = 0; i < num_threads; ++i) {
        threads[i] = std::thread([&iomutex,&topology,&topodepth, &num_objects_last_depth, i] {


            hwloc_obj_t choose_pu = hwloc_get_obj_by_depth(topology, topodepth - 1,
                                                    i);

            char *str;
            hwloc_bitmap_asprintf(&str, choose_pu->cpuset);
            printf("cpumask [%ud] => hwloc [%s]\n", choose_pu->cpuset, str);
            std::cout << "Thread #" << i << ": on CPU " << sched_getcpu() << std::endl;

            try{
                if (hwloc_set_cpubind(topology, choose_pu->cpuset, HWLOC_CPUBIND_THREAD) < 0) {
                    std::cerr << "Error calling hwloc_set_cpubind\n";
                    return ;
                }
            }catch (...){
                std::cerr << "Error calling hwloc_set_cpubind\n";
                return ;
            }



            while (true){
                {
                    std::lock_guard<std::mutex> iolock(iomutex);
                    printf("cpumask [%ud] => hwloc [%s]\n", choose_pu->cpuset, str);
                    std::cout << "Thread #" << i << ": on CPU " << sched_getcpu() << std::endl;

                    report_thread_cpu();
                    std::cout << std::endl;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));

            }





        });

    }
    for (auto& t : threads) {
        t.join();
    }


}
int main(){
    test_hwloc();
}