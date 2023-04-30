//
// Created by waxz on 4/23/23.
//

#include "common/task.h"
#include "common/suspend.h"
#include "common/string_logger.h"

#include <observable/observable.hpp>
#include "coroutine/coroutine.h"


#include <chrono>
#include <sstream>
#include <ctime>
#include <iostream>
#include <iterator>
#include <locale>
#include <iostream>
#include <iomanip>
#include <ctime>

#include "common/signals.h"


#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include "math.h"

void preciseSleep(double seconds) {
    using namespace std;
    using namespace std::chrono;

    static double estimate = 5e-3;
    static double mean = 5e-3;
    static double m2 = 0;
    static int64_t count = 1;

    while (seconds > estimate) {
        auto start = high_resolution_clock::now();
        this_thread::sleep_for(milliseconds(1));
        auto end = high_resolution_clock::now();

        double observed = (end - start).count() / 1e9;
        seconds -= observed;

        ++count;
        double delta = observed - mean;
        mean += delta / count;
        m2   += delta * (observed - mean);
        double stddev = sqrt(m2 / (count - 1));
        estimate = mean + stddev;
    }

    // spin lock
    auto start = high_resolution_clock::now();
    while ((high_resolution_clock::now() - start).count() / 1e9 < seconds);
}


int main(int argc, char** argv){

    //void sigintHandler(int sig)
    std::atomic_bool program_run(true);
    auto my_handler = common::fnptr<void(int)>([&](int sig){ std::cout << "get sig " << sig;program_run = false;});

    common::set_signal_handler(my_handler);


    std::string time_str;
    u_int8_t j = 250;
    for(int i = 0 ; i < 20;i++){

        preciseSleep(1.0/20.0);
        formatTimestamp(common::FromUnixNow(),time_str);
        std::cout << int(j++) << " now = " << time_str << std::endl;
    }


    /*
     simple task scheduler:
     task store in single layer task pool, without dependency
     each task record its last run time point, trigger condition id current time is beyond task delay time. so in perfect condition, task runs at every  delay time;

     */



    /*
     task dependency

     scheduled task and reactive task

     - scheduled task is triggered by clock
     - reactive task is triggered by scheduler task or reactive task

     reactive pattern
     reactive can be implemented using observable signal or coroutine channel

     SchedTask_1 --> ReactTask_1
                 |
                 |-> ReactTask_2 --> ReactTask_3

     SchedTask_2  -> ReactTask_4


     each task may be blocked or failed, how to retry

     */
    common::TaskManager manager;
    std::string time_str_1,time_str_2;

    auto sub_1 = observable::subject<void(int)> { };
    std::string sub_1_time_str;
    sub_1.subscribe([&](auto const & msg) {

        common::Time recv_time = common::FromUnixNow();
        formatTimestamp(common::FromUnixNow(),sub_1_time_str);

//        MLOGI("%s : sub1 get msg %d", sub_1_time_str.c_str(),msg);

    });

    coroutine::Channel<int> channel;
    std::string rt_1_time_str;

    coroutine::routine_t rt1 = coroutine::create([&channel,&rt_1_time_str]() {
        while(true){
            int msg = channel.pop();


            common::Time recv_time = common::FromUnixNow();
            formatTimestamp(common::FromUnixNow(),rt_1_time_str);

            MLOGI("%s : sub1 get msg %d", rt_1_time_str.c_str(),msg);

            coroutine::yield();
        }
    });
    coroutine::resume(rt1);


    int cnt = 0;
    manager.addTask([&]{
        std::cout << "run 1 ";
        formatTimestamp(common::FromUnixNow(),time_str_1);
        std::cout << "now = " <<time_str_1 << std::endl;
        cnt++;
//        channel.push(cnt);
        sub_1.notify(cnt);
        return true;
    },10000, 1);

    manager.addTask([&]{
        std::cout << "run 2 ";
        formatTimestamp(common::FromUnixNow(),time_str_2);
        std::cout << "now = " <<time_str_2 << std::endl;
        return true;
    },20000, 2);


    common::Suspend suspend;

    common::Time timestamp = common::FromUnixNow();


    manager.intiTime();
    while (program_run){

        manager.call();

//        using namespace std::chrono_literals;

//        suspend.sleep(0.005);

    }


}