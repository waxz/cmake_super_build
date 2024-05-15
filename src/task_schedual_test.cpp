//
// Created by waxz on 4/23/23.
//

#include "common/task.h"
#include "common/suspend.h"
#include "common/string_logger.h"
#include "common/functions.h"
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

inline float angle_normal(float angle){
    float normalizedAngle = angle - (std::ceil((angle + M_PI)/(2*M_PI))-1)*2*M_PI;  // (-Pi;Pi]:

    return normalizedAngle;
}

int main(int argc, char** argv){

    //void sigintHandler(int sig)
    std::atomic_bool program_run(true);
    auto my_handler = common::fnptr<void(int)>([&](int sig){ std::cout << "get sig " << sig;program_run = false;});

    set_signal_handler(my_handler);

    {
        const char fmt[] = R"(task_id: "%s"
param: "%s")";

        char buffer[200] = {0};
        sprintf(buffer,fmt,"pick_1","default");
        std::cout << "buffer: --\n" << buffer << "\n--\n";




        return 0;
    }


    {

        int64_t nano_epoch = 1712393215714294030;

        common::Time t1 = common::FromUniversal(nano_epoch);
        std::cout << "t1.stamp : " << t1 << std::endl;
        std::string o;
        common::formatTimestamp(t1,o);

        std::cout << "t1 o = " << o << std::endl;

        common::Time t2 = common::FromUnixNow();
        std::cout << "t2.stamp : " << t2 << std::endl;
        common::formatTimestamp(t2,o);

        std::cout << "t2 o = " << o << std::endl;

        auto d1 = t2 - t1;
        auto d2 = t1 - t2;
        auto t3 = t1 + d1;
        common::formatTimestamp(t3,o);

        std::cout << "t3 o = " << o << std::endl;

        std::cout <<"d1:" << common::ToMillSeconds(d1) << ", d2:" << common::ToMillSeconds(d2) << std::endl;


        std::chrono::nanoseconds dur(nano_epoch);

        std::chrono::time_point<std::chrono::system_clock> dt(dur);

        auto timestamp = std::chrono::time_point_cast<std::chrono::nanoseconds>(dt);

        {
            std::time_t time = std::chrono::system_clock::to_time_t(
                    std::chrono::time_point_cast<std::chrono::nanoseconds>(
                            std::chrono::system_clock::time_point{std::chrono::nanoseconds { nano_epoch}} ) );

            std::chrono::microseconds u_s = std::chrono::duration_cast<std::chrono::microseconds>(
                    timestamp.time_since_epoch() - std::chrono::duration_cast<std::chrono::seconds>(
                            timestamp.time_since_epoch()));
            std::chrono::nanoseconds n_s = std::chrono::duration_cast<std::chrono::nanoseconds>(
                    timestamp.time_since_epoch() - std::chrono::duration_cast<std::chrono::seconds>(
                            timestamp.time_since_epoch()));
            std::cout << std::put_time(std::localtime(&time), "%Y-%m-%d %H:%M:%S")
              << ":" << std::setw(6) << std::setfill('0') << u_s.count()<<" us, " << n_s.count() << " ns\n";
        }


    }




    {
        // cpu load is equal to work_load
        float work_load = 0.05;
        long work_time =  long(100000.0 * work_load);
        long sleep_time = 100000 - work_time;
        auto start_time = common::FromUnixNow();

        common::Suspend suspender;
        while (program_run){

            auto t1 = common::FromUnixNow();
            while (common::ToMicroSeconds(common::FromUnixNow() - t1) < work_time){

            }

            suspender.sleep_us(sleep_time);

            if(common::ToMicroSeconds(common::FromUnixNow() - start_time)  > 5000000){
                break;
            }
        }


    }
    common::TaskManager manager(20);
    std::string time_str;

    float fps = 100.0;
    manager.set_loop(fps,500);


    long base = 1e6/fps;

    for(int i = 0 ; i < 15;i++){

        for(int j = 0 ; j < i+2; j++){

            auto stamp = common::FromUnixNow();
            int run_count = 0;
            long max_error = 0;
            manager.add_task( "loop", [&time_str, i, j, stamp, run_count, max_error, base]() mutable {
                auto now = common::FromUnixNow();
                auto interval = common::ToMicroSeconds(now - stamp);
                stamp = now;

                formatTimestamp(now, time_str);
                std::cout << "task [" << i << "," << j << "]: run: " << time_str << "\n";


                if (run_count > 10) {

                    max_error = std::max(max_error, std::abs((i + 1) * base - interval));
                    std::cout << "task [" << i << "," << j << "]:interval: " << interval << ", max_error: " << max_error
                              << "\n";

                }
                run_count++;


                return true;
            }, i * 10.0);
        }

    }


    manager.add_task("lazy",[&] {
        std::cout << "run lazy task" << std::endl;
        auto s = manager.report();

        std::string ss = s.str();
        std::cout << ss << std::endl;
        return true;
    }, 30.0);


    while (program_run){
        manager.call();
    }
     auto s = manager.report();

    std::string ss = s.str();
    std::cout << ss << std::endl;

    return 0;

    u_int8_t j = 250;

    {
        common::Time t1_time_start = common::FromUnixNow();
        long max_error = 0;

        float sleep_s = 0.1;
        long sleep_us = sleep_s*1e6;
        for(int i = 0 ; i < 100;i++){

            preciseSleep(1.0/10.0);
            formatTimestamp(common::FromUnixNow(),time_str);
            std::cout << int(j++) << " now = " << time_str << std::endl;
            auto now = common::FromUnixNow();
            auto interval = common::ToMicroSeconds(now - t1_time_start);

            if(i > 10){

                max_error = std::max(max_error,  std::abs(sleep_us -interval) );
                std::cout << "interval: " << interval <<", max_error: " << max_error<<  "\n";

            }
            t1_time_start = now;

        }
    }



    common::LoopHelper loopHelper;
    loopHelper.set_fps(50.0);
    loopHelper.set_accuracy(100);

    {
        common::Time t1_time_start = common::FromUnixNow();

        long max_error = 0;
        long flag = 0;
        while (program_run){

            loopHelper.start();
            auto now = common::FromUnixNow();
            auto interval = common::ToMicroSeconds(now - t1_time_start);
            if(flag > 10){

                max_error = std::max(max_error,  std::abs(loopHelper.target_sleep_us -interval) );
                std::cout << "interval: " << interval <<", max_error: " << max_error<<  "\n";

            }
            t1_time_start = now;
            preciseSleep(1.0/100.0);
            loopHelper.sleep();
            flag++;
        }

    }
    std::cout<<"Exit"<< std::endl;

    return 0;

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

    std::string time_str_1,time_str_2;

#if 0
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


#endif

    int cnt = 0;
    common::Time t1_time_start = common::FromUnixNow();
    long max_error = 0;
    manager.add_task("task1",[&] {
        std::cout << "cnt = " << cnt << "\n";
        std::cout << "run 1 ";
        formatTimestamp(common::FromUnixNow(), time_str_1);
        cnt++;
        auto now = common::FromUnixNow();
        if (cnt > 10) {
            std::cout << "now = " << time_str_1 << std::endl;

            if (std::abs(common::ToMicroSeconds(now - t1_time_start) - 5000) > max_error) {
                max_error = std::abs(common::ToMicroSeconds(now - t1_time_start) - 5000);
            }
            std::cout << "interval: " << common::ToMicroSeconds(now - t1_time_start) << "\n" << "max_error: "
                      << max_error << "\n";

        }
        t1_time_start = now;
        if (cnt == 10) {
            manager.add_task("task3",[&] {
                std::cout << "run 3 ";
                formatTimestamp(common::FromUnixNow(), time_str_1);
                std::cout << "now = " << time_str_1 << std::endl;


                return cnt < 30;
            }, 100.0);
        }
//        channel.push(cnt);
//        sub_1.notify(cnt);
        return true;
    }, 500.0);

    manager.add_task("task2",[&] {
        std::cout << "run 2 ";
        formatTimestamp(common::FromUnixNow(), time_str_2);
        std::cout << "now = " << time_str_2 << std::endl;
        return true;
    }, 200.0);


    common::Suspend suspend;

    common::Time timestamp = common::FromUnixNow();

    {
        manager.add_task("temptask1",[] {
            std::cout << "run temp 1 ";
            return true;
        }, 100.0);
    }
    {
        auto f_2 = []{
            std::cout << "run temp 2 ";
            return true;
        };

        manager.add_task("f2",f_2, 100.0);
    }

    while (program_run){

        manager.call();

//        using namespace std::chrono_literals;

//        suspend.sleep(0.005);

    }


}