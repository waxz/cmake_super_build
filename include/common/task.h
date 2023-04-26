//
// Created by waxz on 22-11-14.
//

#ifndef COMMON_TASK_H
#define COMMON_TASK_H

#include <functional>
#include <vector>
#include <algorithm>

#include "clock_time.h"

namespace common{
    struct TaskManager{
        struct Task{
            //  lower Task.prio value means higher priority.
            int prio = 10;
            bool valid = true;
            common::Time time;
            long delay_us = 100;
            std::function<bool()> func;
            Task(const std::function<bool()>& func_,long delay_us_ = 100,int prio_ = 10):prio(prio_),valid(true), time(common::FromUnixNow()),delay_us(delay_us_),func(func_){
            }
            Task(const Task& rhv){
                valid = rhv.valid;
                prio = rhv.prio;
                time = rhv.time;
                delay_us = rhv.delay_us;
                func = rhv.func;
            }

        };
        std::vector<Task> task_queue;
        std::vector<int> task_queue_index;


        /*
         const std::function<bool()>& func
         return true, keep running at given rate
         return false, run once
         */
        void addTask(const std::function<bool()>& func,long  delay_us = 100, int prio = 10){
            task_queue.emplace_back(func,delay_us,prio);
            task_queue_index.resize(task_queue.size());
            std::generate(task_queue_index.begin(), task_queue_index.end(), [n = 0]() mutable { return n++; });
        }
        bool call(){

            common::Time now = common::FromUnixNow();

            bool run = false;

            if (!task_queue.empty()) {
                task_queue_index.resize(task_queue.size());

                std::generate(task_queue_index.begin(), task_queue_index.end(), [n = 0]() mutable { return n++; });

                std::sort(task_queue_index.begin(),task_queue_index.end(),[&](auto& v1, auto& v2){
                    return task_queue[v1].prio < task_queue[v2].prio;
                });


                for(size_t i = 0 ; i < task_queue_index.size(); i++)
                {
                    now = common::FromUnixNow();
                    auto& task_opt=task_queue[task_queue_index[i]];
                    auto diff = now - task_opt.time;
                    long time_diff = common::ToMicroSeconds(diff);

                    if( time_diff >= task_opt.delay_us){
                        run = true;
                        task_opt.valid = task_opt.func();
//                        auto t1 = now;
//                        auto t2 = common::FromUniversal(ToUniversal(task_opt.time) + diff.count() );
                        auto t_next = common::FromUniversal(ToUniversal(task_opt.time) + common::FromMicroseconds(task_opt.delay_us).count() );
//                        std::cout << "check diff1 " <<    (t1 - task_opt.time).count() << "," << (t1-t2).count() << ", " << (t1 - t3).count() << std::endl;
//                        std::cout << "task_opt.delay_us  " << task_opt.delay_us<< std::endl;
//                        std::cout << "diff.count()  " << diff.count()<< std::endl;
//                        std::cout << "common::FromMicroseconds(task_opt.delay_us).count()  " << common::FromMicroseconds(task_opt.delay_us).count()<< std::endl;

                        task_opt.time = t_next  ;
//                        task_opt.time = now;

//                        if(task_opt.valid){task_opt.time = now;}
                    }
                    {
                        for(size_t j = 0 ; j < i; j++)
                        {
                            now = common::FromUnixNow();
                            auto& task_opt_next=task_queue[task_queue_index[j]];
                            if(common::ToMicroSeconds(now - task_opt_next.time) >= task_opt_next.delay_us){
                                run = true;
                                task_opt_next.valid = task_opt_next.func();
                                auto t_next = common::FromUniversal(ToUniversal(task_opt_next.time) + common::FromMicroseconds(task_opt_next.delay_us).count() );

                                task_opt_next.time = t_next ;
//                                if(task_opt_next.valid){ task_opt_next.time = now;}
                            }
                        }
                    }
                }
                if(run){

                    auto it  = std::remove_if(task_queue.begin(),task_queue.end(),[](auto& e){
                        return !e.valid;
                    });

                    task_queue.erase(it, task_queue.end());

                }


            }

            return !task_queue.empty();

        }

    };
}
#endif //COMMON_TASK_H
