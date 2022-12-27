
#include <taskflow/taskflow.hpp>  // the only include you need

#include <thread>

#include "common/clock_time.h"
#include "common/suspend.h"
int test_1(){

    tf::Executor executor;
    tf::Taskflow taskflow("simple");

    auto [A, B, C, D] = taskflow.emplace(
            []() { std::cout << "TaskA\n" << std::this_thread::get_id() <<"\n"; },
            []() { std::cout << "TaskB\n" << std::this_thread::get_id() <<"\n"; },
            []() { std::cout << "TaskC\n" << std::this_thread::get_id() <<"\n"; },
            []() { std::cout << "TaskD\n" << std::this_thread::get_id() <<"\n"; }
    );

    A.precede(B, C);  // A runs before B and C
    D.succeed(B, C);  // D runs after  B and C

    executor.run(taskflow).wait();

    return 0;
}



int test_2(){

    tf::Executor executor(1);
    tf::Taskflow taskflow("demo");
    auto A = taskflow.emplace([]{
        std::cout << "TaskA\n" << std::this_thread::get_id() <<"\n";

    }).name("A");
    auto E = taskflow.emplace([]{
        std::cout << "TaskE\n" << std::this_thread::get_id() <<"\n";

    }).name("A");

    auto B = taskflow.emplace([](tf::Subflow& sf){
        sf.emplace([]{
//            std::cout << "TaskB1\n" << std::this_thread::get_id() <<"\n";

        }).name("B1");

    }).name("B");


    int i = 0;
    common::Suspend s1;
    auto cond = taskflow.emplace([&]{
        i++;
        std::cout << "Taskcond \n" << std::this_thread::get_id() <<"\n";

        s1.sleep(1000);
        return (i < 10) ? 0 : 1;
    });

    common::Time t1, t2;
    A.precede(cond);

    cond.precede(cond,E);

    t1 = common::FromUnixNow();
    executor.run(taskflow).wait();

    t2 = common::FromUnixNow();

    std::cout<< "use time " << common::ToMicroSeconds(t2-t1) << "micro seconds\n";
    return 0;
}

int main(){

    test_2();

}