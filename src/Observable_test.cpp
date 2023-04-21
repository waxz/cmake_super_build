//
// Created by waxz on 23-4-2.
//

#include <iostream>
#include <observable/observable.hpp>
#include <iostream>
#include <chrono>
#include "common/string_logger.h"


#include "coroutine/coroutine.h"

#include "tinyco/schedule.h"

#include <inttypes.h>
#include <stdint.h>
#include <cstdio>
#include <cstring>
#include <iostream>

// include context header file
#include <libcopp/coroutine/coroutine_context_container.h>
// include task header file
#include <libcotask/task.h>
/*
coroutine use a thread_local static object

 */

#include "stackless_coroutine.hpp"

template <class R> struct value_t {
    R return_value;
    std::promise<R> p;
    int finished_count = 0;
};
void test_stackless(){
    auto f =  stackless_coroutine::make_block(
            [](auto &context, auto &value) { value.return_value = 1; },
            stackless_coroutine::make_while_true([](auto &context, auto &value) {
                ++value.return_value;
                return context.do_return();
            })

    ) ;

    bool destructed = false;
    struct variables {
        int i = 0;
        bool *b = nullptr;
        ~variables() { *b = true; }
        variables(int it, bool *bt) : i{it}, b{bt} {}
    };

    auto block =
            stackless_coroutine::make_block(stackless_coroutine::make_while_true(
                    [](auto &context, variables &v) {
                        return context.do_async_yield(v.i);
                    },
                    [](auto &context, variables &v) { ++v.i; }));

    std::vector<int> v;
    {
        auto gen = stackless_coroutine::make_generator<int, variables>(block, 0,
                                                                       &destructed);
        for (auto i : gen) {
            v.push_back(i);
            if (i == 10)
                break;
        }
    }
    std::vector<int> answer = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10};

}

void routine_func2(int i)
{


}

void thread_func()
{


}

void test_co_1() {


    coroutine::ordinator = coroutine::Ordinator(1025*1024);


    coroutine::Channel<int> channel;

    std::vector<coroutine::routine_t> coroutine_list;

    coroutine::routine_t rt1 = coroutine::create([&channel]() {



        for(int i = 0 ; i < 10; i++){
            MLOGI("hello %d", i);
            std::cout << std::endl;
//            int v = channel.pop();
//            MLOGI("hello channel %d", v);
            coroutine::yield();
        }

    });
    coroutine::routine_t rt2 = coroutine::create([&channel]() {
        MLOGI("hello %d", 1);
        int i = channel.pop();
        MLOGI("hello channel %d", i);
    });

    {
        coroutine::routine_t rt3 = coroutine::create([&]() {
//            int i = channel.pop();
//            MLOGI("hello channel %d", i);

            for(int i = 0 ; i < 10; i++){
//            int v = channel.pop();
//            MLOGI("hello channel %d", v);
                MLOGI("hello %d", i);
                std::cout << std::endl;

                coroutine::resume(rt1);
                coroutine::yield();

            }

        });

        rt2 = rt3;
    }
    channel.push(11);
    channel.push(12);

    std::cout << "00" << std::endl;
    coroutine::resume(rt1);

    std::cout << "01" << std::endl;
    for(int i = 0; i < 10; i++){
        coroutine::resume(rt2);

    }

}

int test_ob_1(){

    auto sub_1 = observable::subject<void(std::string)> { };
    sub_1.subscribe([](auto const & msg) {
        MLOGI("receive msg %s", msg.c_str());
    });
    auto sub = observable::subject<void(std::string)> { };
    sub.subscribe([&](auto const & msg) {
        sub_1.notify(msg);

        MLOGI("receive msg %s", msg.c_str());
    });
    sub.subscribe([](auto const & msg) {
        MLOGI("receive msg %s", msg.c_str());
    });

    // "Hello, world!" will be printed to stdout.
    sub.notify("Hello, world!");


    std::thread t1([&]{

        sub.subscribe([](auto const & msg) {
            MLOGI("receive msg %s", msg.c_str());
        });

    });
    t1.join();


    std::thread t2([&]{

        for(int i = 0 ;i < 10;i++){

            char buffer[50];
            sprintf(buffer,"hello %d", i);
            std::cout << "send " << buffer << "\n";
            sub.notify(buffer);

        }
    });

    t2.join();


    auto a = observable::value<int> { 5 };
    auto b = observable::value<int> { 5 };
    auto avg = observe(
            (a + b) / 2.0f
    );
    auto eq_msg = observe(
            select(a == b, "equal", "not equal")
    );

    avg.subscribe([](auto val) { std::cout << val << std::endl; });
    eq_msg.subscribe([](auto const & msg) { std::cout << msg << std::endl; });

    // "10" and "not equal" will be printed to stdout in an
    // unspecified order.
    b = 15;

    return 0;
}

// define a coroutine runner
int my_runner(void *) {
    copp::coroutine_context *addr = copp::this_coroutine::get_coroutine();

    for(int i = 0 ; i < 100 ;i++){
        std::cout << "cortoutine " << addr << " is running at " << i << ", thread " << std::this_thread::get_id() << std::endl;

        std::cout << "cortoutine " << addr << " is running." << std::endl;

        addr->yield();

        std::cout << "cortoutine " << addr << " is resumed." << std::endl;

    }

    return 1;
}

int test_libcopp()
{

    typedef copp::coroutine_context_default coroutine_type;
    typedef cotask::task<> my_task_t;

    // create a coroutine
    copp::coroutine_context_default::ptr_t co_obj = coroutine_type::create(my_runner);
    std::cout << "cortoutine " << co_obj << " is created." << std::endl;


    // start a coroutine
    co_obj->start();
    std::thread t1([&]{

        for(int i = 0 ; i < 3;i++){
            std::cout << "==== cortoutine resume " << " running at " << i << ", thread " << std::this_thread::get_id() << std::endl;

            co_obj->resume();
            using namespace std::chrono_literals;
            std::this_thread::sleep_for(100ms);
        }
    });
    std::thread t2([&]{
        for(int i = 0 ; i < 3;i++){
            std::cout << "==== cortoutine resume " << " running at " << i << ", thread " << std::this_thread::get_id() << std::endl;
            co_obj->resume();
            using namespace std::chrono_literals;
            std::this_thread::sleep_for(100ms);
        }
    });
    t1.join();
    t2.join();

    // yield from my_runner
    std::cout << "cortoutine " << co_obj << " is yield." << std::endl;
    co_obj->resume();

    // create a task using factory function [with lambda expression]
    my_task_t::ptr_t task = my_task_t::create([]() {
        std::cout << "task " << cotask::this_task::get<my_task_t>()->get_id() << " started" << std::endl;
        cotask::this_task::get_task()->yield();
        std::cout << "task " << cotask::this_task::get<my_task_t>()->get_id() << " resumed" << std::endl;
        return 0;
    });

    my_task_t::ptr_t task2 = my_task_t::create([]() {
        std::cout << "task " << cotask::this_task::get<my_task_t>()->get_id() << " started" << std::endl;
        cotask::this_task::get_task()->yield();
        std::cout << "task " << cotask::this_task::get<my_task_t>()->get_id() << " resumed" << std::endl;
        return 0;
    });


    std::cout << "task " << task->get_id() << " created" << std::endl;
    // start a task
    task->start();

    std::cout << "task " << task->get_id() << " yield" << std::endl;
    task->resume();


    std::cout << "cortoutine " << co_obj << " exit and return " << co_obj->get_ret_code() << "." << std::endl;


    {

        MLOGI("%s", "start await test");


        int test_code = 128;

        MLOGI("%s", "first_task create");

        my_task_t::ptr_t first_task = my_task_t::create([&]() {
            puts("|first task running and will be yield ...");
            cotask::this_task::get_task()->yield();
            for(int i = 0 ; i < 5;i++){
                puts("|loop task running and will be yield ...");

                cotask::this_task::get_task()->yield();
            }

            puts("|first task resumed ...");
            printf("test code already reset => %d\n", ++test_code);
        });
        // add many then task using lambda expression
        MLOGI("%s", "first_task then");

        first_task
                ->then([=]() {
                    puts("|second task running...");
                    printf("test code should be inited 128 => %d\n", test_code);
                })
                ->then([&]() {
                    puts("|haha ... this is the third task.");
                    printf("test code is the same => %d\n", ++test_code);
                    return "return value will be ignored";
                })
                ->then(
                        [&](EXPLICIT_UNUSED_ATTR void *priv_data) {
                            puts("|it's boring");
                            printf("test code is %d\n", ++test_code);
//                            assert(&test_code == priv_data);



                            return 0;
                        },
                        &test_code);
        test_code = 0;
        // start a task
        MLOGI("%s", "first_task start");

        first_task->start();
        MLOGI("%s", "first_task resume");

        first_task->resume();
        MLOGI("%s", "first_task then");

        // these code below will failed.
        first_task->then([]() {
            puts("this will run immediately.");
            return 0;
        });
        MLOGI("%s", "await_task create");

        my_task_t::ptr_t await_task = my_task_t::create([&]() {
            puts("**** await_task for first_task.");
            return 0;
        });




        MLOGI("%s", "await_task await_task");

        await_task->await_task(first_task);
        MLOGI("%s", "await_task start");

        await_task->start();
        for(int i = 0 ;i < 100;i++){
            first_task->resume();
            await_task->resume();
        }

        printf("|task start twice will failed: %d\n", first_task->start());
        printf("|test_code end with %d\n", test_code);

    }
    return 0;
}

void test_tinyco(){
    co_go(
            []
            {
                std::cout << "111" << std::endl;
                co_go(
                        []
                        {
                            std::cout << "222" << std::endl;
                            co_yield();
                            std::cout << "222222" << std::endl;
                        }
                );
                std::cout << "111111" << std::endl;

                for(int i = 0 ; i < 10;i++){
                    std::cout << "run " << i << std::endl;
                    co_yield();
                }

            }
    );

    co_go(
            []
            {
                std::cout << "333" << std::endl;
                co_go(
                        []
                        {
                            std::cout << "444" << std::endl;
                            co_yield();
                            std::cout << "444444" << std::endl;
                        }
                );
                std::cout << "333333" << std::endl;
            }
    );

    tinyco::Schedule* sche = tinyco::Schedule::getSchedule();



    std::cout << "end" << std::endl;
}

int main(){
//    test_co_1();
    test_stackless();
//    test_ob_1();
//    test_tinyco();
}