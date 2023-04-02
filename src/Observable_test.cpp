//
// Created by waxz on 23-4-2.
//

#include <iostream>
#include <observable/observable.hpp>
#include "coroutine.h"
#include <iostream>
#include <chrono>
#include "common/string_logger.h"





/*
coroutine use a thread_local static object

 */

coroutine::Channel<int> channel;

string async_func()
{
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    return "22";
}

void routine_func1()
{
    MLOGI("%s","wait ");

    int i = channel.pop();
    MLOGI("%s %d","get ", i);

    std::cout << i << std::endl;

    i = channel.pop();
    MLOGI("%s %d","get ", i);

    std::cout << i << std::endl;
}

void routine_func2(int i)
{
    std::cout << "20" << std::endl;
    coroutine::yield();

    std::cout << "21" << std::endl;

    //run function async
    //yield current routine if result not returned
    string str = coroutine::await(async_func);
    std::cout << str << std::endl;
}

void thread_func()
{
    //create routine with callback like std::function<void()>
    coroutine::routine_t rt1 = coroutine::create(routine_func1);
    coroutine::routine_t rt2 = coroutine::create(std::bind(routine_func2, 2));

    std::cout << "00" << std::endl;
    coroutine::resume(rt1);

    std::cout << "01" << std::endl;
    coroutine::resume(rt2);

    std::cout << "02" << std::endl;

    channel.push(66);
    MLOGI("%s","channel.push(66)");
    std::cout << std::endl;



    std::cout << "03" << std::endl;
    coroutine::resume(rt2);

    std::cout << "04" << std::endl;
    channel.push(11);

    std::cout << "05" << std::endl;

    std::this_thread::sleep_for(std::chrono::milliseconds(6000));
    coroutine::resume(rt2);

    //destroy routine, free resouce allocated
    //Warning: don't destroy routine by itself
    coroutine::destroy(rt1);
    coroutine::destroy(rt2);
}

void test_co_1(){
    std::thread t1(thread_func);
    std::thread t2([]{
        channel.push(77);
        channel.push(88);


    });
    t1.join();
    t2.join();

}


int main()
{


    coroutine::Channel<std::string> channel_msg;
    coroutine::Channel<const char* > channel_msg_ptr;


    coroutine::routine_t rt1 = coroutine::create([&channel_msg,&channel_msg_ptr](){
        int i = 0;
        while (true){
            auto msg = channel_msg.pop();
            auto msg_ptr = channel_msg_ptr.pop();
            MLOGI("%s %d, msg = %s, msg_ptr = %s", "hello", i,msg.c_str(), msg_ptr);
            i++;
            coroutine::yield();
        }
    });

    coroutine::resume(rt1);
    coroutine::resume(rt1);



    auto sub = observable::subject<void(std::string)> { };
    sub.subscribe([&channel_msg,&channel_msg_ptr](auto const & msg) {
        channel_msg.push(msg);
        channel_msg_ptr.push(msg.c_str());

        std::cout << msg << std::endl;
    });

    // "Hello, world!" will be printed to stdout.
    sub.notify("Hello, world!");

    for(int i = 0 ;i < 10;i++){

        char buffer[50];
        sprintf(buffer,"hello %d", i);
        std::cout << "send " << buffer << "\n";
        sub.notify(buffer);

    }


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