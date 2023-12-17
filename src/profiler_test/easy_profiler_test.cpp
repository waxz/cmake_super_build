
#include <chrono>
#include <thread>
#include <vector>
#include <iostream>
#include <condition_variable>
#include <cstdlib>
#include <math.h>

#include <easy/profiler.h>
#include <easy/arbitrary_value.h>
#include <easy/reader.h>

void localSleep(uint64_t magic = 200000) {
    //PROFILER_BEGIN_FUNCTION_BLOCK_GROUPED(profiler::colors::Blue);
    volatile int i = 0;
    for (; i < magic; ++i);
}

//////////////////////////////////////////////////////////////////////////

void foo() {
    EASY_FUNCTION(profiler::colors::Magenta); // Magenta block with name "foo"

    EASY_BLOCK("Calculating sum"); // Begin block with default color == Amber100
    int sum = 0;
    for (int i = 0; i < 10; ++i) {
//        EASY_BLOCK("Addition", profiler::colors::Red); // Scoped red block (no EASY_END_BLOCK needed)
        sum += i;
    }
    EASY_END_BLOCK; // End of "Calculating sum" block

    EASY_BLOCK("Calculating multiplication", profiler::colors::Blue500); // Blue block
    int mul = 1;
    for (int i = 1; i < 11; ++i)
        mul *= i;
    //EASY_END_BLOCK; // This is not needed because all blocks are ended on destructor when closing braces met
}

void bar() {
    EASY_FUNCTION(0xfff080aa); // Function block with custom ARGB color
}

void baz() {
    EASY_FUNCTION(); // Function block with default color == Amber100
}

struct Point{

    void print(){
        EASY_FUNCTION(); // Function block with default color == Amber100

    }
};

int main() {
    EASY_PROFILER_ENABLE;
    EASY_MAIN_THREAD;
    profiler::startListen();
//    EASY_NONSCOPED_BLOCK("Frame", true, 5., profiler::ON, -15.f, profiler::colors::White);
    EASY_NONSCOPED_BLOCK("loop")
    localSleep();


    for (int i = 0; i < 100; i++) {
        int v[] = {1 * i, 2 * i, 3 * i};
        EASY_ARRAY("int[3]", v, 3, profiler::colors::Red);
        EASY_VALUE("i", i);
        EASY_VALUE("i", i);
        EASY_VALUE("i", i, EASY_VIN("a"));

        foo();
        bar();
        baz();
    }


    auto func_1 = []{
        EASY_BLOCK("func_1"); // Begin block with default color == Amber100

    };

    std::thread t1 = std::thread([&func_1]{

        Point p;
        for (int i = 0; i < 100; i++) {
            int v[] = {1 * i, 2 * i, 3 * i};
            EASY_VALUE("thread update i", i);
            foo();
            func_1();

            p.print();
        }

    });
    std::thread t2 = std::thread([&func_1]{
        Point p;

        for (int i = 0; i < 100; i++) {
            int v[] = {1 * i, 2 * i, 3 * i};
            EASY_VALUE("thread update i", i);
            foo();
            func_1();
            p.print();

        }

    });
    if(t1.joinable()){
        t1.join();
    }
    if(t2.joinable()){
        t2.join();
    }
    EASY_END_BLOCK;
    profiler::dumpBlocksToFile("easy_test.prof");
    return 0;
}
