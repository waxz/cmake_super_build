//
// Created by waxz on 4/22/23.
//

#include "common/functions.h"

#include<type_traits>
#include<utility>
#include <iostream>


void foo(void (*fn)())
{
    fn();
}


int main(int argc, char** argv){

    int i = 42;
    auto fn = common::fnptr<void()>([&i]{ i++, std::cout << "i = " << i << "\n";});
    foo(fn);  // compiles!
    foo(fn);  // compiles!
    foo(fn);  // compiles!

}