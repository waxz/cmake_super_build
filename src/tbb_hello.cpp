//
// Created by waxz on 8/28/22.
//
#include <iostream>

#include "oneapi/tbb/parallel_for.h"
#include "oneapi/tbb/blocked_range.h"



int main() {
    std::cout << "Hello, World tbb!" << std::endl;


    tbb::parallel_for(1,10,[](auto & i){

        std::cout << "Hello, World from tbb : " << i << std::endl;

    });
    return 0;
}