//
// Created by waxz on 23-3-4.
//

#include <oneapi/tbb.h>

#include <iostream>
#include <vector>


struct AppBody{

    void operator()(const tbb::blocked_range<size_t>& r)const {

        for(size_t  i = r.begin(); i != r.end(); i++){
            std::cout << "hello app " << i << std::endl;

        }
    }


};

int main(int argc, char** argv){

    using namespace oneapi::tbb;


    parallel_for(0,5, [](const auto& i){

        std::cout << "hello lambda " << i << std::endl;

    });

    AppBody app;
    blocked_range<size_t> r(0,5);
    parallel_for(r , app);




}