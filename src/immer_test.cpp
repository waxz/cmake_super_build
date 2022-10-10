//
// Created by waxz on 22-10-10.
//
#include <immer/vector.hpp>
#include <immer/vector_transient.hpp>
#include <immer/map.hpp>
#include "common/clock_time.h"

#include <iostream>
immer::vector<int> myiota(immer::vector<int> v, int first, int last)
{
    auto t = v.transient();
    for (auto i = first; i < last; ++i)
        t.push_back(i);
    return t.persistent();
}

struct ProfileTimer{

    common::Time start;
    common::Time end;
    char* msg;
    ProfileTimer(char * msg_):start(common::FromUnixNow()),msg(msg_){
    }
    ~ProfileTimer(){
        std::cout << msg << " use time " << common::ToMillSeconds(common::FromUnixNow() - start) << " ms" << std::endl;
    }

};
int main()
{
    const auto v0 = immer::vector<int>{1,2,3};


    {
        auto t = v0.transient();
        ProfileTimer timer("myiota transient");

        for(int i=0; i< 100000;i++){
            t.push_back(i);

        }

    }
    {
        auto t = std::vector<int>{};

        ProfileTimer timer("std vector push_back");

        for(int i=0; i< 100000;i++){
            t.push_back(i);

        }

    }


    const auto v1 = v0.push_back(13);

    const auto v3 = v0;


    std::cout << "pointer compare : p1 : " << &(v0[0]) << ", p2 : " << &(v1[0])  << ", p3 : " << &(v3[0])<< std::endl;

    const auto v2 = v1.set(0, 42);
    std::cout << "compare v[0] " <<  v1[0]  << ", " <<  v2[0]  << std::endl;

    const auto m0 = immer::map<std::string, std::string>{};

}
