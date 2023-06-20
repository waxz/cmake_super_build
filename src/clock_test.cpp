//
// Created by waxz on 6/16/23.
//
#include <iostream>
#include <chrono>
#include <vector>
#include <algorithm>
#include <deque>
#include "common/suspend.h"
#include "common/clock_time.h"

template<typename T>
struct ValueStamped{

    common::Time time = common::FromUnixNow();
    T value ;
};

//https://stackoverflow.com/questions/41958581/difference-between-upper-bound-and-lower-bound-in-stl
/*

value a a a b b b c c c
index 0 1 2 3 4 5 6 7 8
bound       l     u

 Where l represents the lower bound of b, and u represents the upper bound of b.

So if there are range of values that are "equal" with respect to the comparison being used, lower_bound gives you the first of this, upper_bound gives you one-past-the-end of these. This is the normal pattern of STL ranges [first, last).
 */
template<typename T>
struct MyClassLessThan
{
    bool operator() (const ValueStamped<T> & left, const ValueStamped<T> & right)
    {
        std::cout << "check: left: " << left.value;
        return left.time < right.time;
    }
    bool operator() (const ValueStamped<T> & left, const common::Time&  right)
    {
        std::cout << "check: left: " << left.value << ", diff: " << common::ToMicroSeconds(right - left.time)<<"\n";

        return left.time < right;
    }
    bool operator() (const common::Time& left, const ValueStamped<T> & right)
    {
        std::cout << "check: right: " << right.value << ", diff: " << common::ToMicroSeconds(right.time - left)<<"\n";

        return left < right.time;
    }
};

template<typename T>
struct ValueStampedBuffer{
    std::deque<ValueStamped<float>> buffer;
    void add(float value){

        buffer.push_back(ValueStamped<float>{common::FromUnixNow(), value});

        if(buffer.size() > 2000){
            buffer.erase(buffer.begin(),buffer.begin() + 1000);
        }
    }
    bool empty(){
        return buffer.empty();
    }

    size_t size(){
        return buffer.size();
    }
    bool query(const common::Time& time, T & value){

        if(buffer.size() < 3){
            return false;
        }

        if(time == buffer.back().time){

            value = buffer.back().value;
            return true;
        }

        // first v > t

        auto pred = [time](auto&x){return x.time < time;};
        auto mid = std::partition (buffer.begin(),buffer.end(),pred);


//        auto it = std::stable_partition(buffer.begin(),buffer.end(), [time](auto& v) { return v.time > time; });





        if(mid!= buffer.end()){
            auto low_it = std::prev(mid);
            auto up_it = mid;

            auto s1 =  common::ToMicroSeconds(up_it->time - low_it->time);
            auto s2 =  common::ToMicroSeconds(time - low_it->time);

//            std::cout << "interpolate, up: "  << std::distance(buffer.begin(), up_it)  << ", low:  " << std::distance(buffer.begin(), low_it) << std::endl;

//            std::cout << "interpolate, value : " << up_it->value << ", " << low_it->value << std::endl;

//            std::cout << "interpolate: time " << s1*1e-6 << ", " << s2*1e-6<< std::endl;


            value = low_it->value + (up_it->value - low_it->value)*s2/s1;

            return true;
        }else{
            return false;
        }

        return false;
    }
    ValueStamped<float>& back(){
        return buffer.back();
    }

};
bool IsOdd (int i) { return (i%2)==1; }


int main(){
    {
        std::vector<int> foo {1,2,3,4,5,6,7,8,9};
        std::vector<int> odd;

        auto pred = [](auto&x ){return x < 4;};
        auto mid = std::partition (foo.begin(),foo.end(),pred);

        std::cout << "mid: " << mid - foo.begin() << "\n";

        auto it = std::partition_point(foo.begin(),foo.end(),pred);
        odd.assign (foo.begin(),it);

        // print contents of odd:
        std::cout << "odd:";
        for (int& x:odd) std::cout << ' ' << x;
        std::cout << '\n';

        std::cout << "foo:";
        for (int& x:foo) std::cout << ' ' << x;
        std::cout << '\n';
    }
    typedef std::chrono::high_resolution_clock Clock;
    {

        auto t1 = Clock::now();
        common::preciseSleep(0.1);
        auto t2 = Clock::now();
        std::cout << (t2-t1).count() << "  ns \n";
        std::cout << (t2-t1).count()*1e-9 << "  s \n";

    }
    {

        common::Suspend s;
        auto t1 = Clock::now();

        s.sleep(100);
        auto t2 = Clock::now();
        std::cout << (t2-t1).count() << "  ns \n";
        std::cout << (t2-t1).count()*1e-9 << "  s \n";

    }
    {

        common::Suspend s;
        auto t1 = common::FromUnixNow();

        s.sleep(100);
        auto t2 = common::FromUnixNow();
        std::cout << (t2-t1).count() << "  ns \n";
        std::cout << (t2-t1).count()*1e-7 << "  s \n";

        std::cout << common::ToMillSeconds(t2-t1)  << "  ms \n";
        std::cout << common::ToMicroSeconds(t2-t1)  << "  us \n";
        std::cout << common::ToMicroSeconds(t2-t1)*1e-6  << "  s \n";


    }
    {
        std::vector<u_int16_t> arr{1,2,3,4,5};
        u_int32_t v = 3;
        auto it = std::find(arr.begin(), arr.end(),v);
        if(it != arr.end()){
            std::cout << "find " << *it << std::endl;
        }else{
            std::cout << "fail "  << std::endl;

        }

        int arr1[5] = {1,2,3,4,5};
        int arr2[5] = {0};
        std::copy(std::begin(arr1), std::end(arr1),std::begin(arr2));
        for(int i =0 ; i < 5;i++){
            std::cout << "arr2: " << arr2[i]<< "\n";
        }
    }
    {

        ValueStampedBuffer<float> buffer_1;
        ValueStampedBuffer<float> buffer_2;
        using namespace std::chrono_literals;

        common::Time start = common::FromUnixNow();
        for(int i = 0 ; i < 10;i++){
            buffer_1.add(i*0.1);
            std::this_thread::sleep_for(50ms);

            buffer_2.add(i*0.1);
            std::this_thread::sleep_for(50ms);

            if(i   > 6){
                float qv = 0.0;
                std::cout << "check " << i << std::endl;
                bool ok = buffer_2.query(buffer_1.buffer[i].time,qv);


                if(ok){
                    std::cout << "qv: " << qv << "\n";
                }else{
                    std::cout << "fail\n";

                }
            }
        }

        for(int i = 0 ; i < 10;i++){

            std::cout << "t1 , " << i   << ", diff: " << common::ToMicroSeconds(buffer_1.buffer[i].time - start) * 1e-6 << "\n";
            std::cout << "t2 , " << i   << ", diff: " << common::ToMicroSeconds(buffer_2.buffer[i].time - start) * 1e-6 << "\n";

        }



    }
    return 0;
}