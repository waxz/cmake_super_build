//
// Created by waxz on 4/29/23.
//

#include <type_traits>
#include <vector>
#include <string>
#include <thread>
#include <iostream>
#include <functional>
#include <cstdint>

// 1. use https://github.com/catchorg/Catch2
/*
 add_executable catch_amalgamated.cpp
 include catch_amalgamated.hpp

 in CLion, you can directly run each test case
 */

// 2. use https://github.com/BlackMATov/invoke.hpp
// for function and member function invoke
// and for sfinae check

// some useful links
//https://www.fluentcpp.com/2018/05/15/make-sfinae-pretty-1-what-value-sfinae-brings-to-code/
//https://stackoverflow.com/questions/257288/templated-check-for-the-existence-of-a-class-member-function


//3. use https://github.com/eranpeer/FakeIt
// Using a pre-packaged single_header: -I"<fakeit_folder>/single_header/gtest"
// or use source file: -I"<fakeit_folder>/include" -I"<fakeit_folder>/config/gtest"
#include "invoke.hpp/invoke.hpp"

#include "catch_amalgamated.hpp"
#include "fakeit.hpp"


// *********************************************************************************

struct Point{
    float x;
    float y;
    float z;
//    Point() = delete;
};

struct Cat{
    void hello(int a){
        std::cout << "cat say main"<< std::endl;
    }
};


// *********************************************************************************

namespace inv = invoke_hpp;

void simple_static_function() {
}

int simple_static_function_r() {
    return 42;
}

int simple_static_function_r_with_arg(int v) {
    return v;
}

const int& simple_static_function_r_with_ref_arg(const int& v) {
    return v;
}
class obj_t {
public:
    int value = 42;
    const int value_c = 42;

    void member() {
    }

    int member_r() {
        return 42;
    }

    int member_r_with_arg(int v) {
        return v;
    }

    const int& member_r_with_ref_arg(const int& v) {
        return v;
    }
};

class obj2_t {
};

// *********************************************************************************

uint32_t factorial( uint32_t number ) {
    return number <= 1 ? number : factorial(number-1) * number;
}

struct SomeInterface {
    virtual ~SomeInterface(void) = default;
    virtual int foo(int) = 0;
    virtual int bar(int, int) = 0;
    virtual int baz(int*, int&) = 0;
};
TEST_CASE("Example test", "[example]" ) {

    using namespace fakeit;
    Mock<SomeInterface> mock;

    When(Method(mock, foo).Using(Eq(0))).Return(42);

    auto const result = mock.get().foo(0);

    REQUIRE(result == 42);
    Verify(Method(mock, foo)).Exactly(1);
}

TEST_CASE("Simple generator use") {
    auto number = GENERATE(2, 4, 8);
    CAPTURE(number);
    REQUIRE(number % 2 == 0);
}

TEST_CASE( "check catch" , "[factorial]" ) {
    std::cout<< "check catch" << std::endl;

    REQUIRE( factorial( 1) == 1 );
    REQUIRE( factorial( 2) == 2 );
    REQUIRE( factorial( 3) == 6 );
    REQUIRE( factorial(10) == 3'628'800 );
}

TEST_CASE( "Factorials are computed", "[factorial]" ) {
    std::cout<< "check catch Factorials" << std::endl;

    REQUIRE( factorial( 1) == 1 );
    REQUIRE( factorial( 2) == 2 );
    REQUIRE( factorial( 3) == 6 );
    REQUIRE( factorial(10) == 3'628'800 );
}

TEST_CASE("Fibonacci") {


    // now let's benchmark:
    BENCHMARK("Fibonacci 5") {
                                  return factorial(5);
                              };

}

TEST_CASE("invoke","[invoke]"){
    inv::invoke(simple_static_function);
    std::cout << inv::invoke(simple_static_function_r) << std::endl;
    std::cout << inv::invoke(simple_static_function_r_with_arg, 42) << std::endl;

    static_assert(
            inv::is_invocable<decltype(simple_static_function)>::value,
            "unit test fail");
    static_assert(
            inv::is_invocable<decltype(simple_static_function_r)>::value,
            "unit test fail");
    static_assert(
            inv::is_invocable<decltype(simple_static_function_r_with_arg), int>::value,
            "unit test fail");

    static_assert(
            !inv::is_invocable<decltype(simple_static_function), Point>::value,
            "unit test fail");
    static_assert(
            !inv::is_invocable<decltype(simple_static_function_r), obj_t>::value,
            "unit test fail");
    static_assert(
            !inv::is_invocable<decltype(simple_static_function_r_with_arg)>::value,
            "unit test fail");


    static_assert(
            inv::is_invocable<decltype(&obj_t::member), obj_t>::value,
            "unit test fail");
    static_assert(
            inv::is_invocable<decltype(&obj_t::member), obj_t*>::value,
            "unit test fail");
    static_assert(
            inv::is_invocable<decltype(&obj_t::member), std::reference_wrapper<obj_t>>::value,
            "unit test fail");

    static_assert(
            inv::is_invocable<decltype(&obj_t::member_r), obj_t>::value,
            "unit test fail");
    static_assert(
            inv::is_invocable<decltype(&obj_t::member_r), obj_t*>::value,
            "unit test fail");
    static_assert(
            inv::is_invocable<decltype(&obj_t::member_r), std::reference_wrapper<obj_t>>::value,
            "unit test fail");

    static_assert(
            inv::is_invocable<decltype(&obj_t::member_r_with_arg), obj_t, int>::value,
            "unit test fail");
    static_assert(
            inv::is_invocable<decltype(&obj_t::member_r_with_arg), obj_t*, int>::value,
            "unit test fail");
    static_assert(
            inv::is_invocable<decltype(&obj_t::member_r_with_arg), std::reference_wrapper<obj_t>, int>::value,
            "unit test fail");
}