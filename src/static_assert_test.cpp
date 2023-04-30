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
#include <utility>


#include "common/smart_pointer.h"

//-DDEFINE_ASSERT_MODE=AssertMode::THROW_ -DDEFINE_ASSERT_LEVEL=1
#define DEFINE_ASSERT_MODE AssertMode::THROW_
#define DEFINE_ASSERT_LEVEL 1
#include "dynamic_assert/assertion.h"

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

#include "json.hpp"


// *********************************************************************************

struct Point{
    float x;
    float y;
    float z;
    Point(){
        std::cout << "create a Point "  << std::endl;
    }
    Point(const Point& rhv){
        std::cout << "copy a Point "  << std::endl;
    }
    ~Point(){
        std::cout << "delete a Point "  << std::endl;
    }
    void hello()const{
        printf("position: [%.3f, %.3f, %.3f]",x,y,z);
    }
//    Point() = delete;
};

struct Cat: public std::enable_shared_from_this<Cat>{
    std::string barkType;
    std::string color;
    int weight = 0;
    std::string pawType;
    Cat(){
        std::cout << "create a cat "  << std::endl;
    }
    Cat(const Cat& rhv){
        std::cout << "copy a cat "  << std::endl;
    }
    Cat( Cat&& rhv){
        std::cout << "move a cat "  << std::endl;
    }

    Cat& operator=(Cat&& rhv){
        std::cout << "move assign a cat "  << std::endl;

        return *this;
    }
    ~Cat(){
        std::cout << "delete a cat "  << std::endl;
    }
    std::shared_ptr<Cat> Get() {return shared_from_this();}
    void hello(int a){
        std::cout << "cat say " << a << std::endl;
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


// std::declval
// use decltype(myfunc(std::declval<float>()))

Cat to_common(const Point&p){

    return Cat{};
}

void to_common(const Point&p, Cat& cat){


    p.hello();
    cat.hello(43);
}

template<typename T>
void create_channel(const T& src){
//    inv::invoke_result_t<decltype(to_common), const T&> target;
    decltype(to_common(std::declval<const Point&>())) target;

    target.hello(42);
    to_common(src,target);
}

int myfunc(int a)
{
    return a;
}
float myfunc(float a)
{
    return a;
}

template<typename Class, typename T>
struct PropertyImpl {
    constexpr PropertyImpl(T Class::*aMember, const char* aName) : member{aMember}, name{aName} {}

    using Type = T;

    T Class::*member;
    const char* name;
};

template<typename Class, typename T>
constexpr auto property(T Class::*member, const char* name) {
    return PropertyImpl<Class, T>{member, name};
}

template <typename T, T... S, typename F>
constexpr void for_sequence(std::integer_sequence<T, S...>, F&& f) {
    using unpack_t = int[];
    (void)unpack_t{(static_cast<void>(f(std::integral_constant<T, S>{})), 0)..., 0};
}


TEST_CASE("invoke parameter","[invoke parameter]"){

    Point p;
    p.x = 100.0;
    p.y = 10.0;
    p.z = 0.1;

    create_channel(p);

    decltype(myfunc(std::declval<float>())) a;  // return type
    decltype(myfunc(std::declval<int>())) b;  // return type
    std::decay<Cat> c;
    std::function<decltype(a)(float)> fun;      // function type


}

void to_json(nlohmann::json& j, const Point& p)
{
    j = {{"x", p.x}, {"y", p.y}, {"z", p.z}};
}

void from_json(const nlohmann::json& j, Point& p) {
    j.at("x").get_to(p.x);
    j.at("y").get_to(p.y);
    j.at("z").get_to(p.z);
}

constexpr auto cat_properties = std::make_tuple(
        property(&Cat::barkType, "barkType"),
        property(&Cat::color, "color"),
        property(&Cat::weight, "weight"),
        property(&Cat::pawType, "pawType")
);
void to_json(nlohmann::json& j, const Cat& object)
{

    constexpr auto nbProperties = std::tuple_size<decltype(cat_properties)>::value;
    for_sequence(std::make_index_sequence<nbProperties>{}, [&](auto i) {
        // get the property
        auto& property = std::get<i>(cat_properties);
        // set the value to the member
        j[property.name] = object.*(property.member);
    });
}

void from_json(const nlohmann::json& j, Cat& object) {

    constexpr auto nbProperties = std::tuple_size<decltype(cat_properties)>::value;
    for_sequence(std::make_index_sequence<nbProperties>{}, [&](auto i) {
        // get the property
        auto& property = std::get<i>(cat_properties);
        // set the value to the member
        object.*(property.member) = j[property.name];
    });
}
TEST_CASE("json test", "[json convert]"){

    constexpr auto type_dict = std::make_tuple(
            property(int, "barkType"),
            property(&Cat::color, "color"),
            property(&Cat::weight, "weight"),
            property(&Cat::pawType, "pawType")
    );

    constexpr auto properties = std::make_tuple(
            property(&Cat::barkType, "barkType"),
            property(&Cat::color, "color"),
            property(&Cat::weight, "weight"),
            property(&Cat::pawType, "pawType")
    );

    constexpr auto nbProperties = std::tuple_size<decltype(properties)>::value;

    Cat object;
    object.barkType.assign("pspsps");
    object.color.assign("black");
    object.weight = 120.0;
    object.pawType.assign("ww");
    for_sequence(std::make_index_sequence<nbProperties>{}, [&](auto i) {
        // get the property
        auto& property = std::get<i>(properties);
        // set the value to the member
        std::cout << "name: " << property.name << ", value: " << object.*(property.member) << std::endl;
    });

    nlohmann::json cat_json = object;
    std::cout << "cat_json:\n" << cat_json.dump(4)<< std::endl;
    cat_json["weight"] = 223;

    Cat cat2 = cat_json;
    for_sequence(std::make_index_sequence<nbProperties>{}, [&](auto i) {
        // get the property
        auto& property = std::get<i>(properties);
        // set the value to the member
        std::cout << "cat2 name: " << property.name << ", value: " << cat2.*(property.member) << std::endl;
    });

}




TEST_CASE("typeid","[typde id]"){
    //https://stackoverflow.com/questions/4484982/how-to-convert-typename-t-to-string-in-c

    {
        const char* name = common::TypeName<int>::Get();
        std::cout << "int : " << name << std::endl;
    }
    {
        const char* name = common::TypeName<float>::Get();
        std::cout << "float : " << name << std::endl;
    }
    {
        const char* name = common::TypeName<Point>::Get();
        std::cout << "Point : " << name << std::endl;
    }

}






// template <typename T, typename std::enable_if<std::is_lvalue_reference<T>{} , bool>::type = true>
// template <typename T, typename std::enable_if<!std::is_reference<T>::value, std::nullptr_t>::type = nullptr>
// works: template <typename T, typename std::enable_if<!std::is_lvalue_reference<T>{} , bool>::type = true>
// works: template <typename T, typename std::enable_if<!std::is_reference<T>{} , bool>::type = true>




template <typename T>
void delete_deleter( void * p ) {
    delete static_cast<T*>(p);
}

template <typename T>
struct my_unique_ptr {
    std::function< void (void*) > deleter;
    T * p;
    template <typename U>
    my_unique_ptr( U * p, std::function< void(void*) > deleter = &delete_deleter<U> )
            : p(p), deleter(deleter)
    {}
    ~my_unique_ptr() {
        deleter( p );
    }
};

template <typename T>
struct my_shared_ptr {
    std::function< void (void*) > deleter;
    T * p;
    template <typename U>
    my_shared_ptr( U * p, std::function< void(void*) > deleter = &delete_deleter<U> )
            : p(p), deleter(deleter)
    {}
    ~my_shared_ptr() {
        deleter( p );
    }
};


TEST_CASE("memory pointer","[wild pointer]"){

    std::cout << "********\ncheck wild_ptr" << std::endl;

    {

        std::cout << "********\ncheck wild_ptr cat move" << std::endl;

        common::wild_ptr b;
        Cat cat;
        cat.pawType = "6";
        b.set(std::move(cat));
        b.ref<Cat>();
        std::cout << "get pointer: " << & b.ref<Cat>().weight << std::endl;

    }
    {

        std::cout << "********\ncheck wild_ptr cat copy" << std::endl;

        common::wild_ptr b;
        Cat cat;
        cat.pawType = "6";
        b.set(cat);
        b.ref<Cat>();
        std::cout << "get pointer: " << & b.ref<Cat>().weight << std::endl;

    }
    {

        // raw pointer in stack cannot be manged by shared_ptr
        // shared_ptr call free(pointer_on_stack) will fail
        std::cout << "********\ncheck wild_ptr cat pointer" << std::endl;

        common::wild_ptr b;
        Cat* pcat = new Cat();
        pcat->pawType = "6";
        b.set<Cat>(pcat);
        b.ref<Cat>();
        std::cout << "get pointer: " << & b.ref<Cat>().weight << std::endl;

    }
    {

        // raw pointer in stack cannot be manged by shared_ptr
        // shared_ptr call free(pointer_on_stack) will fail
        std::cout << "********\ncheck wild_ptr cat pointer with enable_shared_from_this" << std::endl;

        common::wild_ptr b;

        std::shared_ptr<Cat> sptr_1 =std::make_shared<Cat>();
        std::shared_ptr<Cat> sptr_2 = sptr_1->Get();
//        Cat cat;
//        std::shared_ptr<Cat> sp = cat.Get();
//        cat.pawType = "6";
        b.set<Cat>(sptr_2);
        b.ref<Cat>();
        std::cout << "get pointer: " << & b.ref<Cat>().weight << std::endl;

    }
    {
        std::cout << "********\ncheck wild_ptr Point pointer " << std::endl;
        common::wild_ptr b;
        std::shared_ptr<Point> sptr_1 =std::make_shared<Point>();
        b.set<Point>(sptr_1);
        b.ref<Point>();
    }
    {
        std::cout << "********\ncheck wild_ptr Point args constructor " << std::endl;
        common::wild_ptr b;
        b.set<Point>();
        b.ref<Point>();

    }
    {
        std::cout << "********\ncheck wild_ptr Point args constructor " << std::endl;
        common::wild_ptr b;
        b.set<Point>();
        b.ref<Point>();
        b.set<Cat>();
        b.ref<Cat>();

    }
    {
        std::cout << "********\ncheck wild_ptr in container " << std::endl;
        std::vector<common::wild_ptr> array;
        array.emplace_back();
        common::wild_ptr& b = array[0];

        array[0].set<Point>();
        array[0].ref<Point>();
        b.ref<Point>();

    }

#if 0


    {
        std::cout << "********\ncheck wild_ptr cat" << std::endl;

        wild_ptr b;
        Cat cat;
        cat.pawType = "6";
        b.set(cat);
    }

    {
        std::cout << "********\ncheck wild_ptr point" << std::endl;

        wild_ptr b;
        Point p;
        p.x = 66;
        b.set(p);
    }
    {
        std::cout << "********\ncheck wild_ptr cat to point" << std::endl;

        wild_ptr b;
        Cat cat;
        b.set(cat);
        Point p;
        b.set(p);

    }
    std::cout << "********\ncheck my_shared_ptr" << std::endl;
    {
        my_shared_ptr<void> ptr(new Cat());
    }
    {
        my_shared_ptr<void> ptr(new Point());

    }
#endif


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