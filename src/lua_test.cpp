//
// Created by waxz on 5/13/23.
//


#include <type_traits>
#include <vector>
#include <string>
#include <thread>
#include <iostream>
#include <functional>
#include <cstdint>
#include <utility>

#if defined(__cplusplus) && __cplusplus >= 201703L && defined(__has_include)
#if __has_include(<filesystem>)
#define GHC_USE_STD_FS
#include <filesystem>
namespace fs = std::filesystem;
#endif
#endif
#ifndef GHC_USE_STD_FS
#include "ghc/filesystem.hpp"
//#include "ghc/filesystem.hpp"

namespace fs = ghc::filesystem;
#endif


#include "common/smart_pointer.h"

//-DDEFINE_ASSERT_MODE=AssertMode::THROW_ -DDEFINE_ASSERT_LEVEL=1
#define DEFINE_ASSERT_MODE AssertMode::THROW_
#define DEFINE_ASSERT_LEVEL 1
#include "dynamic_assert/assertion.h"


#include "invoke.hpp/invoke.hpp"

#include "catch_amalgamated.hpp"
#include "fakeit.hpp"

#define SOL_CHECK_ARGUMENTS 1
#include <sol.hpp>

#include <iostream>

#include <thread>
#include "common/string_logger.h"


TEST_CASE("Simple generator use","[run]") {
    std::cout << "run simple test" << std::endl;

auto number = GENERATE(2, 4, 8);
CAPTURE(number);
REQUIRE(number % 2 == 0);
}
uint32_t factorial( uint32_t number ) {
    return number <= 1 ? number : factorial(number-1) * number;
}
TEST_CASE( "check catch" , "[factorial]" ) {
    std::cout<< "check catch" << std::endl;

    REQUIRE( factorial( 1) == 1 );
    REQUIRE( factorial( 2) == 2 );
    REQUIRE( factorial( 3) == 6 );
    REQUIRE( factorial(10) == 3'628'800 );
}

sol::protected_function_result simple_handler (lua_State*, sol::protected_function_result result) {
    // You can just pass it through to let the
    // call-site handle it
    // Call failed
    sol::error err = result;
    std::string what = err.what();
    std::cout << "call failed, sol::error::what() is " << what << std::endl;
    MLOGW("%s", what.c_str());
    return result;
}

//https://sol2.readthedocs.io/en/v2.20.6/exceptions.html#lua-handlers

int my_exception_handler(lua_State* L, sol::optional<const std::exception&> maybe_exception, sol::string_view description) {
    // L is the lua state, which you can wrap in a state_view if necessary
    // maybe_exception will contain exception, if it exists
    // description will either be the what() of the exception or a description saying that we hit the general-case catch(...)
    std::cout << "An exception occurred in a function, here's what it says ";
    if (maybe_exception) {
        std::cout << "(straight from the exception): ";
        const std::exception& ex = *maybe_exception;
        std::cout << ex.what() << std::endl;
    }
    else {
        std::cout << "(from the description parameter): ";
        std::cout.write(description.data(), description.size());
        std::cout << std::endl;
    }

    // you must push 1 element onto the stack to be
    // transported through as the error object in Lua
    // note that Lua -- and 99.5% of all Lua users and libraries -- expects a string
    // so we push a single string (in our case, the description of the error)
    return sol::stack::push(L, description);
}

TEST_CASE( "lua simple script"){

    std::cout << "run lua" << std::endl;
    lua_State* L = luaL_newstate();
    sol::state_view lua(L);
    lua.open_libraries(sol::lib::base, sol::lib::package,sol::lib::os, sol::lib::table, sol::lib::jit,sol::lib::coroutine);
    lua.set_exception_handler(&my_exception_handler);


    lua.script("print('hello lua!')", simple_handler);

    // call lua code, and check to make sure it has loaded and
    // run properly:
    auto handler = &sol::script_default_on_error;

    lua.script("print('hello again, world')", handler);

}

TEST_CASE( "lua simple script handler"){

    std::cout << "run lua" << std::endl;
    lua_State* L = luaL_newstate();
    sol::state_view lua(L);
    lua.open_libraries(sol::lib::base, sol::lib::package,sol::lib::os, sol::lib::table, sol::lib::jit,sol::lib::coroutine);

    auto simple_handler_lambda =
            [](lua_State*, const sol::protected_function_result& result) {
                // You can just pass it through to let the
                // call-site handle it
                std::cout << "\n******LUA INFO:\nAn exception occurred in a function, here's what it says ";
                sol::error err = result;
                std::cout << "call failed, sol::error::what() is " <<  err.what() << std::endl;
//                MLOGW("%s", what.c_str());
                return result;
            };


    {
        auto result = lua.script(
                "printf('hello hello again, world') \n return 24",
                simple_handler_lambda);
        if (result.valid()) {
            std::cout << "the third script worked, and a "
                         "double-hello statement should "
                         "appear above this one!"
                      << std::endl;
            int value = result;
        }
        else {
            std::cout << "the third script failed, check the "
                         "result type for more information!"
                      << std::endl;
        }
    }
}

TEST_CASE("lua coroutine"){
    std::cout << "run lua" << std::endl;
    lua_State* L = luaL_newstate();
    sol::state_view lua(L);
    lua.open_libraries(sol::lib::base, sol::lib::package,sol::lib::os, sol::lib::table, sol::lib::jit,sol::lib::coroutine);



    lua.script(R"(

print("luajit coroutine")
co = coroutine.create(
    function(i)
        print(i);
    end
)

coroutine.resume(co, 1)   -- 1
print(coroutine.status(co))  -- dead

print("----------")

co = coroutine.wrap(
    function(i)
        print(i);
    end
)

co(1)
print(co)  -- dead

print("----------")

co2 = coroutine.create(
    function()
        for i=1,10 do
            print(i)
            if i == 3 then
                print(coroutine.status(co2))  --running
                print(coroutine.running()) --thread:XXXXXX
            end

            coroutine.yield()
        end
    end
)

coroutine.resume(co2) --1
coroutine.resume(co2) --2
coroutine.resume(co2) --3

print(coroutine.status(co2))   -- suspended
print(coroutine.running())

print("----------")

)");


}

TEST_CASE("lua run file"){
    std::cout << "run lua" << std::endl;
    lua_State* L = luaL_newstate();
    sol::state_view lua(L);
    lua.open_libraries(sol::lib::base, sol::lib::package,sol::lib::os, sol::lib::table, sol::lib::jit,sol::lib::coroutine);

    std::cout << "fs::current_path(): " << fs::current_path() << std::endl;

    fs::path scripts_dir = fs::current_path()/"../../src/lua";
    std::cout << "scripts_dir: " << scripts_dir << std::endl;

    fs::path scripts_path = scripts_dir/"test.lua";

    lua.script_file(scripts_path.c_str());
}


struct Doge {
    int tailwag = 50;

    Doge() {
    }

    Doge(int wags)
            : tailwag(wags) {
    }

    ~Doge() {
        std::cout << "Dog at " << this << " is being destroyed..." << std::endl;
    }
};


struct my_type {
    int value = 10;

    my_type() {
        std::cout << "my_type at " << static_cast<void*>(this) << " being default constructed!" << std::endl;
    }

    my_type(const my_type& other) : value(other.value) {
        std::cout << "my_type at " << static_cast<void*>(this) << " being copy constructed!" << std::endl;
    }

    my_type(my_type&& other) : value(other.value) {
        std::cout << "my_type at " << static_cast<void*>(this) << " being move-constructed!" << std::endl;
    }

    my_type& operator=(const my_type& other) {
        value = other.value;
        std::cout << "my_type at " << static_cast<void*>(this) << " being copy-assigned to!" << std::endl;
        return *this;
    }

    my_type& operator=(my_type&& other) {
        value = other.value;
        std::cout << "my_type at " << static_cast<void*>(this) << " being move-assigned to!" << std::endl;
        return *this;
    }

    ~my_type() {
        std::cout << "my_type at " << static_cast<void*>(this) << " being destructed!" << std::endl;
    }
};

TEST_CASE("memory"){
    std::cout << "run lua" << std::endl;
    lua_State* L = luaL_newstate();
    sol::state_view lua(L);
    lua.open_libraries(sol::lib::base, sol::lib::package,sol::lib::os, sol::lib::table, sol::lib::jit,sol::lib::coroutine);


    std::cout << "=== shared_ptr support ===" << std::endl;

    lua.new_usertype<my_type>("my_type",
                              "value", &my_type::value
    );

    using vector_float = std::vector<float>;

    {

        lua.new_usertype<vector_float>("vector_float",
                                              sol::constructors<vector_float(), vector_float(const size_t&)>(),
                                              "size", &vector_float::size,
                                              "push_back", sol::overload([](vector_float & self, const float& v){ self.push_back(v);},  [](vector_float & self, const double& v){self.push_back(v);} ),
                                             sol::meta_function::length, [](vector_float &self){return  self.size();},
                                              sol::meta_function::index, [](vector_float &self, int i) -> float & { return self[i]; },
                                              sol::meta_function::new_index, [](vector_float &self, int i, float v)   { self[i] = v; },
                                              "iterable", [](vector_float &self) { return sol::as_container(self); }
        );


        vector_float arr{0.1f,0.2f,0.3f,0.4f};

        arr.reserve(10);

        lua.set_function("arr_push_back",[&arr](float v){
            arr.push_back(v);
        });


        lua["arr"] = arr;
        lua.script(R"(

print("print all element in arr")
print("arr len " .. #arr)
print("arr size " .. arr:size() )

for i = 0 , arr:size() -1,1
do
print(i .. " = " .. arr[i])
arr[i] = 10 + i
end

arr:push_back(100.0)
arr_push_back(50)

)" );
        std::cout << "=== check in cpp: arr ===" << std::endl;

        for( int i = 0 ; i < arr.size();i++){
            std::cout << i  << "  = " << arr[i]  <<"\n";
        }
        vector_float arr2 = lua["arr"];
        std::cout << "=== check in cpp: arr2 ===" << std::endl;

        for( int i = 0 ; i < arr2.size();i++){
            std::cout << i  << "  = " << arr2[i]  <<"\n";
        }



    }
    {
        auto simple_handler_lambda =
                [](lua_State*, const sol::protected_function_result& result) {
                    // You can just pass it through to let the
                    // call-site handle it
                    std::cout << "\n******LUA INFO:\nAn exception occurred in a function, here's what it says ";
                    sol::error err = result;
                    std::cout << "call failed, sol::error::what() is " <<  err.what() << std::endl;
//                MLOGW("%s", what.c_str());
                    return result;
                };


        lua.set_function("hello",[&](int v){

            std::cout << " hello " << v << " from lua" << std::endl;

        });
        lua.script("hello(20);");

        std::thread t1([&]{
            lua.script("hello(333); ");


            auto result = lua.script(
                    "hello1(333);",
                    simple_handler_lambda);
            if (result.valid()) {
                std::cout << "run lua script ok"
                          << std::endl;

            }
            else {
                std::cout << "run lua script fail"
                          << std::endl;
            }



        });

        t1.join();

    }
    {
        std::shared_ptr<my_type> shared = std::make_shared<my_type>();
        lua["shared"] = std::move(shared);

    }
    {
        std::cout << "getting reference to shared_ptr..." << std::endl;
        std::shared_ptr<my_type>& ref_to_shared_ptr = lua["shared"];
        std::cout << "\tshared.use_count(): " << ref_to_shared_ptr.use_count() << std::endl;
        my_type& ref_to_my_type = lua["shared"];
        std::cout << "\tafter getting reference to my_type: " << ref_to_shared_ptr.use_count() << std::endl;
        my_type* ptr_to_my_type = lua["shared"];
        std::cout << "\tafter getting pointer to my_type: " << ref_to_shared_ptr.use_count() << std::endl;

        REQUIRE(ptr_to_my_type == ref_to_shared_ptr.get());
        REQUIRE(&ref_to_my_type == ref_to_shared_ptr.get());
        REQUIRE(ref_to_shared_ptr->value == 10);

        // script affects all of them equally
        lua.script("shared.value = 20");

        REQUIRE(ptr_to_my_type->value == 20);
        REQUIRE(ref_to_my_type.value == 20);
        REQUIRE(ref_to_shared_ptr->value == 20);
    }
    {
        std::cout << "getting copy of shared_ptr..." << std::endl;
        std::shared_ptr<my_type> copy_of_shared_ptr = lua["shared"];
        std::cout << "\tshared.use_count(): " << copy_of_shared_ptr.use_count() << std::endl;
        my_type copy_of_value = lua["shared"];
        std::cout << "\tafter getting value copy of my_type: " << copy_of_shared_ptr.use_count() << std::endl;

        REQUIRE(copy_of_shared_ptr->value == 20);
        REQUIRE(copy_of_value.value == 20);

        // script still affects pointer, but does not affect copy of `my_type`
        lua.script("shared.value = 30");

        REQUIRE(copy_of_shared_ptr->value == 30);
        REQUIRE(copy_of_value.value == 20);
    }
    // set to nil and collect garbage to destroy it
    lua.script("shared = nil");
    lua.collect_garbage();
    lua.collect_garbage();

    std::cout << "garbage has been collected" << std::endl;
    std::cout << std::endl;

}