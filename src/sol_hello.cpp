#define SOL_CHECK_ARGUMENTS 1
#include <sol.hpp>

#include <iostream>

#include <thread>


#include "common/string_logger.h"

#include "assert.hpp"

#include "lyra/lyra.hpp"

// Uses some of the fancier bits of sol2, including the "transparent argument",
// sol::this_state, which gets the current state and does not increment
// function arguments
sol::object fancy_func(sol::object a, sol::object b, sol::this_state s) {
    sol::state_view lua(s);
    if (a.is<int>() && b.is<int>()) {
        return sol::object(lua, sol::in_place, a.as<int>() + b.as<int>());
    }
    else if (a.is<bool>()) {
        bool do_triple = a.as<bool>();
        return sol::object(lua, sol::in_place_type<double>, b.as<double>() * (do_triple ? 3 : 1));
    }
    // Can also use make_object
    return sol::make_object(lua, sol::nil);
}

struct Player {
    int player_id_;

    Player() : player_id_(1) {}

    Player(int player_id) : player_id_(player_id) {}

    void hello() {
        std::cout << "hello player " << player_id_ << std::endl;
    }

    void f1(int i){
        std::cout << "hello int " << i << std::endl;

    }

    void f2(float i){
        std::cout << "hello float " << i << std::endl;

    }
    void f3(double i){
        std::cout << "hello double " << i << std::endl;

    }

    int operator[](int index){
        std::cout << "hello index " << index << std::endl;

        return index;
    }
};


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

void will_throw() {
    throw std::runtime_error("oh no not an exception!!!");
}

void test_error_handler(lua_State* L){
    sol::state_view lua(L);
    lua.script("print('pass lua state to function, test error handler')");
    //
    {
        auto result = lua.script(
                "print('hello hello again, world') \n return 24",
                simple_handler);
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

    {
        auto result
                = lua.script("does.not.exist", simple_handler);
        if (result.valid()) {
            std::cout << "the fourth script worked, which it "
                         "wasn't supposed to! Panic!"
                      << std::endl;
            int value = result;
        }
        else {
            sol::error err = result;
            std::cout << "the fourth script failed, which "
                         "was intentional!\t\nError: "
                      << err.what() << std::endl;
        }
    }
}

void some_function() {
    std::cout << "some function!" << std::endl;
}

void some_other_function() {
    std::cout << "some other function!" << std::endl;
}

struct some_class {
    int variable = 30;

    double member_function() {
        return 24.5;
    }

    void hello(){
        MLOGI("%s", "some_class say hello");

    }
};

void test_binding_class(lua_State* L){
    sol::state_view lua(L);
    lua.script("print('test_binding_class')");

    // put an instance of "some_class" into lua
    // (we'll go into more detail about this later
    // just know here that it works and is
    // put into lua as a userdata
    lua.set("sc1", some_class());
    lua["sc2"] = some_class{};
    some_class sc;
    lua["sc"] = sc;

    // binds a plain function
    lua["f1"] = some_function;
    lua.set_function("f2", &some_other_function);

    // binds just the member function
    lua["m1"] = &some_class::member_function;

    // binds the class to the type
    lua.set_function(
            "m2", &some_class::member_function, some_class {});

    // binds just the member variable as a function
    lua["v1"] = &some_class::variable;

    // binds class with member variable as function
    lua.set_function(
            "v2", &some_class::variable, some_class {});

    lua.script(R"(
	f1() -- some function!
	f2() -- some other function!

	-- need class instance if you don't bind it with the function
	print(m1(sc)) -- 24.5
	-- does not need class instance: was bound to lua with one
	print(m2()) -- 24.5

	-- need class instance if you
	-- don't bind it with the function
	print(v1(sc)) -- 30
	-- does not need class instance:
	-- it was bound with one
	print(v2()) -- 30
	-- can set, still
	-- requires instance
	v1(sc, 212)
	-- can set, does not need
	-- class instance: was bound with one
	v2(254)
	print(v1(sc)) -- 212
	print(v2()) -- 254
	)",simple_handler);
}

int main(int argc, char** argv) {

    std::cout << "LUA"<<std::endl;
    int width = 0;
    auto cli = lyra::cli()
               | lyra::opt( width, "width" )
               ["-w"]["--width"]
                       ("How wide should it be?");

    auto cli_result = cli.parse( { argc, argv } );
    if ( !cli_result )
    {
        std::cerr << "Error in command line: " << cli_result.message() << std::endl;
        exit(1);
    }

    lua_State* L = luaL_newstate();
    sol::state_view lua(L);
    lua.script("print('hello lua!')", simple_handler);

    lua.open_libraries(sol::lib::base, sol::lib::package,sol::lib::os, sol::lib::table, sol::lib::jit,sol::lib::coroutine);

    lua.set_exception_handler(&my_exception_handler);
    // call lua code, and check to make sure it has loaded and
    // run properly:
    auto handler = &sol::script_default_on_error;
    lua.script("print('hello again, world')", handler);

    // Use a custom error handler if you need it
    // This gets called when the result is bad
    auto simple_handler_lambda =
            [](lua_State*, sol::protected_function_result result) {
                // You can just pass it through to let the
                // call-site handle it
                return result;
            };
    // the above lambda is identical to sol::simple_on_error,
    // but it's shown here to show you can write whatever you
    // like


    test_error_handler(lua);
    test_binding_class(lua);

    std::cout << "c++ main exit" << std::endl;

    return 0;



    {
        lua.script(R"(
			function handler (message)
				return "Handled this message: " .. message
			end

			function f (a)
				if a < 0 then
					error("negative number detected")
				end
				return a + 5
			end
		)");

        // Get a protected function out of Lua
        sol::protected_function f(lua["f"], lua["handler"]);

        sol::protected_function_result result = f(-500);
        if (result.valid()) {
            // Call succeeded
            int x = result;
            std::cout << "call succeeded, result is " << x << std::endl;
        }
        else {
            // Call failed
            sol::error err = result;
            std::string what = err.what();
            std::cout << "call failed, sol::error::what() is " << what << std::endl;
            // 'what' Should read
            // "Handled this message: negative number detected"
        }

        std::cout << std::endl;

    }
    if(1){

        lua.script(
                R"(
function handler ( message )
    return "handler " .. message .. debug.traceback()
end
)"
        );

        sol::function luahandler = lua[ "handler" ];

        // Make sure handler works
        luahandler();
        // Set it

        // Some function; just using a lambda to be cheap
        auto doom = []() {
            // Bypasses handler function: puts information directly into lua error
            throw std::logic_error( "dun goofed, little man" );
//        std::cout << "domm1" << std::endl;
        };
        auto luadoom = [&lua]() {
            // Does not bypass error function, will call it
            lua_error( lua.lua_state() );
        };


        lua.set_function( "doom", doom );
        lua.set_function( "luadoom", luadoom );

        sol::function func = lua[ "doom" ];
        sol::function luafunc = lua[ "luadoom" ];

        try{
            std::cout << "try result 1";

            sol::function_result result1 = func();
            std::cout << "result 1";

        }catch (sol::error & e){
            std::cout << "result1 error:\n" << e.what() << '\n';

        }catch (...){
            std::cout << "result1 error 0  \n";

        }
        try{
            std::cout << "try result 2";

            // Appropriately scoped
            luafunc.error_handler = luahandler;
            sol::function_result result2 = luafunc();
            std::cout << "result2 success";

        }catch (sol::error & e){
            std::cout << "result2 error:\n" << e.what() << '\n';

        }catch (...){
            std::cout << "result2 error 0  \n";

        }

        std::cout << "=== error_handler ===" << std::endl;
    }

    lua.script(R"(
 a = 123;
a.1= 2;

)",[](lua_State*, sol::protected_function_result pfr) {

        std::cout << "script error"<< std::endl;

        // pfr will contain things that went wrong, for either loading or executing the script
        // Can throw your own custom error
        // You can also just return it, and let the call-site handle the error if necessary.
        return pfr;
    });

    std::cout << "=== end "<< std::endl;


    lua.script(R"(
function loop()
	while counter ~= 30
	do
		coroutine.yield(counter);
		counter = counter + 1;
	end
	return counter
end

)");


    {

        sol::coroutine loop_coroutine_1 = lua["loop"];
        sol::coroutine loop_coroutine_2 = lua["loop"];

        lua["counter"] = 20;

        // example of using and re-using coroutine
        // you do not have to use coroutines in a loop,
        // this is just the example

        // we start from 0;
        // we want 10 values, and we only want to
        // run if the coroutine "loop_coroutine" is valid
        for (int counter = 0; counter < 20 && loop_coroutine_1 && loop_coroutine_2; ++counter) {
            // Alternative: counter < 10 && cr.valid()

            // Call the coroutine, does the computation and then suspends
            // once it returns, we get the value back from the return
            // and then can use it
            // we can either leave the coroutine like that can come to it later,
            // or loop back around
            int value_1 = loop_coroutine_1();
            std::cout << "no thread value_1 is " << value_1 << std::endl;
            int value_2 = loop_coroutine_2();
            std::cout << "no thread value_2 is " << value_2 << std::endl;
        }
    }
    {

        sol::thread runner_1 = sol::thread::create(lua.lua_state());
        sol::state_view runnerstate_1 = runner_1.state();
        sol::coroutine loop_coroutine_1 = runnerstate_1["loop"];
        sol::thread runner_2 = sol::thread::create(lua.lua_state());
        sol::state_view runnerstate_2 = runner_2.state();
        sol::coroutine loop_coroutine_2 = runnerstate_2["loop"];

        lua["counter"] = 20;

        for (int counter = 0; counter < 20 && loop_coroutine_1 && loop_coroutine_2; ++counter) {
            // Call the coroutine, does the computation and then suspends
            int value_1 = loop_coroutine_1();
            std::cout << "value_1 is " << value_1 << std::endl;
            int value_2 = loop_coroutine_2();
            std::cout << "value_2 is " << value_2 << std::endl;


        }
    }




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

    lua.script(R"(
os.execute([[

echo $ROS_DISTRO >> ./xyz.txt
source /opt/ros/$ROS_DISTRO/setup.sh
rosnode list >> ./xyz.txt 2>&1
]])
)");

    lua.script(R"(
print("test luajit api : table.isempty")

local isempty = require "table.isempty"

print(isempty({}))  -- true
print(isempty({nil, dog = nil}))  -- true
print(isempty({"a", "b"}))  -- false
print(isempty({nil, 3}))  -- false
print(isempty({cat = 3}))  -- false
)");

    lua.script("print('bark bark bark!')");

    lua.script(" c = 10");
    lua.script(" c2 = 1");

    std::thread t1([&]{

        for(int i = 0 ;i < 10; i++){
            lua.script(" c = c + 1");

            lua.script("print('t1, c='..c)");
        }
    });
    std::thread t2([&]{
        for(int i = 0 ;i < 10; i++){

        }

    });

    t1.join();
    t2.join();
    lua.script("print('t0, c='..c)");

    lua["f"] = fancy_func;

    int result = lua["f"](1, 2);
    // result == 3
    c_assert(result == 3);
    double result2 = lua["f"](false, 2.5);
    // result2 == 2.5
    c_assert(result2 == 2.5);

    // call in Lua, get result
    // notice we only need 2 arguments here, not 3 (sol::this_state is transparent)
    lua.script("result3 = f(true, 5.5)");
    double result3 = lua["result3"];
    // result3 == 16.5
    c_assert(result3 == 16.5);

    sol::function transferred_into;
    lua["f2"] = [&lua, &transferred_into](sol::object t, sol::this_state this_L) {
        std::cout << "state of main     : " << (void *) lua.lua_state() << std::endl;
        std::cout << "state of function : " << (void *) this_L.lua_state() << std::endl;
        // pass original lua_State* (or sol::state/sol::state_view)
        // transfers ownership from the state of "t",
        // to the "lua" sol::state
        transferred_into = sol::function(lua, t);
    };





        lua.new_usertype<Player>("Player",
                                 sol::constructors<Player(), Player(int)>(),
                // bind as variable
                                 "player_id", &Player::player_id_,
                // bind as function
                                 "hello", sol::as_function(&Player::hello),
                                 "f", sol::overload(&Player::f1, &Player::f2, &Player::f3),
                                 sol::meta_function::index, [](Player& player,const int& index){

            std::cout << "get PP index"<< std::endl;
                    return index;
        },
                                 sol::meta_function::new_index, [](Player& player,const int& index, int v){

                    std::cout << "set PP index"<< std::endl;
                }
        );

    Player b;
    lua.set("P1", &b);
    lua["P2"] = Player(2);
    lua.script(R"(
P1:hello();
P2:hello();
P1:f(1);
P2:f(1.2);
a = {};
a[0] = 3;
P1[1] = 3;
print(P1[1]);


	)");



    // make usertype metatable
    // "bark" namespacing in Lua
    // namespacing is just putting things in a table
    sol::table bark = lua.create_named_table("bark");


    bark.new_usertype<std::vector<float>>("vector_float",
                                          sol::constructors<std::vector<float>(int)>(),
                                          "size", &std::vector<float>::size,


                                          sol::meta_function::index, [](std::vector<float> &ns, int i) -> float & {
                return ns[i]; // treat like a container, despite is_container specialization
            },
                                          sol::meta_function::new_index, [](std::vector<float> &ns, int i, float v)   {
                ns[i] = v; // treat like a container, despite is_container specialization
            },
                                          "iterable", [](std::vector<float> &ns) {
                return sol::as_container(ns); // treat like a container, despite is_container specialization
            }
    );




    std::cout << "=== any_return ===" << std::endl;
    std::cout << "result : " << result << std::endl;
    std::cout << "result2: " << result2 << std::endl;
    std::cout << "result3: " << result3 << std::endl;
    std::cout << std::endl;
}