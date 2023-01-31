#define SOL_CHECK_ARGUMENTS 1
#include <sol.hpp>

#include <iostream>

#include <thread>

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


    sol::state lua;
    std::cout << "=== opening a state ===" << std::endl;

    // open some common libraries
    lua.open_libraries(sol::lib::base, sol::lib::package,sol::lib::os, sol::lib::table, sol::lib::jit,sol::lib::coroutine);



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