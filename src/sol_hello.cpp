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
    lua.open_libraries(sol::lib::base, sol::lib::package,sol::lib::os);

    lua.script(R"(
os.execute([[ echo test > "/tmp/xyz.txt" ]])
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

    std::cout << "=== any_return ===" << std::endl;
    std::cout << "result : " << result << std::endl;
    std::cout << "result2: " << result2 << std::endl;
    std::cout << "result3: " << result3 << std::endl;
    std::cout << std::endl;
}