//
// Created by waxz on 4/28/23.
//

#ifndef CMAKE_SUPER_BUILD_FUNCTIONS_H
#define CMAKE_SUPER_BUILD_FUNCTIONS_H
#include<type_traits>
#include<utility>
#include <iostream>
//https://stackoverflow.com/a/45365798

namespace common{

    ///std::atomic_bool program_run(true);
    ///auto my_handler = common::fnptr<void(int)>([&](int sig){ std::cout << "get sig " << sig;program_run = false;});
    ///common::set_signal_handler(my_handler);

        template<typename Callable>
        union storage
        {
            storage() {}
            std::decay_t<Callable> callable;
        };

        template<int, typename Callable, typename Ret, typename... Args>
        auto fnptr_(Callable&& c, Ret (*)(Args...))
        {
            static bool used = false;
            static storage<Callable> s;
            using type = decltype(s.callable);

            if(used)
                s.callable.~type();
            new (&s.callable) type(std::forward<Callable>(c));
            used = true;

            return [](Args... args) -> Ret {
                return Ret(s.callable(std::forward<Args>(args)...));
            };
        }

        template<typename Fn, int N = 0, typename Callable>
        Fn* fnptr(Callable&& c)
        {
            return fnptr_<N>(std::forward<Callable>(c), (Fn*)nullptr);
        }

}


#endif //CMAKE_SUPER_BUILD_FUNCTIONS_H
