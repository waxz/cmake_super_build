//
// Created by waxz on 4/22/23.
//


//https://stackoverflow.com/a/45365798


#include<type_traits>
#include<utility>
#include <iostream>

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


void foo(void (*fn)())
{
    fn();
}


int main(int argc, char** argv){

    int i = 42;
    auto fn = fnptr<void()>([&i]{ i++, std::cout << "i = " << i << "\n";});
    foo(fn);  // compiles!
    foo(fn);  // compiles!
    foo(fn);  // compiles!

}