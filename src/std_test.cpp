//
// Created by waxz on 22-12-27.
//



#include <utility>
#include <type_traits>
#include <iostream>
#include <tuple>

class Person
{
public:
    std::string name;
    int age;
    Person(){}
};

namespace std{
    template<>
    struct tuple_size<::Person>{
        static  constexpr  size_t value = 2;
    };
#if 0
    template<>
    struct tuple_element<0,::Person>{
        using type = std::string;
    };

    template<>
    struct tuple_element<1,::Person>{
        using type = int;
    };

    template<>
    struct tuple_element<0,const ::Person&>{
        using type = const std::string&;
    };

    template<>
    struct tuple_element<1,const ::Person&>{
        using type = int &;
    };

#endif

#if 1
    template<size_t Index>
    struct tuple_element<Index,::Person>:tuple_element<Index,tuple<std::string, int>>
    {};
#endif
}

template<std::size_t Index>
std::tuple_element_t<Index, Person>& get(Person& person){
    if constexpr (Index == 0) return person.name;
    if constexpr (Index == 1) return person.age;
}

template<std::size_t Index>
std::tuple_element_t<Index, Person>const& get(const Person& person){
    if constexpr (Index == 0) return person.name;
    if constexpr (Index == 1) return person.age;
}


void test_1(){

    {


        auto c = std::make_tuple(2,3);
        auto&& [a,b] = c;

        a = 3;
        b = 6;

        std::cout << "c " << std::get<0>(c) << ", " << std::get<1>(c) << "\n";

    }
    {
        Person p;
        auto&&[pn,pa] = p;

        pn.assign("tom");
        pa = 34;
        std::cout << "c " << p.name<< ", " << p.age << "\n";

    }
    {
        const Person p;
        auto&[pn,pa] = p;

//        pn.assign("tom");
//        pa = 34;
        std::cout << "c " << p.name<< ", " << p.age << "\n";

    }

}

int main(){

    test_1();

}