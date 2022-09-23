//
// Created by waxz on 9/23/22.
//

#include <type_traits>
#include <iostream>
#include <tuple>
#include <type_traits>
#include "json.hpp"


template <typename T, typename = int>
struct HasX : std::false_type { };

template <typename T>
struct HasX <T, decltype((void) T::x, 0)> : std::true_type { };


template <typename T, typename = int>
struct HasM : std::false_type { };

template <typename T>
struct HasM <T, decltype((void) T::hello, 0)> : std::true_type { };


template <typename T>
constexpr auto haemHelper (T const &, int)
-> decltype( &T::hello, std::true_type{} );

template <typename T>
constexpr std::false_type haemHelper (T const &, long);

template <typename T>
using HasApproxEqualMethod = decltype( haemHelper(std::declval<T>(), 0) );


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

struct Dog {
    std::string barkType;
    std::string color;
    int weight = 0;

    bool operator==(const Dog& rhs) const {
        return std::tie(barkType, color, weight) == std::tie(rhs.barkType, rhs.color, rhs.weight);
    }

    constexpr static auto properties = std::make_tuple(
            property(&Dog::barkType, "barkType"),
            property(&Dog::color, "color"),
            property(&Dog::weight, "weight")
    );
};



template<typename T>
nlohmann::json toJson(const T& object) {
    nlohmann::json data;

    // We first get the number of properties
    constexpr auto nbProperties = std::tuple_size<decltype(T::properties)>::value;

    // We iterate on the index sequence of size `nbProperties`
    for_sequence(std::make_index_sequence<nbProperties>{}, [&](auto i) {
        // get the property
        constexpr auto property = std::get<i>(T::properties);

        // set the value to the member
        data[property.name] = object.*(property.member);
    });

    return data;
}

template<typename T,typename P>
nlohmann::json toJson_fix(const T& object, const P& properties) {
    nlohmann::json data;

    // We first get the number of properties
    constexpr auto nbProperties = std::tuple_size<P>::value;

    // We iterate on the index sequence of size `nbProperties`
    for_sequence(std::make_index_sequence<nbProperties>{}, [&](auto i) {
        // get the property
          auto& property = std::get<i>(properties);

        // set the value to the member
        data[property.name] = object.*(property.member);
    });

    return data;
}


// unserialize function
template<typename T>
T fromJson(const nlohmann::json & data) {
    T object;

    // We first get the number of properties
    constexpr auto nbProperties = std::tuple_size<decltype(T::properties)>::value;

    // We iterate on the index sequence of size `nbProperties`
    for_sequence(std::make_index_sequence<nbProperties>{}, [&](auto i) {
        // get the property
        constexpr auto property = std::get<i>(T::properties);

        // get the type of the property
        using Type = typename decltype(property)::Type;

        // set the value to the member
        // you can also replace `asAny` by `fromJson` to recursively serialize
        object.*(property.member) = (data[property.name]);
    });

    return object;
}

template<typename T,typename P>
T fromJson_fix(const nlohmann::json & data, const P& properties) {
    T object;

    // We first get the number of properties
    constexpr auto nbProperties = std::tuple_size<P>::value;

    // We iterate on the index sequence of size `nbProperties`
    for_sequence(std::make_index_sequence<nbProperties>{}, [&](auto i) {
        // get the property
        auto& property = std::get<i>(properties);

        // get the type of the property

        // set the value to the member
        // you can also replace `asAny` by `fromJson` to recursively serialize
        using Type = typename std::remove_reference<decltype(object.*(property.member))>::type;
        object.*(property.member) =
                data[property.name].template get<Type>();
    });

    return object;
}
struct A{
    int x;
    void hello(){

    }

};

struct B{
    float x;
    void hello2(int t){

    }
};



template <typename... Ts>
struct typelist
{
    using as_tuple = std::tuple<Ts...>;
};

template <typename... As, typename... Bs>
constexpr auto typelist_cat(typelist<As...>, typelist<Bs...>)
-> typelist<As..., Bs...>
{
    return {};
}

template<typename CONTENT, std::size_t START, std::size_t DISTANCE, std::size_t SEQ_LENGTH>
struct mkTuple
{
    using _LT = CONTENT;
    using _RT = typename mkTuple<CONTENT, START + DISTANCE, DISTANCE, SEQ_LENGTH - 1>::tuple;
    using tuple = decltype(typelist_cat(typelist<_LT>{}, _RT{}));
};


template<typename CONTENT, std::size_t START, std::size_t DISTANCE>
struct mkTuple<CONTENT, START, DISTANCE, 1>
{
    using _LT = CONTENT;
    using tuple = typelist<_LT>;
};

using tMyTuple = typename mkTuple<int, 16, 2, 64>::tuple::as_tuple;



struct Cat {
    std::string barkType;
    std::string color;
    int weight = 0;
    std::string pawType;
    std::vector<int> nums;
    std::vector<std::string> msgs;
    std::map<std::string,std::string> dicts;

    bool operator==(const Cat& rhs) const {
        return std::tie(barkType, color, weight,nums,msgs,dicts) == std::tie(rhs.barkType, rhs.color, rhs.weight,rhs.nums,rhs.msgs,rhs.dicts);
    }
    void hello(){

    }


};
int main(int argc, char** argv){


    {
        auto cat_properties = std::make_tuple(
                property(&Cat::barkType, "barkType"),
                property(&Cat::color, "color"),
                property(&Cat::weight, "weight"),
                property(&Cat::pawType, "pawType"),
                property(&Cat::nums, "nums")
                , property(&Cat::msgs, "msgs")
                , property(&Cat::dicts, "dicts")


        );

        Cat cat1;

        cat1.color = "green";
        cat1.barkType = "whaf";
        cat1.weight = 30;
        cat1.nums = std::vector<int>({1,2,3,4,56,7});
        cat1.msgs = std::vector<std::string>({"23","44"});
        cat1.dicts.emplace("name","mmiao");
        cat1.dicts.emplace("age","233");


        auto json_data1 = toJson_fix(cat1,cat_properties);
        std::cout << "json data1: \n" << json_data1.dump() << std::endl;

        Cat cat2 = fromJson_fix<Cat>(json_data1,cat_properties);
        auto json_data2 = toJson_fix(cat2,cat_properties);
        std::cout << "json data2: \n" << json_data2.dump() << std::endl;

        std::cout << "(cat1 == cat2) = " << std::boolalpha << (cat1 == cat2) << std::endl;

    }

    return 0;

    const std::size_t iTupleLength = std::tuple_size<tMyTuple>::value;

    const tMyTuple myTuple;
    std::cout << "Tuple[4].size() =  " << std::get<4>(myTuple) << std::endl;


    std::tuple<int,int> t1(1,1);

    std::tuple<int,int,int> t2(1,1,4);





    Dog dog;

    dog.color = "green";
    dog.barkType = "whaf";
    dog.weight = 30;

    auto jsonDog = toJson(dog); // produces {"color":"green", "barkType":"whaf", "weight": 30}
    auto dog2 = fromJson<Dog>(jsonDog);




    std::cout << "(dog == dog2) = " << std::boolalpha << (dog == dog2) << std::endl;

    std::cout << "A HasX: " << HasX<A>() << std::endl;

    std::cout << "B HasX: " << HasX<B>() << std::endl;
    std::cout << "A HasM: " << HasM<A>() << std::endl;

    std::cout << "B HasM: " << HasM<B>() << std::endl;

    std::cout << "A HasApproxEqualMethod: " << HasApproxEqualMethod<A>() << std::endl;

    std::cout << "B HasApproxEqualMethod: " << HasApproxEqualMethod<B>() << std::endl;

    std::cout << HasX<A>::value << std::endl; // 1
    std::cout << HasX<B>::value << std::endl; // 0

}