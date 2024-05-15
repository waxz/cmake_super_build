//
// Created by waxz on 4/11/24.
//

#ifndef CMAKE_SUPER_BUILD_GENERIC_H
#define CMAKE_SUPER_BUILD_GENERIC_H
#include <utility>
#include <type_traits>

// Type trait to check if a type T is a std::vector
template <typename T>
struct is_std_vector : std::false_type {};

template <typename T, typename Alloc>
struct is_std_vector<std::vector<T, Alloc>> : std::true_type {};


template <typename T>
struct is_unordered_map : std::false_type {};
template <typename T,typename K, typename Alloc>
struct is_unordered_map<std::unordered_map<K, T, Alloc>> : std::true_type {};


template<typename  T,typename std::enable_if<
        !(std::is_same<T, bool>{}
          || std::is_same<T, int>{}
          || std::is_same<T, float>{}
          || std::is_same<T, double>{}
          || std::is_same<T, std::string>{}
          || std::is_same<T, std::vector<bool>>{}
          || std::is_same<T, std::vector<int>>{}
          || std::is_same<T, std::vector<float>>{}
          || std::is_same<T, std::vector<double>>{}
          || std::is_same<T, std::vector<std::string>>{}
          || is_std_vector<T>{}
          || is_unordered_map<T>{}


          ),
        bool>::type = true>
bool generic_enable_if(const char* name, T & object){
    return true;
}


// Helper function to create a tuple of references to struct members
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

/*
template<typename T, typename ProbType>
void from_toml_impl(T& object, ProbType properties,const toml::value& value){
    constexpr auto nbProperties = std::tuple_size<decltype(properties)>::value;
    // We iterate on the index sequence of size `nbProperties`
    for_sequence(std::make_index_sequence<nbProperties>{}, [&](auto i) {
        // get the property
        auto& property = std::get<i>(properties);
        // you can also replace `asAny` by `fromJson` to recursively serialize
//            using Type = typename std::remove_reference<decltype(object.*(property.member))>::type;
        object.*(property.member) = from_toml<std::decay_t<decltype(object.*(property.member))>>(toml::find(value, property.name));
    });
}

*/
#endif //CMAKE_SUPER_BUILD_GENERIC_H
