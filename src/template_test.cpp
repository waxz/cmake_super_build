//
// Created by waxz on 9/23/22.
//

#include <type_traits>
#include <iostream>
#include <tuple>
#include <type_traits>
#include "json.hpp"



struct Dog {
    std::string barkType;
    std::string color;
    int weight = 0;

    bool operator==(const Dog& rhs) const {
        return std::tie(barkType, color, weight) == std::tie(rhs.barkType, rhs.color, rhs.weight);
    }
    void wang()const{
        std::cout << "wang!!" <<std::endl;
    }
};



struct House{
    int area;
    std::string name;
    std::vector<std::vector<float>> points;

};

struct Cat {
    House house;
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

    void miao()const {
        std::cout << "miao!!" <<std::endl;

    }
};


namespace serialization{
    void to_json(nlohmann::json& data,const std::string& name , const House & object );
    void to_json(nlohmann::json& data,const std::string& name , const Cat & object );
    void from_json(nlohmann::json& data,const std::string& name ,  House& object );
    void from_json(nlohmann::json& data,const std::string& name ,  Cat& object );
}
#include "serialization_json.h"

namespace serialization{
    constexpr auto house_properties = std::make_tuple(
            property(&House::area, "area"),
            property(&House::name, "name"),
            property(&House::points, "points")

    );

    constexpr auto cat_properties = std::make_tuple(
            property(&Cat::barkType, "barkType"),
            property(&Cat::color, "color"),
            property(&Cat::weight, "weight"),
            property(&Cat::pawType, "pawType"),
            property(&Cat::nums, "nums") ,
            property(&Cat::msgs, "msgs"),
            property(&Cat::dicts, "dicts"),
            property(&Cat::house, "house")
    );
}

namespace serialization{
    void to_json(nlohmann::json& data,const std::string& name , const House & object ){
//     constexpr  auto properties = house_properties;
        to_json_unpack(data,name,object, house_properties);
    }

    void to_json(nlohmann::json& data,const std::string& name , const Cat & object ){
//    constexpr  auto properties = house_properties;
        to_json_unpack(data,name,object, cat_properties);
    }
    void from_json(nlohmann::json& data,const std::string& name ,  House& object ){

        from_json_unpack(data,name,object,house_properties);

    }
    void from_json(nlohmann::json& data,const std::string& name ,  Cat& object ){

        from_json_unpack(data,name,object,cat_properties);

    }
}

#if 1


bool test_serialization(){
    Cat cat1;

    cat1.color = "green";
    cat1.barkType = "whaf";
    cat1.weight = 30;
    cat1.nums = std::vector<int>({1, 2, 3, 4, 56, 7});
    cat1.msgs = std::vector<std::string>({"23", "44"});
    cat1.dicts.emplace("name", "mmiao");
    cat1.dicts.emplace("age", "233");
    cat1.house.area = 58;
    cat1.house.name = "tree";
    cat1.house.points.push_back(std::vector<float>({7.0,1.0}));
    cat1.house.points.push_back(std::vector<float>({1.0,6.0}));


    nlohmann::json json_data_cat1;
    serialization::to_json(json_data_cat1, "cat1", cat1);
    std::cout << "json_data_cat1: \n" << json_data_cat1.dump() << std::endl;

    Cat cat2;
    serialization::from_json(json_data_cat1, "cat1", cat2);
    nlohmann::json json_data_cat2;
    serialization::to_json(json_data_cat2, "cat2", cat2);
    std::cout << "json_data_cat2: \n" << json_data_cat2.dump() << std::endl;

    std::cout << "(cat1 == cat2) = " << std::boolalpha << (cat1 == cat2) << std::endl;

    return (cat1 == cat2);


}

//https://blog.csdn.net/TH_NUM/article/details/95323440



//c++17 feature
template <typename T>
void test_if_(const T& v){
    if constexpr(std::is_same<T, Dog>::value){

        v.wang();
    }
    if constexpr(std::is_same<T, Cat>::value){

        v.miao();
    }

}




#define DEF_IS(check_type, return_type) \
    template<typename T> \
    typename std::enable_if<std::is_same<T, check_type>::value, return_type>::type

DEF_IS(Dog, void)
test_if_2(const T& v){
    v.wang();

}
DEF_IS(Cat, void)
test_if_2(const T& v){
    v.miao();

}

template<typename T> \
    typename std::enable_if<std::is_same<T, int>::value, void>::type
test_if_3(const T& v){

}
template<typename T> \
    typename std::enable_if<std::is_same<T, Cat>::value, void>::type
test_if_3(const T& v){

}




template<typename T,
        std::enable_if_t< std::is_same<T, Cat>::value, bool> = true
        >
        void test_if_4(const T& v){

}


template<typename T,
        std::enable_if_t< std::is_same<T, Dog>::value, bool> = true
>
void test_if_4(const T& v){

}


template<typename T, std::enable_if_t< std::is_same<T, int>::value, bool> = true>
void test_if_4(const T& v){

}


#define CHECK_IS_SAME(T1, T2) template<typename T1, typename std::enable_if_t< std::is_same<T1, T2>::value, bool> = true>


CHECK_IS_SAME(T, Dog)
void test_if_5(const T& v){

}


CHECK_IS_SAME(T, Cat)
void test_if_5(const T& v){

}
CHECK_IS_SAME(T, int)
void test_if_5(const T& v){

}

#define CHECK_IS_SAME_TYPE(T1, T2) typename std::enable_if_t< std::is_same<T1, T2>::value, bool> = true
template<typename T, CHECK_IS_SAME_TYPE(T,Dog)>
void test_if_6(const T& v){

}

template<typename T, CHECK_IS_SAME_TYPE(T,Cat)>
void test_if_6(const T& v){

}
template<typename T, CHECK_IS_SAME_TYPE(T,int)>
void test_if_6(const T& v){

}

template<typename T1,typename T2, CHECK_IS_SAME_TYPE(T1,Cat),CHECK_IS_SAME_TYPE(T2,Dog)>
void test_if_6_1(const T1& v, const T2 & v2){

}

template<typename T,
        typename std::enable_if<
                !std::is_trivially_destructible<T>{} &&
                (std::is_class<T>{} || std::is_union<T>{}),
                bool>::type = true>
void destroy(T* t)
{
    std::cout << "destroying non-trivially destructible T\n";
}


#define HAS_MEM_FUNC(func, name)                                        \
    template<typename T, typename Sign>                                 \
    struct name {                                                       \
        typedef char yes[1];                                            \
        typedef char no [2];                                            \
        template <typename U, U> struct type_check;                     \
        template <typename _1> static yes &chk(type_check<Sign, &_1::func > *); \
        template <typename   > static no  &chk(...);                    \
        static bool const value = sizeof(chk<T>(0)) == sizeof(yes);     \
    }


HAS_MEM_FUNC(miao, has_maio);

template<typename T> void
doMiao(const T& v) {
    if(has_maio<T, std::string(T::*)()>::value) {
        v.miao();
    } else {
    }
}

int main(int argc, char** argv){

    test_serialization();

    Dog dog;
    Cat cat;
    test_if_(dog);
    test_if_(cat);
    test_if_2(dog);
    test_if_2(cat);
    test_if_3(1);
    test_if_3(cat);

    test_if_4(dog);
    test_if_4(cat);
    test_if_4(1);

    test_if_5(dog);
    test_if_5(cat);
    test_if_5(1);

    test_if_6(dog);
    test_if_6(cat);
    test_if_6(1);

    test_if_6_1(cat,dog);

    std::cout << "doMiao "<< std::endl;
    doMiao(cat);


}
#endif