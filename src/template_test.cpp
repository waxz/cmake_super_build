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
int main(int argc, char** argv){

    test_serialization();

}
#endif