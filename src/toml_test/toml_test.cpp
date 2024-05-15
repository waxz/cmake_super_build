#include <iostream>
#include <fstream>
#include <vector>
#include <type_traits>
#include <unordered_map>
#include <tuple>
#include <utility>

#include "pfr.hpp"

#include "common/generic.h"


//#include "serialize/serialize_toml_base.h"
//#include "serialize/serialize_json_base.h"
#include <nlohmann/json.hpp>
#include <toml.hpp>


// Define a House struct

namespace Test{
    struct House {
        std::string address;
        int rooms;
        House(std::string address_,   int rooms_ ): address{address_},rooms{rooms_}{

        }

        House(){}
        House(const toml::value& value) {
            // Extract integer value from TOML value
            rooms= toml::get<decltype(rooms)>(value.at("rooms"));

            // Extract string value from TOML value
            address = toml::get<decltype(address)>(value.at("address"));
        }
        toml::value into_toml() const // you need to mark it const.
        {
            return toml::value{{"rooms", this->rooms}, {"address", this->address} };
        }


    };



    void from_json(const nlohmann::json & value,House& object ){
        from_json(value["rooms"], object.rooms);
        from_json(value["address"], object.address);

    }

    void to_json( nlohmann::json & value,const House& object ){
        to_json(value["rooms"], object.rooms);
        to_json(value["address"], object.address);
    }
}



namespace ros_helper{

    struct House {
        std::string address;
        int rooms;
        int are = 100;
        House(std::string address_,   int rooms_ ): address{address_},rooms{rooms_}{

        }

        House(){}
        House(const toml::value& value) {
            // Extract integer value from TOML value
            rooms= toml::get<decltype(rooms)>(value.at("rooms"));

            // Extract string value from TOML value
            address = toml::get<decltype(address)>(value.at("address"));
        }
        toml::value into_toml() const // you need to mark it const.
        {
            return toml::value{{"rooms", this->rooms}, {"address", this->address} };
        }


    };



    void from_json(const nlohmann::json & value,House& object ){
        from_json(value["rooms"], object.rooms);
        from_json(value["address"], object.address);

    }

    void to_json( nlohmann::json & value,const House& object ){
        to_json(value["rooms"], object.rooms);
        to_json(value["address"], object.address);
    }
/// GEN[TOML] GEN[JSON]
    struct Channel{
        std::string channel_type;
        std::string topic_name;
        std::string topic_type;
        int qos_queue_size = 10;


        Channel(const toml::value& value) {
            channel_type= toml::get<decltype(channel_type)>(value.at("channel_type"));
            topic_name= toml::get<decltype(topic_name)>(value.at("topic_name"));
            topic_type= toml::get<decltype(topic_type)>(value.at("topic_type"));
            qos_queue_size= toml::get<decltype(qos_queue_size)>(value.at("qos_queue_size"));

        }

        toml::value into_toml() const {
            return toml::value{
                    {"channel_type", this->channel_type},
                    {"topic_name", this->topic_name},
                    {"topic_type", this->topic_type},
                    {"qos_queue_size", this->qos_queue_size}};
        }

    };

    void from_json(const nlohmann::json & value,Channel& object ){
        from_json(value["channel_type"], object.channel_type);
        from_json(value["topic_name"], object.topic_name);
        from_json(value["topic_type"], object.topic_type);
        from_json(value["qos_queue_size"], object.qos_queue_size);

    }


    void to_json( nlohmann::json & value,const Channel& object ){
        to_json(value["channel_type"], object.channel_type);
        to_json(value["topic_name"], object.topic_name);
        to_json(value["topic_type"], object.topic_type);
        to_json(value["qos_queue_size"], object.qos_queue_size);

    }

/// GEN[TOML] GEN[JSON]
    struct Config{
        std::unordered_map < std::string , std::string > config;
        std::unordered_map < std::string ,  House > channel;


        Config(const toml::value& value) {
            config= toml::get<decltype(config)>(value.at("config"));
            channel= toml::get<decltype(channel)>(value.at("channel"));

        }

        toml::value into_toml() const {
            return toml::value{
                    {"config", this->config},
                    {"channel", this->channel}};
        }

    };

    void from_json(const nlohmann::json & value,Config& object ){
        from_json(value["config"], object.config);
        from_json(value["channel"], object.channel);

    }


    void to_json( nlohmann::json & value,const Config& object ){
        to_json(value["config"], object.config);
        to_json(value["channel"], object.channel);

    }
}
namespace Test2{

// Define a Person struct with a vector of hobbies and an unordered_map of attributes
    struct Person {
        std::string name;
        int age;
        std::vector<std::string> hobbies;
        std::unordered_map<std::string, std::string> attributes;
        Test::House house;
        std::vector<Test::House> houses_vec;
        std::unordered_map<std::string, Test::House> houses_map;
        Person(){}
        Person(const toml::value& value){
            name= toml::get<decltype(name)>(value.at("name"));
            age= toml::get<decltype(age)>(value.at("age"));
            hobbies= toml::get< decltype(hobbies)>(value.at("hobbies"));
            attributes= toml::get<decltype(attributes)>(value.at("attributes"));
            house= toml::get<decltype(house)>(value.at("house"));
            houses_vec= toml::get<decltype(houses_vec) >(value.at("houses_vec"));
            houses_map= toml::get<decltype(houses_map) >(value.at("houses_map"));
        }
        toml::value into_toml() const // you need to mark it const.
        {
            return toml::value{{"name", this->name}, {"age", this->age}, {"hobbies", this->hobbies},

                               {"attributes", this->attributes},{"house", this->house}
                    ,{"houses_vec", this->houses_vec},{"houses_map", this->houses_map}
            };
        }
    };

    void from_json(const nlohmann::json & value,Person& object ){
        from_json(value["name"], object.name);
        from_json(value["age"], object.age);
        from_json(value["hobbies"], object.hobbies);
        from_json(value["attributes"], object.attributes);
        from_json(value["house"], object.house);
        from_json(value["houses_vec"], object.houses_vec);
        from_json(value["houses_map"], object.houses_map);
    }

    void to_json(nlohmann::json & value,const Person& object ){
        to_json(value["name"], object.name);
        to_json(value["age"], object.age);
        to_json(value["hobbies"], object.hobbies);
        to_json(value["attributes"], object.attributes);
        to_json(value["house"], object.house);
        to_json(value["houses_vec"], object.houses_vec);
        to_json(value["houses_map"], object.houses_map);
    }
}


// Templated function to create a struct from TOML data
//template<typename T>
//T from_toml(const toml::value& value) {
//    T object;
//    object = toml::get<T>(value);
//    return object;
//}




// Specialization of from_toml for House struct
//template<>
//Test::House from_toml<Test::House>(const toml::value& value) {
//    Test::House house;
//    house.address = toml::find<std::string>(value, "address");
//    house.rooms = toml::find<int>(value, "rooms");
//    return house;
//}

// Specialization of from_toml for Person struct
//template<>
//Person from_toml<Person>(const toml::value& value) {
//    Person person;
//    person.name = toml::find<std::string>(value, "name");
//    person.age = toml::find<int>(value, "age");
//    person.hobbies = toml::find<std::vector<std::string>>(value, "hobbies");
//    person.attributes = toml::find<std::unordered_map<std::string, std::string>>(value, "attributes");
//    person.house = from_toml<House>(toml::find(value, "house"));
//    return person;
//}



struct some_person {
    std::string name;
    unsigned birth_year;
};

// Specialization of from_toml for Person struct

struct Rocket {
    int x;
    double y;
    std::string name;
};

//
//template<typename T, typename ProbType>
//void from_toml_impl(T& object, ProbType properties,const toml::value& value){
//    constexpr auto nbProperties = std::tuple_size<decltype(properties)>::value;
//    // We iterate on the index sequence of size `nbProperties`
//    for_sequence(std::make_index_sequence<nbProperties>{}, [&](auto i) {
//        // get the property
//        auto& property = std::get<i>(properties);
//        // you can also replace `asAny` by `fromJson` to recursively serialize
////            using Type = typename std::remove_reference<decltype(object.*(property.member))>::type;
//        object.*(property.member) = from_toml<std::decay_t<decltype(object.*(property.member))>>(toml::find(value, property.name));
//    });
//}


// Specialization of from_toml for Person struct
//template<>
//Rocket from_toml<Rocket>(const toml::value& value) {
//    Rocket object;
//    constexpr auto properties = std::make_tuple(
//            property(&Rocket::x, "x"),
//            property(&Rocket::y, "y"),
//            property(&Rocket::name, "name")
//    );
//    from_toml_impl(object, properties,value);
//    return object;
//}


//template<>
//Person from_toml<Person>(const toml::value& value) {
//    Person object;
//    constexpr auto properties = std::make_tuple(
//            property(&Person::name, "name"),
//            property(&Person::age, "age"),
//            property(&Person::hobbies, "hobbies"),
//            property(&Person::attributes, "attributes"),
//            property(&Person::house, "house"),
//            property(&Person::houses_vec, "houses_vec"),
//            property(&Person::houses_map, "houses_map")
//
//    );
//    from_toml_impl(object, properties,value);
//    return object;
//}



int main() {




    {
        some_person val{"Edgar Allan Poe", 1809};


        using T1 = decltype(pfr::get<0>(std::declval<const some_person&>()));
        using T2 = decltype(pfr::get<1>(std::declval<const some_person&>()));

        T1 v1 = pfr::get<0>(val);
        T2 v2 = pfr::get<1>(val);


        std::cout << pfr::get<0>(val)                // No macro!
                  << " was born in " << pfr::get<1>(val) << "\n";  // Works with any aggregate initializables!
        std::cout << v1              // No macro!
                  << " was born in " << v2<< "\n";  // Works with any aggregate initializables!

    }

    {
        std::ifstream ifs("example.json");
        std::cout << "\n===== JSON"  << std::endl;

        if (ifs.is_open()){
            try{


                Test2::Person person;
                nlohmann::json person_data =  nlohmann::json::parse(ifs);;

                from_json(person_data["person"], person);
                nlohmann::json person_data_back;
                to_json(person_data_back,person );
                std::cout << "json person_data_back: " << person_data_back.dump() << std::endl;

                // Print the deserialized data
                std::cout << "person Name: " << person.name << std::endl;
                std::cout << "person Age: " << person.age << std::endl;
                std::cout << "person Hobbies:" << std::endl;
                for (const auto& hobby : person.hobbies) {
                    std::cout << "- " << hobby << std::endl;
                }
                std::cout << "person Attributes:" << std::endl;
                for (const auto& attribute : person.attributes) {
                    std::cout << "- " << attribute.first << ": " << attribute.second << std::endl;
                }
                std::cout << "person House:" << std::endl;
                std::cout << "- Address: " << person.house.address << std::endl;
                std::cout << "- Rooms: " << person.house.rooms << std::endl;

                {
                    std::cout << "person houses_vec:" << std::endl;

                    for (auto h :person.houses_vec){
                        std::cout << "- Address: " << h.address << std::endl;
                        std::cout << "- Rooms: " << h.rooms << std::endl;

                    }
                }

                {
//                    std::cout << "person houses_map:" << std::endl;
//
//                    for (auto it = person.houses_map.begin(); it!= person.houses_map.end(); it++){
//                        std::cout << "- key: " << it->first << std::endl;
//
//                        std::cout << "- Address: " << it->second.address << std::endl;
//                        std::cout << "- Rooms: " << it->second.rooms << std::endl;
//
//                    }
                }


            }catch (std::exception &e){
                std::cout  << e.what() << std::endl;

            }catch (...){
                std::cout  << "JSON fail" << std::endl;

            }
            ifs.close();

        }

    }


    try{

        // Read and parse the TOML file
        const auto data = toml::parse("example.toml");

        // Deserialize the data into a Person object
        const auto person_data = toml::find(data, "person");


        Test2::Person person = person_data;//from_toml<Person>(person_data);

        {
            auto person_data_bak = person.into_toml();
            std::cout << "toml person_data_bak: " << person_data_bak << std::endl;

        }



        // Print the deserialized data
        std::cout << "person Name: " << person.name << std::endl;
        std::cout << "person Age: " << person.age << std::endl;
        std::cout << "person Hobbies:" << std::endl;
        for (const auto& hobby : person.hobbies) {
            std::cout << "- " << hobby << std::endl;
        }
        std::cout << "person Attributes:" << std::endl;
        for (const auto& attribute : person.attributes) {
            std::cout << "- " << attribute.first << ": " << attribute.second << std::endl;
        }
        std::cout << "person House:" << std::endl;
        std::cout << "- Address: " << person.house.address << std::endl;
        std::cout << "- Rooms: " << person.house.rooms << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
