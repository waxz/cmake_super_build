//
// Created by waxz on 4/10/24.
//

#include <iostream>

#include <unordered_map>
#include <typeinfo>
#include <type_traits>
#include <utility>


// types
#include "absl/types/any.h"
#include "absl/types/optional.h"
#include "absl/types/variant.h"
#include "absl/types/span.h"
#include "absl/meta/type_traits.h"

#include "common/variant_builder.h"

#include "absl_hash_seed_test.h"

// Define a base interface class
class Base {
public:
    virtual void print() const = 0;
    virtual ~Base() {}
};


// Define concrete implementations of the interface
class Integer : public Base {
private:
    int value = 0; // Default value initializer
public:
    Integer( ) : value(0) {}

    Integer(int v) : value(v) {}

    void print() const override {
        std::cout << "Integer: " << value << std::endl;
    }
};

class String : public Base {
private:
    std::string value = ""; // Default value initializer
public:
    String() : value("") {}

    String(const std::string& v) : value(v) {}

    void print() const override {
        std::cout << "String: " << value << std::endl;
    }
};



// Function template for integral types
template <typename T,
        absl::enable_if_t<std::is_same<int, T>::value, int> = 0>
void foo(T value) {
    // Implementation for integral types
    std::cout << "foo int " << value << "\n";
}

// Function template for floating-point types
template <typename T,
absl::enable_if_t<std::is_same<float, T>::value, int> = 0>
void foo(T value) {
    // Implementation for floating-point types
    std::cout << "foo float " << value << "\n";

}

// Function template for integral types
template <typename T,
        std::enable_if_t<std::is_same<int, T>::value, int> = 0>
void foo2(T value) {
    // Implementation for integral types
    std::cout << "foo int " << value << "\n";
}

// Function template for floating-point types
template <typename T,
        std::enable_if_t<std::is_same<float, T>::value, int> = 0>
void foo2(T value) {
    // Implementation for floating-point types
    std::cout << "foo float " << value << "\n";

}
int main(){

    {
        absl::InlinedVector<int,10> vec ={};
        vec.resize(10,0);
        vec.clear();

        int vec2[10] = {0};
        for(size_t i = 0 ; i < 10;i ++){
            std::cout << "-- i: " << i << " : " << vec[i] << std::endl;
        }

        for(size_t i = 0 ; i < 10;i ++){
            vec.push_back(i);
        }
        for(size_t i = 0 ; i < 10;i ++){
            vec.pop_back();
        }
        for(size_t i = 0 ; i < 10;i ++){
            std::cout << "-- i: " << i << " : " << vec[i] << std::endl;
        }
        for(size_t i = 0 ; i < 11;i ++){
            vec.push_back(i);
        }

        for(size_t i = 0 ; i < 11;i ++){
            std::cout << "-- i: " << i << " : " << vec[i] << std::endl;
        }
        {

            std::vector<absl::variant<int, float,double> > variant_vec;
            variant_vec.push_back(1);
            variant_vec.push_back(1.2f);
            variant_vec.push_back(1.3);
            float ret = 0.0;
            for( int i = 0 ; i < variant_vec.size(); i++){

                absl::visit([&ret](auto& x){ ret =  x;},variant_vec[i]);
                std::cout << "ret: " << ret << std::endl;
            }


        }


    }
    {
        foo(1);
        foo(1.1f);
        foo2(1);
        foo2(1.1f);
    }

    {
        const char* int_typeName = typeid(int).name();

        // Print the name of the type
        std::cout << "int_typeName: " << int_typeName << std::endl;

        const char* Integer_typeName = typeid(Integer).name();

        // Print the name of the type
        std::cout << "Integer_typeName: " << Integer_typeName << std::endl;


    }
    {
        int myInt = 42;

        // Hash the integer using absl::Hash
        size_t hashValue = absl::Hash<int>{}(myInt);

        // Print the hash value
        std::cout << "Hash value of " << myInt << ": " << hashValue << std::endl;

        absl_hash_seed_test test;
        test.hash(myInt);


    }
    {
        int array[] = {1, 2, 3, 4, 5};

        // Create a span representing the entire array
        absl::Span<int> span_array(array, 5);
        absl::Span<int> span_array_2(array );


        span_array[0] = 9;
        std::fill_n(span_array.begin(),5,99);


        std::cout <<"array[0] : " <<  array[0] << "\n";
        try {
            std::cout <<"span_array.at(6) : " << span_array.at(6) << "\n";

        }catch (...){
            std::cout <<"span_array.at(6) fail \n";

        }

        std::cout << "span_array.size: " << span_array.size() << "\n";
        std::cout << "span_array_2.size: " << span_array_2.size() << "\n";

    }
    {
        auto a = absl::any(65);                 // Literal, copyable
        a = absl::any(65.0);

        auto b = absl::any(std::vector<int>()); // Default-initialized, copyable

        // Create an absl::any object containing an integer
        absl::any value = 42.0;

        // Cast the absl::any object to an integer
        try {
            int result = absl::any_cast<int>(value);
            std::cout << "any_cast Value: " << result << std::endl;
        } catch (const absl::bad_any_cast& e) {
            std::cerr << "any_cast Failed to cast the value: " << e.what() << std::endl;
        }

    }


    // Register creators for dynamic types
    {
        generic::TypeRegistry< absl::variant<Integer, String>,  const std::string&,const int& > typeRegistry;
        // Define your creator function
        std::function<String( const std::string&,const int&   )> String_creator = []( const std::string& value, const int& v) -> String {
            std::cout << "String_creator: " << value
                                  << ", v: " << v
                      << "\n";
            return String(value);
        };
        typeRegistry.registerType("String", String_creator);

        typeRegistry.createDynamicType("String", "aaa",12);

                std::function<Integer(const std::string&, const int&)> Integer_creator = [](const std::string& value, const int& v) -> Integer {
            std::cout << "Integer_creator: " <<  value
                                             << ", v: " << v
                                             << "\n";
            return   Integer(std::stoi(value));;
        };
        typeRegistry.registerType("Integer",Integer_creator );
        typeRegistry.createDynamicType("Integer", "12",55);

    }
#if 0
    {




// Register the creator function with the TypeRegistry instance
//
//        std::function<Integer(const std::string&, const char* v2)> Integer_creator = [](const std::string& value, const char* v2) -> Integer {
//            std::cout << "Integer_creator: " << value<< ", v2: " << v2;
//
//            return   Integer(std::stoi(value));;
//        };
//
//        typeRegistry.registerType<Integer, std::string, const char*>("Integer",Integer_creator );


//
//        typeRegistry.registerType<Integer,const std::string&>("integer", [](const std::string& value) { return Integer(std::stoi(value)); });
//        typeRegistry.registerType<String>("string", [](const std::string& value) { return String(value); });

        // Create dynamic types using string literals
        auto dynType1 = typeRegistry.createDynamicType("Integer", "42");
        auto dynType2 = typeRegistry.createDynamicType("String", "Hello, world!");

        std::cout << "\ncheck dynType\n";
        absl::visit([](const auto& value) {
            value.print();
        }, dynType1);

        absl::visit([](const auto& value) {
            value.print();
        }, dynType2);

        std::cout << "\nfinish check dynType\n";

        std::unordered_map<std::string, absl::variant<Integer, String >  > dynamic_dict;
        dynamic_dict["dynType1"] = dynType1;
        dynamic_dict["dynType2"] = dynType2;


        // Access and print values from the dynamic types

        absl::visit([](auto&& value) {
            value.print();
        }, dynType1);

        absl::visit([](auto&& value) {
            value.print();
        }, dynType2);
    }
#endif

}