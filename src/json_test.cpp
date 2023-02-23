//
// Created by waxz on 23-2-22.
//
#include <iostream>
#include <iomanip>
#include "json.hpp"

#include "transform/transform.h"

using json = nlohmann::json;

struct Point { float x, y, z; };

void to_json(json& j, const Point& p)
{
    j = {{"x", p.x}, {"y", p.y}, {"z", p.z}};
}

void from_json(const json& j, Point& p) {
    j.at("x").get_to(p.x);
    j.at("y").get_to(p.y);
    j.at("z").get_to(p.z);
}


namespace transform{
    void to_json(json& j, const transform::Transform2d& p)
    {
        j = {{"matrix",p.matrix}};
    }

    void from_json(const json& j, transform::Transform2d& p) {
        j.at("matrix").get_to(p.matrix);
    }
}

//https://github.com/nlohmann/json/blob/develop/docs/mkdocs/docs/features/arbitrary_types.md
int main(int argc, const char **argv) {
    {
        Point p1 = {1,2,3};
        Point p2 = {3,4,5};

        std::vector<Point> v = {p1, p2};
        json j = v;

        std::cout << std::setw(2) << j << std::endl;
    }

    {
        transform::Transform2d p1 = {0.4,0.5,0.3};
        transform::Transform2d p2 =  {1.34,6.2,0.0};


        std::vector<transform::Transform2d> v = {p1, p2};
        json j = v;

        std::cout << std::setw(2) << j << std::endl;

        std::vector<transform::Transform2d> v2 = j;

        std::cout << "check v2\n";
        for(auto & p: v2){
            std::cout << p;
        }

    }

}