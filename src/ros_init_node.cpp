//
// Created by waxz on 8/27/22.
//
#include "xmlrpcpp/XmlRpc.h"
#include "ros/ros.h"
#include "boost/container/deque.hpp"
bool  getParam(std::string name, XmlRpc::XmlRpcValue & value){

    name = ros::this_node::getName() + "/" + name;
    if (ros::param::has(name)) {
        std::cerr << "ros get xml value, has name: " << name << std::endl;

    } else {
        std::cerr << "ros get xml value,do not  has name: " << name << std::endl;
        return false;

    }

    ros::param::get(name, value);
    return true;

}


int main(int argc, char** argv) {

    {
        ros::init(argc, argv, "test");


    }

    XmlRpc::XmlRpcValue xmlvalue;
    getParam("f",xmlvalue);

    std::cout << "xmlvalue: " << xmlvalue << std::endl;

    for (int i = 0; i < xmlvalue.size(); i++) {

        if (xmlvalue[i].getType() != XmlRpc::XmlRpcValue::TypeStruct) {

            std::cerr
                    << "ros getParam fail: wrong data type [vector<vector>], required std::vector<std::map<std::string, std::string>>"
                    << std::endl;

        }

        for (auto it = xmlvalue[i].begin(); it != xmlvalue[i].end(); it++) {
              std::cout << "it->first: " << it->first << "\n"
              << static_cast<std::string>(xmlvalue[i][it->first]) << std::endl;
        }


    }


}
