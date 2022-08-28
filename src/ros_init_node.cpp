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

}
