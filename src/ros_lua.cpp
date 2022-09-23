//
// Created by waxz on 9/3/22.
//
//
// Created by waxz on 8/27/22.
//
#include "xmlrpcpp/XmlRpc.h"
#include "ros/ros.h"
#include "boost/container/deque.hpp"

#define SOL_CHECK_ARGUMENTS 1

#include <sol.hpp>

#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"

#include "json.hpp"

bool getParam(std::string name, XmlRpc::XmlRpcValue &value) {

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

void cb2(const sensor_msgs::LaserScanConstPtr &msg) {


};

//FROM
//https://stackoverflow.com/a/34571089/5155484

typedef unsigned char uchar;
static const std::string b = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";//=
static std::string base64_encode(const std::string &in) {
    std::string out;

    int val=0, valb=-6;
    for (uchar c : in) {
        val = (val<<8) + c;
        valb += 8;
        while (valb>=0) {
            out.push_back(b[(val>>valb)&0x3F]);
            valb-=6;
        }
    }
    if (valb>-6) out.push_back(b[((val<<8)>>(valb+8))&0x3F]);
    while (out.size()%4) out.push_back('=');
    return out;
}


static std::string base64_decode(const std::string &in) {

    std::string out;

    std::vector<int> T(256,-1);
    for (int i=0; i<64; i++) T[b[i]] = i;

    int val=0, valb=-8;
    for (uchar c : in) {
        if (T[c] == -1) break;
        val = (val<<6) + T[c];
        valb += 6;
        if (valb>=0) {
            out.push_back(char((val>>valb)&0xFF));
            valb-=8;
        }
    }
    return out;
}

struct Player {
    int player_id_;

    Player() : player_id_(1) {}

    Player(int player_id) : player_id_(player_id) {}

    void hello() {
        std::cout << "hello player " << player_id_ << std::endl;
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "test");

    // create an array using push_back
    nlohmann::json j;
//    j.push_back("foo");
//    j.push_back(1);
//    j.push_back(true);
//
//// also use emplace_back
//    j.emplace_back(1.78);

    j["code"] = "print(233)";
    std::cout << j.dump(4) << std::endl;


    sol::state lua;
    lua.open_libraries(sol::lib::base);

    sol::function transferred_into;
    lua["f"] = [&lua, &transferred_into](sol::object t, sol::this_state this_L) {
        std::cout << "state of main     : " << (void *) lua.lua_state() << std::endl;
        std::cout << "state of function : " << (void *) this_L.lua_state() << std::endl;
        // pass original lua_State* (or sol::state/sol::state_view)
        // transfers ownership from the state of "t",
        // to the "lua" sol::state
        transferred_into = sol::function(lua, t);
    };


    XmlRpc::XmlRpcValue xmlvalue;
    getParam("f", xmlvalue);

    std::cout << "xmlvalue: " << xmlvalue << std::endl;

    sensor_msgs::LaserScan scan_msg;

    lua.new_usertype<Player>("Player",
                             sol::constructors<Player(), Player(int)>(),
            // bind as variable
                             "player_id", &Player::player_id_,
            // bind as function
                             "hello", sol::as_function(&Player::hello)
    );
    // typical member function that returns a variable

    // make usertype metatable
    // "bark" namespacing in Lua
    // namespacing is just putting things in a table
    sol::table bark = lua.create_named_table("bark");


    bark.new_usertype<std::vector<float>>("vector_float",
                                          sol::constructors<std::vector<float>(int)>(),
                                          "size", &std::vector<float>::size,


                                          sol::meta_function::index, [](std::vector<float> &ns, int i) -> float & {
                return ns[i]; // treat like a container, despite is_container specialization
            },
                                          sol::meta_function::new_index, [](std::vector<float> &ns, int i, float v)   {
                  ns[i] = v; // treat like a container, despite is_container specialization
            },
                                          "iterable", [](std::vector<float> &ns) {
                return sol::as_container(ns); // treat like a container, despite is_container specialization
            }
    );


    bark.new_usertype<nlohmann::json>("json",
                                          sol::constructors<nlohmann::json( )>(),
                                          sol::meta_function::index, [](nlohmann::json &ns, const std::string & i) {
                return ns[i]; // treat like a container, despite is_container specialization
            },
            "clear", &nlohmann::json::clear,

                                          sol::meta_function::new_index,
                                      sol::overload(
                                              [](nlohmann::json &ns, const std::string & i, const std::string & v)   {
                                                  ns[i] = v; // treat like a container, despite is_container specialization
                                              },
                                              [](nlohmann::json &ns, const std::string & i, float v)   {
                                                  ns[i] = v; // treat like a container, despite is_container specialization
                                              },

                                              [](nlohmann::json &ns, const std::string & i, const nlohmann::json & v)   {
                                                  ns[i] = v; // treat like a container, despite is_container specialization
                                              }
                                              )



    );
// Set a global variable called
    // "arr" to be a vector of 5 lements

//    metatable[sol::meta_method::new_index] = [](std::vector<float>& ns) {
//        return sol::as_container(ns); // treat like a container, despite is_container specialization
//    };
//

    lua["LaserScan"] = sensor_msgs::LaserScan();
    Player b;
    lua.set("P1", &b);
    lua["P2"] = Player(2);
    lua.script(R"(
P1:hello();
P2:hello();

	)");


    for (int i = 0; i < xmlvalue.size(); i++) {

        if (xmlvalue[i].getType() != XmlRpc::XmlRpcValue::TypeStruct) {

            std::cerr
                    << "ros getParam fail: wrong data type [vector<vector>], required std::vector<std::map<std::string, std::string>>"
                    << std::endl;

        }

        for (auto it = xmlvalue[i].begin(); it != xmlvalue[i].end(); it++) {
            std::string k = it->first;
            std::string f = static_cast<std::string>(xmlvalue[i][it->first]);

            std::cout << "k : " << k << "\n"
                      << "f : " << f << std::endl;

            lua.script(R"(
print("***** run function from rosparam");

	)");
            lua.script(f);

        }


    }
    ros::NodeHandle nh;

    std::map<std::string, ros::Subscriber> ros_subscribers;

    sensor_msgs::LaserScan pcl;
    auto sub1 = nh.subscribe<sensor_msgs::LaserScan>("scan", 1,
                                                     [&](const sensor_msgs::LaserScanConstPtr &msg) { pcl = *msg; });

    auto cb = [&](const sensor_msgs::LaserScanConstPtr &msg) {
        pcl = *msg;
    };
    auto sub2 = nh.subscribe<sensor_msgs::LaserScan>("scan", 1, cb);
    auto sub3 = nh.subscribe("scan", 1, cb2);

    std::function<void(const sensor_msgs::LaserScanConstPtr &)> cb4 =  [&](const sensor_msgs::LaserScanConstPtr &msg) {
        pcl = *msg;
    };


//    ros_subscribers.emplace_hint("map", );


    for (std::string line; std::getline(std::cin, line);) {
        std::cout << line << std::endl;



        if( ! line.empty() ){
            if (line[0] == 'q'){
                break;
            }
            lua.script(line);

        }


    }
//    ros::spin();


}
