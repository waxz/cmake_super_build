//
// Created by waxz on 8/27/22.
//

#include <thread>
#include <string>
#include <sstream>
#include <iostream>
#include <algorithm>

#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>


#include "xmlrpcpp/XmlRpc.h"




#include <mosquittopp.h>



#include "common/smart_pointer.h"

#include "boost/container/deque.hpp"


#include "common/suspend.h"
#include "common/string_logger.h"
#include "common/signals.h"
#include "common/string_func.h"
#include "common/clock_time.h"
#include "transform/transform.h"
#include "transform/eigen_transform.h"
#include "common/functions.h"

#include "message/impl/ros/RosMessageManager.h"
#include "message/impl/mqtt/MqttMessageManager.h"

#include "json.hpp"



#include "message/common_message.h"

namespace common_message{

    // ros


}


struct ControlBase{

    /*
     1. robot base type

     single diff
     double diff

     single steering wheel
 *** double steering wheel
     four steering wheel


     2. command type
     cmd_vel: vel_x, rotate_z, vel_y




     */

    int cmd_vel(float vel, float rot){

        return 0;
    }
    int cmd_vel(float vel, float rot, float angle){

        return 0;
    }

};

/*

 Message Manager
 recv/send message
 read/write parameter

 */
#include "message/MessageManager.h"


#if 0

struct MqttMessageManager: message::MessageManager {

    std::shared_ptr<message::MqttClient> client;

    struct SUB_CONTAINER{

        common::wild_ptr buffer;
        std::function<int(const char*, const char*)> on_message;
        std::function<int(int, double, const std::function<void(void*)>)> func;
        std::mutex buffer_mtx;
    };

    struct PUB_CONTAINER{
        std::string topic;
        int qos;
        std::function<int(void*, int, double)> func;
    };


    std::map<std::string, common::wild_ptr> channel_config;

    std::string server_ip;
    int server_port = 0;
    int keep_alive = 0;

    long open(char *arg, int argc, char **argv) override{
        client = std::make_shared<message::MqttClient>(arg, true);
        mosqpp::lib_init();

        int type = 0;
        auto my_handler = common::fnptr<int(int,char*)>([&](int n, char* ch){

            std::cout << "get n = " <<  n << ", ch = " << ch << std::endl;

            if(n==0 && std::strcmp(ch , "server") == 0 ){
                type = 1;
            }
            if(n==0 && std::strcmp(ch , "port") == 0 ){
                type = 2;
            }
            if(n==0 && std::strcmp(ch , "keep_alive") == 0 ){
                type = 3;
            }

            if(n == 1 && type == 1){
                server_ip.assign(ch);
                type = 0;
            }

            if(n == 1 && type == 2){
                if(str2int(&server_port, ch, 10) == STR2INT_SUCCESS){
                    return 0;
                }else{
                    return -1;
                }
                type = 0;
            }
            if(n == 1 && type == 3){
                if(str2int(&keep_alive, ch, 10) == STR2INT_SUCCESS){
                    return 0;
                }else{
                    return -1;
                }
                type = 0;
            }


            return 0;
        });
        for(int i = 0 ; i < argc;i++){
            type = 0;

            split_str(argv[i],":", my_handler);

        }


        std::cout << "get mqtt server " << server_ip << ", " << server_port << ", " << keep_alive << std::endl;
#if 1
        client->message_callback_func = [&](const char* topic, const char* playload){

//            std::cout  << "message_callback_func : " << topic << ", " << playload << std::endl;

            char key_buffer[100];
            sprintf(key_buffer,"%s:%s","SUB",topic);
//            std::cout <<__FILE__ << ":" << __LINE__ << key_buffer<<std::endl;
            auto it = channel_config.find(key_buffer);
            if(it == channel_config.end()){
                std::cout   << key_buffer << " is not exist" << std::endl;
                return 0;
            }

            it->second.ref<SUB_CONTAINER>().on_message(topic, playload);
            return 0;

            return 1;
        };
#endif


        dynamic_assert(!server_ip.empty());
        dynamic_assert(server_port >0);
        dynamic_assert(keep_alive >0);
        return 0;

    }



    template<typename T>
    long add_channel( char *arg){
        char * channel_str[] = {"SUB","PUB"};

        struct ChannelInfo{
            int type = 0;// 1: SUB, 2: PUB, 3: TF Listen , 4 TF Broadcast, 5 Param Reader, 6 Param Write

            std::string topic;
            int qos = 1;
        };

        ChannelInfo channelInfo ;
        auto my_handler = common::fnptr<int(int,char*)>([&channel_str,&channelInfo](int n, char* ch){

            std::cout << "get n = " <<  n << ", ch = " << ch << std::endl;

            if(n == 0  ){
                if (std::strcmp(ch,"SUB")== 0){
                    channelInfo.type = 0;
                }else if(std::strcmp(ch,"PUB")== 0){
                    channelInfo.type=1;
                }
                else{
                    return -1;
                }

                return 0;
            }


            if(n == 1){
                channelInfo.topic.assign(ch);
                return 0;
            }

            if(n == 2){

                if(str2int(&channelInfo.qos, ch, 10) == STR2INT_SUCCESS){
                    return 0;
                }else{
                    return -1;
                }
                return 0;
            }
            return 0;
        });

        split_str(arg,":", my_handler);

        if(channelInfo.type == 0){
            char key_buffer[100];
            sprintf(key_buffer,"%s:%s",channel_str[channelInfo.type],channelInfo.topic.c_str());
//            std::cout <<__FILE__ << ":" << __LINE__ << key_buffer<<std::endl;
            auto it = channel_config.find(key_buffer);
            if(it != channel_config.end()){
                std::cout   << key_buffer << " is already exist" << std::endl;

                return 0;

            }

            std::cout << "creat sub " << key_buffer << std::endl;

            common::wild_ptr  channel_ptr;

            channel_ptr.set<SUB_CONTAINER>();

            std::tie(it,std::ignore )= channel_config.emplace(key_buffer,channel_ptr);

            SUB_CONTAINER& sub_container = it->second.ref<SUB_CONTAINER>();
            common::wild_ptr& buffer = sub_container.buffer;


            buffer.set(std::vector<T>{});
            {

                auto& buffer_ref =  buffer.ref<std::vector<T>>();

            }

            sub_container.on_message = [&](const char* topic, const char* playload){



                auto& buffer_ref =  sub_container.buffer.ref<std::vector<T>>();

                T data = common_message::to_common<T>(playload);

                {
                    std::lock_guard<std::mutex> locker(sub_container.buffer_mtx);
                    buffer_ref.emplace_back(data);

                }
                return 0;
            };

            sub_container.func = [&](int max_num, double s, const std::function<void(void*)>& func){

//                client->loop_misc();

                {
                    std::lock_guard<std::mutex> locker(sub_container.buffer_mtx);
                    auto& buffer_ref =  buffer.ref<std::vector<T>>();
                    int recv_len = buffer_ref.size();

                    int start_index = std::max(  static_cast<int>(0),recv_len- max_num);
                    for(size_t i = start_index; i < recv_len;i++){
                        func(&buffer_ref[i]);
                    }
                    buffer_ref.clear();
                }


                return 0;
            };

            client->addSubOpt(channelInfo.topic.c_str(), channelInfo.qos);


        }else if(channelInfo.type == 1){

            char key_buffer[100];
            sprintf(key_buffer,"%s:%s",channel_str[channelInfo.type],channelInfo.topic.c_str());
            auto it = channel_config.find(key_buffer);
            if(it != channel_config.end()){
                std::cout   << key_buffer << " is already exist" << std::endl;

                return 0;

            }

            std::cout << "creat pub " << key_buffer << std::endl;
            common::wild_ptr  channel_ptr;

            channel_ptr.set<PUB_CONTAINER>();
            std::tie(it,std::ignore )= channel_config.emplace(key_buffer,channel_ptr);

            PUB_CONTAINER& pub_container = it->second.ref<PUB_CONTAINER>();

            pub_container.qos = channelInfo.qos;
            pub_container.topic.assign(channelInfo.topic);

            pub_container.func = [&](void* data, int max_size, double s){
                T* data_ptr = static_cast<T*>(data);
                std::string msg;

                for(int i = 0 ; i < max_size; i++){

                    msg = common_message::from_common<T>((*(data_ptr + i)));

                    client->publish(nullptr,pub_container.topic.c_str(),msg.size(),msg.c_str(),pub_container.qos );

                }

                return 0;
            };

        }



        return 0;
    }
    long start(char *arg) override{
        client->connect_async(server_ip.c_str(),server_port,keep_alive);
        client->loop_start();
        return 0;

    }


    long recv_message(char *channel, size_t max_num, double timeout, const std::function<void (void *)> &callback) override{
        client->loop();

        char * channel_str[] = {"SUB","PUB","TFL","TFB","PAR","PAW"};
        char key_buffer[100];

        sprintf(key_buffer,"%s:%s",channel_str[0],channel);

        auto it = channel_config.find(key_buffer);
        if(it == channel_config.end()){
            std::cout   << key_buffer << " is not exist" << std::endl;
            return 0;
        }
        SUB_CONTAINER& sub_container = it->second.ref<SUB_CONTAINER>();

        sub_container.func(max_num, timeout,callback);


        return 0;
    }

    long send_message(char *channel, void *data, size_t max_num, double timeout) override{

        char * channel_str[] = {"SUB","PUB","TFL","TFB","PAR","PAW"};
        char key_buffer[100];

        sprintf(key_buffer,"%s:%s",channel_str[1],channel);


        auto it = channel_config.find(key_buffer);
        if(it == channel_config.end()){
            std::cout   << key_buffer << " is not exist" << std::endl;
            return 0;
        }

        PUB_CONTAINER& pub_container = it->second.ref<PUB_CONTAINER>();
        pub_container.func(data, max_num, timeout);
//        client->publish(nullptr,"hello2",msg.size(),msg.c_str(),mqtt_topic_qos );


//        client->loop_misc();
        client->loop();

        return 0;
    }
    ~MqttMessageManager(){

    }

};

struct RosMessageManager:message::MessageManager{
    struct NODE_CONTAINER {ros::NodeHandle nh;ros::NodeHandle nh_private;};
    struct SUB_CONTAINER{
        std::string signature;
        std::shared_ptr<ros::CallbackQueue> callback_queue;
        bool rt;
        std::shared_ptr<ros::NodeHandle> nh;
        std::shared_ptr<ros::Subscriber> sub;
        common::wild_ptr buffer;
        std::function<int(int, double, const std::function<void(void*)>)> func;
    };
    struct PUB_CONTAINER{
        std::string signature;
        std::shared_ptr<ros::NodeHandle> nh;
        std::shared_ptr<ros::Publisher> pub;
        common::wild_ptr buffer;
        std::function<int(void*, int, double)> func;
    };
    struct TFL_CONTAINER{
        tf::TransformListener tfl;
        tf::StampedTransform transform;
    };
    struct TFB_CONTAINER{
        tf::TransformBroadcaster tfb;
        tf::StampedTransform transform;

    };
    struct PAR_CONTAINER{};
    struct PAW_CONTAINER{};


    std::map<std::string, common::wild_ptr> channel_config;

    common::wild_ptr node_handler;
    long open(char*arg, int argc, char** argv) override{
        ros::init(argc, argv,arg);
        node_handler.set<NODE_CONTAINER>({ros::NodeHandle(),ros::NodeHandle("~")});
        return 1;
    }

    long start(char *arg) override{

        return 0;

    }

    template<typename T>
    long add_channel( char *arg){


        char * channel_str[] = {"SUB","PUB","TFL","TFB","PAR","PAW"};

        struct ChannelInfo{
            int type = 0;// 1: SUB, 2: PUB, 3: TF Listen , 4 TF Broadcast, 5 Param Reader, 6 Param Write
            std::string topic;
            int queue_size = 1;
        };
        ChannelInfo channelInfo;
        auto my_handler = common::fnptr<int(int,char*)>([&channel_str,&channelInfo](int n, char* ch){

            std::cout << "get n = " <<  n << ", ch = " << ch << std::endl;

            if(n == 0  ){
                if (std::strcmp(ch,"SUB")== 0){
                    channelInfo.type = 0;
                }else if(std::strcmp(ch,"PUB")== 0){
                    channelInfo.type=1;
                }
                else if(std::strcmp(ch,"TFL")== 0){
                    channelInfo.type=2;
                }

                else if(std::strcmp(ch,"TFB")== 0){
                    channelInfo.type =3;
                }
                else{
                    return -1;
                }

                return 0;
            }


            if(n == 1){
                channelInfo.topic.assign(ch);
                return 0;
            }

            if(n == 2){
                if(str2int(&channelInfo.queue_size, ch, 10) == STR2INT_SUCCESS){
                    return 0;
                }else{
                    return -1;
                }
                return 0;
            }
            return 0;
        });

        std::cout <<__FILE__ << ":" << __LINE__ << " crash"<<std::endl;


        split_str(arg,":", my_handler);

        if(channelInfo.type == 0){
            char key_buffer[100];
            sprintf(key_buffer,"%s:%s",channel_str[channelInfo.type],channelInfo.topic.c_str());
            std::cout <<__FILE__ << ":" << __LINE__ << key_buffer<<std::endl;
            auto it = channel_config.find(key_buffer);
            if(it != channel_config.end()){
                std::cout   << key_buffer << " is already exist" << std::endl;

                return 0;

            }

            std::cout << "creat sub " << key_buffer << std::endl;

            std::shared_ptr<ros::CallbackQueue> q = std::make_shared<ros::CallbackQueue>();

            std::shared_ptr<ros::NodeHandle> n = std::make_shared<ros::NodeHandle>();
            std::shared_ptr<ros::Subscriber> sub = std::make_shared<ros::Subscriber>();


            using ROS_MSG_TYPE = decltype(common_message::from_common(std::declval<const T&>()));
            common::wild_ptr  channel_ptr;

            channel_ptr.set<SUB_CONTAINER>({common::TypeName<T>().Get(), q, false, n, sub, common::wild_ptr{}, {}});
            std::tie(it,std::ignore )= channel_config.emplace(key_buffer,channel_ptr);


            n->setCallbackQueue(q.get());
            ros::SubscribeOptions ops;
            SUB_CONTAINER& sub_container = it->second.ref<SUB_CONTAINER>();
            bool & rt = sub_container.rt;
            common::wild_ptr& buffer = sub_container.buffer;
            buffer.set(std::vector<T>{});
            {

                auto& buffer_ref =  buffer.ref<std::vector<T>>();

            }

            sub_container.func = [q,&buffer](int max_num, double s, const std::function<void(void*)>& func){

                auto& buffer_ref =  buffer.ref<std::vector<T>>();
                buffer_ref.clear();

                q->callAvailable(ros::WallDuration(s));

                int recv_len = buffer_ref.size();

                int start_index = std::max(  static_cast<int>(0),recv_len- max_num);
                for(size_t i = start_index; i < recv_len;i++){
                    func(&buffer_ref[i]);
                }

                return 1;
            };

            // pointer should be copied by value
            // reference should be copied by reference
            auto cb = [&rt,&buffer] (typename ROS_MSG_TYPE::ConstPtr msg) mutable {
                rt = true;
                buffer.ref<std::vector<T>>().push_back(common_message::to_common(*msg));
            };

            ops.template init<ROS_MSG_TYPE>(channelInfo.topic, channelInfo.queue_size, cb);
            ops.allow_concurrent_callbacks = true;
            *sub = n->subscribe(ops);
        }else if(channelInfo.type == 1){
            char key_buffer[100];
            sprintf(key_buffer,"%s:%s",channel_str[channelInfo.type],channelInfo.topic.c_str());

            std::cout << "creat pub " << key_buffer << std::endl;
            auto it = channel_config.find(key_buffer);
            if(it != channel_config.end()){
                std::cout   << key_buffer << " is already exist" << std::endl;

                return 0;

            }

            std::shared_ptr<ros::NodeHandle> n = std::make_shared<ros::NodeHandle>();
            std::shared_ptr<ros::Publisher> pub = std::make_shared<ros::Publisher>();

            using ROS_MSG_TYPE = decltype(common_message::from_common(std::declval<const T&>()));
            common::wild_ptr  channel_ptr;
            std::function<int(void*, int, double)> func;

            channel_ptr.set<PUB_CONTAINER>({common::TypeName<T>().Get(),n, pub, common::wild_ptr{},func});
            std::tie(it,std::ignore )= channel_config.emplace(key_buffer,channel_ptr);

            PUB_CONTAINER& pub_container = it->second.ref<PUB_CONTAINER>();

            common::wild_ptr& buffer = pub_container.buffer;
            buffer.set(std::vector<T>{});


            *pub = n->advertise<geometry_msgs::Twist>(channelInfo.topic, channelInfo.queue_size);
            pub_container.func = [pub](void* data, int max_size, double s){
                T* data_ptr = static_cast<T*>(data);
                for(int i = 0 ; i < max_size; i++){
                    pub->publish(common_message::from_common(*(data_ptr + i)));
                }
                return 1;
            };
        }
        return 0;

        /*
         TWO CHANNEL CONFIG: SUB_CONFIG, PUB_CONFIG
         KEY: RECV_BUFFER, BUFFER_PROCESSOR
         scan: buffer, pull
         KEY:SEND_BUFFER, BUFFER_PROCESSOR
         cmd_vel: buffer, push



         in client:
         call recv -> pull data from ros -> get message buffer -> callback process each message

         call send -> push data to message buffer -> ros publish each message

         type cast should implement in BUFFER_PROCESSOR

         */

        return 1;
    }

    long send_message(char* channel, void* data, size_t max_num, double timeout) override {
        char * channel_str[] = {"SUB","PUB","TFL","TFB","PAR","PAW"};
        char key_buffer[100];

        sprintf(key_buffer,"%s:%s",channel_str[1],channel);


        auto it = channel_config.find(key_buffer);
        if(it == channel_config.end()){
            std::cout   << key_buffer << " is not exist" << std::endl;
            return 0;
        }
        PUB_CONTAINER& pub_container = it->second.ref<PUB_CONTAINER>();
        pub_container.func(data, max_num, timeout);
        return 0;
    }

    long recv_message(char* channel, size_t max_num, double timeout, const std::function<void(void*)>& callback) override {
        char * channel_str[] = {"SUB","PUB","TFL","TFB","PAR","PAW"};
        char key_buffer[100];

        sprintf(key_buffer,"%s:%s",channel_str[0],channel);

        auto it = channel_config.find(key_buffer);
        if(it == channel_config.end()){
            std::cout   << key_buffer << " is not exist" << std::endl;
            return 0;
        }
        SUB_CONTAINER& sub_container = it->second.ref<SUB_CONTAINER>();
        sub_container.func(max_num, timeout,callback);


        return 0;
    }


    // get tf
    long recv_tf(common_message::TransformStamped & data, float timeout ){

        char * channel_str[] = {"SUB","PUB","TFL","TFB","PAR","PAW"};

        char key_buffer[100];
        sprintf(key_buffer,"%s",channel_str[2]);
        auto it = channel_config.find(key_buffer);
        if(it != channel_config.end()){
            std::cout   << key_buffer << " is already exist" << std::endl;
        }else{

            common::wild_ptr  channel_ptr;
            channel_ptr.set<TFL_CONTAINER>();

            std::tie(it,std::ignore )= channel_config.emplace(key_buffer,channel_ptr);

        }
        ros::Time t(0);

        try{

            auto& tfl = it->second.ref<TFL_CONTAINER>();
            if(timeout > 0.0){

                common::ToRos(data.time,t);
                tfl.tfl.waitForTransform(data.base_frame, data.target_frame,t, ros::Duration(timeout));

            }
            tfl.tfl.lookupTransform(data.base_frame, data.target_frame,t, tfl.transform);

            common_message::to_common(tfl.transform,data);
            return 0;

        }catch (tf::TransformException &ex) {
            ROS_ERROR("%s", ex.what());
            return -1;
        }
    }

    long send_tf(common_message::TransformStamped & data){
        char * channel_str[] = {"SUB","PUB","TFL","TFB","PAR","PAW"};

        char key_buffer[100];
        sprintf(key_buffer,"%s",channel_str[3]);

        auto it = channel_config.find(key_buffer);
        if(it != channel_config.end()){
            std::cout   << key_buffer << " is already exist" << std::endl;
        }else{

            common::wild_ptr  channel_ptr;
            channel_ptr.set<TFB_CONTAINER>();

            std::tie(it,std::ignore )= channel_config.emplace(key_buffer,channel_ptr);

        }
        auto& tfb = it->second.ref<TFB_CONTAINER>();
        common_message::from_common(data, tfb.transform);
        tfb.tfb.sendTransform( tfb.transform);

        return 0;
    }

    ~RosMessageManager() {

    }
};


struct RosNode{
    std::map<std::string, std::tuple<std::shared_ptr<ros::CallbackQueue>,
            bool,
            std::shared_ptr<ros::NodeHandle>,
            std::shared_ptr<ros::Subscriber>,
            common::wild_ptr
            >> callback_queues;

    template<typename T>
    void addSub(const std::string& topic){
        auto it = callback_queues.find(topic);
        if(it == callback_queues.end()){

            std::cout << "creat sub " << topic << std::endl;

            std::shared_ptr<ros::CallbackQueue> q = std::make_shared<ros::CallbackQueue>();

            std::shared_ptr<ros::NodeHandle> n = std::make_shared<ros::NodeHandle>();
            std::shared_ptr<ros::Subscriber> sub = std::make_shared<ros::Subscriber>();


            using ROS_MSG_TYPE = decltype(common_message::from_common(std::declval<const T&>()));

            std::tie(it,std::ignore )= callback_queues.emplace(topic,std::make_tuple(q, false, n, sub,common::wild_ptr{} ));
            common::wild_ptr& wildPtr = std::get<4>(it->second);

            wildPtr.set(std::vector<T>{});

            bool& rt = std::get<1>(it->second);


            n->setCallbackQueue(q.get());
            ros::SubscribeOptions ops;
            // pointer should be copied by value
            // reference should be copied by reference
            auto cb = [&rt,&wildPtr] (typename ROS_MSG_TYPE::ConstPtr msg) mutable {
                rt = true;
                wildPtr.ref<std::vector<T>>().push_back(common_message::to_common(*msg));
            };
//            *sub = n->subscribe<T>(topic, 1, cb);

            ops.template init<ROS_MSG_TYPE>(topic, 10, cb);
            ops.allow_concurrent_callbacks = true;
            *sub = n->subscribe(ops);


        }else{


        }

    }

    template<typename T>
    bool recv_message(const std::string& topic,double t, int max_num,  const std::function<void(const T&)>& func){
        auto it = callback_queues.find(topic);
        if(it == callback_queues.end()){
            std::cout << "no sub" << std::endl;
            return false;
        }else{
            std::shared_ptr<ros::CallbackQueue> q = std::get<0>(it->second);
            auto& rt = std::get<1>(it->second);
            common::wild_ptr& wildPtr = std::get<4>(it->second);
            wildPtr.ref<std::vector<T>>().clear();
            rt = false;
            q->callAvailable(ros::WallDuration(t));
            auto & buffer = wildPtr.ref<std::vector<T>>();
            int recv_len = buffer.size();

            int start_index = std::max(  static_cast<int>(0),recv_len- max_num);
            for(size_t i = start_index; i < recv_len;i++){
                func(buffer[i]);
            }


            return rt;
        }
        return false;
    }
};
#endif

int main(int argc, char** argv) {



    std::atomic_bool program_run(true);
    auto my_handler = common::fnptr<void(int)>([&](int sig){ std::cout << "get sig " << sig;program_run = false;
        if(ros::ok()){
            ros::shutdown();
        }
    });
    common::set_signal_handler(my_handler);

    common::Suspend s;

    {


#if 0

#endif

        if(1){
            char * mqtt_server = "127.0.0.1:1883";

            std::vector<std::string> mqtt_config{
                    "server:127.0.0.1:1883",
                    "server:127.0.0.1:1883"
            };
            char* mqtt_config_str[] = {"server:127.0.0.1", "port:1883","keep_alive:5","clean_session:1"};

            message::MqttMessageManager mqttMessageManager;
            char* mqtt_client_id = "A";
            mqttMessageManager.open(mqtt_client_id,4, mqtt_config_str);

            mqttMessageManager.add_channel<std::string>("SUB:hello:0");
            mqttMessageManager.add_channel<std::string>("PUB:hello:0");

            mqttMessageManager.add_channel<common_message::Twist>("SUB:chatter:0");
            mqttMessageManager.add_channel<common_message::Twist>("PUB:chatter:0");

            std::vector<common_message::Twist> twist_msgs(5);


            mqttMessageManager.start("");
            int i = 0;
            auto hello_sub = [](void* data){
                std::string* data_ptr = static_cast<std::string*>(data);
                std::cout << "test123 recv msg: " << *data_ptr << std::endl;

            };

            auto sub_cb =[](void* data){
                common_message::Twist* data_ptr = static_cast<common_message::Twist*>(data);
                std::cout << "test456 recv msg: " << data_ptr->linear.x << std::endl;

            };

            std::string msg = "hello from cpp";
            std::vector<std::string> msgs{
                "hello cpp",
                "hello mqtt"
            };
            while (program_run){
                for(auto& m: twist_msgs){
                    m.linear.x += 0.01;
                }


                mqttMessageManager.send_message("hello", msgs.data(),2,0.1);

                mqttMessageManager.recv_message("hello",2,0.1,hello_sub);

                mqttMessageManager.send_message("chatter",twist_msgs.data(), 5, 0.1 );

                mqttMessageManager.recv_message("chatter",10,0.01, sub_cb);

                s.sleep(1);

//                if(i++ > 20){ break;}
            }
            mqttMessageManager.stop();

            std::cout << "exit" << std::endl;
            return 0;
        }


        message::RosMessageManager rosMessageManager;

        rosMessageManager.open("test", argc, argv);

        rosMessageManager.add_channel<common_message::Twist>("SUB:chatter:10");
        rosMessageManager.add_channel<common_message::Twist>("PUB:chatter:10");

        rosMessageManager.add_channel<common_message::Odometry>("PUB:chatter_odom:10");
        rosMessageManager.add_channel<common_message::Odometry>("SUB:chatter_odom:10");


        std::vector<common_message::Twist> msgs(5);
        common_message::Odometry odom;
        odom.header.frame_id.assign("odom");
        odom.child_frame_id.assign("base_link");

        std::vector<common_message::Odometry> odom_msgs(5,odom);

        rosMessageManager.send_message("chatter",msgs.data(), 5, 0.1 );
        rosMessageManager.send_message("chatter_odom",odom_msgs.data(), 5, 0.1 );

        auto twist_sub_cb =[](void* data){
            common_message::Twist* data_ptr = static_cast<common_message::Twist*>(data);
            std::cout << "twist_sub_cb recv msg: " << data_ptr->linear.x << std::endl;

        };

        // todo: callback may fail if topic and datetye not matched
        auto odom_sub_cb =[](void* data){
            common_message::Odometry* data_ptr = static_cast<common_message::Odometry*>(data);
            std::cout << "odom_sub_cb recv msg: " << data_ptr->twist.twist.linear.x << std::endl;

        };
        common_message::TransformStamped tf1, tf2;
        tf1.base_frame.assign("map");
        tf1.target_frame.assign("odom");
        tf2.base_frame.assign("map");
        tf2.target_frame.assign("base");
        int i = 0;
        while (ros::ok()&&program_run){

            for(auto& m: msgs){
                m.linear.x += 0.01;
            }
            for(auto& m: odom_msgs){
                m.pose.pose.position.x += 0.01;
            }


            rosMessageManager.send_message("chatter",msgs.data(), 5, 0.1 );

            rosMessageManager.recv_message("chatter",10,0.01, twist_sub_cb);
            rosMessageManager.send_message("chatter_odom",odom_msgs.data(), 5, 0.1 );

            rosMessageManager.recv_message("chatter_odom",10,0.01, odom_sub_cb);

            tf2.time = common::FromUnixNow();

            tf1.time = common::FromUnixNow();

            tf1.transform = transform::createSe3<float>(0.1*i,0.1,0.0,0.0,0.0,0.0);
            tf2.transform = transform::createSe3<float>(0.1*i,0.1,0.0,0.0,0.0,0.0);

            rosMessageManager.send_tf(tf1);
            rosMessageManager.recv_tf(tf2, 0.01);
            std::cout << "tf1:\n" <<tf1.transform.matrix() << std::endl;

            std::cout << "tf2:\n"  << tf2.transform.matrix() << std::endl;

            s.sleep(10);
            i++;
        }
    }
    std::cout << "exit" << std::endl;

    return 0;
    ros::init(argc, argv, "test");

    std::cout << " create node 1" << std::endl;

    ros::NodeHandle n;
    std::cout << " create node 1 done" << std::endl;

    std::thread t1([&n,&program_run]{

        const char* topic = "chatter";

        ros::Publisher chatter_pub =
                n.advertise<geometry_msgs::Twist>(topic, 1);


        common::Suspend s;
        geometry_msgs::Twist msg;

        char buffer[200];
        std::cout << " start thread" << std::endl;

        while (ros::ok()&&program_run){

            std::stringstream ss;
//            ROS_INFO("send %s : %s", topic, msg.frame_id.c_str());
            for(int i = 0 ; i < 20;i++){
                msg.linear.x += 0.01;
                chatter_pub.publish(msg);

            }
            std::cout << "publish msg: " << msg.linear.x << std::endl;

            s.sleep(500);
        }
    });


#if 0
    std::cout << " create node 2" << std::endl;

    RosNode node;
    std::cout << " create node 2 done" << std::endl;

//    node.addSub("chatter",msg);
    node.addSub<common_message::Twist>("chatter");


    auto cmd_vel_callback = [&](const common_message::Twist& msg){
        std::cout << "recv msg: " << msg.linear.x << std::endl;
    };
    while (ros::ok() && program_run){

        int max_num = 10;
        bool rt = node.recv_message<common_message::Twist>("chatter", 0.1,1, cmd_vel_callback);

        if(rt){
            std::cout << "finish recv_message once:\n ";

        }
        s.sleep(1);

    }


    t1.join();

    return 0;

    std::thread t2([]{
        ros::NodeHandle n;

        const char* topic = "chatter";

        ros::Publisher chatter_pub =
                n.advertise<std_msgs::Header>(topic, 1000);


        common::Suspend s;
        std_msgs::Header msg;

        char buffer[200];
        while (ros::ok()){

            std::stringstream ss;
            msg.seq++;
            ss <<"thread : " << std::this_thread::get_id() << ", chatter messages: " << msg.seq;
            msg.frame_id = ss.str();
//            ROS_INFO("send %s : %s", topic, msg.frame_id.c_str());
            chatter_pub.publish(msg);
            s.sleep(0.1);
        }
    });


    std::thread t3([]{
        ros::NodeHandle n;
        const char* topic = "chatter";
        unsigned int flag = 0;
        int miss_num = 0;
        auto cb = [flag,miss_num ](std_msgs::HeaderConstPtr msg) mutable {
            miss_num +=  (msg->seq > (flag + 1));
            flag = std::max(msg->seq, flag );
            std::cout << " t3: seq " <<  msg->seq << ", flag " << flag << ", miss " << miss_num << std::endl;
        };
        ros::Subscriber sub = n.subscribe<std_msgs::Header>(topic, 10, cb);
        ros::MultiThreadedSpinner spinner(4);
        spinner.spin();
    });

    std::thread t4([]{
        ros::NodeHandle n;
        const char* topic = "chatter";
        unsigned int flag = 0;
        int miss_num = 0;
        auto cb = [flag,miss_num ](std_msgs::HeaderConstPtr msg) mutable {
            miss_num +=  (msg->seq > (flag + 1));
            flag = std::max(msg->seq, flag );
            std::cout << " t4: seq " <<  msg->seq << ", flag " << flag << ", miss " << miss_num << std::endl;
        };

        ros::SubscribeOptions ops;
        ops.template init<std_msgs::Header>(topic, 10, cb);
        ops.allow_concurrent_callbacks = true;
        ros::Subscriber sub1 = n.subscribe(ops);
        ros::MultiThreadedSpinner spinner(4);
        spinner.spin();

    });

    std::thread t5([]{
        ros::NodeHandle n;
        const char* topic = "chatter";
        unsigned int flag = 0;
        int miss_num = 0;
        auto cb = [flag,miss_num ](std_msgs::HeaderConstPtr msg) mutable {
            miss_num +=  (msg->seq > (flag + 1));
            flag = std::max(msg->seq, flag );
            std::cout << " t5: seq " <<  msg->seq << ", flag " << flag << ", miss " << miss_num << std::endl;
        };
        ros::Subscriber sub = n.subscribe<std_msgs::Header>(topic, 10, cb);

        ros::AsyncSpinner spinner(4); // Use 4 threads
        spinner.start();
        ros::waitForShutdown();


    });


    std::thread t6([]{
        ros::NodeHandle n;
        const char* topic = "chatter";
        unsigned int flag = 0;
        int miss_num = 0;
        auto cb = [flag,miss_num ](std_msgs::HeaderConstPtr msg) mutable {
            miss_num +=  (msg->seq > (flag + 1));
            flag = std::max(msg->seq, flag );
            std::cout << " t6: seq " <<  msg->seq << ", flag " << flag << ", miss " << miss_num << std::endl;
        };

        ros::SubscribeOptions ops;
        ops.template init<std_msgs::Header>(topic, 10, cb);
        ops.allow_concurrent_callbacks = true;
        ros::Subscriber sub1 = n.subscribe(ops);

        ros::AsyncSpinner spinner(4); // Use 4 threads
        spinner.start();
        ros::waitForShutdown();

    });

    std::thread t7([]{

        ros::NodeHandle n;
        const char* topic = "chatter";
        unsigned int flag = 0;
        int miss_num = 0;
        auto cb = [flag,miss_num ](std_msgs::HeaderConstPtr msg) mutable {
            miss_num +=  (msg->seq > (flag + 1));
            flag = std::max(msg->seq, flag );
            std::cout << " t7: seq " <<  msg->seq << ", flag " << flag << ", miss " << miss_num << std::endl;
        };
        ros::CallbackQueue callback_queue_a;
        n.setCallbackQueue(&callback_queue_a);

        ros::SubscribeOptions ops;
        ops.template init<std_msgs::Header>(topic, 10, cb);
        ops.allow_concurrent_callbacks = true;


        ros::Subscriber sub_a = n.subscribe(ops);

        std::thread spinner_thread_a([&callback_queue_a]() {
            ros::MultiThreadedSpinner spinner(4);
            spinner.spin(&callback_queue_a);

        });
        spinner_thread_a.join();

    });




    if(t2.joinable()){
        t2.join();
    }

    if(t3.joinable()){
        t3.join();
    }

    if(t4.joinable()){
        t4.join();
    }


    if(t5.joinable()){
        t5.join();
    }


    if(t6.joinable()){
        t6.join();
    }

    if(t7.joinable()){
        t7.join();
    }
#endif

}
