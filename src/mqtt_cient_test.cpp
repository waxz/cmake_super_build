//
// Created by waxz on 9/14/22.
//
#include "message/impl/mqtt/MqttClient.h"


#include "json.hpp"

#include <string>
#include <iostream>
#include <thread>
#include <chrono>


#include "common/signal_handler.h"
#include "common/clock_time.h"
int main(int argc, char** argv){


    common::initSystemSignalHandler();

    std::string mqtt_server_ip = "127.0.0.1";
    int mqtt_server_port = 1883;
    int mqtt_topic_qos =0;
    std::string mqtt_user_name = "user_1";
    std::string mqtt_passward = "user_1";
    std::string mqtt_client_id = "agv-";
    std::string mqtt_will_topic;
    std::string mqtt_will_message;
    int mqtt_keep_alive = 5;

    mosqpp::lib_init();

    message::MqttClient MqttInstance(mqtt_client_id.c_str(),true) ;

//    MqttClient& MqttInstance = MqttClient::Singleton::getInstance<MqttClient>(client_id,true);
    MqttInstance.message_callback_func = [&](const char* topic, const char* playload){

        std::cout  << "message_callback_func : " << topic << ", " << playload << std::endl;
        std::cout  << "message_callback_func sleep 1s "  << std::endl;
        std::cout  << "message_callback_func sleep time " << common::getCurrentDateTime()  << std::endl;

        using namespace  std::chrono_literals;
        std::this_thread::sleep_for(1s);
        return 1;
    };

//    mqtt_will_topic = "online";
//    mqtt_will_message = "die";

    MqttInstance.addSubOpt("hello",0);
    MqttInstance.addSubOpt("hello2",0);

    MqttInstance.listSubOpts();
    MqttInstance.connect_async(mqtt_server_ip.c_str(),mqtt_server_port,mqtt_keep_alive);

    MqttInstance.loop_start();
//    MqttInstance.threaded_set(false);

    std::string   msg;
    int i = 0;
    int rc = 0;
    while (common::SignalOK()){

        if(i < 10){
            rc = MqttInstance.loop();
            MqttInstance.publish(nullptr,"hello",msg.size(),msg.c_str(),mqtt_topic_qos );
            MqttInstance.publish(nullptr,"hello2",msg.size(),msg.c_str(),mqtt_topic_qos );

        }


        msg = "hello data " + std::to_string(i);



        using namespace  std::chrono_literals;
        std::this_thread::sleep_for(10ms);
        i++;


    }

}