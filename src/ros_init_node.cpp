//
// Created by waxz on 8/27/22.
//

#include <thread>
#include <ros/callback_queue.h>
#include "ros/ros.h"

#include "std_msgs/Header.h"
#include "std_msgs/String.h"

#include "xmlrpcpp/XmlRpc.h"

#include "boost/container/deque.hpp"


#include "common/suspend.h"
#include "common/string_logger.h"

struct RosNode{
    std::map<std::string, std::tuple<std::shared_ptr<ros::CallbackQueue>,
            bool,
            std::shared_ptr<ros::NodeHandle>,
            std::shared_ptr<ros::Subscriber>

            >> callback_queues;

    template<typename T>
    void addSub(const std::string& topic, T *data){
        auto it = callback_queues.find(topic);
        if(it == callback_queues.end()){

            std::cout << "creat sub " << topic << std::endl;

            std::shared_ptr<ros::CallbackQueue> q = std::make_shared<ros::CallbackQueue>();

            std::shared_ptr<ros::NodeHandle> n = std::make_shared<ros::NodeHandle>();
            std::shared_ptr<ros::Subscriber> sub = std::make_shared<ros::Subscriber>();


            std::tie(it,std::ignore )= callback_queues.emplace(topic,std::make_tuple(q, false, n, sub));


            bool& rt = std::get<1>(it->second);


            n->setCallbackQueue(q.get());
            ros::SubscribeOptions ops;
            // pointer should be copied by value
            // reference should be copied by reference
            auto cb = [data,&rt] (typename T::ConstPtr msg) mutable {
                *data = *msg;
                rt = true;
            };
//            *sub = n->subscribe<T>(topic, 1, cb);

            ops.template init<std_msgs::Header>(topic, 10, cb);
            ops.allow_concurrent_callbacks = true;
            *sub = n->subscribe(ops);


        }else{


        }

    }

    bool getDta(const std::string& topic,double t = 0.05){
        auto it = callback_queues.find(topic);
        if(it == callback_queues.end()){
            return false;
        }else{
            std::shared_ptr<ros::CallbackQueue> q = std::get<0>(it->second);
            auto& rt = std::get<1>(it->second);
            rt = false;
            q->callAvailable(ros::WallDuration(t));
            return rt;
        }
        return false;
    }
};


int main(int argc, char** argv) {

    ros::init(argc, argv, "test");

    std::thread t1([]{
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
            s.sleep(2000);
        }
    });



    std_msgs::Header msg;
    RosNode node;
    std::cout << "&msg_2" << &msg << std::endl;

//    node.addSub("chatter",msg);
    node.addSub("chatter",&msg);
    common::Suspend s;

    while (ros::ok()){

        bool rt = node.getDta("chatter");

        if(rt){
            std::cout << "=== node get data [" << msg.frame_id << "]" << std::endl;
        }
        s.sleep(1000);

    }

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
}
