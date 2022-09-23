//
// Created by waxz on 9/4/22.
//
#include <sstream>
#include "ros/ros.h"
#include "std_msgs/Header.h"
#include <thread>
int main(int argc, char **argv) {
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
    ros::Publisher chatter_pub =
            n.advertise<std_msgs::Header>("chatter", 1000);

    ros::Publisher chatter2_pub =
            n.advertise<std_msgs::Header>("chatter2", 1000);

    ros::Publisher chatter3_pub =
            n.advertise<std_msgs::Header>("chatter3", 1000);
    ros::Publisher chatter4_pub =
            n.advertise<std_msgs::Header>("chatter4", 1000);

    std_msgs::Header msg;

    ros::Timer timer = n.createTimer(ros::Duration(0.01),
                                     [&](const ros::TimerEvent&) {
                                         std::stringstream ss;
                                         msg.seq++;
                                         ss <<"thread : " << std::this_thread::get_id() << ", chatter messages: " << msg.seq;
                                         msg.frame_id = ss.str();
                                         ROS_INFO("%s", msg.frame_id.c_str());
                                         chatter_pub.publish(msg);
                                         chatter2_pub.publish(msg);
                                         chatter3_pub.publish(msg);
                                         chatter4_pub.publish(msg);

                                     });

    ros::spin();
    return 0;
}