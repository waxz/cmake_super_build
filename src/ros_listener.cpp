//
// Created by waxz on 9/4/22.
//
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/NavSatFix.h"
#include <thread>
#include <chrono>

//void ChatterCallback(const std_msgs::String::ConstPtr& msg) {
//    ROS_INFO(" I heard: [%s]", msg->data.c_str());
//    using namespace std::chrono_literals;
//
//    std::this_thread::sleep_for(0.02s);
//}
#include <GeographicLib/LambertConformalConic.hpp>
unsigned int seq = 0;
unsigned int seq2 = 0;


void ChatterCallback(const std_msgs::Header::ConstPtr& msg) {

    int miss = msg->seq - seq ;
    seq = msg->seq;

    if (miss > 1){
        ROS_INFO("ChatterCallback : thread %d , Miss : %d",std::this_thread::get_id(), miss);
    }
    ROS_INFO(" ChatterCallback heard: [%d]",msg->seq);

    using namespace std::chrono_literals;

    std::this_thread::sleep_for( 20ms );
}

void ChatterCallback2(const std_msgs::Header::ConstPtr& msg) {

    int miss = msg->seq - seq2 ;
    seq2 = msg->seq;

    if (miss > 1){
        ROS_INFO("ChatterCallback2: thread %d , Miss : %d",std::this_thread::get_id(), miss);
    }
    ROS_INFO(" ChatterCallback2 heard: [%d]",msg->seq);

    using namespace std::chrono_literals;

    std::this_thread::sleep_for( 20ms );
}

#if 0
void  setTransformGps(const sensor_msgs::NavSatFixConstPtr& msg)
{
    double cartesian_x = 0;
    double cartesian_y = 0;
    double cartesian_z = 0;
    if (use_local_cartesian_)
    {
        const double hae_altitude = 0.0;
        gps_local_cartesian_.Reset(msg->latitude, msg->longitude, hae_altitude);
        gps_local_cartesian_.Forward(msg->latitude, msg->longitude, msg->altitude, cartesian_x, cartesian_y, cartesian_z);

        // UTM meridian convergence is not meaningful when using local cartesian, so set it to 0.0
        utm_meridian_convergence_ = 0.0;
    }
    else
    {
        double k_tmp;
        double utm_meridian_convergence_degrees;
        GeographicLib::UTMUPS::Forward(msg->latitude, msg->longitude, utm_zone_, northp_,
                                       cartesian_x, cartesian_y, utm_meridian_convergence_degrees, k_tmp);
        utm_meridian_convergence_ = utm_meridian_convergence_degrees * NavsatConversions::RADIANS_PER_DEGREE;
    }

    ROS_INFO_STREAM("Datum (latitude, longitude, altitude) is (" << std::fixed << msg->latitude << ", " <<
                                                                 msg->longitude << ", " << msg->altitude << ")");
    ROS_INFO_STREAM("Datum " << ((use_local_cartesian_)? "Local Cartesian" : "UTM") <<
                             " coordinate is (" << std::fixed << cartesian_x << ", " << cartesian_y << ") zone " << utm_zone_);

    transform_cartesian_pose_.setOrigin(tf2::Vector3(cartesian_x, cartesian_y, msg->altitude));
    transform_cartesian_pose_.setRotation(tf2::Quaternion::getIdentity());
    has_transform_gps_ = true;
}


void  gpsFixCallback(const sensor_msgs::NavSatFixConstPtr& msg)
{
    auto gps_frame_id_ = msg->header.frame_id;

    if (gps_frame_id_.empty())
    {
        ROS_WARN_STREAM_ONCE("NavSatFix message has empty frame_id. Will assume navsat device is mounted at robot's "
                             "origin.");
    }

    // Make sure the GPS data is usable
    bool good_gps = (msg->status.status != sensor_msgs::NavSatStatus::STATUS_NO_FIX &&
                     !std::isnan(msg->altitude) &&
                     !std::isnan(msg->latitude) &&
                     !std::isnan(msg->longitude));

    if (good_gps)
    {
        // If we haven't computed the transform yet, then
        // store this message as the initial GPS data to use
        if (!transform_good_ && !use_manual_datum_)
        {
            setTransformGps(msg);
        }

        double cartesian_x = 0.0;
        double cartesian_y = 0.0;
        double cartesian_z = 0.0;
        if (use_local_cartesian_)
        {
            gps_local_cartesian_.Forward(msg->latitude, msg->longitude, msg->altitude,
                                         cartesian_x, cartesian_y, cartesian_z);
        }
        else
        {
            // Transform to UTM using the fixed utm_zone_
            int zone_tmp;
            bool northp_tmp;
            try
            {
                GeographicLib::UTMUPS::Forward(msg->latitude, msg->longitude,
                                               zone_tmp, northp_tmp, cartesian_x, cartesian_y, utm_zone_);
            }
            catch (const GeographicLib::GeographicErr& e)
            {
                ROS_ERROR_STREAM_THROTTLE(1.0, e.what());
                return;
            }
        }
        latest_cartesian_pose_.setOrigin(tf2::Vector3(cartesian_x, cartesian_y, msg->altitude));
        latest_cartesian_covariance_.setZero();

        // Copy the measurement's covariance matrix so that we can rotate it later
        for (size_t i = 0; i < POSITION_SIZE; i++)
        {
            for (size_t j = 0; j < POSITION_SIZE; j++)
            {
                latest_cartesian_covariance_(i, j) = msg->position_covariance[POSITION_SIZE * i + j];
            }
        }

        gps_update_time_ = msg->header.stamp;
        gps_updated_ = true;
    }
}
#endif


#include <exception>
#include <cmath>
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>



int test_Geocentric() {
    using namespace std;
    using namespace GeographicLib;
    try {
        Geocentric earth(Constants::WGS84_a(), Constants::WGS84_f());
        // Alternatively: const Geocentric& earth = Geocentric::WGS84();
        const double lat0 = 48 + 50/60.0, lon0 = 2 + 20/60.0; // Paris
        LocalCartesian proj(lat0, lon0, 0, earth);
        proj.Reset(lat0, lon0, 0);
        {
            // Sample forward calculation
            double lat = 50.9, lon = 1.8, h = 0; // Calais
            double x, y, z;
            proj.Forward(lat, lon, h, x, y, z);
            cout << x << " " << y << " " << z << "\n";
        }
        {
            // Sample reverse calculation
            double x = -38e3, y = 230e3, z = -4e3;
            double lat, lon, h;
            proj.Reverse(x, y, z, lat, lon, h);
            cout << lat << " " << lon << " " << h << "\n";
        }
    }
    catch (const exception& e) {
        cerr << "Caught exception: " << e.what() << "\n";
        return 1;
    }

    return 1;

}


int main(int argc, char **argv) {

    test_Geocentric();


    ros::init(argc, argv, "listener");
    ros::NodeHandle n;

    int mode = 0;
    if (argc == 1){
        mode = 0;
    }else{

        mode = atoi(argv[1]);
    }

    if (mode == 0){
        ros::Subscriber sub = n.subscribe("chatter", 1, ChatterCallback);
        ros::MultiThreadedSpinner spinner(4);
        spinner.spin();
    }else if(mode == 1){
        ros::SubscribeOptions ops;
        ops.template init<std_msgs::Header>("chatter", 1, ChatterCallback);
        ops.allow_concurrent_callbacks = true;
        ros::Subscriber sub1 = n.subscribe(ops);
        ros::MultiThreadedSpinner spinner(4);
        spinner.spin();
    }else if(mode == 2){
        ros::Subscriber sub = n.subscribe("chatter", 4, ChatterCallback);

        ros::AsyncSpinner spinner(4); // Use 4 threads
        spinner.start();
        ros::waitForShutdown();
    }else if(mode == 3){
        ros::SubscribeOptions ops;
        ops.template init<std_msgs::Header>("chatter", 1, ChatterCallback);
        ops.allow_concurrent_callbacks = true;
        ros::Subscriber sub1 = n.subscribe(ops);

        ros::AsyncSpinner spinner(4); // Use 4 threads
        spinner.start();
        ros::waitForShutdown();
    }else if(mode == 4){

        ros::NodeHandle n_a;


        ros::CallbackQueue callback_queue_a;
        n_a.setCallbackQueue(&callback_queue_a);

        ros::SubscribeOptions ops;
        ops.template init<std_msgs::Header>("chatter2", 1, ChatterCallback2);
        ops.allow_concurrent_callbacks = true;


        ros::Subscriber sub_a = n_a.subscribe(ops);

        std::thread spinner_thread_a([&callback_queue_a]() {
            ros::MultiThreadedSpinner spinner(4);
            spinner.spin(&callback_queue_a);

        });

        ros::Subscriber sub = n.subscribe("chatter", 1, ChatterCallback);
        ros::MultiThreadedSpinner spinner(4);
        spinner.spin();
        spinner_thread_a.join();

    }

    return 0;
}