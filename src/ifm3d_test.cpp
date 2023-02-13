/*
 * Copyright 2021-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */
#include <iostream>
#include <iostream>
#include <memory>
#include <chrono>
#include <vector>
#include <ctime>
#include <iomanip>
#include <thread>
#include <chrono>

#include <ifm3d/camera/camera_base.h>
#include <ifm3d/fg.h>
#include <ifm3d/stlimage.h>

#include <thinks/ppm.hpp>
#include <ifm3d/simpleimage.h>
#include <pcl/point_cloud.h>

#include <pcl/io/pcd_io.h>



//Utility function to format the timestamp
std::string formatTimestamp(ifm3d::TimePointT timestamp)
{
    std::time_t time = std::chrono::system_clock::to_time_t(
            std::chrono::time_point_cast<std::chrono::system_clock::duration>(
                    timestamp));

    std::chrono::milliseconds milli = std::chrono::duration_cast<std::chrono::milliseconds>(
            timestamp.time_since_epoch() - std::chrono::duration_cast<std::chrono::seconds>(
                    timestamp.time_since_epoch()));

    std::ostringstream s;
    s << std::put_time(std::localtime(&time), "%Y-%m-%d %H:%M:%S")
      << ":" << std::setw(3) << std::setfill('0') << milli.count();

    return s.str();
}



bool writePPMFile(ifm3d::SimpleImageBuffer::Img &img, std::string const& filename)
{
    auto const write_width = (size_t)img.width;
    auto const write_height = (size_t)img.height;
    auto write_pixels = std::vector<uint8_t>(write_width * write_height * 3); // 3 for RGB channels
    auto pixel_index = size_t{ 0 };
    for (auto col = size_t{ 0 }; col < write_height; ++col) {
        for (auto row = size_t{ 0 }; row < write_width; ++row) {
            write_pixels[pixel_index * 3 + 0] = static_cast<uint8_t>(*(img.data.data() + pixel_index));
            write_pixels[pixel_index * 3 + 1] = static_cast<uint8_t>(*(img.data.data() + pixel_index));
            write_pixels[pixel_index * 3 + 2] = static_cast<uint8_t>(*(img.data.data() + pixel_index));
            ++pixel_index;
        }
    }
    try
    {
        thinks::ppm::writeRgbImage(filename, write_width, write_height, write_pixels);
    }
    catch (std::exception e)
    {
        std::cerr << e.what();
        return false;
    }
    return true;
}


// scales the data with min max values
template <typename  T>
void scaleImageToRGB(ifm3d::SimpleImageBuffer::Img &input, ifm3d::SimpleImageBuffer::Img &confidence, ifm3d::SimpleImageBuffer::Img &output, double min = 0.0f, double max = 0.0f)
{
    output.width = input.width;
    output.height = input.height;
    output.format = ifm3d::pixel_format::FORMAT_8U;
    output.data.resize(output.width*output.height);

    float scalingFactor = 255.0 / ((max - min) != 0 ? (max - min) : 1);

    for (int index = 0; index < input.width * input.height; index++)
    {
        T value = *((T*)(input.data.data()) + index);
        if ((*(confidence.data.data() + index) & 0x01) == 0x00) // checking valid pixel
            *(output.data.data() + index) = (uint8_t)((value - min)* scalingFactor);
        else
        {
            *(output.data.data() + index) = 0; // All invalid pixels
        }
    }
}
// find the min max of the data
template < typename  T >
void findMinAndMax(ifm3d::SimpleImageBuffer::Img &input, ifm3d::SimpleImageBuffer::Img &confidence, double &min, double &max)
{
    max = 0;
    min = (double)INT_MAX;
    for (int index = 0; index < input.width * input.height; index++)
    {
        T value = *((T*)(input.data.data()) + index);
        if ((*(confidence.data.data() + index) & 0x01) == 0x00)
        {
            min = std::min((T)min, value);
        }
        if ((*(confidence.data.data() + index) & 0x01) == 0x00)
        {
            max = std::max((T)max, value);
        }
    }
}

int main(){


    {


        auto camera_ip_ = "169.254.165.220";
        int xmlrpc_port_ = 80;
        int pcic_port_ = 50010;
        int timeout_millis_ = 1000;



        std::vector<float> extrinsics(6);
        std::vector<float> intrinsics(6);
        std::vector<float> inverse_intrinsics(6);





        bool retval = false;


        //this->cam_ = ifm3d::CameraBase::MakeShared();
        auto cam_ = ifm3d::CameraBase::MakeShared(camera_ip_, xmlrpc_port_);
        // this->cam_ = std::make_shared<ifm3d::CameraBase>(this->camera_ip_, this->xmlrpc_port_);

//        std::uint16_t mask =ifm3d::IMG_AMP | ifm3d::IMG_CART | ifm3d::IMG_RDIS;
        std::uint16_t mask =ifm3d::IMG_AMP | ifm3d::INTR_CAL | ifm3d::IMG_RDIS | ifm3d::IMG_CART;

        //ifm3dpy.IMG_AMP | ifm3dpy.INTR_CAL | ifm3dpy.IMG_RDIS | ifm3dpy.IMG_CART

        using namespace std::chrono_literals;


        std::this_thread::sleep_for(1s);

        // get the JSON configuration data from the camera
        auto jsonConfig = cam_->ToJSON();

        // print out the MAC address
        std::cout << "The MAC address of the camera: "
                  << jsonConfig["ifm3d"]["Net"]["MACAddress"]
                  << std::endl;
        std::cout << "The App config: "
                  << jsonConfig["ifm3d"]["Apps"]
                  << std::endl;


        auto fg_ = std::make_shared<ifm3d::FrameGrabber>(cam_, mask, pcic_port_);

        auto im_ = std::make_shared<ifm3d::StlImageBuffer>();

        ifm3d::SimpleImageBuffer::Ptr img = std::make_shared<ifm3d::SimpleImageBuffer>();





        fg_->SWTrigger();
        retval = fg_->WaitForFrame(im_.get(),timeout_millis_);




        fg_->SWTrigger();
        retval = fg_->WaitForFrame(img.get(),timeout_millis_);

        if(retval){
            std::cout << "get data"<<std::endl;
            ifm3d::TimePointT timestamp = img->TimeStamp();
            std::cout << "got camera("<<camera_ip_<<") frame, timestamp"
                      << std::setw(2) << std::setfill('0')
                      << ": " << formatTimestamp(timestamp)
                      << std::endl;

            // acquiring data from the device
//            ifm3d::SimpleImageBuffer::Img confidence = img->ConfidenceImage();
//            ifm3d::SimpleImageBuffer::Img amplitude = img->AmplitudeImage();
//            ifm3d::SimpleImageBuffer::Img distance = img->DistanceImage();
//
//            // for storing scaled output
//            ifm3d::SimpleImageBuffer::Img distance_scaled;
//            ifm3d::SimpleImageBuffer::Img amplitude_scaled;

            // acquiring data from the device
            ifm3d::SimpleImageBuffer::Img confidence = img->ConfidenceImage();
            ifm3d::SimpleImageBuffer::Img amplitude = img->AmplitudeImage();
            ifm3d::SimpleImageBuffer::Img distance = img->DistanceImage();


            // for storing scaled output
            ifm3d::SimpleImageBuffer::Img distance_scaled;
            ifm3d::SimpleImageBuffer::Img amplitude_scaled;

            ifm3d::SimpleImageBuffer::PointCloud cloud = img->Cloud();


            printf("distance_img %d %d", distance.height, distance.width);
            printf("cloud %d %d", cloud.height, cloud.width);


            double min = 0.0;
            double max = 0.0;
            // max and mindistance for scaling distance image uint8 format
            auto const max_distance = 2.5;
            auto const min_distance = 0.0;
            // for 32F data  O3X camera
            if (distance.format == ifm3d::pixel_format::FORMAT_32F)
            {
                scaleImageToRGB<float>(distance, confidence, distance_scaled, min_distance, max_distance);
                findMinAndMax<float>(amplitude, confidence, min, max);
                scaleImageToRGB<float>(amplitude, confidence, amplitude_scaled, min, max);
            }
                //for 16u data  O3D camera
            else if(distance.format == ifm3d::pixel_format::FORMAT_16U)
            {	  //if data format is 16U then distances are in millimeters, Hence max distance is multiplied by 1000.
                scaleImageToRGB<unsigned short>(distance, confidence, distance_scaled, min_distance, max_distance * 1000);
                findMinAndMax<unsigned short>(amplitude, confidence, min, max);
                scaleImageToRGB<unsigned short>(amplitude, confidence, amplitude_scaled, min, max);
            }
            else
            {
                std::cerr << "Unknown Format" << std::endl;
            }
            std::cerr << "List Cloud Points" << std::endl;

            pcl::PointCloud<pcl::PointXYZ>::Ptr init_cloud (new pcl::PointCloud<pcl::PointXYZ>);
            // populate our PointCloud with points
            init_cloud->width    = cloud.points.size();
            init_cloud->height   = 1;
            init_cloud->is_dense = true;
            init_cloud->points.resize (init_cloud->width * init_cloud->height);

            for (pcl::index_t i = 0; i < init_cloud->size (); ++i){
                init_cloud->at(i).x = cloud.points[i].x;
                init_cloud->at(i).y = cloud.points[i].y;
                init_cloud->at(i).z = cloud.points[i].z;
            }

            pcl::io::savePCDFileASCII("depthcloud.pcd",*init_cloud);

             std::cerr << "List Cloud Points Done" << std::endl;

            //writing images too ppm format
            if (!writePPMFile(distance_scaled, "distanceImage.ppm"))
            {
                std::cerr << "Not able to write the distance data in ppm format" << std::endl;
                return -1;
            }

            if (!writePPMFile(amplitude_scaled, "amplitudeImage.ppm"))
            {
                std::cerr << "Not able to write the amplitude data in ppm format" << std::endl;
                return -1;
            }


            std::cout << "Done with simpleimage ppmio example" << std::endl;


/*

            # 2. Use the extrinsics to warp from imager frame to camera frame
            # NOTE: If the user has written custom extrinsic information to the device,
            # this transformation will be included here.
            im_ext = im.extrinsics() # [x y z roll pitch yaw], units are mm and deg
 */
            extrinsics = img->Extrinsics();
            intrinsics = img->Intrinsics();

            //fx, fy, mx, my, alpha, k1, k2, k5, k3, k4, *_ = im.inverse_intrinsics()
            inverse_intrinsics = img->InverseIntrinsics();
            float tx = extrinsics.at(0);
            float ty = extrinsics.at(1);
            float tz = extrinsics.at(2);
            float rot_x = extrinsics.at(3);
            float rot_y = extrinsics.at(4);
            float rot_z = extrinsics.at(5);

            printf("inverse_intrinsics:[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f]",inverse_intrinsics[0],inverse_intrinsics[1],inverse_intrinsics[2],inverse_intrinsics[3],inverse_intrinsics[4],inverse_intrinsics[5]);


            printf("intrinsics:[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f]",intrinsics[0],intrinsics[1],intrinsics[2],intrinsics[3],intrinsics[4],intrinsics[5]);

            printf("extrinsics:[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f]",extrinsics[0],extrinsics[1],extrinsics[2],extrinsics[3],extrinsics[4],extrinsics[5]);
        }else{
            std::cout << "no data"<<std::endl;

        }

        return 0;


    }
}