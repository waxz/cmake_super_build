//
// Created by waxz on 8/27/22.
//
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include <iostream>

int main(int argc,char** argv){

    cv::Mat C = (cv::Mat_<double>(3,3) << 0, -1, 0, -1, 5, -1, 0, -1, 0);
    std::cout << "C = " << std::endl << " " << C << std::endl ;
}