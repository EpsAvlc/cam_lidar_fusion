/*
 * Created on Mon Jan 21 2019
 *
 * Copyright (c) 2019 HITSZ-NRSL
 * All rights reserved
 *
 * Author: EpsAvlc
 */
#include <iostream>
#include <thread>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


using namespace std;
using namespace cv;

class PointCloudProj
{
public:
    PointCloudProj(ros::NodeHandle nh, ros::NodeHandle nh_local) : nh_(nh), nh_local_(nh_local)
    {
        
    }
private:
 
    ros::NodeHandle nh_, nh_local_;
};

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "kitti_parser_node");
    ros::NodeHandle nh(""), nh_local("~");
    // KittiParser parser(nh, nh_local);
    ros::spin();
}