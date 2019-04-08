/*
 * Created on Fri Apr 05 2019
 *
 * Copyright (c) 2019 HITSZ-NRSL
 * All rights reserved
 *
 * Author: EpsAvlc
 */

#include <iostream>

#include <ros/ros.h>

#include <opencv2/core/eigen.hpp>

using namespace std;
using namespace cv;

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "disp_lidar_cam_transform");

    FileStorage fs("/home/cm/Workspaces/cam_lidar_fusion/src/cam_lidar_fusion/config/config.yaml", FileStorage::READ);

    // Mat cam_Lidar_trans
}