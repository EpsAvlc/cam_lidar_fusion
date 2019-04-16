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
#include <tf/transform_broadcaster.h>

// #include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

using namespace std;
using namespace cv;

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "disp_lidar_cam_transform");
    ros::NodeHandle nh_local("cam_lidar_fusion");

    vector<double> xyzrpy;
    tf::Transform lidar_cam_trans;
    tf::Quaternion lidar_cam_q;
    if(nh_local.getParam("xyzrpy", xyzrpy))
    {
        ROS_INFO("XYZRPY: %lf, %lf, %lf, %lf, %lf, %lf", xyzrpy[0], xyzrpy[1], xyzrpy[2], xyzrpy[3], xyzrpy[4], xyzrpy[5]);
        lidar_cam_q.setRPY(xyzrpy[3], xyzrpy[4], xyzrpy[5]);
        // cout << lidar_cam_q.inverse(). << endl;
        lidar_cam_trans.setRotation(lidar_cam_q);
        lidar_cam_trans.setOrigin(tf::Vector3(xyzrpy[0], xyzrpy[1], xyzrpy[2]));
    }
    
    tf::TransformBroadcaster br;
    ros::Rate loop_rate(10);
    while (ros::ok()) 
    {
        br.sendTransform(tf::StampedTransform(lidar_cam_trans, ros::Time::now(), "camera", "lidar"));
        loop_rate.sleep();
    }
}