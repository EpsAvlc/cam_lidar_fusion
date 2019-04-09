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

    FileStorage fs("/home/cm/Workspaces/cam_lidar_fusion/src/cam_lidar_fusion/config/config.yaml", FileStorage::READ);

    Mat lidar2cam;
    ROS_ASSERT(fs.isOpened());
    fs["velo2cam"] >> lidar2cam;

    tf::Transform lidar_cam_trans;
    lidar_cam_trans.setOrigin(tf::Vector3(lidar2cam.at<double>(0,3), lidar2cam.at<double>(1,3), lidar2cam.at<double>(2,3)));
    // cout << tf::Vector3(lidar2cam.at<double>(3,0), lidar2cam.at<double>(3,1), lidar2cam.at<double>(3,2)); 
    tf::Matrix3x3 lidar_cam_rot;
    lidar_cam_rot.setValue(lidar2cam.at<double>(0,0), lidar2cam.at<double>(0,1), lidar2cam.at<double>(0,2),
                        lidar2cam.at<double>(1,0), lidar2cam.at<double>(1,1), lidar2cam.at<double>(1,2),
                        lidar2cam.at<double>(2,0), lidar2cam.at<double>(2,1), lidar2cam.at<double>(2,2));
    tf::Quaternion lidar_cam_q;
    lidar_cam_rot.getRotation(lidar_cam_q);
    lidar_cam_trans.setRotation(lidar_cam_q);
    tf::TransformBroadcaster br;
    while (ros::ok()) 
        br.sendTransform(tf::StampedTransform(lidar_cam_trans, ros::Time::now(), "camera", "lidar"));
}