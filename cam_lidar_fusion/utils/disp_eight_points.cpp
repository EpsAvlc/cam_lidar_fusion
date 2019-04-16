/*
 * Created on Mon Apr 15 2019
 *
 * Copyright (c) 2019 HITSZ-NRSL
 * All rights reserved
 *
 * Author: EpsAvlc
 */
#include <iostream>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

// #include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>

#include <Eigen/Core>

using namespace std;
using namespace cv;

Eigen::MatrixXd lidar_points_2d;
Eigen::MatrixXd cam_points_2d;

void Callback(const sensor_msgs::ImageConstPtr& img)
{
    auto cv_img = cv_bridge::toCvCopy(img);
    Mat disp_img = cv_img->image;
    cvtColor(disp_img, disp_img, COLOR_GRAY2RGB);
    for(int i = 0; i < 8; i++)
    {
        Point2d lidar_point_px;
        lidar_point_px.x = lidar_points_2d(0, i) / lidar_points_2d(2, i);
        lidar_point_px.y = lidar_points_2d(1, i) / lidar_points_2d(2, i);
        circle(disp_img, lidar_point_px, 3, Scalar(255, 0, 0), -1); // blue

        Point2d cam_point_px;
        cam_point_px.x = cam_points_2d(0, i) / cam_points_2d(2, i);
        cam_point_px.y = cam_points_2d(1, i) / cam_points_2d(2, i);
        circle(disp_img, cam_point_px, 3, Scalar(0, 0, 255), -1); // red
    }
    imshow("disp", disp_img);
    waitKey(10);
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "disp_eight_points");
    ros::NodeHandle nh_local("cam_lidar_fusion");

    vector<double> cam_intrins_data;
    nh_local.getParam("cam_intrins", cam_intrins_data);
    

    // load cam intrins
    Eigen::Matrix3d cam_intrins;
    for(int i = 0; i < cam_intrins_data.size(); i++)
    {
        cam_intrins(i) = cam_intrins_data[i];
    }
    /* Note that the order of data in matrix in ros is diffent from eigen*/
    cam_intrins.transposeInPlace();
    cout << "Camera intrins is: " << endl << cam_intrins << endl;

    // load lidar to cam
    Eigen::Matrix4d lidar_to_cam;
    vector<double> lidar_to_cam_data;
    nh_local.getParam("lidar_to_cam", lidar_to_cam_data);
    for(int i = 0; i < 16; i++)
    {
        lidar_to_cam(i) = lidar_to_cam_data[i];
    }
    lidar_to_cam.transposeInPlace();
    cout << "lidar_to_cam is: " << endl << lidar_to_cam << endl;

    // load lidar points
    vector<double> lidar_points_data;
    nh_local.getParam("lidar_points", lidar_points_data);
    cout << lidar_points_data.size() << endl;
    Eigen::MatrixXd lidar_points(4, 8);
    for(int i = 0; i < 8; i++)
    {
        if(i*3 + 0 > 24)
            break;
        lidar_points(0, i) = lidar_points_data[i*3 + 0];
        lidar_points(1, i) = lidar_points_data[i*3 + 1];
        lidar_points(2, i) = lidar_points_data[i*3 + 2];
        lidar_points(3, i) = 1;
    }
    cout << "lidar_points_are:" << endl << lidar_points.matrix() << endl;
    // load cam points
    vector<double> cam_points_data;
    nh_local.getParam("cam_points", cam_points_data);
    Eigen::MatrixXd cam_points(3, 8);
    for(int i = 0; i < cam_points.size() / 3; i++)
    {
        cam_points(0, i) = cam_points_data[i*3 + 0];
        cam_points(1, i) = cam_points_data[i*3 + 1];
        cam_points(2, i) = cam_points_data[i*3 + 2];
    }
    cout << "cam_points_are:" << endl << cam_points << endl;
    // Projection
    lidar_points_2d = cam_intrins * (lidar_to_cam * lidar_points).block(0,0,3,8);

    cout << "lidar_points_2d: " << endl << lidar_points_2d << endl;;
    cam_points_2d = cam_intrins * cam_points;
    
    //solve pnp
    vector<Point3f> object_points;
    vector<Point2f> img_points;
    for(int i =0; i < 4; i++)
    {
        Point3f obj_pt;
        obj_pt.x = lidar_points(0, i);
        obj_pt.y = lidar_points(1, i);
        obj_pt.z = lidar_points(2, i);
        object_points.push_back(obj_pt);
        cout << obj_pt << endl;

        Point2f img_pt;
        img_pt.x = cam_points_2d(0, i) / cam_points_2d(2, i);
        img_pt.y = cam_points_2d(1, i) / cam_points_2d(2, i);
        img_points.push_back(img_pt);
        cout << img_pt << endl;
    }
    cout << img_points.size() << endl;
    cout << object_points.size() << endl;
    Mat cam_intrins_cv(3, 3, CV_32FC1);
    for(int i = 0; i < 3; i++)
        for(int j = 0; j < 3; j++)
        {
            cam_intrins_cv.at<float>(i, j) = cam_intrins(i, j);
        }
    cout << cam_intrins_cv << endl;
    cv::Mat rvec ,tvec ,RotationR;
    solvePnP(object_points, img_points, cam_intrins_cv, cv::noArray(), rvec, tvec);
    Mat rotation_mat;
    Rodrigues(rvec, rotation_mat);
    cout << "PNP rot: " << endl << rotation_mat << endl;
    cout << "PNP t: " << endl << tvec << endl;

    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/camera_array/cam0/image_raw", 1, Callback);
    ros::spin();
}