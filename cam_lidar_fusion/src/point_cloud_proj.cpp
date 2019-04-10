/*
 * Created on Wed Apr 10 2019
 *
 * Copyright (c) 2019 HITSZ-NRSL
 * All rights reserved
 *
 * Author: EpsAvlc
 */

#include "point_cloud_proj.h"

#include <pcl_conversions/pcl_conversions.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

PointCloudProj::PointCloudProj(ros::NodeHandle& nh, ros::NodeHandle& nh_local): nh_(nh), nh_local_(nh_local)
{
    readParam();
    cam_sub_.subscribe(nh_, cam_topic_, 1);
    lidar_sub_.subscribe(nh_, lidar_topic_, 1);
    sync.reset(new Sync(cam_lidar_fuse_policy(10), cam_sub_, lidar_sub_));
    sync->registerCallback(boost::bind(&PointCloudProj::callback, this, _1, _2));
}

void PointCloudProj::readParam()
{
    vector<double> cam_intrins_data;
    ROS_ASSERT(nh_local_.getParam("cam_intrins", cam_intrins_data));
    ROS_ASSERT(cam_intrins_data.size() == 9);
    for(int i = 0; i < cam_intrins_data.size(); i++)
    {
        cam_intrins_(i) = cam_intrins_data[i];
    }
    /* Note that the order of data in matrix in ros is diffent from eigen*/
    cam_intrins_.transposeInPlace();
    
    vector<double> lidar2cam_data;
    ROS_ASSERT(nh_local_.getParam("lidar2cam", lidar2cam_data));
    ROS_ASSERT(lidar2cam_data.size() == 16);
    for(int i = 0; i < lidar2cam_data.size(); i++)
    {
        lidar_to_cam_(i) = lidar2cam_data[i];
    }
    lidar_to_cam_.transposeInPlace();

    ROS_ASSERT(nh_local_.getParam("cam_topic", cam_topic_));
    ROS_ASSERT(nh_local_.getParam("lidar_topic", lidar_topic_));
}

void PointCloudProj::callback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::PointCloud2ConstPtr& cloud)
{
    /* Projection */
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromROSMsg(*cloud, pcl_cloud);
    Eigen::MatrixXd points_3d_homo(4, pcl_cloud.size());
    for(int i = 0; i < pcl_cloud.size(); i++)
    {
        points_3d_homo(0, i) = pcl_cloud[i].x;
        points_3d_homo(1, i) = pcl_cloud[i].y;
        points_3d_homo(2, i) = pcl_cloud[i].z;
        points_3d_homo(3, i) = 1;
    }
    Eigen::MatrixXd points_3d_in_cam_homo = lidar_to_cam_ * points_3d_homo;
    Eigen::MatrixXd points_3d_in_cam = points_3d_in_cam_homo.block(0, 0, 3, pcl_cloud.size());
    Eigen::MatrixXd points_2d_homo = cam_intrins_ * points_3d_in_cam;

    /* Draw result */
    cv_bridge::CvImagePtr img_ptr = cv_bridge::toCvCopy(image);
    for(int i = 0; i < pcl_cloud.size(); i++)
    {
        int x = points_2d_homo(0, i) / points_2d_homo(2, i);
        int y = points_2d_homo(1, i) / points_2d_homo(2, i);
        if(x < 0 || x > img_ptr->image.cols ||
           y < 0 || y > img_ptr->image.rows || points_2d_homo(2, i) < 0)
            continue;
        uchar color_val = points_3d_in_cam(2, i) > 2 ? 0 : 255;
        Scalar color(color_val);
        circle(img_ptr->image, Point(x, y), 2, color, -1);
    }
    imshow("after", img_ptr->image);
    waitKey(5);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "point_cloud_proj");
    ros::NodeHandle nh(""), nh_local("cam_lidar_fusion");
    PointCloudProj pcp(nh, nh_local);
    ros::spin();
}