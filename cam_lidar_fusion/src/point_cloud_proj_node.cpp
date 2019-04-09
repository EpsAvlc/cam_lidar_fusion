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
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

using namespace std;
using namespace cv;

// global paramters;
string cam_topic_str, lidar_topic_str;
Mat velo2cam;
Mat rect_R;
Mat cam_intrins;
int img_cols, img_rows;

void UpdateParam(ros::NodeHandle nh_local_)
{
    nh_local_.param<string>("cam_topic", cam_topic_str, "/kitti/camera_gray_left/image_raw");
    nh_local_.param<string>("lidar_topic", lidar_topic_str, "/kitti/velo/pointcloud");

    FileStorage fs("/home/cm/Workspaces/cam_lidar_fusion/src/cam_lidar_fusion/config/config.yaml", FileStorage::READ);
    ROS_ASSERT(fs.isOpened());

    fs["velo2cam"] >> velo2cam;
    // fs["velo2cam_t"] >> velo2cam_t;
    fs["cam_intrins"] >> cam_intrins;
    fs["img_cols"] >> img_cols;
    fs["img_rows"] >> img_rows;
    // fs["rect_R"] >> rect_R;
    // ROS_INFO("\n Project information: \n image size: %d X %d \n ", img_cols, img_rows);
}
/** 
 * output : projected pixel point with distance.
 *
 */
void ProjectPointCloudOnImagePlaner(const pcl::PointCloud<pcl::PointXYZ>& transformed_cloud, vector<Point3d>& fit_points)
{
    fit_points.clear();
    // Eigen::MatrixXd rect_R_eigen(4, 4);
    // cv2eigen(rect_R, rect_R_eigen);
    Eigen::Matrix3d camera_intrins_eigen;
    // ROS_INFO("BEFOR EIGENLIZED");
    cv2eigen(cam_intrins, camera_intrins_eigen);
    // ROS_INFO("AFTER EIGENLIZED");
    Eigen::MatrixXd points(3, transformed_cloud.size());
    for(int i = 0; i < transformed_cloud.size(); ++i)
    {
        points(0, i) = transformed_cloud[i].x;
        points(1, i) = transformed_cloud[i].y;
        points(2, i) = transformed_cloud[i].z;
    }
    Eigen::MatrixXd points_pixel = camera_intrins_eigen * points;
    for(int i = 0; i < transformed_cloud.size(); ++i)
    {
        Point3d tmp;
        tmp.x = points_pixel(0, i) / points_pixel(2, i);
        tmp.y = points_pixel(1, i) / points_pixel(2, i);
        tmp.z = points_pixel(2, i);
        // cout << tmp.z << endl;
        if(tmp.x < img_cols && tmp.x > 0 
        && tmp.y < img_rows && tmp.y > 0 && tmp.z > 0)
        {
            fit_points.push_back(tmp);
        }
    }
}

void Callback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::PointCloud2ConstPtr& cloud)
{
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromROSMsg(*cloud, pcl_cloud);

    Eigen::Matrix4f transform;
    cv2eigen(velo2cam, transform);

    pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
    pcl::transformPointCloud(pcl_cloud, transformed_cloud, transform);

    vector<Point3d> proj_points;
    ProjectPointCloudOnImagePlaner(transformed_cloud, proj_points);

    // Get cam image
    cv_bridge::CvImagePtr img_ptr = cv_bridge::toCvCopy(image);
    Mat src_img = img_ptr->image;
    for(int i = 0; i < proj_points.size(); ++i)
    {
        Scalar color(saturate_cast<uchar>(proj_points[i].z / 3. * 255));
        circle(src_img, Point(proj_points[i].x, proj_points[i].y), 2, color, -1);
    }
    
    imshow("after", src_img);
    waitKey(5);
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "kitti_parser_node");
    ros::NodeHandle nh(""), nh_local("~");

    UpdateParam(nh_local);
    message_filters::Subscriber<sensor_msgs::Image> cam_sub(nh, cam_topic_str, 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_sub(nh, lidar_topic_str, 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> cam_lidar_fuse_policy;
    message_filters::Synchronizer<cam_lidar_fuse_policy> sync(cam_lidar_fuse_policy(10), cam_sub, lidar_sub);
    sync.registerCallback(boost::bind(&Callback, _1, _2));

    ros::spin();
}