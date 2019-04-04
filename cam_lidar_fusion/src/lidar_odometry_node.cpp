/*
 * Created on Tue Jan 22 2019
 *
 * Copyright (c) 2019 HITSZ-NRSL
 * All rights reserved
 *
 * Author: EpsAvlc
 */
#include <iostream>
#include <thread>

// ROS header
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// PCL header
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/icp.h>

// BOOST header
#include <boost/make_shared.hpp>

using namespace std;

class LidarOdom
{
public:
    LidarOdom(ros::NodeHandle nh, ros::NodeHandle nh_local)
    :nh_(nh), nh_local_(nh_local)
    {
        this->paramInit(); 
        lidar_sub_ = nh_.subscribe(lidar_topic_str_, 1, &LidarOdom::lidarCallback, this);
    }
private:
    ros::NodeHandle nh_, nh_local_;
    ros::Subscriber lidar_sub_;
    string lidar_topic_str_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cur_cloud_, last_cloud_;
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp_;

    void paramInit()
    {
        nh_local_.param<string>("lidar_topic", lidar_topic_str_, "/kitti/velo/pointcloud");

        cur_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
        last_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);

        icp_.setMaxCorrespondenceDistance(0.1);
        icp_.setTransformationEpsilon(1e-10);
        icp_.setEuclideanFitnessEpsilon(0.01);
        icp_.setMaximumIterations (100);
    }

    void lidarCallback(sensor_msgs::PointCloud2ConstPtr cloud_ptr)
    {
        pcl::fromROSMsg(*cloud_ptr, *cur_cloud_);
        
        if(last_cloud_->size() == 0)
        {
            last_cloud_.swap(cur_cloud_);
            cur_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
            return;
        }
        Eigen::Matrix4f T = this->icp();
        cout << T << endl;
        // cout << "cur size: " << cur_cloud_->size() << endl;
        // cout << "last size: " << last_cloud_->size() << endl;
        last_cloud_.swap(cur_cloud_);
        cur_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    }

    Eigen::Matrix4f icp()
    {
        icp_.setInputSource(last_cloud_);
        icp_.setInputTarget(cur_cloud_);
        pcl::PointCloud<pcl::PointXYZ> Final;
        icp_.align(Final);
        return icp_.getFinalTransformation();
    }
};

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "lidar_odom_node");
    ros::NodeHandle nh(""), nh_local("~");

    LidarOdom lo(nh, nh_local);
    ros::spin();
}