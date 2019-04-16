/*
 * Created on Wed Apr 10 2019
 *
 * Copyright (c) 2019 HITSZ-NRSL
 * All rights reserved
 *
 * Author: EpsAvlc
 */

#include <Eigen/Core>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
 
class PointCloudProj
{
public:
    PointCloudProj(ros::NodeHandle& nh, ros::NodeHandle& nh_local);
private:
    void readParam();
    void callback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::PointCloud2ConstPtr& cloud);
    /** ros params **/
    ros::NodeHandle nh_, nh_local_;
    message_filters::Subscriber<sensor_msgs::Image> cam_sub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_sub_;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> cam_lidar_fuse_policy;
    typedef message_filters::Synchronizer<cam_lidar_fuse_policy> Sync;
    boost::shared_ptr<Sync> sync;
    /** class variable**/
    Eigen::Matrix3d cam_intrins_;
    Eigen::Matrix4d lidar_to_cam_;
    Eigen::Matrix4d pose_init_;
    std::string lidar_topic_, cam_topic_;
};
