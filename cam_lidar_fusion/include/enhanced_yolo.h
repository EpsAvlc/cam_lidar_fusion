/*
 * Created on Tue Apr 16 2019
 *
 * Copyright (c) 2019 HITSZ-NRSL
 * All rights reserved
 *
 * Author: EpsAvlc
 */

#ifndef ENHANCED_YOLO_H_
#define ENHANCED_YOLO_H_

#include <Eigen/Core>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include "darknet_ros_msgs/BoundingBoxes.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

class EnhancedYOLO
{
public:
    EnhancedYOLO(ros::NodeHandle& nh, ros::NodeHandle& nh_local);
private:
    void readParam();
    void callback(const sensor_msgs::ImageConstPtr& yolo_img, const sensor_msgs::PointCloud2ConstPtr& cropped_cloud, const darknet_ros_msgs::BoundingBoxesConstPtr& bd_boxes);
    void drawCube(cv::Mat& src_img, cv::Point3d min_xyz, cv::Point3d max_xyz);
    /* Kmeans for seperate foreground and background */
    std::vector<cv::Point3f> clusterPoints(std::vector<cv::Point3f>& points);
    bool filterBboxByArea(const darknet_ros_msgs::BoundingBox& bbox, double range);

    /** ros params **/
    ros::NodeHandle nh_, nh_local_;
    message_filters::Subscriber<sensor_msgs::Image> yolo_sub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_sub_;
    message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> bd_box_sub_;

    ros::Publisher detect_result_cloud_pub_;
    ros::Publisher enhanced_yolo_img_pub_;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2, darknet_ros_msgs::BoundingBoxes> enhanced_yolo_policy;
    typedef message_filters::Synchronizer<enhanced_yolo_policy> Sync;
    boost::shared_ptr<Sync> sync;

    /** class variable **/
    Eigen::Matrix3d cam_intrins_;
    std::map<std::string, std::pair<double, double>> area_thres_;
};

#endif // !ENHANCED_YOLO_H_
