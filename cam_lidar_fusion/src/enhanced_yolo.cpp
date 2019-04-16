/*
 * Created on Tue Apr 16 2019
 *
 * Copyright (c) 2019 HITSZ-NRSL
 * All rights reserved
 *
 * Author: EpsAvlc
 */

#include "enhanced_yolo.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/time.h>
#include <pcl/filters/passthrough.h>

using namespace std;
using namespace cv;

EnhancedYOLO::EnhancedYOLO(ros::NodeHandle& nh, ros::NodeHandle& nh_local): nh_(nh), nh_local_(nh_local)
{
    readParam();
    yolo_sub_.subscribe(nh_, "/darknet_ros/detection_image", 1);
    lidar_sub_.subscribe(nh_, "/cropped_cloud", 1);
    bd_box_sub_.subscribe(nh_, "/darknet_ros/bounding_boxes", 1);
    // because generate cropped cloud cost too much time, so we need to set buffer bigger = 50. 
    sync.reset(new Sync(enhanced_yolo_policy(50), yolo_sub_, lidar_sub_, bd_box_sub_));
    sync->registerCallback(boost::bind(&EnhancedYOLO::callback, this, _1, _2, _3));
    detect_result_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("detect_result_cloud", 1);
    enhanced_yolo_img_pub_ = nh_.advertise<sensor_msgs::Image>("enhanced_yolo_img", 1);
}

void EnhancedYOLO::readParam()
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
}

void EnhancedYOLO::callback(const sensor_msgs::ImageConstPtr& yolo_img, const sensor_msgs::PointCloud2ConstPtr& cropped_cloud, const darknet_ros_msgs::BoundingBoxesConstPtr& bd_boxes)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cropped_cloud, *pcl_cloud);

    Eigen::MatrixXd points_3d_in_cam(3, pcl_cloud->size());
    for(int i = 0; i < pcl_cloud->size(); i++)
    {
        points_3d_in_cam(0, i) = (*pcl_cloud)[i].x;
        points_3d_in_cam(1, i) = (*pcl_cloud)[i].y;
        points_3d_in_cam(2, i) = (*pcl_cloud)[i].z;
    }    
    Eigen::MatrixXd points_2d_homo = cam_intrins_ * points_3d_in_cam;

    Eigen::MatrixXd points_2d(2, pcl_cloud->size());
    for(int i = 0; i < pcl_cloud->size(); i++)
    {
        points_2d(0, i) = points_2d_homo(0, i) / points_2d_homo(2, i);
        points_2d(1, i) = points_2d_homo(1, i) / points_2d_homo(2, i);
        // cout << points_2d(0, i) << ", " << points_2d(1, i) << endl;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    cv_bridge::CvImagePtr img_ptr = cv_bridge::toCvCopy(yolo_img);
    Mat yolo_detect_img = img_ptr->image;
    for(auto bd_box : bd_boxes->bounding_boxes)
    {
        vector<Point3f> points;
        Point3f tmp;
        for(int i = 0; i < pcl_cloud->size(); i++)
        { 
            if(points_2d(0, i) < bd_box.xmax && points_2d(0, i) > bd_box.xmin 
            && points_2d(1, i) < bd_box.ymax && points_2d(1, i) > bd_box.ymin)
            {
                tmp.x = points_3d_in_cam(0, i);
                tmp.y = points_3d_in_cam(1, i);
                tmp.z = points_3d_in_cam(2, i);
                points.push_back(tmp);
            }
        }
        Mat labels;
        kmeans(points, 2,  labels, TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 10, 1.0), 3, KMEANS_PP_CENTERS);
        vector<Point3f> points_class[2];
        
        for(int i = 0; i < points.size(); i++ )
        {
            points_class[labels.at<int>(i)].push_back(points[i]);
        }
        int foreground_label = points_class[0][0].z < points_class[1][0].z ? 0 : 1;

        double box_x = 0, box_y = 0, box_z = 0;
        for(int i = 0; i < points_class[foreground_label].size(); i++)
        {
            pcl::PointXYZ tmp_pt;
            tmp_pt.x = points_class[foreground_label][i].x;
            tmp_pt.y = points_class[foreground_label][i].y;
            tmp_pt.z = points_class[foreground_label][i].z;
            out_cloud->push_back(tmp_pt);
            
            box_x += tmp_pt.x;
            box_y += tmp_pt.y;
            box_z += tmp_pt.z;
        }
        // label box with 3d coors
        box_x /= points_class[foreground_label].size();
        box_y /= points_class[foreground_label].size();
        box_z /= points_class[foreground_label].size();
        rectangle(yolo_detect_img, Rect(bd_box.xmin+5, bd_box.ymin+5, 400, 30), Scalar(255, 0, 0), -1);
        string coor_str;
        coor_str = to_string(box_x) +", " + to_string(box_y) + ", " + to_string(box_z);
        putText(yolo_detect_img, coor_str, Point(bd_box.xmin+5, bd_box.ymin + 25), cv::FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 255, 255));
    }

    sensor_msgs::PointCloud2 out_cloud_ros;
    pcl::toROSMsg(*out_cloud, out_cloud_ros);
    out_cloud_ros.header.frame_id = "world";
    out_cloud_ros.header.stamp = cropped_cloud->header.stamp;
    detect_result_cloud_pub_.publish(out_cloud_ros);

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", yolo_detect_img).toImageMsg();
    msg->header.stamp = yolo_img->header.stamp;
    enhanced_yolo_img_pub_.publish(msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "enhanced_yolo");
    ros::NodeHandle nh(""), nh_local("cam_lidar_fusion");
    EnhancedYOLO eyolo(nh, nh_local);

    // ROS_INFO("Init");
    ros::spin();
}
