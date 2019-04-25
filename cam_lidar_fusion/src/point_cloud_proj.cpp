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
#include <pcl/io/pcd_io.h>
#include <pcl/console/time.h>
#include <pcl/filters/passthrough.h>
// #include "pointXYZPixel.h"
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>

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
    projected_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("cropped_cloud", 1);
}

void PointCloudProj::readParam()
{
    vector<double> cam_intrins_data;
    nh_local_.getParam("cam_intrins", cam_intrins_data);
    ROS_ASSERT(cam_intrins_data.size() == 9);
    for(int i = 0; i < cam_intrins_data.size(); i++)
    {
        cam_intrins_(i) = cam_intrins_data[i];
    }
    /* Note that the order of data in matrix in ros is diffent from eigen*/
    cam_intrins_.transposeInPlace();
    
    vector<double> xyzrpy_init;
    if(nh_local_.getParam("xyzrpy_init", xyzrpy_init))
    {
        Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(xyzrpy_init[3], Eigen::Vector3d::UnitX()));
        Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(xyzrpy_init[4], Eigen::Vector3d::UnitY()));
        Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(xyzrpy_init[5], Eigen::Vector3d::UnitZ()));
        pose_init_.block(0, 0, 3, 3) = (yawAngle*pitchAngle*rollAngle).matrix();
        pose_init_(0, 3) = xyzrpy_init[0];
        pose_init_(1, 3) = xyzrpy_init[1];
        pose_init_(2, 3) = xyzrpy_init[2];
        pose_init_(3, 3) = 1;
    }

    vector<double> xyzrpy;
    if(nh_local_.getParam("xyzrpy", xyzrpy))
    {
        Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(xyzrpy[3], Eigen::Vector3d::UnitX()));
        Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(xyzrpy[4], Eigen::Vector3d::UnitY()));
        Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(xyzrpy[5], Eigen::Vector3d::UnitZ()));
        lidar_to_cam_.block(0, 0, 3, 3) = (yawAngle*pitchAngle*rollAngle).matrix();
        lidar_to_cam_(0, 3) = xyzrpy[0];
        lidar_to_cam_(1, 3) = xyzrpy[1];
        lidar_to_cam_(2, 3) = xyzrpy[2];
        lidar_to_cam_(3, 3) = 1;
    }
    else
    {
        vector<double> lidar2cam_data;
        nh_local_.getParam("/cam_lidar_fusion/lidar2cam", lidar2cam_data);
        ROS_ASSERT(lidar2cam_data.size() == 16);
        for(int i = 0; i < lidar2cam_data.size(); i++)
        {
            lidar_to_cam_(i) = lidar2cam_data[i];
        }
        lidar_to_cam_.transposeInPlace();
    }

    nh_local_.getParam("cam_topic", cam_topic_);
    nh_local_.getParam("lidar_topic", lidar_topic_);
}

void PointCloudProj::callback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::PointCloud2ConstPtr& cloud)
{
    // ROS_INFO("FUCK!!");
    pcl::console::TicToc tt;
    tt.tic();

    /* Projection */
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud, *pcl_cloud);
    {
        /* filter */
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud (pcl_cloud);
        pass.setFilterFieldName ("x");
        pass.setFilterLimits (0.0, 100);
        pass.filter (*pcl_cloud);

        pass.setFilterFieldName ("z");
        pass.setFilterLimits (-0.8, 2);
        pass.filter (*pcl_cloud);
    } 
    Eigen::MatrixXd points_3d_homo(4, pcl_cloud->size());

    for(int i = 0; i < pcl_cloud->size(); i++)
    {
        points_3d_homo(0, i) = (*pcl_cloud)[i].x;
        points_3d_homo(1, i) = (*pcl_cloud)[i].y;
        points_3d_homo(2, i) = (*pcl_cloud)[i].z;
        points_3d_homo(3, i) = 1;
    }

    Eigen::MatrixXd points_3d_in_cam_homo = lidar_to_cam_ * pose_init_ *  points_3d_homo;

    // cout << "Final transform: " << endl << lidar_to_cam_ * pose_init_ << endl;

    // broad cast tf
    // static tf::TransformBroadcaster br;
    // tf::Transform trans;
    // Eigen::Isometry3d lidar_to_cam_aff;
    // lidar_to_cam_aff.matrix() = lidar_to_cam_ * pose_init_;
    // tf::transformEigenToTF(lidar_to_cam_aff, trans);
    // br.sendTransform(tf::StampedTransform(trans, ros::Time::now(), "camera", "lidar"));


    Eigen::MatrixXd points_3d_in_cam = points_3d_in_cam_homo.block(0, 0, 3, pcl_cloud->size());
    Eigen::MatrixXd points_2d_homo = cam_intrins_ * points_3d_in_cam;

    /* Draw result */
    cv_bridge::CvImagePtr img_ptr = cv_bridge::toCvCopy(image);
    /* Save pc and img*/
    // {
    //     static int count = 0;
    //     imwrite("/home/cm/Workspaces/cam_lidar_fusion/src/cam_lidar_fusion/bags/" + to_string(count) + ".jpg", img_ptr->image);
    //     pcl::io::savePCDFileASCII("/home/cm/Workspaces/cam_lidar_fusion/src/cam_lidar_fusion/bags/" + to_string(count) + ".pcd", pcl_cloud);
    //     count ++;
    // }

    // cvtColor(img_ptr->image, img_ptr->image, COLOR_GRAY2RGB);

    // for(int i = 0; i < pcl_cloud->size(); i++)
    // {
    //     int x = points_2d_homo(0, i) / points_2d_homo(2, i);
    //     int y = points_2d_homo(1, i) / points_2d_homo(2, i);
    //     if(x < 0 || x > img_ptr->image.cols ||
    //        y < 0 || y > img_ptr->image.rows || points_2d_homo(2, i) < 0)
    //         continue;
    //     Scalar color_val(saturate_cast<uchar>(points_3d_in_cam(2, i) / 20 * 255), saturate_cast<uchar>(points_3d_in_cam(2, i) / 10 * 255), saturate_cast<uchar>(points_3d_in_cam(2, i) / 5 * 255));
    //     Scalar color(color_val);
    //     circle(img_ptr->image, Point(x, y), 2, color_val, -1);
    // }
    // imshow("after", img_ptr->image);
    // waitKey(5);

    /* Publish the filtered cloud which are in camera view.*/
    pcl::PointCloud<pcl::PointXYZ> out_cloud;
    pcl::PointXYZ tmp;

    for(int i = 0; i < pcl_cloud->size(); i++)
    {
        int pixel_x = points_2d_homo(0, i) / points_2d_homo(2, i);
        int pixel_y = points_2d_homo(1, i) / points_2d_homo(2, i);

        if(pixel_x < 0 || pixel_x > img_ptr->image.cols ||
           pixel_y < 0 || pixel_y > img_ptr->image.rows)
           continue;
        
        tmp.x = points_3d_in_cam_homo(0, i);
        tmp.y = points_3d_in_cam_homo(1, i);
        tmp.z = points_3d_in_cam_homo(2, i);
        out_cloud.push_back(tmp);
    }
    cout << out_cloud.size() << endl;
    sensor_msgs::PointCloud2 out_cloud_ros;
    pcl::toROSMsg(out_cloud, out_cloud_ros);
    out_cloud_ros.header.frame_id = "world";
    out_cloud_ros.header.stamp = cloud->header.stamp;
    
    projected_cloud_pub.publish(out_cloud_ros);

    // cout << "Project cost time : " << tt.toc()/1000 << "s" << endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "point_cloud_proj");
    ros::NodeHandle nh(""), nh_local("cam_lidar_fusion");
    PointCloudProj pcp(nh, nh_local);
    ros::spin();
}