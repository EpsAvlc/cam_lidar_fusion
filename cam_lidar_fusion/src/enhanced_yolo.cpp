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
    yolo_sub_.subscribe(nh_, "/camera/image", 1);
    lidar_sub_.subscribe(nh_, "/cropped_cloud", 1);
    bd_box_sub_.subscribe(nh_, "/darknet_ros/bounding_boxes", 1);
    // because generate cropped cloud cost too much time, so we need to set buffer bigger = 50. 
    sync.reset(new Sync(enhanced_yolo_policy(50), yolo_sub_, lidar_sub_, bd_box_sub_));
    sync->registerCallback(boost::bind(&EnhancedYOLO::callback, this, _1, _2, _3));
    detect_result_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("detect_result_cloud", 1);
    enhanced_yolo_img_pub_ = nh_.advertise<sensor_msgs::Image>("enhanced_yolo_img", 1);

    area_thres_["person"] = pair<double, double>(1.8, 0.5);
    area_thres_["car"] = pair<double, double>(2.7, 1.8);
}

void EnhancedYOLO::readParam()
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
}

vector<Point3f> EnhancedYOLO::clusterPoints(vector<Point3f>& points)
{
    Mat labels;
    kmeans(points, 2,  labels, TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 10, 1.0), 3, KMEANS_PP_CENTERS);
    
    vector<Point3f> points_class[2];
    for(int i = 0; i < points.size(); i++ )
    {
        points_class[labels.at<int>(i)].push_back(points[i]);
    }
    int foreground_label = points_class[0][0].z < points_class[1][0].z ? 0 : 1;
    return points_class[foreground_label];
}

bool EnhancedYOLO::filterBboxByArea(const darknet_ros_msgs::BoundingBox& bbox, double range)
{
    int bbox_area = (bbox.xmax - bbox.xmin) * (bbox.ymax - bbox.ymin);
    string bbox_class = bbox.Class;
    Eigen::MatrixXd rect_corners(3, 4);
    rect_corners.block(0,0,3,1) << 0, 0, range;
    rect_corners.block(0,1,3,1) << area_thres_[bbox_class].first, 0, range;
    rect_corners.block(0,2,3,1) << area_thres_[bbox_class].first, area_thres_[bbox_class].second, range;
    rect_corners.block(0,3,3,1) << 0, area_thres_[bbox_class].second, range;
    
    Eigen::MatrixXd rect_corners_2d_homo = cam_intrins_ * rect_corners;
    vector<Point2d> rect_corners_2d;
    for(int i = 0; i < 4; i ++)
    {
        Point2d tmp;
        tmp.x = rect_corners_2d_homo(0, i) / rect_corners_2d_homo(2, i);
        tmp.y = rect_corners_2d_homo(1, i) / rect_corners_2d_homo(2, i);
        rect_corners_2d.push_back(tmp);
    }
    double width = rect_corners_2d[1].x - rect_corners_2d[0].x;
    double height = rect_corners_2d[2].y - rect_corners_2d[1].y;
    int hypo_area = static_cast<int>(width * height);
    // cout << bbox_area << ", " << hypo_area << endl;
    if(bbox_area < hypo_area * 0.5 || bbox_area > hypo_area * 1.5)
    {
        // cout << hypo_area * 0.5 << endl;
        return false;
    }
    return true;
}

void EnhancedYOLO::drawCube(Mat& img, Point3d min_xyz, Point3d max_xyz)
{
    Eigen::MatrixXd corners(3,8);
    double min_max_x[2] = {min_xyz.x, max_xyz.x};
    double min_max_y[2] = {min_xyz.y, max_xyz.y};
    double min_max_z[2] = {min_xyz.z, max_xyz.z};
    int corner_index = 0;
    for(int i = 0; i < 2; i++)
    {
        for(int j = 0; j < 2; j++)
        {
            for(int k = 0; k < 2; k++)
            {
                corners(0, corner_index) = min_max_x[i]; 
                corners(1, corner_index) = min_max_y[j];
                corners(2, corner_index) = min_max_z[k];    
                corner_index ++;      
            }
        }
    }
    Eigen::MatrixXd corners_2d_homo = cam_intrins_ * corners;
    vector<Point2d> corners_2d;
    for(int i = 0; i < 8; i++)
    {
        Point2d tmp;
        tmp.x = corners_2d_homo(0, i) / corners_2d_homo(2, i);
        tmp.y = corners_2d_homo(1, i) / corners_2d_homo(2, i);
        corners_2d.push_back(tmp);
    }
    line(img, corners_2d[0], corners_2d[1], Scalar(197,07,30), 2);
    line(img, corners_2d[0], corners_2d[2], Scalar(197,07,30), 2);
    line(img, corners_2d[0], corners_2d[4], Scalar(197,07,30), 2);
    line(img, corners_2d[1], corners_2d[3], Scalar(197,07,30), 2);
    line(img, corners_2d[1], corners_2d[5], Scalar(197,07,30), 2);
    line(img, corners_2d[2], corners_2d[3], Scalar(197,07,30), 2);
    line(img, corners_2d[2], corners_2d[6], Scalar(197,07,30), 2);
    line(img, corners_2d[3], corners_2d[7], Scalar(197,07,30), 2);
    line(img, corners_2d[4], corners_2d[5], Scalar(197,07,30), 2);
    line(img, corners_2d[4], corners_2d[6], Scalar(197,07,30), 2);
    line(img, corners_2d[5], corners_2d[7], Scalar(197,07,30), 2);
    line(img, corners_2d[6], corners_2d[7], Scalar(197,07,30), 2);
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
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    cv_bridge::CvImagePtr img_ptr = cv_bridge::toCvCopy(yolo_img);
    Mat src_img = img_ptr->image;
    if(src_img.channels() == 1)
        cvtColor(src_img, src_img, CV_GRAY2BGR);
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
        if(points.size() < 2)
        {
            continue;
        }
        
        vector<Point3f> points_fg = this->clusterPoints(points);

        Point3d min_xyz(10000, 10000, 10000), max_xyz(-10000, -10000, -10000);
        for(int i = 0; i < points_fg.size(); i++)
        {
            pcl::PointXYZ tmp_pt;
            tmp_pt.x = points_fg[i].x;
            tmp_pt.y = points_fg[i].y;
            tmp_pt.z = points_fg[i].z;
            out_cloud->push_back(tmp_pt);
            
            if(min_xyz.x > tmp_pt.x)
                min_xyz.x = tmp_pt.x;
            if(min_xyz.y > tmp_pt.y)
                min_xyz.y = tmp_pt.y;
            if(min_xyz.z > tmp_pt.z)
                min_xyz.z = tmp_pt.z;

            if(max_xyz.x < tmp_pt.x)
                max_xyz.x = tmp_pt.x;
            if(max_xyz.y < tmp_pt.y)
                max_xyz.y = tmp_pt.y;
            if(max_xyz.z < tmp_pt.z)
                max_xyz.z = tmp_pt.z;
        }
        if(! this->filterBboxByArea(bd_box, (min_xyz.z + max_xyz.z) / 2) || max_xyz.z - min_xyz.z >5)
        {
            rectangle(src_img, Point(bd_box.xmin, bd_box.ymin), Point(bd_box.xmax, bd_box.ymax), Scalar(30,07,197), 2);
            putText(src_img, "fake", Point(bd_box.xmin+5, bd_box.ymin + 25), cv::FONT_HERSHEY_TRIPLEX, 1, Scalar(0, 255, 255));
        }
        else
        {
            this->drawCube(src_img, min_xyz, max_xyz);
            putText(src_img, bd_box.Class, Point(bd_box.xmin+5, bd_box.ymin + 25), cv::FONT_HERSHEY_TRIPLEX, 1, Scalar(0, 255, 255));
        }
    }
    imshow("disp", src_img);
    imwrite("//home//cm//enhanced_yolo.png", src_img);
    waitKey(5);

    sensor_msgs::PointCloud2 out_cloud_ros;
    pcl::toROSMsg(*out_cloud, out_cloud_ros);
    out_cloud_ros.header.frame_id = "world";
    out_cloud_ros.header.stamp = cropped_cloud->header.stamp;
    detect_result_cloud_pub_.publish(out_cloud_ros);

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", src_img).toImageMsg();
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
