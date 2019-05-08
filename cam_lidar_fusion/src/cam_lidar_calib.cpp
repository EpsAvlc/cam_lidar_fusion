/*
 * Created on Wed May 08 2019
 * Author: EpsAvlc
 */

#include "cam_lidar_calib.h"

using namespace std;

CamLidarCalib::CamLidarCalib(ros::NodeHandle& nh, ros::NodeHandle& nh_local):nh_(nh), nh_local_(nh_local)
{
    readParam();
    cam_sub_ = nh_.subscribe(cam_topic_, 1, &CamLidarCalib::callback, this);
}

void CamLidarCalib::readParam()
{
    vector<double> cam_intrins_data;
    nh_local_.getParam("cam_intrins", cam_intrins_data);
    for(int i = 0; i < cam_intrins_data.size(); i++)
    {
        cam_intrins_(i) = cam_intrins_data[i];
    }
    cam_intrins_.transposeInPlace();

    cout << "Camera intrins: "<< endl << cam_intrins_ << endl;
    nh_local_.getParam("cam_topic", cam_topic_);
    cout << "Subscribe to image topic: " << cam_topic_ << endl;
}

void CamLidarCalib::callback(const sensor_msgs::ImageConstPtr& image)
{
    if(calib_state_ == INITIAL)
    {
        
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cam_lidar_calib");
    ros::NodeHandle nh(""), nh_local("");
    CamLidarCalib clc(nh, nh_local);
    ros::spin();
}
