/*
 * Created on Wed May 08 2019
 * Author: EpsAvlc
 */

#ifndef CAM_LIDAR_CALIB_H__
#define CAM_LIDAR_CALIB_H__

#include <ros/ros.h>
#include <Eigen/Core>
#include <sensor_msgs/Image.h>
#include <opencv2/core/core.hpp>

class CamLidarCalib
{
public:
    CamLidarCalib(ros::NodeHandle& nh, ros::NodeHandle& nh_local);
private:
    void readParam();
    void callback(const sensor_msgs::ImageConstPtr& image);
    enum CalibState{
        INITIAL,
        REFINE,
    };
    /* ROS parameters. */
    ros::NodeHandle nh_, nh_local_;
    std::string cam_topic_;
    ros::Subscriber cam_sub_;
    /* Class variables. */
    Eigen::Matrix3d cam_intrins_;
    CalibState calib_state_;

    /** Initial stage variables. **/
    cv::Mat last_frame_, cur_frame_;
};

#endif