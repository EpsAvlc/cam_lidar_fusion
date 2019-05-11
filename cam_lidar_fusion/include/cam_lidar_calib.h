/*
 * Created on Wed May 08 2019
 * Author: EpsAvlc
 */

#ifndef CAM_LIDAR_CALIB_H__
#define CAM_LIDAR_CALIB_H__

#include <ros/ros.h>
#include <Eigen/Core>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>

class CamLidarCalib
{
public:
    CamLidarCalib(ros::NodeHandle& nh, ros::NodeHandle& nh_local);
private:
    void readParam();
    void callback(const sensor_msgs::ImageConstPtr& image);
    void featureDetectionAndMatching();
    void cameraMotionEstimate();
    void allClear();

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
    cv_bridge::CvImagePtr last_frame_, cur_frame_;
    std::vector<cv::KeyPoint> last_keyPoints_, cur_keyPoints_;
    cv::Mat last_descriptors_, cur_descriptors_;

    std::vector<std::vector<cv::DMatch> > knn_matches_;
    std::vector<cv::DMatch> good_matches_;
    cv::Ptr<cv::xfeatures2d::SURF> detector_;
    cv::Ptr<cv::DescriptorMatcher> matcher_;
    std::vector<cv::Point2f> last_match_points_, cur_match_points_;

    Eigen::Matrix3d camera_R_;
    
};

#endif