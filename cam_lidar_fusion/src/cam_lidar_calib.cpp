/*
 * Created on Wed May 08 2019
 * Author: EpsAvlc
 */

#include "cam_lidar_calib.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

using namespace std;
using namespace cv;

CamLidarCalib::CamLidarCalib(ros::NodeHandle& nh, ros::NodeHandle& nh_local):nh_(nh), nh_local_(nh_local)
{
    readParam();
    cam_sub_ = nh_.subscribe(cam_topic_, 1, &CamLidarCalib::callback, this);

    calib_state_ = INITIAL;
    detector_ = xfeatures2d::SURF::create(5000);
    matcher_ = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
    camera_R_.setZero();
    camera_R_(0,2) = 1;
    camera_R_(1,0) = -1;
    camera_R_(2,1) = -1;
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
    allClear();
    cv_bridge::CvImagePtr img_ptr = cv_bridge::toCvCopy(image);
    cur_frame_ = img_ptr;
    if(calib_state_ == INITIAL)
    {
        if(last_frame_ == nullptr)
        {
            last_frame_ = cur_frame_;
            detector_->detectAndCompute(last_frame_->image, noArray(), last_keyPoints_, last_descriptors_);
            return;
        }
        featureDetectionAndMatching();
        cameraMotionEstimate();
    }
    last_frame_ = cur_frame_;
}

void CamLidarCalib::allClear()
{
    knn_matches_.clear();
    good_matches_.clear();
    last_match_points_.clear();
    cur_match_points_.clear();
}

void CamLidarCalib::featureDetectionAndMatching()
{
    detector_->detectAndCompute(cur_frame_->image, noArray(), cur_keyPoints_, cur_descriptors_);

    const float ratio_thresh = 0.5f;
    matcher_->knnMatch(last_descriptors_, cur_descriptors_, knn_matches_, 2);

    for (size_t i = 0; i < knn_matches_.size(); i++)
    {
        if (knn_matches_[i][0].distance < ratio_thresh * knn_matches_[i][1].distance)
        {
            good_matches_.push_back(knn_matches_[i][0]);
        }
    }

    Mat img_matches;
    if(good_matches_.size() > 0)
    {
        if(last_keyPoints_.size() > 0 && cur_keyPoints_.size() > 0)
        {
            /* drawMatches */
            // drawMatches(last_frame_->image, last_keyPoints_, 
            //     cur_frame_->image, cur_keyPoints_, good_matches_,
            //     img_matches, Scalar::all(-1),Scalar::all(-1), std::vector<char>(), 
            //     DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
            // //-- Show detected matches
            // imshow("Good Matches", img_matches );
            // waitKey(50);
        }
        else
        {
            ROS_ERROR("No KeyPoints.");
        }
    }
    else
    {
        ROS_ERROR("No Good Matchers.");
    }
    
    last_descriptors_ = cur_descriptors_;
    last_keyPoints_.swap(cur_keyPoints_);
}

void CamLidarCalib::cameraMotionEstimate()
{
    for(int i = 0; i < good_matches_.size(); i++)
    {
        last_match_points_.push_back(last_keyPoints_[good_matches_[i].trainIdx].pt);
        cur_match_points_.push_back(cur_keyPoints_[good_matches_[i].queryIdx].pt);
    }

    Mat fundamental;
    fundamental = findFundamentalMat(last_match_points_, cur_match_points_, CV_FM_8POINT);

    Mat essential;
    static Point2d principal_point(cam_intrins_(0,2), cam_intrins_(1,2));
    static double focal_length = cam_intrins_(0, 0);
    essential = findEssentialMat(last_match_points_, cur_match_points_, focal_length, principal_point, RANSAC);

    Mat R, t;
    recoverPose(essential, last_match_points_, cur_match_points_, R, t, focal_length, principal_point);
    Eigen::Matrix3d R_eigen;
    cv2eigen(R, R_eigen);
    camera_R_ = R_eigen * camera_R_;

    /* Disp in tf*/
    tf::Matrix3x3 R_tf;
    tf::matrixEigenToTF(camera_R_, R_tf);
    tf::Transform T(R_tf, tf::Vector3(1, 1, 0));
    static tf::TransformBroadcaster broadcaster;
    broadcaster.sendTransform(tf::StampedTransform(T, ros::Time::now(), "world", "base_link"));
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cam_lidar_calib");
    ros::NodeHandle nh(""), nh_local("");
    CamLidarCalib clc(nh, nh_local);
    ros::spin();
}
