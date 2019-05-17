#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<sensor_msgs::Image>("/kitti/camera_color_left/image_raw", 1);
  cv::Mat image = cv::imread(argv[1], CV_LOAD_IMAGE_GRAYSCALE);
  // ROS_INFO("image size: %d", image.cols);
  // cv::imshow("src", image);
  // cv::waitKey(30);
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", image).toImageMsg();
  ros::Rate loop_rate(5);
  while (ros::ok()) {
    msg->header.stamp = ros::Time::now();
    pub.publish(msg);
    ros::spinOnce();
    // ROS_INFO("send once");
    loop_rate.sleep();
  }
}