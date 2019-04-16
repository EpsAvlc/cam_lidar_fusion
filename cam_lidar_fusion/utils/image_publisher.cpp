#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
//   image_transport::ImageTransport it(nh);
  ros::Publisher pub = nh.advertise<sensor_msgs::Image>("camera/image", 1);
  cv::Mat image = cv::imread(argv[1], CV_LOAD_IMAGE_GRAYSCALE);
  cv::waitKey(30);
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", image).toImageMsg();
  

  ros::Rate loop_rate(5);
  while (nh.ok()) {
    msg->header.stamp = ros::Time::now();
    pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
}