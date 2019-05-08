/*
 * Created on Mon Oct 22 2018
 * Author: EpsAvlc
 * Note: Remove ground from the points cloud.
 *      Algrothm proposed by 
 *     《Fast segmentation of 3D point clouds: A paradigm on LiDAR data for autonomous vehicle applications》
 */

#ifndef IMU_LIDAR_FUSE_GROUND_TRANSFORM_H_
#define IMU_LIDAR_FUSE_GROUND_TRANSFORM_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/console/time.h>
#include <velodyne_pointcloud/point_types.h>
#include <Eigen/Core>

class GroundPlaneFitter
{
public:
    /** \brief constructor.
     */
    GroundPlaneFitter(ros::NodeHandle& nh, ros::NodeHandle& nh_local);
    ~GroundPlaneFitter();
private:
    typedef pcl::console::TicToc TicToc;
    /** \brief update parameters from ROS parameter server.
     */
    void updateParams();
    /** \brief message_filter's callback function.
     *  \param[in] lidar_data -- data published by velodyne lidar.
     */
    void callBack(const sensor_msgs::PointCloud2ConstPtr fused_cloud);
    /** \brief extract LPR seeds from plane.
     *  \param[in] in -- input point cloud.
     */
    void extractInitialSeeds(pcl::PointCloud<pcl::PointXYZI>& in_cloud);
    /** \brief estimate the ground plane model
     */
    void estimatePlane();
    /** \brief Clear all the vectors. 
     */
    inline void allClear(){
        point_ground_.clear();
        point_no_ground_.clear();
    }
    /** \brief Fuck the paper method.
     */
    void paperMethod(pcl::PointCloud<pcl::PointXYZI>& in_cloud);

    /** Class private variables.
     */ 
    ros::NodeHandle nh_;
    ros::NodeHandle nh_local_;
    ros::Subscriber sub_;
    ros::Publisher point_no_ground_pub_;
    ros::Publisher point_ground_pub_;
    ros::Publisher ground_height_pub_;
    Eigen::Vector3f norm_vector_; // norm_vector that describe the plane.
    double d_; // d that describe the plane. n'x + d = 0 => d = -n'x.
    pcl::PointCloud<pcl::PointXYZI> point_ground_;
    pcl::PointCloud<pcl::PointXYZI> point_no_ground_;
    TicToc tt_;
    /** ROS parameters. 
    */
    std::string topic_str_;
    int n_iter_; // number of iterations
    int n_lpr_;  // number of points used to estimate the LPR
    double thres_seeds_; // threshold for points to be considered initial seeds.
    double thres_dist_;  // threshold distance from the plane.
};

#endif // IMU_LIDAR_FUSE_GROUND_TRANSFORM_H_