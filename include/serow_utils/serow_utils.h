#ifndef __SEROW_UTILS_H__
#define __SEROW_UTILS_H__
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <laser_geometry/laser_geometry.h>
#include <vector>
using namespace std;

class serow_utils
{
private:
    ros::NodeHandle n;
    bool isQuadruped, odom_inc;
    ros::Subscriber laserScan_sub, support_sub, odom_sub, godom_sub, com_sub, gcom_sub, left_sub, right_sub, legodom_sub, compodom_sub, comlegodom_sub;
    ros::Publisher pointcloudFromLaserScan_pub, support_path_pub, leg_odom_path_pub, com_path_pub, comleg_path_pub, ground_truth_odom_path_pub, ground_truth_com_path_pub, left_path_pub,
        right_path_pub, comp_odom_path_pub, odom_path_pub, odom_pose_pub, left_pose_pub, right_pose_pub, comp_pose_pub;
    nav_msgs::Path odom_path_msg, leg_odom_path_msg, com_path_msg, legcom_path_msg, support_path_msg, left_path_msg, right_path_msg, ground_truth_odom_path_msg, ground_truth_com_path_msg, comp_odom_path_msg;
    nav_msgs::Odometry odom_msg;
    ros::Publisher LF_path_pub, LH_path_pub, RF_path_pub, RH_path_pub, LF_pose_pub, LH_pose_pub, RF_pose_pub, RH_pose_pub;
    nav_msgs::Path LF_path_msg, LH_path_msg, RF_path_msg, RH_path_msg;
    ros::Subscriber LF_sub, LH_sub, RF_sub, RH_sub;
    laser_geometry::LaserProjection projector_;

    int publish_buffer_size, subscribe_buffer_size, vector_size;
    void compodomCb(const nav_msgs::Odometry::ConstPtr &msg);
    void subscribe();
    void advertise();
    void supportCb(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void odomCb(const nav_msgs::Odometry::ConstPtr &msg);
    void legOdomCb(const nav_msgs::Odometry::ConstPtr &msg);
    void leftCb(const nav_msgs::Odometry::ConstPtr &msg);
    void rightCb(const nav_msgs::Odometry::ConstPtr &msg);
    void LFCb(const nav_msgs::Odometry::ConstPtr &msg);
    void RFCb(const nav_msgs::Odometry::ConstPtr &msg);
    void LHCb(const nav_msgs::Odometry::ConstPtr &msg);
    void RHCb(const nav_msgs::Odometry::ConstPtr &msg);
    void laserCb(const sensor_msgs::LaserScan::ConstPtr &scan_in);
    void comCb(const nav_msgs::Odometry::ConstPtr &msg);
    void CoMlegOdomCb(const nav_msgs::Odometry::ConstPtr &msg);
    void gcomCb(const nav_msgs::Odometry::ConstPtr &msg);
    void godomCb(const nav_msgs::Odometry::ConstPtr &msg);

public:
    void connect(const ros::NodeHandle nh);
    void run();
};
#endif