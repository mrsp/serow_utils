#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <ros/ros.h>
#include <vector>
using namespace std;

class serow_utils{
private:

        ros::NodeHandle n;
        ros::Subscriber  support_sub, odom_sub, godom_sub, com_sub, gcom_sub, left_sub, right_sub, legodom_sub, compodom_sub,comlegodom_sub;
        ros::Publisher  support_path_pub,leg_odom_path_pub, com_path_pub, comleg_path_pub, ground_truth_odom_path_pub, ground_truth_com_path_pub, left_path_pub, 
        right_path_pub, comp_odom_path_pub, odom_path_pub, odom_pose_pub, left_pose_pub, right_pose_pub;
    	nav_msgs::Path odom_path_msg, leg_odom_path_msg, com_path_msg, legcom_path_msg, support_path_msg, left_path_msg,right_path_msg, ground_truth_odom_path_msg, ground_truth_com_path_msg, comp_odom_path_msg;

        geometry_msgs::PoseStamped temp_pose;


    void compodomCb(const nav_msgs::Odometry::ConstPtr& msg)
    {
        comp_odom_path_msg.header = msg->header;
        temp_pose.header = msg->header;
        temp_pose.pose = msg->pose.pose;
		comp_odom_path_msg.poses.push_back(temp_pose);
		comp_odom_path_pub.publish(comp_odom_path_msg);
    }


    void subscribe()
    {
        odom_sub = n.subscribe("SERoW/odom",10,&serow_utils::odomCb,this);
        compodom_sub = n.subscribe("/SERoW/comp/odom",10,&serow_utils::compodomCb,this);
        godom_sub = n.subscribe("/SERoW/ground_truth/odom",10,&serow_utils::godomCb,this);
        com_sub = n.subscribe("SERoW/CoM/odom",10,&serow_utils::comCb,this);
        gcom_sub = n.subscribe("/SERoW/ground_truth/CoM/odom",10,&serow_utils::gcomCb,this);
        left_sub = n.subscribe("/SERoW/LLeg/odom",10,&serow_utils::leftCb,this);
        right_sub = n.subscribe("/SERoW/RLeg/odom",10,&serow_utils::rightCb,this);
        legodom_sub = n.subscribe("/SERoW/leg_odom",10,&serow_utils::legOdomCb,this);
        support_sub = n.subscribe("/SERoW/support/pose",10,&serow_utils::supportCb,this);
        comlegodom_sub = n.subscribe("/SERoW/CoM/leg_odom",10,&serow_utils::CoMlegOdomCb,this);
    }
    void advertise()
    {

        comp_odom_path_msg.poses.resize(2);
        ground_truth_odom_path_msg.poses.resize(2);
        ground_truth_com_path_msg.poses.resize(2);



		left_path_msg.poses.resize(2);
		right_path_msg.poses.resize(2);
		odom_path_msg.poses.resize(2);
		leg_odom_path_msg.poses.resize(2);
		com_path_msg.poses.resize(2);
        support_path_msg.poses.resize(2);

	    support_path_pub= n.advertise<nav_msgs::Path>("/SERoW/support/path",2);
		left_path_pub = n.advertise<nav_msgs::Path>("/SERoW/LLeg/path",2);
		right_path_pub = n.advertise<nav_msgs::Path>("/SERoW/RLeg/path",2);
		odom_path_pub = n.advertise<nav_msgs::Path>("/SERoW/odom/path",2);
		leg_odom_path_pub = n.advertise<nav_msgs::Path>("/SERoW/leg_odom/path",2);
		com_path_pub = n.advertise<nav_msgs::Path>("/SERoW/CoM/odom/path",2);
        ground_truth_odom_path_pub = n.advertise<nav_msgs::Path>("/SERoW/ground_truth/odom/path",2);
        ground_truth_com_path_pub = n.advertise<nav_msgs::Path>("/SERoW/ground_truth/CoM/odom/path",2);
        comp_odom_path_pub = n.advertise<nav_msgs::Path>("/SERoW/comp/odom/path",2);
	    comleg_path_pub = n.advertise<nav_msgs::Path>("/SERoW/CoM/leg_odom/path",2);
        odom_pose_pub = n.advertise<geometry_msgs::PoseStamped>("/SERoW/pose",2);
        left_pose_pub = n.advertise<geometry_msgs::PoseStamped>("/SERoW/LLeg/pose",2);
        right_pose_pub = n.advertise<geometry_msgs::PoseStamped>("/SERoW/RLeg/pose",2);
    }
    void supportCb(const geometry_msgs::PoseStamped::ConstPtr& msg){
            support_path_msg.header = msg->header;
            temp_pose.header = msg->header;
            temp_pose.pose = msg->pose;
            support_path_msg.poses.push_back(temp_pose);
            support_path_pub.publish(support_path_msg);
    }

    void odomCb(const nav_msgs::Odometry::ConstPtr& msg)
    {
        odom_path_msg.header = msg->header;
        temp_pose.header = msg->header;
        temp_pose.pose = msg->pose.pose;
		odom_path_msg.poses.push_back(temp_pose);
		odom_path_pub.publish(odom_path_msg);

        odom_pose_pub.publish(temp_pose);
    }


    void legOdomCb(const nav_msgs::Odometry::ConstPtr& msg)
    {
		leg_odom_path_msg.header = msg->header;
        temp_pose.header = msg->header;
        temp_pose.pose = msg->pose.pose;
		leg_odom_path_msg.poses.push_back(temp_pose);
		leg_odom_path_pub.publish(leg_odom_path_msg);
    }


    void leftCb(const nav_msgs::Odometry::ConstPtr& msg){
            left_path_msg.header = msg->header;
            temp_pose.header = msg->header;
            temp_pose.pose = msg->pose.pose;
            left_path_msg.poses.push_back(temp_pose);
            left_path_pub.publish(left_path_msg);
            left_pose_pub.publish(temp_pose);
    }

    void rightCb(const nav_msgs::Odometry::ConstPtr& msg){
            right_path_msg.header = msg->header;
            temp_pose.header = msg->header;
            temp_pose.pose = msg->pose.pose;
            right_path_msg.poses.push_back(temp_pose);
            right_path_pub.publish(right_path_msg);
            right_pose_pub.publish(temp_pose);
    }


    void comCb(const nav_msgs::Odometry::ConstPtr& msg){
            com_path_msg.header =  msg->header;
            temp_pose.header = msg->header;
            temp_pose.pose = msg->pose.pose;
            com_path_msg.poses.push_back(temp_pose);
            com_path_pub.publish(com_path_msg);
    }

    void CoMlegOdomCb(const nav_msgs::Odometry::ConstPtr& msg){
            legcom_path_msg.header =  msg->header;
            temp_pose.header = msg->header;
            temp_pose.pose = msg->pose.pose;
            legcom_path_msg.poses.push_back(temp_pose);
            comleg_path_pub.publish(legcom_path_msg);
    }
    void gcomCb(const nav_msgs::Odometry::ConstPtr& msg){
            ground_truth_com_path_msg.header =  msg->header;
            temp_pose.header = msg->header;
            temp_pose.pose = msg->pose.pose;
            ground_truth_com_path_msg.poses.push_back(temp_pose);
            ground_truth_com_path_pub.publish(ground_truth_com_path_msg);
    }


    void godomCb(const nav_msgs::Odometry::ConstPtr& msg){
            ground_truth_odom_path_msg.header =  msg->header;
            temp_pose.header = msg->header;
            temp_pose.pose = msg->pose.pose;
            ground_truth_odom_path_msg.poses.push_back(temp_pose);
            ground_truth_odom_path_pub.publish(ground_truth_odom_path_msg);
    }


    public:
        void connect(const ros::NodeHandle nh){
            n=nh;
            subscribe();
            advertise();
        }


};







int main( int argc, char** argv )
{
    ros::init(argc, argv, "serow_utils");
    ros::NodeHandle n;
    if(!ros::master::check())
    {
        cerr<<"Could not contact master!\nQuitting... "<<endl;
        return -1;
    }

    serow_utils* su = new serow_utils();
    su->connect(n);

    ros::spin();

}
