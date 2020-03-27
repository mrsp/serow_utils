#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <laser_geometry/laser_geometry.h>
#include <vector>
using namespace std;

class serow_utils{
private:

        ros::NodeHandle n;
        bool isQuadruped, odom_inc;
        ros::Subscriber  laserScan_sub, support_sub, odom_sub, godom_sub, com_sub, gcom_sub, left_sub, right_sub, legodom_sub, compodom_sub,comlegodom_sub;
        ros::Publisher  pointcloudFromLaserScan_pub, support_path_pub,leg_odom_path_pub, com_path_pub, comleg_path_pub, ground_truth_odom_path_pub, ground_truth_com_path_pub, left_path_pub, 
        right_path_pub, comp_odom_path_pub, odom_path_pub, odom_pose_pub, left_pose_pub, right_pose_pub, comp_pose_pub;
    	nav_msgs::Path odom_path_msg, leg_odom_path_msg, com_path_msg, legcom_path_msg, support_path_msg, left_path_msg,right_path_msg, ground_truth_odom_path_msg, ground_truth_com_path_msg, comp_odom_path_msg;
        nav_msgs::Odometry odom_msg;
        ros::Publisher LF_path_pub, LH_path_pub, RF_path_pub, RH_path_pub, LF_pose_pub, LH_pose_pub, RF_pose_pub, RH_pose_pub;
        nav_msgs::Path LF_path_msg, LH_path_msg, RF_path_msg, RH_path_msg;
        ros::Subscriber LF_sub,  LH_sub, RF_sub, RH_sub;
        geometry_msgs::PoseStamped temp_pose;
        laser_geometry::LaserProjection projector_;

    void compodomCb(const nav_msgs::Odometry::ConstPtr& msg)
    {
        comp_odom_path_msg.header = msg->header;
        temp_pose.header = msg->header;
        temp_pose.pose = msg->pose.pose;
        comp_odom_path_msg.poses.push_back(temp_pose);
        comp_odom_path_pub.publish(comp_odom_path_msg);
        comp_pose_pub.publish(temp_pose);
    }


    void subscribe()
    {
        odom_sub = n.subscribe("serow/odom",10,&serow_utils::odomCb,this);
        odom_inc = false;
        compodom_sub = n.subscribe("serow/comp/odom0",10,&serow_utils::compodomCb,this);
        godom_sub = n.subscribe("serow/ground_truth/odom",10,&serow_utils::godomCb,this);
        com_sub = n.subscribe("serow/CoM/odom",10,&serow_utils::comCb,this);
        gcom_sub = n.subscribe("serow/ground_truth/CoM/odom",10,&serow_utils::gcomCb,this);
        legodom_sub = n.subscribe("serow/leg_odom",10,&serow_utils::legOdomCb,this);
        support_sub = n.subscribe("serow/support/pose",10,&serow_utils::supportCb,this);
        comlegodom_sub = n.subscribe("serow/CoM/leg_odom",10,&serow_utils::CoMlegOdomCb,this);
        laserScan_sub =  n.subscribe("scan",10,&serow_utils::laserCb,this);
        if(!isQuadruped){
            left_sub = n.subscribe("serow/LLeg/odom",10,&serow_utils::leftCb,this);
            right_sub = n.subscribe("serow/RLeg/odom",10,&serow_utils::rightCb,this);
        }
        else
        {
            LF_sub = n.subscribe("serow/LFLeg/odom",10,&serow_utils::LFCb,this);
            LH_sub = n.subscribe("serow/LHLeg/odom",10,&serow_utils::LHCb,this);
            RF_sub = n.subscribe("serow/RFLeg/odom",10,&serow_utils::RFCb,this);
            RH_sub = n.subscribe("serow/RHLeg/odom",10,&serow_utils::RHCb,this);
        }
        

    }
    void advertise()
    {



        //Shared Topics
        comp_odom_path_msg.poses.resize(2);
        ground_truth_odom_path_msg.poses.resize(2);
        ground_truth_com_path_msg.poses.resize(2);
		odom_path_msg.poses.resize(2);
		leg_odom_path_msg.poses.resize(2);
		com_path_msg.poses.resize(2);
        support_path_msg.poses.resize(2);


		odom_path_pub = n.advertise<nav_msgs::Path>("serow/odom/path",2);
		leg_odom_path_pub = n.advertise<nav_msgs::Path>("serow/leg_odom/path",2);
		com_path_pub = n.advertise<nav_msgs::Path>("serow/CoM/odom/path",2);
        ground_truth_odom_path_pub = n.advertise<nav_msgs::Path>("serow/ground_truth/odom/path",2);
        ground_truth_com_path_pub = n.advertise<nav_msgs::Path>("serow/ground_truth/CoM/odom/path",2);
        comp_odom_path_pub = n.advertise<nav_msgs::Path>("serow/comp/odom0/path",2);
	    comleg_path_pub = n.advertise<nav_msgs::Path>("serow/CoM/leg_odom/path",2);
	    support_path_pub= n.advertise<nav_msgs::Path>("serow/support/path",2);
	    comp_pose_pub = n.advertise<geometry_msgs::PoseStamped>("serow/comp/pose0",10);
        odom_pose_pub = n.advertise<geometry_msgs::PoseStamped>("serow/pose",2);
        pointcloudFromLaserScan_pub = n.advertise<sensor_msgs::PointCloud>("serow/pointcloud",10);

        //Humanoid/Biped Topics
        if(!isQuadruped)
        {
            left_path_msg.poses.resize(2);
            right_path_msg.poses.resize(2);
            left_path_pub = n.advertise<nav_msgs::Path>("serow/LLeg/path",2);
            right_path_pub = n.advertise<nav_msgs::Path>("serow/RLeg/path",2);
            left_pose_pub = n.advertise<geometry_msgs::PoseStamped>("serow/LLeg/pose",2);
            right_pose_pub = n.advertise<geometry_msgs::PoseStamped>("serow/RLeg/pose",2);
        }
        //Centauro/Quaruped Topics
        else
        {
            LF_path_msg.poses.resize(2);
            RF_path_msg.poses.resize(2);
            LH_path_msg.poses.resize(2);
            RH_path_msg.poses.resize(2);
            LF_path_pub = n.advertise<nav_msgs::Path>("serow/LFLeg/path",2);
            RF_path_pub = n.advertise<nav_msgs::Path>("serow/RFLeg/path",2);
            LH_path_pub = n.advertise<nav_msgs::Path>("serow/LHLeg/path",2);
            RH_path_pub = n.advertise<nav_msgs::Path>("serow/RHLeg/path",2);
            LF_pose_pub = n.advertise<geometry_msgs::PoseStamped>("serow/LFLeg/pose",2);
            RF_pose_pub = n.advertise<geometry_msgs::PoseStamped>("serow/RFLeg/pose",2);
            LH_pose_pub = n.advertise<geometry_msgs::PoseStamped>("serow/LHLeg/pose",2);
            RH_pose_pub = n.advertise<geometry_msgs::PoseStamped>("serow/RHLeg/pose",2);
        }
        

	

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
        odom_inc = true;
        odom_msg = *msg;
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


    void LFCb(const nav_msgs::Odometry::ConstPtr& msg){
            LF_path_msg.header = msg->header;
            temp_pose.header = msg->header;
            temp_pose.pose = msg->pose.pose;
            LF_path_msg.poses.push_back(temp_pose);
            LF_path_pub.publish(LF_path_msg);
            LF_pose_pub.publish(temp_pose);
    }

    void RFCb(const nav_msgs::Odometry::ConstPtr& msg){
            RF_path_msg.header = msg->header;
            temp_pose.header = msg->header;
            temp_pose.pose = msg->pose.pose;
            RF_path_msg.poses.push_back(temp_pose);
            RF_path_pub.publish(RF_path_msg);
            RF_pose_pub.publish(temp_pose);
    }

    void LHCb(const nav_msgs::Odometry::ConstPtr& msg){
            LH_path_msg.header = msg->header;
            temp_pose.header = msg->header;
            temp_pose.pose = msg->pose.pose;
            LH_path_msg.poses.push_back(temp_pose);
            LH_path_pub.publish(LH_path_msg);
            LH_pose_pub.publish(temp_pose);
    }

    void RHCb(const nav_msgs::Odometry::ConstPtr& msg){
            RH_path_msg.header = msg->header;
            temp_pose.header = msg->header;
            temp_pose.pose = msg->pose.pose;
            RH_path_msg.poses.push_back(temp_pose);
            RH_path_pub.publish(RH_path_msg);
            RH_pose_pub.publish(temp_pose);
    }

    void laserCb(const sensor_msgs::LaserScan::ConstPtr& scan_in)
    {
        if(odom_inc)
        {
        sensor_msgs::PointCloud cloud;
        tf::Transform bt = tf::Transform(tf::Quaternion(odom_msg.pose.pose.orientation.x,
                            odom_msg.pose.pose.orientation.y,
                            odom_msg.pose.pose.orientation.z,
                            odom_msg.pose.pose.orientation.w),
                 tf::Vector3(odom_msg.pose.pose.position.x,odom_msg.pose.pose.position.y,odom_msg.pose.pose.position.z));

        tf::StampedTransform bst =   tf::StampedTransform(bt, scan_in->header.stamp, odom_msg.header.frame_id, scan_in->header.frame_id);
        tf::Transformer btf;
        btf.setTransform(bst);
        projector_.transformLaserScanToPointCloud(odom_msg.header.frame_id,*scan_in,
          cloud,btf);

        pointcloudFromLaserScan_pub.publish(cloud);
        }
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
            ros::NodeHandle n_p("~");
            n_p.param<bool>("/serow/isQuadruped", isQuadruped, false);

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
