#include <serow_utils/serow_utils.h>

void serow_utils::compodomCb(const nav_msgs::Odometry::ConstPtr &msg)
{
    if(comp_odom_path_msg.poses.size()>vector_size)
        comp_odom_path_msg.poses.clear();

    geometry_msgs::PoseStamped temp_pose;
    comp_odom_path_msg.header = msg->header;
    temp_pose.header = msg->header;
    temp_pose.pose = msg->pose.pose;
    comp_odom_path_msg.poses.push_back(temp_pose);
    comp_odom_path_pub.publish(comp_odom_path_msg);
    comp_pose_pub.publish(temp_pose);
}

void serow_utils::subscribe()
{
    publish_buffer_size = 1000;
    subscribe_buffer_size = 1000;
    vector_size = 500;
    odom_sub = n.subscribe("serow/base/odom", subscribe_buffer_size, &serow_utils::odomCb, this);
    compodom_sub = n.subscribe("serow/comp/base/odom0", subscribe_buffer_size, &serow_utils::compodomCb, this);
    godom_sub = n.subscribe("serow/ground_truth/base/odom", subscribe_buffer_size, &serow_utils::godomCb, this);
    com_sub = n.subscribe("serow/CoM/odom", subscribe_buffer_size, &serow_utils::comCb, this);
    gcom_sub = n.subscribe("serow/ground_truth/CoM/odom", subscribe_buffer_size, &serow_utils::gcomCb, this);
    legodom_sub = n.subscribe("serow/base/leg_odom", subscribe_buffer_size, &serow_utils::legOdomCb, this);
    support_sub = n.subscribe("serow/support/pose", subscribe_buffer_size, &serow_utils::supportCb, this);
    comlegodom_sub = n.subscribe("serow/CoM/leg_odom", subscribe_buffer_size, &serow_utils::CoMlegOdomCb, this);
    laserScan_sub = n.subscribe("scan", subscribe_buffer_size, &serow_utils::laserCb, this);

    if (!isQuadruped)
    {
        left_sub = n.subscribe("serow/LLeg/odom", subscribe_buffer_size, &serow_utils::leftCb, this);
        right_sub = n.subscribe("serow/RLeg/odom", subscribe_buffer_size, &serow_utils::rightCb, this);
    }
    else
    {
        LF_sub = n.subscribe("serow/LFLeg/odom", subscribe_buffer_size, &serow_utils::LFCb, this);
        LH_sub = n.subscribe("serow/LHLeg/odom", subscribe_buffer_size, &serow_utils::LHCb, this);
        RF_sub = n.subscribe("serow/RFLeg/odom", subscribe_buffer_size, &serow_utils::RFCb, this);
        RH_sub = n.subscribe("serow/RHLeg/odom", subscribe_buffer_size, &serow_utils::RHCb, this);
    }
}
void serow_utils::advertise()
{

    //Shared Topics
    odom_path_pub = n.advertise<nav_msgs::Path>("serow/base/odom/path", publish_buffer_size);
    leg_odom_path_pub = n.advertise<nav_msgs::Path>("serow/base/leg_odom/path", publish_buffer_size);
    com_path_pub = n.advertise<nav_msgs::Path>("serow/CoM/odom/path", publish_buffer_size);
    ground_truth_odom_path_pub = n.advertise<nav_msgs::Path>("serow/ground_truth/base/odom/path", publish_buffer_size);
    ground_truth_com_path_pub = n.advertise<nav_msgs::Path>("serow/ground_truth/CoM/odom/path", publish_buffer_size);
    comp_odom_path_pub = n.advertise<nav_msgs::Path>("serow/comp/base/odom0/path", publish_buffer_size);
    comleg_path_pub = n.advertise<nav_msgs::Path>("serow/CoM/leg_odom/path", publish_buffer_size);
    support_path_pub = n.advertise<nav_msgs::Path>("serow/support/path", publish_buffer_size);
    comp_pose_pub = n.advertise<geometry_msgs::PoseStamped>("serow/comp/base/pose0", publish_buffer_size);
    odom_pose_pub = n.advertise<geometry_msgs::PoseStamped>("serow/base/pose", publish_buffer_size);
    pointcloudFromLaserScan_pub = n.advertise<sensor_msgs::PointCloud>("serow/pointcloud", publish_buffer_size);

    //Humanoid/Biped Topics
    if (!isQuadruped)
    {
        left_path_pub = n.advertise<nav_msgs::Path>("serow/LLeg/path", publish_buffer_size);
        right_path_pub = n.advertise<nav_msgs::Path>("serow/RLeg/path", publish_buffer_size);
        left_pose_pub = n.advertise<geometry_msgs::PoseStamped>("serow/LLeg/pose", publish_buffer_size);
        right_pose_pub = n.advertise<geometry_msgs::PoseStamped>("serow/RLeg/pose", publish_buffer_size);
    }
    //Centauro/Quaruped Topics
    else
    {
        LF_path_pub = n.advertise<nav_msgs::Path>("serow/LFLeg/path", publish_buffer_size);
        RF_path_pub = n.advertise<nav_msgs::Path>("serow/RFLeg/path", publish_buffer_size);
        LH_path_pub = n.advertise<nav_msgs::Path>("serow/LHLeg/path", publish_buffer_size);
        RH_path_pub = n.advertise<nav_msgs::Path>("serow/RHLeg/path", publish_buffer_size);
        LF_pose_pub = n.advertise<geometry_msgs::PoseStamped>("serow/LFLeg/pose", publish_buffer_size);
        RF_pose_pub = n.advertise<geometry_msgs::PoseStamped>("serow/RFLeg/pose", publish_buffer_size);
        LH_pose_pub = n.advertise<geometry_msgs::PoseStamped>("serow/LHLeg/pose", publish_buffer_size);
        RH_pose_pub = n.advertise<geometry_msgs::PoseStamped>("serow/RHLeg/pose", publish_buffer_size);
    }
}

void serow_utils::publish()
{
    if(support_pose_inc)
    {
        if(support_path_msg.poses.size()>vector_size)
            support_path_msg.poses.clear();

        support_path_msg.header = support_pose_msg.header;
        support_path_msg.poses.push_back(support_pose_msg);
        support_path_pub.publish(support_path_msg);
        support_pose_inc = false;
    }

    if(odom_inc)
    {
        if(odom_path_msg.poses.size()>vector_size)
            odom_path_msg.poses.clear();

        geometry_msgs::PoseStamped temp_pose;
        odom_path_msg.header = odom_msg.header;
        temp_pose.header = odom_msg.header;
        temp_pose.pose = odom_msg.pose.pose;
        odom_path_msg.poses.push_back(temp_pose);
        odom_path_pub.publish(odom_path_msg);
        odom_pose_pub.publish(temp_pose);
        odom_inc = false;
    }
    if(leg_odom_inc)
    {
         if(leg_odom_path_msg.poses.size()>vector_size)
            leg_odom_path_msg.poses.clear();

        geometry_msgs::PoseStamped temp_pose;
        leg_odom_path_msg.header = leg_odom_msg.header;
        temp_pose.header = leg_odom_msg.header;
        temp_pose.pose = leg_odom_msg.pose.pose;
        leg_odom_path_msg.poses.push_back(temp_pose);
        leg_odom_path_pub.publish(leg_odom_path_msg);
        leg_odom_inc = false;
    }

    if(LLeg_inc)
    {
        if(left_path_msg.poses.size()>vector_size)
            left_path_msg.poses.clear();

        geometry_msgs::PoseStamped temp_pose;
        left_path_msg.header = LLeg_odom_msg.header;
        temp_pose.header = LLeg_odom_msg.header;
        temp_pose.pose = LLeg_odom_msg.pose.pose;
        left_path_msg.poses.push_back(temp_pose);
        left_path_pub.publish(left_path_msg);
        left_pose_pub.publish(temp_pose);
        LLeg_inc = false;
    }
    if(RLeg_inc)
    {
        if(right_path_msg.poses.size()>vector_size)
            right_path_msg.poses.clear();
    
        geometry_msgs::PoseStamped temp_pose;
        right_path_msg.header = RLeg_odom_msg.header;
        temp_pose.header = RLeg_odom_msg.header;
        temp_pose.pose = RLeg_odom_msg.pose.pose;
        right_path_msg.poses.push_back(temp_pose);
        right_path_pub.publish(right_path_msg);
        right_pose_pub.publish(temp_pose);
        RLeg_inc = false;
    }




    if(LFLeg_inc)
    {
        if(LF_path_msg.poses.size()>vector_size)
            LF_path_msg.poses.clear();
        
        geometry_msgs::PoseStamped temp_pose;
        LF_path_msg.header = LFLeg_odom_msg.header;
        temp_pose.header = LFLeg_odom_msg.header;
        temp_pose.pose = LFLeg_odom_msg.pose.pose;
        LF_path_msg.poses.push_back(temp_pose);
        LF_path_pub.publish(LF_path_msg);
        LF_pose_pub.publish(temp_pose);
        LFLeg_inc = false;
    }


    if(RFLeg_inc)
    {
        if(RF_path_msg.poses.size()>vector_size)
            RF_path_msg.poses.clear();

        geometry_msgs::PoseStamped temp_pose;
        RF_path_msg.header = RFLeg_odom_msg.header;
        temp_pose.header = RFLeg_odom_msg.header;
        temp_pose.pose = RFLeg_odom_msg.pose.pose;
        RF_path_msg.poses.push_back(temp_pose);
        RF_path_pub.publish(RF_path_msg);
        RF_pose_pub.publish(temp_pose);
        RFLeg_inc = false;
    }


    if(LHLeg_inc)
    {
        if(LH_path_msg.poses.size()>vector_size)
            LH_path_msg.poses.clear();

        geometry_msgs::PoseStamped temp_pose;
        LH_path_msg.header = LHLeg_odom_msg.header;
        temp_pose.header = LHLeg_odom_msg.header;
        temp_pose.pose = LHLeg_odom_msg.pose.pose;
        LH_path_msg.poses.push_back(temp_pose);
        LH_path_pub.publish(LH_path_msg);
        LH_pose_pub.publish(temp_pose);
        LHLeg_inc = false;
    }

    if(RHLeg_inc)
    {
        if(RH_path_msg.poses.size()>vector_size)
            RH_path_msg.poses.clear();

        geometry_msgs::PoseStamped temp_pose;
        RH_path_msg.header = RHLeg_odom_msg.header;
        temp_pose.header = RHLeg_odom_msg.header;
        temp_pose.pose = RHLeg_odom_msg.pose.pose;
        RH_path_msg.poses.push_back(temp_pose);
        RH_path_pub.publish(RH_path_msg);
        RH_pose_pub.publish(temp_pose);
        RHLeg_inc = false;
    }

    if(com_odom_inc)
    {
        if(com_path_msg.poses.size()>vector_size)
                com_path_msg.poses.clear();
                
        geometry_msgs::PoseStamped temp_pose;
        com_path_msg.header = com_odom_msg.header;
        temp_pose.header = com_odom_msg.header;
        temp_pose.pose = com_odom_msg.pose.pose;
        com_path_msg.poses.push_back(temp_pose);
        com_path_pub.publish(com_path_msg);
        com_odom_inc = false;
    }

    if (com_leg_odom_inc)
    {
        if (legcom_path_msg.poses.size() > vector_size)
            legcom_path_msg.poses.clear();

        geometry_msgs::PoseStamped temp_pose;
        legcom_path_msg.header = com_leg_odom_msg.header;
        temp_pose.header = com_leg_odom_msg.header;
        temp_pose.pose = com_leg_odom_msg.pose.pose;
        legcom_path_msg.poses.push_back(temp_pose);
        comleg_path_pub.publish(legcom_path_msg);
        com_leg_odom_inc = false;
    }

    if(gt_com_odom_inc)
    {
        if(ground_truth_com_path_msg.poses.size()>vector_size)
                ground_truth_com_path_msg.poses.clear();

            geometry_msgs::PoseStamped temp_pose;
            ground_truth_com_path_msg.header = gt_com_odom_msg.header;
            temp_pose.header = gt_com_odom_msg.header;
            temp_pose.pose = gt_com_odom_msg.pose.pose;
            ground_truth_com_path_msg.poses.push_back(temp_pose);
            ground_truth_com_path_pub.publish(ground_truth_com_path_msg);
            gt_com_odom_inc = false;
    }
    
    if(gt_odom_inc)
    {
        if(ground_truth_odom_path_msg.poses.size()>vector_size)
            ground_truth_odom_path_msg.poses.clear();

        geometry_msgs::PoseStamped temp_pose;
        ground_truth_odom_path_msg.header = gt_odom_msg.header;
        temp_pose.header = gt_odom_msg.header;
        temp_pose.pose = gt_odom_msg.pose.pose;
        ground_truth_odom_path_msg.poses.push_back(temp_pose);
        ground_truth_odom_path_pub.publish(ground_truth_odom_path_msg);
        gt_odom_inc = false;
    }
}
void serow_utils::supportCb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    support_pose_msg = *msg;
    support_pose_inc = true;
}

void serow_utils::odomCb(const nav_msgs::Odometry::ConstPtr &msg)
{
    odom_inc = true;
    odom_msg = *msg;   
}

void serow_utils::legOdomCb(const nav_msgs::Odometry::ConstPtr &msg)
{
   leg_odom_inc = true;
   leg_odom_msg = *msg;
}

void serow_utils::leftCb(const nav_msgs::Odometry::ConstPtr &msg)
{
    LLeg_inc = true;
    LLeg_odom_msg = *msg;
}

void serow_utils::rightCb(const nav_msgs::Odometry::ConstPtr &msg)
{
    RLeg_inc = true;
    RLeg_odom_msg = *msg;
}

void serow_utils::LFCb(const nav_msgs::Odometry::ConstPtr &msg)
{
   LFLeg_inc = true;
   LFLeg_odom_msg = *msg;
}

void serow_utils::RFCb(const nav_msgs::Odometry::ConstPtr &msg)
{
    RFLeg_inc = true;
    RFLeg_odom_msg = *msg;
}

void serow_utils::LHCb(const nav_msgs::Odometry::ConstPtr &msg)
{
    LHLeg_inc = true;
    LHLeg_odom_msg = *msg;
}

void serow_utils::RHCb(const nav_msgs::Odometry::ConstPtr &msg)
{
    RHLeg_inc = true;
    RHLeg_odom_msg = *msg;
}

void serow_utils::laserCb(const sensor_msgs::LaserScan::ConstPtr &scan_in)
{
    if (odom_inc)
    {
        sensor_msgs::PointCloud cloud;
        tf::Transform bt = tf::Transform(tf::Quaternion(odom_msg.pose.pose.orientation.x,
                                                        odom_msg.pose.pose.orientation.y,
                                                        odom_msg.pose.pose.orientation.z,
                                                        odom_msg.pose.pose.orientation.w),
                                         tf::Vector3(odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, odom_msg.pose.pose.position.z));

        tf::StampedTransform bst = tf::StampedTransform(bt, scan_in->header.stamp, odom_msg.header.frame_id, scan_in->header.frame_id);
        tf::Transformer btf;
        btf.setTransform(bst);
        projector_.transformLaserScanToPointCloud(odom_msg.header.frame_id, *scan_in,
                                                  cloud, btf);

        pointcloudFromLaserScan_pub.publish(cloud);
    }
}

void serow_utils::comCb(const nav_msgs::Odometry::ConstPtr &msg)
{
  com_odom_inc = true;
  com_odom_msg = *msg;
}

void serow_utils::CoMlegOdomCb(const nav_msgs::Odometry::ConstPtr &msg)
{
    com_leg_odom_inc = true;
    com_leg_odom_msg = *msg;
}
void serow_utils::gcomCb(const nav_msgs::Odometry::ConstPtr &msg)
{
   gt_com_odom_inc = true;
   gt_com_odom_msg = *msg;
}

void serow_utils::godomCb(const nav_msgs::Odometry::ConstPtr &msg)
{

    gt_odom_inc = true;
    gt_odom_msg = *msg;
}

void serow_utils::connect(const ros::NodeHandle nh)
{
    n = nh;
    ros::NodeHandle n_p("~");
    support_pose_inc = false;
    odom_inc = false;
    leg_odom_inc = false;
    LLeg_inc = false;
    RLeg_inc = false;
    LFLeg_inc = false;
    RFLeg_inc = false;
    LHLeg_inc = false;
    RHLeg_inc = false;
    com_odom_inc = false;
    com_leg_odom_inc = false;
    gt_com_odom_inc = false;
    gt_odom_inc = false;
    n_p.param<bool>("/serow/isQuadruped", isQuadruped, false);
    subscribe();
    advertise();
}
void serow_utils::run()
{
    ros::Rate r(100.0);
    while(ros::ok())
    {
        publish();
        ros::spinOnce();
        r.sleep();
    }
}