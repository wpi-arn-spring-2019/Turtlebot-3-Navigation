#include <odom_correction.hpp>

namespace Turtlebot
{

OdomCorrection::OdomCorrection(ros::NodeHandle &nh)
{
    m_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/pf_pose", 10, &OdomCorrection::poseCallback, this);
}

void OdomCorrection::correctOdom()
{
    if(m_have_pose)
    {
        tf::StampedTransform odom_transform;
        try
        {
            m_list.lookupTransform("/odom", "/base_footprint", ros::Time(0), odom_transform);
        }
        catch(tf::TransformException &ex)
        {
            ROS_ERROR("%s",ex.what());
        }
        tf::Point pt;
        tf::pointMsgToTF(m_pose->pose.position, pt);
        tf::Quaternion q;
        tf::quaternionMsgToTF(m_pose->pose.orientation, q);
        tf::Transform transform;
        transform.setOrigin(pt);
        transform.setRotation(q);
        transform = transform * odom_transform.inverse();
        tf::StampedTransform final_trans;
        final_trans.stamp_ = ros::Time::now();
        final_trans.frame_id_ = "/map";
        final_trans.child_frame_id_ = "/odom";
        final_trans.setOrigin(transform.getOrigin());
        final_trans.setRotation(transform.getRotation());
        m_broad.sendTransform(final_trans);
    }
}

void OdomCorrection::poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    if(!m_have_pose)
    {
        m_have_pose = true;
    }
    m_pose = msg;
}

}
