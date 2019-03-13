#pragma once
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

namespace Turtlebot
{

class OdomCorrection
{
public:
    OdomCorrection(ros::NodeHandle &nh);
    ~OdomCorrection() = default;

    void correctOdom();

private:
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

    ros::Subscriber m_pose_sub;
    tf::TransformBroadcaster m_broad;
    tf::TransformListener m_list;
    geometry_msgs::PoseStamped::ConstPtr m_pose;
    bool m_have_pose = false;

};

}
