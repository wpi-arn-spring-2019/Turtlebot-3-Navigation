#pragma once
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <turtlebot_msgs/Trajectory.h>


namespace Turtlebot
{

class Controller
{
public:
    Controller(ros::NodeHandle &nh, ros::NodeHandle &pnh);
    ~Controller();

    void control();

private:
    void trajectoryCallback(const turtlebot_msgs::Trajectory::ConstPtr &msg);
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void pubControls(const geometry_msgs::Twist &control);

    void integratePoseToCurrentTime();
    void integrateOdomToCurrentTime();

    ros::Subscriber m_traj_sub;
    ros::Subscriber m_pose_sub;
    ros::Subscriber m_odom_sub;
    ros::Publisher m_vel_pub;

    ros::Time m_current_time;

    turtlebot_msgs::Trajectory::ConstPtr m_traj;
    geometry_msgs::PoseStamped::ConstPtr m_pose;
    geometry_msgs::Pose m_pose_at_control;
    nav_msgs::Odometry::ConstPtr m_odom;
    nav_msgs::Odometry::ConstPtr m_prev_odom;
    nav_msgs::Odometry m_odom_at_pose;
    nav_msgs::Odometry m_odom_at_control;

    bool m_have_trajectory = false;
    bool m_have_pose = false;
    bool m_have_odom = false;

};

}
