#pragma once
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <pd_controller.hpp>
#include <pid_controller.hpp>
#include <pd_ff_controller.hpp>
#include <pid_ff_controller.hpp>
#include <tf/tf.h>
#include <turtlebot_msgs/Trajectory.h>
#include <turtlebot_state.hpp>


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
    void pubControls(const geometry_msgs::Twist &control) const;

    void initializeController(ros::NodeHandle &pnh);
    const TurtlebotState getCurrentState();
    const TurtlebotState getDesiredState() const;
    const TurtlebotState integrateDesiredStateToCurrentTime(const int &traj_it, const double &dt) const;
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

    enum controller_type{PD = 1, PID = 2, PD_FF = 3, PID_FF = 4};
    controller_type m_cont_type;
    double m_kp;
    double m_ki;
    double m_kd;

    PIDFeedForwardController *m_pid_ff_cont;


    bool m_goal_reached = false;
    bool m_have_trajectory = false;
    bool m_have_pose = false;
    bool m_have_odom = false;

};

}
