#pragma once
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <turtlebot_state.hpp>

namespace Turtlebot
{

template <class ContType>

class SmithPredictor
{
public:
    SmithPredictor(const double &time_delay) : m_time_delay(time_delay){}
    ~SmithPredictor() = default;

    const geometry_msgs::Twist predictControls(const ContType &cont,
                                               const TurtlebotState &current_state,
                                               const TurtlebotState &desired_state,
                                               const geometry_msgs::PoseWithCovarianceStamped &pose,
                                               const nav_msgs::Odometry &odom);

private:


    double m_time_delay;
};

}
