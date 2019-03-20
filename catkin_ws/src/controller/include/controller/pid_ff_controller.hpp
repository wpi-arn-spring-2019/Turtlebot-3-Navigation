#pragma once
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlebot_state.hpp>

namespace Turtlebot
{

class PIDFeedForwardController
{
public:
    PIDFeedForwardController(){}
    ~PIDFeedForwardController() = default;

    const geometry_msgs::Twist getControls(const TurtlebotState &current_state, const TurtlebotState &desired_state);
    void setGains(const double &kp_w, const double &ki_w, const double &kd_w,
                  const double &kp_v, const double &ki_v, const double &kd_v);

private:

    double m_kp_w;
    double m_ki_w;
    double m_kd_w;
    double m_kp_v;
    double m_ki_v;
    double m_kd_v;



};

}
