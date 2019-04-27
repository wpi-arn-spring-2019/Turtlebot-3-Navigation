#pragma once
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <turtlebot_state.hpp>
#include <turtlebot_msgs/cntrl_acc.h>
#include <turtlebot_msgs/cntrl_vel.h>
#include <turtlebot_msgs/cntrl_lin_vel.h>
#include <turtlebot_msgs/cntrl_ang_vel.h>
#include <turtlebot_msgs/cntrl_pos.h>

namespace Turtlebot
{

class DYNController
{
public:
    DYNController(){}
    ~DYNController() = default;
    const geometry_msgs::TwistStamped getControls(const TurtlebotState &current_state, const TurtlebotState &desired_state);
    void setGains(const double &lam0_dyn_fed_lin, const double &lam1_dyn_fed_lin);
    turtlebot_msgs::cntrl_acc m_acc_error ;
    turtlebot_msgs::cntrl_vel m_vel_error ;
    turtlebot_msgs::cntrl_pos m_pos_error ;
    turtlebot_msgs::cntrl_lin_vel m_lin_vel_error ;
    turtlebot_msgs::cntrl_ang_vel m_ang_vel_error ;

private:

    ros::Time m_prev_time;
    double m_lam0_dyn_fed_lin ;
    double m_lam1_dyn_fed_lin ;
    bool m_first_it = true;
    geometry_msgs::TwistStamped m_dyn_vel_cmd;

};

}
