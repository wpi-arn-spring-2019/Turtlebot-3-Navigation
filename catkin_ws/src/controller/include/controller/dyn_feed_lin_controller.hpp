#pragma once
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <turtlebot_state.hpp>

namespace Turtlebot
{

class DYNController
{
public:
    DYNController();
    ~DYNController();
    const geometry_msgs::TwistStamped getControls(const TurtlebotState &current_state, const TurtlebotState &desired_state);
    void setGains(const double &lam0_dyn_fed_lin, const double &lam1_dyn_fed_lin, const double &lam2_dyn_fed_lin,
             const double &gam0_dyn_fed_lin, const double &gam1_dyn_fed_lin, const double &gam2_dyn_fed_lin);
private:

    ros::Time m_prev_time;
    double m_gam0_dyn_fed_lin ;
    double m_gam1_dyn_fed_lin ;
    double m_gam2_dyn_fed_lin ;
    double m_lam0_dyn_fed_lin ;
    double m_lam1_dyn_fed_lin ;
    double m_lam2_dyn_fed_lin ;
    bool m_first_it = true;


};

}
