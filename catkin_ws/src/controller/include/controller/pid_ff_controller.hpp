#pragma once
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <turtlebot_state.hpp>
#include <turtlebot_msgs/Tune.h>

namespace Turtlebot
{

class PIDFeedForwardController
{
public:
    PIDFeedForwardController();
    ~PIDFeedForwardController();

    void initializeController();
    const geometry_msgs::TwistStamped getControls(const TurtlebotState &current_state, const TurtlebotState &desired_state);
    void setGains(const double &kp_w, const double &ki_w, const double &kd_w,
                  const double &kp_v, const double &ki_v, const double &kd_v);
    double err_pos_x ; double err_pos_y ; double err_vel_x ;
    double err_vel_y ; double err_acc_x ; double err_acc_y ;

    turtlebot_msgs::Tune m_pose_error ;
private:

    ros::Time m_prev_time;
    double m_kp_w;
    double m_ki_w;
    double m_kd_w;
    double m_kp_v;
    double m_ki_v;
    double m_kd_v;
    TurtlebotState *m_prev_state;
    double m_integral_w;
    double m_integral_v;
    bool m_first_it = true;

};

}
