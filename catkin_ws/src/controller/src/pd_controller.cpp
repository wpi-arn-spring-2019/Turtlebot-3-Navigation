#include <pd_controller.hpp>

namespace Turtlebot
{

PDController::~PDController()
{
    delete m_prev_state;
}

const geometry_msgs::TwistStamped PDController::getControls(const TurtlebotState &current_state, const TurtlebotState &desired_state)
{
    if(m_first_it)
    {
        m_prev_time = ros::Time::now();
        m_prev_state = new TurtlebotState(current_state);
        m_first_it = false;
    }
    ros::Time current_time = ros::Time::now();
    const double &dt = ros::Duration(current_time - m_prev_time).toSec();
    geometry_msgs::TwistStamped control;
    control.header.stamp = current_time;
    const double &error_w = current_state.th_dot - desired_state.th_dot;
    double derivative_w = (current_state.th_dot - m_prev_state->th_dot) / dt;
    if(std::isnan(derivative_w) || std::isinf(derivative_w))
    {
        derivative_w = 0;
    }
    const double &control_w =  - m_kp_w * error_w  + m_kd_w * derivative_w;
    control.twist.angular.z = control_w;
    const double &error_v = current_state.v - desired_state.v;
    double derivative_v = (current_state.v - m_prev_state->v) / dt;
    if(std::isnan(derivative_v) || std::isinf(derivative_v))
    {
        derivative_v = 0;
    }
    const double &control_v =  - m_kp_v * error_v + m_kd_v * derivative_v;
    control.twist.linear.x = control_v;
    m_prev_time = current_time;
    m_prev_state = new TurtlebotState(current_state);
    return control;
}

void PDController::setGains(const double &kp_w, const double &kd_w, const double &kp_v, const double &kd_v)
{
    m_kp_w = kp_w;
    m_kd_w = kd_w;
    m_kp_v = kp_v;
    m_kd_v = kd_v;
}

}
