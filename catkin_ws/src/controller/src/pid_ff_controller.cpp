#include <pid_ff_controller.hpp>

namespace Turtlebot
{

PIDFeedForwardController::PIDFeedForwardController()
{
    initializeController();
}

PIDFeedForwardController::~PIDFeedForwardController()
{
    delete m_prev_state;
}

void PIDFeedForwardController::initializeController()
{
    m_integral_w = 0;
    m_integral_v = 0;
}

const geometry_msgs::Twist PIDFeedForwardController::getControls(const TurtlebotState &current_state, const TurtlebotState &desired_state)
{
    if(m_first_it)
    {
        m_prev_time = ros::Time::now();
        m_prev_state = new TurtlebotState(current_state);
    }
    const double &dt = ros::Duration(ros::Time::now() - m_prev_time).toSec();
    geometry_msgs::Twist control;
    const double &error_w = current_state.th_dot - desired_state.th_dot;
    double derivative_w = (current_state.th_dot - m_prev_state->th_dot) / dt;
    if(std::isnan(derivative_w) || std::isinf(derivative_w))
    {
        derivative_w = 0;
    }
    m_integral_w += error_w * dt;
    const double &control_w = desired_state.th_dot - m_kp_w * error_w - m_ki_w * m_integral_w - m_kd_w * derivative_w;
    control.angular.z = control_w;
    const double &error_v = current_state.v - desired_state.v;
    double derivative_v = (current_state.v - m_prev_state->v) / dt;
    if(std::isnan(derivative_v) || std::isinf(derivative_v))
    {
        derivative_v = 0;
    }
    m_integral_v += error_v * dt;
    const double &control_v = desired_state.th_dot - m_kp_v * error_v - m_ki_v * m_integral_v - m_kd_v * derivative_v;
    control.linear.x = control_v;
    return control;
}

void PIDFeedForwardController::setGains(const double &kp_w, const double &ki_w, const double &kd_w, const double &kp_v, const double &ki_v, const double &kd_v)
{
    m_kp_w = kp_w;
    m_ki_w = ki_w;
    m_kd_w = kd_w;
    m_kp_v = kp_v;
    m_ki_v = ki_v;
    m_kd_v = kd_v;
}

}
