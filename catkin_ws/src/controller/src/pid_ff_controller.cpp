#include <pid_ff_controller.hpp>

namespace Turtlebot
{

const geometry_msgs::Twist PIDFeedForwardController::getControls(const TurtlebotState &current_state, const TurtlebotState &desired_state)
{

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
