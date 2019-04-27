#include <pd_ff_controller.hpp>

namespace Turtlebot
{

PDFeedForwardController::~PDFeedForwardController()
{
    delete m_prev_state;
}

const geometry_msgs::TwistStamped PDFeedForwardController::getControls(const TurtlebotState &current_state, const TurtlebotState &desired_state)
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
    const double &control_w = desired_state.th_dot - m_kp_w * error_w + m_kd_w * derivative_w;
    control.twist.angular.z = control_w;
    const double &error_v = current_state.v - desired_state.v;
    double derivative_v = (current_state.v - m_prev_state->v) / dt;
    if(std::isnan(derivative_v) || std::isinf(derivative_v))
    {
        derivative_v = 0;
    }
    const double &control_v = desired_state.v - m_kp_v * error_v + m_kd_v * derivative_v;
    control.twist.linear.x = control_v;
    m_prev_time = current_time;
    m_prev_state = new TurtlebotState(current_state);

    m_pos_error.header.stamp = ros::Time::now();
    m_pos_error.cur_pos_x = current_state.x;
    m_pos_error.des_pos_x = desired_state.x;
    m_pos_error.err_pos_x = current_state.x - desired_state.x ;
    m_pos_error.cur_pos_y = current_state.y;
    m_pos_error.des_pos_y = desired_state.y;
    m_pos_error.err_pos_y = current_state.y - desired_state.y ;

    m_vel_error.header.stamp = ros::Time::now();
    m_vel_error.cur_vel_x = current_state.x_dot;
    m_vel_error.des_vel_x = desired_state.x_dot;
    m_vel_error.err_vel_x = current_state.x_dot - desired_state.x_dot;
    m_vel_error.cur_vel_y = current_state.y_dot;
    m_vel_error.des_vel_y = desired_state.y_dot;
    m_vel_error.err_vel_y = current_state.y_dot - desired_state.y_dot;

    m_acc_error.header.stamp = ros::Time::now();
    m_acc_error.cur_acc_x = current_state.x_ddot;
    m_acc_error.des_acc_x = desired_state.x_ddot;
    m_acc_error.err_acc_x = current_state.x_ddot - desired_state.x_ddot;
    m_acc_error.cur_acc_y = current_state.y_ddot;
    m_acc_error.des_acc_y = desired_state.y_ddot;
    m_acc_error.err_acc_y = current_state.y_ddot - desired_state.y_ddot;

    m_lin_vel_error.header.stamp = ros::Time::now();
    m_lin_vel_error.cur_lin_vel = current_state.v;
    m_lin_vel_error.des_lin_vel = desired_state.v;
    m_lin_vel_error.err_lin_vel = current_state.v - desired_state.v;

    m_ang_vel_error.header.stamp = ros::Time::now();
    m_ang_vel_error.cur_ang_vel = current_state.th_dot;
    m_ang_vel_error.des_ang_vel = desired_state.th_dot;
    m_ang_vel_error.err_ang_vel = current_state.th_dot - desired_state.th_dot;


    return control;
}

void PDFeedForwardController::setGains(const double &kp_w, const double &kd_w, const double &kp_v, const double &kd_v)
{
    m_kp_w = kp_w;
    m_kd_w = kd_w;
    m_kp_v = kp_v;
    m_kd_v = kd_v;
}

}
