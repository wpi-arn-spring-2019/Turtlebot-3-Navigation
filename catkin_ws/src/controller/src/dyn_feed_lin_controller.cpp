#include <dyn_feed_lin_controller.hpp>

namespace Turtlebot
{

const geometry_msgs::TwistStamped DYNController::getControls(const TurtlebotState &current_state, const TurtlebotState &desired_state)
{

    if(m_first_it)
    {
        m_prev_time = ros::Time::now();
        m_first_it = false;
    }

    ros::Time current_time = ros::Time::now();
    const double &dt = ros::Duration(current_time - m_prev_time).toSec();
    geometry_msgs::TwistStamped control;
    control.header.stamp = current_time;

    const double &error_dist_x = current_state.x - desired_state.x;
    const double &error_vel_x =  current_state.x_dot - desired_state.x_dot;

    const double &error_dist_y = current_state.y - desired_state.y;
    const double &error_vel_y = current_state.y_dot - desired_state.y_dot;

    const double &cur_acc_x = desired_state.x_ddot  - m_lam1_dyn_fed_lin*error_vel_x - m_lam0_dyn_fed_lin*error_dist_x ;
    const double &cur_acc_y = desired_state.y_ddot  - m_lam1_dyn_fed_lin*error_vel_y - m_lam0_dyn_fed_lin*error_dist_y ;

    double input_vel_x = desired_state.x_dot + cur_acc_x * dt;
    double input_vel_y = desired_state.y_dot + cur_acc_y * dt;

    double input_vel = std::sqrt(std::pow(input_vel_x, 2) + std::pow(input_vel_y, 2));

    double radius_curv = (std::pow((input_vel_x*input_vel_x +input_vel_y*input_vel_y), 1.5))/
            (input_vel_x*cur_acc_y - input_vel_y*cur_acc_x) ;

    double input_ang_vel = input_vel / radius_curv;

    m_prev_time = current_time;

    m_dyn_vel_cmd.twist.linear.x = input_vel ;
    m_dyn_vel_cmd.twist.angular.z = input_ang_vel ;

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


    return m_dyn_vel_cmd;

}

void DYNController::setGains(const double &lam0_dyn_fed_lin, const double &lam1_dyn_fed_lin)
{
    m_lam0_dyn_fed_lin = lam0_dyn_fed_lin;
    m_lam1_dyn_fed_lin = lam1_dyn_fed_lin;
}

}
