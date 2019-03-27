#include <smith_predictor.hpp>

namespace Turtlebot
{

template class SmithPredictor<PDController>;
template class SmithPredictor<PIDController>;
template class SmithPredictor<PDFeedForwardController>;
template class SmithPredictor<PIDFeedForwardController>;

template <class ContType>

SmithPredictor<ContType>::SmithPredictor(const ContType &cont)
{
    m_cont = new ContType(cont);
    m_predict_cont = new ContType(cont);
}

template <class ContType>

SmithPredictor<ContType>::~SmithPredictor()
{
    delete m_cont;
    delete m_predict_cont;
}

template <class ContType>

const geometry_msgs::TwistStamped SmithPredictor<ContType>::predictControls(const TurtlebotState &current_state,
                                                                            const TurtlebotState &desired_state,
                                                                            const TurtlebotState &next_desired_state,
                                                                            const nav_msgs::Odometry &odom)
{
    const geometry_msgs::TwistStamped &current_controls = m_cont->getControls(current_state, desired_state);
    if(m_first_it)
    {
        m_first_it = false;
        m_prev_controls = current_controls;
    }
    const TurtlebotState &predicted_state = predictFeedback(current_controls, odom);
    geometry_msgs::TwistStamped predicted_controls = m_predict_cont->getControls(predicted_state, next_desired_state);
    m_prev_controls = predicted_controls;
    return predicted_controls;
}

template <typename ContType>

const TurtlebotState SmithPredictor<ContType>::predictFeedback(const geometry_msgs::TwistStamped &current_controls,
                                                               const nav_msgs::Odometry &odom)
{
    const double &dt = ros::Duration(odom.header.stamp - m_prev_controls.header.stamp).toSec();
    const double &predicted_vel = odom.twist.twist.linear.x / m_prev_controls.twist.linear.x * current_controls.twist.linear.x;
    const double &predicted_th_dot = odom.twist.twist.angular.z / m_prev_controls.twist.angular.z * current_controls.twist.angular.z;
    tf::Quaternion q;
    tf::quaternionMsgToTF(odom.pose.pose.orientation, q);
    const double &predicted_th = tf::getYaw(q) + predicted_th_dot * dt;
    const double &predicted_x = odom.pose.pose.position.x + predicted_vel * dt * cos(tf::getYaw(q));
    const double &predicted_y = odom.pose.pose.position.y + predicted_vel * dt * sin(tf::getYaw(q));
    return TurtlebotState(predicted_x, predicted_y, predicted_th, predicted_vel, predicted_th_dot);
}


}
