#include <extended_kalman_filter.hpp>

namespace Turtlebot
{

ExtendedKalmanFilter::ExtendedKalmanFilter(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{
    m_odom_sub = nh.subscribe<nav_msgs::Odometry>("/odom", 10, &ExtendedKalmanFilter::odomCallback, this);
    m_imu_sub = nh.subscribe<sensor_msgs::Imu>("/imu", 10, &ExtendedKalmanFilter::imuCallback, this);
}

ExtendedKalmanFilter::~ExtendedKalmanFilter()
{

}

void ExtendedKalmanFilter::filterOdom()
{
    calcG();
}

const Eigen::MatrixXd ExtendedKalmanFilter::calcG()
{
    tf::Quaternion q;
    tf::quaternionMsgToTF(m_imu_at_last_odom.orientation, q);
    const double &theta = tf::getYaw(q);
    ros::Duration dt = m_imu_at_odom.header.stamp - m_imu_at_last_odom.header.stamp;
    Eigen::MatrixXd G(6, 6);
    G << 1, 0, 0, 0,
         0, 1, 0, 0;

}

void ExtendedKalmanFilter::integrateIMUToOdom()
{
    const ros::Duration &dt = m_odom->header.stamp - m_imu->header.stamp;
    m_imu_at_odom.header.stamp = m_odom->header.stamp;
    const double &jerk_x = (m_imu->linear_acceleration.x - m_prev_imu->linear_acceleration.x) / dt.toSec();
    const double &jerk_y = (m_imu->linear_acceleration.y - m_prev_imu->linear_acceleration.y) / dt.toSec();
    m_imu_at_odom.linear_acceleration.x = m_imu->linear_acceleration.x + jerk_x * dt.toSec();
    m_imu_at_odom.linear_acceleration.y = m_imu->linear_acceleration.y + jerk_y * dt.toSec();
    m_imu_at_odom.linear_acceleration.z = 0;
    const double &ang_acc_z = (m_imu->angular_velocity.z - m_prev_imu->angular_velocity.z) / dt.toSec();
    m_imu_at_odom.angular_velocity.x = 0;
    m_imu_at_odom.angular_velocity.y = 0;
    m_imu_at_odom.angular_velocity.z = m_imu->angular_velocity.z + ang_acc_z * dt.toSec();
    tf::Quaternion q;
    tf::quaternionMsgToTF(m_imu->orientation, q);
    const double &yaw = tf::getYaw(q) + m_imu_at_odom.angular_velocity.z * dt.toSec() + ang_acc_z * std::pow(dt.toSec(), 2) / 2;
    geometry_msgs::Quaternion q_;
    tf::quaternionTFToMsg(tf::createQuaternionFromYaw(yaw), q_);
    m_imu_at_odom.orientation = q_;
}

void ExtendedKalmanFilter::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    if(!m_have_odom)
    {
        m_have_odom = true;
    }
    m_odom = msg;
    if(m_have_imu)
    {
        integrateIMUToOdom();
        filterOdom();
    }
}

void ExtendedKalmanFilter::imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    if(!m_have_imu)
    {
        m_have_imu = true;
        m_prev_imu = msg;
    }
    m_imu = msg;
}


}
