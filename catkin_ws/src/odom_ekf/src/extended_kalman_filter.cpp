#include <extended_kalman_filter.hpp>

namespace Turtlebot
{

ExtendedKalmanFilter::ExtendedKalmanFilter(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{
    m_odom_sub = nh.subscribe<nav_msgs::Odometry>("/odom", 10, &ExtendedKalmanFilter::odomCallback, this);
    m_imu_sub = nh.subscribe<sensor_msgs::Imu>("/imu", 10, &ExtendedKalmanFilter::imuCallback, this);
    m_odom_pub = nh.advertise<nav_msgs::Odometry>("/odom/filtered", 10);
    initializeFilter(pnh);
}

ExtendedKalmanFilter::~ExtendedKalmanFilter()
{

}

void ExtendedKalmanFilter::initializeFilter(ros::NodeHandle &pnh)
{
    m_covariance.resize(6, 6);
    m_covariance.setIdentity();
    m_imu_covariance.resize(6, 6);
    m_imu_covariance.setZero();
    pnh.getParam("imu_var_x", m_imu_covariance(0, 0));
    pnh.getParam("imu_var_y", m_imu_covariance(1, 1));
    pnh.getParam("imu_var_th", m_imu_covariance(2, 2));
    pnh.getParam("imu_var_xd", m_imu_covariance(3, 3));
    pnh.getParam("imu_var_yd", m_imu_covariance(4, 4));
    pnh.getParam("imu_var_thd", m_imu_covariance(5, 5));
    for(int diag = 0; diag < 6; diag++)
    {
        m_imu_covariance(diag, diag) = std::sqrt(m_imu_covariance(diag, diag));
    }
    m_odom_covariance.resize(6, 6);
    m_odom_covariance.setZero();
    pnh.getParam("odom_var_xd", m_odom_covariance(3, 3));
    pnh.getParam("odom_var_yd", m_odom_covariance(4, 4));
    pnh.getParam("odom_var_thd", m_odom_covariance(5, 5));
    for(int diag = 3; diag < 6; diag++)
    {
        m_odom_covariance(diag, diag) = std::sqrt(m_odom_covariance(diag, diag));
    }
}

void ExtendedKalmanFilter::filterOdom()
{
    const Eigen::MatrixXf &G = calcG();
    const Eigen::MatrixXf covariance_ = G * m_covariance * G.transpose() + m_imu_covariance;
    const Eigen::MatrixXf &u_ = calcUBar();
}

const Eigen::MatrixXf ExtendedKalmanFilter::calcG()
{
    tf::Quaternion q_;
    tf::quaternionMsgToTF(m_odom_filtered.pose.pose.orientation, q_);
    const float &th = tf::getYaw(q_);
    const float &dt = ros::Duration(m_odom->header.stamp - m_imu_at_last_odom.header.stamp).toSec();
    const float &x_ddot = m_imu_at_last_odom.linear_acceleration.x * cos(th);
    const float &y_ddot = m_imu_at_last_odom.linear_acceleration.y * sin(th);
    const float &dxdth = -(3 * std::pow(dt, 2) * x_ddot * sin(th)) / 2;
    const float &dydth = (3 * std::pow(dt, 2) * y_ddot * cos(th)) / 2;
    const float &dxddth = -dt * x_ddot * sin(th);
    const float &dyddth = dt * y_ddot * cos(th);
    const float &dxdxd = dt;
    const float &dydyd = dt;
    const float &dthdthd = dt;
    Eigen::MatrixXf G(6, 6);
    G.setIdentity();
    G(0, 2) = dxdth;
    G(1, 2) = dydth;
    G(3, 2) = dxddth;
    G(4, 2) = dyddth;
    G(0, 3) = dxdxd;
    G(1, 4) = dydyd;
    G(2, 5) = dthdthd;
    return G;
}

const Eigen::MatrixXf ExtendedKalmanFilter::calcUBar()
{
    tf::Quaternion q;
    tf::quaternionMsgToTF(m_odom_filtered.pose.pose.orientation, q);
    const float &th_ = tf::getYaw(q);
    const float &dt = ros::Duration(m_odom->header.stamp - m_imu_at_last_odom.header.stamp).toSec();
    float x_ddot = m_imu_at_last_odom.linear_acceleration.x * cos(th_);
    if(std::isnan(x_ddot))
    {
        x_ddot = 0.0f;
    }
    float y_ddot = m_imu_at_last_odom.linear_acceleration.y * sin(th_);
    if(std::isnan(y_ddot))
    {
        y_ddot = 0.0f;
    }
    const float &x_dot = m_odom_filtered.twist.twist.linear.x + x_ddot * dt;
    const float &y_dot = m_odom_filtered.twist.twist.linear.y + y_ddot * dt;
    const float &x = m_odom_filtered.pose.pose.position.x + x_dot * dt + x_ddot * std::pow(dt, 2) / 2;
    const float &y = m_odom_filtered.pose.pose.position.y + y_dot * dt + y_ddot * std::pow(dt, 2) / 2;
    float th_ddot = (m_imu_at_odom.angular_velocity.z - m_imu_at_last_odom.angular_velocity.z) / dt;
    if(std::isnan(th_ddot) || th_ddot == std::numeric_limits<float>::infinity())
    {
        th_ddot = 0;
    }
    const float &th_dot = m_imu_at_odom.angular_velocity.z + th_ddot * dt;
    const float &th = th_ + th_dot * dt + th_ddot * std::pow(dt, 2) / 2;
    Eigen::MatrixXf u_(6, 1);
    u_ << x, y, th, x_dot, y_dot, th_dot;
    return u_;
}

void ExtendedKalmanFilter::integrateIMUToOdom()
{
    const float &dt = ros::Duration(m_odom->header.stamp - m_imu->header.stamp).toSec();
    m_imu_at_odom.header.stamp = m_odom->header.stamp;
    float jerk_x = (m_imu->linear_acceleration.x - m_prev_imu->linear_acceleration.x) / dt;
    float jerk_y = (m_imu->linear_acceleration.y - m_prev_imu->linear_acceleration.y) / dt;
    if(jerk_x == std::numeric_limits<float>::infinity())
    {
        jerk_x = 0.0f;
    }
    if(jerk_y == std::numeric_limits<float>::infinity())
    {
        jerk_y = 0.0f;
    }
    m_imu_at_odom.linear_acceleration.x = m_imu->linear_acceleration.x + jerk_x * dt;
    m_imu_at_odom.linear_acceleration.y = m_imu->linear_acceleration.y + jerk_y * dt;
    m_imu_at_odom.linear_acceleration.z = 0;
    float ang_acc_z = (m_imu->angular_velocity.z - m_prev_imu->angular_velocity.z) / dt;
    if(std::isnan(ang_acc_z))
    {
        ang_acc_z = 0.0f;
    }
    m_imu_at_odom.angular_velocity.x = 0;
    m_imu_at_odom.angular_velocity.y = 0;
    m_imu_at_odom.angular_velocity.z = m_imu->angular_velocity.z + ang_acc_z * dt;
    tf::Quaternion q;
    tf::quaternionMsgToTF(m_imu->orientation, q);
    const float &yaw = tf::getYaw(q) + m_imu_at_odom.angular_velocity.z * dt + ang_acc_z * std::pow(dt, 2) / 2;
    geometry_msgs::Quaternion q_;
    tf::quaternionTFToMsg(tf::createQuaternionFromYaw(yaw), q_);
    m_imu_at_odom.orientation = q_;
}

void ExtendedKalmanFilter::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    if(!m_have_odom && m_have_imu)
    {
        m_have_odom = true;
        m_odom_filtered = *msg;
        m_odom_covariance(0, 0) = std::sqrt(msg->pose.covariance[0]);
        m_odom_covariance(1, 1) = std::sqrt(msg->pose.covariance[7]);
        m_odom_covariance(2, 2) = std::sqrt(msg->pose.covariance[35]);
        m_odom = msg;
        integrateIMUToOdom();
        m_imu_at_last_odom = m_imu_at_odom;
    }
    m_odom = msg;
    if(m_have_imu)
    {
        integrateIMUToOdom();
        filterOdom();
        m_imu_at_last_odom = m_imu_at_odom;
    }
}

void ExtendedKalmanFilter::imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    if(!m_have_imu)
    {
        m_have_imu = true;
        m_prev_imu = msg;
    }
    m_prev_imu = m_imu;
    m_imu = msg;
}


}
