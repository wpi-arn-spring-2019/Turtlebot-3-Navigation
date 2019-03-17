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
    const float &dt = ros::Duration(m_odom->header.stamp - m_imu->header.stamp).toSec();
    const float &x_ddot = m_imu_at_last_odom.linear_acceleration.x * cos(th);
    const float &y_ddot = m_imu_at_last_odom.linear_acceleration.y * sin(th);
    const float &x_dot = m_odom_filtered.twist.twist.linear.x + x_ddot * dt;
    const float &y_dot = m_odom_filtered.twist.twist.linear.y + y_ddot * dt;
    const float &dxdth = -m_imu_at_last_odom.linear_acceleration.x * sin(th) * std::pow(dt, 2) / 2;
    const float &dxdthd = -m_imu_at_last_odom.linear_acceleration.x * sin(th) * std::pow(dt, 2) * 3 / 2;
    const float &dydth = m_imu_at_last_odom.linear_acceleration.y * cos(th) * std::pow(dt, 2) / 2;
    const float &dydthd = m_imu_at_last_odom.linear_acceleration.x * cos(th) * std::pow(dt, 2) * 3 / 2;
    const float &a = y_ddot * sin(th) * (x_dot + dt * x_ddot * cos(th)) - x_ddot * cos(th) * (y_dot + dt * y_ddot * sin(th));
    const float &b = std::pow(x_dot + dt * x_ddot * cos(th), 2) + std::pow(y_dot + dt * y_ddot * sin(th), 2);
    const float &dthdth = -(std::pow(dt, 2) * cos(th) * (3 * std::pow(dt, 2) * std::pow(x_ddot, 2) * fabs(a) * checkSign(b) * std::pow(cos(th), 2) -
                             fabs(a) * fabs(b) + x_ddot * y_ddot * checkSign(a) * fabs(b) * cos(th) + 3 * dt * x_dot * x_ddot * fabs(a) * checkSign(b) * cos(th)) * 3) /
                             2 * std::pow(fabs(b), 5 / 2);
    const float &c = x_dot + dt * x_ddot * cos(th);
    const float &d = y_dot + dt * y_ddot * sin(th);
    const float &e = std::pow(c, 2) + std::pow(d, 2);
    const float &f = std::pow(e, 3 / 2);
    const float &g = std::pow(e, 5 / 2);
    const float &h = y_ddot * sin(th) * c - x_ddot * cos(th) * d;
    const float &i = 2 * x_dot + 2 * dt * x_ddot * cos(th);
    const float &dthdxd = std::pow(dt, 2) * x_ddot * y_ddot * checkSign(h) * cos(th) * sin(th) / (2 * f) - dt * ((dt * x_ddot * fabs(h) * checkSign(e) * cos(th) * i * 3) / (2 * e) -
                           dt * x_ddot * y_ddot * checkSign(h) * cos(th) * sin(th) / f) - std::pow(dt, 2) * x_ddot * fabs(h) * checkSign(e) * cos(th) * i * 3 / (4 * g);
    const float &j = y_ddot * sin(th) * (x_dot + dt * x_ddot * cos(th) - x_ddot * cos(th)) * (y_ddot * sin(th));
    const float &k = std::pow(x_dot + dt * x_ddot * cos(th), 2) + std::pow(y_dot + dt * y_ddot * sin(th), 2);
    const float &dthdyd = -(std::pow(dt, 2) * x_ddot * cos(th) * (3 * y_ddot * fabs(j) * checkSign(k) + x_ddot * checkSign(j) * fabs(k) * cos(th) + 3 * dt * y_ddot *
                            fabs(j * checkSign(k) * 3)) / (2 * fabs(std::pow(k, 5 / 2))));
    const float &dthdthd = dt;
    const float &dxddth = - m_imu_at_last_odom.linear_acceleration.x * sin(th);
    const float &dyddth = m_imu_at_last_odom.linear_acceleration.y * cos(th);
    const float &l = x_dot + dt * x_ddot * cos(th);
    const float &m = y_dot + dt * y_ddot * sin(th);
    const float &n = std::pow(l, 2) + std::pow(m, 2);
    const float &o = fabs(std::pow(n, 3 / 2));
    const float &p = x_ddot * y_dot * cos(th) - x_dot * y_ddot * sin(th);
    const float &dthddth = dt * x_ddot * fabs(p) * checkSign(n) * cos(th) * ( 2 * dt * x_ddot * sin(th) * l - 2 * dt * y_ddot * cos(th) * m) * 3 / (2 * fabs(std::pow(n, 5 / 2))) -
                           dt * x_ddot * checkSign(p) * cos(th) * (x_dot * y_ddot * cos(th) + x_ddot * y_dot * sin(th)) / o;
    const float &q = std::pow(x_dot + dt *x_ddot * cos(th), 2) + std::pow(y_dot + dt * y_ddot * sin(th), 2);
    const float &r = x_ddot * y_dot * cos(th) - x_dot * y_ddot * sin(th);
    const float &dthddxd = -((dt * x_ddot * y_ddot * checkSign(r) * sin(2 * th)) / (2 * fabs(std::pow(q, 3 /2)))) - (dt * x_ddot * fabs(r) * checkSign(q) * cos(th) * (2 * x_dot +
                              2 * dt * x_ddot * cos(th)) * 3) / (2 * fabs(std::pow(q, 5 / 2)));
    const float &s = std::pow(x_dot + dt * x_ddot * cos(th), 2) + std::pow(y_dot + dt * y_ddot * sin(th), 2);
    const float &t = x_ddot * y_dot * cos(th) - x_dot * y_ddot * sin(th);
    const float &dthddyd = dt * std::pow(x_ddot, 2) * checkSign(t) * std::pow(cos(th), 2) / fabs(std::pow(s, 3 / 2)) - (dt * x_ddot * checkSign(t) * cos(th) * (2 * y_dot + 2 * dt *
                           y_ddot * sin(th)) * 3 / (2 * fabs(std::pow(s, 5 / 2))));
    const float &u = x_dot + dt * x_ddot * cos(th);
    const float &v = y_dot + dt * y_ddot * sin(th);
    const float &w = std::pow(u, 2) + std::pow(v, 2);
    const float &x = y_ddot * sin(th) * u - x_ddot * cos(th) * v;
    const float &dthddthd = dt * fabs(x) * cos(th) / fabs(std::pow(w, 3/ 2)) - (dt * x_ddot * checkSign(x) * cos(th) * (cos(th) * v - dt * y_ddot * cos(th) * sin(th)) /
                            fabs(std::pow(w, 3 / 2))) - (std::pow(dt, 2) * x_ddot * fabs(x) * checkSign(w) * std::pow(cos(th), 2) * u * 4 / fabs(std::pow(w, 5 / 2)));
    Eigen::MatrixXf G(6, 6);
    G << 1, 0, dxdth,   0,       0,       dxdthd,
         0, 1, dydth,   0,       0,       dydthd,
         0, 0, dthdth,  dthdxd,  dthdyd,  dthdthd,
         0, 0, dxddth,  1,       0,       0,
         0, 0, dyddth,  0,       1,       0,
         0, 0, dthddth, dthddxd, dthddyd, dthddthd;
    return G;
}

const float ExtendedKalmanFilter::checkSign(const float &val)
{
    if(val == 0)
    {
        return 0;
    }
    else if (std::signbit(val))
    {
        return -1.0f;
    }
    else
    {
        return 1.0f;
    }
}

const Eigen::MatrixXf ExtendedKalmanFilter::calcUBar()
{
    tf::Quaternion q;
    tf::quaternionMsgToTF(m_odom_filtered.pose.pose.orientation, q);
    const float &th_ = tf::getYaw(q);
    const float &dt = ros::Duration(m_odom->header.stamp - m_imu->header.stamp).toSec();
    const float &x_ddot = m_imu_at_last_odom.linear_acceleration.x * cos(th_);
    const float &y_ddot = m_imu_at_last_odom.linear_acceleration.y * sin(th_);
    const float &x_dot = m_odom_filtered.twist.twist.linear.x + x_ddot * dt;
    const float &y_dot = m_odom_filtered.twist.twist.linear.y + y_ddot * dt;
    const float &x = m_odom_filtered.pose.pose.position.x + x_dot * dt + x_ddot * std::pow(dt, 2) / 2;
    const float &y = m_odom_filtered.pose.pose.position.y + y_dot * dt + y_ddot * std::pow(dt, 2) / 2;
    const float &r = fabs(std::pow(std::pow(x_dot, 2) + std::pow(y_dot, 2), 3 / 2) /
                           x_dot * y_ddot - y_dot * x_ddot);
    const float &th_ddot = x_ddot / r;
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
    const float &jerk_x = (m_imu->linear_acceleration.x - m_prev_imu->linear_acceleration.x) / dt;
    const float &jerk_y = (m_imu->linear_acceleration.y - m_prev_imu->linear_acceleration.y) / dt;
    m_imu_at_odom.linear_acceleration.x = m_imu->linear_acceleration.x + jerk_x * dt;
    m_imu_at_odom.linear_acceleration.y = m_imu->linear_acceleration.y + jerk_y * dt;
    m_imu_at_odom.linear_acceleration.z = 0;
    const float &ang_acc_z = (m_imu->angular_velocity.z - m_prev_imu->angular_velocity.z) / dt;
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
    if(!m_have_odom)
    {
        m_have_odom = true;
        m_odom_filtered = *msg;
        m_odom_covariance(0, 0) = std::sqrt(msg->pose.covariance[0]);
        m_odom_covariance(1, 1) = std::sqrt(msg->pose.covariance[7]);
        m_odom_covariance(2, 2) = std::sqrt(msg->pose.covariance[35]);
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
