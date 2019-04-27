#pragma once
#include <ros/ros.h>
#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>

namespace Turtlebot
{

class ExtendedKalmanFilter
{
public:
    ExtendedKalmanFilter(ros::NodeHandle &nh, ros::NodeHandle &pnh);
    ~ExtendedKalmanFilter() = default;

private:

    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);

    void initializeFilter(ros::NodeHandle &pnh);
    void filterOdom();
    const Eigen::MatrixXf calcG();
    const Eigen::MatrixXf calcCovarianceBar(const Eigen::MatrixXf &G);
    const Eigen::MatrixXf calcUBar();
    const Eigen::MatrixXf calczBar(const Eigen::MatrixXf &u_);
    const Eigen::MatrixXf calcH(const Eigen::MatrixXf &u_);
    const Eigen::MatrixXf calcKalmanGain(const Eigen::MatrixXf &cov_, const Eigen::MatrixXf &H);
    const Eigen::MatrixXf calcU(const Eigen::MatrixXf &u_, const Eigen::MatrixXf &K);
    void calcCovariance(const Eigen::MatrixXf & cov_, const Eigen::MatrixXf &H, const Eigen::MatrixXf &K);

    void integrateIMUToOdom();
    void setz();
    void pubOdom(const Eigen::MatrixXf &u);

    ros::Subscriber m_odom_sub;
    ros::Subscriber m_imu_sub;
    ros::Publisher m_odom_pub;

    nav_msgs::Odometry m_odom_filtered;

    nav_msgs::Odometry::ConstPtr m_odom;
    Eigen::MatrixXf m_z;
    nav_msgs::Odometry::ConstPtr m_prev_odom;
    sensor_msgs::Imu::ConstPtr m_imu;
    sensor_msgs::Imu m_imu_at_odom;
    sensor_msgs::Imu m_imu_at_last_odom;
    sensor_msgs::Imu::ConstPtr m_prev_imu;

    Eigen::MatrixXf m_covariance;
    Eigen::MatrixXf m_imu_covariance;
    Eigen::MatrixXf m_odom_covariance;

    bool m_have_odom = false;
    bool m_have_imu = false;

    bool m_compensate;


};

}
