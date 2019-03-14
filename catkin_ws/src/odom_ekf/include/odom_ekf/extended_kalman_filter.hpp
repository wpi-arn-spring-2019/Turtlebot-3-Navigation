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
    ~ExtendedKalmanFilter();

private:

    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);

    void filterOdom();

    const Eigen::MatrixXd calcG();



    void integrateIMUToOdom();
    void pubOdom();

    ros::Subscriber m_odom_sub;
    ros::Subscriber m_imu_sub;
    ros::Publisher m_odom_pub;

    nav_msgs::Odometry::ConstPtr m_odom;
    sensor_msgs::Imu::ConstPtr m_imu;
    sensor_msgs::Imu m_imu_at_odom;
    sensor_msgs::Imu m_imu_at_last_odom;
    sensor_msgs::Imu::ConstPtr m_prev_imu;

    bool m_have_odom = false;
    bool m_have_imu = false;


};

}
