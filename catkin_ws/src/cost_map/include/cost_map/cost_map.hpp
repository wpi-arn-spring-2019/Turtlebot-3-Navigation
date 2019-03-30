#pragma once
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <fake_scan.hpp>
#include <nav_msgs/Odometry.h>
#include <cmath>


namespace Turtlebot
{
    class CostMap
    {
    public:
        CostMap(ros::NodeHandle &nh, ros::NodeHandle &pnh);
        ~CostMap();
    private:
        sensor_msgs::LaserScan current_scan;
        sensor_msgs::LaserScan referance_scan;
        geometry_msgs::PoseWithCovarianceStamped current_pose;
        nav_msgs::OccupancyGrid cost_map;
        nav_msgs::OccupancyGrid::ConstPtr m_map;
        int once_flag = 1;
        bool cost_map_available = 0;
        bool scan_available = 0;

        ros::Subscriber current_scan_sub;
        ros::Subscriber current_pose_sub;
        ros::Subscriber map_sub;
        ros::Publisher cost_map_pub;
        FakeScan fake_scan;
        float threshold;

        void currentScanCallback(const sensor_msgs::LaserScan::ConstPtr &msg);
        void currentPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
        void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg);
        void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
        void generateCostMap();
        void integratePoseToCurrentTime(tf::Pose &pose);
        void integrateOdomToScanTime();
        void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr &msg);


        FakeScan *m_fake_scan;
        nav_msgs::Odometry m_odom_at_scan;
        nav_msgs::Odometry m_odom_at_last_scan;
        nav_msgs::Odometry::ConstPtr m_odom;
        nav_msgs::Odometry::ConstPtr m_prev_odom;
        sensor_msgs::LaserScan::ConstPtr m_scan;
        sensor_msgs::LaserScan m_prev_scan;
        geometry_msgs::PoseWithCovarianceStamped m_prev_pose;

        bool m_have_scan = false;
        bool m_have_odom = false;
        bool m_have_map = false;
        bool m_have_pose_estimate = false;












    };











}
