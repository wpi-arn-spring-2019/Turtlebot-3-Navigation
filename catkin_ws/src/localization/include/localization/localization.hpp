#pragma once
#include <ros/ros.h>
#include <fake_scan.hpp>
#include <geometry_msgs/Pose.h>
#include <particle.hpp>
#include <pose_estimation_icp.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

namespace Turtlebot
{

class Localization
{
public:
    Localization(ros::NodeHandle &nh, ros::NodeHandle &pnh);
    ~Localization();

private:
    void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr &msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg);
    const std::vector<Particle> sampleParticles();
    void takeActionParticles(std::vector<Particle> &particles);

    ros::Subscriber m_scan_sub;
    ros::Subscriber m_odom_sub;
    ros::Subscriber m_map_sub;
    tf::TransformBroadcaster m_broad;

    PoseEstimationICP *m_pose_icp;

    std::vector<Particle> m_particles;



    sensor_msgs::LaserScan::ConstPtr m_scan;
    nav_msgs::Odometry::ConstPtr m_odom;
    nav_msgs::Odometry::ConstPtr m_odom_at_scan;
    nav_msgs::Odometry::ConstPtr m_odom_at_last_scan;
    bool m_have_map = false;
    nav_msgs::OccupancyGrid::ConstPtr m_map;


    int m_num_particles;

};

}
