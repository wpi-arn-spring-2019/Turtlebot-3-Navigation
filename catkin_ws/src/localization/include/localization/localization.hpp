#pragma once
#include <ros/ros.h>
#include <fake_scan.hpp>
#include <geometry_msgs/Pose.h>
#include <particle.hpp>
#include <point.hpp>
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
    void Localize();

private:
    void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr &msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg);
    void initializeLocalization();
    const std::vector<Point> getFreeSpace();
    const Particle getRandomParticle(const std::vector<Point> &open_points);
    const Point getMapCoords(const int &location);
    const std::deque<Particle> sampleParticles();
    void takeActionParticles(std::deque<Particle> &particles);
    void calcParticleWeights(std::deque<Particle> &particles);
    const double calcDistanceScore(const tf::Point &particle_pt, const tf::Point &sensor_pt);
    const double calcRotationScore(const tf::Quaternion &particle_q, const tf::Quaternion &sensor_q);
    void pruneAndNormalizeParticles();
    const tf::StampedTransform calcFinalTransform();
    void setPreviousPose(const tf::StampedTransform &transform);

    ros::Subscriber m_scan_sub;
    ros::Subscriber m_odom_sub;
    ros::Subscriber m_map_sub;
    tf::TransformBroadcaster m_broad;

    PoseEstimationICP *m_pose_icp;
    FakeScan *m_fake_scan;

    std::deque<Particle> m_particles;
    geometry_msgs::Pose m_prev_pose;

    sensor_msgs::LaserScan::ConstPtr m_scan;
    sensor_msgs::LaserScan m_prev_scan;
    bool m_have_scan = false;
    nav_msgs::Odometry::ConstPtr m_odom;
    nav_msgs::Odometry::ConstPtr m_odom_at_scan;
    nav_msgs::Odometry::ConstPtr m_odom_at_last_scan;
    bool m_have_map = false;
    nav_msgs::OccupancyGrid::ConstPtr m_map;
    bool m_initialized = false;


    int m_num_particles;
    double m_percent_to_drop;
    double m_percent_to_average;


};

}
