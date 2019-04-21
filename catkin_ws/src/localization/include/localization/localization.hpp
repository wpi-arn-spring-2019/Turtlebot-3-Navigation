#pragma once
#include <ros/ros.h>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <fake_scan.hpp>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <particle.hpp>
#include <point.hpp>
#include <pose_estimation_icp.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <time.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/MarkerArray.h>

typedef boost::normal_distribution<> GaussianDistribution;
typedef boost::mt19937 RandomGenerator;
typedef boost::variate_generator<RandomGenerator, GaussianDistribution> GaussianGenerator;

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
    void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
    void initializeLocalization();
    void Localize();
    const Point getMapCoords(const int &location);
    const std::deque<Particle> sampleParticles();
    void takeActionParticles(std::deque<Particle> &particles);
    void calcParticleWeights(std::deque<Particle> &particles);
    const double calcDistanceScore(const tf::Point &particle_pt, const tf::Point &sensor_pt);
    const double calcRotationScore(const tf::Quaternion &particle_q, const tf::Quaternion &sensor_q);
    void pruneAndNormalizeParticles();
    const tf::Pose calcFinalPose();    
    void pubParticles();
    void integratePoseToCurrentTime(tf::Pose &pose);
    void setPreviousPose(const tf::Pose &pose);
    void pubFinalPose();
    std::vector<double> calcCovarianceMatrix();
    void integrateOdomToScanTime();

    ros::Subscriber m_scan_sub;
    ros::Subscriber m_odom_sub;
    ros::Subscriber m_map_sub;
    ros::Subscriber m_pose_sub;
    ros::Publisher m_particle_pub;
    ros::Publisher m_pose_pub;

    PoseEstimationICP *m_pose_icp;
    FakeScan *m_fake_scan;

    std::deque<Particle> m_particles;
    geometry_msgs::PoseWithCovarianceStamped m_prev_pose;

    sensor_msgs::LaserScan::ConstPtr m_scan;
    sensor_msgs::LaserScan m_prev_scan;
    bool m_have_scan = false;
    nav_msgs::Odometry::ConstPtr m_odom;
    nav_msgs::Odometry::ConstPtr m_prev_odom;
    bool m_have_odom = false;
    nav_msgs::Odometry m_odom_at_scan;
    nav_msgs::Odometry m_odom_at_last_scan;
    bool m_have_map = false;
    nav_msgs::OccupancyGrid::ConstPtr m_map;
    bool m_initialized = false;
    bool m_have_pose_estimate = false;

    RandomGenerator m_rng;
    GaussianGenerator *m_gen_s;
    GaussianGenerator *m_gen_st;
    GaussianGenerator *m_gen_sens_x;
    GaussianGenerator *m_gen_sens_y;
    GaussianGenerator *m_gen_sens_yaw;

    double m_sensor_var_x;
    double m_sensor_var_y;
    double m_sensor_var_yaw;

    int m_num_particles;
    double m_percent_to_drop;
    double m_percent_to_average;


};

}
