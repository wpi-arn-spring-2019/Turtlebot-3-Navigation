#include <localization.hpp>

namespace Turtlebot
{

Localization::Localization(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{
    m_scan_sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 10, &Localization::laserScanCallback, this);
    m_odom_sub = nh.subscribe<nav_msgs::Odometry>("/odom", 10, &Localization::odomCallback, this);
    m_map_sub = nh.subscribe<nav_msgs::OccupancyGrid>("/map", 1, &Localization::mapCallback, this);
    m_pose_icp = new PoseEstimationICP;
    pnh.getParam("num_particles", m_num_particles);
}

Localization::~Localization()
{
    delete m_pose_icp;
}

const std::vector<Particle> Localization::sampleParticles()
{
    std::srand(ros::Time::now().toSec());
    std::vector<Particle> sampled_particles;
    for(int sample_id = 0; sample_id < m_num_particles; sample_id++)
    {
        double rand_sample = (std::rand() % 1000) / 1000;
        for(const auto &particle : m_particles)
        {
            rand_sample -= particle.weight;
            if(rand_sample <= 0)
            {
                sampled_particles.push_back(particle);
                break;
            }
        }
    }
    return sampled_particles;
}

void Localization::takeActionParticles(std::vector<Particle> &particles)
{
    for(auto &particle : particles)
    {
        const double &dx = m_odom_at_scan->pose.pose.position.x - m_odom_at_last_scan->pose.pose.position.x;
        const double &dy = m_odom_at_scan->pose.pose.position.y - m_odom_at_last_scan->pose.pose.position.y;
        const double &x = particle.pose.getOrigin().getX() + dx;
        const double &y = particle.pose.getOrigin().getY() + dy;
        particle.pose.getOrigin().setX(x);
        particle.pose.getOrigin().setY(y);
        tf::Quaternion q, last_q;
        q.setW(m_odom_at_scan->pose.pose.orientation.w);
        q.setX(m_odom_at_scan->pose.pose.orientation.x);
        q.setY(m_odom_at_scan->pose.pose.orientation.y);
        q.setZ(m_odom_at_scan->pose.pose.orientation.z);
        last_q.setW(m_odom_at_last_scan->pose.pose.orientation.w);
        last_q.setX(m_odom_at_last_scan->pose.pose.orientation.x);
        last_q.setY(m_odom_at_last_scan->pose.pose.orientation.y);
        last_q.setZ(m_odom_at_last_scan->pose.pose.orientation.z);
        tf::Quaternion dq = q - last_q;
        double qw = particle.pose.getRotation().getW() + dq.getW();
        double qx = particle.pose.getRotation().getX() + dq.getX();
        double qy = particle.pose.getRotation().getY() + dq.getY();
        double qz = particle.pose.getRotation().getZ() + dq.getZ();
        const double &normalizer = std::sqrt(std::pow(qw, 2) + std::pow(qx, 2) + std::pow(qy, 2) + std::pow(qz, 2));
        particle.pose.getRotation().setW(qw / normalizer);
        particle.pose.getRotation().setX(qx / normalizer);
        particle.pose.getRotation().setY(qy / normalizer);
        particle.pose.getRotation().setZ(qz / normalizer);
    }
}

void Localization::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    m_scan = msg;
    m_odom_at_scan = m_odom;
}

void Localization::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    m_odom = msg;
}

void Localization::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    if(!m_have_map)
    {
        m_have_map = true;
        m_map = msg;
    }
}

}
