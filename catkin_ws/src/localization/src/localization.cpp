#include <localization.hpp>

namespace Turtlebot
{

Localization::Localization(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{
    m_scan_sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 10, &Localization::laserScanCallback, this);
    m_odom_sub = nh.subscribe<nav_msgs::Odometry>("/odom", 10, &Localization::odomCallback, this);
    m_map_sub = nh.subscribe<nav_msgs::OccupancyGrid>("/map", 1, &Localization::mapCallback, this);
    m_pose_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, &Localization::poseCallback, this);
    m_particle_pub = nh.advertise<visualization_msgs::MarkerArray>("/particles", 1);
    m_pose_icp = new PoseEstimationICP;
    pnh.getParam("num_particles", m_num_particles);
    pnh.getParam("percent_to_drop", m_percent_to_drop);
    pnh.getParam("percent_to_average", m_percent_to_average);
    std::srand(ros::Time::now().toSec());
}

Localization::~Localization()
{
    delete m_fake_scan;
    delete m_pose_icp;
    delete m_gen_x;
    delete m_gen_y;
    delete m_gen_yaw;
}

void Localization::Localize()
{
    if(!m_have_map || !m_have_scan || !m_have_odom || !m_have_pose_estimate)
    {
        return;
    }
    if(!m_initialized)
    {
        initializeLocalization();
        m_initialized = true;
    }
    std::deque<Particle> particles = sampleParticles();
    takeActionParticles(particles);
    calcParticleWeights(particles);
    m_particles = particles;
    pubParticles();
    pruneAndNormalizeParticles();
    tf::StampedTransform final_tf = calcFinalTransform();
    m_broad.sendTransform(final_tf);
    setPreviousPose(final_tf);
}

void Localization::initializeLocalization()
{
    const std::vector<Point> &open_points = getFreeSpace();
    for(int particle_it = 0; particle_it < open_points.size(); particle_it++)
    {
        m_particles.push_back(getRandomParticle(open_points));
    }
}

const std::vector<Point> Localization::getFreeSpace()
{
    std::vector<Point> points;
    for(int point_it = 0; point_it < m_map->data.size(); point_it++)
    {
        if(int(m_map->data[point_it]) == 0)
        {
            points.push_back(getMapCoords(point_it));
        }
    }
    return points;
}

const Particle Localization::getRandomParticle(const std::vector<Point> &open_points)
{    
    const int &rand_sample = std::rand() % open_points.size();
    tf::Pose pose;
    const double &x = open_points[rand_sample].x;
    const double &y = open_points[rand_sample].y;
    pose.setOrigin(tf::Vector3(x, y, 0));
    tf::Quaternion q;
    q.setW(m_prev_pose.orientation.w);
    q.setX(m_prev_pose.orientation.x);
    q.setY(m_prev_pose.orientation.y);
    q.setZ(m_prev_pose.orientation.z);
    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    pose.setRotation(tf::createQuaternionFromYaw(yaw));
    return Particle(pose, 1.0 / open_points.size());
}

const Point Localization::getMapCoords(const int &location)
{    
    const int &height = m_map->info.height;
    const int &width = m_map->info.width;
    const double &resolution = m_map->info.resolution;
    const double &x = double(location % height - height / 2 - 1) * resolution;
    const double &y = double(int(location / height - width / 2 - 1)) * resolution;
    return Point(x, y);
}

const std::deque<Particle> Localization::sampleParticles()
{
    std::deque<Particle> sampled_particles;
    for(int sample_id = 0; sample_id < m_num_particles; sample_id++)
    {
        double rand_sample = double(std::rand() % 100000) / 100000;
        for(const auto &particle : m_particles)
        {
            rand_sample -= particle.weight;
            if(rand_sample <= 0.0)
            {
                sampled_particles.push_back(particle);
                break;
            }
        }
    }    
    return sampled_particles;
}

void Localization::takeActionParticles(std::deque<Particle> &particles)
{
    for(auto &particle : particles)
    {
        ros::Duration dt = m_odom_at_scan.header.stamp - m_odom_at_last_scan.header.stamp;
        double acc_x, acc_y, acc_ang;
        if(dt.toNSec() == 0)
        {
            acc_x = 0;
            acc_y = 0;
            acc_ang = 0;
        }
        else
        {
            acc_x = (m_odom_at_scan.twist.twist.linear.x - m_odom_at_last_scan.twist.twist.linear.x) / dt.toSec();
            acc_y = (m_odom_at_scan.twist.twist.linear.y - m_odom_at_last_scan.twist.twist.linear.y) / dt.toSec();
            acc_ang = (m_odom_at_scan.twist.twist.angular.z - m_odom_at_last_scan.twist.twist.angular.z) / dt.toSec();
        }
        double roll, pitch, yaw;
        tf::Matrix3x3(particle.pose.getRotation()).getRPY(roll, pitch, yaw);
        const double &yaw_f = yaw + m_odom_at_scan.twist.twist.angular.z * dt.toSec() + acc_ang * std::pow(dt.toSec(), 2) / 2;
        const double &x_f = particle.pose.getOrigin().getX() + (m_odom_at_scan.twist.twist.linear.x * dt.toSec() + acc_x * std::pow(dt.toSec(), 2) / 2) * cos(yaw);
        const double &y_f = particle.pose.getOrigin().getY() + (m_odom_at_scan.twist.twist.linear.y * dt.toSec() + acc_y * std::pow(dt.toSec(), 2) / 2) * sin(yaw);
        particle.pose.setOrigin(tf::Vector3(x_f + m_gen_x->operator ()(),
                                            y_f + m_gen_y->operator ()(),
                                            0));
        particle.pose.setRotation(tf::createQuaternionFromYaw(yaw_f + m_gen_yaw->operator ()()));
    }    
}

void Localization::calcParticleWeights(std::deque<Particle> &particles)
{
    const sensor_msgs::LaserScan &fake_scan = m_prev_scan;// m_fake_scan->getFakeScan(m_prev_pose);
    m_prev_scan = *m_scan;
    tf::Pose prev_pose;
    prev_pose.setOrigin(tf::Vector3(m_prev_pose.position.x, m_prev_pose.position.y, 0));
    tf::Quaternion q;
    tf::quaternionMsgToTF(m_prev_pose.orientation, q);
    prev_pose.setRotation(q);
    const tf::Pose &sensor_pose_estimate = prev_pose * m_pose_icp->getTransform(fake_scan, *m_scan);
    for(auto &particle : particles)
    {
        const double &distance_score = calcDistanceScore(particle.pose.getOrigin(), sensor_pose_estimate.getOrigin());
        const double &rotation_score = calcRotationScore(particle.pose.getRotation(), sensor_pose_estimate.getRotation());
        const double &weight = distance_score + rotation_score;        
        particle.weight = weight;
    }
    std::sort(particles.begin(), particles.end());    
}

const double Localization::calcDistanceScore(const tf::Point &particle_pt, const tf::Point &sensor_pt)
{
    const double &dx = fabs(particle_pt.getX() - sensor_pt.getX());
    const double &dy = fabs(particle_pt.getY() - sensor_pt.getY());
    const double &dz = fabs(particle_pt.getZ() - sensor_pt.getZ());
    double dist = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2) + std::pow(dz, 2));
    if(dist <= 0.000001)
    {
        dist = 0.00001;
    }
    return 100.0 / (dist);
}

const double Localization::calcRotationScore(const tf::Quaternion &particle_q, const tf::Quaternion &sensor_q)
{    
    double roll, pitch, yaw_p, yaw_s;
    tf::Matrix3x3(particle_q).getRPY(roll, pitch, yaw_p);
    tf::Matrix3x3(sensor_q).getRPY(roll, pitch, yaw_s);
    double d_yaw = yaw_p - yaw_s;
    if(fabs(d_yaw) <= 0.000001)
    {
        d_yaw = 0.00001;
    }
    return 1.0 / fabs(d_yaw);
}

void Localization::pruneAndNormalizeParticles()
{
    const int &num_particles_to_prune = m_particles.size() * (m_percent_to_drop / 100);
    for(int i = 0; i < num_particles_to_prune; i++)
    {
        m_particles.pop_front();
    }
    double total_weight = 0;
    for(const auto &particle : m_particles)
    {
        total_weight += particle.weight;
    }
    for(auto &particle : m_particles)
    {
        particle.weight = particle.weight / total_weight;
    }
}

const tf::StampedTransform Localization::calcFinalTransform()
{
    const int &num_particles_to_average = m_num_particles * (m_percent_to_average / 100);
    const int &start_it = m_particles.size() - num_particles_to_average;
    double x = 0;
    double y = 0;
    double yaw = 0;
    for(int  particle_it = start_it;  particle_it < m_particles.size(); particle_it++)
    {
        x += m_particles[particle_it].pose.getOrigin().getX();
        y += m_particles[particle_it].pose.getOrigin().getY();
        double roll, pitch, yaw_;
        tf::Matrix3x3(m_particles[particle_it].pose.getRotation()).getRPY(roll, pitch, yaw_);
        yaw += yaw_;        
    }
    x = x / double(num_particles_to_average) - m_odom_at_scan.pose.pose.position.x;
    y = y / double(num_particles_to_average) - m_odom_at_scan.pose.pose.position.y;
    tf::Quaternion q;
    tf::quaternionMsgToTF(m_odom_at_scan.pose.pose.orientation, q);
    double roll, pitch, yaw_;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw_);
    yaw = yaw / double(num_particles_to_average) - yaw_;
    tf::StampedTransform transform;
    transform.setOrigin(tf::Vector3(x, y, 0));
    transform.setRotation(tf::createQuaternionFromYaw(yaw));  
    transform.stamp_ = ros::Time::now();
    transform.child_frame_id_ = "/odom";
    transform.frame_id_ = "/map";
    return transform;
}

void Localization::pubParticles()
{
    visualization_msgs::MarkerArray marker_arr;
    ros::Time stamp = ros::Time::now();
    int id = 0;
    for(const auto &particle : m_particles)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "/map";
        marker.header.stamp = stamp;
        marker.id = id;
        id++;
        marker.action = visualization_msgs::Marker::ADD;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.pose.position.x = particle.pose.getOrigin().getX();
        marker.pose.position.y = particle.pose.getOrigin().getY();
        marker.pose.position.z = 0;
        marker.pose.orientation.w = 1;
        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0;
        marker.pose.orientation.z = 0;
        marker.scale.x = 0.025;
        marker.scale.y = 0.025;
        marker.scale.z = 0.025;
        marker.color.a = 1.0;
        marker.color.r = 0;
        marker.color.g = 0;
        marker.color.b = 1.0;
        marker_arr.markers.push_back(marker);
    }
    m_particle_pub.publish(marker_arr);
}

void Localization::integratePoseToCurrentTime()
{

}

void Localization::setPreviousPose(const tf::StampedTransform &transform)
{
    m_prev_pose.position.x = transform.getOrigin().getX(); + m_odom_at_scan.pose.pose.position.x;
    m_prev_pose.position.y = transform.getOrigin().getY(); + m_odom_at_scan.pose.pose.position.y;
    m_prev_pose.position.z = 0;
    tf::Quaternion q;
    tf::quaternionMsgToTF(m_odom_at_scan.pose.pose.orientation, q);
    q = transform.getRotation() + q;
    q.normalize();
    geometry_msgs::Quaternion gq;
    tf::quaternionTFToMsg(q, gq);
    m_prev_pose.orientation = gq;
    m_odom_at_last_scan = m_odom_at_scan;
}

void Localization::integrateOdomToScanTime()
{
    ros::Duration dt = m_scan->header.stamp - m_odom->header.stamp;
    m_odom_at_scan.header.stamp = m_scan->header.stamp;
    const double &acc_x = (m_odom->twist.twist.linear.x - m_prev_odom->twist.twist.linear.x) / dt.toSec();
    const double &acc_y = (m_odom->twist.twist.linear.y - m_prev_odom->twist.twist.linear.y) / dt.toSec();
    const double &acc_ang = (m_odom->twist.twist.angular.z - m_prev_odom->twist.twist.angular.z) / dt.toSec();
    m_odom_at_scan.twist.twist.linear.x = m_odom->twist.twist.linear.x + acc_x * dt.toSec();
    m_odom_at_scan.twist.twist.linear.y = m_odom->twist.twist.linear.y + acc_y * dt.toSec();
    m_odom_at_scan.twist.twist.angular.z = m_odom->twist.twist.angular.z + acc_ang * dt.toSec();
    tf::Quaternion q;
    tf::quaternionMsgToTF(m_odom->pose.pose.orientation, q);
    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    const double &radius_curvature = std::pow(m_odom_at_scan.twist.twist.linear.x, 2) +
                                     std::pow(m_odom_at_scan.twist.twist.linear.y, 2) /
                                     m_odom_at_scan.twist.twist.angular.z;
    const double &yaw_f = m_odom_at_scan.twist.twist.linear.z * dt.toSec();
    double x_f;
    double y_f;
    if(std::isnan(radius_curvature))
    {
        x_f = m_odom->pose.pose.position.x + (m_odom->twist.twist.linear.x * dt.toSec() + acc_x * std::pow(dt.toSec(), 2) / 2) * cos(yaw);
        y_f = m_odom->pose.pose.position.y + (m_odom->twist.twist.linear.y * dt.toSec() + acc_y * std::pow(dt.toSec(), 2) / 2) * sin(yaw);
    }
    else
    {
        x_f = m_odom->pose.pose.position.x + radius_curvature * cos(yaw_f) * cos(yaw);
        y_f = m_odom->pose.pose.position.y + radius_curvature * sin(yaw_f) * sin(yaw);
    }
    tf::Quaternion q_f = tf::createQuaternionFromYaw(yaw_f);
    q_f.normalize();
    m_odom_at_scan.pose.pose.position.x = x_f;
    m_odom_at_scan.pose.pose.position.y = y_f;
    geometry_msgs::Quaternion q_;
    tf::quaternionTFToMsg(q_f, q_);
    m_odom_at_scan.pose.pose.orientation = q_;
}

void Localization::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{    
    if(!m_have_scan && m_have_odom)
    {
        m_have_scan = true;
        m_prev_scan = *msg;
        m_odom_at_last_scan = *m_odom;
    }
    m_scan = msg;
    if(m_have_odom)
    {
        integrateOdomToScanTime();
    }
}

void Localization::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    if(!m_have_odom)
    {
        m_have_odom = true;
        m_prev_odom = msg;
        RandomGenerator rng;
        rng.seed(ros::Time::now().toSec());
        GaussianDistribution dist_x(0.0, std::sqrt(msg->pose.covariance[0]));
        GaussianDistribution dist_y(0.0, std::sqrt(msg->pose.covariance[7]));
        GaussianDistribution dist_yaw(0.0, std::sqrt(msg->pose.covariance[35]));
        m_gen_x = new GaussianGenerator(rng, dist_x);
        m_gen_y = new GaussianGenerator(rng, dist_y);
        m_gen_yaw = new GaussianGenerator(rng, dist_yaw);
    }
    m_odom = msg;
}

void Localization::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    if(!m_have_map)
    {
        m_have_map = true;
        m_fake_scan = new FakeScan(*msg);
        m_map = msg;
    }
}

void Localization::poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    if(!m_have_pose_estimate)
    {
        m_have_pose_estimate = true;
    }
    ROS_INFO_STREAM("Pose Estimate Updated Manually");
    m_prev_pose = msg->pose.pose;
}

}
