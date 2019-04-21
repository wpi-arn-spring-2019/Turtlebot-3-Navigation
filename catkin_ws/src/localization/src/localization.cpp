#include <localization.hpp>

namespace Turtlebot
{

Localization::Localization(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{
    m_scan_sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 10, &Localization::laserScanCallback, this);
    m_odom_sub = nh.subscribe<nav_msgs::Odometry>("/odom/filtered", 10, &Localization::odomCallback, this);
    m_map_sub = nh.subscribe<nav_msgs::OccupancyGrid>("/map", 1, &Localization::mapCallback, this);
    m_pose_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, &Localization::poseCallback, this);
    m_particle_pub = nh.advertise<visualization_msgs::MarkerArray>("/particles", 1);
    m_pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/pf_pose", 10);
    m_pose_icp = new PoseEstimationICP;
    pnh.getParam("num_particles", m_num_particles);
    pnh.getParam("percent_to_drop", m_percent_to_drop);
    pnh.getParam("percent_to_average", m_percent_to_average);
    pnh.getParam("sensor_var_x", m_sensor_var_x);
    pnh.getParam("sensor_var_y", m_sensor_var_y);
    pnh.getParam("sensor_var_yaw", m_sensor_var_yaw);
    std::srand(ros::Time::now().toSec());
}

Localization::~Localization()
{
    delete m_fake_scan;
    delete m_pose_icp;
    delete m_gen_s;
    delete m_gen_st;
    delete m_gen_sens_x;
    delete m_gen_sens_y;
    delete m_gen_sens_yaw;
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
    const tf::Pose final_pose = calcFinalPose();    
    setPreviousPose(final_pose);
    pubFinalPose();
}

void Localization::initializeLocalization()
{
    GaussianDistribution initial_pose_x(0.0, std::sqrt(m_prev_pose.pose.covariance[0]));
    GaussianDistribution initial_pose_y(0.0, std::sqrt(m_prev_pose.pose.covariance[7]));
    GaussianDistribution initial_pose_th(0.0, std::sqrt(m_prev_pose.pose.covariance[35]));
    GaussianGenerator gen_x(m_rng, initial_pose_x);
    GaussianGenerator gen_y(m_rng, initial_pose_y);
    GaussianGenerator gen_th(m_rng, initial_pose_th);
    for(int particle_it = 0; particle_it < m_num_particles; particle_it++)
    {

        const double &x = m_prev_pose.pose.pose.position.x + gen_x.operator ()();
        const double &y = m_prev_pose.pose.pose.position.y + gen_y.operator ()();
        tf::Quaternion q;
        tf::quaternionMsgToTF(m_prev_pose.pose.pose.orientation, q);
        const double &th = tf::getYaw(q) + gen_th.operator ()();
        tf::Pose pose;
        pose.setOrigin(tf::Vector3(x, y, 0));
        pose.setRotation(tf::createQuaternionFromYaw(th));
        m_particles.push_back(Particle(pose, 1.0 / double(m_num_particles)));
    }
}

const Point Localization::getMapCoords(const int &location)
{    
    const int &height = m_map->info.height;
    const double &origin_x = m_map->info.origin.position.x;
    const double &origin_y = m_map->info.origin.position.y;
    const double &resolution = m_map->info.resolution;
    const double &x = double(location % height) * resolution + origin_x;
    const double &y = double(int(location / height)) * resolution + origin_y;
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
        const double &xp = particle.pose.getOrigin().getX();
        const double &yp = particle.pose.getOrigin().getY();
        const double &yawp = tf::getYaw(particle.pose.getRotation());
        const double &xi = m_odom_at_last_scan.pose.pose.position.x;
        const double &yi = m_odom_at_last_scan.pose.pose.position.y;
        const double &xf = m_odom_at_scan.pose.pose.position.x;
        const double &yf = m_odom_at_scan.pose.pose.position.y;
        const double &sigma_trans = std::sqrt(std::pow(xi - xf, 2) + std::pow(yi - yf, 2));
        const double &ang_bet = std::atan2(yf - yi, xf - xi);        
        tf::Quaternion q, prev_q;
        tf::quaternionMsgToTF(m_odom_at_scan.pose.pose.orientation, q);       
        tf::quaternionMsgToTF(m_odom_at_last_scan.pose.pose.orientation, prev_q);
        const double &yawi = tf::getYaw(prev_q);
        const double &yawf = tf::getYaw(q);
        const double &sigma_rot1 = ang_bet -  tf::getYaw(prev_q);
        const double &sigma_rot2 = yawf - tf::getYaw(prev_q) - sigma_rot1;
        const double &sigma_rot1_ = sigma_rot1 + m_gen_s->operator ()();
        const double &sigma_trans_ = sigma_trans + m_gen_st->operator ()();
        const double &sigma_rot2_ = sigma_rot2 + m_gen_s->operator ()();
        const double &x = xp + sigma_trans_ * cos (yawi + sigma_rot1_);
        const double &y = yp + sigma_trans_ * sin (yawi + sigma_rot1_);
        const double &yaw = yawp + sigma_rot1_ + sigma_rot2_;
        particle.pose.setOrigin(tf::Vector3(x, y, 0));
        particle.pose.setRotation(tf::createQuaternionFromYaw(yaw));
    }
}

void Localization::calcParticleWeights(std::deque<Particle> &particles)
{
    const sensor_msgs::LaserScan &fake_scan = m_fake_scan->getFakeScan(m_prev_pose);    
    tf::Pose prev_pose;
    prev_pose.setOrigin(tf::Vector3(m_prev_pose.pose.pose.position.x, m_prev_pose.pose.pose.position.y, 0));
    tf::Quaternion q;
    tf::quaternionMsgToTF(m_prev_pose.pose.pose.orientation, q);
    prev_pose.setRotation(q);    
    tf::Pose sensor_pose_estimate = prev_pose * m_pose_icp->getTransform(fake_scan, *m_scan);
    sensor_pose_estimate.setOrigin(tf::Vector3(sensor_pose_estimate.getOrigin().getX() + m_gen_sens_x->operator ()(),
                                               sensor_pose_estimate.getOrigin().getY() + m_gen_sens_y->operator ()(),
                                               0));
    double yaw = tf::getYaw(sensor_pose_estimate.getRotation());
    yaw += m_gen_sens_yaw->operator ()();
    sensor_pose_estimate.setRotation(tf::createQuaternionFromYaw(yaw));
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
    double dist = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
    if(dist <= 0.00001)
    {
        dist = 0.00001;
    }
    return 5.0f / dist;
}

const double Localization::calcRotationScore(const tf::Quaternion &particle_q, const tf::Quaternion &sensor_q)
{    
    const double &yaw_p = tf::getYaw(particle_q);
    const double &yaw_s = tf::getYaw(sensor_q);
    double d_yaw = fabs(yaw_p - yaw_s);
    if(d_yaw <= 0.00001)
    {
        d_yaw = 0.00001;
    }
    return 1.0f / d_yaw;
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

const tf::Pose Localization::calcFinalPose()
{
    const int &num_particles_to_avg = m_percent_to_average / 100 * m_num_particles;
    const int &start_it = m_particles.size() - num_particles_to_avg;
    double total_weight = 0;
    std::vector<Particle> particles_to_avg;
    for(int particle_it = start_it; particle_it < m_particles.size(); particle_it++)
    {
        particles_to_avg.push_back(m_particles[particle_it]);
        total_weight += m_particles[particle_it].weight;
    }
    for(auto &particle : particles_to_avg)
    {
        particle.weight = particle.weight / total_weight;
    }
    double x = 0;
    double y = 0;
    double yaw = 0;
    for(const auto &particle : particles_to_avg)
    {
        x += particle.pose.getOrigin().getX() * particle.weight;
        y += particle.pose.getOrigin().getY() * particle.weight;
        const double &yaw_ = tf::getYaw(particle.pose.getRotation());
        yaw += yaw_ * particle.weight;
    }
    tf::Pose final_pose;
    final_pose.setOrigin(tf::Vector3(x, y, 0));
    final_pose.setRotation(tf::createQuaternionFromYaw(yaw));
    integratePoseToCurrentTime(final_pose);
    return final_pose;
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

void Localization::integratePoseToCurrentTime(tf::Pose &pose)
{
    ros::Duration dt = ros::Time::now() - m_odom_at_scan.header.stamp;
    const double &acc_x = (m_odom_at_scan.twist.twist.linear.x - m_prev_odom->twist.twist.linear.x) / dt.toSec();
    const double &acc_y = (m_odom_at_scan.twist.twist.linear.y - m_prev_odom->twist.twist.linear.y) / dt.toSec();
    const double &acc_ang = (m_odom_at_scan.twist.twist.angular.z - m_prev_odom->twist.twist.angular.z) / dt.toSec();
    const double &yaw = tf::getYaw(pose.getRotation());
    const double &v_x = m_odom_at_scan.twist.twist.linear.x + acc_x * dt.toSec();
    const double &v_y = m_odom_at_scan.twist.twist.linear.y + acc_y * dt.toSec();
    const double &v_ang = m_odom_at_scan.twist.twist.angular.z + acc_ang * dt.toSec();
    const double &yaw_f = yaw + m_odom_at_scan.twist.twist.linear.z * dt.toSec() + acc_ang * std::pow(dt.toSec(), 2) / 2;
    double r = std::sqrt(std::pow(v_x, 2) + std::pow(v_y, 2)) / v_ang;
    if(std::isnan(r) || std::isinf(r))
    {
        r = 0;
    }
    const double &x_f = pose.getOrigin().getX() - r * sin(yaw) + r * sin(yaw_f);
    const double &y_f = pose.getOrigin().getY() + r * cos(yaw) - r * cos(yaw_f);
    pose.setOrigin(tf::Vector3(x_f, y_f, 0));
    tf::Quaternion q_f = tf::createQuaternionFromYaw(yaw_f);
    pose.setRotation(q_f);
}

void Localization::setPreviousPose(const tf::Pose &pose)
{
    tf::poseTFToMsg(pose, m_prev_pose.pose.pose);
    m_odom_at_last_scan = m_odom_at_scan;
}

void Localization::pubFinalPose()
{
    geometry_msgs::PoseWithCovarianceStamped pose;
    pose.header.stamp = ros::Time::now();    
    pose = m_prev_pose;
    const std::vector<double> &covariances = calcCovarianceMatrix();
    pose.pose.covariance[0] = covariances[0];
    pose.pose.covariance[7] = covariances[1];
    pose.pose.covariance[35] = covariances[2];
    m_pose_pub.publish(pose);
}

std::vector<double> Localization::calcCovarianceMatrix()
{
    std::vector<double> covariances(3);
    double mean_x = 0;
    double mean_y = 0;
    double mean_th = 0;
    double sum_weight_squares = 0;
    for(const auto &particle : m_particles)
    {
        mean_x += particle.weight * particle.pose.getOrigin().getX();
        mean_y += particle.weight * particle.pose.getOrigin().getY();
        mean_th += particle.weight * tf::getYaw(particle.pose.getRotation());
        sum_weight_squares += std::pow(particle.weight, 2);
    }
    double sigma_x = 0;
    double sigma_y = 0;
    double sigma_th = 0;
    for(const auto &particle : m_particles)
    {
        sigma_x += particle.weight * std::pow((particle.pose.getOrigin().getX() - mean_x), 2);
        sigma_y += particle.weight * std::pow((particle.pose.getOrigin().getY() - mean_y), 2);
        sigma_th += particle.weight * std::pow((tf::getYaw(particle.pose.getRotation()) - mean_th), 2);
    }
    covariances[0] = sigma_x / (1 - sum_weight_squares);
    covariances[1] = sigma_y / (1 - sum_weight_squares);
    covariances[2] = sigma_th / (1 - sum_weight_squares);
    return covariances;
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
    const double &yaw = tf::getYaw(q);
    const double &yaw_f = yaw + m_odom_at_scan.twist.twist.linear.z * dt.toSec();
    double r = m_odom_at_scan.twist.twist.linear.x / m_odom_at_scan.twist.twist.angular.z;
    if(std::isnan(r) || std::isinf(r))
    {
        r = 0;
    }
    const double &x_f = m_odom->pose.pose.position.x - r * sin(yaw) + r * sin(yaw_f);
    const double &y_f = m_odom->pose.pose.position.y + r * cos(yaw) - r * cos(yaw_f);
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
    if(!m_have_scan && m_have_odom && m_have_map)
    {
        m_have_scan = true;        
        m_fake_scan = new FakeScan(*m_map, *msg);
        m_odom_at_last_scan = *m_odom;
        m_scan = msg;
    }
    if(m_have_odom)
    {
        m_prev_scan = *m_scan;
        m_scan = msg;
        integrateOdomToScanTime();
        Localize();
    }
}

void Localization::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    if(!m_have_odom)
    {
        m_odom = msg;
        m_have_odom = true;        
        m_rng.seed(ros::Time::now().toSec());
        GaussianDistribution sens_x(0.0, std::sqrt(m_sensor_var_x));
        GaussianDistribution sens_y(0.0, std::sqrt(m_sensor_var_y));
        GaussianDistribution sens_yaw(0.0, std::sqrt(m_sensor_var_yaw));
        m_gen_sens_x = new GaussianGenerator(m_rng, sens_x);
        m_gen_sens_y = new GaussianGenerator(m_rng, sens_y);
        m_gen_sens_yaw = new GaussianGenerator(m_rng, sens_yaw);
    }
    m_prev_odom = m_odom;
    const double &cov_trans = std::sqrt(msg->pose.covariance[0] / 50 + msg->pose.covariance[7] / 50);
    const double &cov_rot = std::sqrt(msg->pose.covariance[35] / 50);
    const double &cov_sigma_rot = cov_rot + cov_trans;
    const double &cov_sigma_trans = cov_trans + 2 * cov_rot;
    GaussianDistribution s(0.0, cov_sigma_rot);
    GaussianDistribution st(0.0, cov_sigma_trans);;
    m_gen_s = new GaussianGenerator(m_rng, s);
    m_gen_st = new GaussianGenerator(m_rng, st);
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

void Localization::poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    if(!m_have_pose_estimate)
    {
        m_have_pose_estimate = true;
    }
    ROS_INFO_STREAM("Pose Estimate Updated Manually");
    m_prev_pose = *msg;
}

}
