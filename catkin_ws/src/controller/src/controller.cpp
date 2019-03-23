#include <controller.hpp>

namespace Turtlebot
{

Controller::Controller(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{
    m_traj_sub = nh.subscribe<turtlebot_msgs::Trajectory>("/trajectory", 10, &Controller::trajectoryCallback, this);
    m_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/pf_pose", 10, &Controller::poseCallback, this);
    m_odom_sub = nh.subscribe<nav_msgs::Odometry>("/odom/filtered", 10, &Controller::odomCallback, this);
    m_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    m_call_type = boost::bind(&Controller::dynamicReconfigureCallback, this, _1, _2);
    m_server = new dynamic_reconfigure::Server<controller::ControllerConfig>(m_config_mutex);
    m_server->setCallback(m_call_type);
    initializeController(pnh);
}

Controller::~Controller()
{
    delete m_server;
    delete m_pid_ff_cont;
}

void Controller::initializeController(ros::NodeHandle &pnh)
{
    m_server->getConfigDefault(m_config);
    pnh.getParam("kp_w", m_kp_w);
    pnh.getParam("ki_w", m_ki_w);
    pnh.getParam("kd_w", m_kd_w);
    pnh.getParam("kp_v", m_kp_v);
    pnh.getParam("ki_v", m_ki_v);
    pnh.getParam("kd_v", m_kd_v);
    m_config.kp_w = m_kp_w;
    m_config.ki_w = m_ki_w;
    m_config.kd_w = m_kd_w;
    m_config.kp_v = m_kp_v;
    m_config.ki_v = m_ki_v;
    m_config.kd_v = m_kd_v;
    int type;
    pnh.getParam("controller_type", type);
    if(type == 0)
    {

    }
    else if(type == 1)
    {

    }
    else if(type == 2)
    {

    }
    else if(type == 3)
    {
        m_config.controller_type = type;
        m_cont_type = controller_type::PID_FF;
        m_pid_ff_cont = new PIDFeedForwardController();
        m_pid_ff_cont->initializeController();
        m_pid_ff_cont->setGains(m_kp_w, m_ki_w, m_kd_w, m_kp_v, m_ki_v, m_kd_v);
    }
    else
    {
        ROS_ERROR("Invalid Controller Type Specified. Options Are: pd, pid, pd_ff, pid_ff");
    }
    boost::recursive_mutex::scoped_lock lock(m_config_mutex);
    m_server->updateConfig(m_config);
    lock.unlock();
}

void Controller::control()
{
    if(m_have_pose && m_have_odom && m_have_trajectory && !m_goal_reached)
    {
        m_current_time = ros::Time::now();
        switch(m_cont_type)
        {
        case PD:

            break;
        case PID:

            break;
        case PD_FF:

            break;

        case PID_FF:
            const TurtlebotState &current_state = getCurrentState();
            const TurtlebotState &desired_state = getDesiredState();
            //ROS_INFO_STREAM(current_state.th_dot << " " << desired_state.th_dot);
            pubControls(m_pid_ff_cont->getControls(current_state, desired_state));
            break;
        }        
    }
}

const TurtlebotState Controller::getCurrentState()
{        
    integrateOdomToCurrentTime();
    integratePoseToCurrentTime();
    const double &x = m_pose_at_control.position.x;
    const double &y = m_pose_at_control.position.y;
    tf::Quaternion q;
    tf::quaternionMsgToTF(m_pose_at_control.orientation, q);
    const double &th = tf::getYaw(q);
    const double &v = std::sqrt(std::pow(m_odom_at_control.twist.twist.linear.x, 2)+
                                std::pow(m_odom_at_control.twist.twist.linear.y, 2));
    const double &th_dot = m_odom_at_control.twist.twist.angular.z;
    return TurtlebotState(x, y, th, v, th_dot);
}

const TurtlebotState Controller::getDesiredState() const
{
    const double &start_time = m_traj->header.stamp.toSec();
    double dt = start_time - m_current_time.toSec();
    int traj_it = 0;
    while(dt <= 0)
    {
        if(traj_it >= m_traj->durations.size() && !m_goal_reached)
        {
            ROS_ERROR("Reached End of Trajectory Without Converging to Desired Trajectory");
        }
        dt += m_traj->durations[traj_it];
        traj_it++;
    }

    const double &time_since_last_traj_pt = m_traj->durations[traj_it] + (m_current_time.toSec() - start_time - m_traj->durations[traj_it] * traj_it);
    return integrateDesiredStateToCurrentTime(traj_it, time_since_last_traj_pt);
}

const TurtlebotState Controller::integrateDesiredStateToCurrentTime(const int &traj_it, const double &dt) const
{
    double acc_ang;
    double acc_linear;
    acc_ang = (m_traj->yaw_rates[traj_it] - m_traj->yaw_rates[traj_it - 1]) / m_traj->durations[traj_it];
    acc_linear = (m_traj->speeds[traj_it] - m_traj->speeds[traj_it - 1]) / m_traj->durations[traj_it];
    if(std::isnan(acc_ang) || std::isinf(acc_ang))
    {
        acc_ang = 0;
    }
    if(std::isnan(acc_linear) || std::isinf(acc_linear))
    {
        acc_linear = 0;
    }    
    const double &th_dot = m_traj->yaw_rates[traj_it - 1] + acc_ang * dt;
    const double &th = m_traj->headings[traj_it - 1] + th_dot * dt + acc_ang * std::pow(dt, 2) / 2;
    const double &v = m_traj->speeds[traj_it] + acc_linear * dt;
    const double &x = m_traj->x_values[traj_it - 1] + (v * dt + acc_linear * std::pow(dt, 2) / 2) * cos(th);
    const double &y = m_traj->x_values[traj_it - 1] + (v * dt + acc_linear * std::pow(dt, 2) / 2) * sin(th);
    return TurtlebotState(x, y, th, v, th_dot);

}

void Controller::integratePoseToCurrentTime()
{
    const double &dt = ros::Duration(m_current_time - m_pose->header.stamp).toSec();
    double acc_x = (m_odom->twist.twist.linear.x - m_prev_odom->twist.twist.linear.x) / dt;
    double acc_y = (m_odom->twist.twist.linear.y - m_prev_odom->twist.twist.linear.y) / dt;
    double acc_ang = (m_odom->twist.twist.angular.z - m_prev_odom->twist.twist.angular.z) / dt;
    if(std::isnan(acc_x) || std::isinf(acc_x))
    {
        acc_x = 0;
    }
    if(std::isnan(acc_y) || std::isinf(acc_y))
    {
        acc_x = 0;
    }
    if(std::isnan(acc_ang) || std::isinf(acc_ang))
    {
        acc_ang = 0;
    }
    tf::Quaternion q;
    tf::quaternionMsgToTF(m_odom->pose.pose.orientation, q);
    const double &yaw = tf::getYaw(q);
    const double &yaw_f = yaw + m_odom_at_pose.twist.twist.linear.z * dt;
    const double &x_f = m_pose->pose.position.x + (m_odom->twist.twist.linear.x * dt + acc_x * std::pow(dt, 2) / 2) * cos(yaw);
    const double &y_f = m_pose->pose.position.y + (m_odom->twist.twist.linear.y * dt + acc_y * std::pow(dt, 2) / 2) * sin(yaw);
    tf::Quaternion q_f = tf::createQuaternionFromYaw(yaw_f);
    q_f.normalize();
    m_pose_at_control.position.x = x_f;
    m_pose_at_control.position.y = y_f;
    geometry_msgs::Quaternion q_;
    tf::quaternionTFToMsg(q_f, q_);
    m_pose_at_control.orientation = q_;
}

void Controller::integrateOdomToCurrentTime()
{
    const double &dt = ros::Duration(m_current_time - m_odom->header.stamp).toSec();
    m_odom_at_control.header.stamp = m_current_time;
    double acc_x = (m_odom->twist.twist.linear.x - m_prev_odom->twist.twist.linear.x) / dt;
    double acc_y = (m_odom->twist.twist.linear.y - m_prev_odom->twist.twist.linear.y) / dt;
    double acc_ang = (m_odom->twist.twist.angular.z - m_prev_odom->twist.twist.angular.z) / dt;
    if(std::isnan(acc_x) || std::isinf(acc_x))
    {
        acc_x = 0;
    }
    if(std::isnan(acc_y) || std::isinf(acc_y))
    {
        acc_x = 0;
    }
    if(std::isnan(acc_ang) || std::isinf(acc_ang))
    {
        acc_ang = 0;
    }
    m_odom_at_control.twist.twist.linear.x = m_odom->twist.twist.linear.x + acc_x * dt;
    m_odom_at_control.twist.twist.linear.y = m_odom->twist.twist.linear.y + acc_y * dt;
    m_odom_at_control.twist.twist.angular.z = m_odom->twist.twist.angular.z + acc_ang * dt;
    tf::Quaternion q;
    tf::quaternionMsgToTF(m_odom->pose.pose.orientation, q);
    const double &yaw = tf::getYaw(q);
    const double &yaw_f = yaw + m_odom_at_control.twist.twist.linear.z * dt;
    const double &x_f = m_odom->pose.pose.position.x + (m_odom->twist.twist.linear.x * dt + acc_x * std::pow(dt, 2) / 2) * cos(yaw);
    const double &y_f = m_odom->pose.pose.position.y + (m_odom->twist.twist.linear.y * dt + acc_y * std::pow(dt, 2) / 2) * sin(yaw);
    tf::Quaternion q_f = tf::createQuaternionFromYaw(yaw_f);
    q_f.normalize();
    m_odom_at_control.pose.pose.position.x = x_f;
    m_odom_at_control.pose.pose.position.y = y_f;
    geometry_msgs::Quaternion q_;
    tf::quaternionTFToMsg(q_f, q_);
    m_odom_at_control.pose.pose.orientation = q_;
}

void Controller::pubControls(const geometry_msgs::Twist &control) const
{
    m_vel_pub.publish(control);
}

void Controller::dynamicReconfigureCallback(controller::ControllerConfig &config, uint32_t level)
{
    if(config.controller_type != m_config.controller_type)
    {
        const int &type = config.controller_type;
        if(type == 0)
        {
            m_cont_type = Controller::PD;
        }
        if(type == 1)
        {
            m_cont_type = Controller::PID;
        }
        if(type == 2)
        {
            m_cont_type = Controller::PD_FF;
        }
        if(type == 3)
        {
            m_cont_type = Controller::PID_FF;
            m_pid_ff_cont = new PIDFeedForwardController;
            m_pid_ff_cont->initializeController();
            m_pid_ff_cont->setGains(m_kp_w, m_ki_w, m_kd_w, m_kp_v, m_ki_v, m_kd_v);
        }
    }
    m_kp_w = config.kp_w;
    m_ki_w = config.ki_w;
    m_kd_w = config.kd_w;
    m_kp_v = config.kp_v;
    m_ki_v = config.ki_v;
    m_kd_v = config.kd_v;
}

void Controller::trajectoryCallback(const turtlebot_msgs::Trajectory::ConstPtr &msg)
{
    if(!m_have_trajectory)
    {
        m_have_trajectory = true;
    }
    m_goal_reached = false;
    m_traj = msg;
}

void Controller::poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    if(!m_have_pose)
    {
        m_have_pose = true;
    }
    m_pose = msg;
}

void Controller::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    if(!m_have_odom)
    {
        m_have_odom = true;
        m_prev_odom = msg;
    }
    m_odom = msg;
}

}
