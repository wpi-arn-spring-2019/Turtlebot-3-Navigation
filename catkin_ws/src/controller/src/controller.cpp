#include <controller.hpp>

namespace Turtlebot
{

Controller::Controller(ros::NodeHandle &nh, ros::NodeHandle &pnh, const double &rate) : m_rate(rate)
{
    m_traj_sub = nh.subscribe<turtlebot_msgs::Trajectory>("/local_trajectory", 10, &Controller::trajectoryCallback, this);
    m_pose_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/pf_pose", 10, &Controller::poseCallback, this);
    m_odom_sub = nh.subscribe<nav_msgs::Odometry>("/odom/filtered", 10, &Controller::odomCallback, this);
    m_goal_reached_sub = nh.subscribe<std_msgs::Bool>("/goal_reached", 10, &Controller::goalReachedCallback, this);
    m_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    m_call_type = boost::bind(&Controller::dynamicReconfigureCallback, this, _1, _2);
    m_server = new dynamic_reconfigure::Server<controller::ControllerConfig>(m_config_mutex);
    m_kp_gains_w.resize(5);
    m_ki_gains_w.resize(5);
    m_kd_gains_w.resize(5);
    m_kp_gains_v.resize(5);
    m_ki_gains_v.resize(5);
    m_kd_gains_v.resize(5);
    m_server->setCallback(m_call_type);
    initializeController(pnh);
}

Controller::~Controller()
{
    delete m_server;
    delete m_pd_cont;
    delete m_pid_cont;
    delete m_pd_cont;
    delete m_pid_ff_cont;
    delete m_smith_pred_pd;
    delete m_smith_pred_pid;
    delete m_smith_pred_pd_ff;
    delete m_smith_pred_pid_ff;
}

void Controller::initializeController(ros::NodeHandle &pnh)
{       
    m_server->getConfigDefault(m_config);
    getGains(pnh);
    int type;
    pnh.getParam("controller_type", type);
    if(type == 0)
    {
        m_config.controller_type = type;
        m_config.kp_w = m_kp_gains_w[0];
        m_config.ki_w = 0;
        m_config.kd_w = m_kd_gains_w[0];
        m_config.kp_v = m_kp_gains_v[0];
        m_config.ki_v = 0;
        m_config.kd_v = m_kd_gains_v[0];
        m_cont_type = controller_type::PD;
        m_pd_cont = new PDController();        
        m_pd_cont->setGains(m_kp_gains_w[0], m_kd_gains_w[0],
                            m_kp_gains_v[0], m_kd_gains_v[0]);
        m_smith_pred_pd = new SmithPredictor<PDController>(*m_pd_cont);
    }
    else if(type == 1)
    {      
        m_config.controller_type = type;
        m_config.kp_w = m_kp_gains_w[1];
        m_config.ki_w = m_ki_gains_w[1];
        m_config.kd_w = m_kd_gains_w[1];
        m_config.kp_v = m_kp_gains_v[1];
        m_config.ki_v = m_ki_gains_v[1];
        m_config.kd_v = m_kd_gains_v[1];
        m_cont_type = controller_type::PID;
        m_pid_cont = new PIDController();        
        m_pid_cont->initializeController();
        m_pid_cont->setGains(m_kp_gains_w[1], m_ki_gains_w[1], m_kd_gains_w[1],
                             m_kp_gains_v[1], m_ki_gains_v[1], m_kd_gains_v[1]);
        m_smith_pred_pid = new SmithPredictor<PIDController>(*m_pid_cont);
    }
    else if(type == 2)
    {
        m_config.controller_type = type;
        m_config.kp_w = m_kp_gains_w[2];
        m_config.ki_w = 0;
        m_config.kd_w = m_kd_gains_w[2];
        m_config.kp_v = m_kp_gains_v[2];
        m_config.ki_v = 0;
        m_config.kd_v = m_kd_gains_v[2];
        m_cont_type = controller_type::PID_FF;
        m_pd_ff_cont = new PDFeedForwardController();        
        m_pd_ff_cont->setGains(m_kp_gains_w[2], m_kd_gains_w[2],
                               m_kp_gains_v[2], m_kd_gains_v[2]);
        m_smith_pred_pd_ff = new SmithPredictor<PDFeedForwardController>(*m_pd_ff_cont);
    }
    else if(type == 3)
    {
        m_config.controller_type = type;
        m_config.kp_w = m_kp_gains_w[3];
        m_config.ki_w = m_ki_gains_w[3];
        m_config.kd_w = m_kd_gains_w[3];
        m_config.kp_v = m_kp_gains_v[3];
        m_config.ki_v = m_ki_gains_v[3];
        m_config.kd_v = m_kd_gains_v[3];
        m_cont_type = controller_type::PID_FF;
        m_pid_ff_cont = new PIDFeedForwardController();        
        m_pid_ff_cont->initializeController();
        m_pid_ff_cont->setGains(m_kp_gains_w[3], m_ki_gains_w[3], m_kd_gains_w[3],
                                m_kp_gains_v[3], m_ki_gains_v[3], m_kd_gains_v[3]);
        m_smith_pred_pid_ff = new SmithPredictor<PIDFeedForwardController>(*m_pid_ff_cont);
    }
    else
    {
        ROS_ERROR("Invalid Controller Type Specified. Options Are: pd, pid, pd_ff, pid_ff");
    }
    boost::recursive_mutex::scoped_lock lock(m_config_mutex);
    m_server->updateConfig(m_config);
    lock.unlock();
}

void Controller::getGains(ros::NodeHandle &pnh)
{
    pnh.getParam("kp_w_pd", m_kp_gains_w[0]);
    pnh.getParam("kd_w_pd", m_kd_gains_w[0]);
    pnh.getParam("kp_v_pd", m_kp_gains_v[0]);
    pnh.getParam("kd_v_pd", m_kd_gains_v[0]);
    pnh.getParam("kp_w_pid", m_kp_gains_w[1]);
    pnh.getParam("ki_w_pid", m_ki_gains_w[1]);
    pnh.getParam("kd_w_pid", m_kd_gains_w[1]);
    pnh.getParam("kp_v_pid", m_kp_gains_v[1]);
    pnh.getParam("ki_v_pid", m_ki_gains_v[1]);
    pnh.getParam("kd_v_pid", m_kd_gains_v[1]);
    pnh.getParam("kp_w_pd_ff", m_kp_gains_w[2]);
    pnh.getParam("kd_w_pd_ff", m_kd_gains_w[2]);
    pnh.getParam("kp_v_pd_ff", m_kp_gains_v[2]);
    pnh.getParam("kd_v_pd_ff", m_kd_gains_v[2]);
    pnh.getParam("kp_w_pid_ff", m_kp_gains_w[3]);
    pnh.getParam("ki_w_pid_ff", m_ki_gains_w[3]);
    pnh.getParam("kd_w_pid_ff", m_kd_gains_w[3]);
    pnh.getParam("kp_v_pid_ff", m_kp_gains_v[3]);
    pnh.getParam("ki_v_pid_ff", m_ki_gains_v[3]);
    pnh.getParam("kd_v_pid_ff", m_kd_gains_v[3]);
}

void Controller::control()
{
    if(m_have_pose && m_have_odom && m_have_trajectory && !m_goal_reached)
    {
        m_current_time = ros::Time::now();
        const TurtlebotState &current_state = getCurrentState();
        const TurtlebotState &desired_state = getDesiredState(false);
        const TurtlebotState &next_desired_state = getDesiredState(true);
        switch(m_cont_type)
        {
        case PD:
            pubControls(m_smith_pred_pd->predictControls(current_state, desired_state, next_desired_state, m_odom_at_control, *m_prev_odom, *m_prev_prev_odom));
            break;
        case PID:
            pubControls(m_smith_pred_pid->predictControls(current_state, desired_state, next_desired_state, m_odom_at_control, *m_prev_odom, *m_prev_prev_odom));
            break;
        case PD_FF:
            pubControls(m_smith_pred_pd_ff->predictControls(current_state, desired_state, next_desired_state, m_odom_at_control, *m_prev_odom, *m_prev_prev_odom));
            break;
        case PID_FF:
            pubControls(m_smith_pred_pid_ff->predictControls(current_state, desired_state, next_desired_state, m_odom_at_control, *m_prev_odom, *m_prev_prev_odom));
            break;
        }
    }
}

const TurtlebotState Controller::getCurrentState()
{
    integrateOdomToCurrentTime();
    integratePoseToCurrentTime();
    const double &x = m_pose_at_control.pose.position.x;
    const double &y = m_pose_at_control.pose.position.y;
    tf::Quaternion q;
    tf::quaternionMsgToTF(m_pose_at_control.pose.orientation, q);
    const double &th = tf::getYaw(q);
    const double &v = std::sqrt(std::pow(m_odom_at_control.twist.twist.linear.x, 2)+
                                std::pow(m_odom_at_control.twist.twist.linear.y, 2));
    const double &th_dot = m_odom_at_control.twist.twist.angular.z;
    const double &x_dot = m_odom_at_control.twist.twist.linear.x;
    const double &y_dot = m_odom_at_control.twist.twist.linear.y;
    const double &dt_prev = ros::Duration(m_odom->header.stamp - m_prev_odom->header.stamp).toSec();
    const double &x_ddot = (m_odom->twist.twist.linear.x - m_prev_odom->twist.twist.linear.x) / dt_prev;
    const double &y_ddot = (m_odom->twist.twist.linear.y - m_prev_odom->twist.twist.linear.y) / dt_prev;
    const double &dt_prev_prev = ros::Duration(m_prev_odom->header.stamp - m_prev_prev_odom->header.stamp).toSec();
    const double &x_dddot = (x_ddot - (m_prev_odom->twist.twist.linear.x - m_prev_prev_odom->twist.twist.linear.x) / dt_prev_prev) / dt_prev_prev;
    const double &y_dddot = (y_ddot - (m_prev_odom->twist.twist.linear.y - m_prev_prev_odom->twist.twist.linear.y) / dt_prev_prev) / dt_prev_prev;
    return TurtlebotState(x, y, th, v, th_dot, x_dot, y_dot, x_ddot, y_ddot, x_dddot, y_dddot);
}

const TurtlebotState Controller::getDesiredState(const bool &next) const
{
    const double &start_time = m_traj->header.stamp.toSec();    
    int traj_it = 0;    
    double dt = start_time - m_current_time.toSec();
    if(next)
    {
        dt -= 1 / m_rate;
    }
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
    double acc_ang = 0;
    double acc_linear = 0;
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
    double jerk_x = 0;
    double jerk_y = 0;
    if(traj_it < m_traj->durations.size())
    {
      jerk_x = (((m_traj->speeds[traj_it + 1] - m_traj->speeds[traj_it]) / m_traj->durations[traj_it]) -
               ((m_traj->speeds[traj_it + 1] - m_traj->speeds[traj_it]) / m_traj->durations[traj_it])) / m_traj->durations[traj_it] * cos(th);
      jerk_y = (((m_traj->speeds[traj_it + 1] - m_traj->speeds[traj_it]) / m_traj->durations[traj_it]) -
               ((m_traj->speeds[traj_it + 1] - m_traj->speeds[traj_it]) / m_traj->durations[traj_it])) / m_traj->durations[traj_it] * sin(th);
    }
    if(std::isnan(jerk_x) || std::isinf(jerk_x))
    {
        jerk_x = 0;
    }
    if(std::isnan(jerk_y) || std::isinf(jerk_y))
    {
        jerk_y = 0;
    }
    const double &acc_x = acc_linear * cos(th);
    const double &acc_y = acc_linear * sin(th);
    const double &v = m_traj->speeds[traj_it] + acc_linear * dt;
    const double &v_x = v * cos(th);
    const double &v_y = v * sin(th);
    const double &x = m_traj->x_values[traj_it - 1] + (v * dt + acc_linear * std::pow(dt, 2) / 2) * cos(th) + jerk_x * std::pow(dt, 3) / 6;
    const double &y = m_traj->x_values[traj_it - 1] + (v * dt + acc_linear * std::pow(dt, 2) / 2) * sin(th) + jerk_y * std::pow(dt, 3) / 6;
    return TurtlebotState(x, y, th, v, th_dot, v_x, v_y, acc_x, acc_y, jerk_x, jerk_y);
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
    const double &x_f = m_pose->pose.pose.position.x + (m_odom->twist.twist.linear.x * dt + acc_x * std::pow(dt, 2) / 2) * cos(yaw);
    const double &y_f = m_pose->pose.pose.position.y + (m_odom->twist.twist.linear.y * dt + acc_y * std::pow(dt, 2) / 2) * sin(yaw);
    tf::Quaternion q_f = tf::createQuaternionFromYaw(yaw_f);
    q_f.normalize();
    m_pose_at_control.pose.position.x = x_f;
    m_pose_at_control.pose.position.y = y_f;
    geometry_msgs::Quaternion q_;
    tf::quaternionTFToMsg(q_f, q_);
    m_pose_at_control.pose.orientation = q_;
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

void Controller::pubControls(const geometry_msgs::TwistStamped &control) const
{
    m_vel_pub.publish(control.twist);
}

void Controller::dynamicReconfigureCallback(controller::ControllerConfig &config, uint32_t level)
{
    if(config.controller_type != m_config.controller_type)
    {
        const int &type = config.controller_type;
        if(type == 0)
        {
            m_cont_type = Controller::PD;
            m_pd_cont = new PDController;            
            m_pd_cont->setGains(m_kp_gains_w[0], m_kd_gains_w[0],
                                m_kp_gains_v[0], m_kd_gains_v[0]);
            m_smith_pred_pd = new SmithPredictor<PDController>(*m_pd_cont);
            config.kp_w = m_kp_gains_w[0];
            config.ki_w = 0;
            config.kd_w = m_kd_gains_w[0];
            config.kp_v = m_kp_gains_v[0];
            config.ki_v = 0;
            config.kd_v = m_kd_gains_v[0];
        }
        if(type == 1)
        {
            m_cont_type = Controller::PID;
            m_pid_cont = new PIDController;            
            m_pid_cont->initializeController();
            m_pid_cont->setGains(m_kp_gains_w[1], m_ki_gains_w[1], m_kd_gains_w[1],
                                 m_kp_gains_v[1], m_ki_gains_v[1], m_kd_gains_v[1]);
            m_smith_pred_pid = new SmithPredictor<PIDController>(*m_pid_cont);
            config.kp_w = m_kp_gains_w[1];
            config.ki_w = m_ki_gains_w[1];
            config.kd_w = m_kd_gains_w[1];
            config.kp_v = m_kp_gains_v[1];
            config.ki_v = m_ki_gains_v[1];
            config.kd_v = m_kd_gains_v[1];
        }
        if(type == 2)
        {
            m_cont_type = Controller::PD_FF;
            m_pd_ff_cont = new PDFeedForwardController;            
            m_pd_ff_cont->setGains(m_kp_gains_w[2], m_kd_gains_w[2],
                                   m_kp_gains_v[2], m_kd_gains_v[2]);
            m_smith_pred_pd_ff = new SmithPredictor<PDFeedForwardController>(*m_pd_ff_cont);
            config.kp_w = m_kp_gains_w[2];
            config.ki_w = 0;
            config.kd_w = m_kd_gains_w[2];
            config.kp_v = m_kp_gains_v[2];
            config.ki_v = 0;
            config.kd_v = m_kd_gains_v[2];
        }
        if(type == 3)
        {
            m_cont_type = Controller::PID_FF;
            m_pid_ff_cont = new PIDFeedForwardController;            
            m_pid_ff_cont->initializeController();
            m_pid_ff_cont->setGains(m_kp_gains_w[3], m_ki_gains_w[3], m_kd_gains_w[3],
                                    m_kp_gains_v[3], m_ki_gains_v[3], m_kd_gains_v[3]);
            m_smith_pred_pid_ff = new SmithPredictor<PIDFeedForwardController>(*m_pid_ff_cont);
            config.kp_w = m_kp_gains_w[3];
            config.ki_w = m_ki_gains_w[3];
            config.kd_w = m_kd_gains_w[3];
            config.kp_v = m_kp_gains_v[3];
            config.ki_v = m_ki_gains_v[3];
            config.kd_v = m_kd_gains_v[3];
        }
    }
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

void Controller::poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
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
        m_odom = msg;
        m_prev_odom = m_odom;
    }
    m_prev_prev_odom = m_prev_odom;
    m_prev_odom = m_odom;
    m_odom = msg;
}

void Controller::goalReachedCallback(const std_msgs::Bool::ConstPtr &msg)
{
    m_goal_reached = msg->data;
}

}
